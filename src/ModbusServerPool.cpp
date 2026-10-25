#include "ModbusServerPool.h"

#include <new>
#include "ModbusNoExcept.h"

ModbusServerPool::Session::Session(ModbusServerPool& owner, ModbusTransport* medium,
                                   ModbusSocketFraming framing) noexcept
    : pool(owner),
      transport(medium),
      codec(framing == ModbusSocketFraming::Tcp
                ? static_cast<ModbusCodec&>(tcpCodec)
                : static_cast<ModbusCodec&>(rtuCodec)),
      link(*medium, codec),
      lastActivityMs(mb_millis()) {
    // A server decodes requests. Saying so is what lets the RTU-over-TCP codec
    // predict lengths at all: the same function code has different lengths in
    // each direction.
    codec.expect(ModbusExpect::Request);

    // RTU inside a TCP stream has no inter-character timing to speak of - the
    // stream delivers in bursts whenever the network feels like it - so length
    // framing is the only honest choice, and gap framing would split frames
    // more or less at random.
    if (framing == ModbusSocketFraming::RtuOverTcp) {
        rtuCodec.setFraming(ModbusFraming::Length);
    }

    link.begin();
}

ModbusServerPool::Session::~Session() { link.end(); }

void ModbusServerPool::Session::reopen(ModbusTransport* medium) {
    transport = medium;
    link.attach(*medium, codec);
    link.resetStats();                 // counted into _retired when it closed
    lastActivityMs = mb_millis();
    served = false;
    closed = false;
    codec.expect(ModbusExpect::Request);
    link.begin();
}

void ModbusServerPool::Session::onAdu(const ModbusAdu& adu) {
    lastActivityMs = mb_millis();

    // This connection has asked for something, so it is a CLIENT and not merely
    // a socket. From here on it is never displaced to make room for somebody
    // else, however long it then goes quiet - because "quiet" is what a client
    // polling every thirty seconds looks like, and throwing one of those off to
    // admit a stranger would be a far worse failure than refusing the stranger.
    served = true;
    pool._engine.setSerialLine(pool._config.serialLine);

    ModbusDataMap& target = pool.mapFor(adu.unit);
    const bool broadcast = (adu.unit == MB_ADDRESS_BROADCAST);

    size_t length = 0;
    if (&target == &pool._map) {
        length = pool._engine.handle(adu.unit, broadcast, adu.pdu, adu.pduLen,
                                     pool._response, sizeof pool._response);
    } else {
        // A per-unit map, for a gateway. The engine is built around whichever
        // map is being served, so one code path answers every unit.
        ModbusServerEngine perUnit(target);
        perUnit.setIdentity(pool._engine.identity());
        perUnit.setSerialLine(pool._config.serialLine);
        length = perUnit.handle(adu.unit, broadcast, adu.pdu, adu.pduLen,
                                pool._response, sizeof pool._response);
    }

    // Silence is a legitimate answer: a broadcast, or a request we could not
    // make sense of. Not an error, and not reported as one.
    if (length == 0) return;

    ModbusAdu response;
    response.unit = adu.unit;
    response.transactionId = adu.transactionId;   // Echoed unchanged, always
    response.pdu = pool._response;
    response.pduLen = (uint16_t)length;

    if (link.send(response)) pool._answered++;
}

void ModbusServerPool::Session::onFrameError(ModbusFrameError error, size_t bytes) {
    (void)bytes;
    if (pool._eventHandler && error == ModbusFrameError::Protocol) {
        // Worth raising by name: on a TCP socket this almost always means the
        // far end is speaking Modbus RTU over TCP, which is a different thing
        // with the same port number.
        pool._eventHandler(pool._eventContext, ModbusEvent::FramingError, 0,
                           "the far end is not speaking Modbus TCP");
    }
}

// =============================================================================
// THE POOL
// =============================================================================

ModbusServerPool::ModbusServerPool(ModbusListener& listener, ModbusSocketFraming framing) noexcept
    : _listener(listener), _framing(framing), _engine(_map) {}

ModbusServerPool::~ModbusServerPool() {
    end();
    for (size_t i = 0; i < _spare.size(); i++) delete _spare[i];
    for (size_t i = 0; i < _unitMaps.size(); i++) delete _unitMaps[i].map;
}

bool ModbusServerPool::begin(const ModbusServerConfig& config) {
    _config = config;
    _engine.setSerialLine(config.serialLine);
    return _listener.begin();
}

void ModbusServerPool::end() {
    while (!_sessions.empty()) closeSession(_sessions.size() - 1);
    _listener.end();
}

ModbusDataMap& ModbusServerPool::mapFor(uint8_t unit) {
    if (unit == MB_ADDRESS_BROADCAST) return _map;
    if (unit == _config.unitId || _config.unitId == 0) return _map;

    for (size_t i = 0; i < _unitMaps.size(); i++) {
        if (_unitMaps[i].unit == unit) return *_unitMaps[i].map;
    }
    UnitMap entry;
    entry.unit = unit;
    // nullptr on an exhausted heap, where the build has no exceptions. Falling
    // back to the default map is wrong in principle and right in practice: it
    // keeps the device answering rather than faulting, and the alternative is
    // a null dereference in the request path.
    entry.map = new (std::nothrow) ModbusDataMap();
    if (!entry.map) return _map;
    _unitMaps.push(entry);
    return *entry.map;
}

void ModbusServerPool::closeSession(size_t index) {
    if (index >= _sessions.size()) return;
    Session* session = _sessions[index];
    ModbusTransport* transport = session->transport;

    // Take its counters before they go with it.
    _retired.add(session->link.stats());

    // Closed and detached BEFORE the transport goes back, and kept: the next
    // connection takes it rather than a fresh allocation.
    session->link.end();
    session->link.detach();
    session->transport = nullptr;

    // The listener allocated the transport, so the listener frees it. Only it
    // knows how it was made.
    _listener.release(transport);

    _sessions.erase(index);
    if (!_spare.push(session)) delete session;
}

ModbusLinkStats ModbusServerPool::stats() const {
    ModbusLinkStats total = _retired;
    for (size_t i = 0; i < _sessions.size(); i++) {
        total.add(const_cast<ModbusLink&>(_sessions[i]->link).stats());
    }
    return total;
}

void ModbusServerPool::resetStats() {
    _retired.reset();
    for (size_t i = 0; i < _sessions.size(); i++) _sessions[i]->link.resetStats();
}

void ModbusServerPool::onTrace(ModbusLink::TraceHandler handler, void* context) {
    _trace = handler;
    _traceContext = context;
    for (size_t i = 0; i < _sessions.size(); i++) {
        _sessions[i]->link.onTrace(handler, context);
    }
}

// Which connection, if any, may be thrown off to make room for a newcomer.
//
// Two conditions, and both matter:
//
//   It has never asked for anything.  A connection that has completed even one
//       request is a client, and a client is entitled to its slot no matter how
//       long it goes between polls - thirty seconds of silence is an ordinary
//       polling interval, not evidence of death. Displacing one of those to
//       admit a stranger would be a worse failure than refusing the stranger.
//
//   It has been silent a good while.  Without this, a burst of connections
//       displaces itself: sixty arriving at once would each throw off the one
//       before, and the pool would churn instead of settling. A burst is
//       refused; a socket that has sat there saying nothing for seconds is not
//       part of a burst.
//
// What is left is exactly the case this exists for: a port scan, a half-open
// connection, a client that connected and died before it asked anything. Those
// are sockets, not clients, and they must not be allowed to make a healthy
// device unreachable.
size_t ModbusServerPool::displaceableSession(mb_time_t now) const {
    if (!_evictWhenFull) return _sessions.size();

    size_t victim = _sessions.size();
    mb_time_t longest = 0;
    for (size_t i = 0; i < _sessions.size(); i++) {
        if (_sessions[i]->served) continue;

        const mb_time_t silence = sessionIdleFor(*_sessions[i], now);
        if (silence < kEvictionGraceMs) continue;

        if (victim == _sessions.size() || silence > longest) {
            victim = i;
            longest = silence;
        }
    }
    return victim;
}

// How long a session has been silent, measured safely.
//
// mb_elapsed() is wrap-around arithmetic: given a timestamp AFTER `now` it
// returns a number near the full 32-bit range, not a negative one. A session
// stamped a millisecond after the reading it is compared against therefore
// reads as idle for forty-nine days, and is closed the instant it connects.
//
// No test on a frozen clock can catch it: with time held still the two values
// are equal and the arithmetic is fine. It takes a real clock, or a simulated
// one that ticks on every reading, to show.
mb_time_t ModbusServerPool::sessionIdleFor(const Session& session, mb_time_t now) {
    if (mb_isFuture(now, session.lastActivityMs)) return 0;
    return mb_elapsed(session.lastActivityMs, now);
}


void ModbusServerPool::poll() {
    // Read fresh for the accept phase, and read AGAIN below for the session
    // phase. One reading for both would be a bug, not a saving: a session
    // created during the accept loop stamps itself with a clock that has since
    // moved on, so measuring its idleness against the earlier reading is
    // measuring against a moment before it existed.
    const mb_time_t acceptedAt = mb_millis();

    // Accept whatever is waiting. More than one may have arrived since the last
    // poll, so this loops rather than taking one per call - a burst of clients
    // reconnecting after a network blip would otherwise be admitted one every
    // scan.
    for (;;) {
        ModbusTransport* transport = _listener.accept();
        if (!transport) break;

        if (_sessions.size() >= _maxConnections) {
            // A full pool is not the same as a BUSY pool.
            //
            // Sockets do not tell you they have died. A laptop that slept, a
            // SCADA package that leaks connections, a port scanner, a client
            // whose thread hung: each leaves a connection that will never say
            // another word and never close. Hold the limit against those and a
            // device becomes unreachable to everybody new while looking
            // perfectly healthy from the inside - which is exactly when an
            // engineer needs to connect to it.
            //
            // So the least recently active connection is displaced, provided it
            // has been silent long enough to be certain it is not mid-exchange.
            // A client that is genuinely being served has spoken within
            // milliseconds; one that has said nothing for the grace period is
            // not using the slot it is holding.
            const size_t victim = displaceableSession(acceptedAt);
            if (victim < _sessions.size()) {
                _evicted++;
                if (_eventHandler) {
                    _eventHandler(_eventContext, ModbusEvent::LinkDown, 0,
                                  "a silent connection was displaced to make room");
                }
                closeSession(victim);
                // Falls through and accepts the new one below.
            } else {
                // Every connection is genuinely in use. Accepted and
                // immediately closed, rather than left queued in the operating
                // system's backlog - where the client would sit waiting for a
                // server that is never going to answer it.
                _refused++;
                _listener.release(transport);
                if (_eventHandler) {
                    _eventHandler(_eventContext, ModbusEvent::LinkDown, 0,
                                  "a connection was refused: every slot is in use");
                }
                continue;
            }
        }

        // Checked, because on an Arduino core this returns nullptr rather than
        // throwing: the build has no exceptions, so an exhausted heap is a
        // null pointer and the next line is a hard fault.
        //
        // A device out of memory should REFUSE a connection, not die of one.
        // Refusing is survivable and visible; the client retries, the
        // application sees the event, and whatever freed memory in the
        // meantime lets the next one in.
        // A closed session is reused; only a pool that has never held this
        // many connections makes a new one. Both lists are sized to the limit
        // then, so taking or closing a connection never grows them.
        Session* session = nullptr;
        if (!_spare.empty()) {
            session = _spare.back();
            _spare.erase(_spare.size() - 1);
            session->reopen(transport);
        } else if (_sessions.reserve(_maxConnections) && _spare.reserve(_maxConnections)) {
            session = new (std::nothrow) Session(*this, transport, _framing);
        }
        if (!session) {
            _refused++;
            _listener.release(transport);
            if (_eventHandler) {
                _eventHandler(_eventContext, ModbusEvent::LinkDown, 0,
                              "a connection was refused: out of memory");
            }
            continue;
        }

        // A trace set once covers every connection, including this one, which
        // has not existed until now.
        if (_trace) session->link.onTrace(_trace, _traceContext);
        _sessions.push(session);
        if (_eventHandler) {
            _eventHandler(_eventContext, ModbusEvent::LinkUp, 0, "a client connected");
        }
    }

    const mb_time_t now = mb_millis();

    // Walked by index and backwards, because a session may close during its own
    // poll and erasing from under a forward iterator is how this goes wrong.
    for (size_t i = _sessions.size(); i-- > 0;) {
        Session* session = _sessions[i];
        session->link.poll(*session);

        const bool gone = !session->link.isUp();
        const bool idle = _idleTimeoutMs != 0 &&
                          sessionIdleFor(*session, now) > _idleTimeoutMs;

        if (gone || idle) {
            if (_eventHandler) {
                _eventHandler(_eventContext, ModbusEvent::LinkDown, 0,
                              gone ? "a client disconnected"
                                   : "a client was closed for being idle");
            }
            closeSession(i);
        }
    }

    // Failsafe is a property of the device, not of any one connection: it fires
    // when NO client has talked to us, not when one of several has gone quiet.
    if (_config.failsafeTimeout == 0) return;
    mb_time_t quietest = 0;
    bool any = false;
    for (size_t i = 0; i < _sessions.size(); i++) {
        const mb_time_t silence = mb_elapsed(_sessions[i]->lastActivityMs, now);
        if (!any || silence < quietest) { quietest = silence; any = true; }
    }
    if (!any) return;   // Nobody connected at all; nothing to have lost contact with
    if (quietest > _config.failsafeTimeout) {
        _map.applyFailsafe();
        for (size_t i = 0; i < _unitMaps.size(); i++) _unitMaps[i].map->applyFailsafe();
    }
}
