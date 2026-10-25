#include "ModbusGateway.h"

#include <new>

#include <string.h>
#include "ModbusNoExcept.h"

// =============================================================================
// SESSION
// =============================================================================

ModbusGateway::Session::Session(ModbusGateway& owner, ModbusTransport* medium) noexcept
    : gateway(owner), transport(medium), link(*medium, codec),
      lastActivityMs(mb_millis()) {
    codec.expect(ModbusExpect::Request);
    link.begin();
}

ModbusGateway::Session::~Session() { link.end(); }

void ModbusGateway::Session::reopen(ModbusTransport* medium) {
    transport = medium;
    link.attach(*medium, codec);
    link.resetStats();
    lastActivityMs = mb_millis();
    outstanding = 0;
    generation = ++gateway._generations;
    codec.expect(ModbusExpect::Request);
    link.begin();
}

void ModbusGateway::Session::onAdu(const ModbusAdu& adu) {
    lastActivityMs = mb_millis();

    // A local map ALWAYS wins over a route. That is what lets one unit be
    // carved out of a routed range - the gateway's own diagnostics, say,
    // sitting at unit 255 inside a range that otherwise goes to the field bus.
    if (gateway.servesLocally(adu.unit)) {
        gateway.serveLocally(*this, adu);
        return;
    }

    if (!gateway.routes(adu.unit)) {
        // No route. Not a timeout and not a device failure: the gateway is
        // misconfigured or the client asked for something that was never here.
        // 0x0A says exactly that, immediately, instead of making the client
        // wait out a timeout for an answer that was never coming.
        gateway.refuse(*this, adu, MB_EX_GATEWAY_PATH_UNAVAILABLE);
        gateway._pathUnavailable++;
        return;
    }

    if (outstanding >= gateway._maxOutstanding) {
        // A path exists and there is genuinely no room left on it. Still 0x0A -
        // "the gateway was unable to allocate an internal communication path",
        // which is precisely this.
        //
        // Reaching here should be rare. A front-side retry arriving while the
        // first request is still on the field bus is NORMAL, not an overload,
        // and it queues; see setMaxOutstanding.
        gateway.refuse(*this, adu, MB_EX_GATEWAY_PATH_UNAVAILABLE);
        gateway._pathUnavailable++;
        return;
    }

    gateway.forward(*this, adu);
}

// =============================================================================
// THE GATEWAY
// =============================================================================

void ModbusGateway::BackSink::onAdu(const ModbusAdu& adu) {
    // Answers from the field bus. Anything the client does not claim is
    // another master's traffic on a shared segment, which is normal and is not
    // ours to forward anywhere.
    gateway._client.handleAdu(adu);
}

void ModbusGateway::BackSink::onFrameError(ModbusFrameError error, size_t bytes) {
    (void)bytes;
    if (gateway._eventHandler && error == ModbusFrameError::Checksum) {
        gateway._eventHandler(gateway._eventContext, ModbusEvent::ChecksumError, 0,
                              "a frame from the field bus arrived corrupt");
    }
}

ModbusGateway::ModbusGateway(ModbusListener& front, ModbusLink& backLink) noexcept
    : _front(front), _backLink(backLink), _client(backLink), _backSink(*this),
      _engine(_map) {}

ModbusGateway::~ModbusGateway() {
    end();
    for (size_t i = 0; i < _spare.size(); i++) delete _spare[i];
    for (size_t i = 0; i < _localUnits.size(); i++) delete _localUnits[i].map;
}

bool ModbusGateway::begin(const ModbusClientConfig& downstream) {
    ModbusClientConfig config = downstream;

    // The downstream link is the constraint, not a preference. A serial segment
    // carries no transaction identifier, so exactly one request may be
    // outstanding on it however many clients are queued at the front.
    config.maxInFlight = 1;

    // DO NOT RETRY AT TWO LAYERS - or rather, do not retry as HARD.
    //
    // The client at the front is already retrying; that is what a Modbus client
    // does, and its timeout knows nothing about the field bus. If the gateway
    // spends a full retry budget of its own, the two multiply: the client gives
    // up before the gateway has finished trying, and the answer it gets is a
    // plain timeout rather than the 0x0B that would have told it WHICH end was
    // at fault.
    //
    // That is the whole value of a gateway's exceptions. "Gateway target device
    // failed to respond" says the network is fine and the device is not, which
    // a timeout can never say. Losing it to a retry budget that never finished
    // in time is a bad trade.
    //
    // But ZERO retries is the other error: on a real field bus a scheduling
    // blip can fail one read in fifty, because a single lost frame has nowhere
    // to go. A transient is not a
    // device failure and should not be reported as one.
    //
    // So: ONE retry. The arithmetic has to work out, and it does. With the
    // cold-start budget below at three times the floor, two attempts cost six
    // floors; a client using the ordinary default gives up at ten. The gateway
    // therefore always concludes - and can say 0x0B - inside the client's FIRST
    // attempt, with room to spare.
    if (downstream.maxRetries == ModbusClientConfig().maxRetries) {
        config.maxRetries = 1;
    }

    // And answer FASTER than a client gives up.
    //
    // Before anything has been measured, a client and a gateway using the same
    // defaults reach their first timeout at the same instant - so the client
    // sees a plain timeout rather than the 0x0B the gateway was about to send,
    // and learns nothing about which end failed.
    //
    // A third of the usual cold-start budget leaves the gateway room to
    // conclude and answer well inside the client's first attempt. Once both
    // ends have measured something the problem solves itself: the client's
    // round trip THROUGH the gateway is necessarily longer than the gateway's
    // own round trip to the device, so its timeout is naturally the larger.
    if (downstream.coldStartTimeout == MODBUS_AUTO) {
        // Three times the floor rather than the usual ten. The floor already
        // includes the airtime the field bus physically needs, so this is
        // generous in absolute terms while still leaving room inside a client's
        // own first attempt.
        config.coldStartTimeout = _client.timeoutFloorMs() * 3;
    }

    if (!_client.begin(config)) return false;   // no memory for the downstream queue

    // Room for every request the downstream queue can hold, and for those it
    // refuses until their refusal is reported.
    for (size_t i = _forwards.size(); i < 2 * config.queueCapacity + 8; i++) {
        if (!_forwards.push(Forward())) break;
    }

    // A gateway's downstream link is a client, so it decodes responses.
    _backLink.codec().expect(ModbusExpect::Response);

    if (!_backLink.begin()) return false;
    return _front.begin();
}

void ModbusGateway::end() {
    while (!_sessions.empty()) closeSession(_sessions.size() - 1);
    _front.end();
    _client.cancelAll();
}

void ModbusGateway::route(uint8_t unit) { routeRange(unit, unit); }

void ModbusGateway::routeRange(uint8_t first, uint8_t last) {
    if (first > last) { const uint8_t swap = first; first = last; last = swap; }
    Route route = { first, last };
    _routes.push(route);
}

bool ModbusGateway::routes(uint8_t unit) const {
    if (unit == MB_ADDRESS_BROADCAST) return true;   // Broadcast goes to the bus
    for (size_t i = 0; i < _routes.size(); i++) {
        if (unit >= _routes[i].first && unit <= _routes[i].last) return true;
    }
    return _catchAll;
}

ModbusDataMap& ModbusGateway::localMap(uint8_t unit) {
    for (size_t i = 0; i < _localUnits.size(); i++) {
        if (_localUnits[i].unit == unit) return *_localUnits[i].map;
    }
    LocalUnit entry;
    entry.unit = unit;
    // nullptr on an exhausted heap, where the build has no exceptions. Falling
    // back to the default map is wrong in principle and right in practice: it
    // keeps the device answering rather than faulting, and the alternative is
    // a null dereference in the request path.
    entry.map = new (std::nothrow) ModbusDataMap();
    if (!entry.map) return _map;
    _localUnits.push(entry);
    return *entry.map;
}

bool ModbusGateway::servesLocally(uint8_t unit) const {
    for (size_t i = 0; i < _localUnits.size(); i++) {
        if (_localUnits[i].unit == unit) return true;
    }
    return false;
}

void ModbusGateway::closeSession(size_t index) {
    if (index >= _sessions.size()) return;
    Session* session = _sessions[index];
    ModbusTransport* transport = session->transport;

    // Closed and detached BEFORE the transport goes back, and kept for the
    // next connection.
    session->link.end();
    session->link.detach();
    session->transport = nullptr;
    session->generation = 0;
    _front.release(transport);
    _sessions.erase(index);
    if (!_spare.push(session)) delete session;
}

void ModbusGateway::refuse(Session& session, const ModbusAdu& adu, uint8_t exceptionCode) {
    if (adu.pduLen == 0) return;
    if (adu.unit == MB_ADDRESS_BROADCAST) return;   // Nothing answers a broadcast

    uint8_t pdu[2];
    const size_t length = mbpdu::encodeException(pdu, sizeof pdu, adu.pdu[0], exceptionCode);
    if (length == 0) return;

    ModbusAdu response;
    response.unit = adu.unit;
    response.transactionId = adu.transactionId;
    response.hasTransactionId = adu.hasTransactionId;
    response.pdu = pdu;
    response.pduLen = (uint16_t)length;
    session.link.send(response);
}

void ModbusGateway::serveLocally(Session& session, const ModbusAdu& adu) {
    ModbusDataMap& target = localMap(adu.unit);
    const bool broadcast = (adu.unit == MB_ADDRESS_BROADCAST);

    ModbusServerEngine engine(target);
    engine.setIdentity(_engine.identity());
    engine.setSerialLine(false);   // The front is TCP

    const size_t length = engine.handle(adu.unit, broadcast, adu.pdu, adu.pduLen,
                                        _scratch, sizeof _scratch);
    _servedLocally++;
    if (length == 0) return;

    ModbusAdu response;
    response.unit = adu.unit;
    response.transactionId = adu.transactionId;
    response.hasTransactionId = adu.hasTransactionId;
    response.pdu = _scratch;
    response.pduLen = (uint16_t)length;
    session.link.send(response);
}

void ModbusGateway::forward(Session& session, const ModbusAdu& adu) {
    // Everything the answer will need is kept in a forward slot. The response
    // arrives on a later poll, by which time this ADU's buffer has been reused
    // and the session may be gone entirely.
    size_t slot = 0;
    while (slot < _forwards.size() && _forwards[slot].used) slot++;
    if (slot == _forwards.size()) {
        // No room left at all: what 0x0A is for.
        refuse(session, adu, MB_EX_GATEWAY_PATH_UNAVAILABLE);
        _pathUnavailable++;
        return;
    }
    Forward& f = _forwards[slot];
    f.used = true;
    f.origin = &session;
    f.generation = session.generation;
    f.transactionId = adu.transactionId;
    f.hasTransactionId = adu.hasTransactionId;
    f.unit = adu.unit;
    f.function = adu.pdu[0];

    session.outstanding++;
    _forwarded++;

    _client.forward(adu.unit, adu.pdu, adu.pduLen,
                    ModbusPassthroughHandler(&ModbusGateway::forwarded, this, slot));
}

void ModbusGateway::forwarded(void* self, uintptr_t index, ModbusResult result,
                              const uint8_t* pdu, uint16_t pduLen) {
    static_cast<ModbusGateway*>(self)->concludeForward((size_t)index, result, pdu, pduLen);
}

void ModbusGateway::concludeForward(size_t index, ModbusResult result, const uint8_t* pdu,
                                uint16_t pduLen) {
    const Forward f = _forwards[index];
    _forwards[index].used = false;
    Session* origin = f.origin;
    const uint16_t transactionId = f.transactionId;
    const bool hasTransactionId = f.hasTransactionId;
    const uint8_t unit = f.unit;
    const uint8_t function = f.function;

    // The session may have gone while the request was on the field bus -
    // a client that asked and hung up. Checked by searching the live list
    // rather than by trusting the pointer, because the pointer would be
    // dangling and there is no way to ask it whether it still exists.
    bool alive = false;
    for (size_t i = 0; i < _sessions.size(); i++) {
        if (_sessions[i] == origin) { alive = true; break; }
    }
    // And by its generation: a closed session is reused, and an answer for
    // a client that hung up must not reach the one that took its place.
    if (alive && origin->generation != f.generation) alive = false;
    if (!alive) return;

    if (origin->outstanding) origin->outstanding--;

    ModbusAdu response;
    response.unit = unit;
    response.transactionId = transactionId;
    response.hasTransactionId = hasTransactionId;

    if (pdu && pduLen) {
        // Byte for byte, including an exception the device raised itself -
        // which belongs to the device and must not be replaced with one of
        // the gateway's own.
        response.pdu = pdu;
        response.pduLen = pduLen;
        origin->link.send(response);
        return;
    }

    // Nothing came back. This is the case 0x0B exists for: the route was
    // there and the device was not. A plain timeout at the client tells it
    // nothing about WHICH of the two went wrong, and a client that cannot
    // tell "the network is down" from "that device is off" cannot raise a
    // useful alarm.
    if (result == ModbusResult::Timeout || result == ModbusResult::ChecksumError) {
        _targetFailed++;
        uint8_t exception[2];
        const size_t length = mbpdu::encodeException(exception, sizeof exception,
                                                     function,
                                                     MB_EX_GATEWAY_TARGET_FAILED);
        if (length) {
            response.pdu = exception;
            response.pduLen = (uint16_t)length;
            origin->link.send(response);
        }
        if (_eventHandler) {
            _eventHandler(_eventContext, ModbusEvent::ExceptionReceived, unit,
                          "the device behind the gateway did not answer");
        }
        return;
    }

    if (result == ModbusResult::Ok) return;   // A broadcast: no answer owed

    _pathUnavailable++;
    uint8_t exception[2];
    const size_t length = mbpdu::encodeException(exception, sizeof exception,
                                                 function,
                                                 MB_EX_GATEWAY_PATH_UNAVAILABLE);
    if (length) {
        response.pdu = exception;
        response.pduLen = (uint16_t)length;
        origin->link.send(response);
    }
}

void ModbusGateway::poll() {
    // Accept whatever is waiting.
    for (;;) {
        ModbusTransport* transport = _front.accept();
        if (!transport) break;

        if (_sessions.size() >= _maxConnections) {
            _front.release(transport);
            if (_eventHandler) {
                _eventHandler(_eventContext, ModbusEvent::LinkDown, 0,
                              "a connection was refused: the gateway is full");
            }
            continue;
        }
        // A closed session is reused; only a gateway that has never held this
        // many connections makes a new one.
        Session* session = nullptr;
        if (!_spare.empty()) {
            session = _spare.back();
            _spare.erase(_spare.size() - 1);
            session->reopen(transport);
        } else if (_sessions.reserve(_maxConnections) && _spare.reserve(_maxConnections)) {
            session = new (std::nothrow) Session(*this, transport);
            if (session) session->generation = ++_generations;
        }
        if (!session) {
            _front.release(transport);
            if (_eventHandler) {
                _eventHandler(_eventContext, ModbusEvent::LinkDown, 0,
                              "a connection was refused: out of memory");
            }
            continue;
        }
        _sessions.push(session);
    }

    // The downstream link FIRST, so an answer that has arrived is delivered
    // before the front is read again - which keeps a busy front from starving
    // the responses the clients are already waiting for.
    _backLink.poll(_backSink);

    // Backwards, because a session may close during its own poll and erasing
    // from under a forward iterator is how this goes wrong.
    for (size_t i = _sessions.size(); i-- > 0;) {
        Session* session = _sessions[i];
        session->link.poll(*session);
        if (!session->link.isUp()) {
            if (_eventHandler) {
                _eventHandler(_eventContext, ModbusEvent::LinkDown, 0,
                              "a client disconnected");
            }
            closeSession(i);
        }
    }

    _client.poll();
}
