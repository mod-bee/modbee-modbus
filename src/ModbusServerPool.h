#pragma once

#include "ModbusLink.h"
#include "ModbusResult.h"
#include "ModbusServer.h"
#include "ModbusServerEngine.h"
#include "ModbusTransport.h"
#include "codec/ModbusTcpCodec.h"
#include "codec/ModbusRtuCodec.h"

#include "ModbusArray.h"

// =============================================================================
// A SERVER FACING MANY CLIENTS
// =============================================================================
//
// One data map and one engine, N connections.
//
// This is the shape a serial server does not need and a TCP one cannot do
// without. A Modbus TCP server faces several clients at once - a SCADA package,
// an HMI, an engineer's laptop, all polling the same device - each with its own
// socket, its own transaction stream and its own lifecycle.
//
// WHAT IS SHARED is what the device exposes: the data map, the identity, the
// access rules. WHAT IS PER CONNECTION is framing and transaction state. Making
// that split explicit is the whole of the design; everything else follows from
// it.

// Which framing accepted connections speak.
//
// RTU over a TCP socket is not Modbus TCP - it is a different thing with the
// same port number, produced by every terminal server ever sold. Supporting it
// is a one-line choice here and saves an integrator a day.
enum class ModbusSocketFraming : uint8_t {
    Tcp = 0,       // MBAP. What "Modbus TCP" means.
    RtuOverTcp,    // An RTU ADU, CRC and all, inside a TCP stream.
};

class ModbusServerPool : public ModbusServerSide {
public:
    ModbusServerPool(ModbusListener& listener,
                     ModbusSocketFraming framing = ModbusSocketFraming::Tcp) noexcept;
    ~ModbusServerPool() override;

    ModbusServerPool(const ModbusServerPool&) = delete;
    ModbusServerPool& operator=(const ModbusServerPool&) = delete;

    bool begin(const ModbusServerConfig& config = ModbusServerConfig());
    void end() override;

    // What every connection sees. Bind the application's variables here.
    ModbusDataMap& map() override { return _map; }
    ModbusDataMap& mapFor(uint8_t unit);
    ModbusServerEngine& engine() { return _engine; }

    void setIdentity(const ModbusDeviceIdentity& identity) override {
        _engine.setIdentity(identity);
    }
    void setFilter(ModbusRequestFilter filter) override { _engine.setFilter(filter); }

    // Accepts what is waiting, drives every connection, and reaps the ones
    // that have gone.
    void poll() override;

    size_t connectionCount() const override { return _sessions.size(); }
    uint32_t requestsAnswered() const { return _answered; }

    // Connections dropped because the pool was full. A client that reconnects
    // on every transaction - some SCADA packages do - will find the limit, and
    // seeing this climb is how an integrator learns that is what is happening.
    uint32_t connectionsRefused() const { return _refused; }

    // The most connections to hold at once. A new one past the limit is
    // accepted and immediately closed, rather than left queued in the operating
    // system's backlog where the client would wait for a server that is never
    // going to answer.
    void setMaxConnections(size_t limit) { _maxConnections = limit ? limit : 1; }

    // Close a connection that has said nothing for this long. A client that
    // goes away without closing leaves a socket that never reports an error and
    // holds a slot for ever.
    //
    // Sixty seconds by default: far longer than any polling client's gap
    // between requests, and short enough that dead sockets clear themselves
    // without anybody noticing. Zero disables it.
    void setIdleTimeout(mb_time_t ms) { _idleTimeoutMs = ms; }

    // When every slot is taken and another client arrives, displace the least
    // recently active connection rather than refusing the newcomer - provided
    // that connection has been silent long enough to be certain it is not
    // mid-exchange.
    //
    // On by default, because the alternative is a device made unreachable by
    // sockets that died without saying so. Turn it off where the client set is
    // fixed and known, and an unexpected connection should be refused rather
    // than served.
    void setEvictWhenFull(bool enabled) { _evictWhenFull = enabled; }

    // Connections refused because every slot was genuinely in use, and
    // connections displaced because a slot was being held by a silent one.
    uint32_t refusedCount() const { return _refused; }
    uint32_t evictedCount() const { return _evicted; }

    // How long EVERY client may be silent before the safe states are applied.
    // Settable after begin(), because an application configures its safety
    // after it has opened the port, not before.
    void setFailsafeTimeout(mb_time_t ms) override { _config.failsafeTimeout = ms; }

    void onEvent(ModbusEventHandler handler, void* context) override {
        _eventHandler = handler;
        _eventContext = context;
    }

    // --- What the whole server has seen ---
    //
    // Summed over every connection, including those that have since closed. A
    // per-connection figure would be nearly useless: on a server whose client
    // reconnects for each transaction - some SCADA packages do - the newest
    // link has always just seen one frame.
    ModbusLinkStats stats() const override;
    void resetStats() override;

    // Raw byte tracing across every connection, applied to the ones open now
    // and to every one accepted afterwards.
    void onTrace(ModbusLink::TraceHandler handler, void* context) override;

private:
    // One connection. Owns its transport, its codec and its link; borrows the
    // engine and the map.
    struct Session : public ModbusAduSink {
        Session(ModbusServerPool& pool, ModbusTransport* transport,
                ModbusSocketFraming framing) noexcept;
        ~Session() override;

        // A closed session taking a new connection: the same buffers and
        // codecs, so a server that has had N clients at once never allocates
        // for a connection again.
        void reopen(ModbusTransport* medium);

        void onAdu(const ModbusAdu& adu) override;
        void onFrameError(ModbusFrameError error, size_t bytes) override;

        ModbusServerPool& pool;
        ModbusTransport* transport;

        // Both are held rather than one being chosen, because a codec cannot be
        // constructed polymorphically without an allocation per connection and
        // a virtual destructor chain that buys nothing. One is used; the other
        // costs a few hundred bytes and no thought.
        ModbusTcpCodec tcpCodec;
        ModbusRtuCodec rtuCodec;
        ModbusCodec& codec;

        ModbusLink link;
        mb_time_t lastActivityMs;

        // Whether this connection has ever completed a request. A connection
        // that has is a client, and is never displaced to make room; one that
        // has not is still only a socket.
        bool served = false;
        bool closed = false;
    };

    ModbusListener& _listener;
    ModbusSocketFraming _framing;
    ModbusDataMap _map;
    ModbusServerEngine _engine;
    ModbusServerConfig _config;

    ModbusArray<Session*> _sessions;
    ModbusArray<Session*> _spare;     // closed, kept for the next connection
    size_t _maxConnections = 8;

    // How long a connection that has never asked for anything must have sat
    // there before it may be displaced. Five seconds: long enough that a burst
    // of arrivals is refused rather than churning through its own slots, short
    // enough that a scan or a half-open socket does not hold a device
    // unreachable for a minute.
    static const mb_time_t kEvictionGraceMs = 5000;
    bool _evictWhenFull = true;
    uint32_t _evicted = 0;

    size_t displaceableSession(mb_time_t now) const;
    static mb_time_t sessionIdleFor(const Session& session, mb_time_t now);
    mb_time_t _idleTimeoutMs = 60000;   // See setIdleTimeout()

    struct UnitMap { uint8_t unit; ModbusDataMap* map; };
    ModbusArray<UnitMap> _unitMaps;

    uint32_t _answered = 0;
    uint32_t _refused = 0;

    // The counters of connections that have already closed. Without this, every
    // disconnection would silently erase the history of the link it took with
    // it.
    ModbusLinkStats _retired;

    ModbusLink::TraceHandler _trace = nullptr;
    void* _traceContext = nullptr;

    ModbusEventHandler _eventHandler = nullptr;
    void* _eventContext = nullptr;

    uint8_t _response[MB_MAX_PDU];

    void closeSession(size_t index);
};
