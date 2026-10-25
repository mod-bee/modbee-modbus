#pragma once

#include "ModbusClient.h"
#include "ModbusLink.h"
#include "ModbusServerEngine.h"
#include "ModbusTransport.h"
#include "codec/ModbusRtuCodec.h"
#include "codec/ModbusTcpCodec.h"

#include "ModbusArray.h"

// =============================================================================
// A GATEWAY
// =============================================================================
//
// TCP in front, a serial segment behind, routed by unit id.
//
//      SCADA ─┐
//        HMI ─┼─ TCP ──▶ [ gateway ] ──RS-485──▶ unit 1  unit 2  unit 17 ...
//     laptop ─┘
//
// The oldest and most useful box in the Modbus world, and the one this
// library's session model exists to make possible: several clients at the
// front, one shared bus behind, and each of the two ends knowing nothing about
// the other.
//
// --- What it does NOT do ---
//
// It does not decode what it carries. A device behind the gateway may support
// function codes this library does not, and a gateway that re-encoded from a
// decoded structure would quietly drop whatever the structure could not hold.
// Requests for a routed unit are forwarded BYTE FOR BYTE and the answers come
// back the same way.
//
// --- The two exceptions it owes ---
//
// A gateway has two failure modes of its own, and the specification gives it a
// code for each. Getting these right is most of what separates a gateway from a
// forwarding loop:
//
//   0x0A Gateway Path Unavailable
//        No route to that unit, or no room to start another transaction.
//        The gateway is misconfigured or overloaded.
//
//   0x0B Gateway Target Device Failed To Respond
//        The route exists and the device did not answer. This is the one that
//        matters: it tells a client "the network is fine, the device is not
//        there", which a plain timeout never can.

class ModbusGateway {
public:
    // `front` accepts the clients; `backLink` is the downstream segment.
    //
    // The back link is a ModbusLink rather than a transport so the caller
    // chooses its codec - RTU is the usual answer, but a gateway fronting
    // another TCP device is a real configuration too.
    ModbusGateway(ModbusListener& front, ModbusLink& backLink) noexcept;
    ~ModbusGateway();

    ModbusGateway(const ModbusGateway&) = delete;
    ModbusGateway& operator=(const ModbusGateway&) = delete;

    bool begin(const ModbusClientConfig& downstream = ModbusClientConfig());
    void end();

    // --- Routing ---

    // Forward this unit id downstream.
    void route(uint8_t unit);

    // Forward a run of unit ids. The usual way to configure one: "everything
    // from 1 to 32 is on the bus behind me".
    void routeRange(uint8_t first, uint8_t last);

    // Forward everything that is not served locally. Convenient, and slightly
    // dangerous: a client's typo then becomes a request on the field bus and a
    // timeout rather than an immediate 0x0A.
    void routeEverythingElse(bool enabled) { _catchAll = enabled; }

    bool routes(uint8_t unit) const;

    // Units the gateway answers ITSELF rather than forwarding - its own
    // diagnostics, a cached block, a virtual device. A local map always wins
    // over a route, so this is how one unit is carved out of a routed range.
    ModbusDataMap& localMap(uint8_t unit);
    bool servesLocally(uint8_t unit) const;

    void setIdentity(const ModbusDeviceIdentity& identity) { _engine.setIdentity(identity); }

    // Applied to every forwarded request as well as every local one, so a
    // gateway can be read-only to the world while the panel behind it is not.
    void setFilter(ModbusRequestFilter filter) { _engine.setFilter(filter); }

    void poll();

    // --- Diagnostics ---

    size_t connectionCount() const { return _sessions.size(); }
    uint32_t forwarded() const { return _forwarded; }
    uint32_t servedLocally() const { return _servedLocally; }
    uint32_t pathUnavailable() const { return _pathUnavailable; }
    uint32_t targetFailed() const { return _targetFailed; }

    void setMaxConnections(size_t limit) { _maxConnections = limit ? limit : 1; }

    // Requests from ONE front connection that may be waiting on the back end
    // at once. A queue depth, not an in-flight depth.
    //
    // The distinction matters and it is the defining hazard of a gateway. The
    // segment behind carries exactly one transaction at a time - serial Modbus
    // has no transaction identifier, so two outstanding requests cannot be told
    // apart - and that is enforced by the downstream client, not here.
    //
    // What a client at the FRONT does meanwhile is retry. Its timeout knows
    // nothing about the field bus, so a retry arrives while the first request
    // is still out there. Refusing it with 0x0A would turn every slow device
    // into a stream of "gateway path unavailable", which says the gateway is
    // misconfigured when the gateway is merely busy - and sends an integrator
    // looking in entirely the wrong place.
    //
    // So a retry QUEUES, and 0x0A is kept for what it means: no route, or no
    // room left at all.
    void setMaxOutstanding(size_t limit) { _maxOutstanding = limit ? limit : 1; }

    void onEvent(ModbusEventHandler handler, void* context) {
        _eventHandler = handler;
        _eventContext = context;
        _client.onEvent(handler, context);
    }

    ModbusClient& downstream() { return _client; }

private:
    struct Session : public ModbusAduSink {
        Session(ModbusGateway& owner, ModbusTransport* medium) noexcept;
        ~Session() override;

        // A closed session taking a new connection (see ModbusServerPool).
        void reopen(ModbusTransport* medium);

        void onAdu(const ModbusAdu& adu) override;

        ModbusGateway& gateway;
        ModbusTransport* transport;
        ModbusTcpCodec codec;
        ModbusLink link;
        mb_time_t lastActivityMs;

        // Transactions this session has waiting downstream. Held so a response
        // can be returned on the socket that asked for it, under the
        // transaction identifier that asked.
        uint16_t outstanding = 0;

        // Which connection this is. A session is reused, so an answer for a
        // client that has gone must not reach the one that took its place.
        uint32_t generation = 0;
    };

    // Routes what comes back off the field bus into the downstream client.
    //
    // A separate object rather than the gateway itself implementing the sink:
    // the gateway is already a sink for nothing, and having one class be the
    // sink for two different links is how a frame from the front ends up being
    // treated as an answer from the back.
    struct BackSink : public ModbusAduSink {
        explicit BackSink(ModbusGateway& owner) noexcept : gateway(owner) {}
        void onAdu(const ModbusAdu& adu) override;
        void onFrameError(ModbusFrameError error, size_t bytes) override;
        ModbusGateway& gateway;
    };

    ModbusListener& _front;
    ModbusLink& _backLink;
    ModbusClient _client;
    BackSink _backSink;

    // The gateway's own map, for units it serves rather than forwards.
    ModbusDataMap _map;
    ModbusServerEngine _engine;

    ModbusArray<Session*> _sessions;
    ModbusArray<Session*> _spare;     // closed, kept for the next connection
    uint32_t _generations = 0;

    // A request out on the field bus: who asked, and what the answer needs.
    // Held here rather than captured by the completion, so forwarding never
    // allocates. Sized at begin() to what the downstream queue can hold.
    struct Forward {
        Session* origin = nullptr;
        uint32_t generation = 0;
        uint16_t transactionId = 0;
        bool hasTransactionId = false;
        uint8_t unit = 0;
        uint8_t function = 0;
        bool used = false;
    };
    ModbusArray<Forward> _forwards;
    static void forwarded(void* self, uintptr_t index, ModbusResult result,
                          const uint8_t* pdu, uint16_t pduLen);
    void concludeForward(size_t index, ModbusResult result, const uint8_t* pdu,
                         uint16_t pduLen);
    size_t _maxConnections = 8;

    // Deep enough that a front-side retry queues rather than being refused,
    // shallow enough that a client hammering an absent unit does not fill the
    // gateway with requests that will all time out.
    size_t _maxOutstanding = 8;

    struct Route { uint8_t first; uint8_t last; };
    ModbusArray<Route> _routes;
    bool _catchAll = false;

    struct LocalUnit { uint8_t unit; ModbusDataMap* map; };
    ModbusArray<LocalUnit> _localUnits;

    uint32_t _forwarded = 0;
    uint32_t _servedLocally = 0;
    uint32_t _pathUnavailable = 0;
    uint32_t _targetFailed = 0;

    ModbusEventHandler _eventHandler = nullptr;
    void* _eventContext = nullptr;

    uint8_t _scratch[MB_MAX_PDU];

    void closeSession(size_t index);
    void refuse(Session& session, const ModbusAdu& adu, uint8_t exceptionCode);
    void serveLocally(Session& session, const ModbusAdu& adu);
    void forward(Session& session, const ModbusAdu& adu);
};
