#pragma once

#include "ModbusClient.h"
#include "ModbusServer.h"

// =============================================================================
// A NODE
// =============================================================================
//
// One link, and both roles over it.
//
// A device is very often both: an RTU multi-master segment where everybody
// polls everybody, a panel that answers a SCADA package and also drives its own
// I/O blocks, a gateway. Making that a FLAG rather than a different class is
// what keeps the two cases from drifting apart.
//
// This is also where incoming frames are ROUTED. On a serial line every device
// hears every frame, so an arriving ADU may be:
//
//   a response to something we asked        -> the client
//   a request addressed to us               -> the server
//   another master's traffic entirely       -> neither, and that is normal
//
// The order matters. The client is offered it first, because it can say
// definitively whether the frame answers a request IT has outstanding, where
// the server can only say whether the unit id looks like ours - and on a
// two-master segment those overlap.

class ModbusNode : public ModbusAduSink {
public:
    ModbusNode(ModbusTransport& transport, ModbusCodec& codec) noexcept;

    // Opens the link. False if the medium cannot be used.
    bool begin();
    void end();

    // Turns the client half on. A node with no client never transmits a
    // request; a node with no server never answers one.
    void enableClient(const ModbusClientConfig& config = ModbusClientConfig());
    void enableServer(const ModbusServerConfig& config = ModbusServerConfig());

    bool clientEnabled() const { return _clientEnabled; }
    bool serverEnabled() const { return _serverEnabled; }

    // Drives everything: drains the link, routes what came out, expires what
    // timed out, and sends what is ready. Call it often.
    void poll();

    ModbusClient& client() { return _client; }
    ModbusServer& server() { return _server; }
    ModbusLink& link() { return _link; }

    // Frames heard that were neither ours to answer nor answers to us. On a
    // multi-master segment this counts the other master's traffic, which is
    // healthy; on a single-master one, growth here means something unexpected
    // is talking.
    uint32_t framesForOthers() const { return _forOthers; }

    void onEvent(ModbusEventHandler handler, void* context) {
        _client.onEvent(handler, context);
        _server.onEvent(handler, context);
        _eventHandler = handler;
        _eventContext = context;
    }

    // --- ModbusAduSink ---
    void onAdu(const ModbusAdu& adu) override;
    void onFrameError(ModbusFrameError error, size_t bytes) override;

private:
    ModbusLink _link;
    ModbusClient _client;
    ModbusServer _server;

    bool _clientEnabled = false;
    bool _serverEnabled = false;
    uint32_t _forOthers = 0;

    ModbusEventHandler _eventHandler = nullptr;
    void* _eventContext = nullptr;

    void updateExpectation();
};
