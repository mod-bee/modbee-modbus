#pragma once

#include "ModbusLink.h"
#include "ModbusServerSide.h"
#include "ModbusResult.h"
#include "ModbusServerEngine.h"
#include "ModbusTiming.h"

#include "ModbusArray.h"

// =============================================================================
// THE SERVER
// =============================================================================
//
// What used to be called a slave. Answers other devices' requests against this
// device's own variables.
//
// Binding is on ModbusDataMap and is where most of the work is: the server
// itself only decides WHICH requests are ours to answer, and what to do when
// the master stops talking.

// ModbusServerConfig is in ModbusServerSide.h, with the interface this
// class implements.

class ModbusServer : public ModbusServerSide {
public:
    explicit ModbusServer(ModbusLink& link) noexcept;

    void begin(const ModbusServerConfig& config = ModbusServerConfig());
    ModbusServerConfig& config() { return _config; }

    // What this server exposes. Bind the application's own variables here.
    ModbusDataMap& map() override { return _map; }

    // For a gateway, or for a device that answers several unit ids with
    // different register maps. Created on first use.
    //
    // The DEFAULT map - the one map() returns - serves whatever unit id the
    // config names, and any unit with no map of its own. Adding a per-unit map
    // is therefore additive: nothing that worked before it stops working.
    ModbusDataMap& mapFor(uint8_t unit);
    bool hasMapFor(uint8_t unit) const;

    ModbusServerEngine& engine() { return _engine; }

    void setIdentity(const ModbusDeviceIdentity& identity) override {
        _engine.setIdentity(identity);
    }
    void setFilter(ModbusRequestFilter filter) override { _engine.setFilter(filter); }
    void setFailsafeTimeout(mb_time_t ms) override { _config.failsafeTimeout = ms; }

    // Drives the server: applies failsafe when the master has gone quiet.
    // Does NOT drain the link - the owner does that and routes ADUs here.
    void poll() override;

    // Offers a received ADU. Returns true when it was ours to answer.
    //
    // False means it was addressed elsewhere, and on a serial line that is a
    // normal, frequent event: every device on the segment hears every frame.
    // A server that answered one addressed to another device would put two
    // responses on the bus and corrupt both.
    bool handleAdu(const ModbusAdu& adu) override;

    // Milliseconds since the last request addressed to us, or 0 if none has
    // ever arrived. The number behind the failsafe, and worth exposing: "when
    // did the master last talk to me" is the first question in any
    // commissioning problem.
    mb_time_t silenceMs() const;

    // Requests answered, refused, and ignored as addressed elsewhere.
    uint32_t requestsAnswered() const { return _answered; }
    uint32_t requestsRefused() const { return _engine.counters().busExceptionError; }
    uint32_t requestsForOthers() const { return _forOthers; }

    void onEvent(ModbusEventHandler handler, void* context) override {
        _eventHandler = handler;
        _eventContext = context;
    }

    ModbusLink& link() { return _link; }

private:
    ModbusLink& _link;
    ModbusDataMap _map;
    ModbusServerEngine _engine;
    ModbusServerConfig _config;

    // Per-unit maps, for a gateway. A sparse list rather than a 256-entry
    // array: almost every server has exactly one map, and paying 256 pointers
    // for the exception is the wrong way round.
    struct UnitMap {
        uint8_t unit;
        ModbusDataMap* map;
    };
    ModbusArray<UnitMap> _unitMaps;

    mb_time_t _lastRequestAt = 0;
    bool _heardFromMaster = false;
    bool _failsafeApplied = false;

    uint32_t _answered = 0;
    uint32_t _forOthers = 0;

    ModbusEventHandler _eventHandler = nullptr;
    void* _eventContext = nullptr;

    // Whether a unit id is ours to answer.
    bool addressedToUs(uint8_t unit) const;

    uint8_t _response[MB_MAX_PDU];

public:
    ~ModbusServer();
};
