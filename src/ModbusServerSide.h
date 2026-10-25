#pragma once

// =============================================================================
// The parts of a node it carries only when asked for
// =============================================================================
//
// ModbusAPI reaches its server, and its point set, only through the two
// interfaces below, and creates one only when the application asks for
// something it does: a binding, slave(), listen(), points(). Nothing a node
// that only asks calls ever names the classes behind them, so a board that
// reads a few sensors does not carry the code that answers requests.
//
// The interfaces are kept apart from those classes on purpose: ModbusAPI.cpp
// sees only these, so the compiler cannot turn a call through one back into a
// direct call to the one class it happens to see.

#include "ModbusDataMap.h"
#include "ModbusLink.h"
#include "ModbusResult.h"
#include "ModbusServerEngine.h"
#include "ModbusTiming.h"

class ModbusClient;

struct ModbusServerConfig {
    // The unit id we answer as, on a serial line.
    //
    // 0 means "answer every unit id", which is right for a TCP server - a
    // client reaching us over TCP has already addressed us by IP, and the unit
    // id is then only meaningful to a gateway - and wrong for a serial one,
    // where answering another device's address puts two responses on the bus.
    uint8_t unitId = 1;

    // Silence from the master after which the failsafe states are applied.
    // Zero disables it, which is the default: driving an output is an action,
    // and doing it unasked is its own hazard.
    //
    // Set it on anything driving real outputs. A pump commanded on by a master
    // that has since lost power stays on for ever otherwise, and nothing in
    // Modbus says it should not.
    mb_time_t failsafeTimeout = 0;

    // Whether this server answers the serial-only function codes. Their
    // semantics describe a shared bus and one device behind one address, and a
    // TCP server is neither.
    bool serialLine = true;
};

// The answering half of a node. ModbusServer answers on the node's own link (a
// serial line); ModbusServerPool answers on the connections a listener hands it
// (TCP).
class ModbusServerSide {
public:
    virtual ~ModbusServerSide() {}

    virtual ModbusDataMap& map() = 0;
    virtual void poll() = 0;

    // A request that arrived on the node's own link. Only a serial server
    // takes one; a pool has its own connections.
    virtual bool handleAdu(const ModbusAdu& adu) { (void)adu; return false; }

    virtual void setIdentity(const ModbusDeviceIdentity& identity) = 0;
    virtual void setFilter(ModbusRequestFilter filter) = 0;
    virtual void onEvent(ModbusEventHandler handler, void* context) = 0;
    virtual void setFailsafeTimeout(mb_time_t ms) = 0;

    // Connections and their traffic, for a server that has its own.
    virtual size_t connectionCount() const { return 0; }
    virtual ModbusLinkStats stats() const { return ModbusLinkStats(); }
    virtual void resetStats() {}
    virtual void onTrace(ModbusLink::TraceHandler handler, void* context) {
        (void)handler;
        (void)context;
    }
    virtual void end() {}
};

// Points defined by settings (ModbusPointSet), serviced from the node's poll().
class ModbusPointsPart {
public:
    virtual ~ModbusPointsPart() {}
    virtual void poll(ModbusClient& client) = 0;
};
