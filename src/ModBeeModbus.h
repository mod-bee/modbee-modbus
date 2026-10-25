#pragma once

// =============================================================================
// ModBee Modbus
// =============================================================================
//
// Modbus RTU, ASCII and TCP. Client and server. Arduino, Linux, macOS and
// Windows, from one source tree.
//
//     #include <ModBeeModbus.h>
//
// --- Start here ---
//
//   ModbusRTU     Modbus over RS-485 or any serial line
//   ModbusASCII   Modbus over a link that cannot hold RTU's timing
//   ModbusTCP     Modbus over a network, as a client or a server
//   ModbusBridge  TCP in front, a serial segment behind, routed by unit id
//
// One object each, and between them they are the whole library for most
// people. See docs/getting-started.md.
//
// --- Underneath, if you need it ---
//
// Nothing above is a wall. Everything it composes stays public and reachable:
//
//   ModbusClient      reads and writes other devices        (the master)
//   ModbusServer      answers other devices' requests       (the slave)
//   ModbusServerPool  one register map, many TCP clients
//   ModbusGateway     a network in front, a serial segment behind
//   ModbusPoller      repeating reads, with data quality
//   ModbusPoint       one value defined by settings: read or write, decoded,
//                     re-aimed in place, with data quality
//
//   ModbusDataMap     what a server exposes: bindings to your own variables
//   ModbusTransport   one byte channel; ModbusListener produces them
//   ModbusCodec       RTU, ASCII or TCP framing
//   ModbusPdu         every function code, no I/O and no clock
//
// --- The one rule ---
//
// NOTHING BLOCKS. Every call queues and returns; answers arrive in the
// variables you pointed at, on a later poll(). One poll() in your loop is the
// whole contract, and the same code runs in an Arduino loop(), in a PLC task
// and in a Linux daemon.

#include "ModbusPlatform.h"
#include "ModbusTypes.h"
#include "ModbusResult.h"
#include "ModbusTiming.h"
#include "ModbusTyped.h"

#include "ModbusPdu.h"
#include "ModbusCrc.h"
#include "ModbusCodec.h"
#include "ModbusTransport.h"

#include "codec/ModbusRtuCodec.h"
#include "codec/ModbusAsciiCodec.h"
#include "codec/ModbusTcpCodec.h"

#include "ModbusDataMap.h"
#include "ModbusLink.h"
#include "ModbusOperations.h"
#include "ModbusServerEngine.h"

#include "ModbusAPI.h"
#include "ModbusClient.h"
#include "ModbusServer.h"
#include "ModbusNode.h"
#include "ModbusServerPool.h"
#include "ModbusGateway.h"
#include "ModbusPoller.h"
#include "ModbusPoint.h"

// Transports are included last, and each compiles to nothing on the platforms
// it does not serve. The application picks one and passes it in, which keeps
// the core free of any dependency on a particular medium or core.
#include "transport/ModbusStreamTransport.h"
#include "transport/ModbusSerialTransport.h"
#include "transport/ModbusTcpTransport.h"
