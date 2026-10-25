#pragma once

#include "ModbusTypes.h"

// =============================================================================
// WHAT HAPPENED TO AN OPERATION
// =============================================================================
//
// The contract, and it is the whole of what makes the queue trustworthy:
//
//   EVERY ACCEPTED OPERATION REACHES EXACTLY ONE CONCLUSION.
//
// Not "usually", and not "unless it was unconfirmed". An application that opens
// a slot per request and waits to be told can then hold that slot without ever
// stalling it.
//
// An operation that is accepted and silently never concluded does not look like
// a bug. It looks like a control loop that quietly stops issuing requests after
// a few hours, which is far harder to find.

enum class ModbusResult : uint8_t {
    // --- Accepted ---
    Queued = 0,       // Accepted; the outcome arrives on a later poll()
    Ok,               // Completed; the value is already in your variable

    // --- Refused before anything reached the wire ---
    NotInitialised,   // begin() has not been called
    InvalidArgument,  // Null pointer, zero count, or a quantity Modbus disallows
    NotMapped,        // A local address has no variable bound to it
    Duplicate,        // An identical operation is already pending
    QueueFull,        // The operation queue is at capacity
    UnknownUnit,      // The destination is not reachable
    Superseded,       // A newer write to the same address replaced it before it went out
    Cancelled,        // Discarded by cancelAll() or a restart

    // --- Concluded on the wire ---
    Timeout,          // No response after every retry
    Refused,          // The server raised an exception not mapped to a result of its
                      // own (0x02 is NotMapped and 0x03 InvalidArgument)
    Acknowledged,     // Exception 0x05: accepted, and it will take a while. POLL, do
                      // not retry - the server is working on it.
    Busy,             // Exception 0x06: retry later. Distinct from Acknowledged, and
                      // the difference is the whole of what the caller does next.
    GatewayPathUnavailable,  // Exception 0x0A
    GatewayTargetFailed,     // Exception 0x0B: the device behind the gateway is not there

    // --- Link faults ---
    LinkDown,         // The transport is not usable
    ChecksumError,    // A reply arrived and was corrupt
    ProtocolError,    // A reply arrived and made no sense
};

// How many members ModbusResult has.
//
// For a binding that mirrors this enum member for member and casts between the
// two rather than translating, as a PLC function-block wrapper might. A
// static_assert on this number turns "somebody added a result on one side" into
// a build error instead of a value that means something else on the far side.
#define MB_RESULT_COUNT 19

// True when an operation was accepted, immediately or for a later poll().
inline bool mb_accepted(ModbusResult result) {
    return result == ModbusResult::Queued || result == ModbusResult::Ok;
}

// True when the operation finished and did what was asked.
inline bool mb_succeeded(ModbusResult result) {
    return result == ModbusResult::Ok;
}

// True when trying again is the right response. Notably NOT Acknowledged, which
// means the server took the job and is still doing it - retrying there sends a
// second copy of a command already running.
inline bool mb_worthRetrying(ModbusResult result) {
    return result == ModbusResult::Timeout ||
           result == ModbusResult::Busy ||
           result == ModbusResult::ChecksumError ||
           result == ModbusResult::GatewayTargetFailed;
}

const char* mb_resultName(ModbusResult result);

// Maps an exception code onto the result the caller is given.
inline ModbusResult mb_exceptionToResult(uint8_t exceptionCode) {
    switch (exceptionCode) {
        case MB_EX_ILLEGAL_DATA_ADDRESS:     return ModbusResult::NotMapped;
        case MB_EX_ILLEGAL_DATA_VALUE:       return ModbusResult::InvalidArgument;
        case MB_EX_ACKNOWLEDGE:              return ModbusResult::Acknowledged;
        case MB_EX_SERVER_DEVICE_BUSY:       return ModbusResult::Busy;
        case MB_EX_GATEWAY_PATH_UNAVAILABLE: return ModbusResult::GatewayPathUnavailable;
        case MB_EX_GATEWAY_TARGET_FAILED:    return ModbusResult::GatewayTargetFailed;
        default:                             return ModbusResult::Refused;
    }
}

// Where a caller's request has got to, for one that polls rather than
// registering a completion handler.
enum class ModbusRequestState : uint8_t {
    Idle,      // Nothing outstanding and nothing waiting to be read
    Pending,   // Queued or in flight
    Done,      // Completed successfully
    Failed,    // Completed, and did not succeed
};

// =============================================================================
// EVENTS
// =============================================================================

enum class ModbusEvent : uint8_t {
    // Link
    LinkUp,
    LinkDown,
    ChecksumError,
    FramingError,
    NoiseDiscarded,

    // Client
    RequestTimeout,
    ExceptionReceived,
    UnexpectedResponse,      // A reply matching nothing we asked for

    // Server
    RequestRefused,
    FailsafeApplied,

    // Lifecycle
    Reconnecting,
};

const char* mb_eventName(ModbusEvent event);

// Told when something happens that is not an operation's own conclusion.
//
// A plain function pointer plus a context rather than a std::function: this is
// registered once and called from paths that must not allocate, and the context
// pointer is how a C++ caller gets its own object back without one.
typedef void (*ModbusEventHandler)(void* context, ModbusEvent event,
                                   uint8_t unit, const char* detail);
