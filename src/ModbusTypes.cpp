#include "ModbusTypes.h"

const char* mb_functionName(uint8_t function) {
    switch (mb_baseFunction(function)) {
        case MB_FC_READ_COILS:               return "Read Coils";
        case MB_FC_READ_DISCRETE_INPUTS:     return "Read Discrete Inputs";
        case MB_FC_READ_HOLDING_REGISTERS:   return "Read Holding Registers";
        case MB_FC_READ_INPUT_REGISTERS:     return "Read Input Registers";
        case MB_FC_WRITE_SINGLE_COIL:        return "Write Single Coil";
        case MB_FC_WRITE_SINGLE_REGISTER:    return "Write Single Register";
        case MB_FC_READ_EXCEPTION_STATUS:    return "Read Exception Status";
        case MB_FC_DIAGNOSTICS:              return "Diagnostics";
        case MB_FC_GET_COMM_EVENT_COUNTER:   return "Get Comm Event Counter";
        case MB_FC_GET_COMM_EVENT_LOG:       return "Get Comm Event Log";
        case MB_FC_WRITE_MULTIPLE_COILS:     return "Write Multiple Coils";
        case MB_FC_WRITE_MULTIPLE_REGISTERS: return "Write Multiple Registers";
        case MB_FC_REPORT_SERVER_ID:         return "Report Server ID";
        case MB_FC_READ_FILE_RECORD:         return "Read File Record";
        case MB_FC_WRITE_FILE_RECORD:        return "Write File Record";
        case MB_FC_MASK_WRITE_REGISTER:      return "Mask Write Register";
        case MB_FC_READ_WRITE_REGISTERS:     return "Read/Write Multiple Registers";
        case MB_FC_READ_FIFO_QUEUE:          return "Read FIFO Queue";
        case MB_FC_ENCAPSULATED_INTERFACE:   return "Encapsulated Interface Transport";
        default:                             return "unknown function";
    }
}

const char* mb_exceptionName(uint8_t exceptionCode) {
    switch (exceptionCode) {
        case MB_EX_NONE:                     return "no exception";
        case MB_EX_ILLEGAL_FUNCTION:         return "illegal function";
        case MB_EX_ILLEGAL_DATA_ADDRESS:     return "illegal data address";
        case MB_EX_ILLEGAL_DATA_VALUE:       return "illegal data value";
        case MB_EX_SERVER_DEVICE_FAILURE:    return "server device failure";
        case MB_EX_ACKNOWLEDGE:              return "acknowledge (poll for completion)";
        case MB_EX_SERVER_DEVICE_BUSY:       return "server device busy (retry later)";
        case MB_EX_NEGATIVE_ACKNOWLEDGE:     return "negative acknowledge";
        case MB_EX_MEMORY_PARITY_ERROR:      return "memory parity error";
        case MB_EX_GATEWAY_PATH_UNAVAILABLE: return "gateway path unavailable";
        case MB_EX_GATEWAY_TARGET_FAILED:    return "gateway target device failed to respond";
        default:                             return "unknown exception";
    }
}

const char* mb_tableName(ModbusTable table) {
    switch (table) {
        case ModbusTable::Coil:            return "coil";
        case ModbusTable::DiscreteInput:   return "discrete input";
        case ModbusTable::HoldingRegister: return "holding register";
        case ModbusTable::InputRegister:   return "input register";
    }
    return "unknown table";
}

ModbusTable mb_tableForFunction(uint8_t function) {
    switch (mb_baseFunction(function)) {
        case MB_FC_READ_COILS:
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_MULTIPLE_COILS:
            return ModbusTable::Coil;

        case MB_FC_READ_DISCRETE_INPUTS:
            return ModbusTable::DiscreteInput;

        case MB_FC_READ_INPUT_REGISTERS:
            return ModbusTable::InputRegister;

        default:
            // Everything else that touches data touches holding registers:
            // 0x03, 0x06, 0x10, 0x16, 0x17, 0x18. Codes that touch no table at
            // all land here too, which is why the header says not to ask.
            return ModbusTable::HoldingRegister;
    }
}

// --- Result and event names ---
#include "ModbusResult.h"

const char* mb_resultName(ModbusResult result) {
    switch (result) {
        case ModbusResult::Queued:                 return "Queued";
        case ModbusResult::Ok:                     return "Ok";
        case ModbusResult::NotInitialised:         return "NotInitialised";
        case ModbusResult::InvalidArgument:        return "InvalidArgument";
        case ModbusResult::NotMapped:              return "NotMapped";
        case ModbusResult::Duplicate:              return "Duplicate";
        case ModbusResult::QueueFull:              return "QueueFull";
        case ModbusResult::UnknownUnit:            return "UnknownUnit";
        case ModbusResult::Superseded:             return "Superseded";
        case ModbusResult::Cancelled:              return "Cancelled";
        case ModbusResult::Timeout:                return "Timeout";
        case ModbusResult::Refused:                return "Refused";
        case ModbusResult::Acknowledged:           return "Acknowledged";
        case ModbusResult::Busy:                   return "Busy";
        case ModbusResult::GatewayPathUnavailable: return "GatewayPathUnavailable";
        case ModbusResult::GatewayTargetFailed:    return "GatewayTargetFailed";
        case ModbusResult::LinkDown:               return "LinkDown";
        case ModbusResult::ChecksumError:          return "ChecksumError";
        case ModbusResult::ProtocolError:          return "ProtocolError";
    }
    return "Unknown";
}

const char* mb_eventName(ModbusEvent event) {
    switch (event) {
        case ModbusEvent::LinkUp:             return "LinkUp";
        case ModbusEvent::LinkDown:           return "LinkDown";
        case ModbusEvent::ChecksumError:      return "ChecksumError";
        case ModbusEvent::FramingError:       return "FramingError";
        case ModbusEvent::NoiseDiscarded:     return "NoiseDiscarded";
        case ModbusEvent::RequestTimeout:     return "RequestTimeout";
        case ModbusEvent::ExceptionReceived:  return "ExceptionReceived";
        case ModbusEvent::UnexpectedResponse: return "UnexpectedResponse";
        case ModbusEvent::RequestRefused:     return "RequestRefused";
        case ModbusEvent::FailsafeApplied:    return "FailsafeApplied";
        case ModbusEvent::Reconnecting:       return "Reconnecting";
    }
    return "Unknown";
}

#include "ModbusTyped.h"
#include "ModbusNoExcept.h"

const char* mb_orderName(ModbusOrder order) {
    switch (order) {
        case ModbusOrder::ABCD: return "ABCD (big-endian, high word first)";
        case ModbusOrder::CDAB: return "CDAB (low word first)";
        case ModbusOrder::BADC: return "BADC (byte-swapped within words)";
        case ModbusOrder::DCBA: return "DCBA (fully reversed)";
    }
    return "unknown order";
}
