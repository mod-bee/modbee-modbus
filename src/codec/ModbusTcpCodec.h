#pragma once

#include "../ModbusCodec.h"

// =============================================================================
// MODBUS TCP
// =============================================================================
//
//     MBAP header                                    PDU
//     [transaction:2][protocol:2][length:2][unit:1]  [function][data...]
//
// 260 bytes at most: seven of header plus 253 of PDU.
//
// Self-delimiting, and that is the whole difference from RTU. The `length`
// field says how many bytes follow it - the unit id plus the PDU - so a frame
// needs no silence to end it and no checksum to prove it arrived whole. TCP
// has already done both.
//
// --- The transaction identifier ---
//
// Chosen by the client and echoed unchanged by the server. It is what lets a
// TCP client have SEVERAL requests outstanding at once, where a serial client
// may have exactly one: with it, a response says which request it answers, and
// without it there is nothing to go on but "the one we sent last".
//
// It is also why a late response cannot corrupt a TCP transaction the way it
// can a serial one. A reply arriving after its own timeout carries the old
// identifier and is simply unmatched.
//
// --- Bytes are a stream, not messages ---
//
// TCP guarantees order and delivery, and guarantees NOTHING about boundaries. A
// single read may deliver half a frame, three frames, or two and a half. Every
// case below is real and each has its own test.

class ModbusTcpCodec : public ModbusCodec {
public:
    ModbusTcpCodec() noexcept;

    // --- ModbusCodec ---
    size_t encode(uint8_t* out, size_t capacity, const ModbusAdu& adu) override;
    size_t feed(const uint8_t* in, size_t length, mb_time_t nowUs,
                ModbusAduSink& sink) override;
    void reset() override;
    void expect(ModbusExpect what) override { _expect = what; }
    ModbusExpect expecting() const override { return _expect; }

    // None. The framing carries its own length, so there is nothing to wait
    // for before transmitting.
    uint32_t txGapUs() const override { return 0; }
    size_t maxAduBytes() const override { return MB_TCP_ADU_MAX; }
    const char* name() const override { return "TCP"; }

    // How many frames were dropped because the MBAP declared something
    // impossible. Sustained growth means the far end is not speaking Modbus TCP
    // - very often it is speaking Modbus RTU over a TCP socket, which is a
    // different thing with the same port number.
    uint32_t protocolErrors() const { return _protocolErrors; }

    // MBAP layout, named so nothing has to count bytes in its head.
    static const size_t HEADER_BYTES = 7;
    static const size_t OFFSET_TRANSACTION = 0;
    static const size_t OFFSET_PROTOCOL = 2;
    static const size_t OFFSET_LENGTH = 4;
    static const size_t OFFSET_UNIT = 6;

    // The only protocol identifier Modbus defines. Anything else is another
    // protocol sharing the port.
    static const uint16_t PROTOCOL_MODBUS = 0;

private:
    ModbusExpect _expect = ModbusExpect::Request;
    uint8_t _buffer[MB_TCP_ADU_MAX];
    size_t _length = 0;
    uint32_t _protocolErrors = 0;

    // Drops the frame in progress and everything after it in the buffer.
    //
    // A TCP stream that has lost sync cannot be resynchronised by shifting one
    // byte and trying again, the way an RTU line can: there is no checksum to
    // confirm a guess with, so a wrong guess produces plausible rubbish
    // indefinitely. The connection is the unit of recovery, and the session
    // closes it.
    void abandon(ModbusAduSink& sink, ModbusFrameError why);
};
