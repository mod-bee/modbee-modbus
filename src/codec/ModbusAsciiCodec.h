#pragma once

#include "../ModbusCodec.h"
#include "../ModbusCrc.h"

// =============================================================================
// MODBUS ASCII
// =============================================================================
//
//     ':'  hex(unit)  hex(PDU...)  hex(LRC)  CR  LF
//
// 513 characters at most: the colon, two hex digits for each of the 256 bytes
// an RTU frame could hold, and the two-character terminator.
//
// Every byte travels as two hex characters, so the wire carries twice the
// bytes and the frame is delimited by PRINTABLE characters rather than by
// silence. That is the whole point of it: ASCII exists for links that cannot
// guarantee RTU's inter-character timing - a modem, a radio, a terminal server
// - where a gap of a second in the middle of a frame is survivable.
//
// The checksum is an LRC, not a CRC: the two's complement of the sum of the
// binary bytes, before hex encoding.
//
// Case: transmitted UPPERCASE, as the specification says. Accepted either way,
// because devices in the field do not all agree.

class ModbusAsciiCodec : public ModbusCodec {
public:
    ModbusAsciiCodec() noexcept;

    // --- ModbusCodec ---
    size_t encode(uint8_t* out, size_t capacity, const ModbusAdu& adu) override;
    size_t feed(const uint8_t* in, size_t length, mb_time_t nowUs,
                ModbusAduSink& sink) override;
    void tick(mb_time_t nowUs, ModbusAduSink& sink) override;
    void reset() override;
    void expect(ModbusExpect what) override { _expect = what; }
    ModbusExpect expecting() const override { return _expect; }

    // None: ':' and CRLF delimit the frame, so no silence is required.
    uint32_t txGapUs() const override { return 0; }
    size_t maxAduBytes() const override { return MB_ASCII_ADU_MAX; }
    const char* name() const override { return "ASCII"; }

    // Silence after which a frame that never reached its CRLF is abandoned.
    //
    // The specification allows a full second between characters, which is what
    // ASCII is FOR: it exists for links that cannot hold RTU's timing. So this
    // is generous by default and is not a framing mechanism - it only stops a
    // half-received frame sitting in the buffer for ever after the far end goes
    // away mid-transmission.
    void setInterCharTimeout(mb_time_t us) { _interCharUs = us; }
    mb_time_t interCharTimeoutUs() const { return _interCharUs; }

    static const char FRAME_START = ':';
    static const char FRAME_END_1 = '\r';
    static const char FRAME_END_2 = '\n';

private:
    ModbusExpect _expect = ModbusExpect::Request;

    // Decoded bytes, not characters: each pair of hex digits is folded into one
    // byte as it arrives, so the buffer is half the size of the wire form and
    // the LRC can be checked without a second pass.
    uint8_t _bytes[MB_RTU_ADU_MAX];
    size_t _count = 0;

    bool _inFrame = false;
    int _pendingNibble = -1;   // High nibble seen, waiting for its partner

    mb_time_t _lastCharUs = 0;
    mb_time_t _interCharUs = 1000000;   // One second, per the specification
    bool _hasChars = false;

    void startFrame();
    void deliver(ModbusAduSink& sink);
};
