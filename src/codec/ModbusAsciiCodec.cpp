#include "ModbusAsciiCodec.h"

#include <string.h>
#include "../ModbusNoExcept.h"

ModbusAsciiCodec::ModbusAsciiCodec() noexcept {}

void ModbusAsciiCodec::reset() {
    _count = 0;
    _inFrame = false;
    _pendingNibble = -1;
    _hasChars = false;
}

void ModbusAsciiCodec::startFrame() {
    // A ':' always starts a frame, WHEREVER it appears - including in the
    // middle of one already being assembled. That is the recovery mechanism:
    // a frame interrupted by noise or by a transmitter restarting is abandoned
    // the moment the next one begins, with no timeout to wait out.
    _count = 0;
    _inFrame = true;
    _pendingNibble = -1;
}

// =============================================================================
// ENCODING
// =============================================================================

size_t ModbusAsciiCodec::encode(uint8_t* out, size_t capacity, const ModbusAdu& adu) {
    if (!out || !adu.pdu || adu.pduLen == 0 || adu.pduLen > MB_MAX_PDU) return 0;

    // ':' + 2 chars per byte for (unit + PDU + LRC) + CRLF
    const size_t bodyBytes = 1 + (size_t)adu.pduLen;
    const size_t needed = 1 + (bodyBytes + 1) * 2 + 2;
    if (needed > capacity || needed > MB_ASCII_ADU_MAX) return 0;

    // The LRC is taken over the BINARY bytes, before hex encoding. Computing it
    // over the hex characters instead produces a frame every device rejects.
    uint8_t body[MB_RTU_ADU_MAX];
    body[0] = adu.unit;
    memcpy(body + 1, adu.pdu, adu.pduLen);
    const uint8_t lrc = mb_lrc(body, bodyBytes);

    size_t at = 0;
    out[at++] = (uint8_t)FRAME_START;
    for (size_t i = 0; i < bodyBytes; i++) {
        out[at++] = (uint8_t)mb_hexDigit((uint8_t)(body[i] >> 4));
        out[at++] = (uint8_t)mb_hexDigit((uint8_t)(body[i] & 0x0F));
    }
    out[at++] = (uint8_t)mb_hexDigit((uint8_t)(lrc >> 4));
    out[at++] = (uint8_t)mb_hexDigit((uint8_t)(lrc & 0x0F));
    out[at++] = (uint8_t)FRAME_END_1;
    out[at++] = (uint8_t)FRAME_END_2;
    return at;
}

// =============================================================================
// DECODING
// =============================================================================

void ModbusAsciiCodec::deliver(ModbusAduSink& sink) {
    // Unit + at least a function code + LRC.
    if (_count < 3) {
        sink.onFrameError(ModbusFrameError::Framing, _count);
        _inFrame = false;
        _count = 0;
        return;
    }

    // The LRC is the last byte, and a frame plus its own LRC sums to zero.
    const uint8_t lrc = _bytes[_count - 1];
    if (mb_lrc(_bytes, _count - 1) != lrc) {
        sink.onFrameError(ModbusFrameError::Checksum, _count);
        _inFrame = false;
        _count = 0;
        return;
    }

    ModbusAdu adu;
    adu.unit = _bytes[0];
    adu.pdu = _bytes + 1;
    adu.pduLen = (uint16_t)(_count - 2);
    sink.onAdu(adu);

    _inFrame = false;
    _count = 0;
}

size_t ModbusAsciiCodec::feed(const uint8_t* in, size_t length, mb_time_t nowUs,
                              ModbusAduSink& sink) {
    if (!in || length == 0) return 0;

    for (size_t i = 0; i < length; i++) {
        const char c = (char)in[i];
        _lastCharUs = nowUs;
        _hasChars = true;

        if (c == FRAME_START) {
            if (_inFrame && _count > 0) {
                // A frame was interrupted by the next one starting. Reported
                // rather than swallowed: it means the far end restarted
                // mid-transmission, or something on the line did.
                sink.onFrameError(ModbusFrameError::Framing, _count);
            }
            startFrame();
            continue;
        }

        if (!_inFrame) {
            // Anything outside a frame is noise. Counted a character at a time,
            // because a link left connected to nothing produces it
            // continuously and the COUNT is what distinguishes that from a
            // stray byte.
            sink.onFrameError(ModbusFrameError::Noise, 1);
            continue;
        }

        if (c == FRAME_END_1) continue;   // CR carries nothing; LF ends the frame

        if (c == FRAME_END_2) {
            if (_pendingNibble >= 0) {
                // An odd number of hex digits cannot be a whole number of
                // bytes, so the frame is not one.
                sink.onFrameError(ModbusFrameError::Framing, _count);
                _inFrame = false;
                _count = 0;
                continue;
            }
            deliver(sink);
            continue;
        }

        const int nibble = mb_hexValue(c);
        if (nibble < 0) {
            // A character that is not hex inside a frame. The frame is
            // abandoned and we wait for the next ':' - there is nothing to
            // resynchronise on in between.
            sink.onFrameError(ModbusFrameError::Framing, _count);
            _inFrame = false;
            _count = 0;
            continue;
        }

        if (_pendingNibble < 0) {
            _pendingNibble = nibble;
            continue;
        }

        // A unit id, the longest PDU and the LRC. One byte more is a frame no
        // Modbus device can have sent, and delivering it would hand on a PDU
        // longer than MB_MAX_PDU.
        if (_count >= 1 + MB_MAX_PDU + 1) {
            // Longer than any Modbus frame can be.
            sink.onFrameError(ModbusFrameError::Overrun, _count);
            _inFrame = false;
            _count = 0;
            _pendingNibble = -1;
            continue;
        }

        _bytes[_count++] = (uint8_t)((_pendingNibble << 4) | nibble);
        _pendingNibble = -1;
    }

    return length;
}

void ModbusAsciiCodec::tick(mb_time_t nowUs, ModbusAduSink& sink) {
    if (!_inFrame || !_hasChars) return;
    if (_interCharUs == 0) return;
    if (mb_elapsed(_lastCharUs, nowUs) < _interCharUs) return;

    // A frame that never reached its CRLF. Not a framing MECHANISM - ':' and
    // CRLF do that - only a way of stopping a half-received frame sitting in
    // the buffer for ever after the far end goes away mid-transmission.
    if (_count) sink.onFrameError(ModbusFrameError::Framing, _count);
    _inFrame = false;
    _count = 0;
    _pendingNibble = -1;
}
