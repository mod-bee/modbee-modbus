#include "ModbusTcpCodec.h"

#include <string.h>
#include "../ModbusNoExcept.h"

ModbusTcpCodec::ModbusTcpCodec() noexcept {}

void ModbusTcpCodec::reset() { _length = 0; }

void ModbusTcpCodec::abandon(ModbusAduSink& sink, ModbusFrameError why) {
    if (why == ModbusFrameError::Protocol) _protocolErrors++;
    sink.onFrameError(why, _length);
    _length = 0;
}

// =============================================================================
// ENCODING
// =============================================================================

size_t ModbusTcpCodec::encode(uint8_t* out, size_t capacity, const ModbusAdu& adu) {
    if (!out || !adu.pdu || adu.pduLen == 0 || adu.pduLen > MB_MAX_PDU) return 0;

    const size_t needed = HEADER_BYTES + adu.pduLen;
    if (needed > capacity || needed > MB_TCP_ADU_MAX) return 0;

    mb_packBE16(out + OFFSET_TRANSACTION, adu.transactionId);
    mb_packBE16(out + OFFSET_PROTOCOL, PROTOCOL_MODBUS);

    // The length covers the unit id and the PDU - everything AFTER this field,
    // not the whole frame. Off by one here and every frame is rejected by every
    // device, which at least fails loudly.
    mb_packBE16(out + OFFSET_LENGTH, (uint16_t)(1 + adu.pduLen));

    out[OFFSET_UNIT] = adu.unit;
    memcpy(out + HEADER_BYTES, adu.pdu, adu.pduLen);
    return needed;
}

// =============================================================================
// DECODING
// =============================================================================

size_t ModbusTcpCodec::feed(const uint8_t* in, size_t length, mb_time_t nowUs,
                            ModbusAduSink& sink) {
    (void)nowUs;   // TCP framing needs no timing at all
    if (!in || length == 0) return 0;

    size_t consumed = 0;

    while (consumed < length) {
        // Take what will fit, then see how many whole frames came out. A single
        // read may deliver half a frame, three frames, or two and a half; all
        // three are ordinary.
        const size_t room = MB_TCP_ADU_MAX - _length;
        if (room == 0) {
            // Cannot happen with a well-formed stream, because a frame is
            // delivered the moment its declared length arrives. Reaching here
            // means the length field lied and was not caught, so the stream is
            // not Modbus TCP.
            abandon(sink, ModbusFrameError::Overrun);
            continue;
        }

        const size_t take = (length - consumed) < room ? (length - consumed) : room;
        memcpy(_buffer + _length, in + consumed, take);
        _length += take;
        consumed += take;

        // Now pull out every complete frame the buffer holds.
        for (;;) {
            if (_length < HEADER_BYTES) break;   // Header still arriving

            const uint16_t protocol = mb_unpackBE16(_buffer + OFFSET_PROTOCOL);
            if (protocol != PROTOCOL_MODBUS) {
                // Another protocol on the same port. Very often this is Modbus
                // RTU tunnelled over TCP, which is a different thing with the
                // same port number, and telling the two apart is exactly what
                // this field is for.
                abandon(sink, ModbusFrameError::Protocol);
                break;
            }

            const uint16_t declared = mb_unpackBE16(_buffer + OFFSET_LENGTH);

            // The length covers the unit id plus the PDU, so it is at least 2
            // - one unit id and one function code - and at most 254.
            if (declared < 2 || declared > 1 + MB_MAX_PDU) {
                abandon(sink, ModbusFrameError::Framing);
                break;
            }

            const size_t frameLen = HEADER_BYTES - 1 + declared;
            if (_length < frameLen) break;   // Payload still arriving

            ModbusAdu adu;
            adu.transactionId = mb_unpackBE16(_buffer + OFFSET_TRANSACTION);
            adu.hasTransactionId = true;
            adu.unit = _buffer[OFFSET_UNIT];
            adu.pdu = _buffer + HEADER_BYTES;
            adu.pduLen = (uint16_t)(declared - 1);
            sink.onAdu(adu);

            // Keep whatever followed. Two frames in one segment is ordinary,
            // and dropping the tail would lose a frame that had already
            // arrived safely.
            const size_t leftover = _length - frameLen;
            if (leftover) memmove(_buffer, _buffer + frameLen, leftover);
            _length = leftover;
        }
    }

    return consumed;
}
