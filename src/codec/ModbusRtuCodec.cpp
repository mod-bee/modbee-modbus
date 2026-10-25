#include "ModbusRtuCodec.h"

#include <string.h>
#include "../ModbusNoExcept.h"

const char* mb_frameErrorName(ModbusFrameError error) {
    switch (error) {
        case ModbusFrameError::Checksum: return "checksum";
        case ModbusFrameError::Framing:  return "framing";
        case ModbusFrameError::Overrun:  return "overrun";
        case ModbusFrameError::Noise:    return "noise";
        case ModbusFrameError::Protocol: return "protocol";
    }
    return "unknown";
}

const char* mb_framingName(ModbusFraming framing) {
    switch (framing) {
        case ModbusFraming::Auto:   return "auto";
        case ModbusFraming::Gap:    return "gap";
        case ModbusFraming::Length: return "length";
    }
    return "unknown";
}

ModbusRtuCodec::ModbusRtuCodec() noexcept {}

void ModbusRtuCodec::setLineRate(uint32_t bitsPerSecond) { _bitsPerSecond = bitsPerSecond; }
void ModbusRtuCodec::setServiceInterval(mb_time_t intervalUs) { _serviceIntervalUs = intervalUs; }
void ModbusRtuCodec::setIdleTimeout(mb_time_t idleUs) { _idleUs = idleUs; }

mb_time_t ModbusRtuCodec::idleTimeoutUs() const {
    // t3.5 where the line rate says what it is. Otherwise the configured
    // value, and failing that a default long enough to survive the buffering a
    // USB serial adapter introduces mid-frame - which is milliseconds, not
    // microseconds - while still being far shorter than any response timeout.
    const uint32_t gap = t3_5Us();
    if (gap) return gap;
    return _idleUs ? _idleUs : 5000u;
}
void ModbusRtuCodec::setFraming(ModbusFraming framing) { _requested = framing; }

ModbusFraming ModbusRtuCodec::framingMode() const {
    if (_requested != ModbusFraming::Auto) return _requested;

    // LENGTH FRAMING WHEREVER THE DIRECTION IS KNOWN.
    //
    // This is not the specification's own mechanism, and choosing it as the
    // default deserves the argument.
    //
    // Gap framing asks: has the line been silent for t3.5? That is the right
    // question only if silence on the line and silence at THIS END are the same
    // thing. They are not, on anything that buffers:
    //
    //   a USB serial adapter    delivers a frame's bytes in one burst, whenever
    //                           its own 1 ms polling interval comes round
    //   a pseudo-terminal       delivers whenever the kernel schedules the
    //                           writer, with no pacing at all
    //   a general-purpose OS    can stall this process for longer than t3.5
    //                           between two reads of the same frame
    //
    // Every one of those inserts a gap the framer cannot tell from a frame
    // boundary, and each such gap splits one frame into two that both fail
    // their checksum. It is not rare: it was observed on the first end-to-end
    // run of the gateway example, over a socat pseudo-terminal, with a service
    // interval well inside t1.5 - which is exactly the condition that made the
    // old heuristic choose gap framing.
    //
    // Length framing asks a question the medium cannot interfere with: how long
    // does this function code say the frame is? Read that many bytes, check the
    // CRC. Buffering, bursts and scheduling stalls are all invisible to it.
    //
    // Its one requirement is knowing whether requests or responses are being
    // decoded, because the same function code has different lengths each way.
    // Where that is not known - a listener, a sniffer, a node in both roles -
    // there is no table to consult and gap framing is the only option left.
    //
    // The idle backstop covers what the predictor cannot measure:
    // the two response shapes that carry no length, and any vendor function
    // code we do not know. Those are delivered after silence, which is where a
    // buffered medium's gaps stop mattering because the line really has gone
    // quiet.
    if (_expect != ModbusExpect::Either) return ModbusFraming::Length;

    // A listener cannot predict lengths, so it gets gap framing and has to
    // live with the timing requirement - which also means it needs the line
    // rate.
    return _bitsPerSecond == 0 ? ModbusFraming::Length : ModbusFraming::Gap;
}

const char* ModbusRtuCodec::framingReason() const {
    if (_requested == ModbusFraming::Gap)    return "gap framing was pinned by configuration";
    if (_requested == ModbusFraming::Length) return "length framing was pinned by configuration";

    if (_expect == ModbusExpect::Either) {
        if (_bitsPerSecond == 0) {
            return "length framing: this node reads both directions, so lengths cannot "
                   "be predicted - and with no line rate, gaps cannot be measured either";
        }
        return "gap framing: this node reads both directions, so the function code "
               "cannot tell it how long a frame is";
    }

    return "length framing: the direction is known, so a frame's length can be read "
           "from its function code - which no amount of buffering between the line "
           "and this process can interfere with";
}

uint32_t ModbusRtuCodec::txGapUs() const {
    // t3.5 where it is known. Where it is not, a conservative stand-in: 1.75 ms
    // is t3.5 at 19200 and longer than t3.5 at anything faster, so transmitting
    // after it is never too early on a link we cannot measure.
    const uint32_t gap = t3_5Us();
    return gap ? gap : MODBUS_FIXED_T3_5_US;
}

void ModbusRtuCodec::reset() {
    _length = 0;
    _hasBytes = false;
    _hunting = false;
}

void ModbusRtuCodec::consume(size_t count) {
    if (count >= _length) {
        _length = 0;
        return;
    }
    memmove(_buffer, _buffer + count, _length - count);
    _length -= count;
}

void ModbusRtuCodec::reportNoise(ModbusAduSink& sink, size_t bytes) {
    if (bytes) sink.onFrameError(ModbusFrameError::Noise, bytes);
}

// =============================================================================
// ENCODING
// =============================================================================

size_t ModbusRtuCodec::encode(uint8_t* out, size_t capacity, const ModbusAdu& adu) {
    if (!out || !adu.pdu || adu.pduLen == 0 || adu.pduLen > MB_MAX_PDU) return 0;

    const size_t needed = 1 + (size_t)adu.pduLen + 2;
    if (needed > capacity || needed > MB_RTU_ADU_MAX) return 0;

    out[0] = adu.unit;
    memcpy(out + 1, adu.pdu, adu.pduLen);

    // Low byte first. The one little-endian field on the whole Modbus wire, and
    // the reason mb_crc16Append exists rather than a bare mb_packBE16.
    mb_crc16Append(out + 1 + adu.pduLen, mb_crc16(out, 1 + adu.pduLen));
    return needed;
}

// =============================================================================
// LENGTH PREDICTION
// =============================================================================

size_t ModbusRtuCodec::predictLength(const uint8_t* adu, size_t available,
                                     ModbusExpect expect) {
    // [unit][function][...]. Nothing can be said before the function code.
    if (available < 2) return LENGTH_NEED_MORE;

    const uint8_t function = adu[1];

    // An exception response is the same two bytes whatever it answers, so it
    // is checked before the per-function tables and before the direction: a
    // server receiving one has heard another master's traffic, and a client
    // receiving one is being refused.
    if (mb_isExceptionFunction(function)) {
        return 1 + 2 + 2;    // unit, FC|0x80, code, CRC
    }

    // A listener cannot choose a table, because it does not know which
    // direction it is hearing.
    if (expect == ModbusExpect::Either) return LENGTH_UNKNOWABLE;

    const bool request = (expect == ModbusExpect::Request);

    switch (function) {
        // --- Fixed-length both ways ---
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            return 1 + 5 + 2;                     // unit, FC + addr + value, CRC

        case MB_FC_MASK_WRITE_REGISTER:
            return 1 + 7 + 2;

        // --- Reads: fixed request, counted response ---
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            if (request) return 1 + 5 + 2;
            if (available < 3) return LENGTH_NEED_MORE;
            return 1 + 2 + (size_t)adu[2] + 2;    // unit, FC + byteCount, data, CRC

        // --- Multiple writes: counted request, fixed response ---
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            if (!request) return 1 + 5 + 2;
            if (available < 7) return LENGTH_NEED_MORE;
            return 1 + 6 + (size_t)adu[6] + 2;    // unit, FC+addr+qty+count, data, CRC

        // --- 0x17: counted both ways, at different offsets ---
        case MB_FC_READ_WRITE_REGISTERS:
            if (request) {
                if (available < 11) return LENGTH_NEED_MORE;
                return 1 + 10 + (size_t)adu[10] + 2;
            }
            if (available < 3) return LENGTH_NEED_MORE;
            return 1 + 2 + (size_t)adu[2] + 2;

        // --- Serial-only, no parameters in the request ---
        case MB_FC_READ_EXCEPTION_STATUS:
            return request ? 1 + 1 + 2 : 1 + 2 + 2;

        case MB_FC_GET_COMM_EVENT_COUNTER:
            return request ? 1 + 1 + 2 : 1 + 5 + 2;   // status(2) + count(2)

        case MB_FC_GET_COMM_EVENT_LOG:
        case MB_FC_REPORT_SERVER_ID:
            if (request) return 1 + 1 + 2;
            if (available < 3) return LENGTH_NEED_MORE;
            return 1 + 2 + (size_t)adu[2] + 2;

        // --- File records: counted both ways, at the same offset ---
        case MB_FC_READ_FILE_RECORD:
        case MB_FC_WRITE_FILE_RECORD:
            if (available < 3) return LENGTH_NEED_MORE;
            return 1 + 2 + (size_t)adu[2] + 2;

        // --- FIFO: fixed request, two-byte count in the response ---
        case MB_FC_READ_FIFO_QUEUE:
            if (request) return 1 + 3 + 2;
            if (available < 4) return LENGTH_NEED_MORE;
            return 1 + 3 + (size_t)mb_unpackBE16(&adu[2]) + 2;

        // --- 0x08: the request declares no length ---
        //
        // Every sub-function the specification defines carries exactly two data
        // bytes, so that is assumed for a request. A response is echoed for
        // Return Query Data and could be any length, so it is unknowable.
        case MB_FC_DIAGNOSTICS:
            return request ? (size_t)(1 + 5 + 2) : LENGTH_UNKNOWABLE;

        // --- 0x2B: fixed request for MEI 0x0E, walkable response ---
        case MB_FC_ENCAPSULATED_INTERFACE:
            if (!request) return LENGTH_UNKNOWABLE;
            if (available < 3) return LENGTH_NEED_MORE;
            if (adu[2] != MB_MEI_READ_DEVICE_ID) return LENGTH_UNKNOWABLE;
            return 1 + 4 + 2;

        default:
            // An unknown function code cannot be measured. Under length framing
            // that means the buffer is carried until silence or until it
            // overruns, rather than being consumed at a guess - guessing would
            // eat the start of the next frame as well.
            return LENGTH_UNKNOWABLE;
    }
}

// =============================================================================
// DELIVERY
// =============================================================================

void ModbusRtuCodec::deliverBuffered(ModbusAduSink& sink) {
    if (_length == 0) return;

    // A frame shorter than address + function + CRC cannot be one. On a line
    // that has just gone quiet after noise, this is the usual case.
    if (_length < 4) {
        sink.onFrameError(ModbusFrameError::Framing, _length);
        _length = 0;
        return;
    }

    if (!mb_crc16Check(_buffer, _length)) {
        sink.onFrameError(ModbusFrameError::Checksum, _length);
        _length = 0;
        return;
    }

    ModbusAdu adu;
    adu.unit = _buffer[0];
    adu.pdu = _buffer + 1;
    adu.pduLen = (uint16_t)(_length - 3);
    sink.onAdu(adu);
    _length = 0;
}

bool ModbusRtuCodec::tryLengthFrame(ModbusAduSink& sink) {
    if (_length < 4) return false;

    const size_t predicted = predictLength(_buffer, _length, _expect);
    if (predicted == LENGTH_NEED_MORE) return false;

    if (predicted == LENGTH_UNKNOWABLE) {
        // Two shapes carry no length anywhere in them - a diagnostics response
        // and a device-identification response - and an unknown function code
        // cannot be measured at all. Nothing is consumed at a guess, because a
        // guess that is too long eats the start of the next frame as well.
        //
        // These are delivered by the idle backstop instead. That is the whole
        // reason length framing has one.
        return false;
    }

    if (predicted > MB_RTU_ADU_MAX) {
        // Structurally impossible: a byte count that would put the frame past
        // what RTU allows. The leading byte is not the start of a frame.
        sink.onFrameError(ModbusFrameError::Framing, 1);
        consume(1);
        _resyncs++;
        return true;   // Progress was made; try again from the new front
    }

    if (_length < predicted) return false;   // Still arriving

    if (mb_crc16Check(_buffer, predicted)) {
        ModbusAdu adu;
        adu.unit = _buffer[0];
        adu.pdu = _buffer + 1;
        adu.pduLen = (uint16_t)(predicted - 3);
        sink.onAdu(adu);
        consume(predicted);
        return true;
    }

    // The checksum failed, so the frame did not start where we thought it did.
    // Drop ONE byte and try again from the next.
    //
    // One byte rather than the whole buffer: the common cause is a noise byte
    // ahead of a good frame, and dropping everything would take the good frame
    // with it. A lost byte inside a frame resynchronises on the next frame's
    // address instead, which costs one frame and recovers by itself.
    sink.onFrameError(ModbusFrameError::Checksum, 1);
    consume(1);
    _resyncs++;
    return true;
}


bool ModbusRtuCodec::recoverLengthFrame(ModbusAduSink& sink) {
    if (_length < 4) return false;

    // Every possible start offset, cheapest test first.
    //
    // A candidate is accepted only when the PREDICTED LENGTH and the CHECKSUM
    // agree. Either alone is too weak: a checksum alone matches by chance once
    // in 65536, and over a couple of hundred offsets that is a frame invented
    // out of noise several times a day on a busy bus. Requiring the function
    // code, the counts inside it and the checksum to line up together is what
    // makes this safe to run at all.
    for (size_t offset = 0; offset + 4 <= _length; offset++) {
        const uint8_t* start = _buffer + offset;
        const size_t available = _length - offset;

        const size_t predicted = predictLength(start, available, _expect);

        if (predicted != LENGTH_NEED_MORE && predicted != LENGTH_UNKNOWABLE) {
            if (predicted <= available && predicted >= 4 &&
                mb_crc16Check(start, predicted)) {
                if (offset) reportNoise(sink, offset);

                ModbusAdu adu;
                adu.unit = start[0];
                adu.pdu = start + 1;
                adu.pduLen = (uint16_t)(predicted - 3);
                sink.onAdu(adu);
                consume(offset + predicted);
                if (offset) _resyncs++;
                return true;
            }
            continue;
        }

        if (predicted != LENGTH_UNKNOWABLE) continue;

        // No length to predict. The checksum is the only evidence, so every
        // candidate end is tried - with the CRC extended one byte at a time
        // rather than recomputed from scratch, which keeps this linear in the
        // buffer rather than quadratic.
        uint16_t running = mb_crc16(start, 2);
        for (size_t frameLen = 5; frameLen <= available; frameLen++) {
            running = mb_crc16Update(running, start + frameLen - 3, 1);
            if (running != mb_crc16Read(start + frameLen - 2)) continue;

            if (offset) reportNoise(sink, offset);
            ModbusAdu adu;
            adu.unit = start[0];
            adu.pdu = start + 1;
            adu.pduLen = (uint16_t)(frameLen - 3);
            sink.onAdu(adu);
            consume(offset + frameLen);
            if (offset) _resyncs++;
            return true;
        }
    }

    return false;
}

// =============================================================================
// FEEDING
// =============================================================================

size_t ModbusRtuCodec::feed(const uint8_t* in, size_t length, mb_time_t nowUs,
                            ModbusAduSink& sink) {
    if (!in || length == 0) return 0;

    const ModbusFraming framing = framingMode();
    const mb_time_t idle = idleTimeoutUs();

    size_t consumed = 0;
    for (size_t i = 0; i < length; i++) {
        // Silence since the previous byte ends whatever was being assembled,
        // whichever strategy is in force. Under gap framing that is the frame
        // boundary; under length framing it is a safety net that stops a runt
        // sitting in the buffer for ever.
        if (_hasBytes && _length != 0 && mb_elapsed(_lastByteUs, nowUs) >= idle) {
            // Silence since the previous byte ends whatever was being
            // assembled. Under gap framing that IS the frame boundary; under
            // length framing it is the backstop, without which a noise byte
            // ahead of a frame leaves the predictor waiting for bytes that are
            // never coming.
            if (framing == ModbusFraming::Gap) {
                deliverBuffered(sink);
            } else {
                while (_length >= 4 && recoverLengthFrame(sink)) {
                }
                if (_length != 0) {
                    reportNoise(sink, _length);
                    _length = 0;
                }
            }
            _hunting = false;
        }

        if (_length >= MB_RTU_ADU_MAX) {
            // Longer than any RTU frame. Under length framing the predictor has
            // already had every chance; under gap framing the line has been
            // talking without a break for longer than a frame can be.
            sink.onFrameError(ModbusFrameError::Overrun, _length);
            _length = 0;
            _hunting = true;
            _resyncs++;
        }

        _buffer[_length++] = in[i];
        _lastByteUs = nowUs;
        _hasBytes = true;
        consumed++;

        if (framing == ModbusFraming::Length) {
            // Take as many whole frames as the buffer now holds. The loop
            // matters: a USB serial adapter delivers several frames in one read,
            // and stopping after the first would leave the rest to be found only
            // when the next byte arrives.
            while (tryLengthFrame(sink)) {
                if (_length == 0) break;
            }
            _hunting = (_length != 0);
        }
    }

    return consumed;
}

void ModbusRtuCodec::tick(mb_time_t nowUs, ModbusAduSink& sink) {
    if (!_hasBytes || _length == 0) return;
    if (mb_elapsed(_lastByteUs, nowUs) < idleTimeoutUs()) return;

    if (framingMode() == ModbusFraming::Gap) {
        deliverBuffered(sink);
        _hunting = false;
        return;
    }

    // Length framing's backstop. The head has had every chance and the line has
    // gone quiet, so whatever is here is a complete unit: either it contains a
    // frame somewhere, or it is noise.
    //
    // Several frames may be recovered in one go, because a burst can leave more
    // than one behind.
    while (_length >= 4 && recoverLengthFrame(sink)) {
        // Keep going: the tail may hold another.
    }

    if (_length != 0) {
        reportNoise(sink, _length);
        _length = 0;
    }
    _hunting = false;
}
