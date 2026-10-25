#pragma once

#include "../ModbusCodec.h"
#include "../ModbusCrc.h"

// =============================================================================
// MODBUS RTU
// =============================================================================
//
//     [unit address : 1][PDU : 1..253][CRC-16 : 2, low byte first]
//
// 256 bytes at most. No start or end delimiter: a frame is delimited by
// SILENCE, which is the whole difficulty of RTU and the reason this file ships
// two strategies rather than one.
//
// --- Gap framing: the specification's own ---
//
// A frame ends after t3.5 character times of silence, and characters within one
// frame are never more than t1.5 apart. Both are computed from the line rate,
// EXCEPT above 19200 baud where the specification fixes them at 750 us and
// 1.750 ms - because the scaled figures stop being resolvable. At 115200 the
// scaled t3.5 would be 334 us, which most operating systems and a busy Arduino
// loop cannot schedule anywhere near.
//
// Gap framing is correct and it is what the specification says. It needs the
// application to call poll() far more often than t1.5, and it needs the
// transport to report the line rate.
//
// --- Length framing: what actually works on a general-purpose OS ---
//
// Read the function code, work out how long the frame must be, take exactly
// that many bytes, check the CRC. No timing at all.
//
// This is what makes RTU work on Linux, on Windows, over a USB serial adapter
// that buffers 16 ms of traffic, and in an Arduino loop that also drives a
// display. It is what diagslave's "-f #  tolerance for inter-frame gap
// detection" option exists to work around, seen from the other side.
//
// Its cost: it must know whether it is decoding requests or responses, because
// the same function code has different lengths in each direction. And two
// shapes cannot be length-framed at all:
//
//   0x08 Diagnostics    the sub-function data length is not declared anywhere
//   0x2B/0x0E responses the object list has to be walked to be measured
//
// Those fall back to gap framing when the line rate is known, and to a timeout
// when it is not. Both are noted by framingMode().
//
// --- Choosing ---
//
// Auto picks Gap when the transport states a line rate AND the measured
// service interval is comfortably inside t1.5, and Length otherwise. The choice
// is reported, because "why is my RTU splitting frames" is the most common
// question in every Modbus library and the answer should be one call away.

class ModbusRtuCodec : public ModbusCodec {
public:
    ModbusRtuCodec() noexcept;

    // `bitsPerSecond` comes from the transport. Zero means unknown, which rules
    // gap framing out.
    void setLineRate(uint32_t bitsPerSecond);

    // The interval the application actually calls poll() at, in microseconds,
    // measured by the link. Gap framing cannot be enforced any finer than this:
    // a t1.5 of 750 us is meaningless to a loop that runs every 5 ms.
    void setServiceInterval(mb_time_t intervalUs);

    // Silence after which a buffer that has produced no frame is treated as
    // over, in microseconds.
    //
    // Length framing needs this as a BACKSTOP and cannot be correct without
    // one. A noise byte ahead of a frame usually makes the predictor compute a
    // length far longer than the frame really is, and the predictor then waits
    // for bytes that are never coming - so something other than the predictor
    // has to notice that the line has gone quiet. This is that something, and
    // it is what diagslave's "-f  tolerance for inter-frame gap detection"
    // option is tuning, seen from the other side.
    //
    // t3.5 is used where the line rate is known. This value stands in where it
    // is not, and must be longer than any gap the medium itself may introduce
    // mid-frame - a USB serial adapter's buffering, say.
    void setIdleTimeout(mb_time_t idleUs);
    mb_time_t idleTimeoutUs() const;

    // Pins the strategy. Auto is the default and is usually right.
    void setFraming(ModbusFraming framing);

    // Which strategy is in force, after Auto has decided.
    ModbusFraming framingMode() const;

    // Why, in a sentence an integrator can act on.
    const char* framingReason() const;

    uint32_t t1_5Us() const { return mb_t1_5Us(_bitsPerSecond); }
    uint32_t t3_5Us() const { return mb_t3_5Us(_bitsPerSecond); }

    // --- ModbusCodec ---
    size_t encode(uint8_t* out, size_t capacity, const ModbusAdu& adu) override;
    size_t feed(const uint8_t* in, size_t length, mb_time_t nowUs,
                ModbusAduSink& sink) override;
    void tick(mb_time_t nowUs, ModbusAduSink& sink) override;
    void reset() override;
    void expect(ModbusExpect what) override { _expect = what; }
    ModbusExpect expecting() const override { return _expect; }
    uint32_t txGapUs() const override;
    size_t maxAduBytes() const override { return MB_RTU_ADU_MAX; }
    const char* name() const override { return "RTU"; }

    // How many times a bad checksum forced a resynchronisation. Sustained
    // growth means noise, a floating bus, or a baud rate mismatch.
    uint32_t resyncCount() const { return _resyncs; }

    // --- Length prediction, exposed for testing ---
    //
    // The expected ADU length given the bytes so far, or 0 when more bytes are
    // needed to tell, or MB_RTU_LENGTH_UNKNOWABLE for the two shapes that
    // carry no length at all.
    static size_t predictLength(const uint8_t* adu, size_t available,
                                ModbusExpect expect);

    static const size_t LENGTH_NEED_MORE = 0;
    static const size_t LENGTH_UNKNOWABLE = (size_t)-1;

private:
    uint32_t _bitsPerSecond = 0;
    mb_time_t _serviceIntervalUs = 0;
    mb_time_t _idleUs = 0;
    ModbusFraming _requested = ModbusFraming::Auto;
    ModbusExpect _expect = ModbusExpect::Request;

    uint8_t _buffer[MB_RTU_ADU_MAX];
    size_t _length = 0;
    mb_time_t _lastByteUs = 0;
    bool _hasBytes = false;

    // Set when a checksum failed and the buffer was abandoned. Cleared by the
    // next silence, so a frame that arrives mid-recovery is not half-consumed.
    bool _hunting = false;

    uint32_t _resyncs = 0;

    // Tries to take a complete frame off the front of the buffer. Returns true
    // if it made progress - either delivering a frame or discarding a byte that
    // provably cannot start one.
    bool tryLengthFrame(ModbusAduSink& sink);

    // The backstop. Searches the whole buffer for a frame starting at any
    // offset, requiring the predicted length AND the checksum to agree, and
    // reports whatever it had to skip as noise.
    //
    // Only ever called once the line has gone quiet, which is what keeps it
    // both cheap and safe: cheap because it runs on a recovery path rather than
    // per byte, and safe because a frame still arriving cannot be mistaken for
    // a complete one when nothing is arriving.
    bool recoverLengthFrame(ModbusAduSink& sink);

    // Delivers whatever is in the buffer as one frame, if it checks out.
    void deliverBuffered(ModbusAduSink& sink);

    // Drops `count` bytes from the front, keeping what follows.
    void consume(size_t count);

    void reportNoise(ModbusAduSink& sink, size_t bytes);
};
