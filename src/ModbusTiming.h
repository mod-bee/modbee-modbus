#pragma once

#include "ModbusPlatform.h"

// =============================================================================
// MEASURED TIMING
// =============================================================================
//
// Every timing field defaults to MODBUS_AUTO, meaning the library measures it.
// Setting a field pins it and turns adaptation off FOR THAT FIELD ALONE; the
// rest still tunes itself.
//
// That is what lets one configuration suit a 9600 baud RS-485 segment and a
// 115200 one, a VFD that answers in 4 ms and a soft-starter that takes 300,
// without the application being told about either.

// A timing field left at MODBUS_AUTO is derived from measurement. Any other
// value pins that field. Zero was never a legal setting for any of them, so
// the sentinel is unambiguous.
#define MODBUS_AUTO 0

// A configured setting, or the built-in figure where none was given. Every
// tunable reads through this, so MODBUS_AUTO means the same thing everywhere
// and two accessors cannot disagree about one number.
template <typename T>
inline T mb_setting(T configured, T builtIn) {
    return configured != (T)MODBUS_AUTO ? configured : builtIn;
}

// =============================================================================
// SERIAL LINE CHARACTER TIMING
// =============================================================================

// Bits per character on a Modbus serial line. ALWAYS 11:
//
//   with parity     1 start + 8 data + 1 parity + 1 stop
//   without parity  1 start + 8 data + 2 stop
//
// The specification mandates the second stop bit precisely so the character
// time does not change. A library that assumes plain 8N1 and uses 10 computes
// every derived gap ten percent short, which on a marginal line turns into
// frames that split at the receiver and cannot be explained from either end.
#define MODBUS_BITS_PER_CHARACTER 11

// Above this rate the specification FIXES the two timers rather than scaling
// them, because the scaled figures stop being resolvable on real hardware.
#define MODBUS_FIXED_TIMING_ABOVE_BPS 19200

// The fixed values, in microseconds, used above that rate.
#define MODBUS_FIXED_T1_5_US  750u
#define MODBUS_FIXED_T3_5_US 1750u

// Microseconds one character occupies on the wire at this rate.
inline uint32_t mb_characterTimeUs(uint32_t bitsPerSecond) {
    if (bitsPerSecond == 0) return 0;
    return (uint32_t)((1000000ull * MODBUS_BITS_PER_CHARACTER) / bitsPerSecond);
}

// Maximum silence permitted BETWEEN characters of one frame. Longer than this
// and the frame is over (or broken).
inline uint32_t mb_t1_5Us(uint32_t bitsPerSecond) {
    if (bitsPerSecond == 0) return 0;
    if (bitsPerSecond > MODBUS_FIXED_TIMING_ABOVE_BPS) return MODBUS_FIXED_T1_5_US;
    // 1.5 character times, rounded up: a gap stated shorter than it should be
    // splits frames, where one stated longer merely waits.
    return (mb_characterTimeUs(bitsPerSecond) * 3 + 1) / 2;
}

// Silence that delimits one frame from the next, and the quiet a transmitter
// must leave before it starts.
inline uint32_t mb_t3_5Us(uint32_t bitsPerSecond) {
    if (bitsPerSecond == 0) return 0;
    if (bitsPerSecond > MODBUS_FIXED_TIMING_ABOVE_BPS) return MODBUS_FIXED_T3_5_US;
    return (mb_characterTimeUs(bitsPerSecond) * 7 + 1) / 2;
}

// Microseconds `bytes` take to clock out at this rate. Used to size a response
// window against what the medium can physically deliver, so a timeout is never
// shorter than the airtime of the answer it is waiting for.
inline uint32_t mb_airtimeUs(uint32_t bitsPerSecond, size_t bytes) {
    if (bitsPerSecond == 0) return 0;
    return (uint32_t)((1000000ull * MODBUS_BITS_PER_CHARACTER * bytes) / bitsPerSecond);
}

// =============================================================================
// BOUNDS
// =============================================================================

// The floor under every derived timeout: a timeout shorter than the interval
// between this node's own looks at the link cannot be enforced, whatever the
// device is doing. Measured from that interval, so an application running a
// tight loop derives a tighter floor than one carrying heavy work.
//
// MODBUS_TIMING_FLOOR_MS stands in until the interval has been measured, and
// MIN is the point below which a millisecond clock says nothing.
#define MODBUS_TIMING_FLOOR_MS      20
#define MODBUS_TIMING_FLOOR_MIN_MS   2
#define MODBUS_TIMING_CEILING_MS 60000

// Samples before an estimate is trusted.
#define MODBUS_ESTIMATOR_WARMUP 4

// How much of a retry backoff is jittered, as a percentage. A backoff that
// doubles deterministically leaves two masters on one bus in step, colliding
// on every attempt.
#define MODBUS_RETRY_SPREAD_PERCENT 50

// Conventional pause after a broadcast, during which the client sends nothing.
// Every server on the segment is executing the write and none of them will
// answer, so there is no other signal that they are done.
#define MODBUS_TURNAROUND_MS 100

// Attempts a retry counter can express. The counters are single bytes, so a
// configured budget above this would be compared against a counter that wraps
// before reaching it: the comparison would never come true and a lost
// operation would never report a timeout. Read every configured retry budget
// through mb_retryLimit().
#define MODBUS_MAX_RETRIES 255

inline uint8_t mb_retryLimit(uint32_t configured) {
    return configured > MODBUS_MAX_RETRIES ? (uint8_t)MODBUS_MAX_RETRIES
                                           : (uint8_t)configured;
}

// A deterministic spread in [0, span), keyed on whatever the caller passes.
// Deterministic because a device has no entropy at start-up; all that is
// needed is that two masters, or two attempts, land on different numbers.
inline mb_time_t mb_spread(uint32_t seed, mb_time_t span) {
    if (span == 0) return 0;
    uint32_t hash = seed * 2654435761u;      // Knuth multiplicative
    hash ^= hash >> 13;
    hash *= 2246822519u;
    hash ^= hash >> 16;
    return (mb_time_t)(hash % (uint32_t)span);
}

// =============================================================================
// ROUND-TRIP ESTIMATOR
// =============================================================================

// Smoothed estimate of a recurring interval: TCP's round-trip estimator
// (RFC 6298) in integer arithmetic.
//
// The variance term is what makes it fit a Modbus bus. A VFD that usually
// answers in 4 ms and occasionally takes 40 has a modest mean and a wide
// spread, and a timeout derived from the mean alone would report it failed
// several times an hour.
class ModbusEstimator {
public:
    void reset() {
        _srtt = 0;
        _rttvar = 0;
        _samples = 0;
    }

    void addSample(mb_time_t sample) {
        if (sample > MODBUS_TIMING_CEILING_MS) sample = MODBUS_TIMING_CEILING_MS;

        if (_samples == 0) {
            _srtt = sample;
            _rttvar = sample / 2;
            _samples = 1;
            return;
        }

        if (_samples < MODBUS_ESTIMATOR_WARMUP) {
            // Count the sample before smoothing it in, so a link whose
            // round trips all measure zero still reaches readiness.
            _samples++;
        }

        // Signed, because a sample below the current estimate has to pull it
        // down as well as up.
        const int32_t diff = (int32_t)sample - (int32_t)_srtt;
        const int32_t magnitude = diff < 0 ? -diff : diff;

        _rttvar = (mb_time_t)((int32_t)_rttvar + (magnitude - (int32_t)_rttvar) / 4);

        // Asymmetric: a derived timeout feeds back into the round trip it
        // measures, so growing at a quarter and shrinking at an eighth keeps
        // the loop from oscillating.
        _srtt = (mb_time_t)((int32_t)_srtt + (diff > 0 ? diff / 4 : diff / 8));
    }

    bool isReady() const { return _samples >= MODBUS_ESTIMATOR_WARMUP; }
    bool hasSample() const { return _samples > 0; }
    mb_time_t smoothed() const { return _srtt; }

    // How long to wait before concluding the answer is not coming: the mean
    // plus four times the observed spread.
    //
    // The floor is the caller's to state, because it describes the host the
    // estimate will be acted on by - which an estimator of link timing cannot
    // know.
    mb_time_t timeout(mb_time_t floorMs) const {
        const mb_time_t margin = _rttvar * 4;
        mb_time_t value = _srtt + (margin > 1 ? margin : 1);

        if (value < floorMs) value = floorMs;
        if (value > MODBUS_TIMING_CEILING_MS) value = MODBUS_TIMING_CEILING_MS;
        return value;
    }

private:
    mb_time_t _srtt = 0;
    mb_time_t _rttvar = 0;
    uint8_t _samples = 0;
};
