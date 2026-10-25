#pragma once

#include <stdint.h>
#include <stddef.h>

// --- Platform detection ---

#if defined(ARDUINO)
    #define MODBUS_PLATFORM_ARDUINO 1
#else
    #define MODBUS_PLATFORM_HOST 1
#endif

// --- Time ---

// Fixed at 32 bits on every target. `unsigned long` is 32-bit on ESP32 and MSVC
// but 64-bit on Linux/macOS, which would change rollover behaviour between
// platforms. Pinning the width keeps timeout arithmetic identical everywhere,
// so a bug that only appears after 49.7 days appears on every target or none.
typedef uint32_t mb_time_t;

// Milliseconds since startup. Wraps every ~49.7 days.
mb_time_t mb_millis();

// Microseconds since startup. Wraps every ~71.6 minutes.
//
// That wrap is short enough to hit in an afternoon, and the interframe gap is
// derived from it. Every subtraction on this clock goes through mb_elapsed().
mb_time_t mb_micros();

// Elapsed time between two stamps. Unsigned arithmetic makes this correct
// across a wrap, provided the interval itself is shorter than the wrap period.
// Prefer it over a bare subtraction: it says the wrap was considered.
inline mb_time_t mb_elapsed(mb_time_t since, mb_time_t now) {
    return (mb_time_t)(now - since);
}

// Whether a deadline is still ahead. Half the range counts as the future, which
// is the standard test for a clock that wraps rather than saturates.
inline bool mb_isFuture(mb_time_t now, mb_time_t deadline) {
    const mb_time_t remaining = mb_elapsed(now, deadline);
    return remaining != 0 && remaining < 0x80000000u;
}

// --- Clock source ---

// The library reads time only through mb_millis() and mb_micros(), and both
// dispatch through a replaceable source. Overriding it serves two purposes:
//
//   Porting  - drive the library from an RTOS tick, a hardware timer, or any
//              clock a bare-metal target provides, without touching the core.
//   Testing  - advance time deterministically, so timing behaviour does not
//              depend on how fast the host runs, and so the 49.7 day rollover
//              can be exercised in milliseconds.
//
// A source must be monotonic within its width and must wrap rather than
// saturate, matching the platform clocks it replaces.
typedef mb_time_t (*mb_clock_fn)();

// Installs a clock source. Passing nullptr for either restores the platform
// clock for that one.
void mb_setClockSource(mb_clock_fn millisSource, mb_clock_fn microsSource);

// Restores both platform clocks.
void mb_resetClockSource();

// --- Yielding ---

// Gives the rest of the system a turn.
//
// Called only from the two places the library can legitimately spin: a send
// that the medium has not made room for yet. Everything else returns and waits
// for the next poll(), which is the whole design - but a frame half-written to
// a socket or a UART cannot be abandoned mid-way without corrupting it, so
// those two loops retry, and a retry loop that does not yield is a loop that
// starves whatever would have cleared the condition.
//
// On Arduino this is the core's own yield(), which is what feeds the watchdog
// and runs the WiFi stack on an ESP8266 - without it, a full transmit buffer on
// that part is a reset rather than a wait.
void mb_yield();
