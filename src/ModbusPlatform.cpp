#include "ModbusPlatform.h"

#if defined(MODBUS_PLATFORM_ARDUINO)

#include <Arduino.h>

// On a board every include is above here, so the whole file goes without
// exception tables; the host part further down still includes <thread>.
#include "ModbusNoExcept.h"

static mb_time_t platformMillis() { return (mb_time_t)millis(); }
static mb_time_t platformMicros() { return (mb_time_t)micros(); }

#else

#include <chrono>

namespace {

// Anchored at first use so both clocks start near zero, matching Arduino. A
// host clock that starts at an arbitrary large value would hide rollover bugs
// on the host and expose them only on the target.
std::chrono::steady_clock::time_point startupTime() {
    static const std::chrono::steady_clock::time_point t0 =
        std::chrono::steady_clock::now();
    return t0;
}

} // namespace

static mb_time_t platformMillis() {
    using namespace std::chrono;
    const auto delta = steady_clock::now() - startupTime();
    return (mb_time_t)(uint64_t)duration_cast<milliseconds>(delta).count();
}

static mb_time_t platformMicros() {
    using namespace std::chrono;
    const auto delta = steady_clock::now() - startupTime();
    return (mb_time_t)(uint64_t)duration_cast<microseconds>(delta).count();
}

#endif

// --- Clock source ---

namespace {

// Constant-initialised with the platform clocks, so there is no static
// initialisation order concern and no null check on the hot path.
mb_clock_fn g_millisSource = &platformMillis;
mb_clock_fn g_microsSource = &platformMicros;

} // namespace

void mb_setClockSource(mb_clock_fn millisSource, mb_clock_fn microsSource) {
    g_millisSource = millisSource ? millisSource : &platformMillis;
    g_microsSource = microsSource ? microsSource : &platformMicros;
}

void mb_resetClockSource() {
    g_millisSource = &platformMillis;
    g_microsSource = &platformMicros;
}

mb_time_t mb_millis() { return g_millisSource(); }
mb_time_t mb_micros() { return g_microsSource(); }

// --- Yielding ---

#if defined(MODBUS_PLATFORM_ARDUINO)

void mb_yield() {
    // The core's own. On an ESP8266 this is what services the WiFi stack and
    // the software watchdog; skipping it turns a busy transmit buffer into a
    // reset.
    yield();
}

#else

#include <thread>

void mb_yield() { std::this_thread::yield(); }

#endif
#include "ModbusNoExcept.h"
