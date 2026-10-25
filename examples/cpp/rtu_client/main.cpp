// A Modbus RTU master reading a device on a serial port.
//
//     modbee_rtu_client <device> [baud] [unit]
//     modbee_rtu_client /dev/ttyUSB0 19200 17

#include <ModBeeModbus.h>

#include <cstdio>
#include <cstdlib>
#include <csignal>

static volatile sig_atomic_t g_running = 1;
static void stop(int) { g_running = 0; }

int main(int argc, char** argv) {
    // Line buffered on purpose. Output to a pipe - which is what a service
    // manager, a container log or a redirect gives you - is block buffered by
    // default, so a program killed before its buffer fills appears to have said
    // nothing at all. On a device that is diagnosed from its log, that is the
    // difference between a fault you can see and one you cannot.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <device> [baud] [unit]\n", argv[0]);
        return 2;
    }
    const char* device = argv[1];
    const uint32_t baud = argc > 2 ? (uint32_t)atoi(argv[2]) : 19200;
    const uint8_t  unit = argc > 3 ? (uint8_t)atoi(argv[3]) : 17;

    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    ModbusRTU mb;
    if (!mb.begin(device, baud, ModbusParity::Even)) {
        std::fprintf(stderr, "could not open %s\n", device);
        return 1;
    }

    uint16_t level    = 0;
    uint16_t setpoint = 0;
    bool     running  = false;

    // Said once. From here on the values keep themselves up to date.
    const size_t levelWatch = mb.watchIreg(unit, 30, &level, 1, 500);
    const size_t setWatch   = mb.watchHreg(unit, 40, &setpoint, 1, 1000);
    const size_t runWatch   = mb.watchCoil(unit, 0, &running, 1, 500);

    mb.onEvent([](void*, ModbusEvent event, uint8_t unit, const char* detail) {
        std::printf("[%s] unit %u %s\n", mb_eventName(event), unit, detail ? detail : "");
    });

    std::printf("reading unit %u on %s at %u 8E1. Ctrl-C to stop.\n", unit, device, baud);

    mb_time_t lastReport = 0;
    while (g_running) {
        mb.poll();

        if (mb_elapsed(lastReport, mb_millis()) < 2000) continue;
        lastReport = mb_millis();

        std::printf("level %4u [%-9s]  setpoint %4u [%-9s]  pump %s [%-9s]  rtt %ums\n",
                    level, mb_qualityName(mb.quality(levelWatch)),
                    setpoint, mb_qualityName(mb.quality(setWatch)),
                    running ? "on " : "off", mb_qualityName(mb.quality(runWatch)),
                    mb.roundTrip(unit));

        // Simple control. Note there is no request queue to manage: write the
        // same coil twice before the first has gone out and the queued value is
        // replaced, not stacked - so this never builds a backlog even when the
        // bus is slow.
        if (mb.quality(levelWatch) == ModbusQuality::Good) {
            if (level < 200 && !running) mb.writeCoil(unit, 0, true);
            if (level > 900 && running)  mb.writeCoil(unit, 0, false);
        }
    }

    mb.end();
    return 0;
}
