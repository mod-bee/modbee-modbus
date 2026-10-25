// A Modbus RTU slave on a serial port.
//
//     modbee_rtu_server <device> [baud] [unit]
//     modbee_rtu_server /dev/ttyUSB0 19200 17
//
// Point mbpoll, modpoll or any RTU master at the same port and it looks like an
// ordinary field device.

#include <ModBeeModbus.h>

#include <cstdio>
#include <cstdlib>
#include <csignal>

static volatile sig_atomic_t g_running = 1;
static void stop(int) { g_running = 0; }

// These variables ARE the registers.
static bool     pumpRun    = false;
static bool     valveOpen  = false;
static bool     highLevel  = false;
static uint16_t setpoint   = 500;
static uint16_t runHours   = 0;
static uint16_t tankLevel  = 0;

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
    // 8E1 is the Modbus default. A device wired for 8N1 will not talk to one
    // wired for 8E1 no matter how right everything else is, and it is the most
    // common reason a bus that "should work" does not.
    if (!mb.begin(device, baud, ModbusParity::Even)) {
        std::fprintf(stderr, "could not open %s\n", device);
        return 1;
    }
    mb.slave(unit);

    mb.addCoil(0, &pumpRun);
    mb.addCoil(1, &valveOpen);
    mb.addIsts(0, &highLevel);
    mb.addHreg(40, &setpoint);
    mb.addHreg(41, &runHours);
    mb.addIreg(30, &tankLevel);

    // If the master stops talking, the plant should not be left in whatever
    // state the last command happened to leave it.
    mb.failsafeCoil(0, false);
    mb.failsafeCoil(1, false);
    mb.failsafeAfter(5000);
    mb.onFailsafe([](ModbusTable, uint16_t address, uint16_t) {
        std::printf("** master silent - coil %u driven safe **\n", address);
    });

    mb.onCoilChanged(0, [](ModbusTable, uint16_t, uint16_t value, uint8_t from) {
        std::printf("unit %u %s the pump\n", from, value ? "started" : "stopped");
    });
    mb.onHregChanged(40, [](ModbusTable, uint16_t, uint16_t value, uint8_t from) {
        std::printf("unit %u set the setpoint to %u\n", from, value);
    });

    mb.identity("ModBee", "modbee_rtu_server", "0.1.0");

    std::printf("unit %u on %s at %u 8E1. framing: %s\n",
                unit, device, baud, mb.framingReason());

    mb_time_t lastTick = 0;
    while (g_running) {
        mb.poll();

        if (mb_elapsed(lastTick, mb_millis()) < 1000) continue;
        lastTick = mb_millis();

        // Ordinary application code. Writing the variable updates the register;
        // there is no separate "publish" step.
        if (pumpRun) {
            tankLevel = (uint16_t)(tankLevel + 5 > 1000 ? 1000 : tankLevel + 5);
            runHours++;
        } else if (tankLevel >= 3) {
            tankLevel -= 3;
        }
        highLevel = tankLevel > 900;
    }

    mb.end();
    return 0;
}
