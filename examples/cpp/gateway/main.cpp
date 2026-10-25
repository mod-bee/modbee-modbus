// A Modbus bridge: TCP in front, a serial segment behind, routed by unit id.
//
//     modbee_gateway <serial-device> [baud] [tcp-port]
//     modbee_gateway /dev/ttyUSB0 19200 1502
//
//      SCADA -+
//        HMI -+- TCP --> [ this ] --RS-485--> unit 1  unit 2  unit 17 ...
//     laptop -+
//
// Several clients on the network side share one serial bus that can only carry
// one conversation at a time. It does NOT decode what it carries: a device
// behind it may support function codes this library does not, so requests for a
// routed unit are forwarded byte for byte.

#include <ModBeeModbus.h>

#include <cstdio>
#include <cstdlib>
#include <csignal>

static volatile sig_atomic_t g_running = 1;
static void stop(int) { g_running = 0; }

// The bridge's own registers, at unit 255 - carved out of the routed range so a
// client can ask the bridge about itself without reaching the field bus.
static uint16_t diagnostics[8] = { 0 };

int main(int argc, char** argv) {
    // Line buffered on purpose. Output to a pipe - which is what a service
    // manager, a container log or a redirect gives you - is block buffered by
    // default, so a program killed before its buffer fills appears to have said
    // nothing at all. On a device that is diagnosed from its log, that is the
    // difference between a fault you can see and one you cannot.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <serial-device> [baud] [tcp-port]\n", argv[0]);
        return 2;
    }
    const char* device = argv[1];
    const uint32_t baud = argc > 2 ? (uint32_t)atoi(argv[2]) : 19200;
    const uint16_t port = argc > 3 ? (uint16_t)atoi(argv[3]) : 1502;

    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    ModbusBridge bridge;
    bridge.serial(device, baud, ModbusParity::Even);   // the bus behind
    bridge.listen(port);                               // the clients in front
    bridge.route(1, 32);                               // who is out there

    // A local binding always wins over a route, which is how one unit is carved
    // out of a routed range.
    bridge.addIregs(MB_UNIT_SELF, 0, diagnostics, 8);

    bridge.identity("ModBee", "modbee_gateway", "0.1.0");
    bridge.onEvent([](void*, ModbusEvent event, uint8_t unit, const char* detail) {
        std::printf("[%s] unit %u %s\n", mb_eventName(event), unit, detail ? detail : "");
    });

    if (!bridge.begin()) {
        std::fprintf(stderr, "could not start: serial %s, port %u\n", device, port);
        return 1;
    }

    std::printf("bridge: TCP %u -> %s at %u baud, units 1-32\n", port, device, baud);
    std::printf("field bus framing: %s\n", bridge.framingReason());

    mb_time_t lastReport = 0;
    while (g_running) {
        bridge.poll();

        diagnostics[0] = (uint16_t)bridge.clients();
        diagnostics[1] = (uint16_t)bridge.forwarded();
        diagnostics[2] = (uint16_t)bridge.servedLocally();
        diagnostics[3] = (uint16_t)bridge.noPath();
        diagnostics[4] = (uint16_t)bridge.deviceAbsent();
        diagnostics[5] = (uint16_t)(mb_millis() / 1000);

        if (mb_elapsed(lastReport, mb_millis()) < 10000) continue;
        lastReport = mb_millis();

        // The last two numbers mean different things, and an alarm list that
        // conflates them sends somebody to the wrong end of the plant.
        //
        //   no path        the BRIDGE could not help: no route, or no room.
        //   device absent  the route was there and the DEVICE was not.
        std::printf("clients %zu | forwarded %u | local %u | "
                    "no path %u | device absent %u\n",
                    bridge.clients(), bridge.forwarded(), bridge.servedLocally(),
                    bridge.noPath(), bridge.deviceAbsent());
    }

    bridge.end();
    return 0;
}
