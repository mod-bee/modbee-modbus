// A Modbus TCP device, serving several clients at once.
//
//     modbee_tcp_server [port]
//
// Point any Modbus tool at it - modpoll, mbpoll, a SCADA package - and it looks
// like an ordinary device.

#include <ModBeeModbus.h>

#include <cstdio>
#include <cstdlib>
#include <csignal>

static volatile sig_atomic_t g_running = 1;
static void stop(int) { g_running = 0; }

// These variables ARE the registers.
static bool     coils[64]      = { false };
static bool     inputs[64]     = { false };
static uint16_t holding[256]   = { 0 };
static uint16_t readings[256]  = { 0 };

int main(int argc, char** argv) {
    // Line buffered on purpose. Output to a pipe - which is what a service
    // manager, a container log or a redirect gives you - is block buffered by
    // default, so a program killed before its buffer fills appears to have said
    // nothing at all. On a device that is diagnosed from its log, that is the
    // difference between a fault you can see and one you cannot.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    const uint16_t port = argc > 1 ? (uint16_t)atoi(argv[1]) : 1502;

    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    ModbusTCP mb;
    if (!mb.listen(port)) {
        std::fprintf(stderr, "could not listen on port %u\n", port);
        return 1;
    }

    // Blocks, not 256 separate bindings.
    mb.addCoils(0, coils, 64);
    mb.addIsts(0, inputs, 64);
    mb.addHregs(0, holding, 256);
    mb.addIregs(0, readings, 256);

    for (uint16_t i = 0; i < 256; i++) readings[i] = (uint16_t)(0x2000 + i);

    // A register worked out when it is asked for, rather than stored.
    mb.addIregHandler(1000, 1, [](uint16_t) -> uint16_t {
        return (uint16_t)(mb_millis() / 1000);
    });

    // Defaults, shown because a server is where they matter: a connection that
    // has said nothing for a minute is closed, and a newcomer may displace a
    // socket that has never asked for anything - but never a client that has.
    mb.maxClients(8);
    mb.idleTimeout(60000);

    mb.onHregChangedRange(0, 256,
                          [](ModbusTable, uint16_t address, uint16_t value, uint8_t unit) {
        std::printf("unit %u wrote holding %u = %u\n", unit, address, value);
    });

    mb.identity("ModBee", "modbee_tcp_server", "0.1.0");

    mb.onEvent([](void*, ModbusEvent event, uint8_t, const char* detail) {
        std::printf("[%s] %s\n", mb_eventName(event), detail ? detail : "");
    });

    std::printf("Modbus TCP device on port %u. Ctrl-C to stop.\n", port);

    while (g_running) mb.poll();

    std::printf("\n%zu client(s) connected at the end\n", mb.clients());
    mb.end();
    return 0;
}
