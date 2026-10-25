// Reading a Modbus device over the network, and reporting data quality.
//
//     modbee_tcp_client <host> [port] [unit]

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
        std::fprintf(stderr, "usage: %s <host> [port] [unit]\n", argv[0]);
        return 2;
    }
    const char* host = argv[1];
    const uint16_t port = argc > 2 ? (uint16_t)atoi(argv[2]) : 1502;
    const uint8_t unit  = argc > 3 ? (uint8_t)atoi(argv[3]) : 1;

    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    ModbusTCP mb;
    if (!mb.connect(host, port)) {
        std::fprintf(stderr, "could not connect to %s:%u\n", host, port);
        return 1;
    }

    uint16_t block[8] = { 0 };
    float    wide     = 0.0f;
    bool     flags[8] = { false };

    const size_t blockWatch = mb.watchHreg(unit, 0, block, 8, 500);
    const size_t wideWatch  = mb.watchFloat(unit, 100, &wide, 500);
    const size_t flagWatch  = mb.watchCoil(unit, 0, flags, 8, 500);

    std::printf("reading %s:%u unit %u. Ctrl-C to stop.\n", host, port, unit);

    mb_time_t lastReport = 0;
    while (g_running) {
        mb.poll();

        if (mb_elapsed(lastReport, mb_millis()) < 2000) continue;
        lastReport = mb_millis();

        std::printf("registers [%-9s %5ums]:", mb_qualityName(mb.quality(blockWatch)),
                    mb.age(blockWatch));
        for (int i = 0; i < 8; i++) std::printf(" %5u", block[i]);
        std::printf("\n");

        std::printf("float     [%-9s]: %.3f\n", mb_qualityName(mb.quality(wideWatch)), wide);

        std::printf("coils     [%-9s]:", mb_qualityName(mb.quality(flagWatch)));
        for (int i = 0; i < 8; i++) std::printf(" %d", flags[i] ? 1 : 0);
        std::printf("\n");

        if (!mb.allGood()) std::printf("  ** not every reading is trustworthy **\n");
        std::printf("  round trip %ums\n\n", mb.roundTrip(unit));
    }

    mb.end();
    return 0;
}
