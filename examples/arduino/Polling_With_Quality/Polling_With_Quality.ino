// Reading four devices, and knowing which readings to believe.
//
// The problem this solves: a variable holding 21.5 tells you nothing about
// whether 21.5 is still true. On a bus where one device has quietly failed,
// its last reading looks identical to a healthy one - it has simply stopped
// changing, which is also what a steady process looks like.

#include <ModBeeModbus.h>

const uint32_t BAUD = 19200;

ModbusRTU mb;

struct Sensor {
    const char* name;
    uint8_t     unit;
    uint16_t    address;
    uint16_t    value;
    size_t      watch;
};

Sensor sensors[] = {
    { "tank level",    17, 30, 0, 0 },
    { "line pressure", 18, 30, 0, 0 },
    { "flow",          19, 30, 0, 0 },
    { "temperature",   20, 30, 0, 0 },
};
const size_t SENSOR_COUNT = sizeof sensors / sizeof sensors[0];

uint32_t lastReport = 0;

void setup() {
    Serial.begin(115200);
    Serial1.begin(BAUD, SERIAL_8E1);
    mb.begin(&Serial1, BAUD);

    // Say what to read and how often. That is the whole setup - no timers, no
    // state machine, no request queue of your own.
    for (size_t i = 0; i < SENSOR_COUNT; i++) {
        sensors[i].watch = mb.watchIreg(sensors[i].unit, sensors[i].address,
                                        &sensors[i].value, 1, 1000);
    }

    // A device that has gone shows up as an EVENT, rather than as a value that
    // stopped moving.
    mb.onEvent([](void*, ModbusEvent event, uint8_t unit, const char* detail) {
        Serial.print(F("["));
        Serial.print(mb_eventName(event));
        Serial.print(F("] unit "));
        Serial.print(unit);
        if (detail) { Serial.print(' '); Serial.print(detail); }
        Serial.println();
    });
}

void loop() {
    mb.poll();

    if (millis() - lastReport < 5000) return;
    lastReport = millis();

    Serial.println(F("---- readings ----"));
    for (size_t i = 0; i < SENSOR_COUNT; i++) {
        const ModbusQuality quality = mb.quality(sensors[i].watch);

        Serial.print(sensors[i].name);
        Serial.print(F(": "));
        Serial.print(sensors[i].value);
        Serial.print(F("  ["));
        Serial.print(mb_qualityName(quality));
        Serial.print(F(", "));
        Serial.print((unsigned)mb.age(sensors[i].watch));
        Serial.println(F("ms old]"));

        // Unknown and Bad are NOT the same thing.
        //
        //   Unknown  never read at all - an address typed wrong, a device
        //            never fitted. A commissioning problem.
        //   Bad      it WAS read and has since gone stale - a cable, a device
        //            that lost power. A running fault.
        //
        // An alarm list that treats them alike sends somebody to the wrong end
        // of the plant.
    }

    if (!mb.allGood()) {
        Serial.println(F("** at least one reading is not trustworthy **"));
    }
}
