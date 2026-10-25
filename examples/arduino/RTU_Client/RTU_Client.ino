// Reading and writing another Modbus device over RS-485.
//
// NOTHING BLOCKS. Every call queues and returns, and the answer arrives in the
// variable you pointed at on a later poll().

#include <ModBeeModbus.h>

const int      RS485_DE_PIN = 4;
const uint32_t BAUD         = 19200;
const uint8_t  DEVICE       = 17;     // The unit we are talking to

ModbusRTU mb;

// The answers land HERE, so these must still exist when they arrive. Globals
// always do.
uint16_t readings[2] = { 0, 0 };
bool     pumpRunning = false;
float    flow        = 0.0f;

uint32_t lastAsk = 0;

void setup() {
    Serial.begin(115200);
    Serial1.begin(BAUD, SERIAL_8E1);

    mb.directionPin(RS485_DE_PIN);
    mb.begin(&Serial1, BAUD);         // No slave() call: we only ask questions

    Serial.println(F("Modbus RTU master"));
}

void loop() {
    mb.poll();

    if (millis() - lastAsk >= 1000) {
        lastAsk = millis();

        // Two input registers. Queued, not sent - it goes out when the line is
        // free. The handler is optional; leave it off and the values simply
        // appear.
        mb.readIregs(DEVICE, 30, readings, 2, [](ModbusResult result) {
            if (result != ModbusResult::Ok) {
                Serial.print(F("read failed: "));
                Serial.println(mb_resultName(result));
            }
        });

        mb.readCoil(DEVICE, 0, pumpRunning);

        // A 32-bit float across two registers. NOTHING ON THE WIRE says which
        // way round it is, so the ordering is an argument - named the way the
        // device's manual names it. If the manual does not say, see the
        // rtu_client host example, which reads it all four ways at once.
        mb.readFloat(DEVICE, 100, flow, ModbusOrder::CDAB);
    }

    // Write whenever you like, even every scan. A write to an address that is
    // already queued REPLACES the queued value rather than stacking behind it,
    // so the bus carries the value you settled on - not a history of every step
    // you passed through on the way.
    if (Serial.available()) {
        mb.writeHreg(DEVICE, 40, (uint16_t)Serial.parseInt(), [](ModbusResult r) {
            Serial.print(F("setpoint write: "));
            Serial.println(mb_resultName(r));
        });
    }
}
