// A Modbus RTU device on RS-485.
//
// A SCADA package, an HMI or a PLC reads this board's registers as an ordinary
// Modbus slave. Nothing here polls and nothing blocks: bind your variables,
// call poll(), and get on with the application.

#include <ModBeeModbus.h>

const int      RS485_DE_PIN = 4;      // Driver enable; leave out if none
const uint32_t BAUD         = 19200;
const uint8_t  MY_ADDRESS   = 17;     // How the master addresses this board

ModbusRTU mb;

// --- Your variables ARE the registers ---
//
// Nothing copies between these and the Modbus map. A master reading input
// register 30 gets whatever `level` holds at that instant.
bool     pumpRun   = false;
bool     valveOpen = false;
uint16_t setpoint  = 500;
uint16_t level     = 0;
uint16_t pressure  = 0;
bool     levelHigh = false;

void setup() {
    Serial.begin(115200);

    // 8E1 is the Modbus default. Open the port yourself - the baud rate and
    // pins are board specific - then tell the library the same baud rate,
    // because RTU framing is measured in character times.
    Serial1.begin(BAUD, SERIAL_8E1);

    mb.directionPin(RS485_DE_PIN);    // RS-485 transmit enable
    mb.begin(&Serial1, BAUD);
    mb.slave(MY_ADDRESS);             // ...and answer as unit 17

    // Coils and holding registers can be written by the master.
    // Discrete inputs and input registers cannot - that is the protocol, not a
    // permission we are withholding.
    mb.addCoil(0,  &pumpRun);
    mb.addCoil(1,  &valveOpen);
    mb.addHreg(40, &setpoint);
    mb.addIreg(30, &level);
    mb.addIreg(31, &pressure);
    mb.addIsts(10, &levelHigh);

    // If the master stops talking for five seconds, put the outputs somewhere
    // safe. Worth setting on anything driving real plant: a pump commanded on
    // by a master that has since lost power stays on for ever otherwise.
    mb.failsafeCoil(0, false);        // Pump off
    mb.failsafeCoil(1, false);        // Valve shut
    mb.failsafeAfter(5000);

    // Told when the master writes, once the WHOLE request has been applied -
    // so you never see a half-written block.
    mb.onHregChanged(40, [](ModbusTable, uint16_t, uint16_t value, uint8_t) {
        Serial.print(F("setpoint now "));
        Serial.println(value);
    });

    // Deliberately a different handler. The one above reports the MASTER
    // acting; this one reports US acting because the master stopped. Log only
    // changes and an incident review shows every commanded action and no
    // safety-driven one.
    mb.onFailsafe([](ModbusTable, uint16_t address, uint16_t) {
        Serial.print(F("FAILSAFE applied to address "));
        Serial.println(address);
    });

    // What a commissioning tool sees when it asks what this device is.
    mb.identity("ModBee", "PumpPanel", "1.0.0");

    Serial.print(F("Modbus RTU slave, unit "));
    Serial.println(MY_ADDRESS);
}

void loop() {
    mb.poll();                        // The whole contract

    level     = analogRead(A0);
    pressure  = analogRead(A1);
    levelHigh = level > setpoint;

    digitalWrite(LED_BUILTIN, pumpRun);
}
