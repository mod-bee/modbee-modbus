// Reading another Modbus device over the network, and knowing whether to
// believe the readings.

#include <ModBeeModbus.h>
#include <WiFi.h>

const char* SSID        = "your-network";
const char* PASSWORD    = "your-password";
const char* DEVICE_HOST = "192.168.1.50";
const uint8_t UNIT      = 1;

WiFiClient socket;
ModbusArduinoClientTransport<WiFiClient> transport(socket, DEVICE_HOST, MODBUS_TCP_PORT);
ModbusTCP mb;

uint16_t level   = 0;
float    flow    = 0.0f;
bool     running = false;

size_t levelWatch, flowWatch;

// Defined before use: the Arduino builder generates prototypes for a .ino, so
// the other order compiles there and nowhere else.
void control(uint16_t) {}
void holdSafeState() {}

void setup() {
    Serial.begin(115200);

    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
    Serial.println();

    mb.connect(transport);

    // Say what to read and how often; the values appear on their own.
    levelWatch = mb.watchIreg(UNIT, 30, &level, 1, 500);
    flowWatch  = mb.watchFloat(UNIT, 100, &flow, 500, ModbusOrder::CDAB);
    mb.watchCoil(UNIT, 0, &running, 1, 500);

    // A deadband, because an analogue reading dithers in its last bit for ever
    // and an event on every read is an event on nothing.
    mb.deadband(levelWatch, 3);

    mb.onChanged([](size_t watch, uint16_t address) {
        Serial.print(F("changed: watch "));
        Serial.print((unsigned)watch);
        Serial.print(F(" at address "));
        Serial.println(address);
    });
}

void loop() {
    mb.poll();

    // The distinction that matters. `level` holding 1234 says nothing about
    // whether 1234 is still TRUE - and on a bus where one device has quietly
    // failed, a stale reading looks exactly like a steady process.
    if (mb.quality(levelWatch) == ModbusQuality::Good) {
        control(level);
    } else {
        holdSafeState();
    }
}
