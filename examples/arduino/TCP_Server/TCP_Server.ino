// A Modbus TCP device on an ESP32 or ESP8266.
//
// Several clients at once - a SCADA package, an HMI and an engineer's laptop
// all polling the same board. One register map serves all of them.

#include <ModBeeModbus.h>
#include <WiFi.h>

const char* SSID     = "your-network";
const char* PASSWORD = "your-password";

WiFiServer socketServer(MODBUS_TCP_PORT);          // port 502
ModbusArduinoServer<WiFiServer, WiFiClient> listener(socketServer, 3);
ModbusTCP mb;

// These variables ARE the registers.
bool     outputs[16]  = { false };
bool     inputs[16]   = { false };
uint16_t setpoints[8] = { 0 };
uint16_t readings[8]  = { 0 };

void setup() {
    Serial.begin(115200);

    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
    Serial.println();
    Serial.print(F("Modbus TCP device on "));
    Serial.println(WiFi.localIP());

    mb.listen(listener);
    mb.maxClients(3);            // A board this size will not hold many

    // Sixty seconds is already the default; it is here to be seen. A client
    // that vanishes - a laptop lid closed, a cable pulled - leaves a socket
    // that never reports an error, and enough of those would hold every slot.
    mb.idleTimeout(60000);

    // Blocks, not sixteen separate bindings: one entry and one lookup, and a
    // contiguous I/O area maps straight across.
    mb.addCoils(0, outputs, 16);
    mb.addIsts(0, inputs, 16);
    mb.addHregs(0, setpoints, 8);
    mb.addIregs(0, readings, 8);

    // A register worked out at the moment it is asked for, rather than stored.
    mb.addIregHandler(100, 1, [](uint16_t) -> uint16_t {
        return (uint16_t)(millis() / 1000);          // Uptime in seconds
    });

    for (int i = 0; i < 16; i++) mb.failsafeCoil(i, false);
    mb.failsafeAfter(10000);     // If EVERY client goes quiet

    mb.identity("ModBee", "IO-16", "1.0.0");
}

void loop() {
    mb.poll();

    for (int i = 0; i < 8; i++) readings[i] = (uint16_t)analogRead(A0 + i);
}
