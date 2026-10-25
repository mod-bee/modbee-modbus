# Recipes

Working answers to the jobs that actually come up. Each one is complete enough
to paste and change the addresses. The host programs use `running` as their stop
flag; [A daemon on a PC](#a-daemon-on-a-pc) shows how to set one up.

- [Modbus RTU over TCP — serial converters](#modbus-rtu-over-tcp--serial-converters)
- [Your own serial-to-Ethernet bridge](#your-own-serial-to-ethernet-bridge)
- [Several devices on one RS-485 bus](#several-devices-on-one-rs-485-bus)
- [An ESP32 serving a SCADA system over WiFi](#an-esp32-serving-a-scada-system-over-wifi)
- [Reading a VFD: floats, word order and scaling](#reading-a-vfd-floats-word-order-and-scaling)
- [Writing a control word without clobbering somebody](#writing-a-control-word-without-clobbering-somebody)
- [Command and read back in one transaction](#command-and-read-back-in-one-transaction)
- [A panel that is a server upstream and a client downstream](#a-panel-that-is-a-server-upstream-and-a-client-downstream)
- [Modbus ASCII over a radio or modem](#modbus-ascii-over-a-radio-or-modem)
- [Telling every device at once](#telling-every-device-at-once)
- [Serial numbers and text](#serial-numbers-and-text)
- [A daemon on a PC](#a-daemon-on-a-pc)

---

## Modbus RTU over TCP — serial converters

This is the one that confuses everybody, so it is worth being precise.

There are **two different things** that both travel over TCP port 502:

| | What is on the wire | Who does it |
| --- | --- | --- |
| **Modbus TCP** | an MBAP header — transaction id, protocol id, length, unit — then the PDU. **No CRC.** | native Ethernet devices, PLCs, SCADA |
| **RTU over TCP** | a complete **RTU frame**, unit id and CRC and all, dropped into a TCP stream | serial-to-Ethernet converters, terminal servers, most cheap "Modbus gateways" |

They are not compatible, they look similar in a packet capture, and a device
speaking one to a client expecting the other simply never answers. If a
converter's manual says "transparent", "serial tunnelling", "raw socket" or
gives you a port that is *not* 502, it is almost certainly RTU over TCP.

**One line either way:**

```cpp
#include <ModBeeModbus.h>

ModbusTCP mb;

int main() {
    mb.rtuOverTcp(true);              // BEFORE connect() or listen()
    mb.connect("192.168.1.200", 4001);   // converters often use 4001, 8899, 26

    uint16_t level = 0;
    mb.watchIreg(17, 30, &level, 1, 1000);

    while (running) mb.poll();
}
```

Everything else is identical — the unit id still selects the device on the
serial side of the converter, because the converter is not reading the frame,
only passing the bytes along.

### Which one is my converter speaking?

If you are not sure, try both. It costs two minutes and the failure is silent
either way:

```cpp
// Opens one way, pings, and polls until the ping has concluded.
static ModbusResult probe(ModbusTCP& mb, bool rtu, const char* host, uint16_t port,
                          uint8_t unit) {
    static ModbusResult outcome;
    outcome = ModbusResult::Queued;

    mb.rtuOverTcp(rtu);                     // before connect()
    if (!mb.connect(host, port)) return ModbusResult::LinkDown;
    mb.ping(unit, [](ModbusResult r) { outcome = r; });
    while (outcome == ModbusResult::Queued) mb.poll();
    mb.end();
    return outcome;
}

printf("rtu over tcp: %s\n", mb_resultName(probe(mb, true,  host, 4001, unit)));
printf("modbus tcp:   %s\n", mb_resultName(probe(mb, false, host, 502,  unit)));
```

`ping()` is the right probe: it proves the path without touching a register, so
it cannot be confused by an address that happens to be wrong. `Timeout` means
the framing (or the unit id) is wrong. Any answer at all, including `Refused`,
means it is right: some native Modbus TCP devices refuse the ping's function
code, Diagnostics, because the specification defines it for serial lines only.

### The thing converters get wrong, and what the library does about it

A serial line delimits frames with **silence**. A TCP stream has no silence in
it at all — the converter hands over whatever arrived when the network felt
like delivering it, sometimes half a frame, sometimes two frames at once.

So for RTU over TCP the library uses **length framing**, not gap framing: read
the function code, work out how long the frame must be, take exactly that many
bytes, check the CRC. It is set automatically when you call `rtuOverTcp(true)`
and you do not have to think about it. If frames still go missing, ask:

```cpp
printf("%s\n", mb.framingReason());
```

### Serving RTU over TCP

The same flag, and a device behind a converter with the socket the other way
round:

```cpp
mb.rtuOverTcp(true);
mb.listen(4001);
mb.addHregs(0, setpoints, 16);
```

---

## Your own serial-to-Ethernet bridge

Rather than buying the converter, be it. Several clients on the network share
one RS-485 segment that can only carry one conversation at a time:

```cpp
#include <ModBeeModbus.h>

ModbusBridge bridge;
uint16_t diagnostics[8] = { 0 };

int main() {
    bridge.serial("/dev/ttyUSB0", 19200);      // the bus behind
    bridge.listen(502);                        // the clients in front
    bridge.route(1, 32);                       // who is out there
    bridge.maxClients(8);

    // Our own registers, at unit 255 (MB_UNIT_SELF), so a client can ask the
    // bridge about itself without a request reaching the field bus.
    bridge.addIregs(MB_UNIT_SELF, 0, diagnostics, 8);
    bridge.identity("ModBee", "Bridge", "1.0.0");

    if (!bridge.begin()) return 1;

    while (running) {
        bridge.poll();
        diagnostics[0] = (uint16_t)bridge.clients();
        diagnostics[1] = (uint16_t)bridge.forwarded();
        diagnostics[2] = (uint16_t)bridge.noPath();
        diagnostics[3] = (uint16_t)bridge.deviceAbsent();
    }
}
```

Wire **`noPath()`** and **`deviceAbsent()`** into separate alarms. They are the
two things a plain timeout can never tell you apart:

- `noPath` (exception 0x0A) — *the bridge* could not help. A unit id that is not
  routed, or no room for another transaction. A configuration or loading
  problem.
- `deviceAbsent` (0x0B) — the route was there and *the device* did not answer.
  A cable, a power supply, a failed unit.

One sends an engineer to the panel. The other sends them to the field.

---

## Several devices on one RS-485 bus

```cpp
ModbusRTU mb;

struct Sensor { const char* name; uint8_t unit; uint16_t value; size_t watch; };

Sensor sensors[] = {
    { "tank level",    17, 0, 0 },
    { "line pressure", 18, 0, 0 },
    { "flow",          19, 0, 0 },
};

void setup() {
    Serial1.begin(19200, SERIAL_8E1);
    mb.directionPin(4);
    mb.begin(&Serial1, 19200);

    for (auto& s : sensors) {
        s.watch = mb.watchIreg(s.unit, 30, &s.value, 1, 1000);
    }
}

void loop() {
    mb.poll();

    // One dead device does not take the bus down with it: a unit that stops
    // answering is polled with backoff rather than at full rate.
    if (!mb.allGood()) raiseCommsAlarm();
}
```

Read [Best practices](best-practices.md) before choosing those intervals. One
second on a three-device bus at 19200 is nothing; one hundred milliseconds on a
twenty-device bus at 9600 will not fit, and the library cannot make airtime that
does not exist.

---

## An ESP32 serving a SCADA system over WiFi

```cpp
#include <ModBeeModbus.h>
#include <WiFi.h>

WiFiServer socketServer(MODBUS_TCP_PORT);
ModbusArduinoServer<WiFiServer, WiFiClient> listener(socketServer, 4);
ModbusTCP mb;

bool     outputs[16]  = { false };
uint16_t setpoints[8] = { 0 };
uint16_t readings[8]  = { 0 };

void setup() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) delay(250);

    if (!listener.ready()) { /* not enough memory for four connections */ }
    mb.listen(listener);
    mb.maxClients(4);            // no more than the listener holds
    mb.idleTimeout(60000);       // a client that vanishes leaves a live socket

    mb.addCoils(0, outputs, 16);
    mb.addHregs(0, setpoints, 8);
    mb.addIregs(0, readings, 8);

    // If EVERY client goes quiet - the network died, not one workstation.
    for (int i = 0; i < 16; i++) mb.failsafeCoil(i, false);
    mb.failsafeAfter(10000);

    mb.identity("ModBee", "IO-16", "1.0.0");
}

void loop() {
    mb.poll();
    for (int i = 0; i < 8; i++) readings[i] = (uint16_t)analogRead(A0 + i);
    for (int i = 0; i < 16; i++) digitalWrite(OUTPUT_PINS[i], outputs[i]);
}
```

---

## Reading a VFD: floats, word order and scaling

Drives are where the word-order question bites hardest, and where manuals are
least helpful.

```cpp
float    outputHz   = 0.0f;
uint16_t currentRaw = 0;

void setup() {
    // The manual says "IEEE 754, word swapped" - that is CDAB.
    mb.watchFloat(DRIVE, 100, &outputHz, 500, ModbusOrder::CDAB);

    // Most drives do not use floats at all. They scale an integer, and the
    // scaling is in the manual: "0.1 A per count" means divide by ten.
    mb.watchIreg(DRIVE, 108, &currentRaw, 1, 500);
}

void loop() {
    mb.poll();
    const float amps = currentRaw / 10.0f;
}
```

If the manual does not say which way round, ask the drive:

```cpp
ModbusOrderProbe probe;     // a global: it is filled in on a later poll()

mb.probeOrder(DRIVE, 100, probe, [](ModbusResult result) {
    if (!mb_succeeded(result)) return;
    for (int i = 0; i < 4; i++) {
        printf("%-5s float %10.3f   int32 %12ld\n",
               mb_orderName((ModbusOrder)i), probe.asFloat32[i], (long)probe.asInt32[i]);
    }
});
```

Run it with the drive at a known speed. One of those four lines will read 50.0
or 30.0 or whatever the keypad says. That is the ordering — write it in your
code with a comment saying how you found it.

---

## Writing a control word without clobbering somebody

A control word packs several commands into one register — run, reset, direction,
jog. Read-modify-write on it is a trap: between your read and your write,
somebody else's write disappears.

```cpp
// WRONG on any bus with more than one client:
uint16_t control;
mb.readHreg(DRIVE, 0, control);          // ... time passes ...
mb.writeHreg(DRIVE, 0, control | 0x0001);

// RIGHT: one transaction, atomic at the device.
mb.maskHreg(DRIVE, 0, 0xFFFE, 0x0001);   // set bit 0, leave the rest alone
mb.maskHreg(DRIVE, 0, 0xFFFE, 0x0000);   // clear bit 0, leave the rest alone
```

The AND mask says which bits to **keep** (a 0 marks a bit you are changing); the
OR mask gives the new value of those bits. To change several bits at once, clear
each of them in the AND mask.

`maskHreg` is function code 0x16, the only atomic read-modify-write Modbus has:

```
result = (current AND andMask) OR (orMask AND NOT andMask)
```

Not every device implements it. If yours refuses with an illegal-function
exception, you are back to read-modify-write — and then you must be the only
thing writing that register, and you should say so in a comment.

---

## Command and read back in one transaction

Function code 0x17 writes and reads in a single exchange, write first, with no
chance for another device to get a turn in between:

```cpp
const uint16_t command[2] = { 0x0001, 0x00FA };   // start, 25.0 Hz
uint16_t status[4] = { 0 };

mb.readWriteHregs(DRIVE,
                  200, status, 4,        // read status 200..203
                  0, command, 2,         // after writing 0..1
                  [](ModbusResult r) { ... });
```

Useful for a command register whose result appears immediately, and for cutting
two round trips down to one on a slow bus. The write half is limited to **121**
registers rather than the usual 123.

---

## A panel that is a server upstream and a client downstream

Two objects, one per serial port: a server facing the SCADA system and a client
reading the panel's own sensors. This is the commonest shape for real equipment
and it needs no special support:

```cpp
ModbusRTU upstream;     // SCADA talks to us
ModbusRTU downstream;   // we talk to our own sensors

uint16_t setpoint = 500;
uint16_t level    = 0;

void setup() {
    Serial1.begin(19200, SERIAL_8E1);
    upstream.begin(&Serial1, 19200);
    upstream.slave(17);
    upstream.addHreg(40, &setpoint);
    upstream.addIreg(30, &level);
    upstream.failsafeHreg(40, 0);
    upstream.failsafeAfter(5000);

    Serial2.begin(9600, SERIAL_8N1);   // whatever the sensors are set to
    downstream.begin(&Serial2, 9600);
    downstream.watchIreg(1, 0, &level, 1, 500);
}

void loop() {
    upstream.poll();
    downstream.poll();
}
```

`level` is bound as our input register 30 *and* is the destination of our own
read from the sensor. The value arrives from downstream and is published
upstream with no copying step.

On one shared segment — a node that answers *and* asks on the same wires — use a
single object and call both `slave()` and the read calls on it. The library
routes replies to the client half and requests to the server half.

---

## Modbus ASCII over a radio or modem

RTU needs character timing a radio link cannot hold. ASCII does not: it frames
on `:` and CRLF and allows a full second between characters.

```cpp
ModbusASCII mb;
uint16_t    registers[32] = { 0 };

void setup() {
    Serial1.begin(9600, SERIAL_8E1);
    mb.begin(&Serial1, 9600);
    mb.slave(17);
    mb.addHregs(0, registers, 32);
}
```

The cost is two characters on the wire per byte of data, so plan on roughly half
the throughput. On a link that was never fast, that is usually the right trade.

---

## Telling every device at once

Unit 0 is a broadcast: every device on the segment performs the write and **none
of them answers**.

```cpp
mb.writeCoil(MB_ADDRESS_BROADCAST, 10, false);   // everybody stop
```

Two rules, both from the protocol rather than from us:

- **Writes only.** There is no such thing as a broadcast read — 247 devices
  answering at once is just a collision.
- **You get no confirmation.** The call concludes when the bytes are transmitted,
  not when anything has acted. The library then holds the bus quiet for the
  turnaround delay, because that is the only way to know the devices have had
  time to finish.

Broadcast is right for "all stop" and for setting a clock. It is wrong for
anything you need to know worked.

---

## Serial numbers and text

Text lives two characters to a register, the first in the high byte:

```cpp
// As a device, exposing our serial number:
char serial[20] = "SN-0001234";
mb.addString(500, serial, sizeof serial);

// As a client, reading somebody's:
char theirs[24] = { 0 };
mb.readText(17, 500, theirs, sizeof theirs, 10);   // 10 registers = 20 characters
```

And what a commissioning tool sees when it asks what the device is:

```cpp
mb.identity("Acme Controls", "PumpPanel-2", "1.4.0");
```

---

## A daemon on a PC

The same code, with a real `main()`:

```cpp
#include <ModBeeModbus.h>
#include <csignal>

static volatile sig_atomic_t running = 1;
static void stop(int) { running = 0; }

int main() {
    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    ModbusRTU mb;
    if (!mb.begin("/dev/ttyUSB0", 19200, ModbusParity::Even)) return 1;

    uint16_t level = 0;
    const size_t watch = mb.watchIreg(17, 30, &level, 1, 1000);

    while (running) {
        mb.poll();
        if (mb.quality(watch) == ModbusQuality::Good) publish(level);
    }

    mb.end();      // concludes anything outstanding as Cancelled
    return 0;
}
```

`poll()` never blocks, so a plain `while` loop spins. On a host, give it
something to do or a short sleep — a millisecond is far below any Modbus timing
and keeps a core free:

```cpp
while (running) {
    mb.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
```

Do not sleep longer than about a tenth of your shortest poll interval. The
library measures how often you call `poll()` and will not promise a timeout it
cannot enforce between two looks at the link.

---

More on choosing intervals, keeping a bus quiet and what to do when it will not
work: **[Best practices](best-practices.md)**.
