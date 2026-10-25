# Getting started

This page takes you from nothing to a working device, in the order you will
actually do it. No prior Modbus knowledge is assumed; if you have some, skip to
[Your first device](#your-first-device).

- [Modbus in five minutes](#modbus-in-five-minutes)
- [How this library is put together](#how-this-library-is-put-together)
- [Your first device](#your-first-device)
- [Your first client](#your-first-client)
- [Both at once](#both-at-once)
- [Over a network](#over-a-network)
- [Keeping values up to date](#keeping-values-up-to-date)
- [Values bigger than a register](#values-bigger-than-a-register)
- [Being told things](#being-told-things)
- [Safety](#safety)
- [When it does not work](#when-it-does-not-work)
- [Where to go next](#where-to-go-next)

---

## Modbus in five minutes

Modbus is a **question-and-answer** protocol from 1979 that refuses to die,
because it is simple enough that everything implements it.

**One side asks, the other answers.** The asking side is the **client** (older
documents call it the *master*). The answering side is the **server** (the
*slave*). These pages say client and server throughout; the one place the old
word survives is `slave(unit)`, the call that makes a serial node answer as a
unit id. A server never speaks first — it has no way to tell you anything until
you ask.

**Every device has an address**, 1 to 247, called the **unit id**. On a serial
bus this is how one device knows a question is for it. Unit 0 is a broadcast:
everybody acts, nobody answers.

**There are four kinds of data**, and every Modbus device in the world describes
itself in these terms:

| Modbus name | This library | What it is | Who may write it |
| --- | --- | --- | --- |
| Coil | `Coil` | a single bit | anyone |
| Discrete Input | `Ists` | a single bit | you only — a client can only read it |
| Holding Register | `Hreg` | 16 bits | anyone |
| Input Register | `Ireg` | 16 bits | you only — a client can only read it |

Those are not our names. `Ists` is a discrete input and `Ireg` is an input
register, and every Modbus library and every device manual uses them. Inventing
better names would only make your device manual harder to follow.

A rough rule: **things a client commands** are coils and holding registers;
**things a client measures** are discrete inputs and input registers.

**There are three ways to carry it:**

| | What it is | When to use it |
| --- | --- | --- |
| **RTU** | binary, over RS-485 or RS-232 | the usual choice for field wiring |
| **ASCII** | the same, in printable hex | a modem, a radio, a terminal server — anything that cannot hold RTU's character timing |
| **TCP** | the same, over Ethernet or WiFi | anything on a network |

They carry identical content. A device on RTU and the same device on TCP have
the same registers at the same addresses.

**One last thing that bites everybody.** Device manuals number registers from 1,
or from 40001, or from 4x0001. The wire numbers them from **0**. A manual that
says "40001" almost always means holding register **0** on the wire; "40100"
means **99**. This library speaks wire addresses, because that is what the bytes
carry. If your first reading is one register out, this is why.

---

## How this library is put together

Four objects. Pick the one that matches your wiring:

```cpp
ModbusRTU    mb;   // RS-485 or RS-232
ModbusASCII  mb;   // the same, in printable hex
ModbusTCP    mb;   // Ethernet or WiFi
ModbusBridge b;    // TCP in front, a serial segment behind
```

Each one is **both roles at once** if you want it to be. A panel that answers a
SCADA system *and* reads three sensors of its own is one object, not two.

Everything else follows three ideas.

### 1. Your variables are the registers

```cpp
uint16_t setpoint = 500;

mb.addHreg(40, &setpoint);
```

From that line on, holding register 40 **is** `setpoint`. A client reading it
gets whatever the variable holds at that instant. A client writing it changes the
variable. There is no publish step, no copy, no sync call — reading and writing
`setpoint` in your own code is all there is to it.

### 2. Nothing blocks

```cpp
mb.readHreg(17, 100, value);   // returns immediately
```

That call **queues** the read and returns. `value` is filled in on some later
`poll()`, typically a few milliseconds later. Your loop keeps running.

This is the single biggest difference from most Modbus libraries, and it is why
a device with four sensors on a slow bus still updates its display smoothly.

### 3. `poll()` is the whole contract

```cpp
void loop() {
    mb.poll();
}
```

`poll()` moves everything: transmits what is queued, reads what has arrived,
matches replies to requests, retries what timed out, answers anything asked of
you, and applies failsafes. Call it as often as you can. There is nothing else
to drive.

---

## Your first device

Something else reads your registers. This is an Arduino sketch on RS-485. On a
PC the only difference is how the port opens:
`mb.begin("/dev/ttyUSB0", 19200)` (or `"COM3"` on Windows) in place of the
`Serial1` lines, and no `directionPin()`, because a USB RS-485 adapter switches
its own driver.

```cpp
#include <ModBeeModbus.h>

ModbusRTU mb;

// These variables ARE the registers.
bool     pumpRun   = false;     // coil 0     - a client may switch it
bool     highLevel = false;     // discrete 0 - a client may only read it
uint16_t setpoint = 500;        // holding 40 - a client may set it
uint16_t tankLevel = 0;         // input 30   - a client may only read it

void setup() {
    Serial1.begin(19200, SERIAL_8E1);   // 8E1 is the Modbus default
    mb.directionPin(4);                 // RS-485 driver enable, if you have one (before begin)
    mb.begin(&Serial1, 19200);
    mb.slave(17);                       // we answer as unit 17

    mb.addCoil(0, &pumpRun);
    mb.addIsts(0, &highLevel);
    mb.addHreg(40, &setpoint);
    mb.addIreg(30, &tankLevel);
}

void loop() {
    mb.poll();

    // Ordinary application code. Nothing here knows about Modbus.
    tankLevel = readSensor();
    highLevel = tankLevel > 900;
    digitalWrite(PUMP_PIN, pumpRun);
}
```

That is a complete, standards-compliant Modbus device. Point any Modbus tool at
it and it behaves like one you bought.

### 8E1, and why your bus does not work

Modbus's default serial format is **8 data bits, even parity, 1 stop bit**. A
device wired for 8N1 will not talk to one wired for 8E1 no matter how right
everything else is, and it is the most common reason a bus that "should work"
does not. Check it first, before the wiring.

### Blocks, when there are a lot of them

Binding sixteen coils one at a time works, but this is better:

```cpp
bool     outputs[16]  = { false };
uint16_t setpoints[8] = { 0 };

mb.addCoils(0, outputs, 16);      // coils 0..15
mb.addHregs(0, setpoints, 8);     // holding 0..7
```

One entry and one lookup instead of sixteen, and a contiguous I/O area maps
straight across.

### A register worked out on demand

Sometimes there is no variable — the value only exists when somebody asks:

```cpp
mb.addIregHandler(100, 1, [](uint16_t) -> uint16_t {
    return (uint16_t)(millis() / 1000);     // uptime in seconds
});
```

---

## Your first client

Now the other way round: you read somebody else.

```cpp
#include <ModBeeModbus.h>

ModbusRTU mb;

uint16_t level    = 0;
uint16_t setpoint = 500;

void setup() {
    Serial1.begin(19200, SERIAL_8E1);
    mb.begin(&Serial1, 19200);          // no slave() call: we are only asking
}

void loop() {
    mb.poll();

    mb.readIreg(17, 30, level);         // queues; `level` updates later
    mb.writeHreg(17, 40, setpoint);     // queues; only the latest value goes out
}
```

Those two calls are in `loop()` on purpose. Call them every scan — it is safe,
and it is the point:

- The same read, queued twice before the first has gone out, is **refused as a
  duplicate**. One request on the wire, not two.
- The same write, queued twice, **replaces** the queued value rather than
  stacking behind it. A setpoint moving faster than the bus can carry it sends
  the value you settled on, not a history of every step it passed through.

So you never have to write a scheduler, a rate limiter or a "has it changed"
check. That is already the behaviour.

It is safe, but it is not free: the queue drains as fast as the bus allows, so a
read called every scan becomes a read issued continuously, at the maximum rate
the wire can carry. On a shared RS-485 segment that is usually not what you
want. [Best practices](best-practices.md#yes-you-can-call-it-in-the-loop) covers
when to read in the loop, when to watch at an interval, and when to ask only
because something made it relevant.

### Knowing how it went

The value a call returns says whether the request was **accepted**, not whether
it worked. `Queued` means it is on its way. To learn how it ended, either pass a
handler:

```cpp
mb.readHreg(17, 100, value, [](ModbusResult result) {
    if (mb_succeeded(result)) useIt();
    else Serial.println(mb_resultName(result));
});
```

or ask later:

```cpp
ModbusResult outcome;
if (mb.status(17, 100, outcome) == ModbusRequestState::Done) { ... }
```

**Every accepted operation reaches exactly one conclusion** — success, refusal,
timeout, superseded, queue full. Never none, never two. That is what lets you
open a slot per request and know it will never sit there for ever.

---

## Both at once

One object, both roles, on one shared RS-485 segment: a node that answers as
unit 17 and also reads unit 20 on the same wires.

```cpp
void setup() {
    Serial1.begin(19200, SERIAL_8E1);
    mb.begin(&Serial1, 19200);
    mb.slave(17);                      // answer as unit 17 …
    mb.addHreg(40, &setpoint);
}

void loop() {
    mb.poll();
    mb.readIreg(20, 30, sensor);       // … and read unit 20 ourselves
}
```

On a shared RS-485 segment this is a real configuration and the library handles
it: replies that answer our own requests go to the client half, requests
addressed to us go to the server half, and everybody else's traffic is ignored.

---

## Over a network

`ModbusTCP` is the same object with a different way of opening. On a PC:

```cpp
ModbusTCP mb;

// As a server: bind your variables exactly as before, then listen.
mb.addHreg(40, &setpoint);
mb.listen();                          // port 502

// Or as a client: connect, then read and write exactly as before.
mb.connect("192.168.1.50");           // port 502
mb.readHreg(1, 0, value);
```

Three things are different from serial:

- A TCP server answers **every unit id**. The client has already found it by IP
  address, so there is no `slave()` call.
- Several clients can be connected at once, all sharing one register map.
  `maxClients()` sets how many: eight by default, and on Arduino no more than
  the socket adapter holds (two unless you say otherwise).
- A dropped connection comes back on its own: a client reconnects every two
  seconds until it succeeds (`reconnectAfter()` changes that).

On Linux, ports below 1024 need extra privileges, so test on a high port such
as `mb.listen(1502)`. On Arduino you hand the library your board's own socket
classes through a small adapter; see
[the API reference](api.md#modbustcp) and the `TCP_Server` and `TCP_Client`
examples.

---

## Keeping values up to date

Reading in `loop()` works. But a variable holding `1234` tells you nothing about
whether `1234` is **still true**, and on a bus where one device has quietly
failed, its last reading looks exactly like a healthy one that has not changed.

So say what to read and how often, once:

```cpp
size_t levelWatch;

void setup() {
    levelWatch = mb.watchIreg(17, 30, &level, 1, 500);   // every 500 ms
}

void loop() {
    mb.poll();

    if (mb.quality(levelWatch) == ModbusQuality::Good) control(level);
    else                                               holdSafeState();
}
```

Four answers, and the difference between the last two matters:

| | What it means |
| --- | --- |
| `Good` | fresh: read within about two of the intervals you asked for. Act on it. |
| `Uncertain` | late (up to four intervals), or the last read failed once. One missed poll on a busy bus. |
| `Bad` | it **was** read and has since gone stale, or has failed twice running — a cable, a device that lost power. A **running fault**. |
| `Unknown` | never read at all — an address typed wrong, a device never fitted. A **commissioning problem**. |

`Unknown` and `Bad` are different on purpose. An alarm list that conflates them
sends somebody to the wrong end of the plant.

`mb.allGood()` answers "is everything I am watching trustworthy" in one call, and
`mb.age(handle)` gives the milliseconds since a value last arrived.

---

## Values bigger than a register

A register is 16 bits. A flow rate is a 32-bit float, which is two registers —
and **nothing on the Modbus wire says which way round they go**. Four orderings
are in common use, and device manuals name them:

```cpp
mb.readFloat(17, 100, flow, ModbusOrder::CDAB);
mb.writeInt32(17, 200, counter, ModbusOrder::ABCD);
```

If the manual does not say — and many do not — read both ways round and look:

```cpp
ModbusOrderProbe probe;           // a global: it is filled in on a later poll()

mb.probeOrder(17, 100, probe);
// later, once it has completed:
for (int i = 0; i < 4; i++) {
    Serial.print(mb_orderName((ModbusOrder)i));
    Serial.print(' ');
    Serial.println(probe.asFloat32[i]);
}
```

One of those four numbers is a plausible flow rate. That is the device's
ordering. This turns "the analogue reading is nonsense" from an afternoon into a
minute.

Text works the same way — two characters per register, the first in the high
byte:

```cpp
char name[17];                                // 16 characters and the terminator
mb.readText(17, 300, name, sizeof name, 8);   // 8 registers, 16 characters
```

---

## Being told things

```cpp
// A client wrote one of YOUR registers.
// `unit` is the unit id the request was addressed to.
mb.onHregChanged(40, [](ModbusTable, uint16_t address, uint16_t value, uint8_t unit) {
    Serial.print(F("setpoint (unit ")); Serial.print(unit);
    Serial.print(F(") set to ")); Serial.println(value);
});

// A watched value moved by more than its deadband.
mb.deadband(levelWatch, 3);        // ignore dither in the last bit
mb.onChanged([](size_t watch, uint16_t address) { ... });

// Anything that is not one request's own outcome: a link up or down, a corrupt
// frame, a device refusing something.
mb.onEvent([](void*, ModbusEvent event, uint8_t unit, const char* detail) {
    Serial.println(mb_eventName(event));
});
```

A change event fires **once the whole request has been applied**, so you never
see a half-written block, and only when the value actually changed. Setting a
register yourself does not raise one — a change event reports *somebody else*
acting on you.

---

## Safety

A pump commanded on by a client that has since lost power stays on for ever, and
nothing in Modbus says it should not. So say what should happen:

```cpp
mb.failsafeCoil(0, false);     // the pump goes off
mb.failsafeHreg(41, 0);        // the speed reference goes to zero
mb.failsafeAfter(5000);        // if the client is silent for five seconds

mb.onFailsafe([](ModbusTable, uint16_t address, uint16_t) {
    Serial.print(F("** client silent - output driven safe: "));
    Serial.println(address);
});
```

Addresses with nothing set **hold their last value**, because driving an output
is an action and doing it unasked is its own hazard.

`onFailsafe` is deliberately not the change handler. One reports a client acting;
the other reports *you* acting because a client stopped. Log both, or an incident
review will show every commanded action and no safety-driven one.

To refuse a write outright — a panel in local mode, say:

```cpp
mb.onRequest([](uint8_t unit, const ModbusRequest& request) -> uint8_t {
    if (localMode && mb_isWriteFunction(request.function)) {
        return MB_EX_ILLEGAL_FUNCTION;
    }
    return MB_EX_NONE;
});
```

---

## When it does not work

In the order worth trying.

**1. Check the serial format.** 8E1, both ends. See above.

**2. Check the address base.** If everything is one register out, your manual
numbers from 1 and the wire numbers from 0.

**3. Ask what the link has seen.**

```cpp
ModbusLinkStats s = mb.stats();
```

| Rising | Usually means |
| --- | --- |
| `checksumErrors` | wrong baud rate, or wiring noise |
| `noiseBytes` | a floating bus — check your bias resistors |
| `framingErrors` | something on the bus that is not Modbus |
| nothing at all | the device is not there, or not listening on that unit id |

**4. Prove the path without touching a register.**

```cpp
mb.ping(17, [](ModbusResult r) { Serial.println(mb_resultName(r)); });
```

`ping()` sends a Diagnostics echo (0x08). Many TCP devices refuse that code,
because the specification defines it for serial lines only, so over TCP an
answer of `Refused` still proves the path: something at the far end replied.
Only `Timeout` or `LinkDown` means nothing did.

**5. Ask why RTU is framing the way it is.**

```cpp
Serial.println(mb.framingReason());
```

"Why is my RTU splitting frames" is the most common question in every Modbus
library. This answers it in a sentence.

**6. Look at the actual bytes.**

```cpp
mb.onTrace([](void*, bool sent, const uint8_t* bytes, size_t length) {
    Serial.print(sent ? F("TX ") : F("RX "));
    for (size_t i = 0; i < length; i++) { Serial.print(bytes[i], HEX); Serial.print(' '); }
    Serial.println();
});
```

When a device will not talk, what you want is not a decoded summary. It is the
bytes.

---

## Where to go next

| | |
| --- | --- |
| [Best practices](best-practices.md) | **Read this next.** How often to read, how to keep a bus quiet for the commands that matter, and the wiring details that cost people days |
| [Recipes](recipes.md) | Worked answers: RTU over TCP converters, your own bridge, VFDs and word order, control words, ASCII over a radio |
| [API](api.md) | Every call, grouped by what you are trying to do |
| [Architecture](ARCHITECTURE.md) | The five layers, and why each boundary is where it is |
| `examples/` | Working sketches and programs for every role and transport |

And the escape hatch, for the day the short form does not have what you need:

```cpp
mb.client()    // the full client, with every function code
mb.server()    // the full serial server (RTU and ASCII)
mb.map()       // the register map, with block and string bindings
mb.poller()    // the polling engine
mb.points()    // values defined by settings: unit, address, type, scale
mb.link()      // framing, statistics, the raw byte path
```

Nothing is hidden behind the one-object API. It composes those objects; it does
not wrap them, and using both together is normal.
