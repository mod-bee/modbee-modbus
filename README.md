# ModBee Modbus

**Modbus that never blocks, and tells you the truth about what happened.**

RTU, ASCII and TCP. Client and server. Arduino, Linux, macOS and Windows, from
one C++17 source tree.

```cpp
#include <ModBeeModbus.h>

ModbusRTU mb;
uint16_t  setpoint = 500;

void setup() {
    Serial1.begin(19200, SERIAL_8E1);
    mb.begin(&Serial1, 19200);
    mb.slave(17);                 // answer as unit 17
    mb.addHreg(40, &setpoint);    // this variable IS holding register 40
}

void loop() {
    mb.poll();                    // the whole contract
}
```

One object. One `poll()`. Your variables are the registers.

A note on words: this library says **client** for the side that asks and
**server** for the side that answers. Older documents, and most device manuals,
say *master* and *slave*; they mean the same thing. The one place the old word
survives is `slave(unit)`, the call that makes a serial node answer as a unit id,
because that is the word people search for.

---

## Reading another device uses the same object

```cpp
ModbusRTU mb;

uint16_t level    = 0;
uint16_t setpoint = 500;

void setup() {
    Serial1.begin(19200, SERIAL_8E1);
    mb.begin(&Serial1, 19200);
}

void loop() {
    mb.poll();

    mb.readIreg(17, 30, level);        // queues; `level` updates on a later poll
    mb.writeHreg(17, 40, setpoint);    // queues; only the value you settled on goes out
}
```

Call those every scan if you like. Nothing blocks, nothing stacks up, and
nothing is sent inside the call that made it.

---

## If you have used a Modbus library before

You know the shape. What is different is what happens between your call and the
wire.

| | Most libraries | ModBee |
| --- | --- | --- |
| A read | blocks until the answer or the timeout | queues and returns; the answer lands in your variable |
| The same read twice | two requests on the wire | the second is refused as `Duplicate` |
| A setpoint written every scan | a queue of every value you passed through | **only the value you settled on**, and whoever was waiting on a replaced write is *told* |
| A request that fails | returns an error, sometimes | **every accepted operation reaches exactly one conclusion** |
| A 32-bit float | you work out the word order | `mb.readFloat(17, 100, flow, ModbusOrder::CDAB)`, and `probeOrder()` when the data sheet does not say |
| A value that stops arriving | looks exactly like one that has not changed | `quality()` says `Good`, `Uncertain`, `Bad` or `Unknown` |
| RTU through a USB adapter | splits frames, and nobody can say why | length framing where it can; `framingReason()` explains the choice in a sentence |
| A gateway timing out | a timeout | **0x0A** "no route" or **0x0B** "the device is not there", so a client can tell them apart |

---

## Four objects, and that is the library

```cpp
ModbusRTU    mb;   // RS-485 or RS-232
ModbusASCII  mb;   // the same, for links that cannot hold RTU's timing
ModbusTCP    mb;   // Ethernet or WiFi, client or server
ModbusBridge b;    // TCP in front, a serial segment behind
```

Each one can be client and server at once. A device that answers a SCADA system
*and* reads three sensors of its own is one object, not two.

Nothing underneath is hidden. `mb.client()`, `mb.server()`, `mb.map()`,
`mb.link()`, `mb.poller()` and `mb.points()` stay public and reachable, so the day
you need something the short form does not offer, you reach past it without
rewriting what you already have.

---

## What it does for you

Between your call and the wire, with no configuration at all:

- **Queues the work.** Operations queue as you make them. Nothing blocks and
  nothing is sent inside the call that made it.
- **Discards what is redundant.** Queue the same read twice and the second is
  refused. Write the same value twice and so is that.
- **Sends only the latest.** A control loop moving a setpoint faster than the
  bus can carry it sends the value it settled on. Whoever was waiting on the
  replaced write is told `Superseded`, not left hanging.
- **Concludes everything, exactly once.** Success, refusal, timeout, superseded,
  queue full: every accepted operation reaches one conclusion and one only.
  That is what lets you open a slot per request and never stall it.
- **Times itself.** The response timeout is measured per unit and includes the
  airtime the medium physically needs. A slow VFD and a fast I/O block on one
  bus each get the timeout they need, and a 125-register read on a 9600 baud
  line is not declared dead while it is still arriving.
- **Retries and reports.** With backoff and jitter, so two clients on one bus do
  not stay in step.

There is nothing to tune. `mb.timeout()` and `mb.retries()` exist so you *can*
pin something a segment demonstrably needs; an object opened with no
configuration at all is a fully working one.

---

## Knowing whether to believe a reading

A variable holding `1234` tells you nothing about whether `1234` is still true.
On a bus where one device has quietly failed, its last reading looks identical
to a healthy one.

```cpp
size_t levelWatch;

void setup() {
    // ... begin() as above ...
    levelWatch = mb.watchIreg(17, 30, &level, 1, 500);   // said once: every 500 ms
}

void loop() {
    mb.poll();

    if (mb.quality(levelWatch) == ModbusQuality::Good) control(level);
    else                                               holdSafeState();
}
```

`Unknown` (never read at all: a commissioning problem) and `Bad` (read once and
now stale: a running fault) are deliberately different answers. An alarm list
that conflates them sends somebody to the wrong end of the plant.

---

## Function codes

| Code | Function | Client | Server |
| --- | --- | :---: | :---: |
| 0x01 | Read Coils | yes | yes |
| 0x02 | Read Discrete Inputs | yes | yes |
| 0x03 | Read Holding Registers | yes | yes |
| 0x04 | Read Input Registers | yes | yes |
| 0x05 | Write Single Coil | yes | yes |
| 0x06 | Write Single Register | yes | yes |
| 0x0F | Write Multiple Coils | yes | yes |
| 0x10 | Write Multiple Registers | yes | yes |
| 0x16 | Mask Write Register | yes | yes |
| 0x17 | Read/Write Multiple Registers | yes | yes |
| 0x2B / 0x0E | Read Device Identification | yes | yes |
| 0x07 | Read Exception Status | yes | serial only |
| 0x08 | Diagnostics | yes | serial only |
| 0x0B | Get Comm Event Counter | raw PDU | serial only |
| 0x0C | Get Comm Event Log | raw PDU | serial only |
| 0x11 | Report Server ID | yes | serial only |
| 0x18 | Read FIFO Queue | yes | your handler |
| 0x14 / 0x15 | Read / Write File Record | raw PDU | your handler |

"Serial only" follows the specification: those codes describe a shared serial
bus, so a TCP server refuses them with Illegal Function. "Raw PDU" means the
client sends it with `client().forward()`, which carries any request byte for
byte. "Your handler" means the server answers it through a function handler you
register, because what it reads (a file store, a FIFO) belongs to your
application.

Every exception code is reported too, including the two most libraries skip:
**0x05 Acknowledge** ("accepted, poll, do not retry") and **0x06 Busy** ("retry
later"). They are different instructions, so they come back as different results.

**0x16 Mask Write** is worth calling out: it is the only atomic
read-modify-write Modbus has. Without it, two clients setting different bits of
one register erase each other's.

---

## Platforms

| | |
| --- | --- |
| **Arduino** | 32-bit boards. `library.properties` declares `esp32`, `esp8266`, `samd`, `mbed`, `rp2040`, `stm32` and `renesas_uno`; CI compiles every example for ESP32, ESP8266, SAMD (MKR WiFi 1010) and RP2040 (Pico) on every push. |
| **Linux, macOS, Windows** | CMake 3.16 or later and a C++17 compiler. Built and verified on all three. |

A link costs about 2 KB of RAM before any of your own data, so the 8-bit AVR
boards (Uno R3, Mega, Nano) are out of reach.

On Arduino, serial links take any `Stream` you have opened (`Serial1`, a
`HardwareSerial` on chosen pins). TCP works with any core's `Client` and
`Server` classes, `WiFiClient` and `WiFiServer` for example, through the small
adapters described in the [API reference](docs/api.md#modbustcp).

---

## Install

### Arduino IDE

Download this repository as a ZIP and add it with **Sketch → Include Library →
Add .ZIP Library…**, or clone it into the `libraries` folder of your sketchbook.
Then:

```cpp
#include <ModBeeModbus.h>
```

The Arduino sketches are in `examples/arduino` and appear in the IDE under
**File → Examples → ModBee Modbus**.

### CMake

The library builds as a static library with the target `ModBee::modbus`.
Switch off its own examples when you include it in your project:

```cmake
set(MODBUS_BUILD_EXAMPLES OFF)
add_subdirectory(modbee-modbus)

target_link_libraries(my_app PRIVATE ModBee::modbus)
```

Or fetch it at configure time:

```cmake
include(FetchContent)
set(MODBUS_BUILD_EXAMPLES OFF)
FetchContent_Declare(modbee_modbus
    GIT_REPOSITORY https://github.com/mod-bee/modbee-modbus.git
    GIT_TAG        main)            # pin a release tag in production
FetchContent_MakeAvailable(modbee_modbus)

target_link_libraries(my_app PRIVATE ModBee::modbus)
```

| Option | Default | |
| --- | --- | --- |
| `MODBUS_BUILD_EXAMPLES` | `ON` | build the host examples in `examples/cpp` |
| `MODBUS_SANITIZE` | `OFF` | build with the address and undefined-behaviour sanitizers |
| `MODBUS_SANITIZE_THREAD` | `OFF` | build with the thread sanitizer (cannot be combined with the one above) |

### Try it with no hardware

```bash
cmake -S . -B build && cmake --build build -j
./build/examples/cpp/modbee_tcp_server 1502 &
./build/examples/cpp/modbee_tcp_client 127.0.0.1 1502
```

---

## Documentation

| | |
| --- | --- |
| [Getting started](docs/getting-started.md) | From nothing to a working device, in order |
| [Best practices](docs/best-practices.md) | How to keep a bus healthy, and why it matters |
| [Recipes](docs/recipes.md) | RTU over TCP, bridges, VFDs, control words, and the rest |
| [API](docs/api.md) | Every call, grouped by what you are trying to do |
| [Architecture](docs/ARCHITECTURE.md) | The five layers, and why each boundary is where it is |

## Examples

| | |
| --- | --- |
| [`examples/arduino/RTU_Server`](examples/arduino/RTU_Server) | A server on RS-485, with failsafe |
| [`examples/arduino/RTU_Client`](examples/arduino/RTU_Client) | A client on RS-485 |
| [`examples/arduino/TCP_Server`](examples/arduino/TCP_Server) | An ESP32 serving several clients at once |
| [`examples/arduino/TCP_Client`](examples/arduino/TCP_Client) | An ESP32 reading a device |
| [`examples/arduino/Polling_With_Quality`](examples/arduino/Polling_With_Quality) | Several sensors, and which readings to believe |
| [`examples/cpp/tcp_server`](examples/cpp/tcp_server), [`tcp_client`](examples/cpp/tcp_client) | Host, TCP |
| [`examples/cpp/rtu_server`](examples/cpp/rtu_server), [`rtu_client`](examples/cpp/rtu_client) | Host, serial |
| [`examples/cpp/gateway`](examples/cpp/gateway) | TCP in front, a serial segment behind |

---

## How it is verified

Against the Modbus specification's own worked examples, byte for byte; against a
simulated channel that loses, truncates, corrupts and duplicates frames; on a
virtual serial bus paced at the real line rate, the only honest test of RTU's
silence-based framing; against other implementations, including pymodbus and
libmodbus; and by compiling every Arduino example for the ESP32, ESP8266, SAMD
and RP2040 cores. The suite runs under the address, undefined-behaviour and
thread sanitizers. [Architecture](docs/ARCHITECTURE.md#how-it-is-verified) has a
little more.

---

## Not for

Modbus TCP Security (TLS). Networks where you need more than 247 serial units.
Anything expecting Modbus to be real-time: it is a polled protocol and this
library does not pretend otherwise.

## Contributing

Issues and pull requests are welcome. Please say how you verified any change in
behaviour. CI builds with `-Wall -Wextra -Werror`, so a new warning fails the
pull request.

## Licence

Version 0.1.0. Released under the
[GNU Affero General Public License v3.0](LICENSE), with additional terms set
out at the end of that file. Commercial use needs a separate commercial
licence: contact info@modbee.org.

---

**Ask for the value. Get on with the application. Be told what happened.**
