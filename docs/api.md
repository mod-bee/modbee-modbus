# API

Every call on the one-object API, grouped by what you are trying to do.

If you have not used the library before, read
[Getting started](getting-started.md) first — it explains the ideas these calls
rest on. This page assumes them and is here to be searched.
[Best practices](best-practices.md) covers how to use them well, and
[Recipes](recipes.md) has worked examples.

- [Opening](#opening)
- [Being a device](#being-a-device)
- [Being a client](#being-a-client)
- [Values bigger than a register](#values-bigger-than-a-register)
- [Watching](#watching)
- [Being told things](#being-told-things)
- [Safety, identity and access](#safety-identity-and-access)
- [Diagnostics and tuning](#diagnostics-and-tuning)
- [The bridge](#the-bridge)
- [Reaching past the facade](#reaching-past-the-facade)
- [Types and enumerations](#types-and-enumerations)
- [Build-time settings](#build-time-settings)

Throughout: `unit` is the device's Modbus address (1–247), `address` is a
**wire** address counted from 0, and `done` is an optional completion handler
that may be left off entirely.

**One thread per node.** A node does no locking of its own, so call all of its
methods, `poll()` included, from the same thread or task. Separate nodes are
independent of each other.

---

## Opening

```cpp
#include <ModBeeModbus.h>
```

### ModbusRTU · ModbusASCII

```cpp
bool begin(Stream* serial, uint32_t baud);                          // Arduino
bool begin(const char* device, uint32_t baud,
           ModbusParity parity = ModbusParity::Even);               // host
bool begin(ModbusTransport& transport, uint32_t baudForTiming = 0); // ModbusRTU: any transport
bool begin(ModbusTransport& transport);                             // ModbusASCII: any transport

void directionPin(int pin, bool activeHigh = true, uint32_t holdUs = 2000);   // Arduino
void slave(uint8_t unitId);
```

`begin()` opens the link as a client. Add `slave(unitId)` to answer as a server
as well; a node may be both at once on one segment.

On Arduino, open the `Stream` yourself first (`Serial1.begin(19200, SERIAL_8E1)`)
and pass the same baud rate to `begin()`: RTU framing is defined in character
times. On a host, `device` is `"/dev/ttyUSB0"`, a pseudo-terminal, or `"COM3"`
on Windows; see [ModbusParity](#modbusparity) for the serial formats.

`directionPin()` drives an RS-485 transceiver's DE/RE line; call it before
`begin()`. `holdUs` is how long the driver stays enabled after the last byte —
too short and the last character is truncated on the wire.

`baudForTiming` on the RTU transport form only affects RTU character timing;
pass 0 to take it from the transport.

RTU only:

```cpp
void forceFraming(ModbusFraming framing);   // Gap or Length, if you must
ModbusRtuCodec& codec();                    // the framer itself
```

`framingReason()`, on every node, says which framing is in use and why, in a
sentence; see [Diagnostics and tuning](#diagnostics-and-tuning).

### ModbusTCP

```cpp
bool connect(const char* host, uint16_t port = MODBUS_TCP_PORT);   // as a client
bool connect(ModbusTransport& transport);

bool listen(uint16_t port = MODBUS_TCP_PORT);                      // as a server
bool listen(ModbusListener& listener);

void maxClients(size_t limit);      // how many at once (8)
void idleTimeout(mb_time_t ms);     // close one that has gone silent (60 s; 0 disables)
void displaceUnusedConnections(bool enabled);   // (on)
void reconnectAfter(mb_time_t ms);  // how soon to re-open a dropped link (2 s; 0 disables)
uint32_t reconnects() const;        // times the link has been re-opened

size_t   clients() const;
uint32_t refused() const;           // turned away: every slot held by a client
uint32_t displaced() const;         // sockets evicted that never asked anything

void rtuOverTcp(bool enabled);      // an RTU frame inside a TCP stream
```

On a host, `connect()` and `listen()` take a host name or address and a port;
`listen()` binds to every interface. A TCP server answers **every unit id**,
because a client reaching it over TCP has already addressed it by IP, so there
is no `slave()` call. `reconnectAfter()` and `reconnects()` belong to every node,
not only TCP: a serial port that disappears and comes back is re-opened the same
way.

`rtuOverTcp()` is what most terminal servers produce, sharing port 502 with the
real thing. Call it before `listen()` or `connect()`. It also selects length
framing, which is the only honest choice inside a stream that has no silence in
it — see [Recipes](recipes.md#modbus-rtu-over-tcp--serial-converters).

**When every slot is taken**, a newcomer may displace a socket that has *never
asked for anything* and has sat there for several seconds — a scan, a half-open
connection, a client that died before its first request. A connection that has
completed even one request is never displaced, however long it then goes quiet,
so a client polling once a minute keeps its slot. When every slot is held by
such a client the newcomer is refused, promptly, and `refused()` counts it.

On Arduino, pass an adapter around the platform's own socket classes: any
core's `Server` and `Client` pair, `WiFiServer` and `WiFiClient` for example.
The server adapter's second argument is how many connections it holds (2 if
left out, no upper limit of its own): a slot for each is taken when it is made,
so a connection costs nothing later, and `ready()` is false when the board could
not find the memory — check it, because `listen()` then fails rather than
holding fewer. The node holds at most the smaller of this and `maxClients()`.

```cpp
WiFiServer socketServer(MODBUS_TCP_PORT);
ModbusArduinoServer<WiFiServer, WiFiClient> listener(socketServer, 3);
if (!listener.ready()) { /* not enough memory for three */ }
mb.listen(listener);

WiFiClient socket;
ModbusArduinoClientTransport<WiFiClient> transport(socket, "192.168.1.50", 502);
mb.connect(transport);
```

### Common to all

```cpp
void poll();          // call as often as you can; this is the whole contract
void end();
bool connected() const;
```

---

## Being a device

### Binding variables

```cpp
bool addCoil(uint16_t address, bool* variable);       // a bit anyone may write
bool addIsts(uint16_t address, bool* variable);       // a bit only you may write
bool addHreg(uint16_t address, uint16_t* variable);   // a register anyone may write
bool addIreg(uint16_t address, uint16_t* variable);   // a register only you may write

bool addCoils(uint16_t address, bool* array, uint16_t count);
bool addIsts (uint16_t address, bool* array, uint16_t count);
bool addHregs(uint16_t address, uint16_t* array, uint16_t count);
bool addIregs(uint16_t address, uint16_t* array, uint16_t count);

bool addString(uint16_t address, char* buffer, uint16_t capacity);

bool remove(ModbusTable table, uint16_t address);
```

A bound variable **is** the register. Reading and writing it in your own code is
all there is to it — there is no publish step.

The array forms bind a contiguous block: one entry and one lookup instead of
`count` of them. The variable must outlive the binding, so a local is a bug the
compiler will not catch.

`addString()` maps text across registers, two characters each, the first in the
high byte — the convention almost everything uses.

### Registers with no variable behind them

```cpp
bool addHregHandler(uint16_t address, uint16_t count,
                    ModbusRegisterReader reader, ModbusRegisterWriter writer = nullptr);
bool addIregHandler(uint16_t address, uint16_t count, ModbusRegisterReader reader);
bool addCoilHandler(uint16_t address, uint16_t count,
                    ModbusBitReader reader, ModbusBitWriter writer = nullptr);
```

```cpp
typedef ModbusCallback<uint16_t(uint16_t offset)>               ModbusRegisterReader;
typedef ModbusCallback<bool(uint16_t offset, uint16_t value)>   ModbusRegisterWriter;
typedef ModbusCallback<bool(uint16_t offset)>                   ModbusBitReader;
typedef ModbusCallback<bool(uint16_t offset, bool value)>       ModbusBitWriter;
```

`offset` counts from the binding's own start address, not from zero. A writer
returning `false` refuses the write, which the client sees as an Illegal Data
Value exception.

```cpp
mb.addHregHandler(100, 4,
    [](uint16_t offset) -> uint16_t { return readChannel(offset); },
    [](uint16_t offset, uint16_t value) { return setChannel(offset, value); });
```

Handlers are called **while the request is being served**, so keep them short —
no blocking, no delays.

#### What a handler can be

Every handler the library stores (`ModbusDone`, the register handlers above, the
change, failsafe and request handlers below) is a `ModbusCallback`. It accepts
any of:

- a plain function, or a lambda with no captures — stored as a pointer, no heap;
- a function taking a `void*` context first, with the context beside it —
  `static void onDone(void* context, ModbusResult result)` passed as
  `ModbusDone(onDone, &state)` — also no heap;
- a lambda with captures, or a `std::function` — copied to the heap, as a
  `std::function` would be.

On a small board, prefer the first two. `ModbusEventHandler` and
`ModbusLink::TraceHandler` are plain function pointers with a context argument,
so a lambda passed to `onEvent()` or `onTrace()` must not capture.

### Reading and writing your own registers

```cpp
bool     Coil(uint16_t address) const;   void Coil(uint16_t address, bool value);
bool     Ists(uint16_t address) const;   void Ists(uint16_t address, bool value);
uint16_t Hreg(uint16_t address) const;   void Hreg(uint16_t address, uint16_t value);
uint16_t Ireg(uint16_t address) const;   void Ireg(uint16_t address, uint16_t value);

bool setString(uint16_t address, const char* text);
bool getString(uint16_t address, char* out, uint16_t size) const;
```

Usually unnecessary — you bound your variable, so use it. These are for a
register served by a handler, or one whose binding is out of scope.

Setting a value here raises **no change event**: a change event reports somebody
else acting on you.

### Knowing that a client wrote

```cpp
uint16_t map().writeCount(ModbusTable, uint16_t address) const;     // one address
uint32_t map().bindingWrites(ModbusTable, uint16_t address) const;  // the whole binding
uint32_t map().totalWrites() const;                                 // everything
uint16_t map().failsafeCount(ModbusTable, uint16_t address) const;
```

Three scopes, because they answer three different questions and each of the
others gets one of them wrong:

- **`writeCount()`** is per address, and held for the 64 most recently written.
  Right for watching one command register. A client sweeping a large block ages
  it out, which is exactly when a range is interesting.
- **`bindingWrites()`** is per binding — *has anything in MY range been
  written* — and cannot age out. Right for a block that publishes an array and
  wants an edge when a client touches any of it. Two bindings never see each
  other's traffic.
- **`totalWrites()`** is the whole map, so it is the honest answer to *is
  anything writing to me at all* and nothing narrower.

---

## Being a client

Nothing blocks. Every call queues and returns; the answer lands in the variable
you pointed at, on a later `poll()`. `dest` must stay valid until then.

The value returned says the request was **accepted**, not that it succeeded.

```cpp
ModbusResult readCoil (uint8_t unit, uint16_t address, bool& dest, ModbusDone done = nullptr);
ModbusResult readCoils(uint8_t unit, uint16_t address, bool* dest, uint16_t count, ModbusDone done = nullptr);
ModbusResult readIsts (uint8_t unit, uint16_t address, bool& dest, ModbusDone done = nullptr);
ModbusResult readIsts (uint8_t unit, uint16_t address, bool* dest, uint16_t count, ModbusDone done = nullptr);

ModbusResult readHreg (uint8_t unit, uint16_t address, uint16_t& dest, ModbusDone done = nullptr);
ModbusResult readHregs(uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count, ModbusDone done = nullptr);
ModbusResult readIreg (uint8_t unit, uint16_t address, uint16_t& dest, ModbusDone done = nullptr);
ModbusResult readIregs(uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count, ModbusDone done = nullptr);

ModbusResult writeCoil (uint8_t unit, uint16_t address, bool value, ModbusDone done = nullptr);
ModbusResult writeCoils(uint8_t unit, uint16_t address, const bool* values, uint16_t count, ModbusDone done = nullptr);
ModbusResult writeHreg (uint8_t unit, uint16_t address, uint16_t value, ModbusDone done = nullptr);
ModbusResult writeHregs(uint8_t unit, uint16_t address, const uint16_t* values, uint16_t count, ModbusDone done = nullptr);
```

Writes **copy** their values immediately, so the array you pass may go out of
scope the moment the call returns. Reads do not: their destination is filled in
later.

Quantity limits are the protocol's, and a request past them is refused as
`InvalidArgument` rather than truncated: **2000** bits or **125** registers per
read, **1968** bits or **123** registers per write.

### The two special ones

```cpp
ModbusResult maskHreg(uint8_t unit, uint16_t address, uint16_t andMask, uint16_t orMask,
                      ModbusDone done = nullptr);
```

Sets and clears individual bits of one register **atomically** — the only
read-modify-write Modbus has. Without it, two clients setting different bits of
one register erase each other's.

```
result = (current AND andMask) OR (orMask AND NOT andMask)
```

```cpp
ModbusResult readWriteHregs(uint8_t unit,
                            uint16_t readAddress, uint16_t* dest, uint16_t readCount,
                            uint16_t writeAddress, const uint16_t* values, uint16_t writeCount,
                            ModbusDone done = nullptr);
```

Writes and reads in **one** transaction, write first — for setting a command and
reading back its result without another device getting a turn in between. The
write half is limited to **121** registers, not 123.

### Asking a device about itself

```cpp
ModbusResult readDeviceId(uint8_t unit, uint8_t* dest, uint16_t capacity,
                          ModbusDone done = nullptr);
ModbusResult ping(uint8_t unit, ModbusDone done = nullptr);
```

`ping()` sends data and expects the same back — the standard link test. It proves
the path end to end without touching a single register, which makes it the right
first question when a device will not answer.

### What the queue does for you

| You do this | The library does this |
| --- | --- |
| The same read twice, before the first goes out | returns `Duplicate`. One request on the wire. |
| The same write twice | returns `Duplicate`. |
| Two different values to one address, before transmission | the queued value is **replaced**; the first caller is told `Superseded` |
| Queue a write identical to one already **on the wire** | queues behind it — not merged, because an echo confirming a value that was never sent would be a lie |
| Queue when the queue is full | returns `QueueFull`. Never a silent drop. |
| Nothing comes back | retries with backoff and jitter, then `Timeout` |
| Queue more work inside a completion handler | safe: the operation is removed from the queue before its handler runs |

**Every accepted operation reaches exactly one conclusion.**

---

## Values bigger than a register

Nothing on the wire says which way round a 32-bit value is laid out across two
registers, so the ordering is an argument, named the way device manuals name it.

```cpp
ModbusResult readFloat (uint8_t unit, uint16_t address, float&    dest, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult readInt32 (uint8_t unit, uint16_t address, int32_t&  dest, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult readUInt32(uint8_t unit, uint16_t address, uint32_t& dest, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult readDouble(uint8_t unit, uint16_t address, double&   dest, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult readInt64 (uint8_t unit, uint16_t address, int64_t&  dest, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);

ModbusResult writeFloat (uint8_t unit, uint16_t address, float    value, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult writeInt32 (uint8_t unit, uint16_t address, int32_t  value, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult writeUInt32(uint8_t unit, uint16_t address, uint32_t value, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult writeDouble(uint8_t unit, uint16_t address, double   value, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);
ModbusResult writeInt64 (uint8_t unit, uint16_t address, int64_t  value, ModbusOrder order = ModbusOrder::ABCD, ModbusDone done = nullptr);

ModbusResult probeOrder(uint8_t unit, uint16_t address, ModbusOrderProbe& dest,
                        ModbusDone done = nullptr);

ModbusResult readText (uint8_t unit, uint16_t address, char* dest, uint16_t capacity,
                       uint16_t registers, ModbusDone done = nullptr);
ModbusResult writeText(uint8_t unit, uint16_t address, const char* text,
                       uint16_t registers, ModbusDone done = nullptr);
```

### Banks of them

A meter publishing eight float32 totals is one request, not eight. These live on
`client()` rather than on the facade, because the facade keeps the short form
short:

```cpp
mb.client().readFloat32Array (unit, address, float*    dest,  count, order, done, stride, table);
mb.client().readInt32Array   (unit, address, int32_t*  dest,  count, order, done, stride, table);
mb.client().readUInt32Array  (unit, address, uint32_t* dest,  count, order, done, stride, table);
mb.client().readFloat64Array (unit, address, double*   dest,  count, order, done, stride, table);
mb.client().readInt64Array   (unit, address, int64_t*  dest,  count, order, done, stride, table);

mb.client().writeFloat32Array (unit, address, const float*    values, count, order, done, stride);
mb.client().writeInt32Array   (unit, address, const int32_t*  values, count, order, done, stride);
mb.client().writeUInt32Array  (unit, address, const uint32_t* values, count, order, done, stride);
mb.client().writeFloat64Array (unit, address, const double*   values, count, order, done, stride);
mb.client().writeInt64Array   (unit, address, const int64_t*  values, count, order, done, stride);

// For a caller that works its element type out at run time rather than naming
// it - a binding behind a generic pin does exactly that.
mb.client().readWideArray (unit, address, void* dest, ModbusWideType, order, count, stride, done, table);
mb.client().writeWideArray(unit, address, const void* values, ModbusWideType, order, count, stride, done);
```

**One request rather than one per value** is the whole point: eight setpoints
are eight round trips or one, and on a serial segment that is the difference
between a scan and a second. It also means a set of values that only mean
anything together cannot half-arrive.

`stride` is the gap in the **caller's** layout, in bytes; 0 means the values are
packed. It exists because an array element can carry more than its value — a
PLC's do — and without it a bank lands in the wrong slots and reads as plausible
nonsense rather than as an error.

`table` is `HoldingRegister` by default and `InputRegister` where the device
publishes there, which is at least as common. Every scalar read above takes it
too, after `done`. A bit list is refused: it cannot carry a 32-bit value.

A write past what one request may carry — 123 registers — is refused before
anything reaches the wire, rather than found out from a device that may not
bother to raise an exception.

A wrong word order is **not an error**. It succeeds and gives a different number,
which is why `probeOrder()` exists: it reads two registers and hands back every
interpretation of them at once.

```cpp
struct ModbusOrderProbe {
    float    asFloat32[4];    // indexed by ModbusOrder
    int32_t  asInt32[4];
    uint32_t asUInt32[4];
};
```

---

## Watching

```cpp
size_t watchHreg (uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count, mb_time_t everyMs);
size_t watchIreg (uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count, mb_time_t everyMs);
size_t watchCoil (uint8_t unit, uint16_t address, bool* dest, uint16_t count, mb_time_t everyMs);
size_t watchIsts (uint8_t unit, uint16_t address, bool* dest, uint16_t count, mb_time_t everyMs);
size_t watchFloat(uint8_t unit, uint16_t address, float* dest, mb_time_t everyMs,
                  ModbusOrder order = ModbusOrder::ABCD);

ModbusQuality quality(size_t watchHandle) const;
mb_time_t     age(size_t watchHandle) const;      // ms since it last arrived
bool          allGood() const;

void deadband(size_t watchHandle, uint16_t amount);
void stopWatching(size_t watchHandle);
```

Each returns a handle. The point is not saving you a timer — it is `quality()`:

| | |
| --- | --- |
| `Good` | read successfully within the last two intervals |
| `Uncertain` | between two and four intervals old, or the last read failed once |
| `Bad` | more than four intervals old, or the last two reads failed |
| `Unknown` | never successfully read at all |

A watch handle counts from 0, so 0 is a valid handle. A refused watch (no memory,
a bad argument) returns `SIZE_MAX`.

A deadband is how much a value must move before `onChanged()` fires. Without
one, an analogue reading dithering in its last bit is an event on every read.

A device that stops answering is polled with backoff rather than at full rate, so
one dead unit does not consume the segment: the interval doubles with each
consecutive failure, up to 16 times the one you asked for, and snaps back on the
first answer.

### Reaching the poller directly

The facade covers the common shapes. `poller()` has the rest, and a binding that
works its element type out at run time needs all of it:

```cpp
mb.poller().addRegisters(unit, address, uint16_t* dest, count, intervalMs, table, stride);
mb.poller().addBits     (unit, address, bool*     dest, count, intervalMs, table, stride);
mb.poller().addFloat32  (unit, address, float*    dest, intervalMs, order);
mb.poller().addInt32    (unit, address, int32_t*  dest, intervalMs, order);
mb.poller().addWide     (unit, address, void*     dest, ModbusWideType, count,
                         intervalMs, order, table, stride);

mb.poller().remove(index);        // stop polling it
mb.poller().resume(index);        // start again, in place, and read at once
mb.poller().isActive(index);
mb.poller().setInterval(index, ms);
mb.poller().lastResult(index);
mb.poller().consecutiveFailures(index);

mb.poller().refreshAll();             // everything due now: after a reconnect, say
mb.poller().setFailureBackoff(16);    // how far a failing entry may stretch; 1 = never
mb.poller().onPoll(handler);          // every completion, failures included
```

`onPoll()` takes a `ModbusPollHandler`, `void(size_t index, ModbusResult result)`,
for a caller that wants to see each failure as it happens rather than only the
quality it leads to.

`stride` is the gap in the caller's layout, in bytes, exactly as for the wide
arrays above — a polled block that assumed its elements were adjacent would fill
every other slot with a plausible number and leave the rest untouched, which
reads as a process fault rather than as a binding fault.

`remove()` and `resume()` are a pair, and the reason `resume()` exists is that
stopping and starting a watch must not add a table entry each time. A program
that watches a device only in one machine state would otherwise grow its poll
table for the life of the program. `resume()` also makes the entry due
immediately: a watch you have just started should read now.

```cpp
mb.poller().retarget(index, unit, address, table, order);
```

Points an entry somewhere else, in place: another unit, address, table or word
order, into the same variable. Nothing learned about the old address carries
over - quality goes back to `Unknown` - and a reply still on its way for the old
address is discarded when it concludes. For a watch whose address comes from
settings.

### Points defined by settings

```cpp
size_t first = mb.points().add(16);            // a run of points, owned by the node
ModbusPoint* p = mb.points().at(first);

ModbusPointConfig c;
c.unit = 3; c.fc = ModbusPointFc::ReadHolding; c.address = 4100;
c.type = ModbusPointType::Float32; c.order = ModbusOrder::CDAB;
c.scale = 1.0f; c.everyMs = 2000; c.staleMs = 0;   // 0: 3 x every
c.timeoutMs = 3000;                               // this unit's timeout (a radio hop)
p->configure(c);                                  // every scan: cheap when unchanged

p->value(); p->ok(mb_millis()); p->quality(mb_millis()); p->age(mb_millis());
p->setWriteValue(v);                              // write points (FC05 / FC06 / FC16)
```

A `ModbusPoint` is one value described entirely by data - unit, function code,
address, type (`Bit`, `Int16`, `UInt16`, `Int32`, `UInt32`, `Float32`,
`RegisterBit`), word order, bit, scale, offset, interval, staleness, timeout -
read and decoded, or encoded and written. `configure()` with a different
target is a different point: quality `Unknown`, the old value dropped, an
answer in flight discarded. The node services every point from `poll()`;
the set owns them, so a point outlives every operation it queued.

Quality: `Unknown` before the first answer, `Good` within two intervals,
`Uncertain` after one failure or once late, `Bad` after two failures or
`staleMs` without a good answer — and the value is held, never zeroed. A
failing point backs off, doubling its interval up to eight times, but is never
left longer than a minute between attempts unless its own interval is longer.
`configure()` returns `false` for a combination that cannot work: a 32-bit type
on FC06, a register bit on a write, a bit above 15.
`mb_pointDecode()` / `mb_pointEncode()` / `mb_pointValid()` are the codec on
their own.

---

## Being told things

```cpp
void onCoilChanged(uint16_t address, ModbusChangeHandler handler);
void onHregChanged(uint16_t address, ModbusChangeHandler handler);
void onHregChangedRange(uint16_t address, uint16_t count, ModbusChangeHandler handler);

void onChanged(ModbusPollChangeHandler handler);
void onEvent(ModbusEventHandler handler, void* context = nullptr);
```

```cpp
typedef ModbusCallback<void(ModbusTable table, uint16_t address,
                            uint16_t value, uint8_t unit)>      ModbusChangeHandler;
typedef ModbusCallback<void(size_t watchHandle, uint16_t address)> ModbusPollChangeHandler;
typedef void (*ModbusEventHandler)(void* context, ModbusEvent event,
                                   uint8_t unit, const char* detail);
```

In a change handler, `unit` is the unit id the request was addressed to.
`onChanged()` reports the first address of the watch that changed.

A change handler fires once the **whole request** has been applied, so you never
see a half-written block, and only when the value actually changed.

`onEvent()` reports what is not one request's own outcome: `LinkUp`, `LinkDown`,
`ChecksumError`, `FramingError`, `NoiseDiscarded`, `RequestTimeout`,
`ExceptionReceived`, `UnexpectedResponse`, `RequestRefused`, `FailsafeApplied`,
`Reconnecting`. `mb_eventName()` turns one into text.

---

## Safety, identity and access

```cpp
void failsafeCoil(uint16_t address, bool safeValue);
void failsafeHreg(uint16_t address, uint16_t safeValue);
void failsafeAfter(mb_time_t ms);          // 0 - the default - never applies them
void onFailsafe(ModbusFailsafeHandler handler);

void identity(const char* vendor, const char* product, const char* revision);
void onRequest(ModbusRequestFilter filter);
```

Addresses with no failsafe set **hold their last value**, because driving an
output is an action and doing it unasked is its own hazard.

`onFailsafe` is deliberately not the change handler: that one reports a client
acting, this one reports **you** acting because a client stopped. On a TCP
server the silence that counts is *every* client's, not one.

```cpp
typedef ModbusCallback<void(ModbusTable table, uint16_t address, uint16_t safeValue)>
        ModbusFailsafeHandler;
typedef ModbusCallback<uint8_t(uint8_t unit, const ModbusRequest& request)>
        ModbusRequestFilter;
```

`identity()` sets the vendor, product and revision strings a client reads with
Read Device Identification (0x2B / 0x0E). It may be called before or after
`begin()`.

Return an `MB_EX_*` code from the filter to refuse a request, or `MB_EX_NONE` to
let it through. `request.function`, `request.address` and `request.quantity`
say what was asked; `mb_isWriteFunction(request.function)` picks out the
writes. This is where a panel in local mode refuses writes, and the only place
it can be said **before** the write has already happened.

---

## Diagnostics and tuning

```cpp
ModbusRequestState status(uint8_t unit, uint16_t address, ModbusResult& outcome) const;
size_t    pending() const;
bool      idle() const;
void      cancelAll();                 // concludes everything as Cancelled

mb_time_t roundTrip(uint8_t unit) const;
mb_time_t silenceFor(uint8_t unit) const;   // ms since it last answered, or MB_NEVER_HEARD
const char* framingReason() const;          // how this link finds frames, and why
const char* lastError() const;              // why begin(), connect() or listen() failed
size_t      clients() const;                // connections held, for a TCP server
ModbusLinkStats stats() const;
void      resetStats();
void      onTrace(ModbusLink::TraceHandler handler, void* context = nullptr);

void timeout(mb_time_t ms);            // leave alone: it is measured per device
void retries(uint8_t count);           // attempts after the first
void queueSize(size_t requests, size_t payloadBytes = 0);
                                       // requests that may wait, and the bytes their
                                       // values share; 0 = 16 / 1 KB board, 64 / 16 KB host.
                                       // Taken when the link opens; too much for the
                                       // board fails the open: "out of memory for the
                                       // request queue" on lastError()

mb.client().setUnitTimeout(unit, ms);  // one unit's own timeout; 0 = measured
```

`setUnitTimeout()` pins one unit's response timeout whatever is measured or
configured: a device behind a radio mesh answers in seconds, and a measured
timeout would never get its first sample (the cold-start figure is about
200 ms) or would collapse after one quick reply. The airtime of each exchange is
still added on top. Points set it for you from their `timeoutMs`.

```cpp
struct ModbusLinkStats {
    uint32_t framesReceived, framesSent, sendFailures;
    uint32_t checksumErrors;   // noise, or a baud rate mismatch
    uint32_t framingErrors;    // structurally impossible frames
    uint32_t overruns;         // longer than any valid ADU
    uint32_t noiseBytes;       // discarded while hunting for a frame start
    uint32_t protocolErrors;   // MBAP protocol id wrong, and the like
    uint32_t echoesSuppressed; // our own transmission heard back (normal on some wiring)
};

typedef void (*TraceHandler)(void* context, bool transmitted,
                             const uint8_t* bytes, size_t length);
```

The **difference** between those counters is what tells you whether the device is
absent, the wiring is wrong or the baud rate is wrong — none of which looks
different from the application's point of view.

`silenceFor()` answers "is that device there" from traffic the application was
making anyway, so asking costs nothing. **`MB_NEVER_HEARD` is not a long
silence** — it means the unit has never answered at all, which is a wrong
address or a device never fitted, where a growing silence is a cable or a unit
that has died. Two different faults, two different ends of the plant.

`lastError()` is the operating system's own words — *"No such file or
directory"*, *"Address already in use"*, *"Permission denied"*. A `bool` says
the port did not open; a code cannot say which of forty things went wrong at a
socket, and *"Permission denied"* on a serial device is a user not in the
`dialout` group, which no status code will ever tell you. Empty when nothing has
failed, and empty on Arduino, where opening a `Stream` the sketch already opened
cannot fail this way.

`lastError()`, `framingReason()` and `connected()` are **virtual on
`ModbusAPI`**, because a caller holding the base class cannot know which kind of
link it has — and those are exactly the questions such a caller needs to ask.
`clients()` works through the base class too, and is zero for anything that is
not a TCP server.

`timeout()` pins one figure for everything. Left alone, the timeout is measured
per device and includes the airtime the medium physically needs, so a slow drive
and a fast I/O block on one bus each get the figure they need.

Modbus TCP allows several requests in flight at once, matched by transaction id.
The library sends one at a time unless told otherwise, because many devices
cannot cope with more. Once a TCP client is connected,
`mb.client().config().maxInFlight = 4;` raises it for a device you have tested.
On RTU and ASCII leave it at one: serial Modbus has no transaction id, so two
answers in flight could not be told apart.

---

## The bridge

TCP in front, a serial segment behind, routed by unit id.

```cpp
ModbusBridge bridge;
bridge.serial("/dev/ttyUSB0", 19200);   // the bus behind
bridge.listen(502);                     // the clients in front
bridge.route(1, 32);                    // who is out there
bridge.begin();

while (running) bridge.poll();
```

```cpp
void serial(const char* device, uint32_t baud, ModbusParity parity = ModbusParity::Even);  // host
void serial(Stream* stream, uint32_t baud);                                      // Arduino
void serial(ModbusTransport& transport, uint32_t baudForTiming = 0);
void directionPin(int pin, bool activeHigh = true, uint32_t holdUs = 2000);      // Arduino

void listen(uint16_t port = MODBUS_TCP_PORT);                                    // host
void listen(ModbusListener& listener);

bool begin();     // opens both ends; false if either will not open
void poll();
void end();

void route(uint8_t unit);
void route(uint8_t first, uint8_t last);
void routeEverythingElse(bool enabled = true);

bool addCoils(uint8_t unit, uint16_t address, bool* array, uint16_t count);
bool addIsts (uint8_t unit, uint16_t address, bool* array, uint16_t count);
bool addHregs(uint8_t unit, uint16_t address, uint16_t* array, uint16_t count);
bool addIregs(uint8_t unit, uint16_t address, uint16_t* array, uint16_t count);

void identity(const char* vendor, const char* product, const char* revision);
void onEvent(ModbusEventHandler handler, void* context = nullptr);
void maxClients(size_t limit);
void maxOutstanding(size_t limit);

size_t   clients() const;
uint32_t forwarded() const;        // requests sent to the field bus
uint32_t servedLocally() const;    // answered by the bridge itself
uint32_t noPath() const;           // refused 0x0A
uint32_t deviceAbsent() const;     // refused 0x0B

const char* framingReason() const;          // the field bus's RTU framing, and why
void forceFraming(ModbusFraming framing);

ModbusGateway& gateway();                   // the engine underneath
ModbusDataMap& map(uint8_t unit);           // one unit's local register map
```

The bridge does **not** decode what it carries: a device behind it may support
function codes this library does not, so requests for a routed unit are forwarded
byte for byte.

`maxClients()` (8 by default) is how many TCP connections it holds;
`maxOutstanding()` (8 by default) is how many of their requests may wait for the
one serial bus at once. `routeEverythingElse()` forwards any unit id that is not
routed and not served locally, for a bus whose devices you do not know in
advance; it is off by default, because a mistyped unit id then becomes a timeout
instead of an immediate 0x0A.

Its own registers live at a unit id of their own — `MB_UNIT_SELF` (255) by
convention — so a client can ask the bridge about itself without a request
reaching the field bus. A local binding always wins over a route.

Two failures a direct connection cannot report:

| | |
| --- | --- |
| **0x0A** Gateway Path Unavailable | no route, or no room. A problem with the **bridge**. |
| **0x0B** Gateway Target Failed | the route was there and the device did not answer. A problem with the **device**. |

Worth wiring into an alarm separately. A plain timeout cannot tell an engineer
which end of the plant to walk to.

---

## Reaching past the facade

```cpp
ModbusClient&   client();
ModbusServer&   server();    // the serial server (RTU and ASCII)
ModbusPoller&   poller();
ModbusPointSet& points();
ModbusLink&     link();
ModbusDataMap&  map();       // the map the node serves, serial or TCP
```

These are the real objects. The facade composes them; it does not wrap them, and
using both together is normal. Reach for them the day you need something the
short form does not offer — diagnostics sub-functions, raw requests, FIFO
queues, per-unit maps — without rewriting what you already have.

`server()`, `map()` and `points()` create what they return the first time they
are called, so a node that only asks never pays for a server. On a TCP server,
`map()` is the map every connection shares; `server()` is the serial server and
is not used by `listen()`.

Two things only reachable this way:

```cpp
// A serial node that answers more than one unit id, each with its own registers.
mb.server().mapFor(18).bindBlock(ModbusTable::HoldingRegister, 0, otherRegisters, 10);

// A function code the library does not serve generically: 0x14 / 0x15 file
// records, 0x18 FIFO queues, or a vendor code.
mb.server().engine().setFunctionHandler(0x41, handler);   // a ModbusFunctionHandler
```

The default map (`map()`) serves the unit id given to `slave()`; each
`mapFor(unit)` adds another unit id the node answers, with registers of its own. A function handler writes the response body after the
function code into `out` and returns its length, or returns 0 with `exception`
set to refuse; see `ModbusServerEngine.h`.

---

## Types and enumerations

### ModbusResult

What happened to an operation.

| | |
| --- | --- |
| `Queued` | accepted; the outcome arrives on a later `poll()` |
| `Ok` | it completed. This is what success looks like |
| `NotInitialised` | `begin()` has not been called |
| `InvalidArgument` | null pointer, zero count, or a quantity Modbus disallows |
| `NotMapped` | the address cannot be served: nothing bound locally, or a range no device could hold (one that would run past 65535) |
| `Duplicate` | an identical operation is already pending |
| `QueueFull` | the queue is at capacity |
| `UnknownUnit` | the destination is not reachable |
| `Superseded` | a newer write to the same address replaced it before it went out |
| `Cancelled` | discarded by `cancelAll()` or a restart |
| `Timeout` | no response after every retry |
| `Refused` | the server raised an exception |
| `Acknowledged` | exception 0x05: accepted, and it will take a while. **Poll, do not retry.** |
| `Busy` | exception 0x06: retry later |
| `GatewayPathUnavailable` | exception 0x0A |
| `GatewayTargetFailed` | exception 0x0B |
| `LinkDown` | the transport is not usable |
| `ChecksumError` | a reply arrived and was corrupt |
| `ProtocolError` | a reply arrived and made no sense |

```cpp
bool mb_accepted(ModbusResult);        // queued or completed
bool mb_succeeded(ModbusResult);       // completed and did what was asked
bool mb_worthRetrying(ModbusResult);   // notably NOT Acknowledged
const char* mb_resultName(ModbusResult);
```

`Acknowledged` and `Busy` are both "not yet", and the difference is the whole of
what you do next: one means the server took the job and is still doing it —
retrying there sends a second copy of a command already running.

### ModbusRequestState

Returned by `status()`, for a caller that would rather ask than be told.

`Idle` · `Pending` · `Done` · `Failed`

### ModbusQuality

`Unknown` · `Good` · `Uncertain` · `Bad` — see [Watching](#watching).
`mb_qualityName()` turns one into text.

### ModbusOrder

| | |
| --- | --- |
| `ABCD` | big-endian throughout, high word first. What the specification implies. |
| `CDAB` | low word first, big-endian within each word. Extremely common. |
| `BADC` | high word first, byte-swapped within each word. Rare. |
| `DCBA` | fully reversed — a little-endian CPU's memory, copied straight out. |

### ModbusTable

`Coil` · `DiscreteInput` · `HoldingRegister` · `InputRegister`

### ModbusParity

| | |
| --- | --- |
| `Even` | 8E1, the Modbus default |
| `Odd` | 8O1 |
| `None` | 8N2: the specification asks for two stop bits when there is no parity |
| `None8N1` | 8N1: off-specification, but common enough to support |

### ModbusFraming

`Auto` (the default: chosen from the line rate, how often `poll()` runs, and
whether the node only answers or also asks) ·
`Gap` (the specification's t3.5 silence) · `Length` (worked out from the
function code). `mb_framingName()` turns one into text.

### Function codes covered

| Code | Function | Client | Server |
| --- | --- | --- | --- |
| 0x01 | Read Coils | `readCoil()`, `readCoils()` | built in |
| 0x02 | Read Discrete Inputs | `readIsts()` | built in |
| 0x03 | Read Holding Registers | `readHreg()`, `readHregs()`, the wide reads | built in |
| 0x04 | Read Input Registers | `readIreg()`, `readIregs()` | built in |
| 0x05 | Write Single Coil | `writeCoil()` | built in |
| 0x06 | Write Single Register | `writeHreg()` | built in |
| 0x0F | Write Multiple Coils | `writeCoils()` | built in |
| 0x10 | Write Multiple Registers | `writeHregs()`, the wide writes | built in |
| 0x16 | Mask Write Register | `maskHreg()` | built in |
| 0x17 | Read/Write Multiple Registers | `readWriteHregs()` | built in |
| 0x2B / 0x0E | Read Device Identification | `readDeviceId()` | built in, from `identity()` |
| 0x07 | Read Exception Status | `client().readExceptionStatus()` | built in, serial only |
| 0x08 | Diagnostics | `ping()`, `client().diagnostics()` | built in, serial only |
| 0x0B | Get Comm Event Counter | `client().forward()` | built in, serial only |
| 0x0C | Get Comm Event Log | `client().forward()` | built in, serial only |
| 0x11 | Report Server ID | `client().reportServerId()` | built in, serial only |
| 0x18 | Read FIFO Queue | `client().readFifoQueue()` | a function handler |
| 0x14 / 0x15 | Read / Write File Record | `client().forward()` | a function handler |

The facade covers the ones an application uses every day. The rest are one step
past it, on `client()`:

```cpp
mb.client().readExceptionStatus(unit, uint8_t& dest, done);
mb.client().diagnostics(unit, subFunction, const uint8_t* data, dataLen,
                        uint8_t* dest, destCapacity, done);
mb.client().reportServerId(unit, uint8_t* dest, destCapacity, done);
mb.client().readDeviceIdentification(unit, readDeviceIdCode, objectId,
                                     uint8_t* dest, destCapacity, done);
mb.client().readFifoQueue(unit, pointerAddress, uint16_t* dest, maxValues, done);
mb.client().forward(unit, const uint8_t* pdu, pduLen, onResponse);   // any PDU at all
```

`forward()` sends a request PDU byte for byte and hands the raw response to a
`ModbusPassthroughHandler`, `void(ModbusResult, const uint8_t* pdu, uint16_t pduLen)`.
`pdu` is valid only during the call, and is null when nothing came back.

0x07, 0x08, 0x0B, 0x0C and 0x11 are **serial-line only** in the specification,
so a TCP server built with this library refuses them with Illegal Function.
0x14, 0x15 and 0x18 address a file store or a queue that belongs to your
application, so the server answers them only through a function handler you
register (see [Reaching past the facade](#reaching-past-the-facade)).

---

## Build-time settings

Defaults chosen so that nothing needs setting. Define one before the library is
compiled (a `-D` compiler flag, or `target_compile_definitions()` in CMake) only
when a project needs more than the default:

| Macro | Board | Host | |
| --- | --- | --- | --- |
| `MODBUS_QUEUE_CAPACITY` | 16 | 64 | requests that may wait at once, per node |
| `MODBUS_PAYLOAD_SPACE` | 1024 | 64 × 253 | bytes the queued requests' values share |
| `MODBUS_MAX_UNITS` | 16 | 248 | units a client keeps its own timing for; past that, the unit heard from longest ago gives up its row |

`queueSize()` changes the first two per node at run time, which is usually the
better choice.

The library reads time only through `mb_millis()` and `mb_micros()`. On a target
with its own clock (an RTOS tick, a hardware timer), install it with
`mb_setClockSource(millisFn, microsFn)`; `mb_resetClockSource()` puts the
platform clock back.
