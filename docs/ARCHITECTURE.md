# Architecture

Five layers, and why each boundary is where it is.

This is the document to read if you want to know *why* the library is shaped the
way it is, or if you are about to reach past the one-object API into the layer
underneath. If you just want to make it work, read
[Getting started](getting-started.md) instead.

```
 ┌───────────────────────────────────────────────────────────────────────┐
 │ L4  FACADE     ModbusRTU · ModbusASCII                                │
 │                ModbusTCP · ModbusBridge   one object, either or both   │
 │                                           roles, nothing hidden        │
 ├───────────────────────────────────────────────────────────────────────┤
 │ L3  ROLE       ModbusClient        reads and writes other devices      │
 │                ModbusServer        answers other devices               │
 │                ModbusServerPool    one server, many TCP clients        │
 │                ModbusGateway       TCP front, serial back              │
 │                ModbusPoller        repeating reads, with quality       │
 │                ModbusPointSet      values defined by settings          │
 │                ModbusNode          one link, either or both roles      │
 ├───────────────────────────────────────────────────────────────────────┤
 │ L2  CORE       ModbusOperations    the queue: dedup, supersede, retry  │
 │                ModbusServerEngine  request in, response out            │
 │                ModbusDataMap       bindings to the application's own   │
 │                                    variables                           │
 ├───────────────────────────────────────────────────────────────────────┤
 │ L1  PROTOCOL   ModbusPdu           every function code, no I/O         │
 │                ModbusCodec         RTU · ASCII · TCP framing           │
 ├───────────────────────────────────────────────────────────────────────┤
 │ L0  LINK       ModbusTransport     one byte channel                    │
 │                ModbusListener      produces channels (TCP)             │
 └───────────────────────────────────────────────────────────────────────┘
```

Dependencies point downward only.

---

## L4 composes; it does not wrap

`ModbusRTU`, `ModbusASCII`, `ModbusTCP` and `ModbusBridge` own a transport, a
codec, a link and the role objects, and forward to them. They add no protocol
behaviour of their own — every guarantee in this document belongs to a layer
underneath, and is the same whether you reach it through the facade or directly.

That matters because it means the facade can never be the thing standing between
you and a device. `client()`, `server()`, `map()`, `link()`, `poller()` and
`points()` are public, return references to the real objects, and are safe to
use alongside the short form. The server side and the points are created the
first time they are asked for, so a node that only asks never carries them. An application that outgrows `mb.readHreg()` keeps everything it has
already written.

---

## L1 has no I/O and no clock

`ModbusPdu` is free functions over buffers. It never allocates, never reads the
clock, and never touches a socket. Everything it decides is decided by the bytes
in front of it.

That is not tidiness. It is what makes the layer **exhaustively testable against
the specification's own worked examples** — 39 captured byte vectors, in both
directions, compared byte for byte — and **fuzzable in isolation**, which is where
parser defects live. A PDU layer entangled with a socket can only be tested
through one.

The class of bug this shape catches is "a declared byte count was trusted as a
bound". A response declares how many bytes follow; a corrupt or hostile one
declares more than it carries, or more than the caller's buffer holds. Both are
one-line checks here, and both are exercised by the fuzzer on every build.

---

## Responses are decoded against their request

A Modbus response carries **no start address**, and a bit-read response carries a
**byte count rather than a bit count**. Only the client knows what it asked for.

So `decodeResponse()` takes the request it answers. That single decision removes
an entire class of protocol extension — the temptation to add an address to every
read response so replies can be matched, which any stock Modbus client then reads
as part of the payload.

It also makes a bit read exact. Deriving the count as `byteCount * 8` over-reports
by up to seven bits every time, and those seven bits land in the caller's array.

---

## The transport is one channel; the listener produces them

`ModbusTransport` is six required methods, a few optional hints, and no
platform types. `ModbusListener` is the
abstraction a serial link does not need and a TCP one cannot do without: a Modbus
TCP server faces several clients at once, each with its own socket, its own
transaction stream and its own lifecycle.

What is **shared** across those connections is what the device exposes — the data
map, the identity, the access rules. What is **per connection** is framing and
transaction state. Making that split explicit is the whole of `ModbusServerPool`,
and everything else follows from it.

`bitsPerSecond()` earns its place: RTU framing is defined in character times, and
a line fast enough to deliver a whole frame between two looks at it leaves no
spacing to measure. The one thing that cannot be derived from the bus is the one
thing the transport already knows.

`echoesTransmissions()` earns its place the hard way. **A Modbus write response
is a byte-for-byte echo of its request.** So any attempt to recognise a hardware
echo by comparing content throws away every write response it receives — the bug
looks like "writes never complete on half-duplex RS-485" and is invisible on a
bench with a loopback-free adapter. Only the transport knows whether the wiring
loops back, so only the transport can say, and suppression is by byte count
rather than by content.

---

## RTU framing ships two strategies, and defaults to the unusual one

The specification says a frame ends after **t3.5 character times of silence**.
That is the right question only if silence on the line and silence at this end
are the same thing — and on anything that buffers, they are not:

- a USB serial adapter delivers a frame's bytes in one burst, on its own schedule
- a pseudo-terminal delivers whenever the kernel schedules the writer
- a general-purpose OS can stall a process for longer than t3.5 mid-frame

Each inserts a gap the framer cannot tell from a frame boundary, and each such
gap splits one frame into two that both fail their checksum.

So **length framing is the default wherever the direction is known**: read the
function code, work out how long the frame must be, take exactly that many bytes,
check the CRC. Buffering, bursts and scheduling stalls are all invisible to it.

Its cost is that it must know whether it is decoding requests or responses,
because the same function code has different lengths each way. A listener — a bus
monitor, or a node in both roles on a shared segment — reads both, so it gets gap
framing and must live with the timing requirement.

Two shapes carry no length anywhere — a diagnostics response and a device
identification response — and neither does a vendor function code. Those are
delivered by an **idle backstop**, which also recovers from noise: a noise byte
ahead of a frame usually makes the predictor compute a length far longer than the
frame really is, and nothing inside the byte stream can tell it it is wrong.
Recovery requires the predicted length *and* the checksum to agree, so a resync
never hands up a frame that merely looks plausible.

Above 19200 baud the specification fixes t1.5 at 750 µs and t3.5 at 1.750 ms
rather than continuing to scale them; the character time itself is always
**11 bits**, start and stop bits included, whatever the parity setting.

`framingReason()` says which strategy was chosen and why, in a sentence, because
"why is my RTU splitting frames" is the most common question in every Modbus
library.

---

## The queue is the reason to use this

Three properties are worth stating on their own.

**Supersede.** A write to an address already queued *replaces* the queued value
rather than stacking behind it, and the superseded caller is **told**. A control
loop moving a setpoint every scan sends the value it settled on, not a history.
The guards are subtle: an operation already **on the wire** is never rewritten,
because the echo would then confirm a value that was never transmitted. A new
write to that address queues behind it instead.

**Exactly one conclusion.** Every accepted operation reaches one, always —
success, refusal, timeout, superseded, queue full. That is what lets an
application open a slot per request and never stall it. An operation that is
accepted and silently never concluded does not look like a bug; it looks like a
control loop that quietly stops issuing requests after a few hours, which is far
harder to find.

**Re-entrancy.** A completion handler is application code, and the natural thing
for one to do is queue the next operation — which changes the queue while the
library is walking it. The rule, stated once in `retireAt()`: *the operation is
removed from the queue before its handler runs, and nothing reads the record
afterwards.*

**Fixed storage.** The queue's slots and the bytes its writes carry are
allocated once, when the link opens, and never move. A running node does not
allocate, and an allocation-counting check holds it to that. Nested conclusions are flattened rather than recursed, so the depth
is bounded however the application behaves.

---

## Timing is measured, not configured

Every timing field defaults to `MODBUS_AUTO`, meaning the library measures it.
Pinning a field turns adaptation off **for that field alone**.

The response timeout is the sum of two things neither of which can be argued
with:

- **what this host can enforce** — twice the measured `poll()` interval. A timeout
  shorter than the gap between two looks at the link cannot be met however fast
  the device answers.
- **what the medium needs** — the airtime of *this particular exchange*. At 19200
  baud a 125-register read is 150 ms of pure airtime before the device has
  thought about anything; at 9600 it is 300 ms. Sizing that from a maximal
  exchange instead is twenty times too generous for a small read, which turns
  every dead device into a twenty-fold waste of the segment.

A unit nothing has been measured about yet gets a generous cold-start figure
(ten times the floor, about 200 ms by default), because declaring a device
absent that was merely thinking is the worse mistake.

On top of that sits a per-unit RFC 6298 estimator, so a slow soft-starter and a
fast I/O block on one bus get the timeout each deserves. Retries carry backoff
and jitter, so two clients that start in step do not stay in step.

---

## The gateway is where the session model pays for itself

TCP in front, a serial segment behind, routed by unit id. Several clients, one
shared bus, and neither end knowing anything about the other.

It does **not** decode what it carries. A device behind it may support function
codes this library does not, and a gateway that re-encoded from a decoded
structure would quietly drop whatever the structure could not hold. Requests for
a routed unit are forwarded byte for byte.

It owes two exceptions of its own, and getting them right is most of what
separates a gateway from a forwarding loop:

- **0x0A Gateway Path Unavailable** — no route, or no room. The gateway is
  misconfigured or overloaded, and says so *immediately* rather than making the
  client wait out a timeout for an answer that was never coming.
- **0x0B Gateway Target Device Failed To Respond** — the route was there and the
  device was not. This is the one that matters: it tells a client "the network is
  fine, the device is off", which a plain timeout can never say.

Three rules, each learnt the hard way:

1. A front-side retry **queues**, it is not refused. A client's timeout knows
   nothing about the field bus, so a retry arriving while the first request is
   still out there is normal — and answering it with 0x0A says the gateway is
   misconfigured when it is merely busy.
2. **Do not retry as hard as the client.** One retry downstream, not three: the
   budgets multiply, and a client that gives up first gets a plain timeout
   instead of the 0x0B that would have told it which end failed.
3. **Answer faster than a client gives up.** The gateway's cold-start budget is a
   third of the usual, so it concludes inside a typical client's first attempt.

A refused request must also produce **exactly one** response. A forwarding path
that both refuses a request and leaves its pass-through handler armed emits two,
and a client that receives two answers to one transaction is desynchronised for
the rest of the connection.

---

## The poll engine is where a number becomes a measurement

A client gives you a value. It does not tell you how old that value is, and a
variable holding 21.5 says nothing about whether 21.5 is still true.

`ModbusPoller` adds four things: repeating reads into the caller's own variables,
a quality judged **against the interval the caller asked for**, a deadband so an
analogue reading dithering in its last bit is not an event on every poll, and a
backoff so a device that has gone does not consume the whole segment at full
rate.

`Unknown` and `Bad` are deliberately different. One means it was never read at
all — a commissioning problem, an address typed wrong. The other means it *was*
read and has since gone stale — a cable, a device that lost power. An alarm list
that conflates them sends somebody to the wrong end of the plant.

---

## How it is verified

- **The specification's own worked examples**, in both directions and in all
  three framings, compared byte for byte.
- **Fuzzing** of the protocol layer and the framers with random and malformed
  input, under the address and undefined-behaviour sanitizers.
- **A hostile channel**: a simulated link that loses, truncates, corrupts and
  duplicates frames and injects noise, with a clock driven by hand so timing
  behaviour does not depend on how fast the host runs.
- **Byte-paced serial**: a virtual bus that clocks every byte out at the stated
  line rate, at 9600, 19200 and 115200 baud, with both framing strategies. It is
  the only honest test of RTU gap framing, because RTU frames are delimited by
  **silence** and no in-process loopback can model silence.
- **Interoperability** against other implementations, including pymodbus and
  libmodbus.
- **Arduino builds**: every example compiled for the ESP32, ESP8266, SAMD and
  RP2040 cores in CI.

The suite runs under the address, undefined-behaviour and thread sanitizers.
CI builds on Linux, macOS and Windows, with warnings treated as errors.
