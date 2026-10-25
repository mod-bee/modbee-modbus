# Best practices

The library will stop you writing something that cannot work. It cannot stop you
writing something that works badly — a bus so busy that the one command that
mattered waited behind forty readings nobody looked at.

This page is about that. It is opinionated on purpose, and every rule says why,
because the why is what lets you break it correctly.

- [The bus is a shared resource](#the-bus-is-a-shared-resource)
- [Yes, you can call it in the loop](#yes-you-can-call-it-in-the-loop)
- [Ask for what you need, when you need it](#ask-for-what-you-need-when-you-need-it)
- [Keep the bus quiet for what matters](#keep-the-bus-quiet-for-what-matters)
- [Ask in blocks, not one at a time](#ask-in-blocks-not-one-at-a-time)
- [Write only when something changed](#write-only-when-something-changed)
- [Act on quality, not on a number](#act-on-quality-not-on-a-number)
- [Say what should happen when you go quiet](#say-what-should-happen-when-you-go-quiet)
- [Leave the timing alone](#leave-the-timing-alone)
- [Getting the wiring right the first time](#getting-the-wiring-right-the-first-time)
- [TCP is not the same problem](#tcp-is-not-the-same-problem)
- [Designing a device other people will use](#designing-a-device-other-people-will-use)
- [Running for months, not minutes](#running-for-months-not-minutes)
- [The short version](#the-short-version)

---

## The bus is a shared resource

On Ethernet, a request costs almost nothing. On RS-485 it costs **time that
nobody else can use**, and there is far less of it than people expect.

At 19200 baud, 8E1, a character is 11 bits — about **0.57 ms**. So:

| Transaction at 19200 | Bytes on the wire | Airtime |
| --- | --- | --- |
| read 1 register | 15 | ~9 ms |
| read 10 registers | 33 | ~19 ms |
| read 20 registers | 53 | ~30 ms |
| read 125 registers (the maximum) | 263 | ~150 ms |
| write 1 register | 16 | ~9 ms |

Add the framing silence either side (about 4 ms at this speed) and however long
the device takes to think — 2 ms for an I/O block, 50 ms or more for a cheap
sensor or a busy drive.

So a realistic figure for a small read at 19200 is **25–35 transactions per
second, for the whole bus**, shared between every device on it. At 9600 it is
half that. The numbers are worth internalising:

| Baud | Roughly, small transactions per second |
| --- | --- |
| 9600 | 12–18 |
| 19200 | 25–35 |
| 38400 | 45–70 |
| 115200 | 100–200 |

<details>
<summary>Where those numbers come from</summary>

A Modbus RTU character is **always 11 bits** — start bit, 8 data, parity, stop —
whatever the parity setting, so a character at 19200 takes 11 / 19200 = 0.573 ms.

An RTU read of *N* registers is 8 bytes out and 5 + 2*N* back, plus t3.5 of
silence either side (2.0 ms each at 19200; the specification fixes it at
1.750 ms above 19200 baud). A 10-register read is therefore 33 characters plus
4 ms of framing — **22.9 ms of wire**, or 44 per second if the device answered
instantly. It does not, so the table above allows for a few milliseconds of
device turnaround, which is where the ranges come from. A slow device will do
worse and no amount of tuning at your end will change that.

</details>

Those are bus totals, not per device. Twelve devices at 19200, polled once a
second each, is twelve of your thirty. Comfortable. The same twelve devices
polled every 100 ms is 120 — four times what exists, and the bus simply falls
behind, with every reading arriving later than the last until nothing is fresh.

**If you take one thing from this page:** work out your budget before you choose
your intervals. It takes two minutes and it is the difference between a system
that works for ten years and one that is mysteriously slow from day one.

---

## Yes, you can call it in the loop

It is safe. It is designed for it. Here is exactly what happens:

```cpp
void loop() {
    mb.poll();
    mb.readIreg(17, 30, level);        // every single scan
    mb.writeHreg(17, 40, setpoint);    // every single scan
}
```

- The second identical read, queued before the first has gone out, is refused as
  **`Duplicate`**. One request on the wire, not two.
- A write to an address already queued **replaces** the queued value rather than
  stacking behind it, so the bus carries the setpoint you settled on rather than
  a history of every step it passed through.
- Nothing blocks, nothing is sent from inside the call, and the queue never
  grows without bound.

So you will not flood anything, and you do not need a timer or a state machine
of your own. **That is the guarantee.**

**But it is not free.** The queue drains as fast as the bus allows, which means
a read called every scan becomes a read issued *as often as the bus can carry
it* — continuously, at maximum rate, for ever. One such call can consume a whole
RS-485 segment on its own.

The pattern is safe. It is rarely what you want.

---

## Ask for what you need, when you need it

Three ways to read, in increasing order of politeness:

**1. On demand — the best one when it applies.**

```cpp
if (operatorPressedRefresh) mb.readHregs(17, 0, settings, 16);
if (startingUp)             mb.readText(17, 500, serial, sizeof serial, 10);
```

A configuration block, a serial number, a fault log: read it when something
makes it relevant. Most values in most systems are like this, and people poll
them out of habit. A device's rated current does not change while it is running.

**2. Watched, at an interval matched to the process.**

```cpp
mb.watchIreg(17, 30, &tankLevel, 1, 2000);     // a tank that takes minutes to fill
mb.watchIreg(18, 10, &motorCurrent, 1, 250);   // a trip you need to see quickly
```

The interval should come from **how fast the thing can actually change and how
quickly you must react**, not from how fast the CPU can ask. A tank level polled
every 100 ms gives you exactly the same information as one polled every two
seconds, and costs twenty times the bus.

**3. In the loop, unconditionally.** Correct, and the least polite. Reach for it
for a single critical value on a bus with nothing else on it, or while you are
bringing something up on the bench.

### A rule of thumb for intervals

| What it is | Interval |
| --- | --- |
| a safety interlock or a trip you must see | as fast as your budget allows — and consider whether Modbus is the right place for it at all |
| a running measurement an operator watches | 250 ms – 1 s |
| a slow process value: level, temperature | 1 – 5 s |
| a total, a counter, a runtime hour meter | 10 s – 1 min |
| configuration, nameplate, serial number | once at start-up, and on request |

---

## Keep the bus quiet for what matters

This is the part that separates a system that behaves under load from one that
does not.

A control word, an E-stop, a setpoint change — these go out **through the same
queue as everything else**, and they wait behind whatever is already on the
wire. If your bus is saturated with polling, the command that matters inherits
all of that delay.

So:

**Poll less, so commands go out sooner.** A bus at 30% occupancy dispatches a
command in a few milliseconds. A bus at 95% dispatches it whenever it gets
round to it, and the number is not one you can predict.

**Stop polling what you are not looking at.**

```cpp
// Declared once, in setup(), then switched off until it is wanted.
detailWatch = mb.watchHreg(DRIVE, 200, detail, 20, 500);
mb.stopWatching(detailWatch);

// In loop(): read the detail only while the diagnostic screen is open.
const bool active = mb.poller().isActive(detailWatch);
if (diagnosticScreenVisible && !active)  mb.poller().resume(detailWatch);  // reads at once
if (!diagnosticScreenVisible && active)  mb.stopWatching(detailWatch);
```

Stop and resume the same watch rather than creating a new one each time: a
handle stays valid after `stopWatching()`, and a program that watched afresh on
every screen change would grow its poll table for as long as it ran.

**Slow down when nothing is happening.** A plant that is stopped does not need
the same attention as one that is running. Stop the fast watches and start slow
ones.

**Check what is queued before doing something expensive.**

```cpp
if (mb.pending() > 8) return;      // the bus is busy; the log dump can wait
mb.readHregs(DRIVE, 300, faultLog, 100);
```

**Never issue a big read on a control path.** A 125-register read is 150 ms of
airtime at 19200. Anything urgent queued behind it waits that long, every time.
Split large reads up, or do them when the plant is idle.

**And the honest version of all this:** if you need a command to be acted on in
a guaranteed number of milliseconds, that is not what a polled serial protocol
is for. Use a hardwired interlock. Modbus is how you tell the plant what to do;
it is not how you stop it in an emergency.

---

## Ask in blocks, not one at a time

```cpp
// Four transactions. About 36 ms of bus at 19200.
mb.readHreg(17, 10, a);
mb.readHreg(17, 11, b);
mb.readHreg(17, 12, c);
mb.readHreg(17, 13, d);

// One transaction. About 12 ms.
uint16_t block[4];
mb.readHregs(17, 10, block, 4);
```

The overhead is per *transaction*, not per register: address, function code,
checksum, two lots of framing silence and one device turnaround, whether you
asked for one register or fifty. Reading a few spare registers you do not need
in order to span a gap is almost always cheaper than a second transaction.

The same applies to writing: `writeHregs()` for a contiguous group beats several
`writeHreg()` calls. With one caveat that matters — **some devices treat a block
write to a command register differently from single writes**, because the block
arrives atomically and the singles do not. If a manual is specific about it,
follow the manual.

---

## Write only when something changed

```cpp
// Fine - the library discards the duplicate.
mb.writeHreg(DRIVE, 40, setpoint);

// Better - nothing is even queued.
if (setpoint != lastSentSetpoint) {
    mb.writeHreg(DRIVE, 40, setpoint, [](ModbusResult r) {   // globals: nothing to capture
        if (mb_succeeded(r)) lastSentSetpoint = setpoint;
    });
}
```

The second form matters when the value is *nearly* constant — an analogue
setpoint that dithers in its last bit is a different value every scan, so
duplicate suppression never catches it and the bus carries a write for ever.
Apply a deadband to what you send, not just to what you display:

```cpp
if (abs((int)setpoint - (int)lastSentSetpoint) >= 2) { ... }
```

Note where `lastSentSetpoint` is updated: **in the completion handler, not next
to the call**. A write that timed out did not happen, and a device that missed
it should not be left at the old value for ever because a variable says it was
told.

---

## Act on quality, not on a number

A variable holding `21.5` tells you nothing about whether 21.5 is still true. On
a bus where one device has quietly failed, its last reading looks exactly like a
healthy one that has not changed.

```cpp
if (mb.quality(levelWatch) == ModbusQuality::Good) control(level);
else                                               holdSafeState();
```

Do this everywhere a reading drives an action. It is one line and it is the
difference between a controller that notices a failed sensor and one that keeps
regulating against a number frozen twenty minutes ago.

Treat the four answers as different things:

| | What it means | What to do |
| --- | --- | --- |
| `Good` | read within the last two intervals | act on it |
| `Uncertain` | a little late, or one read failed | usually act; log it if it persists |
| `Bad` | read once and now well out of date, or failed twice running | **a running fault** — cable, power, failed device |
| `Unknown` | never read at all | **a commissioning fault** — wrong address, device never fitted |

Alarming `Bad` and `Unknown` identically is a small mistake with a large cost:
one sends somebody to the field with a multimeter, the other sends them to the
configuration screen, and the alarm that cannot tell them apart sends them to
the wrong one.

---

## Say what should happen when you go quiet

If you are a device, decide what your outputs do when the client stops talking.
Nothing in Modbus does this for you, and the default — hold the last command
for ever — is how a pump commanded on by a client that has since lost power
stays on all weekend.

```cpp
mb.failsafeCoil(PUMP, false);      // the pump goes off
mb.failsafeHreg(SPEED, 0);         // the speed reference goes to zero
mb.failsafeAfter(5000);
mb.onFailsafe([](ModbusTable, uint16_t address, uint16_t) {
    logEvent("driven safe on client silence", address);
});
```

Choose the timeout from **how often the client actually polls you**, with room
for a retry: five seconds for a client polling every second is about right; one
second is not, and will trip on the first busy moment.

Addresses with no failsafe set hold their last value, which is deliberate —
driving an output is an action, and doing it unasked is its own hazard. Set them
explicitly for anything that moves.

Log `onFailsafe` separately from your change handler. One reports a client
acting on you; the other reports *you* acting because a client stopped. An
incident review that only has the first will show every commanded action and no
safety-driven one.

---

## Leave the timing alone

Timeouts and retries are measured per device and include the airtime the medium
physically needs. A slow soft-starter and a fast I/O block on the same bus each
get the figure they deserve, and it adapts as conditions change.

Pinning a figure turns that off:

```cpp
mb.timeout(500);      // now everything waits 500 ms, including the fast ones
mb.retries(1);
```

Do it when a device's manual gives a number, or when something downstream of you
is itself waiting and you must answer before it gives up. Otherwise do not — the
measured figure is almost always better than a guess, and the usual reason
people pin one is a different problem wearing a timeout's clothes.

**More retries is rarely the fix.** Retries help with occasional corruption.
They do not help with a device that is absent, a wrong baud rate or a wiring
fault — there they triple the time before you find out, and triple the bus
consumed by finding out. If retries are rescuing you routinely, look at
`stats().checksumErrors` and fix the cause.

---

## Getting the wiring right the first time

In the order that actually saves time:

**1. Serial format.** Modbus's default is **8E1** — 8 data bits, even parity,
1 stop bit. A device wired for 8N1 will not talk to one wired for 8E1 however
right everything else is, and this is the single most common reason a bus that
"should work" does not.

**2. Address base.** Manuals number registers from 1, or from 40001, or from
4x0001. The wire numbers from **0**. "40001" almost always means holding
register **0**; "40100" means **99**. If every reading is one register out, this
is why.

**3. Termination and bias.** 120 Ω at each end of the segment, once — not on
every device. Bias resistors somewhere, once. An unbiased idle bus floats, and
a floating bus produces noise bytes that look exactly like a device answering
badly. `stats().noiseBytes` climbing with nothing connected is the signature.

**4. Direction pin hold.** If you drive an RS-485 transceiver's DE line, the
driver must stay enabled until the last stop bit has left. Too short and the
final character is truncated on the wire — which looks like a checksum error at
the other end, not like a wiring problem.

```cpp
mb.directionPin(4, true, 2000);   // pin 4, active high, hold 2 ms after the last byte
```

**5. Watch what touches the serial port before your program does.** A start-up
script that opens the device to check it is there — `exec 3<>/dev/ttyUSB0`, or
anything similar — makes that port the session's **controlling terminal**,
because a shell cannot pass `O_NOCTTY`. Everything launched afterwards inherits
it.

That matters because `tcsetattr` and `tcdrain` raise `SIGTTOU` when the caller
is in a background process group and the device is its controlling terminal —
and the default action of `SIGTTOU` is to **stop the process**. Not fail the
call: suspended, state `T`, no error, nothing logged. Under `timeout`, under
some supervisors, or under shell job control, that is a device that appears to
have crashed for no reason anybody can find.

The library blocks `SIGTTOU` around every terminal-control call so this cannot
happen to it. It is still worth not creating the condition: check the device
with `test -c /dev/ttyUSB0`, not by opening it.

**6. A server that also asks is a different animal.** RTU frames are delimited by
silence, and the library avoids relying on silence wherever it can: if a node
only ever *answers*, it works out each frame's length from its function code,
which no amount of buffering between the line and your program can interfere
with. A node that also *asks* hears both directions, where the same function
code means two different lengths, so it has to measure silence after all.

That happens automatically the first time you make a client call, and it is the
right trade — but it is worth knowing that a device doing both jobs on one
segment is more sensitive to a USB adapter's buffering than a plain server is.
`framingReason()` always says which is in use and why.

**7. One client.** Two clients on one RS-485 segment is possible and
is a specialist job. If you did not deliberately design for it, and readings are
intermittently corrupt, count how many things are talking.

When it still will not work, the diagnosis order is in
[Getting started](getting-started.md#when-it-does-not-work): stats, then `ping()`,
then `framingReason()`, then `onTrace()` and read the bytes.

---

## TCP is not the same problem

The airtime arithmetic mostly goes away. Different things bite instead.

**Keep the connection open.** Some SCADA packages open a socket, do one
transaction and close it. It works, and it costs a TCP handshake per reading.
If you are the client, connect once and stay connected — the library reconnects
on its own if the link drops.

**One transaction at a time is still the safe default.** Modbus TCP permits
several outstanding requests, matched by transaction id, and many devices
cannot cope with it. Leave `mb.client().config().maxInFlight` at 1 unless you
have tested that the particular device is happy.

**Idle connections close themselves.** A client that vanishes — laptop lid
closed, cable pulled — leaves a socket that never reports an error. Left alone,
enough of those make a healthy device unreachable to anybody new.

So a connection silent for sixty seconds is closed; and when every slot is
taken, a newcomer may displace a connection that has **never asked for
anything** and has sat there for several seconds — a scan, a half-open socket, a
client that died before its first request.

A connection that has completed even one request is never displaced, however
long it then goes quiet. Thirty seconds between polls is an ordinary client, and
throwing one off to admit a stranger would be a worse failure than refusing the
stranger.

So when every slot really is held by a working client, a newcomer **is**
refused — promptly, so it can see the refusal and retry. A device at its limit
and a device that has died must not look the same from outside. If that
happens to you regularly, the answer is a larger `maxClients()`, not a cleverer
eviction rule.

```cpp
mb.maxClients(8);         // how many at once
mb.idleTimeout(60000);    // the default; 0 disables it
```

Both are defaults you can change and rarely need to.

**Failsafe means all of them.** On a TCP server the failsafe timer is about
*every* client going quiet, not one. That is the right question: one workstation
closing is not a plant emergency; the network dying is.

**Know which protocol your converter speaks.** RTU-over-TCP and Modbus TCP share
port 502 and look similar in a capture. See
[Recipes](recipes.md#modbus-rtu-over-tcp--serial-converters).

---

## Designing a device other people will use

If you are building something that others will integrate:

- **Contiguous blocks.** Scattered registers force a client into many
  transactions where one would do. Group what is read together.
- **Say what you are.** `identity()` costs three strings and tells every
  commissioning tool what it is looking at.
- **Publish a register map** — address, type, units, scaling, read or write, and
  the word order of anything wider than a register. The last one is the detail
  most manuals omit and every integrator has to discover by experiment.
- **Put measurements in input registers and commands in holding registers.** The
  tables mean something. A client reading an input register knows it cannot
  break anything.
- **Refuse what you should refuse, before it happens.**

  ```cpp
  mb.onRequest([](uint8_t unit, const ModbusRequest& request) -> uint8_t {
      if (localMode && mb_isWriteFunction(request.function)) {
          return MB_EX_ILLEGAL_FUNCTION;
      }
      return MB_EX_NONE;
  });
  ```

  A panel in local mode that silently accepts remote writes is worse than one
  that refuses them, because nobody finds out until it matters.
- **Keep handlers short.** `addHregHandler()` and `onRequest()` run while a
  request is being served. No delays, no blocking I/O, no waiting on another
  bus.

---

## Running for months, not minutes

A device on a bench behaves. The questions that decide whether it is still
working in March are different ones, and this section is what the library does
about them so you do not have to.

### Nothing grows without bound

Every table inside the library has a ceiling, and each one is there because
without it a *remote peer* would choose how much memory your device used:

| | Bounded by |
| --- | --- |
| the operation queue | `queueSize()` (64 on a host, 16 on a board, by default) — taken when the link opens, and past it you get `QueueFull`, never a silent drop |
| concluded results held for `status()` | 64, oldest aged out |
| per-address write counters | the 64 most recently written |
| per-unit round-trip timing | one entry per unit id: up to 16 on a board (the unit heard from longest ago gives up its row), every unit on a host |
| TCP connections | `maxClients()` (and, on Arduino, the `ModbusArduinoServer` limit) — one past the limit is accepted and immediately closed |
| conclusions run per `poll()` | 64, the rest carried to the next poll rather than dropped |

That matters most on a bus where addresses vary. An HMI browsing registers, a
gateway forwarding whatever a client asks for, a commissioning scan: all of them
touch a different address every time, and a table keyed by address with no
ceiling is a heap that fills over days.

### Nothing loops for ever

`poll()` returns. Always, whatever the medium is doing.

With one deliberate exception, worth knowing about: transmitting waits for the
last byte to physically leave the port. On RS-485 there is no choice — switching
the driver off a moment early truncates the final character, and the far end
reports a checksum error that points at nothing. That wait is bounded by the
line rate and nothing else: a full frame is 24 ms at 115200 baud and **2.3
seconds at 1200**. If you are running a slow line, that is the number to have in
mind when something else shares the loop.

The two places the library can legitimately spin are a send the medium has not
made room for — a socket whose peer has stopped reading, a port held off by flow
control. Both are bounded, both call `yield()` between attempts (which on an
ESP8266 is what services the WiFi stack and the watchdog), and both give up
rather than hang. A send that gives up is reported: the link drops, the
application sees `LinkDown`, and `stats()` counts it.

This is worth being explicit about because a device that has hung does not look
like a device that has hung. It looks like one that crashed, and there is no log
line either way.

### Everything you queue is concluded

Including when the medium never accepted it at all. A TCP client whose server is
down queues reads that can never be transmitted; each one concludes as
`LinkDown` rather than sitting in the queue for ever. That is deliberately not
`Timeout`:

- **`Timeout`** — it went out, and nothing came back. *The device.*
- **`LinkDown`** — it never went out. *The transport.*

An application that opens a slot per request can hold that slot without ever
stalling one.

### Running out of memory is survivable

Arduino cores build without exceptions, so an exhausted heap makes `new` return
null rather than throw. Every allocation in the library is checked, and the
failure path is always to refuse rather than to fault: a connection that cannot
be allocated is closed and the client retries, `connect()` and `listen()` return
false, and an event says why. A device short of memory keeps running badly
instead of stopping.

### What it costs

Per link, before any of your own data: roughly **2 KB**, being an assembly
buffer, a transmit buffer and the operation queue. That is why the 8-bit AVR
boards are out of scope.

Per queued operation: nothing new. The queue's slots and the space for write
values are allocated once, when the link opens, and a running node does not
allocate after that. The one exception is a handler that is a lambda *with
captures* (or a `std::function`), which is copied to the heap for as long as the
operation is queued; a plain function or a capture-free lambda costs nothing. A
TCP server adds about 1 KB per open connection.

The deepest call the library makes is roughly 1.5 KB of stack — comfortable on
an ESP32, snug but workable on an ESP8266. If you are close to the edge, the
thing to shrink is the queue (`queueSize()`), not the buffers.

### What a hostile peer cannot do

Tested against a peer that does each of these while an ordinary client checks it
is still being served correctly throughout:

- connect, ask for the largest read there is, and never read the answer
- open and drop 400 connections, a third of them abortively
- send random bytes, a length field of 65535, a length field of zero, a protocol
  id that is not Modbus
- deliver one byte every 50 ms, which is both a congested link and a slow-loris
- open 60 sockets at once

None of them takes the device down, and none of them is ever answered with
anything but silence or an exception. Noise is counted in `stats()` rather than
silently eaten, so a bus with a problem says so.

---

## The short version

1. Work out your bus budget before choosing poll intervals.
2. Reading in the loop is safe. Reading only when you need it is better.
3. Match the interval to the process, not to the CPU.
4. A quiet bus dispatches an urgent command quickly. A busy one does not.
5. One block read beats four single reads.
6. Write on change, and update your "last sent" in the completion handler.
7. Gate every control decision on `quality()`.
8. Set a failsafe on anything that moves.
9. Leave timeouts measured unless a manual gives you a number.
10. 8E1, address base 0, terminate once, bias once.
11. For anything that must happen in a guaranteed time, use a hardwired
    interlock. Modbus is not a real-time protocol and this library does not
    pretend otherwise.
12. Nothing inside the library grows without bound, loops for ever, or faults on
    an exhausted heap — so the things that break a device in month three are
    ours to have got right, not yours to work around.
