#pragma once

#include "ModbusPdu.h"
#include "ModbusResult.h"
#include "ModbusTiming.h"
#include "ModbusTyped.h"

#include "ModbusCallback.h"
#include "ModbusPlatform.h"

// Queue sizes. A host keeps the generous limits it always had; a board holds
// fewer operations and one shared kilobyte for the bytes they carry. Both are
// per node (ModbusClientConfig) and can be raised for a project that needs it.
#ifndef MODBUS_QUEUE_CAPACITY
    #if defined(MODBUS_PLATFORM_HOST)
        #define MODBUS_QUEUE_CAPACITY 64
    #else
        #define MODBUS_QUEUE_CAPACITY 16
    #endif
#endif
#ifndef MODBUS_PAYLOAD_SPACE
    #if defined(MODBUS_PLATFORM_HOST)
        #define MODBUS_PAYLOAD_SPACE (MODBUS_QUEUE_CAPACITY * MB_MAX_PDU)
    #else
        #define MODBUS_PAYLOAD_SPACE 1024
    #endif
#endif

// =============================================================================
// PENDING OPERATIONS
// =============================================================================
//
// The queue between the application and the wire, and the reason this library
// is worth using rather than writing a request builder by hand.
//
// What the application gets, with no configuration at all:
//
//   Stacked      Operations queue as they are made. Nothing blocks and nothing
//                is sent inside the call that made it.
//   Deduplicated Queue the same read twice and the second is refused. Queue the
//                same write twice and so is that.
//   Superseded   Write a setpoint faster than the bus can carry it and the
//                QUEUED value is replaced, not stacked behind. A control loop
//                moving a setpoint every scan sends the value it settled on,
//                not a history of every step - and whoever was waiting on the
//                replaced write is TOLD, rather than left hanging.
//   Retried      With backoff and jitter, and then reported either way.
//   Concluded    Exactly once, always. See ModbusResult.
//
// --- Re-entrancy ---
//
// A completion handler is application code, and the natural thing for one to do
// is queue the next operation: chain a request, retry a write that failed,
// drive a state machine on. That call pushes onto the queue and reallocates it,
// so any reference, pointer or iterator held across a handler is left pointing
// at freed memory.
//
// The rule the whole class follows: AN OPERATION IS REMOVED FROM THE QUEUE
// BEFORE ITS HANDLER RUNS, AND NOTHING READS THE RECORD AFTERWARDS. Every path
// that concludes an operation goes through retireAt() so the rule is stated
// once.
//
// --- Storage ---
//
// Fixed, and allocated once: a slot per operation, the order they were queued
// in, one byte area for what they carry, and rings for the conclusions owed.
// A running node never allocates. Slots do not move, so a record's address is
// stable for as long as it is queued; the ORDER is what changes, which is why
// the methods below still work by position.

// What a read's destination buffer holds.
enum class ModbusResultKind : uint8_t {
    None = 0,     // A write: nothing comes back but an echo
    Bits,         // bool array
    Registers,    // uint16_t array
    Text,         // char buffer, registers unpacked two characters each
    Raw,          // uint8_t array, the response payload verbatim
    Typed,        // A value wider than a register, converted on arrival
    Probe,        // Two registers, read as ModbusOrderProbe (all four word orders)
    Fifo,         // uint16_t array of FIFO values; resultCapacity is in values
};

// Which wide type a Typed result carries.
//
// Converted DURING unpacking, straight from the response's own bytes into the
// caller's variable. No staging buffer, and therefore nothing whose lifetime
// has to be managed between queueing a read and its answer arriving - which is
// what makes `client.readFloat32(17, 100, flow)` a single call with no
// scaffolding around it.
enum class ModbusWideType : uint8_t {
    None = 0,
    Float32, Int32, UInt32,
    Float64, Int64, UInt64,
};

// Registers a wide type occupies.
inline uint16_t mb_wideRegisters(ModbusWideType type) {
    switch (type) {
        case ModbusWideType::Float32:
        case ModbusWideType::Int32:
        case ModbusWideType::UInt32:  return 2;
        case ModbusWideType::Float64:
        case ModbusWideType::Int64:
        case ModbusWideType::UInt64:  return 4;
        case ModbusWideType::None:    return 0;
    }
    return 0;
}

// Told when an operation reaches its one conclusion.
typedef ModbusCallback<void(ModbusResult)> ModbusCompletionHandler;

// Told the raw response PDU of a passthrough operation.
//
// `pdu` is valid only for the duration of the call, and is null when the
// operation concluded without a response - a timeout, or a refusal before
// anything reached the wire. A gateway copies what it needs and returns.
typedef ModbusCallback<void(ModbusResult, const uint8_t* pdu, uint16_t pduLen)>
        ModbusPassthroughHandler;

// One queued operation.
struct ModbusPendingOp {
    uint8_t unit = 0;

    // The request, WITHOUT its payload pointer.
    //
    // `request.data` is always null in storage and is bound to the payload by
    // ModbusOperations::bound() at the moment of encoding. Storing the pointer
    // would be a bug: the byte area closes up when an earlier operation leaves,
    // which moves every payload behind it.
    ModbusRequest request;

    // Where this operation's bytes are in the queue's byte area. Set by the
    // queue; zero length for a request that carries none.
    uint16_t payloadAt = 0;
    uint16_t payloadLen = 0;

    // Where a read's answer goes, in the caller's own memory.
    void* resultPtr = nullptr;
    ModbusResultKind resultKind = ModbusResultKind::None;

    // Bytes available at resultPtr. Needed where the length the wire carries is
    // not implied by the quantity: a register read writes exactly two bytes per
    // register, but a text read copies whatever byte count the response
    // declares, and an odd one is half a register more than the quantity
    // accounts for. Bounding that copy in registers and performing it in bytes
    // is how a peer overruns the caller's buffer.
    uint16_t resultCapacity = 0;

    // The gap the caller's own layout puts between elements, 0 when packed.
    uint16_t resultStride = 0;

    // For ModbusResultKind::Typed: what to convert the registers into, and
    // which way round the device lays them out.
    ModbusWideType wideType = ModbusWideType::None;
    ModbusOrder wideOrder = ModbusOrder::ABCD;
    uint16_t wideCount = 1;   // How many values, not how many registers

    mb_time_t queuedAt = 0;
    mb_time_t sentAt = 0;
    uint8_t retryCount = 0;

    // Set once transmitted. A request stays queued until its answer arrives or
    // it times out, and must not be sent again meanwhile.
    //
    // Modbus echoes every non-broadcast write, so this gates writes as well as
    // reads - which makes "a write that was sent and never concluded"
    // structurally impossible here.
    bool awaitingResponse = false;

    // Identifies this operation on the wire. On TCP it IS the MBAP transaction
    // identifier. On serial there is no such field, so at most one operation
    // per link may be outstanding and this is only used for bookkeeping.
    //
    // Destination, function and address do not identify an operation: a write
    // can be in flight while another superseding it is queued.
    uint16_t opId = 0;

    // A broadcast is written to every server on the segment and answered by
    // none of them, so it concludes on transmission and is followed by a
    // turnaround delay rather than by a response.
    bool broadcast = false;

    // Forward these exact PDU bytes, and hand back exactly what comesous.
    //
    // A gateway must carry a request it does not itself implement: a device
    // behind it may support function codes we do not, and re-encoding from a
    // decoded struct would quietly drop anything the struct cannot hold. So a
    // passthrough operation sends `payload` verbatim and delivers the response
    // PDU verbatim, decoding neither.
    //
    // The matching is correspondingly weaker - the function code must be the
    // one asked for, or its exception form - which is all that can be checked
    // about a PDU whose shape is unknown.
    bool passthrough = false;
    ModbusPassthroughHandler onPassthrough;

    ModbusCompletionHandler onComplete;
};

class ModbusOperations {
public:
    ModbusOperations() noexcept {}
    ~ModbusOperations();
    ModbusOperations(const ModbusOperations&) = delete;
    ModbusOperations& operator=(const ModbusOperations&) = delete;

    // How many operations may be queued, and how many bytes they may carry
    // between them. Each slot is roughly 90 bytes. The storage is allocated at
    // the first operation after this - never per operation. Raised limits take
    // effect once the queue is idle.
    void setCapacity(size_t capacity, size_t payloadSpace = MODBUS_PAYLOAD_SPACE);
    size_t capacity() const { return _capacity; }
    size_t payloadSpace() const { return _payloadSpace; }

    // --- Queueing ---

    // Returns Queued, or Duplicate / QueueFull / Superseded-of-another when
    // refused. A refusal is always STATED; nothing is ever silently dropped,
    // because a dropped setpoint looks exactly like a delivered one.

    // `payload` is the request's data (a write's values, a passthrough's whole
    // PDU), copied into the queue's byte area. QueueFull when either the slots
    // or the byte area are full.
    ModbusResult add(const ModbusPendingOp& op, const uint8_t* payload = nullptr,
                     uint16_t payloadLen = 0);

    // An operation's bytes, and its request with them bound. Valid until the
    // next call that queues or concludes anything.
    const uint8_t* payloadOf(const ModbusPendingOp& op) const {
        return op.payloadLen ? _bytes + op.payloadAt : nullptr;
    }
    ModbusRequest bound(const ModbusPendingOp& op) const {
        ModbusRequest r = op.request;
        r.data = payloadOf(op);
        r.dataLen = op.payloadLen;
        return r;
    }

    // Reports an operation that never entered the queue - one served against
    // this node's own data map, or refused before it could be queued.
    //
    // HELD rather than reported on the spot: reporting inside the call that
    // accepted the operation runs application code before that call has
    // returned, and a caller has not yet recorded what it is waiting for.
    // Drained by dispatchLocal(), which is where every other completion is
    // reported from.
    void reportLocal(const ModbusCompletionHandler& handler, ModbusResult result,
                     uint8_t unit, uint16_t address);
    // The same for a passthrough refused before it was queued: its handler is
    // told the result, with no bytes.
    void reportLocal(const ModbusPassthroughHandler& handler, ModbusResult result,
                     uint8_t unit, uint16_t address);
    void dispatchLocal();

    // --- Driving ---

    // The next operation that may go out on this link, or null.
    //
    // `maxInFlight` is 1 for serial, where Modbus permits exactly one
    // outstanding transaction and there is no transaction identifier to tell
    // two apart. TCP may raise it; many TCP devices still cannot cope, which is
    // why it stays opt-in.
    ModbusPendingOp* nextReady(mb_time_t now, size_t maxInFlight, mb_time_t retryDelayMs);

    // Marks an operation transmitted.
    void markSent(ModbusPendingOp& op, mb_time_t now);

    // Concludes a broadcast, which has no answer to wait for.
    void completeBroadcast(ModbusPendingOp& op);

    // Applies a response to whichever operation it answers.
    //
    // Returns true when one matched. False is NORMAL and not an error: a reply
    // that arrives after its own timeout, while the next request is already
    // out, must be discarded rather than matched to the new one. On a serial
    // line that late reply is the classic corruption, and matching it would
    // hand the application one register's value under another's name.
    // `hasTransactionId` says whether the framing carried one. Where it did,
    // an identifier that matches nothing outstanding means the frame belongs to
    // nobody, and it is DISCARDED rather than matched positionally - falling
    // back would defeat the entire purpose of carrying the identifier, and
    // would accept a response to a request that had already timed out.
    bool applyResponse(uint8_t unit, uint16_t transactionId, bool hasTransactionId,
                       const uint8_t* pdu, size_t pduLen, mb_time_t now,
                       ModbusEstimator* roundTrip);

    // Retires operations that have waited too long, retrying those with budget
    // left.
    //
    // `timeoutMs` is the per-unit budget for the device to THINK. The time the
    // MEDIUM needs is added per operation from `bitsPerSecond`, because a one
    // register read and a 125 register read are the same request and very
    // different amounts of airtime - judging both against one figure declares
    // the second failed while it is still arriving. Zero for a socket, which
    // has no line rate and needs none of this.
    //
    // `onlyUnit` limits the sweep to one unit's operations, so each is judged
    // against its own device's measured round trip. -1 means every unit.
    //
    // `skipUnits`, when given, is a 256-bit set (eight words) of units the
    // sweep leaves alone: the client's last pass covers only units that have
    // no timing of their own, and a unit judged once against its own timeout
    // must not be judged again against the cold-start one. Before this the
    // last pass took every operation, so a device given a long timeout (a
    // radio link) was cut off at the cold-start figure all the same.
    void expire(mb_time_t now, mb_time_t timeoutMs, uint8_t maxRetries,
                uint32_t bitsPerSecond = 0, int16_t onlyUnit = -1,
                const uint32_t* skipUnits = nullptr);

    // Concludes everything, telling each caller why.
    void cancelAll(ModbusResult why);

    // Concludes everything for one unit.
    void cancelUnit(uint8_t unit, ModbusResult why);

    // --- Inspection ---

    size_t pendingCount() const { return _count; }

    // Whether this node has ever asked anybody for anything.
    //
    // Not a statistic. It decides how the RTU codec frames: a node that only
    // ever ANSWERS can predict a frame's length from its function code, and one
    // that also asks cannot, because a request and a response with the same
    // function code are different lengths.
    bool hasIssued() const { return _issued != 0; }
    uint32_t issuedCount() const { return _issued; }

    // How many concluded results are being held for status(), and how many
    // were aged out. Diagnostic: both are bounded by construction.
    size_t finishedCount() const { return _finishedCount; }
    uint32_t finishedDropped() const { return _finishedDropped; }

    // Conclusions still owed from a previous dispatch, and how often a dispatch
    // has had to carry any. Both stay at zero for an ordinary application.
    size_t deferredCount() const { return _deferred.count; }
    uint32_t deferredCarried() const { return _deferredCarried; }

    // Operations that were never transmitted at all, because the medium would
    // not take them. Distinct from a timeout, which means it went out and
    // nothing came back - and the difference is transport versus device.
    uint32_t neverSent() const { return _neverSent; }
    size_t inFlightCount() const;
    bool empty() const { return _count == 0; }
    bool anyLocalPending() const { return _local.count != 0; }

    // Where a caller's request has got to. Pending while it is in the queue,
    // then the outcome until the next request for the same address replaces it.
    ModbusRequestState state(uint8_t unit, uint16_t address, ModbusResult& result) const;

    // Records the outcome of an operation that never entered the queue, so a
    // caller polling state() sees a local address finish like any other.
    void noteFinished(uint8_t unit, uint16_t address, ModbusResult result);

    // Next operation id. Wraps, which is harmless: the queue holds far fewer
    // entries than the id space, so an id cannot be reused while its previous
    // owner is still outstanding.
    uint16_t nextOpId() { return ++_nextOpId; }

    // Takes the storage now rather than at the first request, so an owner can
    // say at start-up that the board cannot hold it. False if it cannot.
    bool reserve();

private:
    // --- Storage ---
    bool insert(const ModbusPendingOp& op, const uint8_t* payload, uint16_t payloadLen);
    bool setPayload(ModbusPendingOp& op, const uint8_t* payload, uint16_t payloadLen);
    void dropPayload(ModbusPendingOp& op);
    void removeAt(size_t index);         // frees the slot and closes the order up
    size_t positionOf(const ModbusPendingOp* op) const;
    size_t byteRoom() const { return _payloadSpace < _bytesAllocated ? _payloadSpace : _bytesAllocated; }
    ModbusPendingOp& at(size_t i) { return _slots[_order[i]]; }
    const ModbusPendingOp& at(size_t i) const { return _slots[_order[i]]; }

    ModbusPendingOp* _slots = nullptr;   // _allocated records, in no order
    bool* _inUse = nullptr;
    uint16_t* _order = nullptr;          // slot numbers, oldest first
    size_t _count = 0;
    size_t _allocated = 0;
    uint8_t* _bytes = nullptr;           // payloads, packed in queue order
    size_t _bytesAllocated = 0;
    size_t _bytesUsed = 0;
    size_t _capacity = MODBUS_QUEUE_CAPACITY;
    size_t _payloadSpace = MODBUS_PAYLOAD_SPACE;
    uint16_t _nextOpId = 0;

    // --- Conclusion ---

    // Removes the operation at `index` and THEN reports `result`. Safe for the
    // handler to queue anything it likes.
    void retireAt(size_t index, ModbusResult result);

    // Reports one outcome, and keeps the reporting flat.
    //
    // Removing the record before the handler runs makes a handler free to queue
    // work, but it does not bound the DEPTH: a handler told its write was
    // superseded that writes the same address again supersedes the write that
    // replaced it, which calls the handler again, and the chain runs until the
    // stack is gone. So a conclusion reached while a handler is already running
    // is queued rather than nested, and drained by the outermost call. Every
    // handler then runs at the same depth, whatever it does.
    // One conclusion owed.
    struct Owed {
        ModbusCompletionHandler done;
        ModbusResult result = ModbusResult::Ok;
    };
    void dispatch(Owed owed);

    // Conclusions owed, oldest first. Sized with the queue. Should one ever
    // fill - only a handler that keeps concluding work from inside handlers
    // can do it - the conclusion is delivered at once rather than dropped:
    // exactly once matters more than the depth it runs at.
    struct OwedRing {
        Owed* items = nullptr;
        uint16_t size = 0, head = 0, count = 0;
        bool push(Owed&& owed);
        Owed pop();
    };

    // Runs conclusions carried over from a dispatch that hit its bound.
    void drainCarriedDeferrals();

    OwedRing _deferred;

    // One dispatch drains at most this many conclusions. Anything beyond waits
    // for the next poll() - never dropped, never run inside an unbounded loop.
    static const size_t kMaxDeferredPerDispatch = 64;
    uint32_t _deferredCarried = 0;
    uint8_t _dispatchDepth = 0;

    // Completions owed for operations served in place. Distinct from _deferred,
    // which exists to flatten a handler that concludes another operation while
    // it runs.
    OwedRing _local;

    // A refused passthrough's handler, held until its conclusion is
    // dispatched. Owed entries carry only completion handlers, so the one owed
    // for a passthrough calls back into here.
    struct HeldPassthrough {
        ModbusPassthroughHandler pass;
        bool used = false;
    };
    static const size_t kHeldPassthroughs = 4;
    HeldPassthrough _held[kHeldPassthroughs];
    static void deliverHeld(void* self, uintptr_t index, ModbusResult result);

    // One retained outcome per (unit, address), kept after the operation is
    // retired so a caller that only looks once a scan still sees it. Queueing a
    // new operation for the same address drops the old outcome, so what is here
    // is never from a request the caller has already moved past.
    struct Finished {
        uint8_t unit;
        uint16_t address;
        ModbusResult result;
    };
    void dropFinished(uint8_t unit, uint16_t address);

    // The cap, and how many conclusions have been aged out of the table
    // because a caller was not reading them. A rising count means status() is
    // being relied on for more points than it holds - not an error, but worth
    // being able to see.
    static const size_t kMaxFinished = 64;
    Finished _finished[kMaxFinished];
    size_t _finishedCount = 0;
    uint32_t _finishedDropped = 0;

    // Operations concluded as LinkDown because the medium never accepted them.
    // Zero on a healthy link; a rising count means the transport, not the peer.
    uint32_t _neverSent = 0;

    // Operations accepted into the queue over this node's life.
    uint32_t _issued = 0;

    // By position in the queue order.
    size_t indexByOpId(uint8_t unit, uint16_t opId) const;
    size_t indexInFlight(uint8_t unit) const;
    static const size_t NO_OP = (size_t)-1;

    // Unpacks a response into the caller's memory.
    bool unpack(const ModbusPendingOp& op, const ModbusResponse& response);
};
