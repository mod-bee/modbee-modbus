#pragma once

#include "ModbusTypes.h"
#include "ModbusTiming.h"

// =============================================================================
// THE ADU CODEC
// =============================================================================
//
// Framing, addressing and checksum: everything between a PDU and the wire.
// Three implementations, one interface.
//
//              ADU                                 Max   Delimited by
//   RTU    [addr][PDU][CRC lo hi]                  256   t3.5 silence
//   ASCII  ':' hex(addr, PDU, LRC) CR LF           513   ':' and CRLF
//   TCP    [txn][proto=0][len][unit][PDU]          260   the MBAP length field
//
// A codec owns its own assembly buffer and its own partial state. One codec
// belongs to one link; two links never share one.

// One ADU, decoded or about to be encoded.
//
// `pdu` points into the codec's assembly buffer on the way in, and into the
// caller's buffer on the way out. Either way it is a VIEW: valid only for the
// duration of the call that produced it.
struct ModbusAdu {
    // Serial address, or the TCP unit identifier. Both are one byte and both
    // say the same thing - which device on the far side this is for - so they
    // are one field.
    uint8_t unit = 0;

    // MBAP transaction identifier, and whether the framing carries one at all.
    //
    // The FLAG matters as much as the value. A TCP response is matched to its
    // request BY this identifier, so one that matches nothing outstanding
    // belongs to nobody - and falling back to "the request we sent last"
    // defeats the entire purpose of carrying it. A serial ADU has no such
    // field, which is precisely why a serial client may have only one request
    // outstanding at a time.
    //
    // Zero is a legal identifier, so the value alone cannot say whether the
    // framing carries one.
    uint16_t transactionId = 0;
    bool hasTransactionId = false;

    const uint8_t* pdu = nullptr;
    uint16_t pduLen = 0;
};

// Why bytes were discarded. Reported rather than swallowed: sustained growth
// in any of these is the difference between "the device is not answering" and
// "the line is wrong", and an integrator cannot tell those apart without the
// counts.
enum class ModbusFrameError : uint8_t {
    Checksum = 0,     // CRC or LRC failed - noise, or a baud rate mismatch
    Framing,          // Structurally impossible: a length that cannot be right
    Overrun,          // Longer than the largest ADU this framing allows
    Noise,            // Bytes discarded while hunting for a frame start
    Protocol,         // MBAP protocol id was not 0, or similar
};

const char* mb_frameErrorName(ModbusFrameError error);

// What the codec should expect to decode.
//
// Length-prediction framing has to know: a five-byte request and a five-byte
// response are different shapes under the same function code, and a framer that
// guesses from structure alone reads a request for 256 to 511 coils as a
// response and never executes it.
enum class ModbusExpect : uint8_t {
    Request = 0,   // We are a server: what arrives is a request
    Response,      // We are a client: what arrives answers our request
    Either,        // A listener or a sniffer. Gap framing only - see below.
};

// Where completed ADUs and discarded bytes are reported.
//
// An interface rather than a std::function: this is called once per frame on
// every link, and a std::function here would allocate on a path that must not.
class ModbusAduSink {
public:
    virtual ~ModbusAduSink() = default;

    // A complete, checksum-verified ADU. `adu.pdu` is valid only until this
    // call returns.
    virtual void onAdu(const ModbusAdu& adu) = 0;

    // Bytes the codec threw away, and why.
    virtual void onFrameError(ModbusFrameError error, size_t bytes) {
        (void)error; (void)bytes;
    }
};

// How an RTU codec decides where a frame ends. Both ship; see ModbusRtuCodec.
enum class ModbusFraming : uint8_t {
    Auto = 0,    // Choose from the line rate and the measured service interval
    Gap,         // The specification's own: t3.5 of silence
    Length,      // Decode the function code, compute the length, read that many
};

const char* mb_framingName(ModbusFraming framing);

class ModbusCodec {
public:
    virtual ~ModbusCodec() = default;

    // Wraps a PDU into an ADU ready for the wire. Returns bytes written, or 0
    // if it will not fit. Same contract as the PDU encoders: nothing is ever
    // written past `capacity`, and on a refusal the buffer is unspecified.
    virtual size_t encode(uint8_t* out, size_t capacity, const ModbusAdu& adu) = 0;

    // Feeds received bytes. Completed ADUs go to `sink` as they are recognised;
    // one call may produce several, or none.
    //
    // `nowUs` is passed in rather than read from the clock, so a codec stays
    // deterministic and its timing behaviour can be tested without waiting for
    // real microseconds to pass.
    //
    // Returns how many bytes were consumed. Fewer than `length` means the
    // assembly buffer is full and the caller should feed the rest next time
    // rather than drop it.
    virtual size_t feed(const uint8_t* in, size_t length, mb_time_t nowUs,
                        ModbusAduSink& sink) = 0;

    // Advances time with no new bytes. RTU gap framing needs this: silence is
    // what ends a frame, and silence produces no bytes to feed.
    virtual void tick(mb_time_t nowUs, ModbusAduSink& sink) { (void)nowUs; (void)sink; }

    // Throws away any partial frame. Called when a link opens or reconnects,
    // so half an old frame is never prepended to a new one.
    virtual void reset() = 0;

    // Says what shape to expect. Must be set before the first feed() on a
    // length-framed link.
    virtual void expect(ModbusExpect what) = 0;

    // What it is currently expecting. Diagnostic, and the answer to "why is
    // this node using that framing" - on RTU the two are the same question.
    virtual ModbusExpect expecting() const = 0;

    // Silence this codec needs before transmitting, in microseconds. t3.5 for
    // RTU; zero for ASCII and TCP, which are self-delimiting.
    virtual uint32_t txGapUs() const { return 0; }

    // The largest ADU this framing can carry.
    virtual size_t maxAduBytes() const = 0;

    // A short name, for logs and for diagnostics that have to be read by a
    // human at three in the morning.
    virtual const char* name() const = 0;
};
