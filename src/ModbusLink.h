#pragma once

#include "ModbusCodec.h"
#include "ModbusTransport.h"

// =============================================================================
// ONE LINK
// =============================================================================
//
// A transport, a codec, and the bookkeeping between them.
//
// The hardening here is not speculative. Each guard below exists because a real
// line does the thing it guards against: a flooded node, a babbling peer, a
// floating pair delivering noise, a transport that loops its own output back.
//
// One link is one channel. A serial port is one link for the life of the node;
// a TCP server has one per accepted socket.

struct ModbusLinkStats {
    uint32_t framesReceived = 0;
    uint32_t framesSent = 0;
    uint32_t sendFailures = 0;

    // Why bytes were thrown away. First-class counters rather than a log line,
    // because the DIFFERENCE between these is what tells an integrator whether
    // the device is absent, the wiring is wrong or the baud rate is wrong -
    // and none of those looks different from the application's point of view.
    uint32_t checksumErrors = 0;   // Noise, or a baud rate mismatch
    uint32_t framingErrors = 0;    // Structurally impossible frames
    uint32_t overruns = 0;         // Longer than any valid ADU
    uint32_t noiseBytes = 0;       // Discarded while hunting for a frame start
    uint32_t protocolErrors = 0;   // MBAP protocol id wrong, and the like

    // Times this node's own transmission came back at it. Normal on a socket,
    // a hardware loopback or a repeater; unusual on half-duplex RS-485. Counted
    // rather than merely suppressed, because a sudden appearance of echo means
    // the wiring changed.
    uint32_t echoesSuppressed = 0;

    void reset() { *this = ModbusLinkStats(); }

    // Adds another link's counters to these.
    //
    // A serial node is one link for its whole life, but a TCP server is one
    // link per connection, each born and buried as clients come and go. Its
    // totals are only meaningful summed - over the connections open now AND the
    // ones already closed - so the counters have to be addable.
    void add(const ModbusLinkStats& other) {
        framesReceived += other.framesReceived;
        framesSent += other.framesSent;
        sendFailures += other.sendFailures;
        checksumErrors += other.checksumErrors;
        framingErrors += other.framingErrors;
        overruns += other.overruns;
        noiseBytes += other.noiseBytes;
        protocolErrors += other.protocolErrors;
        echoesSuppressed += other.echoesSuppressed;
    }
};

class ModbusLink {
public:
    ModbusLink(ModbusTransport& transport, ModbusCodec& codec) noexcept;

    // Deferred form, for a caller that owns its transport and codec as members
    // and therefore cannot hand them over at construction.
    //
    // This is what lets the one-object API (ModbusRTU, ModbusTCP, ModbusASCII)
    // hold everything by value with no heap allocation: the link is a member,
    // and begin() attaches whichever transport the application chose.
    ModbusLink() noexcept;
    void attach(ModbusTransport& transport, ModbusCodec& codec);
    bool attached() const { return _transport != nullptr && _codec != nullptr; }

    // Lets go of the transport without touching it - for a link kept for
    // reuse after its transport has been handed back and may be gone.
    void detach() {
        _transport = nullptr;
        _codec = nullptr;
        _open = false;
    }

    bool begin();
    void end();

    // Whether the medium is usable right now. A TCP link reports false once its
    // peer has gone, which is what tells a session to reconnect.
    bool isUp() const;

    // Only valid once attached. A link that has not been attached has no
    // transport to return a reference to, so asking is a programming error
    // rather than a runtime condition.
    ModbusTransport& transport() { return *_transport; }
    ModbusCodec& codec() { return *_codec; }

    // Drains the transport and reports whatever frames come out.
    //
    // Bounded: at most `maxBytesPerPoll` bytes per call, so a flooded link still
    // leaves the application's loop running. The surplus stays in the transport
    // and is taken next call rather than being dropped - a distinction that
    // matters, because dropping it loses frames that were already safely off
    // the wire.
    void poll(ModbusAduSink& sink);

    // Sends one ADU. False if it would not encode, the medium refused it, or
    // the transport took only part of it - a partial frame is worse than none,
    // since the far end reads it as a corrupt whole rather than as a fragment.
    bool send(const ModbusAdu& adu);

    // Whether the medium has been quiet long enough to transmit into. RTU's
    // t3.5 gate, and always true for the self-delimiting framings.
    //
    // Asked BEFORE committing state to a frame, so a caller never marks an
    // operation sent that then could not go out.
    bool readyToSend() const;

    // Microseconds since the last byte arrived, or since the link opened.
    mb_time_t silenceUs() const;

    // The interval the application actually calls poll() at, smoothed. This is
    // the floor under every derived timeout: a timeout shorter than the gap
    // between two looks at the link cannot be enforced however fast the device
    // answers, and an application polling every 50 ms while asking for a 20 ms
    // timeout would otherwise see phantom timeouts for ever.
    mb_time_t serviceIntervalUs() const { return _serviceUs; }
    mb_time_t serviceIntervalMs() const { return (_serviceUs + 999) / 1000; }

    const ModbusLinkStats& stats() const { return _stats; }
    void resetStats() { _stats.reset(); }

    // Bytes taken off the transport in one poll(). Bounds worst-case time in a
    // single call. Two ADUs' worth by default, which is enough that a burst is
    // not artificially spread over several polls.
    void setMaxBytesPerPoll(size_t bytes) { _maxBytesPerPoll = bytes ? bytes : 1; }

    // Raw byte tracing, both directions. What an integrator actually needs when
    // a device will not answer: not a decoded summary, but the bytes.
    typedef void (*TraceHandler)(void* context, bool transmitted,
                                 const uint8_t* bytes, size_t length);
    void onTrace(TraceHandler handler, void* context) {
        _trace = handler;
        _traceContext = context;
    }

private:
    // Counts what the codec discards, then passes everything on to the caller's
    // sink. Sitting between the two is what keeps the statistics in one place
    // rather than duplicated into every session.
    class CountingSink : public ModbusAduSink {
    public:
        CountingSink(ModbusLink& link, ModbusAduSink& downstream)
            : _link(link), _downstream(downstream) {}
        void onAdu(const ModbusAdu& adu) override;
        void onFrameError(ModbusFrameError error, size_t bytes) override;
    private:
        ModbusLink& _link;
        ModbusAduSink& _downstream;
    };
    friend class CountingSink;

    ModbusTransport* _transport = nullptr;
    ModbusCodec* _codec = nullptr;
    ModbusLinkStats _stats;

    size_t _maxBytesPerPoll = 2 * MB_TCP_ADU_MAX;
    uint8_t _rxChunk[256];

    mb_time_t _lastByteUs = 0;
    mb_time_t _lastPollUs = 0;
    mb_time_t _serviceUs = 0;
    bool _open = false;

    // Bytes still owed back to us by a medium that echoes, counted DOWN as
    // they arrive and discarded before the codec ever sees them.
    //
    // Counted, never matched on content. On Modbus a write response is a
    // byte-for-byte echo of the request that caused it - that is what "echo
    // response" means - so anything that recognises an echo by comparing bytes
    // throws away every write response it receives. The byte COUNT is exact
    // and unambiguous where the content is not.
    //
    // Only armed when the transport says the medium loops back, because only
    // the transport knows whether it does.
    size_t _echoOwed = 0;

    TraceHandler _trace = nullptr;
    void* _traceContext = nullptr;
    void trace(bool transmitted, const uint8_t* bytes, size_t length) const {
        if (_trace) _trace(_traceContext, transmitted, bytes, length);
    }
};
