#include "ModbusLink.h"

#include <string.h>
#include "ModbusNoExcept.h"

ModbusLink::ModbusLink(ModbusTransport& transport, ModbusCodec& codec) noexcept
    : _transport(&transport), _codec(&codec) {}

ModbusLink::ModbusLink() noexcept {}

void ModbusLink::attach(ModbusTransport& transport, ModbusCodec& codec) {
    _transport = &transport;
    _codec = &codec;
}

bool ModbusLink::begin() {
    if (!attached()) return false;
    if (!_transport->begin()) return false;

    // Any half-frame from a previous session belongs to nothing. Keeping it
    // would prepend it to the first frame of this one.
    _codec->reset();
    _echoOwed = 0;
    _lastByteUs = mb_micros();
    _lastPollUs = 0;
    _open = true;
    return true;
}

void ModbusLink::end() {
    if (!attached()) return;
    _transport->end();
    _codec->reset();
    _open = false;
}

bool ModbusLink::isUp() const {
    return _open && _transport && _transport->isConnected();
}

mb_time_t ModbusLink::silenceUs() const {
    return mb_elapsed(_lastByteUs, mb_micros());
}

bool ModbusLink::readyToSend() const {
    if (!isUp()) return false;

    // The gap the codec asks for, raised by anything the transport needs for
    // its own turnaround - an RS-485 transceiver that is slow to switch, say.
    // A floor, never a replacement: whichever is longer wins.
    uint32_t gap = _codec->txGapUs();
    const uint32_t turnaround = _transport->turnaroundUs();
    if (turnaround > gap) gap = turnaround;

    if (gap == 0) return true;
    return silenceUs() >= gap;
}

// =============================================================================
// RECEIVING
// =============================================================================

void ModbusLink::CountingSink::onAdu(const ModbusAdu& adu) {
    _link._stats.framesReceived++;
    _downstream.onAdu(adu);
}

void ModbusLink::CountingSink::onFrameError(ModbusFrameError error, size_t bytes) {
    switch (error) {
        case ModbusFrameError::Checksum: _link._stats.checksumErrors++; break;
        case ModbusFrameError::Framing:  _link._stats.framingErrors++;  break;
        case ModbusFrameError::Overrun:  _link._stats.overruns++;       break;
        case ModbusFrameError::Protocol: _link._stats.protocolErrors++; break;
        case ModbusFrameError::Noise:    _link._stats.noiseBytes += (uint32_t)bytes; break;
    }
    _downstream.onFrameError(error, bytes);
}

void ModbusLink::poll(ModbusAduSink& sink) {
    const mb_time_t nowUs = mb_micros();

    // How long this node went between looks at the link. A timeout shorter than
    // that interval cannot be enforced however fast the device is, so this is
    // what floors every derived timeout.
    if (_lastPollUs != 0) {
        const mb_time_t delta = mb_elapsed(_lastPollUs, nowUs);
        // A stall - a blocking write, a long application scan - is not a service
        // interval, and folding one in would inflate the floor permanently.
        if (delta < 1000000u) {
            _serviceUs = _serviceUs == 0 ? delta : (_serviceUs * 3 + delta) / 4;
        }
    }
    _lastPollUs = nowUs;

    if (!_open || !attached()) return;

    CountingSink counting(*this, sink);

    size_t taken = 0;
    while (taken < _maxBytesPerPoll) {
        const int available = _transport->available();
        if (available <= 0) break;

        size_t want = _maxBytesPerPoll - taken;
        if (want > sizeof _rxChunk) want = sizeof _rxChunk;

        size_t got = 0;
        while (got < want) {
            const int byte = _transport->read();
            if (byte < 0) break;
            _rxChunk[got++] = (uint8_t)byte;
        }
        if (got == 0) break;

        trace(false, _rxChunk, got);
        _lastByteUs = mb_micros();

        // Our own transmission coming back, on a medium that loops. Discarded
        // by COUNT before the codec sees a byte of it, so a write response -
        // which is byte-for-byte identical to the request that caused it - is
        // never mistaken for one.
        size_t offset = 0;
        if (_echoOwed) {
            offset = _echoOwed < got ? _echoOwed : got;
            _echoOwed -= offset;
            _stats.echoesSuppressed += (uint32_t)offset;
            if (offset == got) { taken += got; continue; }
        }

        // Fed in full. A codec that takes fewer than it was given has a full
        // assembly buffer, and the rest is offered again next poll rather than
        // dropped - dropping it would lose bytes that were already safely off
        // the wire.
        size_t fed = offset;
        while (fed < got) {
            const size_t ate = _codec->feed(_rxChunk + fed, got - fed, _lastByteUs, counting);
            if (ate == 0) break;
            fed += ate;
        }

        taken += got;
        if (got < want) break;
    }

    // Silence is what ends a frame under RTU gap framing, and what triggers
    // length framing's recovery. Neither produces bytes to feed, so the codec
    // is told the time even when nothing arrived.
    _codec->tick(mb_micros(), counting);
}

// =============================================================================
// SENDING
// =============================================================================

bool ModbusLink::send(const ModbusAdu& adu) {
    if (!isUp()) { _stats.sendFailures++; return false; }

    uint8_t frame[MB_ASCII_ADU_MAX];
    const size_t length = _codec->encode(frame, sizeof frame, adu);
    if (length == 0) { _stats.sendFailures++; return false; }

    trace(true, frame, length);

    const size_t written = _transport->write(frame, length);
    if (written != length) {
        // A partial frame is worse than none: the far end reads it as a corrupt
        // whole rather than as a fragment, and answers with silence that looks
        // exactly like a device that is not there.
        _stats.sendFailures++;
        return false;
    }

    // A medium that loops will hand these bytes straight back. Counted so they
    // can be discarded on arrival rather than decoded.
    if (_transport->echoesTransmissions()) _echoOwed += length;

    _stats.framesSent++;

    // The gap before the NEXT transmission is measured from the end of this
    // one, so our own frame counts as activity on the line exactly as a
    // device's would.
    _lastByteUs = mb_micros();
    return true;
}
