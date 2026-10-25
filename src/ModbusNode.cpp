#include "ModbusNode.h"
#include "ModbusNoExcept.h"

ModbusNode::ModbusNode(ModbusTransport& transport, ModbusCodec& codec) noexcept
    : _link(transport, codec), _client(_link), _server(_link) {}

bool ModbusNode::begin() {
    return _link.begin();
}

void ModbusNode::end() {
    _client.cancelAll();
    _link.end();
}

void ModbusNode::enableClient(const ModbusClientConfig& config) {
    _client.begin(config);
    _clientEnabled = true;
    updateExpectation();
}

void ModbusNode::enableServer(const ModbusServerConfig& config) {
    _server.begin(config);
    _serverEnabled = true;
    updateExpectation();
}

void ModbusNode::updateExpectation() {
    // What the codec should expect to decode. RTU length framing needs this:
    // the same function code has different lengths in each direction, and a
    // framer that guesses reads a request for 256 to 511 coils as a response
    // and never executes it.
    //
    // A node in both roles hears both, so it must fall back to gap framing -
    // there is no table that covers both directions at once.
    if (_clientEnabled && _serverEnabled) {
        _link.codec().expect(ModbusExpect::Either);
    } else if (_clientEnabled) {
        _link.codec().expect(ModbusExpect::Response);
    } else if (_serverEnabled) {
        _link.codec().expect(ModbusExpect::Request);
    }
}

void ModbusNode::poll() {
    // Incoming first: everything below decides on what it produced.
    _link.poll(*this);

    // Then the two halves. Neither transmits from inside the receive path: on a
    // blocking transport a write holds the caller for the whole airtime, and
    // anything that had already read the clock would be working from a reading
    // the write itself invalidated.
    if (_serverEnabled) _server.poll();
    if (_clientEnabled) _client.poll();
}

void ModbusNode::onAdu(const ModbusAdu& adu) {
    // The client first. It can say DEFINITIVELY whether this frame answers a
    // request it has outstanding, where the server can only say whether the
    // unit id looks like ours - and on a two-master segment those overlap.
    if (_clientEnabled && _client.handleAdu(adu)) return;

    if (_serverEnabled && _server.handleAdu(adu)) return;

    // Neither ours to answer nor an answer to us. On a multi-master segment
    // this is the other master's traffic and is entirely healthy; on a
    // single-master one, growth here means something unexpected is talking.
    _forOthers++;
}

void ModbusNode::onFrameError(ModbusFrameError error, size_t bytes) {
    if (!_eventHandler) return;

    switch (error) {
        case ModbusFrameError::Checksum:
            _eventHandler(_eventContext, ModbusEvent::ChecksumError, 0,
                          "a frame arrived corrupt");
            break;
        case ModbusFrameError::Noise:
            // Reported once for the run rather than per byte, and only when
            // there is enough of it to mean something: a floating RS-485 pair
            // produces this continuously, and an event per byte would bury
            // everything else.
            if (bytes >= 8) {
                _eventHandler(_eventContext, ModbusEvent::NoiseDiscarded, 0,
                              "bytes discarded while hunting for a frame");
            }
            break;
        default:
            _eventHandler(_eventContext, ModbusEvent::FramingError, 0,
                          mb_frameErrorName(error));
            break;
    }
}
