#pragma once

#include <new>

#include "../ModbusTransport.h"

#if defined(MODBUS_PLATFORM_ARDUINO)

#include <Arduino.h>
#include <Stream.h>

// =============================================================================
// ARDUINO SERIAL
// =============================================================================
//
// Wraps any Stream: HardwareSerial for RS-485, SoftwareSerial, or anything else
// implementing the interface.
//
// The sketch opens the Stream itself, because baud rate and pin assignment are
// board specific and this class has no business guessing either.

class ModbusStreamTransport : public ModbusTransport {
public:
    // The BAUD RATE is the one thing about the line that cannot be worked out
    // from the bus, and Modbus RTU framing is defined in character times.
    //
    // The sketch has already passed it to Serial.begin(), so pass it here too
    // and the node stops guessing: t1.5, t3.5 and the gap before transmitting
    // all come off it. Left at zero, the RTU codec falls back to length framing
    // and says so.
    explicit ModbusStreamTransport(Stream* stream = nullptr, uint32_t baudRate = 0) noexcept
        : _stream(stream), _baudRate(baudRate) {}

    // Set after construction, so an object holding this as a member can be
    // declared before the sketch knows which Stream it will use.
    void setStream(Stream* stream, uint32_t baudRate) {
        _stream = stream;
        _baudRate = baudRate;
    }

    // --- RS-485 direction control ---
    //
    // A half-duplex transceiver needs its driver enabled while transmitting and
    // disabled the rest of the time, or it holds the bus and nothing else can
    // speak.
    //
    // `holdUs` is the delay after the last byte before the driver is released.
    // It exists because Stream::write() returns when the byte reaches the UART,
    // not when it leaves the wire: releasing immediately cuts the final
    // character in half, and the far end reports a checksum error that nothing
    // at either end points at the transceiver. One character time is the
    // minimum; the default is two, which costs nothing.
    void setDirectionPin(int pin, bool activeHigh = true, uint32_t holdUs = 0) {
        _directionPin = pin;
        _directionActive = activeHigh;
        _holdUs = holdUs;
        if (_directionPin >= 0) {
            pinMode(_directionPin, OUTPUT);
            digitalWrite(_directionPin, _directionActive ? LOW : HIGH);
        }
    }

    bool begin() override { return _stream != nullptr; }
    void end() override { _stream = nullptr; }

    int available() override { return _stream ? _stream->available() : 0; }
    int read() override { return _stream ? _stream->read() : -1; }

    size_t write(const uint8_t* buffer, size_t length) override {
        if (!_stream) return 0;

        if (_directionPin >= 0) {
            digitalWrite(_directionPin, _directionActive ? HIGH : LOW);
        }

        const size_t written = _stream->write(buffer, length);

        if (_directionPin >= 0) {
            // flush() waits for the transmit buffer to empty. On most cores
            // that means the last byte has left the shift register; where it
            // does not, holdUs covers the difference.
            _stream->flush();
            if (_holdUs) delayMicroseconds(_holdUs);
            digitalWrite(_directionPin, _directionActive ? LOW : HIGH);
        }
        return written;
    }

    bool isConnected() const override { return _stream != nullptr; }
    uint32_t bitsPerSecond() const override { return _baudRate; }

    // A transceiver with direction control does not hand our own transmission
    // back, because its receiver is off while it drives. One without does.
    bool echoesTransmissions() const override { return _echoes; }
    void setEchoes(bool echoes) { _echoes = echoes; }

private:
    Stream* _stream;
    uint32_t _baudRate;
    int _directionPin = -1;
    bool _directionActive = true;
    uint32_t _holdUs = 0;
    bool _echoes = false;
};

// =============================================================================
// ARDUINO TCP
// =============================================================================
//
// Templates, because the Arduino networking classes share no base: WiFiClient,
// EthernetClient and WiFiClientSecure are unrelated types with the same shape.
// A template binds to whichever the sketch already has, with no virtual
// dispatch and no library dependency on any particular core.

template <typename ClientType>
class ModbusArduinoClientTransport : public ModbusTransport {
public:
    // Wraps a client the sketch owns and has already connected, or will.
    explicit ModbusArduinoClientTransport(ClientType& client) noexcept : _client(client) {}

    // Connects on begin(), and reconnects when the session asks it to.
    ModbusArduinoClientTransport(ClientType& client, const char* host, uint16_t port) noexcept
        : _client(client), _host(host), _port(port) {}

    bool begin() override {
        if (!_host) return _client.connected();
        if (_client.connected()) return true;
        return _client.connect(_host, _port) == 1;
    }

    void end() override { _client.stop(); }
    int available() override { return _client.available(); }
    int read() override { return _client.read(); }

    size_t write(const uint8_t* buffer, size_t length) override {
        return _client.write(buffer, length);
    }

    bool isConnected() const override {
        return const_cast<ClientType&>(_client).connected();
    }

    // A socket has no line rate, and saying otherwise would make the RTU codec
    // compute character times for a medium that has none.
    uint32_t bitsPerSecond() const override { return 0; }

private:
    ClientType& _client;
    const char* _host = nullptr;
    uint16_t _port = 0;
};

// One accepted connection, owned by ModbusArduinoServer below.
template <typename ClientType>
class ModbusArduinoAcceptedTransport : public ModbusTransport {
public:
    ModbusArduinoAcceptedTransport() noexcept {}
    explicit ModbusArduinoAcceptedTransport(ClientType client) noexcept : _client(client) {}

    // Takes the next connection into this slot; an empty client lets go.
    void adopt(ClientType client) { _client = client; }

    bool begin() override { return _client.connected(); }
    void end() override { _client.stop(); }
    int available() override { return _client.available(); }
    int read() override { return _client.read(); }
    size_t write(const uint8_t* buffer, size_t length) override {
        return _client.write(buffer, length);
    }
    bool isConnected() const override {
        return const_cast<ClientType&>(_client).connected();
    }
    uint32_t bitsPerSecond() const override { return 0; }

private:
    ClientType _client;   // By value: the Arduino client classes are handles
};

// Accepts connections from a WiFiServer or EthernetServer.
//
// Bounded on purpose: an MCU has kilobytes, and each connection costs a codec
// and an assembly buffer. Past the limit a client is accepted and immediately
// stopped, which is a far better answer than a silent queue the device can
// never work through.
//
// The limit is the caller's, whatever it is: a slot per connection is taken
// once, here, so accepting one later costs no allocation. ready() is false
// when the board could not hold that many - say so and do not listen, because
// a server quietly holding fewer clients than it was told is the failure this
// avoids. Two when the caller says nothing.
template <typename ServerType, typename ClientType>
class ModbusArduinoServer : public ModbusListener {
public:
    static const size_t DEFAULT_CONNECTIONS = 2;

    explicit ModbusArduinoServer(ServerType& server, size_t maxConnections = DEFAULT_CONNECTIONS)
        : _server(server),
          _maxConnections(maxConnections == 0 ? DEFAULT_CONNECTIONS : maxConnections) {
        _slot = new (std::nothrow) ModbusArduinoAcceptedTransport<ClientType>[_maxConnections];
        _used = new (std::nothrow) bool[_maxConnections]();
        if (!_slot || !_used) {
            delete[] _slot; delete[] _used;
            _slot = nullptr; _used = nullptr;
        }
    }
    ~ModbusArduinoServer() override { delete[] _slot; delete[] _used; }
    ModbusArduinoServer(const ModbusArduinoServer&) = delete;
    ModbusArduinoServer& operator=(const ModbusArduinoServer&) = delete;

    // Whether the slots for every connection asked for were found.
    bool ready() const { return _slot != nullptr; }
    size_t maxConnections() const { return _maxConnections; }

    bool begin() override {
        if (!ready()) return false;
        _server.begin(); _listening = true; return true;
    }
    void end() override { _listening = false; }
    bool isListening() const override { return _listening; }

    ModbusTransport* accept() override {
        if (!_listening) return nullptr;

        ClientType client = _server.available();
        if (!client) return nullptr;

        if (_live >= _maxConnections) {
            client.stop();
            _refused++;
            return nullptr;
        }

        // A slot of this listener's own: a connection costs no allocation.
        for (size_t i = 0; i < _maxConnections; i++) {
            if (_used[i]) continue;
            _used[i] = true;
            _live++;
            _slot[i].adopt(client);
            return &_slot[i];
        }
        client.stop();
        _refused++;
        return nullptr;
    }

    void release(ModbusTransport* transport) override {
        if (!_slot) return;
        for (size_t i = 0; i < _maxConnections; i++) {
            if (!_used[i] || &_slot[i] != transport) continue;
            _slot[i].adopt(ClientType());
            _used[i] = false;
            _live--;
            return;
        }
    }

    uint32_t refusedCount() const { return _refused; }

private:
    ServerType& _server;

    // NOT _max. The ESP32 core defines _max as a macro in Arduino.h, so a
    // member of that name is macro-expanded at every use and the class does
    // not compile at all on the board it exists for.
    size_t _maxConnections;
    bool _listening = false;
    ModbusArduinoAcceptedTransport<ClientType>* _slot = nullptr;
    bool* _used = nullptr;
    size_t _live = 0;
    uint32_t _refused = 0;
};

#endif // MODBUS_PLATFORM_ARDUINO
