#pragma once

#include "../ModbusTransport.h"

#if defined(MODBUS_PLATFORM_HOST)

#include <string>
#include <vector>

// =============================================================================
// ONE TCP CONNECTION
// =============================================================================
//
// A client's connection to a server, or a server's connection to one client.
// Both are the same thing: a socket carrying bytes.
//
// Non-blocking throughout. A connect() in progress reports isConnected() false
// until it completes, so a client can poll rather than stall - which matters,
// because a TCP connect to an absent device takes the operating system's own
// timeout to fail, and that is measured in seconds an application loop cannot
// spare.
class ModbusTcpTransport : public ModbusTransport {
public:
    typedef intptr_t socket_t;
    static const socket_t INVALID_SOCKET_HANDLE = -1;

    // A client connection, opened by begin().
    ModbusTcpTransport(const char* host, uint16_t port = MODBUS_TCP_PORT);

    // An accepted connection, already open. Used by ModbusTcpServer.
    explicit ModbusTcpTransport(socket_t accepted);

    ~ModbusTcpTransport() override;

    ModbusTcpTransport(const ModbusTcpTransport&) = delete;
    ModbusTcpTransport& operator=(const ModbusTcpTransport&) = delete;

    bool begin() override;
    void end() override;
    int available() override;
    int read() override;
    size_t write(const uint8_t* buffer, size_t length) override;
    bool isConnected() const override;

    // Zero: a socket has no line rate, and saying otherwise would make the RTU
    // codec compute character times for a medium that has none. A TCP link
    // needs no such timing anyway, because MBAP carries its own length.
    uint32_t bitsPerSecond() const override { return 0; }

    // A socket does not hand our own transmissions back.
    bool echoesTransmissions() const override { return false; }

    // Whether a connect() is still in progress. Distinct from "not connected":
    // one is worth waiting for and the other is worth retrying.
    bool isConnecting() const { return _connecting; }

    const char* lastError() const { return _lastError.c_str(); }
    const char* peer() const { return _peer.c_str(); }

    // Sends that gave up because the peer stopped reading long enough to fill
    // its receive window. The connection is dropped when this happens: a peer
    // that has not taken a byte in a quarter of a second is gone, and holding
    // a half-written frame open serves nobody.
    uint32_t stalledSends() const { return _stalledSends; }

private:
    // How long to keep retrying a send the peer has no room for.
    //
    // Far longer than a healthy peer needs on a local network, and far shorter
    // than any client's own timeout - so a stall is reported as a link failure
    // the application can act on, rather than as a hang it cannot see.
    static const mb_time_t kSendBudgetMs = 250;

    bool fail(const char* what);
    void fillBuffer();

    std::string _host;
    uint16_t _port = MODBUS_TCP_PORT;
    socket_t _socket = INVALID_SOCKET_HANDLE;
    bool _connecting = false;
    bool _owned = true;

    // Read in blocks rather than a byte at a time. A syscall per byte would
    // dominate everything else this library does.
    std::vector<uint8_t> _rx;
    size_t _rxPos = 0;

    uint32_t _stalledSends = 0;

    std::string _lastError;
    std::string _peer;
};

// =============================================================================
// A TCP SERVER
// =============================================================================
//
// Accepts connections and hands each one out as a transport.
//
// This is the abstraction a serial link does not need and a TCP one cannot do
// without: a Modbus TCP server faces several clients at once, each with its own
// socket, its own transaction stream and its own lifecycle. One data map and
// one server engine serve all of them; the framing and the transaction state
// are per connection.
class ModbusTcpServer : public ModbusListener {
public:
    explicit ModbusTcpServer(uint16_t port = MODBUS_TCP_PORT,
                             const char* bindAddress = nullptr);
    ~ModbusTcpServer() override;

    ModbusTcpServer(const ModbusTcpServer&) = delete;
    ModbusTcpServer& operator=(const ModbusTcpServer&) = delete;

    bool begin() override;
    void end() override;
    ModbusTransport* accept() override;
    void release(ModbusTransport* transport) override;
    bool isListening() const override;

    // Connections accepted since begin(). A client that reconnects on every
    // transaction - some SCADA packages do - shows up here as a number that
    // climbs alarmingly, which is worth being able to see.
    uint32_t acceptedCount() const { return _accepted; }

    const char* lastError() const { return _lastError.c_str(); }
    uint16_t port() const { return _port; }

private:
    bool fail(const char* what);

    uint16_t _port;
    std::string _bindAddress;
    ModbusTcpTransport::socket_t _socket = ModbusTcpTransport::INVALID_SOCKET_HANDLE;
    uint32_t _accepted = 0;
    std::string _lastError;
};

#endif // MODBUS_PLATFORM_HOST
