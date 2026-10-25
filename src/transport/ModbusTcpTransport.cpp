#include "ModbusTcpTransport.h"

#if defined(MODBUS_PLATFORM_HOST)

#include <cstring>
#include <cstdio>

#if defined(_WIN32)
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef int socklen_t;
    #define MB_CLOSE_SOCKET closesocket
    #define MB_WOULD_BLOCK (WSAGetLastError() == WSAEWOULDBLOCK)
    #define MB_IN_PROGRESS (WSAGetLastError() == WSAEWOULDBLOCK)
    #define MB_SEND_FLAGS 0
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <netdb.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <sys/socket.h>
    #include <sys/types.h>
    #include <arpa/inet.h>
    #define MB_CLOSE_SOCKET ::close
    #define MB_WOULD_BLOCK (errno == EAGAIN || errno == EWOULDBLOCK)
    #define MB_IN_PROGRESS (errno == EINPROGRESS)
    // A send to a peer that has gone raises SIGPIPE, whose default action ends
    // the process. Refused per call where the platform allows it, and per
    // socket (SO_NOSIGPIPE, below) where it does not.
    #if defined(MSG_NOSIGNAL)
        #define MB_SEND_FLAGS MSG_NOSIGNAL
    #else
        #define MB_SEND_FLAGS 0
    #endif
#endif

namespace {

// Winsock needs starting once per process, and the count is kept by the library
// itself - so a reference count here only has to make sure it happens at all.
struct SocketLibrary {
    SocketLibrary() {
#if defined(_WIN32)
        WSADATA data;
        WSAStartup(MAKEWORD(2, 2), &data);
#endif
    }
    ~SocketLibrary() {
#if defined(_WIN32)
        WSACleanup();
#endif
    }
};

void ensureSocketLibrary() {
    static SocketLibrary library;
    (void)library;
}

bool setNonBlocking(intptr_t socket) {
#if defined(_WIN32)
    u_long mode = 1;
    return ioctlsocket((SOCKET)socket, FIONBIO, &mode) == 0;
#else
    const int flags = fcntl((int)socket, F_GETFL, 0);
    if (flags < 0) return false;
    return fcntl((int)socket, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
}

// Nagle's algorithm holds a small write back waiting for more to combine with
// it. A Modbus request IS a small write, and the thing it is waiting for is the
// response to the request it is holding - so leaving Nagle on adds up to 200 ms
// to every transaction, and looks exactly like a slow device.
void disableNagle(intptr_t socket) {
    int flag = 1;
    setsockopt((int)socket, IPPROTO_TCP, TCP_NODELAY, (const char*)&flag, sizeof flag);
#if defined(SO_NOSIGPIPE)
    setsockopt((int)socket, SOL_SOCKET, SO_NOSIGPIPE, (const char*)&flag, sizeof flag);
#endif
}

} // namespace

// =============================================================================
// CONNECTION
// =============================================================================

ModbusTcpTransport::ModbusTcpTransport(const char* host, uint16_t port)
    : _host(host ? host : ""), _port(port) {
    ensureSocketLibrary();
}

ModbusTcpTransport::ModbusTcpTransport(socket_t accepted)
    : _socket(accepted), _owned(true) {
    ensureSocketLibrary();
    setNonBlocking(accepted);
    disableNagle(accepted);

    sockaddr_in address;
    socklen_t length = sizeof address;
    if (getpeername((int)accepted, (sockaddr*)&address, &length) == 0) {
        char text[64];
        std::snprintf(text, sizeof text, "%s:%u",
                      inet_ntoa(address.sin_addr), (unsigned)ntohs(address.sin_port));
        _peer = text;
    }
}

ModbusTcpTransport::~ModbusTcpTransport() { end(); }

bool ModbusTcpTransport::fail(const char* what) {
    _lastError = what;
    end();
    return false;
}

bool ModbusTcpTransport::begin() {
    // An accepted socket is already open and has no host to connect to.
    if (_socket != INVALID_SOCKET_HANDLE && _host.empty()) return true;

    end();
    _lastError.clear();
    if (_host.empty()) return fail("no host specified");

    addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    char portText[8];
    std::snprintf(portText, sizeof portText, "%u", (unsigned)_port);

    addrinfo* results = nullptr;
    if (getaddrinfo(_host.c_str(), portText, &hints, &results) != 0 || !results) {
        return fail("could not resolve host");
    }

    bool opened = false;
    for (addrinfo* candidate = results; candidate; candidate = candidate->ai_next) {
        const intptr_t handle = (intptr_t)socket(candidate->ai_family,
                                                 candidate->ai_socktype,
                                                 candidate->ai_protocol);
        if (handle < 0) continue;

        setNonBlocking(handle);
        disableNagle(handle);

        const int result = ::connect((int)handle, candidate->ai_addr,
                                     (socklen_t)candidate->ai_addrlen);
        if (result == 0) {
            _socket = handle;
            _connecting = false;
            opened = true;
            break;
        }
        if (MB_IN_PROGRESS) {
            // Still going. Reported as connecting rather than connected, so the
            // application can poll instead of stalling - a TCP connect to an
            // absent device takes the operating system's own timeout to fail,
            // and that is seconds an application loop cannot spare.
            _socket = handle;
            _connecting = true;
            opened = true;
            break;
        }
        MB_CLOSE_SOCKET((int)handle);
    }

    freeaddrinfo(results);
    if (!opened) return fail("could not connect");

    _peer = _host + ":" + portText;
    _rx.clear();
    _rxPos = 0;
    return true;
}

void ModbusTcpTransport::end() {
    if (_socket != INVALID_SOCKET_HANDLE && _owned) {
        MB_CLOSE_SOCKET((int)_socket);
    }
    _socket = INVALID_SOCKET_HANDLE;
    _connecting = false;
    _rx.clear();
    _rxPos = 0;
}

bool ModbusTcpTransport::isConnected() const {
    if (_socket == INVALID_SOCKET_HANDLE) return false;
    if (!_connecting) return true;

    // A connect in progress completes when the socket becomes writable. Asked
    // rather than waited for, so this stays non-blocking.
    fd_set writable;
    FD_ZERO(&writable);
    FD_SET((int)_socket, &writable);
    timeval immediately = { 0, 0 };

    ModbusTcpTransport* self = const_cast<ModbusTcpTransport*>(this);
    if (select((int)_socket + 1, nullptr, &writable, nullptr, &immediately) <= 0) {
        return false;
    }

    int error = 0;
    socklen_t length = sizeof error;
    if (getsockopt((int)_socket, SOL_SOCKET, SO_ERROR, (char*)&error, &length) != 0 ||
        error != 0) {
        self->end();
        return false;
    }
    self->_connecting = false;
    return true;
}

void ModbusTcpTransport::fillBuffer() {
    if (_rxPos < _rx.size()) return;
    if (_socket == INVALID_SOCKET_HANDLE || _connecting) return;

    // Read in blocks. A syscall per byte would dominate everything else this
    // library does.
    uint8_t chunk[1024];
    const ssize_t got = ::recv((int)_socket, (char*)chunk, sizeof chunk, 0);

    if (got > 0) {
        _rx.assign(chunk, chunk + got);
        _rxPos = 0;
        return;
    }

    if (got == 0) {
        // The peer closed cleanly. Not an error, but the link is over.
        end();
        return;
    }

    if (!MB_WOULD_BLOCK) end();   // A real error; the link is over
}

int ModbusTcpTransport::available() {
    fillBuffer();
    return (int)(_rx.size() - _rxPos);
}

int ModbusTcpTransport::read() {
    fillBuffer();
    if (_rxPos >= _rx.size()) return -1;
    return _rx[_rxPos++];
}

size_t ModbusTcpTransport::write(const uint8_t* buffer, size_t length) {
    if (_socket == INVALID_SOCKET_HANDLE || _connecting || !buffer || length == 0) return 0;

    // A partial ADU is worse than none - the far end reads it as a corrupt
    // whole rather than as a fragment - so a full send buffer is retried.
    //
    // BUT IT IS RETRIED FOR A BOUNDED TIME. A peer that stops reading holds its
    // window shut for as long as it likes, and an unbounded retry here is a
    // hang: one stalled client, and the whole node spins at full CPU for ever,
    // serving nobody, feeding no watchdog. That is not a Modbus failure the
    // application can see or recover from - it is the process dying on its
    // feet, and on a microcontroller it is a watchdog reset.
    //
    // 250 ms is far longer than any healthy peer needs to make room on a local
    // network, and far shorter than any client's own timeout. A peer that has
    // not read a byte in that long is gone, so the connection is dropped: the
    // application then sees a link down, which it knows what to do with.
    const mb_time_t deadline = mb_millis() + kSendBudgetMs;

    size_t sent = 0;
    while (sent < length) {
        const ssize_t wrote = ::send((int)_socket, (const char*)buffer + sent,
                                     (int)(length - sent), MB_SEND_FLAGS);
        if (wrote > 0) {
            sent += (size_t)wrote;
            continue;
        }
        if (wrote < 0 && MB_WOULD_BLOCK) {
            if (mb_isFuture(mb_millis(), deadline)) {
                mb_yield();
                continue;
            }
            // Out of patience. Whatever went out is a fragment, so the stream
            // is no longer trustworthy and the connection cannot be reused.
            _stalledSends++;
            end();
            return 0;
        }
        end();
        break;
    }
    return sent;
}

// =============================================================================
// SERVER
// =============================================================================

ModbusTcpServer::ModbusTcpServer(uint16_t port, const char* bindAddress)
    : _port(port), _bindAddress(bindAddress ? bindAddress : "") {
    ensureSocketLibrary();
}

ModbusTcpServer::~ModbusTcpServer() { end(); }

bool ModbusTcpServer::fail(const char* what) {
    _lastError = what;
    end();
    return false;
}

bool ModbusTcpServer::begin() {
    end();
    _lastError.clear();

    const intptr_t handle = (intptr_t)socket(AF_INET, SOCK_STREAM, 0);
    if (handle < 0) return fail("could not create a listening socket");

    // Without this, restarting the server fails for as long as the operating
    // system holds the old port in TIME_WAIT - which is a minute or two, and is
    // an eternity when something is being commissioned.
    int reuse = 1;
    setsockopt((int)handle, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof reuse);

    sockaddr_in address;
    memset(&address, 0, sizeof address);
    address.sin_family = AF_INET;
    address.sin_port = htons(_port);
    address.sin_addr.s_addr = _bindAddress.empty()
                                  ? htonl(INADDR_ANY)
                                  : inet_addr(_bindAddress.c_str());

    if (::bind((int)handle, (sockaddr*)&address, sizeof address) != 0) {
        MB_CLOSE_SOCKET((int)handle);
        return fail("could not bind the port");
    }

    if (::listen((int)handle, 8) != 0) {
        MB_CLOSE_SOCKET((int)handle);
        return fail("could not listen");
    }

    setNonBlocking(handle);
    _socket = handle;
    _accepted = 0;
    return true;
}

void ModbusTcpServer::end() {
    if (_socket != ModbusTcpTransport::INVALID_SOCKET_HANDLE) {
        MB_CLOSE_SOCKET((int)_socket);
        _socket = ModbusTcpTransport::INVALID_SOCKET_HANDLE;
    }
}

bool ModbusTcpServer::isListening() const {
    return _socket != ModbusTcpTransport::INVALID_SOCKET_HANDLE;
}

ModbusTransport* ModbusTcpServer::accept() {
    if (!isListening()) return nullptr;

    sockaddr_in address;
    socklen_t length = sizeof address;
    const intptr_t handle = (intptr_t)::accept((int)_socket, (sockaddr*)&address, &length);
    if (handle < 0) return nullptr;   // Nobody waiting, which is the usual case

    _accepted++;
    return new ModbusTcpTransport(handle);
}

void ModbusTcpServer::release(ModbusTransport* transport) {
    // The listener allocated it, so the listener frees it. Only it knows how.
    delete transport;
}

#endif // MODBUS_PLATFORM_HOST
