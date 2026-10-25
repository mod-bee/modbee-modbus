#include "ModbusSerialTransport.h"

#if defined(MODBUS_PLATFORM_HOST)

#include <cstring>

#if defined(_WIN32)
    #include <windows.h>
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <termios.h>
    #include <sys/ioctl.h>
    #include <signal.h>
    #include <pthread.h>
#endif

namespace {

#if !defined(_WIN32)
// Maps a numeric baud rate onto the termios constant. Returns false when the
// rate has no portable constant, rather than silently picking a wrong speed -
// a port opened at the wrong rate produces nothing but checksum errors, and
// there is no way to tell that from a broken cable.
bool toSpeed(uint32_t baud, speed_t& out) {
    switch (baud) {
        case 1200:    out = B1200;    return true;
        case 2400:    out = B2400;    return true;
        case 4800:    out = B4800;    return true;
        case 9600:    out = B9600;    return true;
        case 19200:   out = B19200;   return true;
        case 38400:   out = B38400;   return true;
        case 57600:   out = B57600;   return true;
        case 115200:  out = B115200;  return true;
        case 230400:  out = B230400;  return true;
#ifdef B460800
        case 460800:  out = B460800;  return true;
#endif
#ifdef B921600
        case 921600:  out = B921600;  return true;
#endif
        default: return false;
    }
}
#endif

} // namespace

ModbusSerialTransport::ModbusSerialTransport(const ModbusSerialConfig& config)
    : _config(config),
      _device(config.device ? config.device : ""),
      _handle(INVALID_HANDLE),
      _peeked(-1) {}

ModbusSerialTransport::~ModbusSerialTransport() { end(); }

void ModbusSerialTransport::setConfig(const ModbusSerialConfig& config) {
    _config = config;
    _device = config.device ? config.device : "";
}

bool ModbusSerialTransport::fail(const char* what) {
    _lastError = what;
    end();
    return false;
}

bool ModbusSerialTransport::isConnected() const { return _handle != INVALID_HANDLE; }

#if defined(_WIN32)

bool ModbusSerialTransport::begin() {
    end();
    _lastError.clear();
    if (_device.empty()) return fail("no serial device specified");

    // The \\.\ prefix is required for COM10 and above.
    std::string path = _device;
    if (path.compare(0, 4, "\\\\.\\") != 0) path = "\\\\.\\" + path;

    const HANDLE handle = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                                      0, NULL, OPEN_EXISTING, 0, NULL);
    if (handle == INVALID_HANDLE_VALUE) return fail("could not open serial device");

    _handle = (handle_t)handle;
    if (!configurePort()) return false;
    _peeked = -1;
    return true;
}

bool ModbusSerialTransport::configurePort() {
    const HANDLE handle = (HANDLE)_handle;

    DCB dcb;
    memset(&dcb, 0, sizeof dcb);
    dcb.DCBlength = sizeof dcb;
    if (!GetCommState(handle, &dcb)) return fail("could not read port state");

    dcb.BaudRate = _config.baudRate;
    dcb.ByteSize = 8;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fAbortOnError = FALSE;

    switch (_config.parity) {
        case ModbusParity::Even:
            dcb.Parity = EVENPARITY; dcb.fParity = TRUE; dcb.StopBits = ONESTOPBIT; break;
        case ModbusParity::Odd:
            dcb.Parity = ODDPARITY;  dcb.fParity = TRUE; dcb.StopBits = ONESTOPBIT; break;
        case ModbusParity::None:
            // Two stop bits, so the character time stays at eleven.
            dcb.Parity = NOPARITY;   dcb.fParity = FALSE; dcb.StopBits = TWOSTOPBITS; break;
        case ModbusParity::None8N1:
            dcb.Parity = NOPARITY;   dcb.fParity = FALSE; dcb.StopBits = ONESTOPBIT; break;
    }

    if (!SetCommState(handle, &dcb)) return fail("could not configure port");

    // Fully non-blocking: a read returns whatever is already buffered and never
    // waits. The protocol's own timeouts are the only ones that matter, and a
    // driver-level wait would make the service interval unmeasurable.
    COMMTIMEOUTS timeouts;
    memset(&timeouts, 0, sizeof timeouts);
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 5000;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    if (!SetCommTimeouts(handle, &timeouts)) return fail("could not set port timeouts");

    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
}

void ModbusSerialTransport::end() {
    if (_handle != INVALID_HANDLE) {
        CloseHandle((HANDLE)_handle);
        _handle = INVALID_HANDLE;
    }
    _peeked = -1;
}

int ModbusSerialTransport::available() {
    if (_handle == INVALID_HANDLE) return 0;
    if (_peeked >= 0) return 1;

    COMSTAT status;
    DWORD errors = 0;
    if (!ClearCommError((HANDLE)_handle, &errors, &status)) return 0;
    if (status.cbInQue > 0) return (int)status.cbInQue;

    const int byte = read();
    if (byte < 0) return 0;
    _peeked = byte;
    return 1;
}

int ModbusSerialTransport::read() {
    if (_peeked >= 0) {
        const int byte = _peeked;
        _peeked = -1;
        return byte;
    }
    if (_handle == INVALID_HANDLE) return -1;

    uint8_t byte = 0;
    DWORD got = 0;
    if (!ReadFile((HANDLE)_handle, &byte, 1, &got, NULL) || got == 0) return -1;
    return byte;
}

size_t ModbusSerialTransport::write(const uint8_t* buffer, size_t length) {
    if (_handle == INVALID_HANDLE || !buffer || length == 0) return 0;

    DWORD written = 0;
    if (!WriteFile((HANDLE)_handle, buffer, (DWORD)length, &written, NULL)) return 0;

    // Wait for the last byte to leave the shift register before returning.
    //
    // On RS-485 this is what stops the driver being switched off mid-character,
    // which is the single most common wiring fault in the field and produces a
    // truncated frame the far end reads as noise.
    FlushFileBuffers((HANDLE)_handle);
    return written;
}

#else   // POSIX

bool ModbusSerialTransport::begin() {
    end();
    _lastError.clear();
    if (_device.empty()) return fail("no serial device specified");

    // O_NOCTTY: opening a terminal must not make it this process's controlling
    // terminal, or a hangup on the port would signal the whole process group.
    const int fd = ::open(_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return fail("could not open serial device");

    _handle = (handle_t)fd;
    if (!configurePort()) return false;
    _peeked = -1;
    return true;
}

// Terminal control must never be able to STOP this process.
//
// tcsetattr, tcdrain and tcflush all raise SIGTTOU when the caller is in a
// BACKGROUND process group and the device is the session's controlling
// terminal. The default action of SIGTTOU is to stop the process - not to fail
// the call, not to return an error: the process is suspended, in state T, with
// no log line and nothing to poll. To anyone watching it has frozen solid.
//
// That is not a corner case. A process lands in a background process group
// whenever it is started by a shell script under job control, by `timeout`, by
// some supervisors, or by anything that puts its child in a new group. And the
// serial port becomes the controlling terminal the moment anything opens it
// without O_NOCTTY - which this transport is careful never to do, but a start-up
// script that checked the device was openable might already have done.
//
// A gateway in a container can sit stopped for ever that way, answering
// nothing, because its launcher touched the device first.
//
// POSIX is explicit that when SIGTTOU is blocked or ignored, the operation is
// allowed to proceed. So it is blocked for the duration of the call and the
// previous mask restored afterwards - per thread, so nothing else is disturbed.
namespace {

class TtyStopGuard {
public:
    TtyStopGuard() : _blocked(false) {
        sigset_t block;
        sigemptyset(&block);
        sigaddset(&block, SIGTTOU);
        _blocked = (pthread_sigmask(SIG_BLOCK, &block, &_previous) == 0);
    }

    ~TtyStopGuard() {
        if (_blocked) pthread_sigmask(SIG_SETMASK, &_previous, nullptr);
    }

    TtyStopGuard(const TtyStopGuard&) = delete;
    TtyStopGuard& operator=(const TtyStopGuard&) = delete;

private:
    sigset_t _previous;
    bool _blocked;
};

} // namespace

bool ModbusSerialTransport::configurePort() {
    const int fd = (int)_handle;

    struct termios options;
    TtyStopGuard noStopping;

    if (tcgetattr(fd, &options) != 0) return fail("could not read port attributes");

    speed_t speed = B19200;
    if (!toSpeed(_config.baudRate, speed)) return fail("unsupported baud rate");

    cfmakeraw(&options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~(PARENB | PARODD | CSTOPB);

    switch (_config.parity) {
        case ModbusParity::Even:
            options.c_cflag |= PARENB;
            break;
        case ModbusParity::Odd:
            options.c_cflag |= (PARENB | PARODD);
            break;
        case ModbusParity::None:
            // Two stop bits, so the character time stays at eleven and every
            // figure derived from it stays right.
            options.c_cflag |= CSTOPB;
            break;
        case ModbusParity::None8N1:
            break;
    }

    // A byte with a parity error must reach us AS A BYTE rather than being
    // stripped or replaced, so the checksum can judge the frame. Silently
    // dropping it would leave a frame one byte short, which reads as a framing
    // fault rather than as the line fault it is.
    options.c_iflag &= ~(INPCK | ISTRIP | PARMRK | IGNPAR);
    options.c_iflag |= IGNPAR;

    // Return immediately with whatever is buffered. The protocol's own timeouts
    // are the only ones that matter, and a driver-level wait would make the
    // service interval unmeasurable.
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &options) != 0) return fail("could not configure port");
    tcflush(fd, TCIOFLUSH);
    return true;
}

void ModbusSerialTransport::end() {
    if (_handle != INVALID_HANDLE) {
        ::close((int)_handle);
        _handle = INVALID_HANDLE;
    }
    _peeked = -1;
}

int ModbusSerialTransport::available() {
    if (_handle == INVALID_HANDLE) return 0;
    if (_peeked >= 0) return 1;

    int waiting = 0;
    if (ioctl((int)_handle, FIONREAD, &waiting) == 0 && waiting > 0) return waiting;

    // FIONREAD is not meaningful on every pseudo-terminal, so fall back to
    // actually reading a byte and holding it. Without this a socat pair reports
    // nothing available for ever and the link looks dead.
    const int byte = read();
    if (byte < 0) return 0;
    _peeked = byte;
    return 1;
}

int ModbusSerialTransport::read() {
    if (_peeked >= 0) {
        const int byte = _peeked;
        _peeked = -1;
        return byte;
    }
    if (_handle == INVALID_HANDLE) return -1;

    uint8_t byte = 0;
    const ssize_t got = ::read((int)_handle, &byte, 1);
    if (got != 1) return -1;
    return byte;
}

size_t ModbusSerialTransport::write(const uint8_t* buffer, size_t length) {
    if (_handle == INVALID_HANDLE || !buffer || length == 0) return 0;

    // A partial frame is worse than none - the far end reads a truncated frame
    // as a corrupt whole, not as a fragment - so a full kernel buffer is
    // retried. BOUNDED, though: a port held off by hardware flow control, or an
    // adapter unplugged mid-frame, otherwise spins here for ever and takes the
    // whole process with it.
    //
    // The budget is the airtime of the data plus a fixed allowance, because
    // what "too long" means depends entirely on the line rate: 256 bytes at
    // 1200 baud legitimately takes 2.3 seconds to leave.
    const mb_time_t deadline = mb_millis() + sendBudgetMs(length);

    size_t sent = 0;
    while (sent < length) {
        const ssize_t wrote = ::write((int)_handle, buffer + sent, length - sent);
        if (wrote > 0) {
            sent += (size_t)wrote;
            continue;
        }
        if (wrote < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (mb_isFuture(mb_millis(), deadline)) {
                mb_yield();
                continue;
            }
            _stalledSends++;
            break;
        }
        break;
    }

    // Wait for the last byte to leave the shift register.
    //
    // On RS-485 this is what stops the driver being switched off mid-character,
    // which is the single most common wiring fault in the field: the far end
    // sees a frame one byte short and reports a checksum error, and nothing at
    // either end points at the transceiver.
    TtyStopGuard noStopping;

    // This blocks until the bytes have physically left, which is deliberate and
    // is the one place the library waits on purpose. It is bounded by the line
    // rate and nothing else - a 256-byte frame is 24 ms at 115200 and 2.3
    // seconds at 1200 - so the cost is the medium's, not ours, and there is no
    // way to switch a driver off at the right moment without paying it.
    //
    // Relevant when choosing a baud rate: at 1200 baud a poll() carrying a
    // large frame takes seconds, and anything else in the same loop waits.
    tcdrain((int)_handle);
    return sent;
}

#endif

#endif // MODBUS_PLATFORM_HOST
