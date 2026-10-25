#pragma once

#include "../ModbusTypes.h"
#include "../ModbusTransport.h"
#include "../ModbusTiming.h"

#if defined(MODBUS_PLATFORM_HOST)

#include <string>

struct ModbusSerialConfig {
    // "/dev/ttyUSB0" or a pseudo-terminal on POSIX, "COM3" on Windows.
    const char* device = nullptr;
    uint32_t baudRate = 19200;
    ModbusParity parity = ModbusParity::Even;

    // Whether the medium hands our own transmissions back to us. See
    // ModbusTransport::echoesTransmissions - this MUST be stated rather than
    // detected, because a Modbus write response is byte-for-byte identical to
    // the request that caused it.
    //
    // True for an RS-485 transceiver whose receiver stays enabled while it
    // drives, which is how a two-wire adapter without direction control
    // behaves.
    bool echoes = false;

    // Microseconds the transceiver needs to turn around, over and above the
    // protocol's own gap. Zero for anything modern.
    uint32_t turnaroundUs = 0;
};

// Serial transport for Linux, macOS and Windows.
//
// The port is opened raw and non-blocking, so read() returns immediately when
// no bytes are waiting. Pseudo-terminals - a socat pair, say - behave
// identically to hardware ports here, which is what makes hardware-free
// testing honest rather than merely convenient.
class ModbusSerialTransport : public ModbusTransport {
public:
    typedef intptr_t handle_t;
    static const handle_t INVALID_HANDLE = -1;

    explicit ModbusSerialTransport(const ModbusSerialConfig& config = ModbusSerialConfig());
    ~ModbusSerialTransport() override;

    // Set after construction, so an object holding this as a member can be
    // declared before the application knows which port it will open. Takes
    // effect on the next begin().
    void setConfig(const ModbusSerialConfig& config);

    ModbusSerialTransport(const ModbusSerialTransport&) = delete;
    ModbusSerialTransport& operator=(const ModbusSerialTransport&) = delete;

    bool begin() override;
    void end() override;
    int available() override;
    int read() override;
    size_t write(const uint8_t* buffer, size_t length) override;
    bool isConnected() const override;

    uint32_t bitsPerSecond() const override { return _config.baudRate; }
    bool echoesTransmissions() const override { return _config.echoes; }
    uint32_t turnaroundUs() const override { return _config.turnaroundUs; }

    // Why the last begin() failed. Empty when there was no error.
    const char* lastError() const { return _lastError.c_str(); }

    // Sends that gave up because the port would not take the bytes in time -
    // hardware flow control held off, or an adapter pulled out mid-frame.
    //
    // A first-class counter rather than a log line, because the alternative to
    // giving up is spinning for ever, and a number that only ever rises when
    // something is genuinely wrong is worth having where an integrator can see
    // it.
    uint32_t stalledSends() const { return _stalledSends; }

private:
    bool configurePort();
    bool fail(const char* what);

    // How long to keep trying to hand `length` bytes to the port.
    //
    // Scaled by the line rate, because what counts as too long depends
    // entirely on it: 256 bytes at 1200 baud legitimately takes 2.3 seconds to
    // leave, and the same frame at 115200 takes 24 ms. Four times the airtime
    // plus 250 ms is generous for a working port and finite for a dead one.
    mb_time_t sendBudgetMs(size_t length) const {
        const uint32_t baud = _config.baudRate ? _config.baudRate : 9600;
        const uint32_t airtimeMs = (uint32_t)((length * MODBUS_BITS_PER_CHARACTER * 1000u) / baud);
        return (mb_time_t)(airtimeMs * 4u + 250u);
    }

    ModbusSerialConfig _config;
    std::string _device;
    handle_t _handle;

    // One byte of lookahead, so available() can report a byte that read() has
    // already pulled out of the operating system's buffer.
    int _peeked;

    uint32_t _stalledSends = 0;

    std::string _lastError;
};

#endif // MODBUS_PLATFORM_HOST
