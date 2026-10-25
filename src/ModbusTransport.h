#pragma once

#include "ModbusPlatform.h"

// The conventional Modbus TCP port. Named so nothing has to remember the
// number, and so a test can use a different one without a magic literal.
//
// Here rather than in the TCP transport, because that file compiles to nothing
// on Arduino - and an Arduino sketch using WiFiServer still needs the number.
#define MODBUS_TCP_PORT 502

// --- Abstract transport interface ---

// One bidirectional byte channel: a serial port, one TCP socket, one UDP
// endpoint. The single point of contact between the protocol core and the
// outside world. The core reads and writes bytes; it never learns which medium
// carries them.
//
// Implementations live in src/transport/ and are selected by the application,
// so any platform can pair with any medium:
//
//                  Serial                        TCP                   UDP
//   Arduino    ModbusStreamTransport      ModbusTcpTransport    ModbusUdpTransport
//   Host       ModbusSerialTransport      ModbusTcpTransport    ModbusUdpTransport
//
// Datagram transports present a byte stream by draining one datagram at a time.
// Every codec resynchronises, so a datagram boundary needs no special handling
// in the core.
class ModbusTransport {
public:
    virtual ~ModbusTransport() = default;

    // Open the underlying medium. Returns false if it cannot be used.
    virtual bool begin() = 0;

    // Close the medium and release resources. Must tolerate repeat calls.
    virtual void end() = 0;

    // Bytes that can be read without blocking.
    virtual int available() = 0;

    // Next byte, or -1 if none is available. Never blocks.
    virtual int read() = 0;

    // Write one whole ADU. Returns bytes accepted; a partial write is treated
    // as a failed send by the caller.
    virtual size_t write(const uint8_t* buffer, size_t length) = 0;

    // Whether the medium is currently usable. A TCP transport reports false
    // once its peer has gone, which is what tells the session to reconnect.
    virtual bool isConnected() const = 0;

    // Bits per second the link clocks at, or 0 where that is not a meaningful
    // question. A serial port knows it and a network socket does not.
    //
    // Stated rather than measured, and Modbus RTU framing depends on it: t1.5
    // and t3.5 are defined in character times, and a line fast enough to
    // deliver a whole frame between two looks at it leaves no spacing to
    // measure. The one thing that cannot be worked out from the bus is
    // therefore the one thing the transport already knows.
    virtual uint32_t bitsPerSecond() const { return 0; }

    // Bits per character on the wire, which is what t1.5 and t3.5 are counted
    // in. Always 11 for Modbus serial: 1 start + 8 data + 1 parity + 1 stop
    // with parity, and 1 start + 8 data + 2 stop without, because the spec
    // mandates the second stop bit precisely so the character time does not
    // change. A transport that answered 10 here would make every derived gap
    // ten percent short.
    virtual uint8_t bitsPerCharacter() const { return 11; }

    // Whether this medium delivers our own transmissions back to us.
    //
    // Stated rather than detected, and it MUST be, because on Modbus a write
    // response is a byte-for-byte echo of the request that caused it. Anything
    // that tries to recognise an echo by its content therefore throws away
    // every write response it ever receives. Only the transport knows whether
    // the wiring loops back, so only the transport can say.
    //
    // True for a hardware loopback, a repeater that retransmits, a socket
    // wired to itself, and an RS-485 transceiver whose receiver is left
    // enabled while it transmits. False for ordinary half-duplex RS-485 with
    // driver control, and for a normal TCP connection.
    virtual bool echoesTransmissions() const { return false; }

    // Silence the medium needs before this node may transmit, over and above
    // what the protocol asks for. Zero for anything that does not care; an
    // RS-485 transceiver that is slow to turn around states it here rather
    // than having the core guess. A floor, never a replacement: the core still
    // raises it if the line turns out slower than this assumed.
    virtual uint32_t turnaroundUs() const { return 0; }
};

// --- Abstract listener interface ---

// Produces transports as peers arrive. A Modbus TCP server accepts several
// concurrent client connections, each with its own socket, its own transaction
// stream and its own lifecycle, so the core cannot simply be handed "the
// transport" the way a serial link can.
//
// Serial has no listener: one port is one link, for the life of the node. Only
// connection-oriented media implement this.
class ModbusListener {
public:
    virtual ~ModbusListener() = default;

    // Start listening. Returns false if the address cannot be bound.
    virtual bool begin() = 0;

    // Stop listening. Does not close transports already handed out: those
    // belong to the sessions holding them, which release them in their own
    // time. Must tolerate repeat calls.
    virtual void end() = 0;

    // The next peer that has arrived, or nullptr when none is waiting. Never
    // blocks. Ownership passes to the caller, which returns it to release().
    virtual ModbusTransport* accept() = 0;

    // Hands a transport back once its session is finished with it. The
    // listener owns the memory, because only it knows how the transport was
    // allocated.
    virtual void release(ModbusTransport* transport) = 0;

    // Whether the listener is currently accepting.
    virtual bool isListening() const = 0;
};
