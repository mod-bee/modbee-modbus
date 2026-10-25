cmake_minimum_required(VERSION 3.16)

project(ModBeeModbus
    VERSION 0.1.0
    DESCRIPTION "Modbus RTU, ASCII and TCP - client and server - for Arduino and hosts"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MODBUS_BUILD_EXAMPLES  "Build the plain C++ examples" ON)
option(MODBUS_BUILD_TESTS     "Build the test suite" ON)
option(MODBUS_SANITIZE        "Build with the address and undefined behaviour sanitizers" OFF)
option(MODBUS_SANITIZE_THREAD "Build with the thread sanitizer (excludes MODBUS_SANITIZE)" OFF)

# The library parses bytes it did not produce, so an out-of-bounds read is the
# failure mode that matters most and the one least likely to show as a wrong
# answer. CI runs the suite under these on every push.
if(MODBUS_SANITIZE)
    if(MSVC)
        add_compile_options(/fsanitize=address)
    else()
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g)
        add_link_options(-fsanitize=address,undefined)
    endif()
endif()

# A host application may drive one node from two threads -- a task servicing
# the link while application code calls in from a slower one. The
# address sanitizer says nothing about that; only the thread sanitizer does,
# and the two cannot be built together.
if(MODBUS_SANITIZE_THREAD)
    if(MODBUS_SANITIZE)
        message(FATAL_ERROR "MODBUS_SANITIZE and MODBUS_SANITIZE_THREAD cannot be combined")
    endif()
    if(NOT MSVC)
        add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
        add_link_options(-fsanitize=thread)
    endif()
endif()

# The host serial transport blocks SIGTTOU per thread with pthread_sigmask (see
# the library's link options below), so the threading library is required.
# Nothing in the library locks: a node is driven from one thread, or the
# application serialises its calls against poll() itself.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# =============================================================================
# LIBRARY
# =============================================================================

# Listed explicitly rather than globbed, so a new file is a deliberate change
# and CMake never silently misses one.
set(MODBUS_SOURCES
    src/ModbusPlatform.cpp
    src/ModbusArray.cpp
    src/ModbusTypes.cpp
    src/ModbusCrc.cpp
    src/ModbusPdu.cpp
    src/ModbusDataMap.cpp
    src/ModbusOperations.cpp
    src/ModbusServerEngine.cpp
    src/ModbusLink.cpp
    src/ModbusClient.cpp
    src/ModbusServer.cpp
    src/ModbusNode.cpp
    src/ModbusServerPool.cpp
    src/ModbusGateway.cpp
    src/ModbusPoller.cpp
    src/ModbusPoint.cpp
    src/ModbusAPI.cpp
    src/ModbusAPIServer.cpp
    src/ModbusBridge.cpp
    src/transport/ModbusSerialTransport.cpp
    src/transport/ModbusTcpTransport.cpp
    src/codec/ModbusRtuCodec.cpp
    src/codec/ModbusTcpCodec.cpp
    src/codec/ModbusAsciiCodec.cpp
)

add_library(modbee-modbus STATIC ${MODBUS_SOURCES})
add_library(ModBee::modbus ALIAS modbee-modbus)

target_include_directories(modbee-modbus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/modbee>
)

if(WIN32)
    target_link_libraries(modbee-modbus PUBLIC ws2_32)
else()
    # The serial transport blocks SIGTTOU around tcsetattr, tcdrain and tcflush.
    # Without that, a process in a background process group whose controlling
    # terminal is the serial port is STOPPED by the kernel rather than failing -
    # suspended, silent, with nothing to poll. pthread_sigmask is how it is
    # blocked per thread rather than process-wide.
    target_link_libraries(modbee-modbus PUBLIC Threads::Threads)
endif()

if(MSVC)
    target_compile_options(modbee-modbus PRIVATE /W4)
else()
    target_compile_options(modbee-modbus PRIVATE -Wall -Wextra)
endif()

# =============================================================================
# EXAMPLES
# =============================================================================

if(MODBUS_BUILD_EXAMPLES AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/cpp/CMakeLists.txt)
    add_subdirectory(examples/cpp)
endif()

# =============================================================================
# TESTS
# =============================================================================

if(MODBUS_BUILD_TESTS AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt)
    enable_testing()
    add_subdirectory(test)
endif()
