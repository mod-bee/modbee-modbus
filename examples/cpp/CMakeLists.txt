# The host examples. Each is one file and links the library.

set(MODBUS_EXAMPLES
    tcp_server
    tcp_client
    rtu_server
    rtu_client
    gateway
)

foreach(example IN LISTS MODBUS_EXAMPLES)
    add_executable(modbee_${example} ${example}/main.cpp)
    target_link_libraries(modbee_${example} PRIVATE modbee-modbus)
endforeach()
