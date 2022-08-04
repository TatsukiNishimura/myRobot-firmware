
add_library(user_libraries INTERFACE)

target_include_directories(user_libraries
    INTERFACE
    .
)

target_sources(user_libraries
    INTERFACE
    serial/serial.cpp
    UDPLib/UDPBase.cpp
    UDPLib/UDPSend.cpp
    UDPLib/UDPReceive.cpp
    QEI/QEI.cpp
    AS5600/AS5600.cpp
    TB6612FNG/TB6612FNG.cpp
    BTS7960/BTS7960.cpp
    rosserial/connectToROS.cpp
    rosserial/EasyConnectToROS.cpp
)
