
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
)
