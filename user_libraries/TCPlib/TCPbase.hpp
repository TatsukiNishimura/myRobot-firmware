#pragma once

#include "EthernetInterface.h"
#include "mbed.h"
#include "param.hpp"
class TCPbase {
 public:
  TCPbase();
  ~TCPbase();

  void configureReceive();
  void configureSend();
  void poll();
  void setPoll(bool _ispolling);
  void send(uint8_t *data, int len);
  void setOnReceiveHandler(const std::function<void(uint8_t *data, int len)> &_onReceive)
  {
    isOnReceiveSet = true;
    onReceive = _onReceive;
  }

 private:
  EthernetInterface eth;
  TCPSocket *client;
  TCPSocket receiveSocket;
  TCPSocket sendSocket;
  SocketAddress destination;
  uint8_t rxBuf[15] = {0};
  bool isPolling;
  bool isReceiveConfigured = false;
  bool isSendConfigured = false;
  bool isOnReceiveSet = false;
  std::function<void(uint8_t *data, int len)> onReceive;
};