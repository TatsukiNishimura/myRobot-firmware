#include "TCPbase.hpp"

TCPbase::TCPbase() : client(nullptr)
{
  eth.set_network(tcpParam::myIp, tcpParam::myNetMask, tcpParam::myGateWay);
  if (eth.connect() != 0)
  {
    error("TCPBase constructor: net connect error");
  }
  else
  {
    printf("net connect success\r\n");
  }
}

TCPbase::~TCPbase()
{
  setPoll(false);
  ThisThread::sleep_for(1s);
  if (client != nullptr)
  {
    client->close();
  }
  receiveSocket.close();
  sendSocket.close();
}

void TCPbase::poll()
{
  if (!isReceiveConfigured)
  {
    error("configureReceive has not been called.");
  }
  if (!isOnReceiveSet)
  {
    error("TCPBase: poll(): callback function has not been set.");
  }
  setPoll(true);
  while (isPolling)
  {
    nsapi_error_t error = 0;
    client = receiveSocket.accept(&error);

    if (error == NSAPI_ERROR_OK)
    {
      while (client->recv(rxBuf, sizeof(rxBuf)) > 0)
      {
        if (onReceive)
        {
          onReceive(rxBuf, sizeof(rxBuf));
        }
        // ThisThread::sleep_for(10ms);
      }
    }
    // ThisThread::sleep_for(50ms);
  }
}

void TCPbase::setPoll(bool _ispolling)
{
  isPolling = _ispolling;
}

void TCPbase::configureReceive()
{
  SocketAddress a;
  eth.get_ip_address(&a);
  printf("ip : %s\r\n", a.get_ip_address());
  if (receiveSocket.open(&eth) != 0)
  {
    error("TCPBase construnctor: socket open fail");
  }
  else
  {
    printf("receive socket opened\r\n");
  }
  receiveSocket.set_blocking(false);
  receiveSocket.bind(tcpParam::myPort);
  receiveSocket.listen(1);

  isReceiveConfigured = true;
}

void TCPbase::configureSend()
{
  if (sendSocket.open(&eth) != 0)
  {
    error("TCPBase construnctor: socket open fail");
  }
  destination.set_ip_address(tcpParam::pcIp);
  destination.set_port(tcpParam::pcPort);
  // sendSocket.set_timeout(3);
  while (1)
  {
    nsapi_error_t error = sendSocket.connect(destination);
    if (error == NSAPI_ERROR_OK)
    {
      printf("connected\r\n");
      break;
    }
    else
    {
      printf("%d\r\n", error);
      ThisThread::sleep_for(1s);
    }
  }
  isSendConfigured = true;
}

void TCPbase::send(uint8_t *data, int len)
{
  if (!isSendConfigured)
  {
    error("configureSend has not been called.");
  }
  if (sendSocket.send(data, len) <= 0)
  {
    printf("reconnection...\n");
    sendSocket.close();
    sendSocket.open(&eth);
    sendSocket.connect(destination);
  }
  else
  {
    printf("sent\r\n");
  }
  // ThisThread::sleep_for(10ms);
}