#include "connectToROS.h"

namespace robohan
{
    float connectToROS::twistData[3];

    connectToROS::connectToROS(UnbufferedSerial &serial) : obj(serial)
    {
    }

    connectToROS::~connectToROS()
    {
    }

    void connectToROS::sendToROS(float *data, uint8_t length, msgs msg)
    {
        uint8_t sendData[256] = {};
        sendData[0] = startMsg;
        sendData[1] = static_cast<uint8_t>(msg);
        sendData[2] = length;
        memcpy(&sendData[3], data, length * sizeof(float));
        sendData[3 + length * sizeof(float)] = endMsg;
        obj.write(sendData, 4 + length * sizeof(float));
    }

    void connectToROS::sendToROS(int *data, uint8_t length, msgs msg)
    {
        uint8_t sendData[256] = {};
        sendData[0] = startMsg;
        sendData[1] = static_cast<uint8_t>(msg);
        sendData[2] = length;
        memcpy(&sendData[3], data, length * sizeof(int));
        sendData[3 + length * sizeof(int)] = endMsg;
        obj.write(sendData, 4 + length * sizeof(int));
    }

    void connectToROS::sendToROS(uint8_t *data, uint8_t length, msgs msg)
    {
        uint8_t sendData[256] = {};
        sendData[0] = startMsg;
        sendData[1] = static_cast<uint8_t>(msg);
        sendData[2] = length;
        memcpy(&sendData[3], data, length * sizeof(uint8_t));
        sendData[3 + length * sizeof(uint8_t)] = endMsg;
        obj.write(sendData, 4 + length * sizeof(uint8_t));
    }

    void connectToROS::readData()
    {
        int recv_data = obj.read(&buffer[recv_data_size], sizeof(uint8_t));

        if (recv_data > 0)
        {
            recv_data_size += recv_data;
            if (recv_data_size >= 256)
            {
                recv_data_size = 0;
            }
            else if (buffer[recv_data_size - 1] == endMsg)
            {
                if (buffer[0] == startMsg)
                {
                    const msgs msgType = static_cast<msgs>(buffer[1]);
                    const uint8_t arraySize = buffer[2];

                    switch (msgType)
                    {
                    case msgs::twist:
                    {
                        if (recv_data_size == arraySize * sizeof(float) + 4)
                        {
                            for (int i = 0; i < arraySize; i++)
                            {
                                for (int i = 0; i < arraySize; i++)
                                {
                                    twistData[i] = *(float *)(&buffer[i * sizeof(float) + 3]);
                                }
                            }
                        }
                        else
                        {
                        }
                        break;
                    }
                    case msgs::command:
                    {
                        if (recv_data_size == arraySize * sizeof(float) + 4)
                        {

                            for (int i = 0; i < arraySize; i++)
                            {
                                // memcpy(&commandData[i], &buffer[i * sizeof(float) + 3], sizeof(float));
                            }
                        }
                        else
                        {
                        }
                        break;
                    }
                    default:
                        break;
                    }
                }
                recv_data_size = 0;
            }
        }
    }
}