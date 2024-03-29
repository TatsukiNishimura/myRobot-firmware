#pragma once

#include "mbed.h"
#include <vector>
#include <numeric>
#include "serialize.h"

enum class readDataState
{
    HEADER1 = 0,
    HEADER2,
    ID,
    LENGTH,
    DATA,
    CHECKSUM,
};

class connectToROS
{
public:
    connectToROS();
    ~connectToROS();
    void startReceive();
    ssize_t writeData(const uint8_t *buf, ssize_t size);
    template <class... Args>
    ssize_t writeSerializedData(uint8_t id, Args... args)
    {
        // 引数をuint8_tの配列に変換
        std::vector<uint8_t> data = serialize(args...);
        std::vector<uint8_t> sendData;
        sendData.emplace_back(header);
        sendData.emplace_back(header);
        sendData.emplace_back(id);
        sendData.emplace_back(static_cast<uint8_t>(data.size()));
        int checksum = 0;
        for (const uint8_t &e : data)
        {
            sendData.emplace_back(e);
            checksum += e;
        }

        sendData.emplace_back(static_cast<uint8_t>(checksum & 0xFF) + 1);

        uint8_t array[sendData.size()] = {0};
        std::copy(sendData.begin(), sendData.end(), array);
        // for (int i = 0; i < sizeof(array); i++)
        // {
        //     printf("%d ", array[i]);
        // }
        // printf("\r\n");
        const ssize_t ret = writeData(array, sizeof(array));
        return ret;
    }

private:
    static void readThread(void const *p);
    void parseThread();
    static void handler(void const *p);
    short getChar();

    PinName TX = USBTX;
    PinName RX = USBRX;
    const uint8_t header = 0xFF;
    int baudRate = 9600;
    size_t buffer_size = 256;
    bool isRunning = false;
    UnbufferedSerial serial;
    unique_ptr<Thread> pt;
    unique_ptr<Thread> rt;
    std::vector<uint8_t> usableReadBuffer;
    Mutex readMutex;
    uint8_t id = 0;
    uint8_t length = 0;
    std::vector<uint8_t> bytes_data;
    readDataState state = readDataState::HEADER1;
    unique_ptr<EventQueue> queuePtr;
    ConditionVariable cond;
    int count = 0;
    DigitalOut led;
};