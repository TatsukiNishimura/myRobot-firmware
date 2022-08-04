#pragma once

#include "mbed.h"
#include <vector>
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
    ssize_t writeSerializedData(int id, Args... args)
    {
        int index = 0;
        int checksum = 0;
        // 引数をuint8_tの配列に変換
        std::vector<uint8_t> data = serialize(args...);
        const ssize_t len = byteSize(args...);
        uint8_t sendData[len + 5] = {0};
        sendData[0] = header;
        sendData[1] = header;
        sendData[2] = id;
        sendData[3] = len;
        for (uint8_t e : data)
        {
            sendData[4 + index] = e;

            checksum += e;
            index++;
        }
        sendData[4 + len] = checksum & 0xFF;

        const ssize_t ret = writeData(sendData, sizeof(sendData));
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

    DigitalOut led;
};