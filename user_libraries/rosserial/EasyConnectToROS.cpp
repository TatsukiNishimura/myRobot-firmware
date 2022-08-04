#include "EasyConnectToROS.hpp"

/**
 * @brief Construct a new connect To R O S::connect To R O S object
 *
 */
easyConnectToROS::easyConnectToROS() : serial(TX, RX, baudRate), led(LED3), t(nullptr), queue(nullptr), logger(D1, D0, 9600), cond(mutex)
{
    led = false;
}

/**
 * @brief Destroy the connect To R O S::connect To R O S object
 *
 */
easyConnectToROS::~easyConnectToROS()
{
    t->join();
}

/**
 * @brief 受信系の関数をまとめたやつ
 *
 */
void easyConnectToROS::startReceive()
{
    t = make_unique<Thread>();
    t->start([this]()
             {while(1)
    {
        this->readThread(this);
    } });
}

void easyConnectToROS::handler(void const *p)
{
    easyConnectToROS *instance = (easyConnectToROS *)p;
    instance->queue->call(readThread, instance);
}

/**
 * @brief 書き込み
 *
 * @param buf
 * @param size
 * @return ssize_t
 */
ssize_t easyConnectToROS::writeData(const uint8_t *buf, ssize_t size)
{
    mutex.lock();
    cond.wait();
    const ssize_t ret = serial.write(buf, size);
    mutex.unlock();
    return ret;
}

/**
 * @brief 受信割り込み時に実際に読み込みをする関数
 *
 * @param p
 */
void easyConnectToROS::readThread(void const *p)
{
    easyConnectToROS *instance = (easyConnectToROS *)p;
    uint8_t c = 0;
    instance->mutex.lock();
    if (const ssize_t len = instance->serial.read(&c, 1))
    {
        switch (instance->state)
        {
        case readDataState::HEADER1:
            if (c == instance->header)
            {
                instance->state = readDataState::HEADER2;
            }
            break;
        case readDataState::HEADER2:
            if (c == instance->header)
            {
                instance->state = readDataState::ID;
            }
            else
            {
                instance->state = readDataState::HEADER1;
            }
            break;
        case readDataState::ID:
            instance->id = c;
            instance->state = readDataState::LENGTH;
            break;
        case readDataState::LENGTH:
            instance->length = c;
            instance->state = readDataState::DATA;
            break;
        case readDataState::DATA:
            if (instance->count < instance->length)
            {
                instance->bytes_data.emplace_back(c);
                if (instance->count == instance->length - 1)
                {
                    instance->state = readDataState::CHECKSUM;
                }
                instance->count++;
            }
            break;
        case readDataState::CHECKSUM:
            int checksum = 0;
            for (const uint8_t &i : instance->bytes_data)
            {
                checksum += i;
            }
            if (c == (checksum & 0xFF) + 1)
            {
                // parse succeed
                for (uint8_t e : instance->bytes_data)
                {
                    instance->logger.printf("%d ", e);
                }
                instance->logger.printf("\n");
                instance->led = !instance->led;
            }

            instance->bytes_data.clear();
            instance->count = 0;
            instance->state = readDataState::HEADER1;
            break;
        }
    }
    instance->cond.notify_all();
    instance->mutex.unlock();
}
