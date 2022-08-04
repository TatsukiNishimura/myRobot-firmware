#include "connectToROS.hpp"

/**
 * @brief Construct a new connect To R O S::connect To R O S object
 *
 */
connectToROS::connectToROS() : serial(TX, RX, baudRate), pt(nullptr), rt(nullptr), queuePtr(nullptr), cond(readMutex), led(LED3)
{
    led = false;
}

/**
 * @brief Destroy the connect To R O S::connect To R O S object
 *
 */
connectToROS::~connectToROS()
{
    usableReadBuffer.clear();
    // startReceive()が一度でも実行されていればthreadをjoinする
    if (isRunning)
    {
        rt->join();
        pt->join();
    }
}

/**
 * @brief 受信系の関数をまとめたやつ
 *
 */
void connectToROS::startReceive()
{
    // デストラクタでスレッドの終了を行うかどうかのフラグ
    isRunning = true;
    // parse()を走らせるthread
    pt = make_unique<Thread>();
    // 受信割り込みをユーザコンテキストに伸ばした時に使うスレッド
    rt = make_unique<Thread>();
    // 受信割り込みをユーザコンテキストに伸ばす時につかうEventQueue
    queuePtr = make_unique<EventQueue>(32 * EVENTS_EVENT_SIZE);
    // callback使わないとコンパイルエラー
    pt->start(callback(this, &connectToROS::parseThread));
    // イベントキューを消化する
    rt->start(callback(queuePtr.get(), &EventQueue::dispatch_forever));
    serial.attach(callback(connectToROS::handler, this));
}

/**
 * @brief 受信割り込み時に発火する関数
 *
 * @param p thisを入れる
 */
void connectToROS::handler(void const *p)
{
    connectToROS *instance = (connectToROS *)p;
    // 割り込み中に割り込みが来ないように一度割り込みを不可にする
    instance->serial.attach(nullptr);
    instance->queuePtr->call(connectToROS::readThread, instance);
}

/**
 * @brief 書き込み
 *
 * @param buf
 * @param size
 * @return ssize_t
 */
ssize_t connectToROS::writeData(const uint8_t *buf, ssize_t size)
{
    const ssize_t ret = serial.write(buf, size);
    return ret;
}

/**
 * @brief 受信割り込み時に実際に読み込みをする関数
 *
 * @param p
 */
void connectToROS::readThread(void const *p)
{
    connectToROS *instance = (connectToROS *)p;
    // 一応mutexかける
    instance->readMutex.lock();

    uint8_t tempBuffer[32] = {0};
    if (const ssize_t len = instance->serial.read(tempBuffer, 1))
    {
        // 受信したデータを移す
        for (uint8_t i = 0; i < len; i++)
        {
            instance->usableReadBuffer.emplace_back(tempBuffer[i]);
        }
        // overflowしたらはみ出した部分を古い方から削除
        if (instance->usableReadBuffer.size() > instance->buffer_size)
        {
            const unsigned int overflow = instance->usableReadBuffer.size() - instance->buffer_size;
            instance->usableReadBuffer.erase(instance->usableReadBuffer.begin(), instance->usableReadBuffer.begin() + overflow);
        }
    }

    // 割り込み再設定
    instance->serial.attach(callback(connectToROS::handler, instance));
    // condition_Variableで受信の終了を通知
    instance->cond.notify_all();
    instance->readMutex.unlock();
}

/**
 * @brief 受信データからidと中身を取り出す
 *
 */
void connectToROS::parseThread()
{
    while (true)
    {
        short ret = getChar();
        if (ret >= 0)
        {
            printf("%d ", ret);
            const uint8_t c = static_cast<uint8_t>(ret);
            switch (state)
            {
            case readDataState::HEADER1:
                if (c == header)
                {
                    state = readDataState::HEADER2;
                }
                break;
            case readDataState::HEADER2:
                if (c == header)
                {
                    state = readDataState::ID;
                }
                else
                {
                    state = readDataState::HEADER1;
                }
                break;
            case readDataState::ID:
                id = c;
                state = readDataState::LENGTH;
                break;
            case readDataState::LENGTH:
                length = c;
                state = readDataState::DATA;
                break;
            case readDataState::DATA:
                if (count < length)
                {
                    bytes_data.emplace_back(c);
                    if (count == length - 1)
                    {
                        state = readDataState::CHECKSUM;
                    }
                    count++;
                }
                break;
            case readDataState::CHECKSUM:
                int checksum = 0;
                for (const uint8_t &i : bytes_data)
                {
                    checksum += i;
                }
                if (c == (checksum & 0xFF) + 1)
                {
                    // parse succeed
                    led = !led;
                }

                bytes_data.clear();
                count = 0;
                state = readDataState::HEADER1;
                break;
            }
        }
    }
}

/**
 * @brief 1文字取得
 *
 * @return short
 */
short connectToROS::getChar()
{
    readMutex.lock();
    // readThreadで受信が終わるまで待機
    cond.wait();
    if (!usableReadBuffer.size())
    {
        return -1;
    }
    const uint8_t ret = usableReadBuffer[0];
    // 1文字読み込んだら1文字削除
    usableReadBuffer.erase(usableReadBuffer.begin());
    readMutex.unlock();
    return ret;
}