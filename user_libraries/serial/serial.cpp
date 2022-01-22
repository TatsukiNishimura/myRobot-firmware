#include "serial.hpp"

namespace userLib
{
    serial::serial() : obj(USBTX, USBRX, baudRate)
    {
    }
    serial::~serial()
    {
    }

    uint32_t serial::printf(const char *str, ...)
    {
        va_list args;
        int n;
        static char strbuf[1024];

        va_start(args, str);
        n = vsnprintf(strbuf, sizeof(strbuf), str, args);
        va_end(args);
        strbuf[n] = '\0';

        obj.write(strbuf, n + 1);
        return n + 1;
    }

    uint32_t serial::printf(std::string data)
    {
        const int n = data.length();
        obj.write(data.c_str(), n);
        return n;
    }
}