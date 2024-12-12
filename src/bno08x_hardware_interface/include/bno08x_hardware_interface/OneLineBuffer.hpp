#pragma once

#include <cstdint>
#include <functional>
#include <cctype>

#ifndef OneLineBuffer_SIZE
#define OneLineBuffer_SIZE 256
#endif

namespace vqw
{

    class OneLineBuffer 
    {
      public:
        using OneLineBuffer_callback_t = std::function<void(const uint8_t *, int)>;

        OneLineBuffer() {}
        OneLineBuffer(OneLineBuffer_callback_t cb) : callback(cb) {}

        void set_Callback(OneLineBuffer_callback_t cb) { callback = cb; }

        virtual int  availableForWrite(void) { return OneLineBuffer_SIZE - ndx; }
        virtual void flush()
        {
            buff[ndx]   = 0;
            if (callback != nullptr) callback(buff, ndx);
            ndx       = 0;
            buff[ndx] = 0;
        }

        virtual std::size_t write(const uint8_t b)       // int write(char c)
        {
            if (ndx >= (OneLineBuffer_SIZE - 2)) { flush(); }
            if ((ndx == 0) && (isspace(b))) return 1;
            if (b == '\n') { flush(); }
            else
                {
                    if (b != '\r')
                        {
                            buff[ndx++] = b;
                            buff[ndx]   = 0;
                        }
                }
            return 1;
        }

        virtual std::size_t write(const uint8_t *pTxt, std::size_t txtSize)
        {
            std::size_t count = 0;
            for (int i = 0; i < (int)txtSize; i++) { count += write(pTxt[i]); }
            return count;
        }

        int size() { return ndx; }
        uint8_t operator[](int x)
        {
            if (x < 0) return 0;
            if (x >= ndx) return 0;
            return buff[x];
        }
        const char *c_str() { return (const char *)buff; }

      private:
        uint8_t                  buff[OneLineBuffer_SIZE + 2];
        int                      ndx = 0;
        OneLineBuffer_callback_t callback;

    };       //---- end of:  class OneLineBuffer

};           // namespace vqw