#pragma once

#include "vqwPipe.h"
#include <stdint.h>
#include <string>
#include <vector>

namespace vqw
{

    class vqwSerializer
    {
      public:
        // for
        vqwSerializer(uint8_t *buffer, int buffer_size, int buffer_offset) : pBuf(buffer), buf_offset(buffer_offset), buf_size(buffer_size) {}
        vqwSerializer(uint8_t *buffer, int buffer_size) : pBuf(buffer), buf_size(buffer_size) {}
        vqwSerializer(const ChannelDataMsgHeader_t *pCDM)
            : pBuf(((uint8_t *)pCDM) + sizeof(ChannelDataMsgHeader_t)), buf_size(pCDM->hdr.msgSize - sizeof(ChannelDataMsgHeader_t))
        {
        }

        int     get_byteCount() { return buf_offset; }
        uint8_t get_result_code() { return result_code; }

        /// @brief serialize a single data field
        /// @param data the field
        void serialize(uint8_t data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(uint16_t data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(uint32_t data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(int8_t data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(int16_t data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(int32_t data) { serialize(((uint8_t *)&data), sizeof(data)); }
        //void serialize(int data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(float data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(double data) { serialize(((uint8_t *)&data), sizeof(data)); }
        void serialize(vqwPipe_Direction data) { serialize(((uint8_t *)&data), sizeof(data)); }

        void serialize(bool data)
        {
            uint8_t TorF = (data) ? 1 : 0;
            serialize(((const uint8_t *)&TorF), sizeof(TorF));
        }

        void serialize(const std::vector<uint8_t> data)       // max packet length is 32k bytes
        {
            uint16_t pkt_length = data.size();
            serialize(pkt_length);
            serialize(&(data[0]), pkt_length);
        }
        void serialize(const std::string data)       // max string length is 255 bytes
        {
            uint8_t str_length = (data.length() > 255) ? 255 : data.length();
            serialize(str_length);
            serialize(((uint8_t *)data.c_str()), str_length);
        }
        void serialize(const std::vector<std::string> &data)
        {
            uint8_t vec_len = (uint8_t)(data.size());
            serialize(vec_len);
            for (auto S : data) { serialize(S); }
        }
        // void serialize(float data[3])
        // {
        //     serialize(data[0]);
        //     serialize(data[1]);
        //     serialize(data[2]);
        // }
        // void serialize(float data[4])
        // {
        //     serialize(data[0]);
        //     serialize(data[1]);
        //     serialize(data[2]);
        //     serialize(data[3]);
        // }
        //+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~

        void deserialize(vqwPipe_Direction &data)
        {
            vqwPipe_Direction *pVal = (vqwPipe_Direction *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(bool &data)
        {
            uint8_t *pVal = (uint8_t *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = ((*pVal) == 0) ? false : true;
        }
        void deserialize(uint8_t &data)
        {
            uint8_t *pVal = (uint8_t *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(uint16_t &data)
        {
            uint16_t *pVal = (uint16_t *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(uint32_t &data)
        {
            uint32_t *pVal = (uint32_t *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(int8_t &data)
        {
            int8_t *pVal = (int8_t *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(int16_t &data)
        {
            int16_t *pVal = (int16_t *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(int32_t &data)
        {
            int32_t *pVal = (int32_t *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        // void deserialize(int &data)
        // {
        //     int *pVal = (int *)(pBuf + buf_offset);
        //     buf_offset += sizeof(*pVal);
        //     data = *pVal;
        // }
        void deserialize(float &data)
        {
            float *pVal = (float *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(double &data)
        {
            double *pVal = (double *)(pBuf + buf_offset);
            buf_offset += sizeof(*pVal);
            data = *pVal;
        }
        void deserialize(std::vector<uint8_t> &data)
        {
            uint16_t *pSz = (uint16_t *)(pBuf + buf_offset);
            uint16_t  sz  = *pSz;
            buf_offset += 2;
            data.clear();
            data.reserve(sz);
            char *pVal = (char *)(pBuf + buf_offset);
            for (int i = 0; i < sz; i++)
                {
                    data.push_back(*pVal);
                    pVal++;
                }
            buf_offset += sz;
        }
        void deserialize(std::string &data)
        {
            uint8_t *pSz = (uint8_t *)(pBuf + buf_offset);
            uint8_t  sz  = *pSz;
            buf_offset++;
            data.clear();
            char *pVal = (char *)(pBuf + buf_offset);
            data.append(pVal, sz);
            buf_offset += sz;
        }

        void deserialize(std::vector<std::string> &data)
        {
            uint8_t *pSz = (uint8_t *)(pBuf + buf_offset);
            uint8_t  sz  = *pSz;
            buf_offset++;
            data.clear();
            data.reserve(sz);
            for (int i = 0; i < sz; i++)
                {
                    data.emplace_back("");
                    deserialize(data[i]);
                }
        }

        // void deserialize(float data[3])
        // {
        //     deserialize(data[0]);
        //     deserialize(data[1]);
        //     deserialize(data[2]);
        // }

        // void deserialize(float data[4])
        // {
        //     deserialize(data[0]);
        //     deserialize(data[1]);
        //     deserialize(data[2]);
        //     deserialize(data[3]);
        // }

        //+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~+-=~
      private:
        uint8_t *pBuf        = nullptr;
        int      buf_offset  = 0;
        int      buf_size    = 256;
        uint8_t  result_code = 0;

        void serialize(const uint8_t *pData, size_t data_size)
        {
            if ((buf_offset + ((int)data_size)) >= buf_size)
                {
                    result_code = 1;
                    return;
                }
            uint8_t *p = pBuf + buf_offset;
            for (int i = 0; i < ((int)data_size); i++) { *p++ = *pData++; }
            buf_offset += ((int)data_size);
        }

    };       //--- end of:  class vqwSerializer

};       // namespace vqw
