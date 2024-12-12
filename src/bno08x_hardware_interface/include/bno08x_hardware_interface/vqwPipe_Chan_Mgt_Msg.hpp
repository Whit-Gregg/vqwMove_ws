#pragma once
#include <cstdint>

#include "vqwPipe_struct.hpp"
#include "vqwSerializer.hpp"
#include <string>
#include <vector>

// vqwPipe_Chan_Mgt_Msg.h

namespace vqw
{

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    class vqwPipe_chan_mgt_heart_beat
    {
      public:
        static constexpr std::uint8_t PRIVATE_DTO_ID = 1;

        //========= fields =================================
        vqwPipe_Direction direction   = vqwPipe_Direction::Request;
        int           sequence_number = 0;
        uint8_t           result_code = 0;
        std::string       result_text;
        //========= end of fields ==========================

        int serialize(uint8_t *pOutBuffer, int OutBufferSize) const       // return number of bytes written into OutBuffer
        {
            vqwSerializer Ser(pOutBuffer, OutBufferSize);
            Ser.serialize(direction);
            Ser.serialize(sequence_number);
            Ser.serialize(result_code);
            Ser.serialize(result_text);
            return Ser.get_byteCount();
        }

        int deserialize(const ChannelDataMsgHeader_t *pCDM)       // return zero for good, non-zero for error
        {
            vqwSerializer Ser(pCDM);
            Ser.deserialize(direction);
            Ser.deserialize(sequence_number);
            Ser.deserialize(result_code);
            Ser.deserialize(result_text);
            return Ser.get_result_code();
        }

        std::string ToString() const
        {
            char buf[512] {0};
            sprintf(buf, "vqwPipe_chan_mgt_heart_beat: %s sequence_number=%d  rc=%d  %s", (direction == vqwPipe_Direction::Request) ? "Request" : "Response",
                    sequence_number, result_code, result_text.c_str());
            return buf;
        }
    };       //---- end of:  class vqwPipe_chan_mgt_heart_beat

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    class vqwPipe_chan_mgt_debug_level
    {
      public:
        static constexpr std::uint8_t PRIVATE_DTO_ID = 2;

        //========= fields =================================
        vqwPipe_Direction direction   = vqwPipe_Direction::Request;
        uint8_t           debug_level = 0;
        uint8_t           result_code = 0;
        std::string       result_text;
        //========= end of fields ==========================

        int serialize(uint8_t *pOutBuffer, int OutBufferSize) const       // return number of bytes written into OutBuffer
        {
            vqwSerializer Ser(pOutBuffer, OutBufferSize);
            Ser.serialize(direction);
            Ser.serialize(debug_level);
            Ser.serialize(result_code);
            Ser.serialize(result_text);
            return Ser.get_byteCount();
        }

        int deserialize(const ChannelDataMsgHeader_t *pCDM)       // return zero for good, non-zero for error
        {
            vqwSerializer Ser(pCDM);
            Ser.deserialize(direction);
            Ser.deserialize(debug_level);
            Ser.deserialize(result_code);
            Ser.deserialize(result_text);
            return Ser.get_result_code();
        }

        std::string ToString() const
        {
            char buf[512] {0};
            sprintf(buf, "vqwPipe_chan_mgt_debug_level: %s debug_level=%d  rc=%d  %s", (direction == vqwPipe_Direction::Request) ? "Request" : "Response",
                    debug_level, result_code, result_text.c_str());
            return buf;
        }
    };       //---- end of:  class vqwPipe_chan_mgt_debug_level

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    class vqwPipe_chan_mgt_channel_assign
    {
      public:
        static constexpr std::uint8_t PRIVATE_DTO_ID = 3;

        //========= fields =================================
        vqwPipe_Direction direction      = vqwPipe_Direction::Request;
        uint8_t           channel_number = 0;
        std::string       channel_type_name;
        std::string       channel_instance_name;
        uint8_t           result_code = 0;
        std::string       result_text;
        //========= end of fields ==========================

        int serialize(uint8_t *pOutBuffer, int OutBufferSize) const       // return number of bytes written into OutBuffer
        {
            vqwSerializer Ser(pOutBuffer, OutBufferSize);
            Ser.serialize(direction);
            Ser.serialize(channel_number);
            Ser.serialize(channel_type_name);
            Ser.serialize(channel_instance_name);
            Ser.serialize(result_code);
            Ser.serialize(result_text);
            return Ser.get_byteCount();
        }

        int deserialize(const ChannelDataMsgHeader_t *pCDM)       // return zero for good, non-zero for error
        {
            vqwSerializer Ser(pCDM);
            Ser.deserialize(direction);
            Ser.deserialize(channel_number);
            Ser.deserialize(channel_type_name);
            Ser.deserialize(channel_instance_name);
            Ser.deserialize(result_code);
            Ser.deserialize(result_text);
            return Ser.get_result_code();
        }

        std::string ToString() const
        {
            char buf[512] {0};
            sprintf(buf, "vqwPipe_chan_mgt_channel_assign: %s channel_number=%d type_name=%s instance_name=%s  rc=%d  %s",
                    (direction == vqwPipe_Direction::Request) ? "Request" : "Response", channel_number, channel_type_name.c_str(),
                    channel_instance_name.c_str(), result_code, result_text.c_str());
            return buf;
        }
    };       //---- end of:  class vqwPipe_chan_mgt_channel_assign

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~



    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    class vqwPipe_chan_Echo
    {
      public:
        static constexpr std::uint8_t PRIVATE_DTO_ID = 4;

        //========= fields =================================
        vqwPipe_Direction direction   = vqwPipe_Direction::Request;
        std::string       text;
        //========= end of fields ==========================

        int serialize(uint8_t *pOutBuffer, int OutBufferSize) const       // return number of bytes written into OutBuffer
        {
            vqwSerializer Ser(pOutBuffer, OutBufferSize);
            Ser.serialize(direction);
            Ser.serialize(text);
            return Ser.get_byteCount();
        }

        int deserialize(const ChannelDataMsgHeader_t *pCDM)       // return zero for good, non-zero for error
        {
            vqwSerializer Ser(pCDM);
            Ser.deserialize(direction);
            Ser.deserialize(text);
            return Ser.get_result_code();
        }

        std::string ToString() const
        {
            char buf[512] {0};
            sprintf(buf, "vqwPipe_chan_Echo: %s   Echo==>%s", (direction == vqwPipe_Direction::Request) ? "Request" : "Response",
                    text.c_str());
            return buf;
        }
    };       //---- end of:  class vqwPipe_chan_Echo

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~




};       // namespace vqw

//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
