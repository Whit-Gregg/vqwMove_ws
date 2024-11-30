#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

#include <vqwPipe.h>
#include <vqwPipe_Channel.h>
//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
class vqwPipe_chan_IMU_data_msg
{
  public:
    static constexpr std::uint8_t PRIVATE_DTO_ID             = 11;
    
    static constexpr std::uint8_t data_type_code_Accel       = 1;
    static constexpr std::uint8_t data_type_code_Gyro        = 2;
    static constexpr std::uint8_t data_type_code_Mag         = 3;
    static constexpr std::uint8_t data_type_code_LinAccel    = 4;
    static constexpr std::uint8_t data_type_code_Orientation = 5;

    //========= fields =================================
    std::uint8_t data_type_code;
    float        Accuracy;
    float        X;
    float        Y;
    float        Z;
    float        W;
    //========= end of fields ==========================

    int serialize(uint8_t *pOutBuffer, int OutBufferSize) const       // return number of bytes written into OutBuffer
    {
        vqw::vqwSerializer Ser(pOutBuffer, OutBufferSize);
        Ser.serialize(data_type_code);
        Ser.serialize(Accuracy);
        Ser.serialize(X);
        Ser.serialize(Y);
        Ser.serialize(Z);
        if (data_type_code == data_type_code_Orientation) Ser.serialize(W);
        return Ser.get_byteCount();
    }

    int deserialize(const vqw::ChannelDataMsgHeader_t *pCDM)       // return zero for good, non-zero for error
    {
        vqw::vqwSerializer Ser(pCDM);
        Ser.deserialize(data_type_code);
        Ser.deserialize(Accuracy);
        Ser.deserialize(X);
        Ser.deserialize(Y);
        Ser.deserialize(Z);
        if (data_type_code == data_type_code_Orientation) Ser.deserialize(W);
        return Ser.get_result_code();
    }

    const char *data_type_name(const std::uint8_t data_type_code_) const
    {
        const char *p = "?";
        switch (data_type_code_)
            {
                case data_type_code_Accel: p = "Accel"; break;
                case data_type_code_Gyro: p = "Gyro"; break;
                case data_type_code_Mag: p = "Mag"; break;
                case data_type_code_LinAccel: p = "LinAccel"; break;
                case data_type_code_Orientation: p = "Orientation"; break;
                default: break;
            }
        return p;
    }

    std::string ToString() const
    {
        char        buf[256] {0};
        const char *pName = data_type_name(data_type_code);
        sprintf(buf, "vqwPipe_chan_IMU_data_msg: DataType=%s  X=%.3f Y=%.3f Z=%.3f", pName, X, Y, Z);
        if (data_type_code == data_type_code_Orientation) sprintf(buf + strlen(buf), " W=%.3f", W);
        return buf;
    }

};       //---- end of:  class vqwPipe_chan_IMU_data_msg
//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
class vqwPipe_ROS_CLI_msg
{
  public:
    static constexpr std::uint8_t PRIVATE_DTO_ID = 12;

    //========= fields =================================
    vqw::vqwPipe_Direction direction = vqw::vqwPipe_Direction::Request;
    std::string            text;
    //========= end of fields ==========================

    int serialize(uint8_t *pOutBuffer, int OutBufferSize) const       // return number of bytes written into OutBuffer
    {
        vqw::vqwSerializer Ser(pOutBuffer, OutBufferSize);
        Ser.serialize(direction);
        Ser.serialize(text);
        return Ser.get_byteCount();
    }

    int deserialize(const vqw::ChannelDataMsgHeader_t *pCDM)       // return zero for good, non-zero for error
    {
        vqw::vqwSerializer Ser(pCDM);
        Ser.deserialize(direction);
        Ser.deserialize(text);
        return Ser.get_result_code();
    }

    std::string ToString() const
    {
        char buf[256] {0};
        sprintf(buf, "vqwPipe_ROS_CLI_msg: direction=%s,  TxT=\"%s\"", ((direction == vqw::vqwPipe_Direction::Request) ? "Request" : "Responce"), text.c_str());
        return buf;
    }
};       //---- end of:  class vqwPipe_ROS_CLI_msg

//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
class vqwPipe_ROS_DebugLog_msg
{
  public:
    static constexpr std::uint8_t PRIVATE_DTO_ID = 13;

    //========= fields =================================
    std::string text;
    //========= end of fields ==========================

    int serialize(uint8_t *pOutBuffer, int OutBufferSize) const       // return number of bytes written into OutBuffer
    {
        vqw::vqwSerializer Ser(pOutBuffer, OutBufferSize);
        Ser.serialize(text);
        return Ser.get_byteCount();
    }

    int deserialize(const vqw::ChannelDataMsgHeader_t *pCDM)       // return zero for good, non-zero for error
    {
        vqw::vqwSerializer Ser(pCDM);
        Ser.deserialize(text);
        return Ser.get_result_code();
    }

    std::string ToString() const
    {
        char buf[512] {0};
        if (text.size() < 450) { sprintf(buf, "vqwPipe_ROS_DebugLog_msg: TxT=\"%s\"", text.c_str()); }
        return buf;
    }
};       //---- end of:  class vqwPipe_ROS_CLI_msg

//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
//-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~