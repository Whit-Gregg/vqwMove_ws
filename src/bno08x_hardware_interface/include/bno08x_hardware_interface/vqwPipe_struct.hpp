#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>

namespace vqw
{
  enum class vqwPipe_Channel_Number_Assignments
  {
    Channel_Mgt = 0,
    Channel_WiFi = 1,
    Channel_SPP = 2,
    Channel_BLE = 3,
    Channel_ROS = 4,
    Channel_Debug = 5
  };

  using channelNumber_t = uint8_t;
  using channelMsgType_t = uint8_t;

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=
  //-=+~-=  application [byte streams] should derive from this struct
  //-=+~-=
  using ChannelDataHeader_t = struct ChannelDataHeader_struct
  {
    static constexpr int recv_msg_max_size = 1024;

    int16_t msgSize; // msg size, which includes this struct's size
    channelNumber_t channelNumber;
  } __attribute__((packed));

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=
  //-=+~-=  application [messages] should derive from this struct
  //-=+~-=
  using ChannelDataMsgHeader_t = struct ChannelDataMsgHeader_struct
  {
    ChannelDataHeader_t hdr;
    channelNumber_t msgType;
    // msgData is next.... (ie. the serialized content of an application message)
    // uint8_t msgData[8];       // actual size will vary.....
  } __attribute__((packed));

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  using ChannelData_callback_t = std::function<void(const ChannelDataHeader_t *pData)>;
  using RougeData_callback_t = std::function<void(const uint8_t *pData, int dataSize)>;
  using DebugMessageCallback_t = std::function<void(std::string msg)>;
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  enum class vqwPipe_Direction : int8_t
  {
    Request = 1,
    Response = 2
  };

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  class vqwPipe_Driver
  {
  public:
    // Returns the number of bytes which have been received and
    // can be fetched with readBytes().
    virtual int available(void) = 0;

    // Returns the number of bytes which may be transmitted by writeBytes() without waiting.
    virtual int availableForWrite(void) = 0;

    virtual bool readByte(uint8_t &the_byte) = 0;
    virtual size_t readBytes(uint8_t *buffer, size_t length) = 0;
    virtual size_t writeBytes(uint8_t *buffer, size_t length) = 0;

    virtual void loop() = 0;
  };
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
} // namespace vqw