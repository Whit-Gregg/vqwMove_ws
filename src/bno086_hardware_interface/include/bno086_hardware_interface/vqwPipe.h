// VQW Pipes

#pragma once

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <DebugLog.h>
// // #define is_Teensy 1
// // #else
// // #define is_ESP32 1
#endif

#include <elapsedMillis.h>

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
    int16_t msgSize; // msg size, which includes this struct's size
    channelNumber_t channelNumber;
  };

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
  };

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  using ChannelData_callback_t = std::function<void(const ChannelDataHeader_t *pData)>;
  using RougeData_callback_t = std::function<void(const uint8_t *pData, int dataSize)>;
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

    virtual size_t readBytes(uint8_t *buffer, size_t length) = 0;
    virtual size_t writeBytes(uint8_t *buffer, size_t length) = 0;

    virtual void loop() = 0;
  };
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
#ifdef is_Teensy
  class vqwPipe_Driver_Teensy : public vqwPipe_Driver
  {
  public:
    vqwPipe_Driver_Teensy(HardwareSerial *pSerial_) : pSerial(pSerial_) {}
    virtual int available(void) { return pSerial->available(); }
    virtual int availableForWrite(void) { return pSerial->availableForWrite(); }
    virtual size_t readBytes(uint8_t *buffer, size_t length)
    {
      elap_since_last_recv = 0;
      return pSerial->readBytes(buffer, length);
    }
    virtual size_t writeBytes(uint8_t *buffer, size_t length) { return pSerial->write(buffer, length); }
    virtual void loop() {};

    uint32_t get_elap_since_last_recv() { return (uint32_t)elap_since_last_recv; }
    void reset_elap_since_last_recv(uint32_t elap = 0) { elap_since_last_recv = elap; }

  private:
    HardwareSerial *pSerial = nullptr;
    elapsedMillis elap_since_last_recv;
  };
#endif
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  // #ifdef is_ESP32
  //     class vqwPipe_Driver_ESP32 : public vqwPipe_Driver
  //     {
  //       public:
  //         vqwPipe_Driver_ESP32(int SerialPort_) : SerialPort(SerialPort_) {}
  //         virtual int    available(void) { ? ? ? }
  //         virtual int    availableForWrite(void) { ? ? ? }
  //         virtual size_t readBytes(uint8_t *buffer, size_t length) { ? ? ? }
  //         virtual size_t writeBytes(uint8_t *buffer, size_t length) { ? ? ? }

  //       private:
  //         int SerialPort;
  //     };
  // #endif
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  /// @brief bi-directional multi-channel data pipe built on top of a vqwPipe_Driver
  class vqwPipe
  {
  public:
    static constexpr int recv_msg_max_size = 1024;
    vqwPipe(vqwPipe_Driver *pDriver_) : pDriver(pDriver_) {}
    vqwPipe(vqwPipe_Driver *pDriver_, const char *name) : pDriver(pDriver_), instance_name(name) {}
    void set_ChannelData_callback(channelNumber_t channel_number, ChannelData_callback_t cb) { ChannelData_callback_map[channel_number] = cb; }
    void set_RougeData_callback(RougeData_callback_t cb) { rougeDataCallback = cb; }

    void loop(); // call this as often as you can.... this drives the msg recv callback methods.

    int sendChannelDataMsg(const ChannelDataHeader_t *pMsg); // returns msg size, or zero if msg not sent.
    int sendRougeData(uint8_t *pData, int dataSize);
        uint32_t get_elap_since_last_recv() { return ((uint32_t)elap_since_last_recv); }


    //======================================================================================
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //~~~~~ the function below requires that the class of the pMsg parameter have
    //~~~~~ a member that declares the MsgType of itself, named PRIVATE_DTO_ID, for example:
    //~~~~~
    //~~~~~     static constexpr std::uint8_t PRIVATE_DTO_ID = 77;
    //~~~~~
    //~~~~~  and
    //~~~~~
    //~~~~~     the class has a method:  int serialize(uint8_t *pOutBuffer, int OutBufferSize);
    //~~~~~     which returns the number of bytes written into pOutBuffer
    //~~~~~
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    template <typename MsgType>
    int Send_ChannelData_message(MsgType *pMsg, channelNumber_t channel_number)
    {
      uint8_t buf[recv_msg_max_size];
      std::size_t sz_bytes = pMsg->serialize(buf + sizeof(ChannelDataMsgHeader_t), recv_msg_max_size);
      ChannelDataMsgHeader_t *pCDM = (ChannelDataMsgHeader_t *)buf;
      pCDM->hdr.msgSize = sz_bytes + sizeof(ChannelDataMsgHeader_t);
      pCDM->hdr.channelNumber = channel_number;
      pCDM->msgType = MsgType::PRIVATE_DTO_ID;
      return sendChannelDataMsg((const ChannelDataHeader_t *)(&(pCDM->hdr)));
    }
    //=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~
    //=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~
  private:
    vqwPipe_Driver *pDriver = nullptr;
    std::string instance_name;
    RougeData_callback_t rougeDataCallback;
    std::map<uint8_t, ChannelData_callback_t> ChannelData_callback_map;

  elapsedMillis elap_since_last_recv;

    elapsedMillis elap_since_heartbeat_sent;
    uint32_t heartbeet_interval_ms = 800;
    int heartbeet_sequence_number = 0;

    ChannelDataHeader_t *recv_msg(); // returns nullptr if no message is available
    void empty_recv_buf_to_rougeDataCallback();
    void send_to_rougeDataCallback(uint8_t c);
    int error_count_msg_too_big = 0;
    int error_count_bad_crc = 0;
    int error_count_bad_suffix = 0;
    int recv_msg_state = 0;
    int recv_msg_index = 0;
    int recv_msg_length = 0;
    uint8_t recv_msg_buf[recv_msg_max_size + 4];

  }; // end of:  class vqwPipe

}; // namespace vqw
