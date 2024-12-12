// VQW Pipes

#pragma once

#include <elapsedMillis.h>
#include "vqwPipe_struct.hpp"
#include "vqwPipe_Channel.hpp"
#include "vqwPipe_Chan_Mgt_Msg.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <memory>

namespace vqw
{
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //class vqwPipe_Channel;
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  /// @brief bi-directional multi-channel data pipe built on top of a vqwPipe_Driver
  //
  class vqwPipe
  {
  public:
    //static constexpr int recv_msg_max_size = 1024;
    vqwPipe(vqwPipe_Driver *pDriver_) : pDriver(pDriver_) { init();}
    vqwPipe(vqwPipe_Driver *pDriver_, const char *name) : pDriver(pDriver_), instance_name(name) { init();}
    void set_ChannelData_callback(channelNumber_t channel_number, ChannelData_callback_t cb) { ChannelData_callback_map[channel_number] = cb; }
    void set_RougeData_callback(RougeData_callback_t cb) { rougeDataCallback = cb; }
    void set_DebugMessageCallback(DebugMessageCallback_t cb) { debugMessageCallback = cb; }

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
      uint8_t buf[ChannelDataHeader_t::recv_msg_max_size + sizeof(ChannelDataMsgHeader_t) + 1];
      std::size_t sz_bytes = pMsg->serialize(buf + sizeof(ChannelDataMsgHeader_t), ChannelDataHeader_t::recv_msg_max_size);
      ChannelDataMsgHeader_t *pCDM = (ChannelDataMsgHeader_t *)buf;
      pCDM->hdr.msgSize = sz_bytes + sizeof(ChannelDataMsgHeader_t);
      pCDM->hdr.channelNumber = channel_number;
      pCDM->msgType = MsgType::PRIVATE_DTO_ID;
      return sendChannelDataMsg((const ChannelDataHeader_t *)(&(pCDM->hdr)));
    }
    //=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~
    //=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~
  private:
    void init();
    vqwPipe_Driver *pDriver = nullptr;
    std::shared_ptr<vqwPipe_Channel> pChannel_MGT;

    std::string instance_name;
    RougeData_callback_t rougeDataCallback;
    DebugMessageCallback_t debugMessageCallback;
    std::map<uint8_t, ChannelData_callback_t> ChannelData_callback_map;

    void dispatch_recieved_msg(ChannelDataHeader_t *pMsg);
    int send_HeartbeatMsg();

    elapsedMillis elap_since_last_recv;

    elapsedMillis elap_since_heartbeat_sent;
    elapsedMillis elap_since_heartbeat_recv;
    uint32_t heartbeet_interval_ms = 1800;
    int heartbeet_sequence_number = 0;
    long heartbeat_total_sent = 0;
    long heartbeat_total_recv = 0;
    int heartbeat_recv_sequence_number = 0;

    void on_vqwPipe_chan_mgt_heart_beat(const vqwPipe_chan_mgt_heart_beat *pData);

    ChannelDataHeader_t *recv_msg(); // returns nullptr if no message is available
    void empty_recv_buf_to_rougeDataCallback();
    void send_to_rougeDataCallback(uint8_t c);
    int error_count_msg_too_big = 0;
    int error_count_bad_crc = 0;
    int error_count_bad_suffix = 0;
    int recv_msg_state = 0;
    int recv_msg_index = 0;
    int recv_msg_length = 0;
    uint8_t recv_msg_buf[ChannelDataHeader_t::recv_msg_max_size + 4];

  }; // end of:  class vqwPipe

}; // namespace vqw
