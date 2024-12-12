#pragma once
// #include "vqwPipe.hpp"
#include "vqwSerializer.hpp"
#include "vqwMsgCallbackHolder.hpp"
#include <map>
#include <string>
#include <vector>

namespace vqw
{
  class vqwPipe;

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~
  //-=+~ vqwPipe_Channel
  //-=+~
  class vqwPipe_Channel
  {
  public:
    vqwPipe_Channel(channelNumber_t chan_numb, vqwPipe *pPipe_) : channel_number(chan_numb), pPipe(pPipe_) { init(); }
    vqwPipe_Channel(channelNumber_t chan_numb, vqwPipe *pPipe_, const char *name) : channel_number(chan_numb),
                                                                                    pPipe(pPipe_), instance_name(name) { init(); }


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
    int Send_ChannelData_message(MsgType *pMsg)
    {
      const int hdr_size = sizeof(ChannelDataMsgHeader_t);
      uint8_t buf[ChannelDataHeader_t::recv_msg_max_size];
      std::size_t sz_bytes = pMsg->serialize(buf + hdr_size, (int)(sizeof(buf) - hdr_size));
      ChannelDataMsgHeader_t *pCDM = (ChannelDataMsgHeader_t *)buf;
      pCDM->hdr.msgSize = sz_bytes + hdr_size;
      pCDM->hdr.channelNumber = channel_number;
      pCDM->msgType = MsgType::PRIVATE_DTO_ID;
      return sendChannelDataMsg((const ChannelDataHeader_t *)(&(pCDM->hdr)));
    }
    //=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~=+-~
    int sendChannelDataMsg(const ChannelDataHeader_t *pMsg);

    template <class T>
    void addCallback(std::function<void(const T *)> cb)
    {
      uint8_t msg_type = T::PRIVATE_DTO_ID;
      vqwMsgCallbackHolder<T> *pCbHolder = new vqwMsgCallbackHolder<T>(cb);
      cb_map[msg_type] = static_cast<vqwMsgCallbackHolder_base *>(pCbHolder);
    }

    channelNumber_t channel_number;

  private:
    vqwPipe *pPipe = nullptr;
    std::string instance_name;
    std::map<uint8_t, vqwMsgCallbackHolder_base *> cb_map;
    void init();

    void myChannelData_callback(const ChannelDataHeader_t *pData)
    {
      const ChannelDataMsgHeader_t *pDataMsg = (const ChannelDataMsgHeader_t *)(pData);
      uint8_t msg_type = pDataMsg->msgType;
      auto cbh = cb_map.find(msg_type);
      if (cbh != cb_map.end())
      {
        vqwMsgCallbackHolder_base *pCBH = cbh->second;
        pCBH->Dispatch(pDataMsg);
      }
    }
  }; //--- end of:  class vqwPipe_Channel

  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
  //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-

}; // namespace vqw
