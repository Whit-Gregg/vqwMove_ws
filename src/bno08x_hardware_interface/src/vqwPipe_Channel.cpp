// this is:  vqwPipe_Channel.cpp

#include "vqwPipe.hpp"
#include "vqwPipe_Channel.hpp"

namespace vqw
{


void vqwPipe_Channel::init()
{
    // vqwPipe_Channel channel_MGT(vqwPipe_Channel_Number_Assignments::Channel_Mgt, this, "MGT");
    // pChannel_MGT = std::make_shared<vqwPipe_Channel>(vqwPipe_Channel_Number_Assignments::Channel_Mgt, this, "MGT");
    pPipe->set_ChannelData_callback(channel_number, [this](const ChannelDataHeader_t *pData)
                                    { this->myChannelData_callback(pData); });
}

int vqwPipe_Channel::sendChannelDataMsg(const ChannelDataHeader_t *pMsg)
{
    return pPipe->sendChannelDataMsg(pMsg);
}

} // end of:   namespace vqw