#include "vqwPipe.hpp"
#include "vqwPipe_Chan_Mgt_Msg.hpp"

#include <CRC8.h>

#define VQW_PIPE_PREFIX_BYTE_1 '$'
#define VQW_PIPE_PREFIX_BYTE_2 '!'
#define VQW_PIPE_SUFFIX_BYTE '\n'

#include <rclcpp/rclcpp.hpp>

namespace vqw
{
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    void vqwPipe::init()
    {
        pChannel_MGT = std::make_shared<vqwPipe_Channel>((channelNumber_t)vqwPipe_Channel_Number_Assignments::Channel_Mgt, this, "MGT");

        pChannel_MGT->addCallback<vqwPipe_chan_mgt_heart_beat>(
            [this](const vqwPipe_chan_mgt_heart_beat *pData)
            {
                this->on_vqwPipe_chan_mgt_heart_beat(pData);
            });
    }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    void vqwPipe::loop() // call this as often as you can....
    {
        pDriver->loop();
        int loop_count = 0;
        ChannelDataHeader_t *pMsg = recv_msg();          // returns nullptr if no message is available
        while ((pMsg != nullptr) && (loop_count++ < 10)) // process up to 10 messages per loop
        {
            dispatch_recieved_msg(pMsg);
            pMsg = recv_msg(); // returns nullptr if no message is available
        }
        if (pMsg != nullptr)
            dispatch_recieved_msg(pMsg);

        //...... send heartbeats ......
        if (elap_since_heartbeat_sent > heartbeet_interval_ms)
        {
            elap_since_heartbeat_sent = 0;
            int rc = send_HeartbeatMsg();

            // #if defined(__linux__)
            if ((heartbeat_total_sent < 10) || ((heartbeat_total_sent % 100) == 0) || (rc < 1))
            {
                RCLCPP_INFO(rclcpp::get_logger("vqwPipe"), "vqwPipe::loop() Heartbeat sent.  Seq=%d   rc=%d", (heartbeet_sequence_number - 1), rc);
            }
            // #endif
        }
    }

    void vqwPipe::on_vqwPipe_chan_mgt_heart_beat(const vqwPipe_chan_mgt_heart_beat *pData)
    {
        if (pData->direction == vqwPipe_Direction::Response)
        {
            // send_ChannelData_message(pData, (channelNumber_t)vqwPipe_Channel_Number_Assignments::Channel_Mgt);
        }
        if (pData->direction == vqwPipe_Direction::Request)
        {
            elap_since_last_recv = 0;
            elap_since_heartbeat_recv = 0;
            heartbeat_total_recv++;
            int expected_sequence_number = heartbeat_recv_sequence_number + 1;
            if (pData->sequence_number != expected_sequence_number)
            {
                RCLCPP_WARN(rclcpp::get_logger("vqwPipe"), "on_vqwPipe_chan_mgt_heart_beat() Expected seq = %d   but recv'd seq = %d  !!!!!!!!!!", expected_sequence_number, pData->sequence_number);
            }   
            heartbeat_recv_sequence_number = pData->sequence_number;
            if ((heartbeat_total_recv < 5) || ((heartbeat_total_recv % 1000) == 0))
            {
                RCLCPP_INFO(rclcpp::get_logger("vqwPipe"), "on_vqwPipe_chan_mgt_heart_beat()  %s", pData->ToString().c_str());
            }
        }
    }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    int vqwPipe::send_HeartbeatMsg()
    {
        heartbeat_total_sent++;
        vqwPipe_chan_mgt_heart_beat HB_msg;
        HB_msg.direction = vqwPipe_Direction::Request;
        HB_msg.sequence_number = heartbeet_sequence_number++;
        HB_msg.result_code = 0;
        HB_msg.result_text = "";

        return Send_ChannelData_message(&HB_msg, (channelNumber_t)vqwPipe_Channel_Number_Assignments::Channel_Mgt);
    }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    void vqwPipe::dispatch_recieved_msg(ChannelDataHeader_t *pMsg)
    {
        if (pMsg != nullptr)
        {
            if (debugMessageCallback != nullptr)
            {
                char buf[128];
                sprintf(buf, "vqwPipe::dispatch_recieved_msg()  Channel=%d  MsgSize=%d", pMsg->channelNumber, pMsg->msgSize);
                debugMessageCallback(buf);
            }
            channelNumber_t channelNumber = pMsg->channelNumber;
            auto cbb = ChannelData_callback_map.find(channelNumber);
            if (cbb != ChannelData_callback_map.end())
            {
                cbb->second(pMsg);
            }
        }
    }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //============================================================
    // output format:
    //		VQW_PIPE_PREFIX_BYTE_1, VQW_PIPE_PREFIX_BYTE_2  ("$!")
    //---------- crc starts here ---------
    //		msg_size	    int16_t		= payload size + sizeof(ChannelDataMsgHeader_t)
    //      channel_number  uint8_t
    //      message_type    uint8_t
    //		payload:
    //			---------- payload starts here ---------
    //       payload        uint8_t[payload_size]
    //			---------- payload ends here ---------
    //---------- crc ends here ---------
    //		crc				uint8_t
    //		"/n"
    //============================================================
    // notes:
    //      lead_in is 2 bytes VQW_PIPE_PREFIX_BYTE_1, VQW_PIPE_PREFIX_BYTE_2  ("$!")
    //      ChannelDataMsgHeader_t is 4 bytes
    //      payload is variable size
    //      crc is 1 byte
    //      trailer is 1 byte "\n"
    //
    //		total message length is payload_data_size + sizeof(ChannelDataMsgHeader_t) + 4
    //		total message length is payload_data_size + 8
    //		crc byte count is payload_data_size + sizeof(ChannelDataMsgHeader_t)
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    int vqwPipe::sendChannelDataMsg(const ChannelDataHeader_t *pMsg)
    {
        int16_t length = pMsg->msgSize;
        int available_buffer_space = pDriver->availableForWrite();
        if (available_buffer_space < (length + 4))
            return -1;
        uint8_t crc = Crc8Block(0, ((uint8_t *)pMsg), length);
        int bytes_sent = 0;

        //...... send msg prefix ......
        uint8_t temp_buf[8];
        temp_buf[0] = VQW_PIPE_PREFIX_BYTE_1;
        temp_buf[1] = VQW_PIPE_PREFIX_BYTE_2;
        bytes_sent = pDriver->writeBytes(temp_buf, 2);
        if (bytes_sent != 2)
        {
#ifdef is_Teensy
            DebugLog.println("vqwPipe::sendChannelDataMsg() #1 pDriver->writeBytes() FAILED!!");
#endif
#ifdef is_ESP32
            ESP_LOGE(TAG, "vqwPipe::sendChannelDataMsg() #1 pDriver->writeBytes() FAILED!!");
#endif
            return -2;
        }

        //...... send msg body ......
        bytes_sent = pDriver->writeBytes(((uint8_t *)pMsg), length);
        if (bytes_sent != length)
        {
#ifdef is_Teensy
            DebugLog.println("vqwPipe::sendChannelDataMsg() #2 pDriver->writeBytes() FAILED!!");
#endif
#ifdef is_ESP32
            ESP_LOGE(TAG, "vqwPipe::sendChannelDataMsg() #2 pDriver->writeBytes() FAILED!!");
#endif
            return -3;
        }

        //...... send msg suffix ......
        temp_buf[0] = crc;
        temp_buf[1] = VQW_PIPE_SUFFIX_BYTE;
        bytes_sent = pDriver->writeBytes(temp_buf, 2);
        if (bytes_sent != 2)
        {
#ifdef is_Teensy
            DebugLog.println("vqwPipe::sendChannelDataMsg() #3 pDriver->writeBytes() FAILED!!");
#endif
#ifdef is_ESP32
            ESP_LOGE(TAG, "vqwPipe::sendChannelDataMsg() #3 pDriver->writeBytes() FAILED!!");
#endif
            return -4;
        }

        return length;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    ChannelDataHeader_t *vqwPipe::recv_msg() // returns nullptr if no message is available
    {
        ChannelDataHeader_t *return_value = nullptr;
        ChannelDataHeader_t *pMsg = ((ChannelDataHeader_t *)recv_msg_buf);

        while (/*(pDriver->available() > 0) &&*/ (return_value == nullptr))
        {
            uint8_t c = 0;
            if (!pDriver->readByte(c))
            {
                break;
            }

            switch (recv_msg_state)
            {
            case 0:
                if (c == VQW_PIPE_PREFIX_BYTE_1)
                {
                    recv_msg_state++;
                }
                else
                {
                    send_to_rougeDataCallback(c);
                }
                break;
            case 1:
                if (c == VQW_PIPE_PREFIX_BYTE_2)
                {
                    recv_msg_state++;
                    recv_msg_index = 0;
                }
                else
                {
                    send_to_rougeDataCallback(VQW_PIPE_PREFIX_BYTE_1);
                    send_to_rougeDataCallback(c);
                    recv_msg_state = 0;
                }
                break;
            case 2: // read the message header byte 1
                recv_msg_index = 0;
                recv_msg_buf[recv_msg_index++] = c;
                recv_msg_state++;
                break;
            case 3: // read the message header byte 2
                recv_msg_buf[recv_msg_index++] = c;
                recv_msg_length = pMsg->msgSize;
                recv_msg_state++;
                if (recv_msg_length > ChannelDataHeader_t::recv_msg_max_size)
                {
                    error_count_msg_too_big++;
                    empty_recv_buf_to_rougeDataCallback();
                    recv_msg_state = 0;
                }
                break;
            case 4: // read the rest of the message body
                recv_msg_buf[recv_msg_index++] = c;
                if (recv_msg_index >= recv_msg_length)
                    recv_msg_state++;
                break;
            case 5: // read the crc byte
            {
                recv_msg_state++;
                uint8_t crc = Crc8Block(0, recv_msg_buf, recv_msg_length);
                if (crc != c)
                {
                    error_count_bad_crc++;
                    empty_recv_buf_to_rougeDataCallback();
                    send_to_rougeDataCallback(c);
                    recv_msg_state = 0; // if crc error, discard msg
                }
                break;
            }
            case 6: // read the suffix byte
                if (c == VQW_PIPE_SUFFIX_BYTE)
                {
                    return_value = (ChannelDataHeader_t *)recv_msg_buf;
                    elap_since_last_recv = 0;
                }
                else
                {
                    error_count_bad_suffix++;
                    empty_recv_buf_to_rougeDataCallback();
                    send_to_rougeDataCallback(c);
                }
                recv_msg_state = 0; // start next msg recv
                break;

            default:
                break;
            }
        }
        return return_value;
    }

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    int vqwPipe::sendRougeData(uint8_t *pData, int dataSize) { return (int)(pDriver->writeBytes(pData, dataSize)); }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    void vqwPipe::send_to_rougeDataCallback(uint8_t c)
    {
        if (rougeDataCallback != nullptr)
        {
            uint8_t buf[2]{0};
            buf[0] = c;
            rougeDataCallback(buf, 1);
        }
    }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    void vqwPipe::empty_recv_buf_to_rougeDataCallback()
    {
        //.....
        if ((rougeDataCallback != nullptr) && (recv_msg_index > 0))
        {
            uint8_t buf[4];
            buf[0] = VQW_PIPE_PREFIX_BYTE_1;
            buf[1] = VQW_PIPE_PREFIX_BYTE_2;
            rougeDataCallback(buf, 2);
            rougeDataCallback(recv_msg_buf, recv_msg_index);
        }
        recv_msg_index = 0;
        recv_msg_state = 0;
    }
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~

}; // namespace vqw
