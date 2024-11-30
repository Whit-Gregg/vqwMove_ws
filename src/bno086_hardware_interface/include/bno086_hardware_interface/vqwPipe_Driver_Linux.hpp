#pragma once

#include "vqwPipe.h"

namespace vqw
{

    class vqwPipe_Driver_Linux : public vqwPipe_Driver
    {
      public:
        // Returns the number of bytes which have been received and
        // can be fetched with readBytes().
        virtual  int available(void);

        // Returns the number of bytes which may be transmitted by writeBytes() without waiting.
        virtual  int availableForWrite(void);

        virtual  size_t readBytes(uint8_t *buffer, size_t length);
        virtual  size_t writeBytes(uint8_t *buffer, size_t length);

        virtual  void loop();

        //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
        bool connected() const { return connected_; }
        bool connect(const std::string &device_name, int speed_baud);
        void disconnect();
        bool restart();

      private:
        std::string serial_port_name;
        int         fd_               = -1;
        bool        connected_        = false;
        int         serial_port_speed = 0;
        int         serial_port_fd    = -1;

        void setSerialDeviceOptions();

    };       // end of: class vqwPipe_Driver_Linux

}       // namespace vqw