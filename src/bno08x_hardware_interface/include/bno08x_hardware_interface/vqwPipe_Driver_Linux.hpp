#pragma once

#include "vqwPipe_struct.hpp"

namespace vqw
{

  class vqwPipe_Driver_Linux : public vqwPipe_Driver
  {
  public:
    // Returns the number of bytes which have been received and
    // can be fetched with readBytes().
    virtual int available(void) override;

    // Returns the number of bytes which may be transmitted by writeBytes() without waiting.
    virtual int availableForWrite(void) override;

    virtual bool readByte(uint8_t &pByte) override;
    virtual size_t readBytes(uint8_t *buffer, size_t length) override;
    virtual size_t writeBytes(uint8_t *buffer, size_t length) override;

    virtual void loop();

    //-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~-=+~
    bool connected() const { return serial_port_fd != -1; }
    bool connect(const std::string &device_name, int speed_baud);
    void disconnect();
    bool restart();

    void set_DebugMessageCallback(DebugMessageCallback_t cb) { debugMessageCallback = cb; }

  private:
    std::string serial_port_name;
    int serial_port_speed = 0;
    int serial_port_fd = -1;
    const int linux_serial_port_output_buffer_size = 4096;

    DebugMessageCallback_t debugMessageCallback;

#define InboundDataBufferSize 256

    uint8_t InboundDataBuffer[InboundDataBufferSize + 4];
    int InboundDataBufferIndex = 0;
    int InboundDataBufferCount = 0;

    long count_of_available_true = 0;
    long count_of_available_false = 0;
    long count_of_available_for_write_true = 0;
    long count_of_available_for_write_false = 0;
    long count_of_available_MOD = 300;
    long count_of_available_for_write_MOD = 300;
    long count_of_read_errors = 0;
    long count_of_read_errors_MOD = 300;
    long count_of_bytes_read = 0;

    void setSerialDeviceOptions();

  }; // end of: class vqwPipe_Driver_Linux

} // namespace vqw