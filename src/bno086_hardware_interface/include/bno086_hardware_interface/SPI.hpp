// SPI.hpp
#pragma once

#include <cstdint>
#include <string>

#include <linux/spi/spidev.h>

namespace bno086_hardware_interface
{
    // see: www.kernal.org/doc/Documentation/spi/spidev

    class SPIClass
    {
    public:
        virtual ~SPIClass(){ if (spi_fd >= 0) Close(); }

        int Open(const char *devName, uint32_t spi_speed=spiSpeed_default);

        void Close();

        int Read(std::uint8_t *pBuffer, int bufferSize); // returns # of bytes read, or < 0 for error

        int Write(const std::uint8_t *pBuffer, int bufferSize); // returns < 0 for error

        std::string get_spiDeviceName() const { return spiDeviceName; }
        uint32_t get_spiSpeed() const { return spiSpeed; }

        static constexpr const char *const spiDevice_default = "/dev/spidev0.0";
        static constexpr const uint32_t spiSpeed_default = 1000000;
        uint8_t spiMode = SPI_MODE_3;
        uint8_t spiBPW = 8;
        uint16_t spiDelay = 0;

    private:
        int spi_fd = -1;
        std::string spiDeviceName;
        uint32_t spiSpeed = spiSpeed_default;
        struct spi_ioc_transfer xfer;
    };
} // namespace bno086_hardware_interface