#include "remote_test/uart_driver.hpp"
#include <string>

constexpr const char * UartDriver::dev_null;
constexpr uint32_t UartDriver::baud;
constexpr FlowControl UartDriver::fc;
constexpr Parity UartDriver::pt;
constexpr StopBits UartDriver::sb;

UartDriver::UartDriver(std::string dev_name)
{
    this->dev_name = dev_name;
    printf("creating uart driver\n");
    IoContext ctx;
    SerialPortConfig config(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(ctx, dev_name, config);
    port_->open();
    printf("port opened\n");
}

UartDriver::~UartDriver()
{
    port_->close();
    printf("port closed\n");
}

void UartDriver::send(const std::vector<uint8_t> &data)
{
    // send serial data
    port_->send(data);
}

void UartDriver::read(std::vector<uint8_t> &data)
{
    // read serial data
    port_->receive(data);
}