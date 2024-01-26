#include "remote_test/uart_driver.hpp"

constexpr const char * UartDriver::dev_name;
constexpr const char * UartDriver::dev_null;
constexpr uint32_t UartDriver::baud;
constexpr FlowControl UartDriver::fc;
constexpr Parity UartDriver::pt;
constexpr StopBits UartDriver::sb;

UartDriver::UartDriver()
{
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

void UartDriver::send(const std::string & data)
{
    // send serial data
    std::vector<uint8_t> buffer(data.begin(), data.end());
    port_->send(buffer);
}

void UartDriver::read(std::string &data)
{
    // read serial data
    std::vector<uint8_t> buffer;
    port_->receive(buffer);
    data = std::string(buffer.begin(), buffer.end());
}