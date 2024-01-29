#include "serial_driver/serial_driver.hpp"
#include <serial_driver/serial_port.hpp>
#include <string>
#include <sys/socket.h>

using spb = asio::serial_port_base;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::StopBits;
using drivers::serial_driver::SerialPortConfig;

class UartDriver
{
public:
    UartDriver(std::string dev_name);

    ~UartDriver();

    void send(const std::vector<uint8_t> &data);

    void read(std::vector<uint8_t> &data);

private:
    std::string dev_name;
    static constexpr const char * dev_null = "/dev/null";
    static constexpr uint32_t baud = 115200;
    static constexpr FlowControl fc = FlowControl::NONE;
    static constexpr Parity pt = Parity::NONE;
    static constexpr StopBits sb = StopBits::ONE;
    std::unique_ptr<SerialPort> port_;
    std::vector<uint8_t> send_recv_buff;
};