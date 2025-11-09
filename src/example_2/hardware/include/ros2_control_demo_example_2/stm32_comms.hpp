#ifndef STM32_COMMS_HPP
#define STM32_COMMS_HPP

#include <libserial/SerialPort.h>
#include <string>
#include <sstream>

LibSerial::BaudRate convertBaudRate(int baudrate)
{
    switch (baudrate)
    {
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    default:
        throw LibSerial::BaudRate::BAUD_115200;
    }
}

class Stm32_comms
{
private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;

public:
    Stm32_comms(/* args */) = default;

    void connect(const std::string &serial_device, int32_t baudrate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convertBaudRate(baudrate));
    }

    void disconnect()
    {
        if (serial_conn_.IsOpen())
            serial_conn_.Close();
    }

    bool isConnected()
    {
        return serial_conn_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.FlushInputBuffer(); // just in case
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try
        {
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout)
        {
            std::cerr << "The ReadByte() call has timed out." << std::endl;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }

        return response;
    }

    void send_empty_msg()
    {
        std::string response = send_msg("\r");
    }

    void read_encoder_values(int &val_1, int &val_2)
    {
        std::string response = send_msg("r");
        std::string delimiter = " ";
        size_t del_pos = response.find(delimiter);
        std::string token_1 = response.substr(0, del_pos);
        std::string token_2 = response.substr(del_pos + delimiter.length());
        val_1 = std::atoi(token_1.c_str());
        val_2 = std::atoi(token_2.c_str());
    }
    void set_motor_values(int val_1, int val_2)
    {
        std::stringstream ss;
        ss << "m " << val_1 << " " << val_2 << "\r";
        send_msg(ss.str());
    }

    void set_pid_values(float k_p, float k_d, float k_i, float k_o){
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }
};

#endif // STM32_COMMS_HPP