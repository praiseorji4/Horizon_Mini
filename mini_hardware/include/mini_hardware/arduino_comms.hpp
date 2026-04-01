#ifndef MINI_HARDWARE_ARDUINO_COMMS_HPP
#define MINI_HARDWARE_ARDUINO_COMMS_HPP

#include <sstream>
#include <string>
#include <algorithm>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
        case 1200:   return LibSerial::BaudRate::BAUD_1200;
        case 1800:   return LibSerial::BaudRate::BAUD_1800;
        case 2400:   return LibSerial::BaudRate::BAUD_2400;
        case 4800:   return LibSerial::BaudRate::BAUD_4800;
        case 9600:   return LibSerial::BaudRate::BAUD_9600;
        case 19200:  return LibSerial::BaudRate::BAUD_19200;
        case 38400:  return LibSerial::BaudRate::BAUD_38400;
        case 57600:  return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cout << "Error! Baud rate " << baud_rate
                      << " not supported! Defaulting to 57600" << std::endl;
            return LibSerial::BaudRate::BAUD_57600;
    }
}

class ArduinoComms
{
public:
    ArduinoComms() = default;

    void connect(const std::string &serial_device,
                 int32_t baud_rate,
                 int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;   // use the value from URDF/config (recommend >= 50)
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    }

    void disconnect()
    {
        serial_conn_.Close();
    }

    bool connected() const
    {
        return serial_conn_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.FlushIOBuffers();   // flush stale bytes before every transaction
        serial_conn_.Write(msg_to_send);

        std::string response;
        try
        {
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "ReadLine() timed out waiting for response to: "
                      << msg_to_send << std::endl;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send
                      << "  Recv: " << response << std::endl;
        }

        return response;
    }

    void send_empty_msg()
    {
        send_msg("\r");
    }

    // Reads 4 encoder values: front-left, front-right, rear-left, rear-right
    // Arduino sends: "fl fr rl rr\r\n"
    void read_encoder_values(int &fl, int &fr, int &rl, int &rr)
    {
        std::string response = send_msg("e\r");

        // Strip all carriage-return and newline characters
        response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
        response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());

        std::istringstream ss(response);
        int vals[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i)
        {
            if (!(ss >> vals[i]))
            {
                std::cerr << "read_encoder_values: failed to parse token " << i
                          << " from response: '" << response << "'" << std::endl;
                break;
            }
        }

        fl = vals[0];
        fr = vals[1];
        rl = vals[2];
        rr = vals[3];
    }

    // Sends 4 motor speed values: front-left, front-right, rear-left, rear-right
    void set_motor_values(int fl, int fr, int rl, int rr)
    {
        std::stringstream ss;
        ss << "m " << fl << ":" << fr << ":" << rl << ":" << rr << "\r";
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        std::stringstream ss;
        ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_{50};   // safe default; override via connect()
};

#endif  // MINI_HARDWARE_ARDUINO_COMMS_HPP