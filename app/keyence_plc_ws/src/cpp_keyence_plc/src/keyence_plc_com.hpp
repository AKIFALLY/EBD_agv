#ifndef KEYENCE_PLC_COM_HPP
#define KEYENCE_PLC_COM_HPP

#include <iostream>
#include <string>
#include <vector>

class KeyencePlcCom {
public:
    static const std::string PLC_NEWLINE;
    KeyencePlcCom(const std::string& ip, int port);
    bool connect();
    bool reconnect();
    void disconnect();
    std::string send_command(const std::string& command);
    std::string read_data(const std::string& device_type, int device_number);
    std::string write_data(const std::string& device_type, int device_number, int write_data);
    std::vector<std::string> read_continuous_data(const std::string& device_type, int device_number, int device_length);
    std::string write_continuous_data(const std::string& device_type, int device_number, const std::vector<int>& write_data);

private:
    std::string receive_until(const std::string& end_marker = PLC_NEWLINE, int buffer_size = 1024);
    std::vector<std::string> split_string(const std::string& str, const std::string& delimiter = " ");

private:
    std::string ip;
    int port;
    int sock;
    int timeout;
};

#endif  // KEYENCE_PLC_COM_HPP
