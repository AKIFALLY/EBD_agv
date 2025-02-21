#include "keyence_plc_com.hpp"
#include <cstring>
#include <chrono>
#include <thread>
#include <sstream>
#include <stdexcept>
#include <arpa/inet.h>
#include <unistd.h>

KeyencePlcCom::KeyencePlcCom(const std::string& ip, int port) : ip(ip), port(port), sock(-1), timeout(5) {}

const std::string KeyencePlcCom::PLC_NEWLINE = "\r\n";

bool KeyencePlcCom::connect() {
    if (sock == -1) {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock == -1) {
            std::cerr << "Error creating socket\n";
            return false;
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

        if (::connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Error connecting to PLC: " << strerror(errno) << "\n";
            return false;
        }

        std::cout << "Successfully connected to PLC at " << ip << ":" << port << "\n";
        return true;
    } else {
        std::cout << "Already connected.\n";
        return true;
    }
}

bool KeyencePlcCom::reconnect() {
    disconnect();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return connect();
}

void KeyencePlcCom::disconnect() {
    if (sock != -1) {
        close(sock);
        sock = -1;
        std::cout << "Disconnected from PLC at " << ip << ":" << port << "\n";
    } else {
        std::cout << "Not connected to PLC.\n";
    }
}

std::string KeyencePlcCom::send_command(const std::string& command) {
    if (sock != -1) {
        ssize_t sent = send(sock, command.c_str(), command.size(), 0);
        if (sent == -1) {
            std::cerr << "Error sending command: " << strerror(errno) << "\n";
            return "";
        }
        return receive_until();
    } else {
        std::cerr << "Not connected to PLC.\n";
        return "";
    }
}

std::string KeyencePlcCom::read_data(const std::string& device_type, int device_number) {
    std::ostringstream command;
    command << "RD " << device_type << device_number << PLC_NEWLINE;
    //std::cout << "CMD:" << command.str();
    return send_command(command.str());
}

std::string KeyencePlcCom::write_data(const std::string& device_type, int device_number, int write_data) {
    std::ostringstream command;
    command << "WR " << device_type << device_number << " " << write_data << PLC_NEWLINE;
    //std::cout << "CMD:" << command.str();
    return send_command(command.str());
}

std::vector<std::string> KeyencePlcCom::read_continuous_data(const std::string& device_type, int device_number, int device_length) {
    std::ostringstream command;
    command << "RDS " << device_type << device_number << " " << device_length << PLC_NEWLINE;
    //std::cout << "CMD:" << command.str();
    std::string response = send_command(command.str());
    return split_string(response);
}

std::string KeyencePlcCom::write_continuous_data(const std::string& device_type, int device_number, const std::vector<int>& write_data) {
    std::ostringstream command;
    command << "WRS " << device_type << device_number << " " << write_data.size() ;
    for (const auto& data : write_data) {
        command << " " << data;
    }
    command << PLC_NEWLINE;
    //std::cout << "CMD:" << command.str();
    return send_command(command.str());
}


std::string KeyencePlcCom::receive_until(const std::string& end_marker, int buffer_size) {
    std::string data;
    std::vector<char> buffer(buffer_size);
    ssize_t received;
    while (true) {
        received = recv(sock, buffer.data(), buffer_size, 0);
        if (received <= 0) {
            break;
        }
        data.append(buffer.begin(), buffer.begin() + received);
        if (data.find(end_marker) != std::string::npos) {
            break;
        }
    }
    if (received == -1) {
        std::cerr << "Error receiving data: " << strerror(errno) << "\n";
        return "";
    }
    return data;
}

std::vector<std::string> KeyencePlcCom::split_string(const std::string& str, const std::string& delimiter) {
    std::vector<std::string> tokens;
    size_t pos = 0;
    size_t found;
    while ((found = str.find(delimiter, pos)) != std::string::npos) {
        tokens.push_back(str.substr(pos, found - pos));
        pos = found + delimiter.length();
    }
    tokens.push_back(str.substr(pos));
    return tokens;
}
