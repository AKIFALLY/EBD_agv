#include <iostream>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "keyence_plc_com.hpp"  // 你的 C++ PLC 通訊類別

class SpeedTest {
public:
    SpeedTest(const std::string& plc_ip, int plc_port) : plc_(plc_ip, plc_port) {}

    void test_read_speed(const std::string& device_type, int device_number, int num_tests = 100) {
        plc_.connect();
        double total_time = 0;
        double min_time = std::numeric_limits<double>::max();
        double max_time = std::numeric_limits<double>::lowest();

        for (int i = 0; i < num_tests; i++) {
            auto start_time = std::chrono::high_resolution_clock::now();
            //int received_data = std::stoi(plc_.read_data(device_type, device_number));
            plc_.read_data(device_type, device_number);
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            
            total_time += elapsed;
            min_time = std::min(min_time, elapsed);
            max_time = std::max(max_time, elapsed);
            
            //std::cout << "Read " << i + 1 << ": Received Data = " << received_data << "\n";
        }

        std::cout << "Read (" << num_tests << " times) -> Avg: " << (total_time / num_tests)
                  << " ms, Min: " << min_time << " ms, Max: " << max_time << " ms\n";
        plc_.disconnect();
    }

    void test_write_speed(const std::string& device_type, int device_number, int write_data, int num_tests = 100) {
        plc_.connect();
        double total_time = 0;
        double min_time = std::numeric_limits<double>::max();
        double max_time = std::numeric_limits<double>::lowest();

        for (int i = 0; i < num_tests; i++) {
            auto start_time = std::chrono::high_resolution_clock::now();
            //std::string success = plc_.write_data(device_type, device_number, write_data);
            plc_.write_data(device_type, device_number, write_data);
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            
            total_time += elapsed;
            min_time = std::min(min_time, elapsed);
            max_time = std::max(max_time, elapsed);
            
            //std::cout << "Write " << i + 1 << ": Success = " << success << "\n";
        }

        std::cout << "Write (" << num_tests << " times) -> Avg: " << (total_time / num_tests)
                  << " ms, Min: " << min_time << " ms, Max: " << max_time << " ms\n";
        plc_.disconnect();
    }

    void test_continuous_read_speed(const std::string& device_type, int device_number, int device_length, int num_tests = 100) {
        plc_.connect();
        double total_time = 0;
        double min_time = std::numeric_limits<double>::max();
        double max_time = std::numeric_limits<double>::lowest();

        for (int i = 0; i < num_tests; i++) {
            auto start_time = std::chrono::high_resolution_clock::now();
            std::vector<std::string> raw_data = plc_.read_continuous_data(device_type, device_number, device_length);
            //std::vector<int> received_data;
            //for (const auto& str : raw_data) {
            //    received_data.push_back(std::stoi(str));
            //}
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            
            total_time += elapsed;
            min_time = std::min(min_time, elapsed);
            max_time = std::max(max_time, elapsed);
            
            //std::cout << "Continuous Read " << i + 1 << ": Received Data = ";
            //for (const auto& data : received_data) {
            //    std::cout << data << " ";
            //}
            //std::cout << "\n";
        }

        std::cout << "Continuous Read (" << num_tests << " times) -> Avg: " << (total_time / num_tests)
                  << " ms, Min: " << min_time << " ms, Max: " << max_time << " ms\n";
        plc_.disconnect();
    }

    void test_continuous_write_speed(const std::string& device_type, int device_number, const std::vector<int>& write_data, int num_tests = 100) {
        plc_.connect();
        double total_time = 0;
        double min_time = std::numeric_limits<double>::max();
        double max_time = std::numeric_limits<double>::lowest();

        for (int i = 0; i < num_tests; i++) {
            auto start_time = std::chrono::high_resolution_clock::now();
            //std::string success = plc_.write_continuous_data(device_type, device_number, write_data);
            plc_.write_continuous_data(device_type, device_number, write_data);
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            
            total_time += elapsed;
            min_time = std::min(min_time, elapsed);
            max_time = std::max(max_time, elapsed);
            
            //std::cout << "Continuous Write " << i + 1 << ": Success = " << success << "\n";
        }

        std::cout << "Continuous Write (" << num_tests << " times) -> Avg: " << (total_time / num_tests)
                  << " ms, Min: " << min_time << " ms, Max: " << max_time << " ms\n";
        plc_.disconnect();
    }

private:
    KeyencePlcCom plc_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::string plc_ip = "192.168.0.100";
    int plc_port = 8501;

    SpeedTest test(plc_ip, plc_port);

    std::cout << "---\n";
    test.test_read_speed("DM", 5000, 100);
    std::cout << "---\n";
    test.test_write_speed("DM", 5000, 123, 100);
    std::cout << "---\n";
    test.test_continuous_read_speed("DM", 5000, 1, 100);
    std::cout << "---\n";
    test.test_continuous_write_speed("DM", 5000, {1,2,3,4,5,6,7,8,9,10}, 100);
    std::cout << "---\n";

    rclcpp::shutdown();
    return 0;
}
