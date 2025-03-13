/* ================================================================================
 * Copyright: (C) 2024, Ayush Salunke, 
 *       Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Authors: Ayush Salunke ayush.salunke@smail.inf.h-brs.de
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *      See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <HriPhysio/spectrogram.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace hriPhysio::Processing;

class SpectrogramPublisher : public rclcpp::Node {
public:
    SpectrogramPublisher() : Node("spectrogram_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("spectrogram_output", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), 
            std::bind(&SpectrogramPublisher::publish_spectrogram, this));
    }

private:
    void publish_spectrogram() {
        std::vector<double> input_data;
        int choice;

        std::cout << "Choose input method:\n";
        std::cout << "1. Enter data manually\n";
        std::cout << "2. Load from CSV file\n";
        std::cout << "Enter choice (1 or 2): ";
        std::cin >> choice;
        std::cin.ignore();

        if (choice == 1) {
            readFromUserInput(input_data);
        } else if (choice == 2) {
            std::string file_path;
            std::cout << "Enter full path of CSV file: ";
            std::cin >> file_path;
            loadCSV(file_path, input_data);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid choice. Exiting.");
            return;
        }

        if (input_data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid data received.");
            return;
        }

        // Process Spectrogram
        Spectrogram spectrogram(input_data.size());
        std::vector<std::vector<double>> output;
        double sample_rate = 100.0;
        spectrogram.process(input_data, output, sample_rate);

        // Format output
        std_msgs::msg::String message;
        std::stringstream ss;
        ss << "Spectrogram Output:\n";
        for (const auto& row : output) {
            bool non_zero = false;
            for (double val : row) {
                if (val != 0) {
                    non_zero = true;
                    break;
                }
            }
            if (non_zero) {
                for (double val : row) {
                    ss << val << " ";
                }
                ss << "\n";
            }
        }

        message.data = ss.str();
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published spectrogram output.");
    }

    void loadCSV(const std::string& file_path, std::vector<double>& values) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", file_path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string sensor_type, value_str, timestamp;
            std::getline(ss, sensor_type, ',');
            std::getline(ss, value_str, ',');
            std::getline(ss, timestamp, ',');

            try {
                values.push_back(std::stod(value_str));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing line: %s (%s)", line.c_str(), e.what());
            }
        }
        file.close();
    }

    void readFromUserInput(std::vector<double>& data) {
        std::cout << "Enter physiological data (comma-separated values): ";
        std::string input_str;
        std::getline(std::cin, input_str);

        std::stringstream ss(input_str);
        double value;
        while (ss >> value) {
            data.push_back(value);
            if (ss.peek() == ',') ss.ignore();
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpectrogramPublisher>());
    rclcpp::shutdown();
    return 0;
}
