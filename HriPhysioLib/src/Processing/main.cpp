/* ================================================================================
 * Copyright: (C) 2024, Prachi Sheth,
 *     Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Author: 
 *     Prachi Sheth <prachi.sheth@smail.inf.h-brs.de>
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <HriPhysio/spectrogram.h>

using namespace hriPhysio::Processing;

/// To read from a csv file.
void readFromCSV(const std::string& filename, std::vector<double>& data) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << "\n";
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double value;
        while (ss >> value) {
            data.push_back(value);
            if (ss.peek() == ',') ss.ignore();
        }
    }

    file.close();
}

// Take user input for real-time data
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

int main() {
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
        std::string filename;
        std::cout << "Enter CSV filename: ";
        std::cin >> filename;
        readFromCSV(filename, input_data);
    } else {
        std::cerr << "Invalid choice. Exiting.\n";
        return 1;
    }

    if (input_data.empty()) {
        std::cerr << "Error: No valid data received.\n";
        return 1;
    }

    // Spectrogram processing
    Spectrogram spectrogram(input_data.size());
    std::vector<std::vector<double>> output;
    double sample_rate = 100.0;  
    spectrogram.process(input_data, output, sample_rate);

    // Print output
    std::cout << " Spectrogram Output:\n";
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
                std::cout << val << " ";
            }
            std::cout << "\n";
        }
    }

    return 0;
}
