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
#include <string>
#include <fstream>
#include <sstream>
#include "HriPhysio/Stream/csvStreamer.h"

// int main() {
//     // Create an instance of CsvStreamer
//     hriPhysio::Stream::CsvStreamer csvStreamer;

//     // Set input and output file names
//     std::string inputFile = "/home/ayush/sdp_ws/src/WS24-physiological-computing/HriPhysioLib/src/Stream/data.csv";
//     std::string outputFile = "/home/ayush/sdp_ws/src/WS24-physiological-computing/HriPhysioLib/src/Stream/output.csv";

//     // Set the input stream file
//     csvStreamer.setName(inputFile);

//     // Open input stream
//     if (!csvStreamer.openInputStream()) {
//         std::cerr << "Error opening input stream for file: " << inputFile << std::endl;
//         return -1;
//     }

//     std::vector<hriPhysio::varType> buffer;
//     std::vector<double> timestamps;

//     // Read from input CSV
//     csvStreamer.receive(buffer, &timestamps);

//     // // Log received data for debugging
//     // if (receivedBuffer.empty() || receivedTimestamps.empty()) {
//     //     std::cerr << "No data read from input file: " << inputFile << std::endl;
//     //     return -1;
//     // }

//     // std::cout << "Data read from input file: " << inputFile << std::endl;
//     // for (size_t i = 0; i < receivedBuffer.size(); ++i) {
//     //     std::cout << "Data[" << i << "]: " << std::get<int32_t>(receivedBuffer[i]) 
//     //               << ", Timestamp: " << receivedTimestamps[i] << std::endl;
//     // }

//     // Set the output stream file
//     csvStreamer.setName(outputFile);

//     // Open output stream
//     if (!csvStreamer.openOutputStream()) {
//         std::cerr << "Error opening output stream for file: " << outputFile << std::endl;
//         return -1;
//     }

//     // // Write data to output stream
//     // std::cout << "Writing data to output file: " << outputFile << std::endl;
//     // csvStreamer.publish(receivedBuffer, &receivedTimestamps);

//     csvStreamer.publish(buffer, &timestamps);

//     std::cout << "Data successfully written to output file: " << outputFile << std::endl;

//     return 0;
// }





int main() {
    hriPhysio::Stream::CsvStreamer csvStreamer;

    std::string inputFilePath = "/home/ayush/sdp_ws/src/WS24-physiological-computing/HriPhysioLib/src/Stream/data.csv";
    csvStreamer.setName(inputFilePath);

    std::ifstream inputFile(inputFilePath);
    if (!inputFile.is_open()) {
        std::cerr << "Error opening input file!" << std::endl;
        return -1;
    }

    std::string line;
    if (std::getline(inputFile, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::getline(lineStream, cell, ',');
        try {
            std::stod(cell);
        } catch (const std::invalid_argument&) {
            std::cout << "Skipping header row: " << line << std::endl;
        }
    }

    std::vector<hriPhysio::varType> dataBuffer;
    std::vector<double> timestamps;

    while (std::getline(inputFile, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        double timestamp;
        int data_value;

        if (std::getline(lineStream, cell, ',')) {
            try {
                timestamp = std::stod(cell); 
            } catch (const std::invalid_argument&) {
                std::cerr << "Invalid timestamp in file: " << cell << std::endl;
                continue; 
            }
        } else {
            continue;
        }

        if (std::getline(lineStream, cell, ',')) {
            try {
                data_value = std::stoi(cell); 
            } catch (const std::invalid_argument&) {
                std::cerr << "Invalid data value in file: " << cell << std::endl;
                continue;
            }
        } else {
            continue;
        }

        timestamps.push_back(timestamp);
        dataBuffer.push_back(data_value);
    }

    inputFile.close();

    if (!csvStreamer.openInputStream()) {
        std::cerr << "Error opening input stream!" << std::endl;
        return -1;
    }

    std::vector<hriPhysio::varType> receivedBuffer;
    std::vector<double> receivedTimestamps;
    csvStreamer.receive(receivedBuffer, &receivedTimestamps);

    for (size_t i = 0; i < receivedBuffer.size(); ++i) {
        std::cout << "Received data[" << i << "]: " << std::get<int32_t>(receivedBuffer[i]) 
                  << ", Timestamp: " << receivedTimestamps[i] << std::endl;
    }

    if (dataBuffer.empty() || timestamps.empty()) {
        std::cerr << "No valid data found in the CSV file!" << std::endl;
        return -1;
    }
    if (!csvStreamer.openOutputStream()) {
        std::cerr << "Error opening output stream!" << std::endl;
        return -1;
    }

    csvStreamer.publish(dataBuffer, &timestamps);

    std::string stringData = "Example string data";
    if (!timestamps.empty()) {
        csvStreamer.publish(stringData, &timestamps[0]);
    } else {
        std::cerr << "No timestamps available for publishing!" << std::endl;
    }

    return 0;
}
