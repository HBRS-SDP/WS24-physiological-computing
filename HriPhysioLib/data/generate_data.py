"""
* ================================================================================
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
"""

import neurokit2 as nk
import pandas as pd
import numpy as np
import os

# Function to generate ECG data
def generate_ecg(duration, sampling_rate):
    ecg_signal = nk.ecg_simulate(duration=duration, sampling_rate=sampling_rate, noise=0.02, heart_rate=70)
    timestamps = np.arange(0, len(ecg_signal)) * (1 / sampling_rate)
    data = pd.DataFrame({
        "sensor_type": "ECG",
        "value": ecg_signal,
        "timestamp": timestamps
    })
    return data

# Function to generate RSP data
def generate_rsp(duration, sampling_rate):
    rsp_signal = nk.rsp_simulate(duration=duration, sampling_rate=sampling_rate, noise=0.02, respiratory_rate=15)
    timestamps = np.arange(0, len(rsp_signal)) * (1 / sampling_rate)
    data = pd.DataFrame({
        "sensor_type": "RSP",
        "value": rsp_signal,
        "timestamp": timestamps  
    })
    return data

def create_folder(folder_name):
    os.makedirs(folder_name, exist_ok=True)

# Function to handle user input and save data
def main():
    print("Select the type of data to generate:")
    print("1. ECG (Electrocardiogram)")
    print("2. RSP (Respiration)")
    
    try:
        choice = int(input("Enter your choice (1 or 2): "))
    except ValueError:
        print("Invalid input. Please enter a number (1 or 2).")
        return

    duration = int(input("Enter the duration (in seconds): "))
    sampling_rate = int(input("Enter the sampling rate (in Hz): "))

    ecg_folder = "ecg_data"
    rsp_folder = "rsp_data"
    create_folder(ecg_folder)
    create_folder(rsp_folder)

    if choice == 1:
        print("Generating ECG data...")
        data = generate_ecg(duration, sampling_rate)
        folder = ecg_folder
        file_prefix = "ECG_data"
    elif choice == 2:
        print("Generating RSP data...")
        data = generate_rsp(duration, sampling_rate)
        folder = rsp_folder
        file_prefix = "RSP_data"
    else:
        print("Invalid choice. Please enter 1 or 2.")
        return

    iteration = len(os.listdir(folder)) + 1
    filename = os.path.join(folder, f"{file_prefix}_{iteration}.csv")

    try:
        data.to_csv(filename, index=False)
        print(f"Data successfully saved to {filename}")
    except IOError as e:
        print(f"Error saving file: {e}")

if __name__ == "__main__":
    main()