
# Synthetic Physiological Data Generator

This project generates synthetic physiological data based on user input for sensor type, duration, and sampling rate from [NeuroKit2 Library](https://github.com/neuropsychology/NeuroKit). The generated data can be useful for testing and research purposes in physiological computing.

## Steps to Setup and Run

### 1. Create a Virtual Environment
It's a good practice to work within a virtual environment to avoid conflicts with other Python projects. You can create and activate a virtual environment as follows:

#### On Windows:
```bash
python -m venv venv
.\venv\Scripts\activate
```

#### On macOS/Linux:
```bash
python3 -m venv venv
source venv/bin/activate
```

### 2. Install the Dependencies
Once the virtual environment is activated, install the required Python libraries by running the following command:

```bash
pip install -r requirements.txt
```

This will install all the necessary dependencies listed in the `requirements.txt` file.

### 3. Run the Python Script
To generate synthetic physiological data, run the Python script using the following command:

```bash
python generate_data.py
```

The script will prompt you for the following inputs:
- **Sensor Type**: Choose the type of physiological sensor data you want to generate: ECG or RSP
- **Duration (secs)**: Enter the duration (in seconds) for which you want to generate the data.
- **Sampling Rate (Hz)**: Enter the sampling rate in Hertz (samples per second).

### Example Input
```
Select the type of data to generate:
1. ECG (Electrocardiogram)
2. RSP (Respiration)
Enter your choice (1 or 2): 1
Enter the duration (in seconds): 5
Enter the sampling rate (in Hz): 1000
Generating ECG data...
```

### 4. View the Generated Data
After running the script, the synthetic data will be generated based on your inputs. The data will be saved to a file, which you can use for analysis or testing purposes.
### Example File
```
Data successfully saved to ecg_data/ECG_data.csv
```
---
You can also visualize the plot to get a better understanding of what is happening.

### Example plot for ECG data

![Screenshot from 2025-01-16 13-20-06](https://github.com/user-attachments/assets/197220e7-0076-4416-ba1e-e28cff54efd9)
