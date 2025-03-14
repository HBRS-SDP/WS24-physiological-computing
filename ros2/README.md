# Testing HRIPhysioLib

# `Processing/spectrogram.cpp`

To Test the working of HriPhysioLib modules, `spectrogram.cpp` file is used; as it reveals hidden frequency patterns.

The Spectrogram class processes physiological signals by applying the short-time Fourier transform (STFT) to extract frequency-domain features. It first segments the input signal into overlapping windows using a Hamming function, then applies the Fast Fourier Transform (FFT) to each window to convert the signal from the time domain to the frequency domain. The resulting power spectrum values are stored in a matrix, forming the spectrogram.

# `main.cpp`

This program processes physiological data to generate a spectrogram. 

- It first allows the user to choose an input method: manually entering data or loading it from a CSV file. 
- If CSV input is chosen, the `loadCSV` function reads the file, extracts numerical values from the "value" column, and stores them in a vector. 
- If manual input is chosen, the `readFromUserInput` function collects comma-separated values from the user and stores them in a vector. After data collection, the program initializes a **Spectrogram** object with the size of the input data and calls its `process` method, which applies the **short-time Fourier transform (STFT)** to compute the time-frequency representation. 
- The spectrogram results are stored in a 2D vector, `output`, which is then printed, displaying only non-zero values. 
- The expected output is a matrix where each row corresponds to a frequency component over time, revealing patterns in the physiological signal.

Manual Data Input:

![Screenshot from 2025-03-05 16-39-33](https://github.com/user-attachments/assets/71b3f8ee-e05a-4b9d-89ca-073e0e227723)

From CSV File:

![Screenshot from 2025-03-05 16-38-43](https://github.com/user-attachments/assets/6a4db55b-1f09-456f-b772-25280976e5d9)

To Test it; run using following command:

- `g++ src/Processing/main.cpp src/Processing/spectrogram.cpp -Iinclude -Iinclude/PocketFFT -o spectrogram_out -std=c++17`

- `./spectrogram_out`

# `spectrogram_publisher.cpp`
Description:

This C++ program is a ROS 2 node that generates and publishes spectrogram data based on user-provided input. It allows the user to either: manually enter data, or load data from a CSV file. The data processing is performed in the same way as `main.cpp`.

The input data is processed using the HriPhysio library's Spectrogram class, and the computed spectrogram is published as a ROS 2 message on the `/spectrogram_output` topic.

Demo Screenshots:

![WhatsApp Image 2025-03-11 at 12 30 16_ef97b36c](https://github.com/user-attachments/assets/7e598e26-b7da-45d4-b16f-8f1a8c9581fe)

![WhatsApp Image 2025-03-11 at 12 30 53_68e52774](https://github.com/user-attachments/assets/15dd07b2-fc84-4c89-b09c-960b36f86bae)

To Test it; run using following command:

- Clone the `ros2/hriphysio_pkg` in your local ros2 workspace.
- Change line 17 (HRIPHYSIO_LIB_PATH) from the CmakeLists.txt (# Add HriPhysioLib manually) to the location of your HriPhysioLib location. Make sure that the HriphysioLib library is properly built without any errors. <br>
- build your workspace:        `colcon build --packages-select hriphysio_pkg` 
- source your ros2 workspace:  `source install/setup.bash`
- run the publisher node :     `ros2 run hriphysio_pkg spectrogram_publisher_node`
