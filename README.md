# HRI Physio Lib 
## Introduction
HRI Physio Lib is a library developed by Austin Kothig for researchers in the field of physiological computing. The structure and basic implementation of this library can be found in the image below:
![HRI Physio Lib](https://github.com/user-attachments/assets/894adccd-6bd8-4dcc-8718-edef5a2d8647)

## Installing the library:
- Setting up the library requires a couple of steps of installation so please bear with us. Follow these steps in the given order to successfully set up the library:
  - Cloning the repository: navigate to your workspace where you want to install the library and run the following command:<br>
    ```
      git clone https://github.com/HBRS-SDP/WS24-physiological-computing.git
    ```
    
  - Installing dependancies: as you know our repository uses three external libraries which have been added as submodules in the /deps folder, so we need to clone them first:<br>
      ```
    cd WS24-physiological-computing/deps/
      ```
      ```
    rm -rf fmt/ && rm -rf labstreaminglayer/ && rm -rf yaml-cpp/
      ```
      ```
    git clone https://github.com/fmtlib/fmt.git
      ```
      ```
    git clone https://github.com/jbeder/yaml-cpp.git
      ```
      ```
    git clone --recurse-submodules https://github.com/sccn/labstreaminglayer.git
      ```
    
  - What we have done currently is add the dependancies to our library. One additional step that needs to be done is that we need to build 'labstreaminglayer':
      ```
    cd labstreaminglayer/
      ```
      ```
    mkdir build && cd build
      ```
      ```
    cmake ..
      ```
      ```
    make
      ```
  - You should see a series of percentage statements. At the end if you see no errors then you are successfully done with installing the dependancies.
  - Next step is building our library so navigate back to the home folder i.e. WS24-physiological-computing and run the following commands:
      ```
    mkdir build && cd build
      ```
      ```
    cmake ..
      ```
      ```
    make
      ```
  - This should result in a successful building of the library. Now final step of this process is to install the library:
    ```
    sudo cmake --build . --config Release --target install
    ```
## Steps to use the spectrogram function:
  - The Spectrogram class processes physiological signals by applying the short-time Fourier transform (STFT) to extract frequency-domain features. It first segments the input signal into overlapping windows using a Hamming function, then applies the Fast Fourier Transform (FFT) to each window to convert the signal from the time domain to the frequency domain. The resulting power spectrum values are stored in a matrix, forming the spectrogram.
  - It allows the user to choose from two input modes, manual input and input from a .csv file.
  - Run the following command to use the program:
    ```
    ./HriphysioLib/build/spectrogram
    ```
  - You should be able to select from the input modes and receive a processed output!
- If you wish to publish this output as a ROS topic then you need to follow a few additional steps. The program can act as a ROS2 node as well. The steps for the same are:
  - Clone the `hriphysio_pkg` in your ROS2 workspace.
  - Change line 17 (HRIPHYSIO_LIB_PATH) from the CmakeLists.txt (# Add HriPhysioLib manually) to the location of your HriPhysioLib location. Make sure that the HriphysioLib library is properly built without any errors.
  - Build your workspace:
    ```
    colcon build --packages-select hriphysio_pkg
    ```
  - Source your workspace:
    ```
    source install/setup.bash
    ```
  - Run the publisher node:
    ```
    ros2 run hriphysio_pkg spectrogram_publisher_node
    ```
  - You can see the output being published on the topic `/spectrogram_output`.
  - If you have any difficulties in setting up a ROS2 workspace then you can visit the following page: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html 

## Contributors:
- Austin Kothig: austin.kothig@uwaterloo.ca
- Trushar Ghanekar: trushar.ghanekar@smail.inf.h-brs.de
- Vedika Chauhan: vedika.chauhan@smail.inf.h-brs.de
- Shrikar Nakhye: shrikar.nakhye@smail.inf.h-brs.de
- Ayush Salunke: ayush.salunke@smail.inf.h-brs.de
- Prachi Sheth: prachi.sheth@smail.inf.h-brs.de
- Yash Somaiya: yash.somaiya@smail.inf.h-brs.de

      
      
  
