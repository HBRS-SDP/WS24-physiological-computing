# HRI Physio Lib 

## Table of Contents:

  - [Introduction](https://github.com/HBRS-SDP/WS24-physiological-computing/edit/devel/README.md#introduction)
  - [Installing the library](https://github.com/HBRS-SDP/WS24-physiological-computing/edit/devel/README.md#introduction)
  - [Module functionalities](https://github.com/HBRS-SDP/WS24-physiological-computing/tree/devel/HriPhysioLib)
  - [Generating fabricated input data](https://github.com/HBRS-SDP/WS24-physiological-computing/tree/devel/HriPhysioLib/data)
  - [ROS2 testing](https://github.com/HBRS-SDP/WS24-physiological-computing/tree/devel/ros2)
  - [Contributors](https://github.com/HBRS-SDP/WS24-physiological-computing/edit/devel/README.md#contributors)

## Introduction
HRI Physio Lib is a library developed by Austin Kothig for researchers in the field of physiological computing. The structure and basic implementation of this library can be found in the image below:
![HRI Physio Lib](https://github.com/user-attachments/assets/894adccd-6bd8-4dcc-8718-edef5a2d8647)

## Installing the library:
- Setting up the library requires a couple of steps of installation so please bear with us. Follow these steps in the given order to successfully set up the library:
  - Cloning the repository: navigate to your workspace where you want to install the library and run the following command (It is recommended that you install this library in your ROS2 workspace):<br>
    ```
      git clone -b devel https://github.com/HBRS-SDP/WS24-physiological-computing.git
    ```
    
  - Installing dependencies: as you know our repository uses three external libraries which have been added as submodules in the /deps folder, so we need to clone them first:<br>
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
    
  - What we have done currently is add the dependencies to our library. One additional step that needs to be done is that we need to build 'labstreaminglayer':
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
  - You should see a series of percentage statements. If you see no errors then you have successfully installed the dependencies.
  - Then we build HriPhysioLib before our final step. Follow the steps below.
     ```
     cd ../../../HriPhysioLib
     ```
     ```
     rm -rf build
     ```
     ```
     mkdir build && cd build
     ```
    ```
    cmake ..
    ```
    ```
    sudo cmake --build . --config Release --target install
    ```
  - The final step is building our library so navigate back to the home folder i.e. WS24-physiological-computing. Run the following commands:
      ```
      cd ../..
      ```
      ```
      mkdir build && cd build
      ```
      ```
      cmake ..
      ```
      ```
      sudo cmake --build . --config Release --target install
      ```
  - This should result in a successful building of the library. Now you can head over to the `/ros2` folder to follow instructions on testing `/spectrogram` functionality of the library and setting up a ros2 communication node for the library.


## Contributors:
- Austin Kothig: austin.kothig@uwaterloo.ca
- Trushar Ghanekar: trushar.ghanekar@smail.inf.h-brs.de
- Vedika Chauhan: vedika.chauhan@smail.inf.h-brs.de
- Shrikar Nakhye: shrikar.nakhye@smail.inf.h-brs.de
- Ayush Salunke: ayush.salunke@smail.inf.h-brs.de
- Prachi Sheth: prachi.sheth@smail.inf.h-brs.de
- Yash Somaiya: yash.somaiya@smail.inf.h-brs.de

      
      
  
