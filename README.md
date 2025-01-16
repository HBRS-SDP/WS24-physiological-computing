# HRI Physio Lib 
## Introduction
HRI Physio Lib is a library developed by Austin Kothig for researchers in the field of physiological computing. The structure and basic implementation of this library can be found in the image below:
![HRI Physio Lib](https://github.com/user-attachments/assets/894adccd-6bd8-4dcc-8718-edef5a2d8647)
## 🧩 **Library Components**

### 📦 **External Libraries**

**HriPhysioLib** integrates external submodules to extend its functionality. These are:

- **`liblsl`** 🛠️: For streaming physiological data using **Lab Streaming Layer (LSL)**.
- **`yaml-cpp`** 🗂️: For parsing and emitting **YAML** files, useful in handling configurations.

### 📂 **Source Modules**

The **source** folder is divided into several modules, each with a specific role:

#### 🛠️ **Core**
The **Core** module provides essential data structures. The key component here is the **Ring Buffer**, a circular structure used for managing physiological data streams in **FIFO** (First In, First Out) order.

- **`ringbuffer.h`**: The heart of this module, providing the ring buffer functionality.

#### 🔧 **Manager**
The **Manager** module handles the coordination between robotic systems 🤖 and physiological data. It includes multithreading for efficient, real-time operations.

- **`robot_manager.h/cpp`**: Enables data exchange between robots and physiological data.
- **`thread_manager.h/cpp`**: Manages multithreaded processes for data streaming and processing.

#### 🤖 **Social**
The **Social** module is all about integrating robots with the outside world using physiological signals to drive behaviors 🚶‍♂️→🤖.

- **`robot_interface.h/cpp`**: The standard interface to communicate with robots.

#### 🔬 **Processing**
This is where the magic of transforming raw data into something meaningful happens! 🧙‍♂️ The **Processing** module includes:

- **`hilbert_transform.h/cpp`**: For applying the **Hilbert Transform** to physiological signals.
- **`pocketfft.h`**: Handles **Fourier Transforms** for time-frequency analysis.
- **`spectrogram.h/cpp`**: Generates **spectrograms** for visualizing signal frequencies over time.

#### 💾 **Stream**
The **Stream** module manages data flow from external sources to the processing pipeline 📊.

- **`csv_streamer.h/cpp`**: Handles data in **CSV format**.
- **`lsl_streamer.h/cpp`**: Facilitates **LSL-based** physiological data streaming.

#### ⚙️ **Utilities**
The **Utilities** module offers various helper tools to smooth your workflow 🛠️.

- **`arg_parser.h/cpp`**: Parses command-line arguments for easy library configuration.
- **`helper.h/cpp`**: Contains useful functions like data conversion and logging.
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
    sudo 

## Contributors:
- Austin Kothig: austin.kothig@uwaterloo.ca
- Trushar Ghanekar: trushar.ghanekar@smail.inf.h-brs.de
- Vedika Chauhan: vedika.chauhan@smail.inf.h-brs.de
- Shrikar Nakhye: shrikar.nakhye@smail.inf.h-brs.de
- Ayush Salunke: ayush.salunke@smail.inf.h-brs.de
- Prachi Sheth: prachi.sheth@smail.inf.h-brs.de
- Yash Somaiya: yash.somaiya@smail.inf.h-brs.de

      
      
  
