# Functionaities of the Modules

## Core
## Manager
## Helper
## Processing

#### `biquadratic.cpp`

This file implements a **Biquadratic filter** for signal processing. It updates filter coefficients dynamically based on the center frequency and applies a bilinear transformation to process input signals, helping remove noise or extract relevant frequency components.

#### `butterworthBandNoch.cpp`

This file implements a **Butterworth Band-Noch filter**, which removes a specific frequency range from a signal while preserving others.

#### `butterworthBandPass.cpp`

This file implements a **Butterworth Band-Pass filter**, allowing only a specific frequency range to pass while reducing frequencies outside this range.

#### `butterworthHighPass.cpp`

This file implements a **Butterworth High-Pass filter**, which removes low-frequency components while preserving higher frequencies.

#### `butterworthLowPass.cpp`

This file implements a **Butterworth Low-Pass filter**, which removes high-frequency noise while retaining lower-frequency components.

#### `hilbertTransform.cpp`

This file implements the **Hilbert Transform**, which is crucial in physiological computing for extracting the analytic signal from raw physiological data.

#### `spectrogram.cpp`

This file applies a **short-time Fourier transform (STFT)**, using the Hamming window to extract time-frequency representations from input signals.

## Social

#### `robotInterface.cpp`

This file implements the **RobotInterface**, a class that provides a common interface for controlling a social robot in human-robot interaction.

## Stream

#### `csvStreamer.cpp`
This file manages reading and writing CSV files for streaming data in the HriPhysio framework. It allows opening files for input or output, writing structured data with timestamps, and reading stored data into buffers. It provides methods to publish and retrieve data from CSV files. The class also includes error handling for file operations and structured logging for debugging.

#### `lslStreamer.cpp`
This file facilitates streaming data using the LabStreamingLayer (LSL) protocol. It supports bidirectional communication, allowing the system to either send or receive data. The class defines methods for opening input (openInputStream) and output (openOutputStream) streams, handling various data types through getLslFormatType, and managing real-time data exchange via publish and receive functions.

#### `rosStreamer.cpp`
This file defines the RosStreamer class, which provides an interface for streaming data over ROS (Robot Operating System). It sets up ROS publishers and subscribers to transmit and receive various data types. The class includes methods for opening input and output streams, publishing data, and receiving data. Some parts of the code, such as handling incoming data, are unfinished or commented out.

#### `streamerInterface.cpp`
This file defines the StreamerInterface class, which serves as a base class for managing data streaming configurations in the hriPhysio framework. It provides methods for setting and retrieving stream properties such as name, data type, frame length, number of channels, and sampling rate. 

#### `yarpStreamer.cpp`
This file implements the YarpStreamer class, which manages data streaming using YARP within the hriPhysio framework, supporting both data publishing and receiving.

## Factory

#### `streamerFactory.cpp`
This file implements the StreamerFactory class, which creates and returns instances of different streaming interfaces (LSL, ROS, YARP) based on the requested type.
  



