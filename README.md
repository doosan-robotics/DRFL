Copyright © 2025 Doosan Robotics Inc


# Build Instructions for DRFL (Doosan Robotics Framework Library)

This document outlines the procedure for building the DRFL on Windows and Linux platforms. For detailed instructions, please refer to the official manual linked below:

[DRFL Manual](https://manual.doosanrobotics.com/en/api/)


## System Requirements


> **Note**: DRFL is designed to operate on x86 architectures. Arm64 architecture is supported exclusively for Linux platforms.

Please ensure your environment meets the following conditions:

- **Library Composition**: Refer to the structure of the library via this [link](https://manual.doosanrobotics.com/help/api/latest/publish/en_us/composition-of-library-36471066.html).
- **Recommended Specifications**: Review the recommended operational specifications [here](https://manual.doosanrobotics.com/help/api/latest/publish/en_us/recommended-operational-specification-50890483.html).


## Build Guidelines
The library files (.a, .dll, etc.) included in this repository are sample versions intended for demonstration purposes. To ensure you are using the latest release of DRFL, please download the current version from the following link:

[Download Latest DRFL](https://robotlab.doosanrobotics.com/en/board/Resources/Software)

### Windows (64-bit)

To build the Windows example, utilize the Visual Studio 2015 solution file provided at the link below:

[Windows Example Solution](https://github.com/doosan-robotics/API-DRFL/blob/main/example/Windows/windows_example/windows_example.sln)


### Linux (64-bit)

#### Ubuntu 

> Ubuntu supports versions : 18.04, 20.04 and 22.04.

1. Navigate to the example directory and compile using g++:
   ```bash
   g++ -c main.cpp
2. Once main.o is created successfully, run the following command to generate the executable:

    **x86(18.04)** 
    ```bash
    g++ -o drfl_test main.o ../../library/Linux/64bits/amd64/{your_ubuntu_version}/libDRFL.a /usr/lib/libPocoFoundation.so /usr/lib/libPocoNet.so
    ```
    **x86(20.04 or 22.04)** 
    ```bash
    g++ -o drfl_test main.o ../../library/Linux/64bits/amd64/{your_ubuntu_version}/libDRFL.a /usr/lib/x86_64-linux-gnu/libPocoFoundation.so /usr/lib/x86_64-linux-gnu/libPocoNet.so
    ```
    **Arm64(18.04)**
    
    ```bash
    g++ -o drfl_test main.o ../../library/Linux/64bits/arn64/{your_ubuntu_version}/libDRFL.a /usr/lib/libPocoFoundation.so /usr/lib/libPocoNet.so
    ```
    **Arm64(20.04 or 22.04)**
    
    ```bash
    g++ -o drfl_test main.o ../../library/Linux/64bits/arn64/{your_ubuntu_version}/libDRFL.a /usr/lib/aarch64-linux-gnu/libPocoFoundation.so /usr/lib/aarch64-linux-gnu/libPocoNet.so
    ```

3. Verify the build and, upon completion, proceed with testing the connection to the actual controller.

    ```bash
    sudo apt-get install libpoco-dev
