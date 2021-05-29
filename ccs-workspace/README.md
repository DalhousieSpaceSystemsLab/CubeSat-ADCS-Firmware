# ADCS Firmware

This repository is for the development of the embedded firmware running on the ADCS hardware's microcontroller. The target device is the Texas Instruments MSP430F5529 mcu. 

The existing firmware repository is currently being migrated to a Code Composer Studio (CCS) IDE project for ease of continued development. Previous development was done using a custom vs-code build system in Linux. The previous version can be viewed by checking out /CMake-Built branch. Continued CCS developement should be done on /dev branch.  

See following link for IDE download:  
https://www.ti.com/tool/CCSTUDIO  

## Firmware Design    

Architecture separated into 3 layers. Each layer is implemented using the layer below it.  
- Core: Target-independent application  
- Devices: APIs and Vendor drivers  
- Drivers: Target-specific register level control   
 
#### Figure-1: Firmware Dependency Graph
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/firmware-dependency-graph.png?raw=true" alt="Firmware Dependency Graph"/>
</p>

#### Figure-2: Process 1: Main Firmware Loop
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/process-1-main-firmware-loop.png?raw=true" alt="Process 1: Main Firmware Loop"/>
</p>

#### Figure-3: Subprocess 1.1: Peripheral Configuration
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/subprocess-1.1-peripheral-configuration.png?raw=true" alt="Subprocess 1.1: Peripheral Configuration"/>
</p>

#### Figure-4: Subprocess 1.2: Process OBC Command
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/subprocess-1.2-process-obc-command.png?raw=true" alt="Subprocess 1.2: Process OBC Command"/>
</p>

#### Figure-5: Subprocess 1.2.1: Check if JSON Command Supported
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/subprocess-1.2.1-check-if-json-command-supported.png?raw=true" alt="Subprocess 1.2.1: Check if JSON Command Supported"/>
</p>

## Repository Structure  

The firmware for deployment on the embedded microcontroller is found in the /firmware-main CCS project. 

Other files such as drivers for testing a specific function of the code are saved in separate CCS projects in the workspace using the following naming convention:  
/filename.example  

This approach is undertaken because there can only be one main.c file per project. Inside of each /filename.example project, the project's properties need to be set so that the include path for the compiler includes the required source and header files in the /firmware-main project in the workspace. This is so that only one copy of the source and header files are used by all the example projects as a "sole source of truth" and avoids version control issues associated with redundant copies of files. 

### Creating New Example   

