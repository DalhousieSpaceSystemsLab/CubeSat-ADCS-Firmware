# ADCS Firmware

This repository is for the development of the embedded firmware running on the ADCS hardware's microcontroller. The target device is the Texas Instruments MSP430F5529 mcu. 

The existing firmware repository is currently being migrated to a Code Composer Studio (CCS) IDE project for ease of continued development. Previous development was done using a custom vs-code build system in Linux. The previous version can be viewed by checking out /CMake-Built branch. Continued CCS developement should be done on /dev branch.  

See following link for IDE download:  
https://www.ti.com/tool/CCSTUDIO  

# Table of Contents
- [Firmware Design](#firmware-design)    
- [Repository Structure](#firmware-and-development-environment)
  * [Creating New Example](#)   
- [OBC Simulation](#)
- [About the LORIS Project](#about-the-loris-project)
- [What is an ADCS](#what-is-an-adcs)

# cloning repository - local setup
1. Start by downloading Code Composer Studio IDE: https://www.ti.com/tool/CCSTUDIO?utm_source=google&utm_medium=cpc&utm_campaign=epd-der-null-code_composer-cpc-evm-google-wwe&utm_content=code_composer&ds_k=code+composer&DCM=yes&gclid=CjwKCAjwoZWHBhBgEiwAiMN66TawHLv5GIA5KT6bzWSHOIJ7lSXV6wT62M3T5EF7DMVeqHHu3dSMyBoC2VYQAvD_BwE&gclsrc=aw.ds#downloads
	
2. Next clone the DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware repo onto your local machine. Location should not be important but I tried two different locations and only the second one under documents worked for me.
	
3. Download and install the compiler TI v18.12.8.LTS from the following link: https://www.ti.com/tool/download/MSP-CGT-18
	
4. Open up code composer studio.
Choose CubeSat-ADCS-Firmware\ccs-workspace as your workspace. 
Select Project>>Import CCS Projects and then click on browse. 
Choose your ccs-workspace folder from the github repo and four projects should show up as found. 
Select all of them and then select finish. 
You should now have these four projects open in your Code Composer Studio.

![image](https://user-images.githubusercontent.com/72110751/124775876-5b0fc700-df15-11eb-9b8c-760c95f6beed.png)
	
5.  Select the project imu-api-example.
Go to Project >> Show Build Settings >> General >> Project >> Device and fill in the boxes as shown. Our microcontroller is the MSP430F5529. Do not worry about the Connection box until the MSP430 is actually plugged into your computer.

![image](https://user-images.githubusercontent.com/72110751/124775921-67941f80-df15-11eb-80a5-ffca14e95d0b.png)
	
6. In build settings to the right of the compiler version option click on More >> Add and then navigate to the folder of your downloaded TI v18.12.8.LTS compiler.
Now return to the show build settings menu and choose TI v18.12.8.LTS from the dropdown.

![image](https://user-images.githubusercontent.com/72110751/124776431-d3768800-df15-11eb-8075-45e704fa73b5.png)
	
7. From the linker command file box under Projectchoose lnk_msp430f5529.cmd which should be an option in the dropdown menu.
	
8. Right click imu_api_example and select build project. The project should build with no errors. If you recieve a warning of unresolved symbols make sure that you have the correct linker selected.
There may be a high number of warnings but many of these are custom warnings set up by previous team members and not actual problems.

![image](https://user-images.githubusercontent.com/72110751/124776577-ef7a2980-df15-11eb-9d54-dd7612294992.png)
	
If you have any questions about this process do not hesitate to contact Jasper Grant through Microsoft teams or at jasper.grant@dal.ca.


## Firmware Design    

Architecture separated into 3 layers. Each layer is implemented using the layer below it.  
- Core: Target-independent application  
- Devices: APIs and Vendor drivers  
- Drivers: Target-specific register level control 

Benefits:  
- Decoupling: Changes to hardware revisions minimally impact firmware
- Portability: Target device can be changed in the future.

#### Figure-1: Firmware Dependency Graph
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/firmware-dependency-graph.png?raw=true" alt="Firmware Dependency Graph"/>
</p>

- ADCS behaves as a slave device to OBC. The hardware/firmware behaviour is entirely controlled by the ADCS software running on OBC. 
- Commands allow the ADCS software to control the states of the hardware firmware, ie:
  * Command to write the speed value of the reaction wheels
  * Command to write the dipole magnitude of the magnetorquer
  * Command to request sensor measurement
- Onboard watchdog runs to reset device state in case of solar??protonation??event or device fault.

#### Figure-2: Process 1: Main Firmware Loop
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/process-1-main-firmware-loop.png?raw=true" alt="Process 1: Main Firmware Loop"/>
</p>

#### Figure-3: Subprocess 1.1: Peripheral Configuration
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/subprocess-1.1-peripheral-configuration.png?raw=true" alt="Subprocess 1.1: Peripheral Configuration"/>
</p>

- Data interchange format is serialized JSON with an XOR checksum
- Communication PHY is 8N1 UART at a 57600 baud rate
- The specification for OBC command is specified in the ADCS payload interface document 

#### Figure-4: Subprocess 1.2: Process OBC Command
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/subprocess-1.2-process-obc-command.png?raw=true" alt="Subprocess 1.2: Process OBC Command"/>
</p>

#### Figure-5: Subprocess 1.2.1: Check if JSON Command Supported
<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/drawio-diagrams/subprocess-1.2.1-check-if-json-command-supported.png?raw=true" alt="Subprocess 1.2.1: Check if JSON Command Supported"/>
</p>

## Creating a new project

<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/images/new-project-creation.jpg?raw=true" alt=""/>
</p>

## Repository Structure  

The firmware for deployment on the embedded microcontroller is found in the /firmware-main CCS project. 

Other files such as drivers for testing a specific function of the code are saved in separate CCS projects in the workspace using the following naming convention:  

**\*-api-example**  for examples that use firmware-main api function calls  
**\*-reg-example**  for examples that use register level manipulation  

<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/images/naming-convention.jpg?raw=true" alt=""/>
</p>

This approach is undertaken because there can only be one main.c file per project. Inside of each /filename.example project, the project's properties need to be set so that the include path for the compiler includes the required source and header files in the /firmware-main project in the workspace. This is so that only one copy of the source and header files are used by all the example projects as a "sole source of truth" and avoids version control issues associated with redundant copies of files.


@todo fill in template below to match repo

### A typical top-level directory layout
    .
    ????????? build                   # Compiled files (alternatively `dist`)
    ????????? docs                    # Documentation files (alternatively `doc`)
    ????????? src                     # Source files (alternatively `lib` or `app`)
    ????????? test                    # Automated tests (alternatively `spec` or `tests`)
    ????????? tools                   # Tools and utilities
    ????????? LICENSE
    ????????? README.md


    .
    ????????? ...
    ????????? test                    # Test files (alternatively `spec` or `tests`)
    ???   ????????? benchmarks          # Load and stress tests
    ???   ????????? integration         # End-to-end, integration tests (alternatively `e2e`)
    ???   ????????? unit                # Unit tests
    ????????? ...


### Creating New Example   

## Flashing and Debugging Code on Motherboard PCB

<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/images/flashing-motherboard-with-launchpad.jpg?raw=true" alt=""/>
</p>

## OBC Simulation

Automated tool was developed to emulate OBC commands to ADCS hardware using a Bus Pirate v3.6 and Python. Allows verification of hardware/firmware behavior when commands issued to system.  

<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/images/bus-pirate.jpg?raw=true" alt="Bus pirate connected to MSP430"/>
</p>

<p align="center">
  <img src="https://github.com/DalhousieSpaceSystemsLab/CubeSat-ADCS-Firmware/blob/dev/resources/images/bus-pirate-terminal-output.jpg?raw=true" alt="Bus pirate terminal output"/>
</p>

# About the LORIS Project

The Low Orbit Reconnaissance & Imaging Satellite (LORIS) is a student project underway at Dalhousie University funded by the Canadian Space Agency (CSA). Dalhousie Space Systems Lab???s (DSS) mission is to deploy a two-unit CubeSat with the objective of developing skills in the areas of space systems engineering and of deploying a satellite capable of taking pictures of Earth and sending them back to a ground station at Dalhousie University. Since the project???s start, students at Dalhousie have been working on designing the subsystems required for the LORIS mission with guidance from the CSA and Dalhousie Faculty. In orbit, the satellite must be capable of maintaining a nadir pointing accuracy to ensure ground station communication and imaging requirements are met. The Attitude Determination and Control System (ADCS) is the subsystem responsible for determining and maintaining the attitude of the satellite and orienting its attitude with the requirements of the mission objective. Due to the pointing requirements, the ADCS subsystem is mission critical.  

The goal of this project is to develop, test, and deliver the hardware and firmware for ADCS on the LORIS CubeSat. Our project will review the current design of the ADCS system and provide manufacturable PCB designs and functioning firmware for the final ADCS flight boards of the LORIS mission. 

Once deployed, LORIS will orbit the Earth in a low earth orbit (LEO) at an altitude of about 400 km. The satellite mission???s payload is two cameras that will be used to take pictures of the Nova Scotia peninsula. Due to this imaging payload, the mission entails a high accuracy pointing requirement.

For more information, please visit
https://dalorbits.ca/2019/07/01/loris-2021/


# What is an ADCS

For spacecraft with mission critical pointing requirements, Attitude Determination and Control Systems (ADCS) are used. Attitude is the orientation of an aerospace vehicle with respect to an inertial frame of reference, in LORIS???s case this frame of reference is the Earth???s. LORIS requires  a nadir-pointing (the vector pointing to center of Earth) accuracy of ?? 5?? along the satellite???s Z axis during nominal operation. On a cubesat, the ADCS is one of the most mission critical subsystems.

<p align="center">
  <img src="https://github.com/cmattatall/adcs_firmware/blob/dev/resources/images/nadir.jpg?raw=true" alt="Satellite Pointing"/>
</p>

The primary purpose ot the ADCS system is to maintain the satellite's nadir pointing. There are several modes of operation for the system (Eclipsed, Detumbling, burnwire, and pointing). The following gif is a visualization of reducing the angular rate of the satellite (post deployment) by operating the ADCS in detumbling mode. The gif was produced from orbital simulations of LORIS as part of Anna Wailand's master's thesis on detumbling a cubesat using the B-dot algorithm and proportional control of the earth's magnetic dipole moment.

<p align="center">
  <img src="https://github.com/cmattatall/adcs_firmware/blob/master/resources/captures/detumbling_sim.gif?raw=true" alt="ADCS Detumbling Animation"/>
</p>

