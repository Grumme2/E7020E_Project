# E7020E_Project

## Project Description

Our project purpose is to see whether a person is inside a perimeter or not. If the person is outside, a lamp on the board will light up.
This is going to be done with a MURATA board and a gps. We also have to implement a way to see the battery level. With the LoRa we are going to transmit the battery level, and if a user exits the perimeter, we are sending that users location.

## Higher grade

The group is aming for a higher grade than a basic pass. We are going to try to implement a charger of Lipo batteries, so that a user can charge a battery. 

## Components
* GPS: 410-237 - PmodGPS, Module UART, Digilent
* Battery charger: MCP73831T-2ACI/OT
* Battery: PIS-1218 - PiJuice Zero LiPo Battery, 500mAh, Pi Supply

https://docs.google.com/document/d/1sqJ3TY8bH1azIesI4otk2LLjVwCMFet3OU3TrjOT8jw/edit?usp=sharing

## Setup/Starting the project

### Prerequisites

In order to have the project working as intended you need to have the following things installed: 

* rustup 
* llvm-8.0
* llvm-config
* openocd
* gcc
* arm-none-eabi-gcc

### Building/running the project

First install rustup by following the instructions on https://rustup.rs/
After the installation you need to switch to the nightly toolchain rustup can use. First list the versions that are available:

`rustup toolchain list`

Then set the nightly toolchain as default:

`rustup default <nightly-toolchain>`

Then you need to install the following dependencies using rustup:

`rustup target add thumbv6m-none-eabi`

In order to start the project you first have to redirect to the **radio-module** folder.

 `cd radio-module`

 From that you need to make sure that you have a file called **stm32l0_dual_bank.cfg** in the **target** folder. If not, create a new file with that name that contains the following: 

 `source [find target/stm32l0.cfg]
set _FLASHNAME $_CHIPNAME.flash1
flash bank $_FLASHNAME stm32lx 0 0 0 0 $_TARGETNAME`

After this you may finally start **openocd**:

`openocd -f openocd.cfg`

Then open a new terminal window and redirect to the **radio-module** directory, and from that you may run the program: 

`cargo run`

Now you should have a fully functional GPS-tracker on your hands!