# D2_Vehicle_Controller Repository
 - D2 Vehicle Main Controller
 - Project name : wsc2025_Master
### Branch
 - main : release branch
 
### Environment
 - STM32cubeIDE environment
    - version : 1.16.1

# Target Board
 - NUCLEO-G491RE
 - ioc Configuration
 
   <img src="NUCLEO-G491RE.png" alt="NUCLEO-G491RE.png" height = "400"/>
   <img src="ioc.png" alt="ioc.png" height = "400"/>
 - Daughterboard

   <img src="BD.png" alt="BD.png" height = "400"/>

    - DCDC Converter 5V
      - Convert 12V into 5V
    - DCDC Converter 3V3
      - Convert 12V into 3.3V
    - Pulse in and level shifter
      - Translate Voltage level (of 5V) to 3.3V
    - CAN Transceiver
      - Transmitt and Receive CAN Message from CAN BUS
    - PWM to AD
      - Converte PWM (Duty scale of 0% to 100%) to DC Level(scale of 0V to 5V respectively)
    - SPDT switch
      - Select signal to be delivered to Motor Accel
    - I2C
      - IMU , Dotmatrix LCD, etcs
    - UART1
      - GNSS 
    - UART4
      - Telemetry

### Peripheral Setup
  - Those are subject to be changed (since it has not been configured as per Datasheet of each peri and actual use case)
  - Update Configuration according to the feature you are developing
    - Timer (for Pulse counter, PWM outpuit)
    - ADC (to read ADC level)
    - PLL (In case you need to adjust pre-scaler for any frequency components, please ask for update...)
    - CAN (Intiallly it has been set to 500kBPS, nominal pre-scaler : 68, Clock devided by 1)
    - GPIO (PU, PD, DriveStrength to be configured)
    - UART (Mode : Async, Baudrate, WordLength, Parity, StopBit)
    - I2C Setup (Mode : I2C, I2C Speed Frequency)
    - ...

# State


# Tasks
 - Task
   - Routine
 
# Vehicle Database

# 

# Version History


