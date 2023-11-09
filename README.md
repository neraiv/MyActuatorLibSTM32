# MyActuatorLibSTM32
My Actuator Library for STM32 using HAL Libraries. This library targets RMD motors. These motors is controlled through CAN-BUS with 1 Mbs u can controll multiple motors with one CAN-BUS (Such a cool devices huh 😄).

# Features
  - ## Gets motor parameters 

    Every read method starts with myRead...
    - Internal temperature
    - Single and multi turn angle
    - Torque and Phases (A,B,C) current
    - Current speed
    - Error status
    - PID parameters

  - ## Can drive motor with multiple control methods

    Every control method starts with myControl...
    - Single turn control with or without speed
    - Multi turn control with or without speed
    - Speed control
    - Stop motor

  - ## Can write parameters

    Every write method starts with myWrite...
    - Write encoder zero

# Usage

- Select your canbus in CubeMx (or If u are using CubeIde u can use integraded CubeMx GUI) set it to 1Mbs (Because motor works with 1Mbs)

- Create as many myActuators as you need and myCan object. If ALL of ur motors are in same CanBus line just one MyCan is enough. If u have many CanBuses u can create new ones.

  ```C
  MyActuator myActuator1;
  MyActuator myActuator2;
  MyCan myCan;
  ```
- Initiliaze CanBus and give ur motor an ID if its already have one

  ```C
  myCan.hcanx = YOUR_CAN_Handle // Like hcan1 for canbus 1
  myActuator1.canID = YOUR_CAN_ID1 //Like 0x143 motor IDs start starts from 0x140 to 0x180
  myActuator2.canID = YOUR_CAN_ID2 
  myInitCanBus(MmyCan);
  ```



