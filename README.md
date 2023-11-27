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
    - Acceleration parameters

  - ## Can drive motor with multiple control methods

    Every control method starts with myControl...
    - Torque control
    - Speed control
    - Single turn control with or without speed
    - Multi turn control with or without speed
    - Stop and shutdown motor

  - ## Can write parameters

    Every write method starts with myWrite...
    - PID parameters to RAM or ROM
    - Acceleration parameters
    - Encoder zero
   
  - ## Error checkers
  - Check and clear error flags
  - Check motor state (runing or stop)

# Usage

- Select your canbus in CubeMx (or If u are using CubeIde u can use integraded CubeMx GUI) set it to 1Mbs (Because motor works with 1Mbs)

- Create as many myActuators as you need and myCan object. If ALL of ur motors are in same CanBus line just one MyCan is enough. If u have many CanBuses u can create new ones.

  ```C
  MyActuator myActuator1;
  MyActuator myActuator2;
  MyCan myCan1;
  ```
- Initiliaze CanBus.

  ```C
  myCan.hcanx = YOUR_CAN_Handle // Like hcan1 for canbus 1
  myInitCanBus(myCan1);
  ```
- Then initliaze ur motor and match them with a myCan using myCreateActuator function. U must give motor ID to myActuator struct to communicate with it via Canbus and give it a maxspeed u can change this speed later.

  ```C
  myCreateActuator(&myActuator1,&myCan1,0x141,500);  // CANID of motor is 0x141 and max speed is 500 degree per second
  myCreateActuator(&myActuator2,&myCan1,0x142,5000); // CANID of motor is 0x142 and max speed is 5000 degree per second
  ```

- Now u can use all functions. Try myReadMotorStatus function to read temperature, encoder, current etc.
  ```C
  myReadMotorStatus(&myActuator1);
  ```

