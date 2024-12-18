/*
 * Neraiv_MyActuatorLib.h
 *
 *  Created on: Nov 17, 2023
 *      Author: yigit
 */

#ifndef INC_NERAIV_MYACTUATORLIB_H_
#define INC_NERAIV_MYACTUATORLIB_H_

#include "main.h"
#include "string.h"

#define MYACTUATOR_V1
//#define MYACTUATOR_V2

#define MAX_TRIES_TO_RECIEVE_DATA 20
/*
 * Can TransmitX struct for MyCan
 */
typedef struct{
	CAN_TxHeaderTypeDef   header;
	uint8_t               data[8];
	uint32_t              mailbox;
}CanTx;

/*
 * Can RecieveX struct for MyCan
 */
typedef struct{
	CAN_RxHeaderTypeDef   header;
	uint8_t 			  data[8];
	uint32_t			  mailbox;
}CanRx;

/*
 * Full canbus struct to store all data
 */
typedef struct{
	CAN_HandleTypeDef* 		hcan;
	CAN_FilterTypeDef		canFilter;
	CanTx					tx;
	CanRx					rx;
}MyCan;

/*
 * PID parameters struct
 */
typedef struct{
	uint8_t 	TorqueKP;
	uint8_t 	TorqueKI;
	uint8_t 	SpdKP;
	uint8_t 	SpdKI;
	uint8_t 	PosKP;
	uint8_t 	PosKI;
	int32_t     accelSpeed;
	int32_t		accelTorque;
}MyPIDVals;

/*
 * Motor status struct to read and store motor values.
 */
typedef struct{
	uint8_t  	temp;
	uint8_t  	brake;
	uint16_t 	voltage;
	int64_t  	multiTurnAngle;
	int16_t		singleTurneAngle;
	int16_t  	torqueCurrent;
	int16_t  	speed;
	uint16_t 	encoder;
	uint16_t 	encoderRaw;
	uint16_t 	encoderOffset;
	int16_t 	phaseACurrent;
	int16_t 	phaseBCurrent;
	int16_t		phaseCCurrent;
}MyStatus;

/*
 * Motor driver system data (This function is not avaliable on V2 motors)
 */
#ifdef MYACTUATOR_V2
typedef struct{
	char     	model[7];
	uint32_t 	runtime;
	uint32_t	versionDate;
}SysData;
#endif
/*
 * Motor errors struct. Driver sets these when motor is in an error state.
 */
typedef struct{
	uint8_t  	errorVoltage;
	uint8_t  	errorTemp;
	uint8_t		errorCommunaciton;
}MyError;

/*
 * Final motor struct to keep all struct together. This is the struct that u want to create when a motor connected.
 */
typedef struct{
	uint32_t  	canID;
	uint16_t	maxSpeed;
    MyCan*		myCan;
	MyError 	errors;
	MyStatus 	myActStat;
	MyPIDVals 	myPIDVals;
#ifdef MYACTUATOR_V2
	SysData   	sysData;
#endif
}MyActuator;


/*
 *
 */
#define ERROR_MOTOR 33

/*
 *	Motor Commands
 *	ToDo : Change commands for V3 motors. Add missing commands
 */
#ifdef MYACTUATOR_V1
	#define READ_PID 						0x30
	#define READ_ACCEL_PID					0x33
	#define READ_ENCODER 					0x90

	#define READ_SINGLE_TURN_ANGLE			0x94
	#define READ_MULTI_TURN_ANGLE 			0x92
	#define READ_MOTOR_STATUS_1 			0x9A //ToDo split it in function
	#define READ_MOTOR_STATUS_2 			0x9C
	#define READ_MOTOR_STATUS_3 			0x9D

	#define WRITE_PID_RAM					0x31
	#define WRITE_PID_ROM					0x32
	#define WRITE_ACCEL_PID_RAM				0x34
	#define WRITE_ENCODER_OFFSET			0x91
	#define WRITE_ENCODER_ZERO_AS_CURRENT 	0x19

	#define CONTROL_TORQUE					0x88
	#define CONTROL_SPEED 					0xA2
	#define CONTROL_MULTI_TURN 				0xA3
	#define CONTROL_MULTI_TURN_WITH_SPEED 	0xA4
	#define CONTROL_SINGLE_TURN 			0xA5
	#define CONTROL_SINGLE_TURN_WITH_SPEED 	0xA6
	#define CONTROL_MOTOR_STOP 				0x81
	#define CONTROL_MOTOR_OFF				0x80


	#define ERROR_CLEAR_FLAGS				0x9B
	#define CONTROL_MOTOR_RUNNING			0x88
#endif

/**
 * @brief Initializes the CAN bus for a MyActuator device.
 *
 * This function sets up the CAN bus configuration for communication with a MyActuator device.
 * It initializes the CAN filter and prepares the CAN interface for transmission and reception.
 *
 * @param _myCan Pointer to a MyCan structure that will be initialized with the CAN bus configuration.
 *               This structure should be pre-allocated by the caller.
 *
 * @return int8_t Returns 0 on successful initialization, or a non-zero error code if initialization fails.
 */
int8_t myInitCanbus(MyCan* _myCan);

/**
 * @brief Creates and initializes a MyActuator object.
 *
 * This function sets up a MyActuator structure with the provided parameters,
 * associating it with a CAN interface and setting its basic properties.
 *
 * @param _myActuator Pointer to the MyActuator structure to be initialized.
 * @param _myCan Pointer to the MyCan structure representing the CAN interface.
 * @param canID The CAN ID to be assigned to the actuator.
 * @param maxspeed The maximum speed limit for the actuator.
 *
 * @return int8_t Returns 0 on successful initialization, or a non-zero error code on failure.
 */
int8_t myCreateActuator(MyActuator* _myActuator, MyCan* _myCan, uint32_t canID, uint16_t maxspeed);


/**
 * 
 * @brief Sends and receives data over the CAN bus.
 * 
 * This function sends a CAN message using the provided MyCan structure, and waits for a response.
 * It will attempt to receive data up to a maximum number of tries, and will return an error if no response is received.
 * 
 * @param _myActuator Pointer to the MyActuator structure representing the actuator.
 */
void myReadPID(MyActuator* _myActuator);


/**
 * @brief Reads the encoder value of the specified actuator.
 * 
 * This function reads the current encoder value of the given actuator and updates
 * the actuator's internal state with the new encoder value.
 * 
 * @param _myActuator Pointer to the MyActuator instance whose encoder value is to be read.
 */
void myReadEncoder(MyActuator* _myActuator);

/**
 * @brief Reads the single turn angle of the specified actuator.
 * 
 * This function reads the current single turn angle of the given actuator and updates
 * the actuator's internal state with the new angle value.
 * 
 * @param _myActuator Pointer to the MyActuator instance whose single turn angle is to be read.
 */
void myReadMotorStatus(MyActuator* _myActuator);

/**
 * @brief Reads the multi turn angle of the specified actuator.
 * 
 * This function reads the current multi turn angle of the given actuator and updates
 * the actuator's internal state with the new angle value.
 * 
 * @param _myActuator Pointer to the MyActuator instance whose multi turn angle is to be read.
 */
void myReadMultiTurnAngle(MyActuator* _myActuator);

/**
 * @brief Reads the single turn angle of the specified actuator.
 * 
 * This function reads the current single turn angle of the given actuator and updates
 * the actuator's internal state with the new angle value.
 * 
 * @param _myActuator Pointer to the MyActuator instance whose single turn angle is to be read.
 */
void myReadAll( MyActuator* _myActuator);

#ifdef MYACTUATOR_V2

/**
 * @brief Reads the system data from the specified MyActuator instance.
 * 
 * This function retrieves and processes the system data from the given 
 * MyActuator instance. The data read can include various parameters 
 * and states relevant to the actuator's operation.
 * 
 * @param _myActuator Pointer to the MyActuator instance from which 
 *                    the system data will be read.
 */
void myReadSystemData(MyActuator* _myActuator);
#endif

/**
 * @brief Writes the PID values to the specified actuator.
 * 
 * This function writes the PID values to the specified actuator, updating
 * the actuator's internal state with the new PID values.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the PID values will be written.
 * @param angleKp The proportional gain for the angle control loop.
 * @param angleKi The integral gain for the angle control loop.
 * @param speedKp The proportional gain for the speed control loop.
 * @param speedKi The integral gain for the speed control loop.
 * @param torqueKp The proportional gain for the torque control loop.
 * @param torqueKi The integral gain for the torque control loop.
 * @param WRITE_PID_RAM_or_ROM The type of memory to write the PID values to (RAM or ROM).
 */
void myWritePID(MyActuator* _myActuator, uint8_t angleKp, uint8_t angleKi, uint8_t speedKp, uint8_t speedKi, uint8_t torqueKp, uint8_t torqueKi, int8_t WRITE_PID_RAM_or_ROM); //ToDo

/**
 * @brief Writes the acceleration value to the specified actuator.
 * 
 * This function writes the acceleration value to the specified actuator, updating
 * the actuator's internal state with the new acceleration value.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the acceleration value will be written.
 * @param accel The acceleration value to be written to the actuator.
 */
void myWriteEncoderZero(MyActuator* _myActuator);

/**
 * @brief Writes the acceleration value to the specified actuator.
 * 
 * This function writes the acceleration value to the specified actuator, updating
 * the actuator's internal state with the new acceleration value.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the acceleration value will be written.
 * @param accel The acceleration value to be written to the actuator.
 */
void myWriteEncoderOffset(MyActuator* _myActuator, uint16_t offset);

/**
 * @brief Writes the acceleration value to the specified actuator.
 * 
 * This function writes the acceleration value to the specified actuator, updating
 * the actuator's internal state with the new acceleration value.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the acceleration value will be written.
 * @param accel The acceleration value to be written to the actuator.
 */
void myControlMultiTurn(MyActuator* _myActuator,int32_t angle, uint16_t speed, uint8_t MULTI_LOOP_CONTROLtype);

/**
 * @brief Writes the acceleration value to the specified actuator.
 * 
 * This function writes the acceleration value to the specified actuator, updating
 * the actuator's internal state with the new acceleration value.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the acceleration value will be written.
 * @param accel The acceleration value to be written to the actuator.
 */
void myControlSingleTurn(MyActuator* _myActuator,uint8_t spin_direction, uint16_t angle, uint16_t speed, uint8_t SINGLE_TURN_CONTROLtype);

/**
 * @brief Writes the acceleration value to the specified actuator.
 * 
 * This function writes the acceleration value to the specified actuator, updating
 * the actuator's internal state with the new acceleration value.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the acceleration value will be written.
 * @param accel The acceleration value to be written to the actuator.
 */
void myControlSpeed(MyActuator* _myActuator, int32_t speed);

/**
 * @brief Writes the acceleration value to the specified actuator.
 * 
 * This function writes the acceleration value to the specified actuator, updating
 * the actuator's internal state with the new acceleration value.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the acceleration value will be written.
 * @param accel The acceleration value to be written to the actuator.
 */
void myControlStop(MyActuator* _myActuator);

/**
 * @brief Writes the acceleration value to the specified actuator.
 * 
 * This function writes the acceleration value to the specified actuator, updating
 * the actuator's internal state with the new acceleration value.
 * 
 * @param _myActuator Pointer to the MyActuator instance to which the acceleration value will be written.
 * @param accel The acceleration value to be written to the actuator.
 */
void myClearErrorFlags(MyActuator* _myActuator);

#endif /* INC_NERAIV_MYACTUATORLIB_H_ */
