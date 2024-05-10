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
/*
 *	You must set CAN Baud Rate to 1Mbs
 */
int8_t myInitCanbus(MyCan* _myCan);

/*
 *
 */
int8_t myCreateActuator(MyActuator* _myActuator, MyCan* _myCan, uint32_t canID, uint16_t maxspeed);

/*
 * Read PID values
 */
void myReadPID(MyActuator* _myActuator);

/*
 * Reads single turn encoder data.
 */
void myReadEncoder(MyActuator* _myActuator);

/*
 * Reads all motor parameters
 */
void myReadMotorStatus(MyActuator* _myActuator);

/*
 * Reads Multi Turn Angle
 */
void myReadMultiTurnAngle(MyActuator* _myActuator);

/*
 * 	Reads All parameters
 */
void myReadAll( MyActuator* _myActuator);

/*
 * Read System Data
 * Not Avaliable for V2 motors
 */
#ifdef MYACTUATOR_V2
void myReadSystemData(MyActuator* _myActuator);
#endif

/*
 * Write PID values
 */
void myWritePID(MyActuator* _myActuator, uint8_t angleKp, uint8_t angleKi, uint8_t speedKp, uint8_t speedKi, uint8_t torqueKp, uint8_t torqueKi, int8_t WRITE_PID_RAM_or_ROM); //ToDo

/*
 * Changes Encoder Zero
 */
void myWriteEncoderZero(MyActuator* _myActuator);

/*
 *
 */
void myWriteEncoderOffset(MyActuator* _myActuator, uint16_t offset);
/*
 * Turns motor to a specific degree, supports multiple turn
 * 500 , 1 dps
 */
void myControlMultiTurn(MyActuator* _myActuator,int32_t angle, uint16_t speed, uint8_t MULTI_LOOP_CONTROLtype);

/*
 * Tek tur içinde verilen noktaya verilen dönme yönünde döndürür
 * Spin direction 0 CW, 1 CCW
 */
void myControlSingleTurn(MyActuator* _myActuator,uint8_t spin_direction, uint16_t angle, uint16_t speed, uint8_t SINGLE_TURN_CONTROLtype);

// Saniyede speed * 0.01 DPS 1 dönüş için geçen süre
/*
 * Verilen yönde döndür. +- ye göre.
 * 50 000 , 0.01 dps
 */
void myControlSpeed(MyActuator* _myActuator, int32_t speed);

/*
 * Stops motor turning
 */
void myControlStop(MyActuator* _myActuator);

/*
 * Error: flags
 */
void myClearErrorFlags(MyActuator* _myActuator);

#endif /* INC_NERAIV_MYACTUATORLIB_H_ */
