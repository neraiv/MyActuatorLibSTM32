/*
 * Neraiv_MyActuatorLib.c
 *
 *  Created on: Nov 17, 2023
 *      Author: yigit
 */

#include "Neraiv_MyActuatorLib.h"

void clearBuffers(MyCan* _myCan){
	memset(_myCan->rx.data,0,8);
	memset(_myCan->tx.data,0,8);
}

void getMotorParameters(MyActuator* _myActuator){
	_myActuator->myActStat.temp = _myActuator->myCan->rx.data[1];
	_myActuator->myActStat.torqueCurrent =((int16_t)_myActuator->myCan->rx.data[3] << 8) | _myActuator->myCan->rx.data[2];
	_myActuator->myActStat.speed = ((int16_t)_myActuator->myCan->rx.data[5] << 8) | _myActuator->myCan->rx.data[4];
	_myActuator->myActStat.encoder = ((uint16_t)_myActuator->myCan->rx.data[7] << 8) | _myActuator->myCan->rx.data[6];
}

int8_t sendAndRecieveData(MyActuator* _myActuator){
	uint8_t tries = 0;

	do{
		HAL_CAN_AddTxMessage(_myActuator->myCan->hcan, &_myActuator->myCan->tx.header, _myActuator->myCan->tx.data, &_myActuator->myCan->tx.mailbox);
		HAL_Delay(1);
		HAL_CAN_GetRxMessage(_myActuator->myCan->hcan, _myActuator->myCan->canFilter.FilterFIFOAssignment, &_myActuator->myCan->rx.header, _myActuator->myCan->rx.data);
		tries++;
		if(tries > MAX_TRIES_TO_RECIEVE_DATA){
			_myActuator->errors.errorCommunaciton = 1;
			return -1;
		}
	}while((_myActuator->myCan->rx.data[0] != _myActuator->myCan->tx.data[0]) && (_myActuator->canID != _myActuator->myCan->rx.header.StdId));

	return 0;
}

void myErrorHandler(){

}

int8_t myInitCanbus(MyCan* myCan){
	myCan->canFilter.FilterActivation = ENABLE;
	myCan->canFilter.FilterBank = 0;
	myCan->canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	myCan->canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	myCan->canFilter.FilterIdHigh = 0;
	myCan->canFilter.FilterIdLow = 0;
	myCan->canFilter.FilterMaskIdHigh = 0x0000;
	myCan->canFilter.FilterMaskIdLow = 0x0000;
	myCan->canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	myCan->canFilter.SlaveStartFilterBank = 14;

	myCan->tx.header.IDE = CAN_ID_STD;
	myCan->tx.header.DLC = 8;

	if(HAL_CAN_ConfigFilter(myCan->hcan, &myCan->canFilter) != HAL_OK){
		return 1;
	}
	if(HAL_CAN_Start(myCan->hcan) != HAL_OK){
		return 1;
	}
	return 0;
}

int8_t myCreateActuator(MyActuator* _myActuator, MyCan* myCan, uint32_t canID, uint16_t maxspeed){
	_myActuator->canID = canID;
	_myActuator->maxSpeed = maxspeed;
	_myActuator->myCan = myCan;
	return 0;
}

void myReadPID(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = READ_PID;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	_myActuator->myPIDVals.PosKI = _myActuator->myCan->rx.data[2];
	_myActuator->myPIDVals.PosKP = _myActuator->myCan->rx.data[3];
	_myActuator->myPIDVals.SpdKP = _myActuator->myCan->rx.data[4];
	_myActuator->myPIDVals.SpdKI = _myActuator->myCan->rx.data[5];
	_myActuator->myPIDVals.TorqueKP = _myActuator->myCan->rx.data[6];
	_myActuator->myPIDVals.TorqueKP = _myActuator->myCan->rx.data[7];


	clearBuffers(_myActuator->myCan);
}

void myReadAccelPID(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = READ_ACCEL_PID;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}
	_myActuator->myPIDVals.accelSpeed =((int32_t)_myActuator->myCan->rx.data[5] << 8 | _myActuator->myCan->rx.data[4]);
	_myActuator->myPIDVals.accelTorque =((int32_t)_myActuator->myCan->rx.data[7] << 8 | _myActuator->myCan->rx.data[6]);

	clearBuffers(_myActuator->myCan);
}

void myReadEncoder(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = READ_ENCODER;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	_myActuator->myActStat.encoder =((uint16_t)_myActuator->myCan->rx.data[3] << 8 | _myActuator->myCan->rx.data[2]);
	_myActuator->myActStat.encoderRaw =((uint16_t)_myActuator->myCan->rx.data[5] << 8 | _myActuator->myCan->rx.data[4]);
	_myActuator->myActStat.encoderOffset =((uint16_t)_myActuator->myCan->rx.data[7] << 8 | _myActuator->myCan->rx.data[6]);

	clearBuffers(_myActuator->myCan);
}

void myReadSingleTurnAngle(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = READ_SINGLE_TURN_ANGLE;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}
	_myActuator->myActStat.singleTurneAngle = ((int16_t) _myActuator->myCan->rx.data[7]<<8 | _myActuator->myCan->rx.data[6]);

	clearBuffers(_myActuator->myCan);
}

void myReadMultiTurnAngle(MyActuator* _myActuator){

	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = READ_MULTI_TURN_ANGLE;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	_myActuator->myActStat.multiTurnAngle = ((int64_t)0x0000<<56 |_myActuator->myCan->rx.data[7] << 48 | _myActuator->myCan->rx.data[6]<<40 | _myActuator->myCan->rx.data[5]<<32 | _myActuator->myCan->rx.data[4]<<24 | _myActuator->myCan->rx.data[3]<<16 | _myActuator->myCan->rx.data[2]<<8 | _myActuator->myCan->rx.data[1]);

	clearBuffers(_myActuator->myCan);
}

void myReadMotorStatus(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = READ_MOTOR_STATUS_1;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	_myActuator->myActStat.temp = (int8_t)_myActuator->myCan->rx.data[1];
	_myActuator->myActStat.voltage =((uint16_t)_myActuator->myCan->rx.data[4] << 8) | _myActuator->myCan->rx.data[3];
	_myActuator->errors.errorVoltage = ((_myActuator->myCan->rx.data[7] & 0x01) ? ERROR_MOTOR : 0);
	_myActuator->errors.errorTemp = ((_myActuator->myCan->rx.data[7] & 0x08) ? ERROR_MOTOR : 0);

	clearBuffers(_myActuator->myCan);

	_myActuator->myCan->tx.data[0] = READ_MOTOR_STATUS_2;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	_myActuator->myActStat.temp = (int8_t)_myActuator->myCan->rx.data[1];
	_myActuator->myActStat.torqueCurrent =((int16_t)_myActuator->myCan->rx.data[3] << 8) | _myActuator->myCan->rx.data[2];
	_myActuator->myActStat.speed =((int16_t)_myActuator->myCan->rx.data[5] << 8) | _myActuator->myCan->rx.data[4];
	_myActuator->myActStat.encoder =((uint16_t)_myActuator->myCan->rx.data[7] << 8) | _myActuator->myCan->rx.data[6];

	clearBuffers(_myActuator->myCan);

	_myActuator->myCan->tx.data[0] = READ_MOTOR_STATUS_3;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	_myActuator->myActStat.temp = _myActuator->myCan->rx.data[1];
	_myActuator->myActStat.phaseACurrent =((int16_t)_myActuator->myCan->rx.data[3] << 8) | _myActuator->myCan->rx.data[2];
	_myActuator->myActStat.phaseBCurrent =((int16_t)_myActuator->myCan->rx.data[5] << 8) | _myActuator->myCan->rx.data[4];
	_myActuator->myActStat.phaseCCurrent =((int16_t)_myActuator->myCan->rx.data[7] << 8) | _myActuator->myCan->rx.data[6];

}

void myWritePID(MyActuator* _myActuator, uint8_t angleKp, uint8_t angleKi, uint8_t speedKp, uint8_t speedKi, uint8_t torqueKp, uint8_t torqueKi, int8_t WRITE_PID_RAM_or_ROM){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = WRITE_PID_RAM_or_ROM;
	_myActuator->myCan->tx.data[2] = angleKp;
	_myActuator->myCan->tx.data[3] = angleKi;
	_myActuator->myCan->tx.data[4] = speedKp;
	_myActuator->myCan->tx.data[5] = speedKi;
	_myActuator->myCan->tx.data[6] = torqueKp;
	_myActuator->myCan->tx.data[7] = torqueKi;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	clearBuffers(_myActuator->myCan);
}

void myWriteAccelToRAM(MyActuator* _myActuator, int32_t accel){

	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = WRITE_ACCEL_PID_RAM;

	_myActuator->myCan->tx.data[4] = (uint8_t)(accel);
	_myActuator->myCan->tx.data[5] = (uint8_t)(accel>>8);
	_myActuator->myCan->tx.data[6] = (uint8_t)(accel>>16);
	_myActuator->myCan->tx.data[7] = (uint8_t)(accel>>24);

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	clearBuffers(_myActuator->myCan);
}

void myWriteEncoderOffset(MyActuator* _myActuator, uint16_t offset){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = WRITE_ENCODER_OFFSET;

	_myActuator->myCan->tx.data[6] = (uint8_t)(offset);
	_myActuator->myCan->tx.data[7] = (uint8_t)(offset>>8);

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	clearBuffers(_myActuator->myCan);
}

void myWriteEncoderZeroAsCurrent(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = WRITE_ENCODER_ZERO_AS_CURRENT;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	_myActuator->myActStat.encoderOffset = ((uint16_t)_myActuator->myCan->rx.data[7] << 8) | _myActuator->myCan->rx.data[6];


	clearBuffers(_myActuator->myCan);
}

void myControlMultiTurn(MyActuator* _myActuator,int32_t angle, uint16_t speed, uint8_t MULTI_TURN_CONTROLtype){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = MULTI_TURN_CONTROLtype;

	if(MULTI_TURN_CONTROLtype == CONTROL_MULTI_TURN_WITH_SPEED){
		_myActuator->myCan->tx.data[2] = (uint8_t)(speed);
		_myActuator->myCan->tx.data[3] = (uint8_t)(speed>>8);
	}

	_myActuator->myCan->tx.data[4] = (uint8_t)(angle);
	_myActuator->myCan->tx.data[5] = (uint8_t)(angle>>8);
	_myActuator->myCan->tx.data[6] = (uint8_t)(angle>>16);
	_myActuator->myCan->tx.data[7] = (uint8_t)(angle>>24);

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	getMotorParameters(_myActuator);

	clearBuffers(_myActuator->myCan);
}

void myControlSingleTurn(MyActuator* _myActuator,uint8_t spin_direction, uint16_t angle, uint16_t speed, uint8_t SINGLE_TURN_CONTROLtype){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = SINGLE_TURN_CONTROLtype;
	_myActuator->myCan->tx.data[1] = spin_direction;

	if(SINGLE_TURN_CONTROLtype == CONTROL_SINGLE_TURN_WITH_SPEED){
		_myActuator->myCan->tx.data[2] = (uint8_t)(speed);
		_myActuator->myCan->tx.data[3] = (uint8_t)(speed>>8);
	}

	_myActuator->myCan->tx.data[4] = (uint8_t)(angle);
	_myActuator->myCan->tx.data[5] = (uint8_t)(angle>>8);

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	getMotorParameters(_myActuator);

	clearBuffers(_myActuator->myCan);
}

void myControlTorque(MyActuator* _myActuator, int16_t torqueCurrent){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = CONTROL_TORQUE;

	_myActuator->myCan->tx.data[4] = (uint8_t)(torqueCurrent);
	_myActuator->myCan->tx.data[5] = (uint8_t)(torqueCurrent>>8);

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	getMotorParameters(_myActuator);

	clearBuffers(_myActuator->myCan);
}

void myControlSpeed(MyActuator* _myActuator, int32_t speed){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = CONTROL_SPEED;

	_myActuator->myCan->tx.data[4] = (uint8_t)(speed);
	_myActuator->myCan->tx.data[5] = (uint8_t)(speed>>8);
	_myActuator->myCan->tx.data[6] = (uint8_t)(speed>>16);
	_myActuator->myCan->tx.data[7] = (uint8_t)(speed>>24);

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	getMotorParameters(_myActuator);

	clearBuffers(_myActuator->myCan);
}

void myControlStop(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = CONTROL_MOTOR_STOP;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	clearBuffers(_myActuator->myCan);
}

void myControlOff(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = CONTROL_MOTOR_OFF;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	clearBuffers(_myActuator->myCan);
}

void myClearErrorFlags(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = ERROR_CLEAR_FLAGS;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	clearBuffers(_myActuator->myCan);
}

void myControlRunning(MyActuator* _myActuator){
	_myActuator->myCan->tx.header.StdId = _myActuator->canID;

	_myActuator->myCan->tx.data[0] = CONTROL_MOTOR_RUNNING;

	if(sendAndRecieveData(_myActuator) < 0){
		myErrorHandler();
		return;
	}

	clearBuffers(_myActuator->myCan);
}
