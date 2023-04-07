#define EXPORT_API __declspec(dllexport) 


#include <iostream>

#include "SimpleMotionV2/simplemotion.h"

bool acquired = false;
smint32 deviceAddress;
smbus busHandle;

struct HapticDeviceState
{
	float position; // Linear position of base unit in mm
	float angle; // Orientation of the base unit in rads 
	float thumbWheel1; //Orientation of the large thumbwheel in rads
	float thumbWheel2; //Orientation of the large thumbwheel in rads
	float tool1Position;
	float tool2Position;
	int buttonMask; //All hand piece buttons & foot pedal as bit mask
};

struct HapticDeviceOutput
{
	float linearCommand;
	float angleCommand;
	float wheel1Command;
	float wheel2Command;
	float externalForce;
	bool calibrate; // Set to true to trigger a zeroing of all position values
	bool statusLED; //Toggle the status led on the handpiece
};



typedef void(*FuncPtr)(const char*);

FuncPtr Debug;

extern "C" {

	void EXPORT_API SetDebugFunction(FuncPtr fp)
	{
		Debug = fp;
	}

	bool EXPORT_API InitHapticDevice(const char* comPort)
	{

		//Connect here to GRANITE

		busHandle = smOpenBus(comPort);


		if (busHandle >= 0)
		{
			//logMessage(Info, "Successfully connected bus " + ui->busName->text());
			//deviceAddress = ui->deviceAddress->value();
			deviceAddress = 1;
			acquired = true;
			printf("Successfully connected bus\n");
		}
		else
		{
			//logMessage(Error, "Couldn't connect to bus " + ui->busName->text());
			acquired = false;
			printf("Couldn't connect to bus\n");
		}


		//TODO port implementation from Qt
		//checkAndReportSMBusErrors(false);

		//logMessage(Info, "Enabling drive");
		printf("Enabling drive\n");
		smSetParameter(busHandle, deviceAddress, SMP_CONTROL_BITS1, SMP_CB1_ENABLE);

		//TODO port implementation from Qt
		//checkAndReportSMBusErrors(true);
		
		if (Debug != NULL && acquired)
			Debug("Granite Connected");
		else
			Debug("Error: Granite NOT Connected");

		return acquired;
	}

	int EXPORT_API UpdateHapticDevice(HapticDeviceState* state, HapticDeviceOutput* out)
	{
	//	if (acquired)
		{
			//read the device state 

			state->position = 0.0f;
			state->angle = 1.0f;
			state->thumbWheel1 = 2.0f;
			state->thumbWheel2 = 3.0f;
			state->tool1Position = 4.0f;
			state->tool2Position = 5.0f;
			state->buttonMask = 6;


			//set the motors
			//sharedMemory->linearCommand = out->linearCommand;
			//sharedMemory->angularCommand = out->angleCommand;
			//sharedMemory->wheel1Command = out->wheel1Command;
			//sharedMemory->wheel2Command = out->wheel2Command;
			//sharedMemory->calibrate = out->calibrate;
			//sharedMemory->statusLED = out->statusLED;

			//============================= AUTO WHEEL ==================================================================================
			//void MW::on_setStartAutoWheel_clicked()
			//{
			//	logMessage(Info, "AutoWhell Started");
			//	smSetParameter(busHandle, deviceAddress, SMP_CONTROL_MODE, CM_POSITION);
			//	smSetParameter(busHandle, deviceAddress, SMP_ABSOLUTE_SETPOINT, 0);
			//	smSetParameter(busHandle, deviceAddress, SMP_CONTROL_MODE, CM_TORQUE);
			//	simTimer.start();
			//}

			//void MW::simTimerOnTriggered()
			//{
			//	smint32 position;

			//	smRead1Parameter(busHandle, deviceAddress, SMP_ACTUAL_POSITION_FB, &position);

			//	logMessage(Info, QString("Position %1").arg(position));
			//	if (abs(position) > 100) {
			//		smSetParameter(busHandle, deviceAddress, SMP_ABSOLUTE_SETPOINT, -8 * position);
			//	}
			//	else {
			//		smSetParameter(busHandle, deviceAddress, SMP_ABSOLUTE_SETPOINT, 0);
			//	}
			//}

			//void MW::on_setStopAutoWheel_clicked()
			//{
			//	logMessage(Info, "AutoWhell Stopped");
			//	simTimer.stop();
			//	smSetParameter(busHandle, deviceAddress, SMP_CONTROL_MODE, CM_POSITION);
			//	smSetParameter(busHandle, deviceAddress, SMP_ABSOLUTE_SETPOINT, 0);


			//}


			//============================= AUTO WHEEL ==================================================================================

		}
		return 0;
	}


	int EXPORT_API CloseHapticDevice()
	{

		if (Debug != NULL)
			Debug("Granite Disconnecting");

		acquired = false;

	//	logMessage(Info, "Closing bus (if open)");
		printf("Closing bus (if open)");
		smCloseBus(busHandle);



		return 0;
	}

}


#ifdef _DEBUG

HapticDeviceState *hdState;
HapticDeviceOutput *hdOutput;

int main()
{
	InitHapticDevice("COM13");

	UpdateHapticDevice(hdState, hdOutput);

	//allow some timeout before open/close
	system("pause");

	CloseHapticDevice();

	system("pause");
}
#endif // _DEBUG

