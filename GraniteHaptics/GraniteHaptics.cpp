#define EXPORT_API __declspec(dllexport) 


#include <iostream>

#include "SimpleMotionV2/simplemotion.h"

bool acquired = false;
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
		smSetTimeout(1000);
		smSetBaudrate(460800);//set SM default baudrate which is needed to reconnect after increased baudrate (so needed for consequent connect if "use high baudrate" option was set)

		busHandle = smOpenBus(comPort);

		if (busHandle < 0)
		{
			acquired = false;
			printf("Couldn't connect to bus\n");
		}
		else {
			printf("Connected to bus\n");
		}

		resetCumulativeStatus(busHandle);//reset possible SM errors

		int setBPS = 3000000;
		
		/*max deviceTimeoutMs valid value 10230 ms. however, this should be set _less_ than timeout period of SM host
			 * the value that we set earlier here with smSetTimeout) so if device host timeouts,
			 *it will cause certain timeout on device and reset baudrate to default for successfull reinitialization*/
		const int deviceTimeoutMs = 500;

		//first set device timeout (watchdog/fault behavior), so if connection is lost, they reset to default baud rate after a certain time period
//note: we change these settings of all bus devices simultaneously because errors will happen if not all devices have same BPS (address 0=broadcast to all)
		smSetParameter(busHandle, 0, SMP_FAULT_BEHAVIOR, (deviceTimeoutMs / 10) << 8);//set timeout
		smSetParameter(busHandle, 0, SMP_BUS_SPEED, setBPS);//set baudrate

		//if all went ok, now device is in new baud rate, switch host PBS too
		smCloseBus(busHandle);
		smSetBaudrate(setBPS);
		busHandle = smOpenBus(comPort);

		if (busHandle < 0)
		{
			acquired = false;
			printf("Couldn't connect to bus\n");
		}
		else {
			printf("Connected in high speed\n");
			acquired = true;
		}

		resetCumulativeStatus(busHandle);//reset possible SM errors

		//change smFastUpdateCycle data format
		smSetParameter(busHandle, 1, SMP_FAST_UPDATE_CYCLE_FORMAT, FAST_UPDATE_CYCLE_FORMAT_ALT1);
		smSetParameter(busHandle, 2, SMP_FAST_UPDATE_CYCLE_FORMAT, FAST_UPDATE_CYCLE_FORMAT_ALT1);
		smSetParameter(busHandle, 3, SMP_FAST_UPDATE_CYCLE_FORMAT, FAST_UPDATE_CYCLE_FORMAT_ALT1);
		smSetParameter(busHandle, 4, SMP_FAST_UPDATE_CYCLE_FORMAT, FAST_UPDATE_CYCLE_FORMAT_ALT1);

		if (Debug != NULL && acquired)
			Debug("Granite Connected");
		else
			Debug("Error: Granite NOT Connected");

		return acquired;
	}

	int EXPORT_API UpdateHapticDevice(HapticDeviceState* state, HapticDeviceOutput* out)
	{
		FastUpdateCycleReadData readData;
		FastUpdateCycleWriteData writeData;
	//	if (acquired)
		{
			//read the device state 

			//state->position = 0.0f;
			//state->angle = 1.0f;
			//state->thumbWheel1 = 2.0f;
			//state->thumbWheel2 = 3.0f;
			//state->tool1Position = 4.0f;
			//state->tool2Position = 5.0f;
			//state->buttonMask = 6;

			
			writeData.ALT1_Write.CB1_Enable = 1;//write enable with fast command. without this, drive gets disabled
			writeData.ALT1_Write.CB1_BypassTrajPlanner = 1;//write bypass trajectory planner with fast command
			writeData.ALT1_Write.CB1_QuickStopSet = 0;//do not activate quick stop
			writeData.ALT1_Write.Setpoint = out->linearCommand;//write setpoint

			smFastUpdateCycleWithStructs(busHandle, 1, writeData, &readData);

			state->position = readData.ALT1_ALT2_Read.PositionFeedback;


			writeData.ALT1_Write.CB1_Enable = 1;//write enable with fast command. without this, drive gets disabled
			writeData.ALT1_Write.CB1_BypassTrajPlanner = 1;//write bypass trajectory planner with fast command
			writeData.ALT1_Write.CB1_QuickStopSet = 0;//do not activate quick stop
			writeData.ALT1_Write.Setpoint = out->angleCommand;//write setpoint

			smFastUpdateCycleWithStructs(busHandle, 2, writeData, &readData);

			state->angle = readData.ALT1_ALT2_Read.PositionFeedback;


			writeData.ALT1_Write.CB1_Enable = 1;//write enable with fast command. without this, drive gets disabled
			writeData.ALT1_Write.CB1_BypassTrajPlanner = 1;//write bypass trajectory planner with fast command
			writeData.ALT1_Write.CB1_QuickStopSet = 0;//do not activate quick stop
			writeData.ALT1_Write.Setpoint = out->wheel1Command;//write setpoint

			smFastUpdateCycleWithStructs(busHandle, 3, writeData, &readData);

			state->thumbWheel1 = readData.ALT1_ALT2_Read.PositionFeedback;


			writeData.ALT1_Write.CB1_Enable = 1;//write enable with fast command. without this, drive gets disabled
			writeData.ALT1_Write.CB1_BypassTrajPlanner = 1;//write bypass trajectory planner with fast command
			writeData.ALT1_Write.CB1_QuickStopSet = 0;//do not activate quick stop
			writeData.ALT1_Write.Setpoint = out->wheel2Command;//write setpoint

			smFastUpdateCycleWithStructs(busHandle, 4, writeData, &readData);

			state->thumbWheel2 = readData.ALT1_ALT2_Read.PositionFeedback;

			//set the motors
			//sharedMemory->linearCommand = out->linearCommand;
			//sharedMemory->angularCommand = out->angleCommand;
			//sharedMemory->wheel1Command = out->wheel1Command;
			//sharedMemory->wheel2Command = out->wheel2Command;
			//sharedMemory->calibrate = out->calibrate;
			//sharedMemory->statusLED = out->statusLED;

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
		smSetParameter(busHandle, 1, SMP_ABSOLUTE_SETPOINT, 0);
		smSetParameter(busHandle, 2, SMP_ABSOLUTE_SETPOINT, 0);
		smSetParameter(busHandle, 3, SMP_ABSOLUTE_SETPOINT, 0);
		smSetParameter(busHandle, 4, SMP_ABSOLUTE_SETPOINT, 0);
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

