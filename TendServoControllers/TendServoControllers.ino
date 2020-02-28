#include <DynamixelSDK.h>

#define ADDR_TORQUE_ENABLE           24           // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION           30
#define ADDR_PRESENT_POSITION        36
#define ADDR_MOVING                  46
#define ADDR_SPEED					 32
#define ADDR_CW_LIMIT			      6
#define ADDR_CCW_LIMIT				  8

// Protocol version
#define PROTOCOL_VERSION              1.0         // See which protocol version is used in the Dynamixel

// Default setting
#define Baudrate                1000000
#define DeviceNAme             "3"				  // DEVICENAME "3" -> Serial3(OpenCM 485 EXP)


#define Servo1					1                 // Dynamixel ID: 1
#define Servo2					2				  // Dynamixel ID: 2
#define Servo3					3
#define Servo4					4
#define Servo5					5
#define Servo6					6
#define Servo7					7
#define Servo8					8
#define Servo9					9
#define Servo10					10
#define Servo11					11
#define Servo12					12

#define TorqueEnable            1                 // Value for enabling the torque
#define TorqueDisable			0                 // Value for disabling the torque
#define CWLimit					100				  // CW angle limit is the minimum value of goal position
#define CCWLimit				800				  // CCW angle limit is the maximum value of goal position
#define Speed					200				  // Speed value [0 ~ 1024]


// Create PortHandler instance
dynamixel::PortHandler* portHandler;

// Create PacketHandler instance
dynamixel::PacketHandler* packetHandler;

//***********Global Variables***********
char NewPosition;
int isMoving = 0;
int comm_result = COMM_TX_FAIL;             // Communication result
uint8_t error = 0;							// Dynamixel error
int16_t present_position = 0;				// Present position

float initial_position = 819.2;
float max_position = initial_position + 34.13;  //original 51.2
float min_position = initial_position - 34.13;

int WaitingTime = 500;




void setup() {
	Serial.begin(115200);
	SerialUSB.begin();

	// Initialize portHandler. Set the port path
	portHandler = dynamixel::PortHandler::getPortHandler(DeviceNAme);

	// Initialize packetHandler. Set the protocol version
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Open port
	if (portHandler->openPort()) {
		Serial.print("Succeeded to open the port!\n");
	}

	// Set port baudrate
	if (portHandler->setBaudRate(Baudrate)) {
		Serial.print("Succeeded to change the baudrate!\n");
	}

	// Enable Dynamixel Torque
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo1, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo2, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo3, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo4, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo5, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo6, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo7, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo8, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo9, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo10, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo11, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	comm_result = packetHandler->write1ByteTxRx(portHandler, Servo12, ADDR_TORQUE_ENABLE, TorqueEnable, &error);
	if (comm_result != COMM_SUCCESS) {
		packetHandler->getTxRxResult(comm_result);
	}
	else if (error != 0) {
		packetHandler->getRxPacketError(error);
	}
	else {
		Serial.print("Dynamixel has been successfully connected \n");
	}

	// Set Control Mode
	// "Wheel Mode" -> both limits are 0.
	// "Joint Mode" -> neither limit is 0.
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo3, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo3, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo4, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo4, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo5, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo5, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo6, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo6, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo7, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo7, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo8, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo8, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo9, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo9, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo10, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo10, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo11, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo11, ADDR_CCW_LIMIT, CCWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo12, ADDR_CW_LIMIT, CWLimit, &error);
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo12, ADDR_CCW_LIMIT, CCWLimit, &error);

	// Set speed
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo3, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo4, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo5, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo6, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo7, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo8, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo9, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo10, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo11, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo12, ADDR_SPEED, Speed, &error);

	// Initial Homing position. Value is 819.2 
	//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_GOAL_POSITION, 0, &error);  // original value 819.2
	//delay(2000);
}

void loop() {
	while (Serial.available()) {
		NewPosition = (char)Serial.read();
		if ((NewPosition == 'L') || (NewPosition == 'l')) {
			Rotate2(Servo1, 819);
			delay(WaitingTime);
			Rotate2(Servo2, 819);
			delay(WaitingTime);
			Rotate2(Servo3, 819);
			delay(WaitingTime);
			Rotate2(Servo4, 819);
			delay(WaitingTime);
			Rotate2(Servo5, 819);
			delay(WaitingTime);
			Rotate2(Servo6, 819);
			delay(WaitingTime);
			Rotate2(Servo7, 819);
			delay(WaitingTime);
			Rotate2(Servo8, 819);
			delay(WaitingTime);
			Rotate2(Servo9, 819);
			delay(WaitingTime);
			Rotate2(Servo10, 819);
			delay(WaitingTime);
			Rotate2(Servo11, 819);
			delay(WaitingTime);
			Rotate2(Servo12, 819);
		}
		if ((NewPosition == 'R') || (NewPosition == 'r')) {
			Rotate2(Servo12, 205);
			delay(WaitingTime);
			Rotate2(Servo11, 205);
			delay(WaitingTime);
			Rotate2(Servo10, 205);
			delay(WaitingTime);
			Rotate2(Servo9, 205);
			delay(WaitingTime);
			Rotate2(Servo8, 205);
			delay(WaitingTime);
			Rotate2(Servo7, 205);
			delay(WaitingTime);
			Rotate2(Servo6, 205);
			delay(WaitingTime);
			Rotate2(Servo5, 205);
			delay(WaitingTime);
			Rotate2(Servo4, 205);
			delay(WaitingTime);
			Rotate2(Servo3, 205);
			delay(WaitingTime);
			Rotate2(Servo2, 205);
			delay(WaitingTime);
			Rotate2(Servo1, 205);
		}
		if ((NewPosition == 'E') || (NewPosition == 'e')) {
			Rotate2(Servo1, 512);
			delay(WaitingTime);
			Rotate2(Servo2, 512);
			delay(WaitingTime);
			Rotate2(Servo3, 512);
			delay(WaitingTime);
			Rotate2(Servo4, 512);
			delay(WaitingTime);
			Rotate2(Servo5, 512);
			delay(WaitingTime);
			Rotate2(Servo6, 512);
			delay(WaitingTime);
			Rotate2(Servo7, 512);
			delay(WaitingTime);
			Rotate2(Servo8, 512);
			delay(WaitingTime);
			Rotate2(Servo9, 512);
			delay(WaitingTime);
			Rotate2(Servo10, 512);
			delay(WaitingTime);
			Rotate2(Servo11, 512);
			delay(WaitingTime);
			Rotate2(Servo12, 512);
		}

	}

}

void Rotate(int position) {

	//packetHandler->read1ByteTxRx(portHandler, Servo1, ADDR_MOVING, (uint8_t*)&isMoving, &error);
	packetHandler->read1ByteTxRx(portHandler, Servo12, ADDR_MOVING, (uint8_t*)&isMoving, &error);


	// If Dynamixel is stopped
	// Send instruction packet to move for goalPosition
	if (isMoving == 0) {
		//comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_GOAL_POSITION, position, &error);
		comm_result = packetHandler->write2ByteTxRx(portHandler, Servo12, ADDR_GOAL_POSITION, position, &error);
	}

	//comm_result = packetHandler->read2ByteTxRx(portHandler, Servo1, ADDR_PRESENT_POSITION, (uint16_t*)&present_position, &error);
	comm_result = packetHandler->read2ByteTxRx(portHandler, Servo12, ADDR_PRESENT_POSITION, (uint16_t*)&present_position, &error);
}

void Rotate2(uint8_t ID, int position) {
	packetHandler->read1ByteTxRx(portHandler, ID, ADDR_MOVING, (uint8_t*)&isMoving, &error);

	// If Dynamixel is stopped
	// Send instruction packet to move for goalPosition
	if (isMoving == 0) {
		comm_result = packetHandler->write2ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, position, &error);
	}
	comm_result = packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, (uint16_t*)&present_position, &error);
}

/*
	//if (isMoving2 == 0) {
	//	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_GOAL_POSITION, goalPosition2, &error);
	//	if (goalPosition2 == max_position) {
	//		goalPosition2 = min_position;
	//	}
	//	else {
	//		goalPosition2 = max_position;
	//	}
	//}
*/

/*
	if ((present_position >= 842) && (direction == false)) {
		//toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000
		//if we are less than 900 and the direction is false
		direction = true;
		goalPosition1 = 0;
		comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_GOAL_POSITION, position, &error);
	}
	if ((present_position <= 800) && (direction == true))
	{
		goalPosition1 = 852;
		direction = false;
		comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_GOAL_POSITION, goalPosition1, &error);
	}
*/