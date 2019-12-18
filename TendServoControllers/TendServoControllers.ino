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

#define TorqueEnable            1                 // Value for enabling the torque
#define TorqueDisable			0                 // Value for disabling the torque
#define CWLimit					min_position      // CW angle limit is the minimum value of goal position
#define CCWLimit				max_position	  // CCW angle limit is the maximum value of goal position
#define Speed					10				  // Speed value [0 ~ 1024]...20... 5 seems working fine

// Create PortHandler instance
dynamixel::PortHandler* portHandler;

// Create PacketHandler instance
dynamixel::PacketHandler* packetHandler;

//***********Global Variables***********
int comm_result = COMM_TX_FAIL;             // Communication result
uint8_t error = 0;							// Dynamixel error
int16_t present_position = 0;				// Present position

float initial_position = 819.2;
float max_position = initial_position + 34.13;  //initial was 51.2 (15 degrees), changed to 10 degrees = 34.13
float min_position = initial_position - 34.13;

int goalPosition1 = 0;
int goalPosition2 = 0;
int isMoving1 = 0;
int isMoving2 = 0;


void setup() {
	Serial.begin(115200);

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
	if (comm_result != COMM_SUCCESS) {
		packetHandler->getTxRxResult(comm_result);
	}
	else {
		Serial.print("Dynamixel has been successfully connected \n");
	}
	// Set Control Mode
	// "Wheel Mode" -> both limits are 0.
	// "Joint Mode" -> neither limit is 0.
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_CW_LIMIT, CWLimit, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_CCW_LIMIT, CCWLimit, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_CW_LIMIT, CWLimit, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_CCW_LIMIT, CCWLimit, &error);

	// Set speed
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_SPEED, Speed, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_SPEED, Speed, &error);

	// Initial Homing position. Value is 819.2 
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_GOAL_POSITION, 819.2, &error);
	comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_GOAL_POSITION, 819.2, &error);
}

void loop() {
	packetHandler->read1ByteTxRx(portHandler, Servo1, ADDR_MOVING, (uint8_t*)&isMoving1, &error);
	packetHandler->read1ByteTxRx(portHandler, Servo2, ADDR_MOVING, (uint8_t*)&isMoving2, &error);
	if (isMoving1 == 0) {
		//Send instruction packet to move for goalPosition
		comm_result = packetHandler->write2ByteTxRx(portHandler, Servo1, ADDR_GOAL_POSITION, goalPosition1, &error);
		comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_GOAL_POSITION, goalPosition1, &error);
		//toggle the position if goalPosition is 1000, set to 0, if 0, set to 1000
		if (goalPosition1 == 1000) {
			goalPosition1 = 0;
		}
		else {
			goalPosition1 = 1000;
		}
	}
	/*if (isMoving2 == 0) {
		comm_result = packetHandler->write2ByteTxRx(portHandler, Servo2, ADDR_GOAL_POSITION, goalPosition2, &error);
		if (goalPosition2 == 1000) {
			goalPosition2 = 0;
		}
		else {
			goalPosition2 = 1000;
		}
	}*/
	packetHandler->read2ByteTxRx(portHandler, Servo1, ADDR_PRESENT_POSITION, (uint16_t*)&present_position, &error);
	packetHandler->read2ByteTxRx(portHandler, Servo2, ADDR_PRESENT_POSITION, (uint16_t*)&present_position, &error);

	//Serial.print(present_position);
	//Serial.print("\t");
	//Serial.println(goalPosition1);
}