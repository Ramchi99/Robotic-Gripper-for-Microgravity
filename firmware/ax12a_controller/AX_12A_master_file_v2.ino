#include <DynamixelShield.h>

/*
Baud Rate Values for Dynamixel Motors:

Value   Baud Rate (bps)    Margin of Error
1       1M                  0.000%
3       500,000             0.000%
4       400,000             0.000%
7       250,000             0.000%
9       200,000             0.000%
16      115200              -2.124%
34      57600               0.794%
103     19200               -0.160%
207     9600                -0.160%
*/

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

//This example is written for DYNAMIXEL AX-12A with Protocol 1.0.

#define OPERATING_MODE_ADDR_LEN     2

/*The Control Table is divided into 2 Areas. 
Data in the RAM Area is reset to initial values 
when the power is reset(Volatile). On the other 
hand, data in the EEPROM Area is maintained even 
when the device is powered off(Non-Volatile).*/

//Control Table of EEPROM Area
#define MODEL_NUMBER_ADDR           0
#define MODEL_NUMBER_ADDR_LEN       2
#define FIRMWARE_VERSION_ADDR       2
#define FIRMWARE_VERSION_ADDR_LEN   1
#define ID_ADDR                     3
#define ID_ADDR_LEN                 1
#define BAUDRATE_ADDR               4
#define BAUDRATE_ADDR_LEN           1
#define RETURN_DELAY_TIME_ADDR      5
#define RETURN_DELAY_TIME_ADDR_LEN  1
#define CW_ANGLE_LIMIT_ADDR         6
#define CW_ANGLE_LIMIT_ADDR_LEN     2
#define CCW_ANGLE_LIMIT_ADDR        8
#define CCW_ANGLE_LIMIT_ADDR_LEN    2
#define TEMPERATURE_LIMIT_ADDR      11
#define TEMPERATURE_LIMIT_ADDR_LEN  1
#define MIN_VOLTAGE_LIMIT_ADDR      12
#define MIN_VOLTAGE_LIMIT_ADDR_LEN  1
#define MAX_VOLTAGE_LIMIT_ADDR      13
#define MAX_VOLTAGE_LIMIT_ADDR_LEN  1
#define MAX_TORQUE_ADDR             14
#define MAX_TORQUE_ADDR_LEN         2
#define STATUS_RETURN_LEVEL_ADDR    16
#define STATUS_RETURN_LEVEL_ADDR_LEN 1
#define ALARM_LED_ADDR              17
#define ALARM_LED_ADDR_LEN          1
#define SHUTDOWN_ADDR               18
#define SHUTDOWN_ADDR_LEN           1

//Control Table of RAM Area
#define TORQUE_ENABLE_ADDR           24
#define TORQUE_ENABLE_ADDR_LEN       1
#define LED_ADDR                     25
#define LED_ADDR_LEN                 1
#define CW_COMPLIANCE_MARGIN_ADDR    26
#define CW_COMPLIANCE_MARGIN_ADDR_LEN 1
#define CCW_COMPLIANCE_MARGIN_ADDR   27
#define CCW_COMPLIANCE_MARGIN_ADDR_LEN 1
#define CW_COMPLIANCE_SLOPE_ADDR     28
#define CW_COMPLIANCE_SLOPE_ADDR_LEN 1
#define CCW_COMPLIANCE_SLOPE_ADDR    29
#define CCW_COMPLIANCE_SLOPE_ADDR_LEN 1
#define GOAL_POSITION_ADDR           30
#define GOAL_POSITION_ADDR_LEN       2
#define MOVING_SPEED_ADDR            32
#define MOVING_SPEED_ADDR_LEN        2
#define TORQUE_LIMIT_ADDR            34
#define TORQUE_LIMIT_ADDR_LEN        2
#define PRESENT_POSITION_ADDR        36
#define PRESENT_POSITION_ADDR_LEN    2
#define PRESENT_SPEED_ADDR           38
#define PRESENT_SPEED_ADDR_LEN       2
#define PRESENT_LOAD_ADDR            40
#define PRESENT_LOAD_ADDR_LEN        2
#define PRESENT_VOLTAGE_ADDR         42
#define PRESENT_VOLTAGE_ADDR_LEN     1
#define PRESENT_TEMPERATURE_ADDR     43
#define PRESENT_TEMPERATURE_ADDR_LEN 1
#define REGISTERED_ADDR              44
#define REGISTERED_ADDR_LEN          1
#define MOVING_ADDR                  46
#define MOVING_ADDR_LEN              1
#define LOCK_ADDR                    47
#define LOCK_ADDR_LEN                1
#define PUNCH_ADDR                   48
#define PUNCH_ADDR_LEN               2

#define TIMEOUT 100    //default communication timeout 10ms



const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;
const uint32_t baudrate = 1000000;

DynamixelShield dxl;

//use as parameters for some configurations

uint8_t returned_id = 0;
uint8_t returned_baudrate = 0;
uint16_t present_position = 0;
uint16_t present_speed = 0;

uint8_t turn_on = 1;
uint8_t turn_off = 0;

uint16_t goalPosition1 = 300;
uint16_t goalPosition2 = 500;

float Kp = 2; // Proportional gain
uint16_t goal_position = 1022; // Default goal position (middle of the range)


void setup() {
  DEBUG_SERIAL.begin(115200);   // Set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         // Wait until the serial port for terminal is opened

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate (AX-12A).
  dxl.begin(baudrate);
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.print("Read for PROTOCOL ");
  DEBUG_SERIAL.print(DXL_PROTOCOL_VERSION, 1);
  DEBUG_SERIAL.print(", ID ");
  DEBUG_SERIAL.println(DXL_ID);

  // Read DYNAMIXEL ID
  dxl.read(DXL_ID, ID_ADDR, ID_ADDR_LEN, (uint8_t*)&returned_id, sizeof(returned_id), TIMEOUT);
  DEBUG_SERIAL.print("ID : ");
  DEBUG_SERIAL.println(returned_id);
  delay(100);
  
  // Read DYNAMIXEL Baudrate
  dxl.read(DXL_ID, BAUDRATE_ADDR, BAUDRATE_ADDR_LEN, (uint8_t*)&returned_baudrate, sizeof(returned_baudrate), TIMEOUT);
  DEBUG_SERIAL.print("Baud Rate : ");
  DEBUG_SERIAL.println(returned_baudrate);
  delay(100);
  
  // Read DYNAMIXEL Present Position
  dxl.read(DXL_ID, PRESENT_POSITION_ADDR, PRESENT_POSITION_ADDR_LEN, (uint8_t*)&present_position, sizeof(present_position), TIMEOUT);
  DEBUG_SERIAL.print("Present Position : ");
  DEBUG_SERIAL.println(present_position);

  // It configures the Dynamixel motor to joint mode, limiting its rotation within specified angles.
  delay(10);
  
  // Turn off torque when configuring items in EEPROM area
  if(dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("DYNAMIXEL Torque off");
  else
    DEBUG_SERIAL.println("Error: Torque off failed");

  delay(10);
  
  // Set CW and CCW angle limits for joint mode (min position = 300, max position = 700)
  uint16_t min_position = 0;
  uint16_t max_position = 1023; //0; //
  if(dxl.write(DXL_ID, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&min_position, CW_ANGLE_LIMIT_ADDR_LEN, TIMEOUT)
        && dxl.write(DXL_ID, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&max_position, CCW_ANGLE_LIMIT_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Set operating mode");
  else
    DEBUG_SERIAL.println("Error: Set operating mode failed");

  delay(10);
  
  // Set goal position to 300
  uint16_t goal_position = 1023;
  if(dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t*)&goal_position, GOAL_POSITION_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Set goal position");
  else
    DEBUG_SERIAL.println("Error: Set goal position failed / Or motor is in wheel mode!");

  delay(10);

  

  // Set maximum torque 
  uint16_t max_torque = 1023 * 0.15; // 15% of max torque (1023)
  if(dxl.write(DXL_ID, MAX_TORQUE_ADDR, (uint8_t*)&max_torque, MAX_TORQUE_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Set max torque");
  else
    DEBUG_SERIAL.println("Error: Set max torque failed");

  delay(10);

  // Set torque limit 
  uint16_t torque_limit = 1023 * 0.15; // 15% of max torque (1023)
  if(dxl.write(DXL_ID, TORQUE_LIMIT_ADDR, (uint8_t*)&torque_limit, TORQUE_LIMIT_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Set torque limit");
  else
    DEBUG_SERIAL.println("Error: Set torque limit failed");

  delay(10);

  // Turn on torque
  if(dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Torque on");
  else
    DEBUG_SERIAL.println("Error: Torque on failed");
}


void loop() {

  delay(20);
  //readLoad();
  //controlMotor();
  //readSpeed();
  controlMotorInJointMode();
  //currentPosition();
  //controlLoop();
  //updateGoalPosition();

}

/*
void moveMotor(uint16_t start, uint16_t end) {
  uint16_t currentPos = start;
  while (currentPos != end) {
    //DEBUG_SERIAL.print("Moving to position: ");
    //DEBUG_SERIAL.println(end);
    
    dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t*)&end, GOAL_POSITION_ADDR_LEN, TIMEOUT);
    
    // Continuously read present position until goal is reached
    do {
      dxl.read(DXL_ID, PRESENT_POSITION_ADDR, PRESENT_POSITION_ADDR_LEN, (uint8_t*)&currentPos, sizeof(currentPos), TIMEOUT);
      //DEBUG_SERIAL.print("Current Position: ");
      DEBUG_SERIAL.println(currentPos);
      delay(100); // Adjust delay according to your motor's speed
    } while (currentPos != end);
  }
}
*/


void readLoad() {
  uint16_t loadValue = 0;
  dxl.read(DXL_ID, PRESENT_LOAD_ADDR, PRESENT_LOAD_ADDR_LEN, (uint8_t*)&loadValue, sizeof(loadValue), TIMEOUT);
  //DEBUG_SERIAL.print("Present Load: ");
  //DEBUG_SERIAL.println(loadValue);
  float load = convertLoadToPercentage(loadValue);
  DEBUG_SERIAL.println(loadValue);
  //DEBUG_SERIAL.println(load);
}

float convertLoadToPercentage(int loadValue) {
  // Convert load value to percentage
  float percentage = 0.0;
  
  // Check direction
  if (loadValue < 1024) {
    // CCW direction
    percentage = (loadValue / 10.23); // Convert to percentage
  } else {
    // CW direction
    percentage = ((1024 - loadValue) / 10.23); // Convert to percentage
  }
  
  return percentage;
}


uint16_t calculateSpeed(uint8_t percentage, bool clockwise) {
  if (percentage > 100) percentage = 100;
  uint16_t speed = (percentage * 1023) / 100; // Scale 0-100 to 0-1023
  if (clockwise) {
    speed += 1024; // Offset for clockwise direction
  }
  return speed;
}


/*
void readSpeed() {
  dxl.read(DXL_ID, PRESENT_SPEED_ADDR, PRESENT_SPEED_ADDR_LEN, (uint8_t*)&present_speed, sizeof(present_speed), TIMEOUT);
  
  int speedValue = present_speed & 0x3FF; // Mask out the direction bit (10th bit)
  bool clockwise = (present_speed & 0x400) != 0; // Check the direction bit
  
  int speedPercentage = (speedValue * 1000) / 1023; // Convert to percentage

  if (clockwise) {
    DEBUG_SERIAL.print("Current Speed: ");
    DEBUG_SERIAL.print(speedPercentage);
    DEBUG_SERIAL.println(" (CW)");
  } else {
    speedPercentage = -speedPercentage; // Make it negative for CCW
    DEBUG_SERIAL.print("Current Speed: ");
    DEBUG_SERIAL.print(speedPercentage);
    DEBUG_SERIAL.println(" (CCW)");
  }
}
*/

/*
void controlMotor() {
  static bool upPressed = false;
  static bool downPressed = false;
  static uint8_t speedPercentage = 5; // Default speed percentage (100% of max speed)

  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'u') {
      upPressed = true;
      downPressed = false;
    } else if (command == 'd') {
      downPressed = true;
      upPressed = false;
    } else if (command == 's') { // Stop command
      upPressed = false;
      downPressed = false;
    } else if (command >= '0' && command <= '9') {
      speedPercentage = (command - '0'); // * 10; // Set speed percentage (0-90%)
    }
  }

  if (upPressed) {
    uint16_t speed = calculateSpeed(speedPercentage, true);
    dxl.write(DXL_ID, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
  } else if (downPressed) {
    uint16_t speed = calculateSpeed(speedPercentage, false);
    dxl.write(DXL_ID, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
  } else {
    uint16_t speed = 0; // Stop the motor
    dxl.write(DXL_ID, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
  }
}
*/

void controlMotorInJointMode() {
  static uint16_t goal_position = 1022; // Default goal position (middle of the range)
  static uint8_t speedPercentage = 100; // Default speed percentage (100% of max speed)
  bool positionChanged = false;
  uint16_t current_position = readPosition();
  
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'u' && goal_position < 1013) {
      goal_position += 10; // Increase goal position
      positionChanged = true;
    } else if (command == 'd' && goal_position > 10) {
      goal_position -= 10; // Decrease goal position
      positionChanged = true;
    } else if (command >= '0' && command <= '9') {
      speedPercentage = (command - '0') * 10; // Set speed percentage (0-90%)
    }
  }

  if (positionChanged) {
    dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t*)&goal_position, GOAL_POSITION_ADDR_LEN, TIMEOUT);
    uint16_t speed = calculateSpeed(speedPercentage, true);
    dxl.write(DXL_ID, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
    //DEBUG_SERIAL.print("Goal Position: ");
    //DEBUG_SERIAL.println(goal_position);
    //Serial.print("   Present Position: ");
    //Serial.println(current_position);
  }
}


/*
void controlLoop() {
  // Read current position
  uint16_t current_position = readPosition();

  // Calculate error
  int16_t error = goal_position - current_position;

  // Calculate speed using proportional control
  int16_t speed = Kp * error;

  // Set the speed to drive the motor
  //dxl.write(DXL_ID, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
  // Set the speed and direction to drive the motor
  if (speed >= 0) {
    dxl.write(DXL_ID, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
  } else {
    // Set direction bit for CW direction
    speed = -speed; // Make speed positive
    speed |= 0x400; // Set direction bit
    dxl.write(DXL_ID, MOVING_SPEED_ADDR, (uint8_t*)&speed, MOVING_SPEED_ADDR_LEN, TIMEOUT);
  }

  // Print goal position and present position
  Serial.print("Goal Position: ");
  Serial.print(goal_position);
  Serial.print("   Present Position: ");
  Serial.println(current_position);

  // Delay for stability
  delay(100);
}
*/

uint16_t readPosition() {
  uint16_t current_position = 0;
  dxl.read(DXL_ID, PRESENT_POSITION_ADDR, PRESENT_POSITION_ADDR_LEN, (uint8_t*)&current_position, sizeof(current_position), TIMEOUT);
  return current_position;
}

void currentPosition() {
  uint16_t current_position = 0;
  dxl.read(DXL_ID, PRESENT_POSITION_ADDR, PRESENT_POSITION_ADDR_LEN, (uint8_t*)&current_position, sizeof(current_position), TIMEOUT);
  //Serial.print("   Current Position: ");
  //Serial.println(current_position);
}

void updateGoalPosition() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'u' && goal_position < 1013) {
      goal_position += 10; // Increase goal position
    } else if (command == 'd' && goal_position > 10) {
      goal_position -= 10; // Decrease goal position
    }
  }
}