#include <Arduino.h>

// #define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

/*
Mega Interrupt pins:
Mega, Mega2560, MegaADK - 2, 3, 18, 19, 20, 21

Wires for Arduino:
- Black stripe = Motor B Output
*/

#define DEBUG true

#define Bluetooth Serial2
#define MotorController Serial1

#define p_startByte 0x01
#define p_endByte 0x04

#define p_sensorDataId 0xD7 // sd = 215
#define p_stopMotorsId 0xE0 // sm = 224
#define p_parentalOverrideId 0xDF // po = 223

#define p_MAXDataLength 8

// Sabertooth Commands
// Limits for each motor
#define SBT_MOTOR1_FULL_FORWARD 127
#define SBT_MOTOR1_FULL_REVERSE 1

#define SBT_MOTOR2_FULL_FORWARD 255
#define SBT_MOTOR2_FULL_REVERSE 128

#define SBT_MOTOR1_STOP 64
#define SBT_MOTOR2_STOP 192

// Shut down both motors
#define SBT_ALL_STOP  0

/* For left motor, A and B pins are flipped to account for sign of motor direction  */
#define LeftMotorEncoderAPin 2
#define LeftMotorEncoderBPin 3

#define RightMotorEncoderAPin 21
#define RightMotorEncoderBPin 20

Encoder leftMotorEncoder(2, 3);
Encoder rightMotorEncoder(21, 20);

/*
Variables
*/

// the possible states of the state-machine
typedef enum {  BT_Idle, BT_SensorData,
                BT_StopMotors, BT_ParentOverride 
             } BluetoothStates;

// current state-machine state
BluetoothStates bluetooth_state = BT_StopMotors;

// Packet timer
uint16_t period = 1200; // in milliseconds
unsigned long time_now = 0;
unsigned long time_now2 = 0;

static byte last_ud = 0;
static byte last_lr = 0;

/*
Structs/Classes
*/

typedef struct GoPacket_t
{
  byte start_byte = p_startByte;
  byte id;
  uint32_t crc32_chksum;
  bool ack = false;
  byte data_size_in_bytes;
  byte data[p_MAXDataLength] = {0};
  byte end_byte = p_endByte;

} GoPacket;

/*
Helper functions
*/

void PrintHex83(uint8_t* data, uint8_t length) // prints 8-bit data in hex
{
    char tmp[length * 2 + 1];
    byte first;
    int j = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        first = (data[i] >> 4) | 48;
        if (first > 57)
            tmp[j] = first + (byte)39;
        else
            tmp[j] = first;
        j++;

        first = (data[i] & 0x0F) | 48;
        if (first > 57)
            tmp[j] = first + (byte)39;
        else
            tmp[j] = first;
        j++;
    }

    tmp[length * 2] = 0;
    Serial.println(tmp);
}

// ask about how to properly handle isr
// problem of knowing that we've not gotten
// a packet/bluetooth has disconnected from Arduino perspective
bool main_acked = false;
uint16_t safetyTimer = 600; // in milliseconds
bool motorsStopped = false;
void safetyIsrMotors()
{
    if (millis() > time_now2 + safetyTimer)
    {
        time_now2 = millis();

        if (main_acked == false)
        {
          // send stop command to motor controller
          MotorController.write(0x00);
          
          if(motorsStopped == false)
            {
              Serial.println("Safety: Stopping Motors");
              motorsStopped = true;
            }
        }
        else if(main_acked == true)
        {
            motorsStopped = false;
        }

        main_acked = false;
    }
}

float m_drive(byte v)
{
  return (-.376 * v) + 48;
}


float m_angular_offset(byte v)
{
  return (v * .117) - 15;
}

/*
bluetooth State-Machine
*/

void processStopMotors(){
  Serial.println("StopMotors State!");
  Serial.println("Stopping Motors: Sending 0x00");

  // send 0x00 to motor controller on board
  MotorController.write(0x00);
}


uint32_t composeUInt32(byte byte1, byte byte2, byte byte3, byte byte4) 
{
  return ((long) byte4 << 24) + ((long) byte3 << 16) + ((long) byte2 << 8) + ((long) byte1);
}

void processSensorData(byte data[]){
  /*
    Ex. Packet - indexes 2-5 are for checksum; indexes 7-15 are for data
    Index:    0     1     2     3     4     5     6     7     8     9     10     11     12     13     14     15     16
           {  01    03    00    00    00    00    01    08    01    02    03     04     05     06     07     08     04  }
  */
  // Serial.println("SensorData State!");

  // Serial.print("AccelXY: "); PrintHex83(data, sizeof(data));

  last_ud = data[0];
  last_lr = data[1];

  last_ud = (byte) constrain(last_ud, 0, 255);
  last_lr = (byte) constrain(last_lr, 0, 255);

  byte m_ud1  = (byte) floor(m_drive(last_ud) + SBT_MOTOR1_STOP);
  byte m_ud2  = (byte) floor(m_drive(last_ud) + SBT_MOTOR2_STOP);
  byte offset = (byte) floor(m_angular_offset(last_lr));

  byte m_vel1 = m_ud1 + offset;
  byte m_vel2 = m_ud2 - offset;

  MotorController.write(m_vel1);
  MotorController.write(m_vel2);
}

void processParentalOverride(){
  Serial.println("ParentalOverride State!");
}


uint16_t state = 0;
byte incomingByte;

byte packet_id;
uint32_t packet_checksum;
byte packet_data_length;
byte sensor_accel_x;
byte sensor_accel_y;

void bluetoothStateMachine()
{
  if (Bluetooth.available() > 0)
  {
    incomingByte = Bluetooth.read();
    // Serial.println(incomingByte);

    switch (state)
    {
    case 0: // 1st Byte (0x01)
      if (incomingByte == p_startByte)
      {
        state++;
      }
      break;

    case 1:
      if (incomingByte == p_stopMotorsId ||
          incomingByte == p_sensorDataId ||
          incomingByte == p_parentalOverrideId)
      {
        packet_id = incomingByte;
        state++;
      }
      else
      {
        state = 0;
      }
      break;

    case 2:
      packet_checksum = (long)incomingByte << 24;
      state++;
      break;

    case 3:
      packet_checksum += (long)incomingByte << 16;
      state++;
      break;

    case 4:
      packet_checksum += (long)incomingByte << 8;
      state++;
      break;

    case 5:
      packet_checksum += (long)incomingByte << 0;
      state++;
      break;

    case 6:
      if (incomingByte == 1 || incomingByte == 0)
      {
        main_acked = incomingByte;
        // Serial.println(main_acked);
        state++;
      }
      else
      {
        state = 0;
      }
      break;

    case 7:
      packet_data_length = incomingByte;
      state++;
      break;

    case 8:
      sensor_accel_x = incomingByte;
      state++;
      break;

    case 9:
      sensor_accel_y = incomingByte;
      state++;
      break;

    case 16:     // 17th Byte (End of packet: 0x04)
      state = 0; // Go back to state 0 to look for a new packet

      // handle state based on ID
      switch (packet_id)
      {
      case p_stopMotorsId:
        processStopMotors();
        break;
      case p_sensorDataId:
        byte data[2] = {sensor_accel_x, sensor_accel_y};
        processSensorData(data);
        // processSensorData(inStreamReadBuffer);
        break;
      case p_parentalOverrideId:
        processParentalOverride();
        break;
      } // end of switch

      // reset packet_checksum
      packet_checksum = 0;

      break;
    default: // defalt case is to increase the state
      state++;
      break;
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  // Wait for newly restarted system to stabilize
  // bluetooth needs 500ms to be ready to ender command mode
  delay(500);

  // setup onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // wait for Arduino Serial Monitor to open
  while (!Serial) {};

  // Setup Serial
  Serial.begin(9600);
  Serial.println("Arduino Up!");

  // setup MotorController
  MotorController.begin(9600);

  // Setup bluetooth
  // Bluetooth.begin(bluetoothBaud);
  // Bluetooth.print("$$$");
  
  Bluetooth.begin(115200);
  
  uint16_t delay_ = 1020;
  Bluetooth.write("AT"); // send at command
  delay(delay_);

  Bluetooth.write("AT+NAMEGoBabyGoBT"); // Set BT Name
  delay(delay_);

  Bluetooth.write("AT+BAUD8"); // Set BT Baud
  delay(delay_);

  //Wait for response from bluetooth: 'CMD'
  delay(200);
}

// encoder vars
unsigned int monitor_time = 18; // in milliseconds
unsigned long timing;
long leftMotorEncoderPosition = -999;
long rightMotorEncoderPosition = -999;
void loop()
{
  // put your main code here, to run repeatedly:

  // handle isr: safety measures for stopping car if not acked
  safetyIsrMotors();
  
  // bluetooth state-machine read data/messages
  bluetoothStateMachine();

  // encoders timing
  // encoders timing
  if (millis() > timing + monitor_time)
  {
    timing = millis();

    long leftMotorEncoderNewPos = leftMotorEncoder.read();
    long rightMotorEncoderNewPos = rightMotorEncoder.read();

    if (leftMotorEncoderNewPos != leftMotorEncoderPosition ||
        rightMotorEncoderNewPos != rightMotorEncoderPosition) {

      leftMotorEncoderPosition = leftMotorEncoderNewPos;
      rightMotorEncoderPosition = rightMotorEncoderNewPos;
      
      #if DEBUG
      
      Serial.print(leftMotorEncoderPosition);
      Serial.print(" ");
      Serial.println(rightMotorEncoderPosition);
      
      #endif
    }
  }
}