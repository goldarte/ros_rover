
#include "mavlink.h"

// Define constants for motors direction
#define L 1
#define R 2

// Define motor speed limits
#define SPEED_MIN -200L
#define SPEED_MAX 200L

// Define limits for servo_raw PWM signals
#define SERVO_RAW_MIN 1100L
#define SERVO_RAW_MAX 1900L

// Debug flags
//#define SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
//#define SERIAL_SERVO_DEBUGGING
//#define SERIAL_MOTOR_DEBUGGING
//#define SERIAL_MSG_DEBUGGING

// Define pins for motor drivers
int DIR2 = 43;  // Ports that define motor spinning direction
int DIR1 = 47;
int PWM1 = 46;  // PWM pins that control motor speed
int PWM2 = 45;
int EN1 = 42;   // Pins that enable drivers
int EN2 = 48;

long SPEED_RANGE = SPEED_MAX-SPEED_MIN+1;
long SERVO_RANGE = SERVO_RAW_MAX-SERVO_RAW_MIN+1;
long SERVO_RAW_AVG = (SERVO_RAW_MIN+SERVO_RAW_MAX)/2;


// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_count = num_hbs;

// Speed variables for motors
int left_speed = 0;
int right_speed = 0;

void setup() {
  // MAVLink interface start
  Serial3.begin(115200);
  Serial.begin(115200);

  // Setup pins for motor drivers
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode (PWM1, OUTPUT);
  pinMode (DIR1, OUTPUT);
  pinMode (PWM2, OUTPUT);
  pinMode (DIR2, OUTPUT);
  digitalWrite(EN1, 1);       // Enable drivers
  digitalWrite(EN2, 1);

#ifdef SERIAL_DEBUGGING
  // [DEB] serial port start
  Serial.println("MAVLink starting.");
#endif
}

// MAVLink
  /* The default UART header for your MCU */ 
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_GROUND_ROVER;   ///< This system is arover
 
  // Define the system type, in this case a rover -> on-board controller
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

void loop() {
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    previousMillisMAVLink = currentMillisMAVLink;
      // Pack the message
    //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
       mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
   
    // Copy the message to the send buffer
       uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef SERIAL_DEBUGGING
    //Serial.println("Send Heartbeat");
#endif
       Serial3.write(buf, len);

    //Mav_Request_Data();
    num_hbs_count++;
    if(num_hbs_count>=num_hbs) {
      // Request streams from Pixhawk
#ifdef SERIAL_DEBUGGING
      Serial.println("Streams requested!");
#endif
      Mav_Request_Data();
      num_hbs_count=0;
    }

  }

  // Check reception buffer
  comm_receive();

}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RC_CHANNELS};
  const uint16_t MAVRates[maxStreams] = {0x14};

    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
#ifdef SERIAL_DEBUGGING
    //Serial.write(buf,len);
#endif
    Serial3.write(buf, len);
  }
}



void comm_receive() {
 
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial3.available()>0) {
    uint8_t c = Serial3.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
#ifdef SERIAL_MSG_DEBUGGING
            Serial.println("Pixhawk HeartBeat Received");
#endif
          }
          break;

        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:  // #36
          {
            /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_servo_output_raw_decode(const mavlink_message_t* msg, mavlink_servo_output_raw_t* servo_output_raw)
             */
            mavlink_servo_output_raw_t raw_servo;
            mavlink_msg_servo_output_raw_decode(&msg, &raw_servo);
            left_speed = (raw_servo.servo1_raw-SERVO_RAW_AVG)*SPEED_RANGE/SERVO_RANGE;
            right_speed = (raw_servo.servo3_raw-SERVO_RAW_AVG)*SPEED_RANGE/SERVO_RANGE;
            SetSpeed(L, left_speed);
            SetSpeed(R, right_speed);
            //left_speed = map(raw_servo.servo1_raw, SERVO_RAW_MIN, SERVO_RAW_MAX, SPEED_MIN, SPEED_MAX);
            //right_speed = map(raw_servo.servo3_raw, SERVO_RAW_MIN, SERVO_RAW_MAX, SPEED_MIN, SPEED_MAX);
#ifdef SERIAL_SERVO_DEBUGGING
            Serial.print("13: ");
            Serial.print(raw_servo.servo1_raw);
            Serial.print(" ");
            Serial.println(raw_servo.servo3_raw);
#endif

#ifdef SERIAL_MOTOR_DEBUGGING
            Serial.print("LR: ");
            Serial.print(left_speed);
            Serial.print(" ");
            Serial.println(right_speed);
#endif
          }
          break;
        
       default:
#ifdef SERIAL_MSG_DEBUGGING
          Serial.print("Received: ");
          Serial.print("[ID: ");
          Serial.print(msg.msgid);
          Serial.print("], [seq: ");
          Serial.print(msg.seq);
          Serial.println("]");
#endif
          break;
      }
    }
  }
}

void SetSpeed(int Motor, int Speed) {
  int DIR, PWM;
  if (Motor == R) {DIR = DIR1; PWM = PWM1;}
  if (Motor == L) {DIR = DIR2; PWM = PWM2;}
  analogWrite(PWM, abs(Speed));
  if ((Speed < -255)||(Speed > 255)) return;
  else if (Speed < 0) digitalWrite(DIR, 0);
  else if (Speed >= 0) digitalWrite(DIR, 1);
}
