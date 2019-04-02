#include <VL6180X.h> //pololu library for the sensor
#include <Wire.h>    //arduino i2c library 
#define ROS          //use this when communicating wirh ROS
#ifdef ROS
  #define USE_USBCON
  #include <ros.h>
  #include <std_msgs/UInt16MultiArray.h>

  std_msgs::UInt16MultiArray measure_msg;
  ros::Publisher p("distance_sensor_topic_left", &measure_msg);
  ros::NodeHandle nh;
  #define BAUD 57600
#endif

#define NS 8 //number of utilized sensors. maximum 8 per board.

uint8_t initial_address = 0x30; //the initial i2c address

//enable/shutdown pins to initialize the sensors
uint8_t shutdown_pins[] = {A0, A1, A2, A3, A4, A5, 9, 10, 33, 27, 13, 12};

//array that stores the sensor measurements
uint16_t measure[NS];

//array that stores the sensor objects
VL6180X lox[NS];

//resets pins by driving them low, disabling the sensors
void setupPins()
{
  
  // initializing SHUTX pins
  int i = 0;
  for (i = 0; i < NS; i++)
  {
    pinMode(shutdown_pins[i], OUTPUT);
  }

  for (i = 0; i < NS; i++)
  {
    digitalWrite(shutdown_pins[i], LOW);
  }
}



void setupAddresses() {
  // all reset
  int i = 0;
  for (i = 0; i < NS; i++)
  {
    digitalWrite(shutdown_pins[i], LOW);
  }

  delay(10);

  uint8_t cur_address = initial_address;
  
  for (i = 0; i < NS; i++)
  {
    //here we set the pins to read since the sensors EN
    //are pulled-up by default
    //not doing this and setting the pin to high through output
    //caused issues with multiple sensors (probably high power draw)
    pinMode(shutdown_pins[i], INPUT);
    delay(10);

    //set address and init sensor with default values
    lox[i].setAddress(cur_address++);
    lox[i].init();
  }

  //changing parameters to allow greater range, ~40cm, 40ms
  for (i = 0; i < NS; i++)
  {
    lox[i].configureDefault();

    lox[i].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 0x21); //1 code is 1 ms
    lox[i].writeReg(VL6180X::READOUT__AVERAGING_SAMPLE_PERIOD, 0x30); //1.3ms + 1 code * 64.5 us

    lox[i].setScaling(2);
//    lox[i].setTimeout(1);/
  }

  delay(10);

  for (i = 0; i < NS; i++)
  {
    lox[i].startRangeContinuous(40);
  }
}

//setups the ros communication (topic publisher)
void setupRosComm()
{

  #ifdef ROS
  measure_msg.data = &measure[0];
  measure_msg.data_length = NS;
  
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(p);
  #endif
}

void setup()
{

 #ifndef ROS
  Serial.begin(115200);
//   wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }
  #endif
  Wire.begin();
  //sets the i2c frequency, should be adjusted depending 
  //on number of sensors, length of channel (noise-capacitance)  
  Wire.setClock(100000L);

  setupPins();
  setupAddresses();
  setupRosComm();

}

//read sensor measurements, if not ROS prints them in arduino serial monitor
void read_all_sensors() {
  int i = 0;

  for(i = 0; i < NS; i++)
  {

    measure[i] = lox[i].readRangeContinuousMillimeters();
       
  }
  #ifndef ROS

  for(i = 0; i < NS; i++)
  {
    Serial.print(measure[i]);
    Serial.print(" ");
  }

  #endif
}

//main loop, calls the read_all_sensors, and publish to ROS if enabled
void loop()
{  
  delay(20);

  #ifndef ROS
  unsigned long time1 = millis();
  #endif
  read_all_sensors();
  #ifndef ROS
  unsigned long time2 = millis();

  Serial.print(time2-time1);
  Serial.println();
  #endif
  #ifdef ROS
  p.publish( &measure_msg );  
  nh.spinOnce();
  #endif
}
