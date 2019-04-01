#include <VL6180X.h>
#include <Wire.h>
//#define ROS/
#ifdef ROS
  #define USE_USBCON
  #include <ros.h>
  #include <std_msgs/UInt16MultiArray.h>

  std_msgs::UInt16MultiArray measure_msg;
  ros::Publisher p("distance_sensor_topic_left", &measure_msg);
  ros::NodeHandle nh;
  #define BAUD 57600
#endif

#define NS 8
// address we will assign if dual sensor is present
uint8_t initial_address = 0x30;
//uint8_t shutdown_pins[] = {14, 32, 15, 33, 27, 12, A0, A1, A5, 21};
uint8_t shutdown_pins[] = {A0, A1, A2, A3, A4, A5, 9, 10, 33, 27, 13, 12};

uint16_t measure[NS];
uint16_t temp_measure;

VL6180X lox[NS];

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
    pinMode(shutdown_pins[i], INPUT);
    delay(10);
    lox[i].setAddress(cur_address++);
    lox[i].init();
  }

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
  Wire.setClock(100000L);

  setupPins();
  setupAddresses();
  setupRosComm();

}

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
