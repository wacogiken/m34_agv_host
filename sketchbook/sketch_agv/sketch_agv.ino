#include <ros.h>
#include <ArduinoHardware.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#define BUF_NUM 1

ros::NodeHandle nh;

std_msgs::Int16MultiArray arduino;
ros::Publisher arduino_pub("arduino", &arduino);
int old;

char label[]="arduino";
std_msgs::MultiArrayDimension layout[1];
int16_t data[BUF_NUM];

#define PORT_NUM 13
int port_no[PORT_NUM] = {0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

void setup()
{
  int i;
  
  for (i=0; i<PORT_NUM; i++) {
    pinMode(port_no[i], INPUT);
  }
  
  nh.initNode();

  arduino.layout.dim = layout;

  arduino.layout.dim[0].label = label;
  arduino.layout.dim[0].size = BUF_NUM;
  arduino.layout.dim[0].stride = sizeof(int16_t)*BUF_NUM;
  arduino.layout.dim_length = 1;
  arduino.layout.data_offset = 0;

  arduino.data = data;
  arduino.data_length = BUF_NUM;

  nh.advertise(arduino_pub);
}

void loop()
{
  int i, j, n;

  for (i=n=0, j=1; i<PORT_NUM; i++, j<<=1) {
      if (digitalRead(port_no[i])==0) {
          n+=j;
      }
  }
  if (n==old) {
    arduino.data[0] = n;
  }
  old = n;

  arduino_pub.publish(&arduino);
  nh.spinOnce();
  delay(50);
}

