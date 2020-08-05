#include "ros/ros.h"
#include <stdio.h>
#include "teleop_servo1/TriggerMsg.h"
#include <termios.h>  
#include <unistd.h>  

int getch(void)  
{  
  int ch = '\n';  
  struct termios buf;  
  struct termios save;  
  
   tcgetattr(0, &save);  
   buf = save;  
   buf.c_lflag &= ~(ICANON|ECHO);  
   buf.c_cc[VMIN] = 1;  
   buf.c_cc[VTIME] = 0;  
   tcsetattr(0, TCSAFLUSH, &buf);  
   ch = getchar();  
   tcsetattr(0, TCSAFLUSH, &save);  
   return ch;  
}  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger_pub");
  ros::NodeHandle nh1;
  ros::Publisher signal_pub = nh1.advertise<teleop_servo1::TriggerMsg>("starting_trigger",10);
  ros::Rate loop_rate(10);
  teleop_servo1::TriggerMsg msg;
  
  int ch;

  while(ros::ok())
  {
    ros::spinOnce();
    
    for(; !(ch=='\n');){  
  
        ch = getch();  
        printf("%d \n", ch);
        break;  
    }
    switch(ch)
      {			
	case 122:		// z
	  msg.trigger = -1;
	  ROS_INFO("k");
	  break;
	case 120:		// x
	  msg.trigger = 1;
	  ROS_INFO("k");
	  break;

       }

    signal_pub.publish(msg);
    printf("send program start trigger = %d\n", msg.trigger);
    }
}
