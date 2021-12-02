#ifndef KOALA_H
#define KOALA_H

#include <string.h>
#include <iostream>
#include <stdio.h>
#include "serial/serial.h"
#include <math.h>

#define SLOW_LOOP_FREQ 1
#define FAST_LOOP_FREQ 20
#define WHEEL_BASE 0.3 //m
#define MAX_SPEED (128*4.5/1000) //max linear speed in m/s
#define MAX_YAW_RATE 1.57 // pi/2 /s

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"


#include "ros/ros.h"


using namespace serial;
using namespace std;
using namespace ros;

int diff_ms(timespec curr, timespec prev);

class koala_bot{

public:
    koala_bot(NodeHandle nh);


    void initialize();
    void stop();
    Serial my_serial_;

    int test_speed_;


private:
    //Configuration Function Prototypes
    bool configureSpeedController(int Kp,int Ki,int Kd);
    bool configurePosition_Controller(int Kp,int Ki,int Kd);
    bool configureSpeedProfile(int left_max_speed,int left_accel,int right_max_speed,int right_accel);

    void slowCallBack(const ros::TimerEvent&);
    void fastCallBack(const ros::TimerEvent&);

    //Set robot state function Prototypes
    bool setDesiredPosition(int left_pos,int right_pos);
    bool setSpeed(int left_motor,int right_motor);
    bool setPWM(int left_pwm, int right_pwm);
    bool setPositionCounter(int left_pos,int right_pos);
    void resetPositionCounter();

    //Get Robot State function Prototypes
    bool readPosition();
    void readSpeed();
    void getStatus();
    void getCurrentPose();
    void getBatteryStatus();

    // Sensor Reading Prototypes
    void readProximitySensors();
    void readLightSensors();
    void read_ADC(int channel);

    // Functions to Configure ROS
    void configureROSComms();
    void getROSParams();

    void velCmdCallBack(const geometry_msgs::Twist &vel_cmd_msg);
    void joyCmdCallBack(const sensor_msgs::Joy &joy_msg);

    // create serial port object
    ros::NodeHandle n_;
    string port_;
    int baud_;

    ros::Publisher status_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher prox_pub_;

    
    ros::Subscriber vel_cmd_sub_;
    ros::Subscriber joy_cmd_sub_;
    ros::Timer timer_slow_;
    ros::Timer timer_fast_;

    float LX_,LY_,LT_,RX_,RY_,RT_,DPX_,DPY_;
    bool BA_,BB_,BX_,BY_;
    int speed_L_,speed_R_;
    bool armed;
    float max_speed;
    float max_yawrate;
    
};

#endif

