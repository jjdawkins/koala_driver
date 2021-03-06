#include "../include/koala.h"


koala_bot::koala_bot(NodeHandle nh){
    n_ = nh;
    configureROSComms();
    getROSParams();
}


void
koala_bot::initialize(){

    my_serial_.setStopbits(serial::stopbits_two);
    my_serial_.setFlowcontrol(serial::flowcontrol_none);
    my_serial_.setBaudrate(baud_);
    my_serial_.setPort(port_);

    cout<<"Attempting to Connect to Koala"<<endl;
    try {
        my_serial_.open();
    }
    catch(std::exception e) {
        std::stringstream output;
        output<<"Failed to open port "<< my_serial_.getPort() << "err: " << e.what() <<endl;
    }

    if(my_serial_.isOpen()){

        cout<<"Koala is Connected on Port: "<<my_serial_.getPort()<<" at Baudrate: "<<my_serial_.getBaudrate()<<endl;
    }
    else{

        ROS_ERROR("Serial Port was Unable to Open, check port settings and Koala mode knob");
    }

    armed = true;

    test_speed_ = 0;
    setPositionCounter(0,0); //zero out position counter

}

void
koala_bot::slowCallBack(const ros::TimerEvent&){

    getStatus();
}
void
koala_bot::fastCallBack(const ros::TimerEvent&){
       // getCurrentPose();
        readProximitySensors();
        //readSpeed();    
}
void
koala_bot::stop(){
    setSpeed(0,0);
}

void
koala_bot::configureROSComms(){

    
    n_.param<float>("max_speed",max_speed,MAX_SPEED);
    n_.param<float>("max_yawrate",max_yawrate,MAX_YAW_RATE);
    
    status_pub_ = n_.advertise<std_msgs::Float32MultiArray>("/status",5);
    prox_pub_ = n_.advertise<std_msgs::Int32MultiArray>("/proximity",5);
    joy_cmd_sub_ = n_.subscribe("/joy",100,&koala_bot::joyCmdCallBack,this);
    vel_cmd_sub_ = n_.subscribe("/cmd_vel",100,&koala_bot::velCmdCallBack,this);
    timer_slow_ = n_.createTimer(ros::Duration(2.0),&koala_bot::slowCallBack,this);
    timer_fast_ = n_.createTimer(ros::Duration(0.2),&koala_bot::fastCallBack,this);    

}
void
koala_bot::getROSParams(){

    if(n_.getParam("port",port_)){
        ROS_INFO("Got Param: %s",port_.c_str());
    }
    else{
        port_ = "/dev/ttyUSB0";
        ROS_WARN("Failed to get Serial Port from Launch File, defaulting to %s \n",port_.c_str());
    }

    if(n_.getParam("baud",baud_)){

        ROS_INFO("Got Param: %d",baud_);

    }
    else{
        baud_ = 38400;
        ROS_WARN("Failed to get Serial Baudrate from Launch File, defaulting to %d \n",baud_);
    }

    my_serial_.setPort(port_.c_str());
    my_serial_.setBaudrate(baud_);
}

bool
koala_bot::configureSpeedController(int Kp,int Ki,int Kd){
    char msg[10];
    sprintf(msg,"A,%d,%d,%d\n",Kp,Ki,Kd);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("a"));
}

bool
koala_bot::configurePosition_Controller(int Kp,int Ki,int Kd){
    char msg[10];
    sprintf(msg,"F,%d,%d,%d\n",Kp,Ki,Kd);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("f"));

}

bool
koala_bot::configureSpeedProfile(int left_max_speed,
                                 int left_accel,
                                 int right_max_speed,
                                 int right_accel){
    char msg[24];
    sprintf(msg,"J,%d,%d,%d,%d\n",left_max_speed,left_accel,right_max_speed,right_accel);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("j"));
}

bool
koala_bot::setDesiredPosition(int left_pos,int right_pos){
    char msg[10];
    sprintf(msg,"C,%d,%d\n",left_pos,right_pos);
    my_serial_.write(msg);

    string ack = my_serial_.readline();
    return(ack.compare("c"));
}

bool
koala_bot::setSpeed(int left_motor,int right_motor){
    char msg[10];

    if(armed){
        if(left_motor>127){
            left_motor = 127;
        }

        if(left_motor<-127){
            left_motor = -127;
        }

        if(right_motor>127){
            right_motor = 127;
        }
        if(right_motor<-127){
            right_motor = -127;
        }
        sprintf(msg,"D,%d,%d\n",left_motor,right_motor);
    }else{
        sprintf(msg,"D,%d,%d\n",0,0);    
    }
    
    my_serial_.write(msg);

    string ack = my_serial_.readline();

    return(ack.compare("d"));
}

bool
koala_bot::setPWM(int left_pwm, int right_pwm){
    char msg[10];
    sprintf(msg,"P,%d,%d\n",left_pwm,right_pwm);
    my_serial_.write(msg);

    string ack = my_serial_.readline();

    return(ack.compare("p"));

}
bool
koala_bot::setPositionCounter(int left_pos,int right_pos){
    char msg[10];
    sprintf(msg,"G,%d,%d\n",left_pos,right_pos);
    my_serial_.write(msg);

    string ack = my_serial_.readline();

    return(ack.compare("g"));
}

void
koala_bot::resetPositionCounter(){
    setPositionCounter(0,0);
}

bool
koala_bot::readPosition(){

    my_serial_.write("H\n");

    string msg = my_serial_.readline();

    printf("%s\n",msg.c_str());
    return 0;
}

void
koala_bot::readSpeed(){

    my_serial_.write("E\n");
    string msg = my_serial_.readline();
    printf("%s\n",msg.c_str());
}

void
koala_bot::getCurrentPose(){

    my_serial_.write("H\n");
    string msg = my_serial_.readline();

    if(msg.size()>0){

        int ind = msg.find_first_of(",");
        int ind2 = msg.find_first_of(",",ind+1);

        string data1 = msg.substr(ind+1,ind2-ind);
        string data2 = msg.substr(ind2+1,msg.size()-ind2);
        int left_pos = atoi(data1.c_str());
        int right_pos = atoi(data2.c_str());

    }

    my_serial_.write("E\n");
    string msg1 = my_serial_.readline();

    if(msg1.size()>0){

        int ind = msg1.find_first_of(",");
        int ind2 = msg1.find_first_of(",",ind+1);

        string ack = msg1.substr(0,ind-1);
        string data1 = msg1.substr(ind+1,ind2-ind);
        string data2 = msg1.substr(ind2+1,msg1.size()-ind2);
        int left_vel = atoi(data1.c_str());
        int right_vel = atoi(data2.c_str());

    }

}

void
koala_bot::getStatus(){

    if(status_pub_.getNumSubscribers()>0){

        int data[6];

        for(int i=0;i<6;i++){
            char write_buf[10];

            sprintf(write_buf,"M,%u\n",i);
            my_serial_.write(write_buf);

            string msg = my_serial_.readline();

            int ind1 = msg.find_first_of(",");
            string dat = msg.substr(ind1+1,msg.size()-ind1);

            char local;
            msg.copy(&local,msg.size(),0);

            data[i]=atoi(dat.c_str());

        }

        std_msgs::Float32MultiArray msg;

      //  status_msg_.header.stamp.sec = current_time_.tv_sec;
      //  status_msg_.header.stamp.nsec = current_time_.tv_nsec;
        float battery_voltage = 20.0*data[0]/1000.0; //20mV per integer converted to volts
        float current_consumption = 8.0*data[1]; // Converted to mA
        float ambient_temp = ((9/5)*(0.1*data[2]))+32; // temperature in degrees F
        float left_motor_current = 4*data[3];
        float right_motor_current = 4*data[4];
        float battery_temp = ((9/5)*(0.1*data[5]))+32;

        msg.data.push_back(battery_voltage);
        msg.data.push_back(battery_temp);
        msg.data.push_back(current_consumption);
        msg.data.push_back(left_motor_current);
        msg.data.push_back(right_motor_current);
        msg.data.push_back(ambient_temp);

        status_pub_.publish(msg);
    }

}
void
koala_bot::getBatteryStatus(){}

// Sensor Reading Prototypes
void koala_bot::readProximitySensors(){
        my_serial_.write("N\n");
    string my_str = my_serial_.readline();

    if(my_str.size()>0){

        vector<string> result;
        stringstream s_stream(my_str); //create string stream from the string
        while(s_stream.good()) {
            string substr;
            getline(s_stream, substr, ','); //get first string delimited by comma
            result.push_back(substr);
        }
    
        std_msgs::Int32MultiArray prox_msg;
        for(int i = 1; i<result.size(); i++) {    //print all splitted strings
          //  cout << result.at(i) << endl;
            prox_msg.data.push_back(atoi(result.at(i).c_str()));
        }    
        
        if(result.size()>15){
            prox_pub_.publish(prox_msg);
        }

    }    
    
}

void koala_bot::readLightSensors(){}
void koala_bot::read_ADC(int channel){}

void koala_bot::velCmdCallBack(const geometry_msgs::Twist &msg){

    float velocity;
    float yaw_rate;

    //Saturate Commanded Speed
    if(msg.linear.x > max_speed){
        velocity = max_speed;
    }else if(msg.linear.x < -max_speed){
        velocity = -max_speed;
    }else{
        velocity = msg.linear.x;
    }

    //Saturate Commanded Yaw Rate
    if(msg.angular.z > max_yawrate){
        yaw_rate = max_yawrate;
    }else if(msg.angular.z < -max_yawrate){
        yaw_rate = -max_yawrate;
    }else{
        yaw_rate = msg.angular.z;
    }

    int speed_L = (-(0.5*yaw_rate*WHEEL_BASE) + velocity)*(1000/4.5); //Calculate desired wheel velocities and convert to
    int speed_R = ((0.5*yaw_rate*WHEEL_BASE) + velocity)*(1000/4.5);

    printf("Left Cmd: %d ; Right Cmd: %d\n",speed_L,speed_R);

    setSpeed(speed_L,speed_R);
}

void koala_bot::joyCmdCallBack(const sensor_msgs::Joy &joy_msg){


    LX_ = joy_msg.axes[0];
    LY_ = joy_msg.axes[1];
    LT_ = -joy_msg.axes[2];
    RX_ = -joy_msg.axes[3];
    RY_ = joy_msg.axes[4];
    RT_ = joy_msg.axes[5];
    DPX_ = joy_msg.axes[6];
    DPY_ = joy_msg.axes[7];

    BA_ = joy_msg.buttons[0];

    if(BA_){
        printf("toggle\n");
        armed= !armed;
    }

}


