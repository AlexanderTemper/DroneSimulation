#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <rotors_control/common.h>
#include <std_msgs/Float32MultiArray.h>

sensor_msgs::Joy current_joy_;
mav_msgs::RollPitchYawrateThrust control_msg_;
std_msgs::Float32MultiArray debug_msg_;

int attitude[3];
int rcCommand[4];  // interval [1000;2000] for THROTTLE and [-500;+500] for

enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

int headFreeModeHold = 0;
ros::Time last_cycle_time;

// altHold variables
int current_distance = 0;
int integral;
int last_error;
float p = 0.4;//0.1
float i = 0.001;
float d = 50;
int hoverat = 500;
int altHoldThrottle = 0;
// Arm Controller
bool altMode = false;


int front = 0;
// Position
float x = 0;
float y = 0;

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
void resetAltitudePid(){
    integral = 0;
    last_error = 0;
}

int holdYController(int cycleTime)
{
    static int l_error = 0;
    static int integral_e = 0;
    static float derivative1 = 0;
    static float derivative2 = 0;
    
    float pterm = 0.08;
    float dterm = 20;
    int error = 0 - y;
    //integral = integral + error;
    float derivative = ((error - l_error)/(float)cycleTime) * 4000;
    float derivativeSum = derivative1 + derivative2 + derivative;
    derivative2 = derivative1;
    derivative1 = derivative;
    
    //ROS_INFO("derivative %i", derivative);
    int ki=0, kp =0, kd = 0;
    kp = -constrain(pterm * error, -250, +250);
    //int ki = constrain(i * integral, -1000, +1000);
    float derivativeFiltered = derivativeSum/3;
    kd = -constrain(dterm * derivativeFiltered, -500, +500);
    
    //ROS_INFO("error %i %i [%i,%i,%i]", error,derivative,kp,ki,kd);
    /*debug_msg_.data.clear();
    debug_msg_.data.push_back(kp);
    debug_msg_.data.push_back(kd);*/
    l_error = error;
    
    return kp + ki + kd;
}
   
int holdXController(int cycleTime)
{
    static int l_error = 0;
    static int integral_e = 0;
    static float derivative1 = 0;
    static float derivative2 = 0;
    
    float pterm = 0.08;
    float dterm = 20;
    int error = 0 - x;
    //integral = integral + error;
    float derivative = ((error - l_error)/(float)cycleTime) * 4000;
    float derivativeSum = derivative1 + derivative2 + derivative;
    derivative2 = derivative1;
    derivative1 = derivative;
    
    //ROS_INFO("derivative %i", derivative);
    int ki=0, kp =0, kd = 0;
    kp = constrain(pterm * error, -250, +250);
    //int ki = constrain(i * integral, -1000, +1000);
    float derivativeFiltered = derivativeSum/3;
    kd = constrain(dterm * derivativeFiltered, -500, +500);
    
    //ROS_INFO("error %i : %i %i",error,kp,kd);
    /*debug_msg_.data.clear();
    debug_msg_.data.push_back(kp);
    debug_msg_.data.push_back(kd);*/
    l_error = error;
    
    return kp + ki + kd;
}
 
int collController(int cycleTime)
{
    static int l_error = 0;
    static int integral_e = 0;
    static float derivative1 = 0;
    static float derivative2 = 0;
    
    float pterm = 0.08;
    float dterm = 20;
    
    ROS_INFO("Front %i ",front );
    if(front < 0){ // No Sensor Value
        return 0;
    }
    
    int error = 1000 - front;
    
    if(error < 0){ // we are not in danger zone
        return 0;
    }
    //integral = integral + error;
    float derivative = ((error - l_error)/(float)cycleTime) * 4000;
    float derivativeSum = derivative1 + derivative2 + derivative;
    derivative2 = derivative1;
    derivative1 = derivative;
    
    //ROS_INFO("derivative %i", derivative);
    int ki=0, kp =0, kd = 0;
    kp = - constrain(pterm * error, -250, +250);
    //int ki = constrain(i * integral, -1000, +1000);
    float derivativeFiltered = derivativeSum/3;
    kd = - constrain(dterm * derivativeFiltered, -500, +500);
    
    if(kp>0){
        kp=0;
    }
    if(kd>0){
        kd=0;
    }
    ROS_INFO("error %i : %i %i",error,kp,kd);
    /*debug_msg_.data.clear();
    debug_msg_.data.push_back(kp);
    debug_msg_.data.push_back(kd);*/
    l_error = error;
    
    return kp + ki + kd;
}

int getAltitudeThrottle(int distance, int target_distance,int cycleTime)
{ 
    if(!altMode){ //if not armed do nothing
        resetAltitudePid();
        return 0;
    }
    int error = target_distance - distance;
    integral = integral + error;
    int derivative = ((error - last_error)/(float)cycleTime) * 4000;
    //ROS_INFO("derivative %i", derivative);
    int kp = constrain(p * error, -400, +400);
    int ki = constrain(i * integral, -1000, +1000);
    int kd = constrain(d * derivative, -500, +500);
    //ROS_INFO("error %i %i [%i,%i,%i]", error,integral,kp,ki,kd);
    last_error = error;
    
    return kp + ki + kd;
}

void setCurrentHeading() {
    //int heading = (int) (attitude[YAW] / 10.0f);
    headFreeModeHold = 0;
}


void calcHeadFree(int *roll, int *pitch){
    int heading = (int) (attitude[YAW] / 10.0f);
    //ROS_INFO("bevore %i [%i,%i,%i] ",heading - headFreeModeHold, rcCommand[THROTTLE],*roll,*pitch);
    float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);

    int rcCommand_PITCH = *pitch * cosDiff + *roll * sinDiff;
    *roll = *roll * cosDiff - *pitch * sinDiff;
    *pitch = rcCommand_PITCH;
}


void calcPushback(int *roll, int *pitch,int cycleTime){
    int heading = (int) (attitude[YAW] / 10.0f);
    
    int force = collController(cycleTime);
    //ROS_INFO("bevore %i [%i,%i,%i] ",heading - headFreeModeHold, rcCommand[THROTTLE],*roll,*pitch);
    float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);

    *roll = force * sinDiff;
    *pitch = force * cosDiff;
    
    //ROS_INFO("bevore roll: %f, pitch: %f ",cosDiff - sinDiff, cosDiff + sinDiff);
}


// ***** Callbacks *****
rotors_control::EigenOdometry odometry;
void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{   
    rotors_control::eigenOdometryFromMsg(odometry_msg, &odometry);
    Eigen::Vector3d euler_angles;
    mav_msgs::getEulerAnglesFromQuaternion(odometry.orientation,&euler_angles);
    
    attitude[ROLL] = euler_angles.x()*(1800.0f/M_PI);
    attitude[PITCH] = euler_angles.y()*(1800.0f/M_PI);
    attitude[YAW] = -euler_angles.z()*(1800.0f/M_PI);
    
    if (attitude[YAW] < 0) {
        attitude[YAW] += 3600;
    }
}

void tofCallback(const geometry_msgs::PosePtr& msg)
{   
   current_distance = msg->position.z * 1000;
   x = msg->position.x * 1000;
   y = msg->position.y * 1000;
}

void tofGroundCallback(const sensor_msgs::LaserScanPtr& msg)
{   
    //current_distance = msg->ranges[0] * 1000;
    //ROS_INFO("Z %i ", current_distance);
}
void tofFrontdCallback(const sensor_msgs::LaserScanPtr& msg)
{   
    int range = (int)(msg->ranges[0] * 1000);
    //current_distance = msg->ranges[0] * 1000;
    if(range > 0){
        front = range;
    } else {
        front = -1;
    }
}

void joyCallback(const sensor_msgs::JoyConstPtr& msg) 
{
    // RC Controll
    /*rcCommand[THROTTLE]= (-msg->axes[0] + 1) * 500 + 1000;
    rcCommand[ROLL] = (-msg->axes[1] * 500);
    rcCommand[PITCH] = (-msg->axes[2] * 500);
    rcCommand[YAW] = (-msg->axes[3] * 500);*/

    // PS3 Controll
    static int keyState = 0;
    
    switch(keyState){
        case 0: 
            if(msg->buttons[0] == 1){ 
                keyState = 1;
                altMode = !altMode;
            }
            break;
        case 1:
            if(msg->buttons[0] == 0){ 
                keyState = 0;
            }
            break;   
        default:
            keyState = 0;
    }
    rcCommand[THROTTLE]= altMode ? 1050 : 1000;
    rcCommand[ROLL] = (-msg->axes[3] * 250);
    rcCommand[PITCH] = (msg->axes[4] * 250);
    rcCommand[YAW] = (-msg->axes[0] * 500);
    
    if(rcCommand[THROTTLE] < 1050){ // Reset Things
        setCurrentHeading();
        resetAltitudePid();
    }
}

//***** Main *****
int main(int argc, char** argv) {
    ros::init(argc, argv, "gateway_node");
    ros::NodeHandle nh_;
    ros::Publisher ctrl_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber tof_sub_;
    ros::Subscriber tof_ground_sub_;
    ros::Subscriber tof_front_sub_;
    
    std::string odometry_topic = "/hummingbird/odometry_sensor1/odometry";
    std::string tof_topic = "/hummingbird/ground_truth/pose";
    std::string tof_ground_topic = "/hummingbird/tof_ground_sensor";
    std::string tof_front_topic = "/hummingbird/tof_front_sensor";

    
    last_cycle_time = ros::Time::now();
    rcCommand[THROTTLE] = 1000;
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;
    resetAltitudePid();
    
    control_msg_.roll = 0;
    control_msg_.pitch = 0;
    control_msg_.yaw_rate = 0;
    control_msg_.thrust.x = 0;
    control_msg_.thrust.y = 0;
    control_msg_.thrust.z = 0;
    
    
    joy_sub_ = nh_.subscribe("joy", 1, &joyCallback);
    odometry_sub_ = nh_.subscribe(odometry_topic,1,odometryCallback);
    tof_ground_sub_ = nh_.subscribe(tof_ground_topic,1,tofGroundCallback);
    tof_front_sub_ = nh_.subscribe(tof_front_topic,1,tofFrontdCallback);
    tof_sub_ = nh_.subscribe(tof_topic,1,tofCallback);
    
    ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);
    
    ros::Publisher debug_pub_ = nh_.advertise<std_msgs::Float32MultiArray> ("debug", 1);

    ros::Rate r(250);
    while (ros::ok())
    {
        int cycleTime = (ros::Time::now().nsec - last_cycle_time.nsec) / 1000;
        if(cycleTime == 0 || cycleTime > 10000)
        {
            ROS_INFO("cycleTime out of scope %i ", cycleTime);
        } else {
            int thrust_alt = getAltitudeThrottle(current_distance, hoverat,cycleTime);
            altHoldThrottle = constrain(rcCommand[THROTTLE] + thrust_alt, 1000, 2000);
            //ROS_INFO("Thr %i %i %i %i ",thrust_alt, rcCommand[THROTTLE],rcCommand[THROTTLE] + thrust_alt, altHoldThrottle);
            
            // Manipulate the roll
            //rcCommand[ROLL]= 
            int roll = rcCommand[ROLL];
            int pitch = rcCommand[PITCH];
            
            //ROS_INFO("after[%i,%i] ",roll,pitch);
            // Replace Roll 
            //roll = constrain(holdYController(cycleTime),-500,500);
            //pitch = constrain(holdXController(cycleTime),-500,500);
            int pushRoll,pushPitch;
            calcPushback(&pushRoll,&pushPitch,cycleTime);
            ROS_INFO("push[%i,%i] ",pushRoll,pushPitch);
            
            roll = constrain(roll + pushRoll,-500,500);
            pitch = constrain(pitch + pushPitch,-500,500);
            
            calcHeadFree(&roll,&pitch);
            
            //debug_pub_.publish(debug_msg_);
            
            control_msg_.roll = roll;
            control_msg_.pitch = pitch;
            control_msg_.yaw_rate = rcCommand[YAW];
            control_msg_.thrust.z = altHoldThrottle;
            ros::Time update_time = ros::Time::now();
            control_msg_.header.stamp = update_time;
            control_msg_.header.frame_id = "rotors_joy_frame";
            ctrl_pub_.publish(control_msg_);
        }
        last_cycle_time = ros::Time::now();
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}


