#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/Joy.h>

float gyro_x;
float gyro_y;
float gyro_z;


float acc_x;
float acc_y;
float acc_z;


void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{   
    gyro_x = imu_msg->angular_velocity.x;
    gyro_y = imu_msg->angular_velocity.y;
    gyro_z = imu_msg->angular_velocity.z;
    
    acc_x = imu_msg->linear_acceleration.x;
    acc_y = imu_msg->linear_acceleration.y;
    acc_z = imu_msg->linear_acceleration.z;
    
}


/*********** RC alias *****************/
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


// Custom mixer data per motor
typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;
    
static uint8_t numberMotor = 4;
static motorMixer_t currentMixer[4];
static int16_t rcCommand[4];  // interval [1000;2000] for THROTTLE and [-500;+500] for

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

struct config{
	int16_t MINTHROTTLE;
	int16_t MAXTHROTTLE;
	int16_t MINCOMMAND;
	int16_t MIDRC;
	int16_t MINCHECK;
	int16_t MAXCHECK;
    uint8_t YAW_DIRECTION;
};
static struct config conf;

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};


void initMixer(){
    conf.MINTHROTTLE = 1020;
    conf.MAXTHROTTLE = 2000;
    conf.MINCOMMAND = 1000;
    conf.MIDRC = 1500;
    conf.MINCHECK = 1000;
    conf.MAXCHECK = 1900;
    conf.YAW_DIRECTION = 1;
    
    // copy motor-based mixers
    for (int i = 0; i < numberMotor; i++) {
        currentMixer[i] = mixerQuadP[i];
    }
}


int16_t axisPID[3];
int16_t motor[4];

void mixTable(void)
{
    int maxMotor;
    int i = 0;

    if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    
     // motors for non-servo mixes
    if (numberMotor > 1) {
        for (i = 0; i < numberMotor; i++) {
            motor[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + -conf.YAW_DIRECTION * axisPID[YAW] * currentMixer[i].yaw;
        }
    }
    
    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++) {
        if (motor[i] > maxMotor) {
            maxMotor = motor[i];
        }
    }
    for (i = 0; i < numberMotor; i++) {
        if (maxMotor > conf.MAXTHROTTLE) {    // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motor[i] -= maxMotor - conf.MAXTHROTTLE;
        }

        motor[i] = constrain(motor[i], conf.MINTHROTTLE, conf.MAXTHROTTLE);
    }
    

}


void joyCallback(const sensor_msgs::JoyConstPtr& msg) {
    sensor_msgs::Joy current_joy_;
    current_joy_ = *msg;

    rcCommand[THROTTLE] = (-msg->axes[0] + 1) * 500 + 1000;

    rcCommand[ROLL] = (-msg->axes[1] * 500)/10;
    rcCommand[PITCH] = (-msg->axes[2] * 500) /1;
    rcCommand[YAW] = (-msg->axes[3] * 500) /1;

    
    ROS_INFO("RC [%i,%i,%i,%i] ", rcCommand[THROTTLE],rcCommand[ROLL],rcCommand[PITCH],rcCommand[YAW]);
}
float scale_angular_velocities(int motor){
    return (motor - 1000);
    
    /*float temp = (float)(motor - 1000) / 1000;
    temp = temp * 2 - 1;
    
    ROS_INFO("temp %f", temp);
    return 600 + (temp * 600) ;*/
}
    std::string actuators_pub_topic = "/hummingbird/command/motor_speed";
 
// rosrun baseflight_controller baseflight_controller_node _imu_topic:=/iris/imu
int main(int argc, char** argv) {
    ros::init(argc, argv, "baseflight_controller_node");
    ros::Subscriber imu_sub_;
    ros::Publisher actuators_pub_;
    ros::Subscriber joy_sub_;
    ros::NodeHandle nh_;
    std::string imu_sub_topic = "/hummingbird/imu";
    std::string actuators_pub_topic = "/hummingbird/command/motor_speed";
 

    ROS_INFO("Topic %s , %s ", imu_sub_topic.c_str(),actuators_pub_topic.c_str());

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(imu_sub_topic,10,imuCallback);
    joy_sub_ = nh_.subscribe("joy", 10, &joyCallback);
    actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);
    
    axisPID[ROLL]=0;
    axisPID[PITCH]=0;
    axisPID[YAW]=0;
    initMixer();
    
    ros::Rate r(100); // 1000 hz
    while (ros::ok())
    {
        //ROS_INFO("Gyro [%f,%f,%f] Acc [%f,%f,%f]",gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z);
        
        
        mixTable();
        //ROS_INFO("Motor [%i,%i,%i,%i] ",motor[0],motor[1],motor[2],motor[3]);

         axisPID[ROLL]=rcCommand[ROLL];
    axisPID[PITCH]=rcCommand[PITCH];
    axisPID[YAW]=rcCommand[YAW];
    
        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
        actuator_msg->angular_velocities.clear();
        actuator_msg->angular_velocities.push_back(scale_angular_velocities(motor[3]));
        actuator_msg->angular_velocities.push_back(scale_angular_velocities(motor[2]));
        actuator_msg->angular_velocities.push_back(scale_angular_velocities(motor[0]));
        actuator_msg->angular_velocities.push_back(scale_angular_velocities(motor[1]));/*scale_angular_velocities(motor[1]));
        actuator_msg->angular_velocities.push_back(scale_angular_velocities(motor[2]));
        actuator_msg->angular_velocities.push_back(scale_angular_velocities(motor[3]));
        actuator_msg->angular_velocities.push_back(scale_angular_velocities(motor[0]));*/
        ros::Time current_time = ros::Time::now();
        actuator_msg->header.stamp.sec = current_time.sec;
        actuator_msg->header.stamp.nsec = current_time.nsec;

        actuators_pub_.publish(actuator_msg);
        
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}


