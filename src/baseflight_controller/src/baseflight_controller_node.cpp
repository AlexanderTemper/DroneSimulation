#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <rotors_control/common.h>
#include <baseflight_controller/pidStatus.h>

int gyro[3];
int attitude[3];

/** Mixer **/
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
    

enum pid {
	PIDROLL,
	PIDPITCH,
	PIDYAW,
	PIDALT,
	PIDPOS,
	PIDPOSR,
	PIDNAVR,
	PIDLEVEL,
	PIDMAG,
	PIDVEL,     // not used currently
	PIDITEMS
};
struct config{
	int MINTHROTTLE;
	int MAXTHROTTLE;
	int MINCOMMAND;
	int MIDRC;
	int MINCHECK;
	int MAXCHECK;
    int YAW_DIRECTION;
    int yawRate;
    int rollPitchRate[2];
    uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
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



static uint8_t numberMotor = 4;
static motorMixer_t currentMixer[4];
static int16_t rcCommand[4];  // interval [1000;2000] for THROTTLE and [-500;+500] for
int16_t axisPID[3];

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}



void initMixer(){
    conf.MINTHROTTLE = 1020;
    conf.MAXTHROTTLE = 2000;
    conf.MINCOMMAND = 1000;
    conf.MIDRC = 1500;
    conf.MINCHECK = 1000;
    conf.MAXCHECK = 1900;
    conf.YAW_DIRECTION = -1;
    
    // copy motor-based mixers
    for (int i = 0; i < numberMotor; i++) {
        currentMixer[i] = mixerQuadP[i];
    }
}

void mixTable(int *motors)
{
    int maxMotor;
    int i = 0;

    //ROS_INFO("PID %i , %i , %i", axisPID[ROLL],axisPID[PITCH],axisPID[YAW]);
    
    if (numberMotor > 3) {
        // prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    
     // motors for non-servo mixes
    if (numberMotor > 1) {
        for (i = 0; i < numberMotor; i++) {
            motors[i] = rcCommand[THROTTLE] * currentMixer[i].throttle + axisPID[PITCH] * currentMixer[i].pitch + axisPID[ROLL] * currentMixer[i].roll + -conf.YAW_DIRECTION * axisPID[YAW] * currentMixer[i].yaw;
        }
    }
    
    maxMotor = motors[0];
    for (i = 1; i < numberMotor; i++) {
        if (motors[i] > maxMotor) {
            maxMotor = motors[i];
        }
    }
    for (i = 0; i < numberMotor; i++) {
        if (maxMotor > conf.MAXTHROTTLE) {    // this is a way to still have good gyro corrections if at least one motor reaches its max.
            motors[i] -= maxMotor - conf.MAXTHROTTLE;
        }

        motors[i] = constrain(motors[i], conf.MINTHROTTLE, conf.MAXTHROTTLE);
    }
}


float scale_angular_velocities(int motor)
{
    /// max_rot_velocity for model = 838 but this is to big
    int max_rot_velocity = 838 - 150;
    
    float temp = (float)(motor - 1000) / 1000; // Scale to 0-1
    return temp * max_rot_velocity + 150;
}

void writeMotor(ros::Publisher actuators_pub_, int *motors)
{
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    
    actuator_msg->angular_velocities.clear();
    actuator_msg->angular_velocities.push_back(scale_angular_velocities(motors[3]));
    actuator_msg->angular_velocities.push_back(scale_angular_velocities(motors[2]));
    actuator_msg->angular_velocities.push_back(scale_angular_velocities(motors[0]));
    actuator_msg->angular_velocities.push_back(scale_angular_velocities(motors[1]));
    
    ros::Time current_time = ros::Time::now();
    actuator_msg->header.stamp.sec = current_time.sec;
    actuator_msg->header.stamp.nsec = current_time.nsec;

    actuators_pub_.publish(actuator_msg);
}


static int errorGyroI[3] = {0,0,0};
static int errorAngleI[2] = {0,0};
ros::Time last_cycle_time;


// Level Pid Controller
static void levelPid(int32_t cycleTime,int16_t command[4],baseflight_controller::pidStatusPtr pid_msg)
{
    int errorAngle = 0;
    int axis;
    int angle[2] = {0,0};
    int32_t delta = 0, deltaSum = 0;
    static int32_t delta1[3] = {0,0,0} ,delta2[3] = {0,0,0};
    int32_t PTerm = 0, ITerm = 0, DTerm = 0;
    static int32_t lastError[3] = {0,0,0};
    int32_t AngleRateTmp = 0, RateError = 0;
    
    
    angle[ROLL] = attitude[ROLL];
    angle[PITCH] = attitude[PITCH];
    
    pid_msg->att.x = attitude[ROLL];
    pid_msg->att.y = attitude[PITCH];
    pid_msg->att.z = attitude[YAW];
    
    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        if (axis != 2) {
            errorAngle = command[axis] - angle[axis];
            // P-Term
            PTerm = (errorAngle * conf.P8[PIDALT]) >> 8;
            command[axis] = PTerm;
        } /*else { //yaw for testin
            int angelz = attitude[YAW];
            if(angelz > 1800){
                angelz = angelz - 3600;
            }
            errorAngle = command[axis] - angelz;
            command[axis] = errorAngle * conf.P8[PIDALT] >> 8;
        }*/
    }

    pid_msg->pid.x = command[ROLL];
    pid_msg->pid.y = command[PITCH];

    //ROS_INFO("%i %i %i",axisPID[ROLL],axisPID[PITCH],axisPID[YAW]);
}


static void gyroPid(int32_t cycleTime,int16_t command[4],baseflight_controller::pidStatusPtr pid_msg)
{
    int axis;
    int32_t delta = 0, deltaSum = 0;
    static int32_t delta1[3] = {0,0,0}, delta2[3] = {0,0,0};
    int32_t PTerm = 0, ITerm = 0, DTerm = 0;
    static int32_t lastError[3] = {0,0,0};
    int32_t AngleRateTmp = 0, RateError = 0;

    if(cycleTime == 0 || cycleTime > 10000)
    {
        ROS_INFO("cycleTime out of scope %i ", cycleTime);
        last_cycle_time = ros::Time::now();
        return;
    }
    
    int gyroData[3];
    gyroData[0] = gyro[0];
    gyroData[1] = gyro[1];
    gyroData[2] = gyro[2];

    pid_msg->gyro.x = gyroData[0];
    pid_msg->gyro.y = gyroData[1];
    pid_msg->gyro.z = gyroData[2];
    
    pid_msg->AngleRateTmp.clear();
    pid_msg->RateError.clear();
    pid_msg->PTerm.clear();

    if(command[THROTTLE]< 1050){
        errorGyroI[0] = 0;
        errorGyroI[1] = 0;
        errorGyroI[2] = 0;
    }
            
    // ----------PID controller----------

    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        if (axis == 2) {
            AngleRateTmp = command[YAW] * conf.yawRate;
        } else {
            AngleRateTmp = command[axis];
        }
        pid_msg->AngleRateTmp.push_back(AngleRateTmp);
        
        // --------low-level gyro-based PID. ----------
        RateError = AngleRateTmp - gyroData[axis];
        pid_msg->RateError.push_back(RateError);

        // -----calculate P component
        PTerm = (RateError * conf.P8[axis]) >> 7;  // p/128
        pid_msg->PTerm.push_back(PTerm);
        
        // -----calculate I component
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * (int32_t) cycleTime) >> 11) * conf.I8[axis]; // T/2048 for normalisatzion
        errorGyroI[axis] = constrain(errorGyroI[axis], -2097152, +2097152); // Limit windup  2^22 values
        ITerm = errorGyroI[axis] >> 13; // gi/8192 to bring it back to normal values  => +/- 256
        pid_msg->ITerm.push_back(ITerm);
        
        //-----calculate D-term
        delta = RateError - lastError[axis];
        lastError[axis] = RateError;
        
        delta = (delta * ((uint16_t) 0xFFFF / (cycleTime >> 4))) >> 6; // d/dt * 2^14   scaling faktor => ~16384 at 4000deg change
        // add moving average here to reduce noise
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;
        DTerm = (deltaSum * conf.D8[axis]) >> 8;
        //pid_msg->DTerm.push_back(DTerm);
        // -----calculate total PID outputc
        axisPID[axis] = PTerm + ITerm + DTerm;
    }

    //pid_msg->pid.x = axisPID[ROLL];
    //pid_msg->pid.y = axisPID[PITCH];
    pid_msg->pid.z = axisPID[YAW];

    //ROS_INFO("%i %i %i",axisPID[ROLL],axisPID[PITCH],axisPID[YAW]);
}

void gatewayCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& msg) {
  
    rcCommand[THROTTLE] = msg->thrust.z;
    rcCommand[ROLL] = msg->roll;
    rcCommand[PITCH] = msg->pitch;
    rcCommand[YAW] = msg->yaw_rate;
    //ROS_INFO("RC-COMMAND [%i,%i,%i,%i] ",rcCommand[ROLL],rcCommand[PITCH],rcCommand[YAW],rcCommand[THROTTLE]);
}

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
    
    gyro[0]=odometry.angular_velocity.x()*(180.0f/M_PI);
    gyro[1]=odometry.angular_velocity.y()*(180.0f/M_PI);
    gyro[2]=-odometry.angular_velocity.z()*(180.0f/M_PI);
}

 
// rosrun baseflight_controller baseflight_controller_node _imu_topic:=/iris/imu
int main(int argc, char** argv) {
    ros::init(argc, argv, "baseflight_controller_node");
    ros::Subscriber odometry_sub_;
    ros::Subscriber cmd_gateway_sub_;
    ros::Publisher actuators_pub_;
    ros::Publisher pid_pub_;
    ros::NodeHandle nh_;
    std::string odometry_topic = "/hummingbird/odometry_sensor1/odometry";
    std::string actuators_pub_topic = "/hummingbird/command/motor_speed";
    int motor[4];

    axisPID[ROLL]=0;
    axisPID[PITCH]=0;
    axisPID[YAW]=0;
    initMixer();
    conf.yawRate = 1;
    conf.P8[ROLL] = 200;
    conf.I8[ROLL] = 10;
    conf.D8[ROLL] = 30;
    conf.P8[PITCH] = 200;
    conf.I8[PITCH] = 10;
    conf.D8[PITCH] = 30;
    conf.P8[YAW] = 200;
    conf.I8[YAW] = 5;
    conf.D8[YAW] = 25;
    
    conf.P8[PIDALT] = 160;
    conf.I8[PIDALT] = 0;
    conf.D8[PIDALT] = 0;
    
    last_cycle_time = ros::Time::now();
    odometry_sub_ = nh_.subscribe(odometry_topic,1,odometryCallback);
    cmd_gateway_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1, &gatewayCallback);

    pid_pub_ = nh_.advertise<baseflight_controller::pidStatus>("baseflight_controller/pidStatus", 1);
    actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);
    
    
    int16_t command[4];
    
    ros::Rate r(250); // 400 hz
    while (ros::ok())
    {
        
        int32_t cycleTime = (ros::Time::now().nsec - last_cycle_time.nsec) / 1000;
        last_cycle_time = ros::Time::now();

        
        
        baseflight_controller::pidStatusPtr pid_msg(new baseflight_controller::pidStatus);
        ros::Time current_time = ros::Time::now();
        pid_msg->header.stamp.sec = current_time.sec;
        pid_msg->header.stamp.nsec = current_time.nsec;
        
        command[THROTTLE] = rcCommand[THROTTLE];
        command[YAW] = rcCommand[YAW];
        command[PITCH] = rcCommand[PITCH];
        command[ROLL] = rcCommand[ROLL];
        
        
        levelPid(cycleTime,command,pid_msg); // manipulates command
    
        gyroPid(cycleTime,command,pid_msg);
        
        
        
        
         pid_pub_.publish(pid_msg);
        //ROS_INFO("Gyro [%f,%f,%f] Acc [%f,%f,%f]",gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z);
        //ROS_INFO("Motor [%i,%i,%i,%i] ",motor[0],motor[1],motor[2],motor[3]);
        mixTable(motor);
        writeMotor(actuators_pub_, motor);
        
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}



