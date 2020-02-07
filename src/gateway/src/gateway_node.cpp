#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rotors_control/common.h>
#include <std_msgs/Float32MultiArray.h>
#include <gateway/tofStatus.h>

sensor_msgs::Joy current_joy_;
mav_msgs::RollPitchYawrateThrust control_msg_;
std_msgs::Float32MultiArray debug_msg_;
ros::Publisher debug_pub_;
float gyroAverage[3];

// Determine Dterm in direction of Sensor based on Acc of IMU
bool dFromAcc = true;

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


typedef enum {
    FRONT = 0,
    LEFT,
    RIGHT,
    REAR
}tofDirection;

typedef struct tof_controller_s{
	int l_error;
	int integral_e;
	float derivative1;
	float derivative2;
    int range;
    std::string tof_topic;
    ros::Subscriber subscriber;
    tofDirection direction;
} tof_controller_t;

// Wrapper for Callback uses
class TofSensor
{
    public:
        tof_controller_t *sensordata;
        void callback(const sensor_msgs::LaserScanPtr& msg){
            int range = (int)(msg->ranges[0] * 1000);
            if(range > 0){
                sensordata->range = range;
            } else {
                sensordata->range = -1;
            }
            //ROS_INFO("%s %i ",sensordata->tof_topic.c_str(), range);
        }
    
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

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

float accTrim[3] = {0};
bool AccInflightCalibrationActive = false;
int InflightcalibratingA = 0;

// Arm Controller
bool altMode = false;
int yawrate = 0;

#define M_LN2_FLOAT 0.69314718055994530942f
#define M_PI_FLOAT  3.14159265358979323846f
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/

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
    
    float pterm = 0.4;
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
    
    float pterm = 0.4;
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
 
typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;
/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

static biquadFilter_t accFilter[3];

void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType)
{
    // setup variables
    const float omega = 2.0f * M_PI_FLOAT * filterFreq * refreshRate * 0.000001f;
    const float sn = sin(omega);
    const float cs = cos(omega);
    const float alpha = sn / (2.0f * Q);

    float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

    switch (filterType) {
    case FILTER_LPF:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        b0 = (1 - cs) * 0.5f;
        b1 = 1 - cs;
        b2 = (1 - cs) * 0.5f;
        a0 = 1 + alpha;
        a1 = -2 * cs;
        a2 = 1 - alpha;
        break;
    case FILTER_NOTCH:
        b0 =  1;
        b1 = -2 * cs;
        b2 =  1;
        a0 =  1 + alpha;
        a1 = -2 * cs;
        a2 =  1 - alpha;
        break;
    case FILTER_BPF:
        b0 = alpha;
        b1 = 0;
        b2 = -alpha;
        a0 = 1 + alpha;
        a1 = -2 * cs;
        a2 = 1 - alpha;
        break;
    }

    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}
static float accumulatedMeasurements[3];
static int accumulatedMeasurementCount = 0;


int getAccInTofDir(tof_controller_t *tof){
    
    switch(tof->direction){
        case FRONT:
            return gyroAverage[X]*1000;
        case REAR:
            return -gyroAverage[X]*1000;
        case RIGHT:
            return -gyroAverage[Y]*1000;
        case LEFT:
            return gyroAverage[Y]*1000;
    }
    
    return 0;
}
int pushController(tof_controller_t *tof,int cycleTime)
{
    float pterm = 0.04;
    float dterm = 10;
    float dtermAcc = 1;
    int offsetSensor = 100; //offset sensor is away from collison
    

    if(tof->range < 0){ // No Sensor Value
        return 0;
    }
    int error = 1000 - (tof->range - offsetSensor);
    if(error < 0){ // we are not in danger zone
        return 0;
    }
    
    
    //ROS_INFO("derivative %i", derivative);
    int ki=0, kp =0, kd = 0;
    //kp = 100- (0.25/0.0024)*(1 - exp(-0.0024*error));
    if(error > 700){
        kp = constrain(pterm * error, -200, +200);
    }
    
    
    
    if(!dFromAcc){
        float derivative = ((error - tof->l_error)/(float)cycleTime) * 4000;
        float derivativeSum = tof->derivative1 + tof->derivative2 + derivative;
        tof->derivative2 = tof->derivative1;
        tof->derivative1 = derivative;
        float derivativeFiltered = derivativeSum/3;
        kd = constrain(dterm * derivativeFiltered, -50, +50);
    } else {
      
        kd = constrain((-getAccInTofDir(tof)/1000.0)*dtermAcc*error, -300, +300);
        if(kd < 0){ //only push away
            kd=0;
        }
        //ROS_INFO("d %i ",kd);
    }
    
    tof->l_error = error;
    ROS_INFO("error %i , p %i d, %i",error,kp,kd);
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


void calcPushback (int *roll, int *pitch,tof_controller_t *tof,int cycleTime){
    int heading = (int) (attitude[YAW] / 10.0f);
    //ROS_INFO("bevore %i [%i,%i,%i] ",heading - headFreeModeHold, rcCommand[THROTTLE],*roll,*pitch);
    float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);
    
    int force = 0;
    force = pushController(tof,cycleTime);
    
    switch(tof->direction){
        case FRONT:
            *roll = -force * sinDiff;
            *pitch = -force * cosDiff;
            break;
        case REAR:
            *roll = force * sinDiff;
            *pitch = force * cosDiff;
            break;
        case RIGHT:
            *pitch = force * sinDiff;
            *roll = -force * cosDiff;
            break;
        case LEFT:
            *pitch = -force * sinDiff;
            *roll = force * cosDiff;
            break;
        defaul:
            break;
    }
}



// ***** Callbacks *****
void imuCallback(const sensor_msgs::Imu& msg)
{   

    float x = msg.linear_acceleration.x;
    float y = msg.linear_acceleration.y;
    float z = msg.linear_acceleration.z;
   
    float fx = biquadFilterApply(&accFilter[X], x);
    float fy = biquadFilterApply(&accFilter[Y], y);
    float fz = biquadFilterApply(&accFilter[Z], z);

    gyroAverage[X] = fx;
    gyroAverage[Y] = fy;
    gyroAverage[Z] = fz;
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
    rcCommand[ROLL] = (-msg->axes[3] * 50);
    rcCommand[PITCH] = (msg->axes[4] * 50);
    rcCommand[YAW] = (-msg->axes[0] * 500);
    
    
    if(msg->buttons[1] == 1){ 
        yawrate = yawrate + 50;
        ROS_INFO("yawrate = %i ", yawrate);
    }
    
    if(msg->buttons[2] == 1){ 
        yawrate = yawrate - 50;
        ROS_INFO("yawrate = %i ", yawrate);
    }
    
    if(rcCommand[THROTTLE] < 1050){ // Reset Things
        setCurrentHeading();
        resetAltitudePid();
    }
    
    rcCommand[YAW] = yawrate;
}


void initTof(ros::NodeHandle *nh,tof_controller_t *tof, std::string topic, tofDirection dir, TofSensor *tofClass){
    tof->tof_topic = topic;
    tof->direction = dir;
    tof->l_error = 0;
    tof->integral_e = 0;
    tof->derivative1 = 0;
    tof->derivative2 = 0;
    tof->range = -1;
    tofClass->sensordata = tof;
    tof->subscriber = nh->subscribe(tof->tof_topic,1,&TofSensor::callback,tofClass);
    
}


int acc[2] = {0};
void updateAccXY()
{
    acc[X] = gyroAverage[X]*1000;
    acc[Y] = gyroAverage[Y]*1000;
  
    calcHeadFree(&acc[Y], &acc[X]);
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
    ros::Subscriber imu_sub_;
    
    std::string odometry_topic = "/hummingbird/odometry_sensor1/odometry";
    std::string tof_topic = "/hummingbird/ground_truth/pose";
    std::string tof_ground_topic = "/hummingbird/tof_ground_sensor";
    std::string imu_topic = "/hummingbird/imu";

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
    imu_sub_ = nh_.subscribe(imu_topic,1,imuCallback);
    tof_sub_ = nh_.subscribe(tof_topic,1,tofCallback);
    ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);
    debug_pub_ = nh_.advertise<std_msgs::Float32MultiArray> ("debug", 1);
    
    gateway::tofStatusPtr tof_status(new gateway::tofStatus);
    
    // init tof Sensors
    tof_controller_t tof_front;
    TofSensor tof_front_class;
    initTof(&nh_,&tof_front,"/hummingbird/tof_front_sensor",FRONT,&tof_front_class);
    
    tof_controller_t tof_back;
    TofSensor tof_back_class;
    initTof(&nh_,&tof_back,"/hummingbird/tof_back_sensor",REAR,&tof_back_class);
    
    tof_controller_t tof_left;
    TofSensor tof_left_class;
    initTof(&nh_,&tof_left,"/hummingbird/tof_left_sensor",LEFT,&tof_left_class);
    
    tof_controller_t tof_right;
    TofSensor tof_right_class;
    initTof(&nh_,&tof_right,"/hummingbird/tof_right_sensor",RIGHT,&tof_right_class);
    
    for (int axis = 0; axis < 3; axis++) {
        biquadFilterInit(&accFilter[axis], 10, 500, BIQUAD_Q, FILTER_LPF);
    }
    
   
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
            int pushRoll,pushPitch;
            
            
            debug_msg_.data.clear();
            // ***** Front TOF *****
            calcPushback(&pushRoll,&pushPitch,&tof_front,cycleTime);
            roll = constrain(roll + pushRoll,-500,500);
            pitch = constrain(pitch + pushPitch,-500,500);
            
            
            // ***** Rear TOF *****
            calcPushback(&pushRoll,&pushPitch,&tof_back,cycleTime);
            roll = constrain(roll + pushRoll,-500,500);
            pitch = constrain(pitch + pushPitch,-500,500);
            
            
            // ***** Right TOF *****
            calcPushback(&pushRoll,&pushPitch,&tof_right,cycleTime);
            roll = constrain(roll + pushRoll,-500,500);
            pitch = constrain(pitch + pushPitch,-500,500);
            
            
            // ***** Left TOF *****
            calcPushback(&pushRoll,&pushPitch,&tof_left,cycleTime);
            roll = constrain(roll + pushRoll,-500,500);
            pitch = constrain(pitch + pushPitch,-500,500);
   
                
            // current acc in x,y
            updateAccXY();
            
            debug_pub_.publish(debug_msg_);
    
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


