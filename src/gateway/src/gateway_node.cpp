#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <rotors_control/common.h>

sensor_msgs::Joy current_joy_;
mav_msgs::RollPitchYawrateThrust control_msg_;


int attitude[3];
static int16_t rcCommand[4];  // interval [1000;2000] for THROTTLE and [-500;+500] for
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

int headFreeModeHold = 0;

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
int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
int current_distance;
void tofCallback(const geometry_msgs::PosePtr& msg)
{   
    current_distance = msg->position.z * 1000;
    //ROS_INFO("Z %i ", current_distance);
}

ros::Time last_cycle_time;
int integral;
int last_error;
float p = 0.01;
float i = 0;
float d = 0;
int hoverat = 500;
int altHoldThrottle = 0;
int getAltitudeThrottle(int distance, int target_distance,int cycleTime)
{
    int dTime = cycleTime;
    
    int error = target_distance - distance;
    integral = constrain(integral + error, -32000, +32000);
    int derivative = error - last_error;

    int kp = constrain(p * error, -400, +400);
    int ki = constrain(i * integral, -500, +500);
    int kd = constrain(d * derivative, -250, +250);
    last_error = error;
    
    return kp + ki + kd;
}

void joyCallback(const sensor_msgs::JoyConstPtr& msg) {
   
    rcCommand[THROTTLE]= (-msg->axes[0] + 1) * 500 + 1000;
    rcCommand[ROLL] = (-msg->axes[1] * 500);
    rcCommand[PITCH] = (-msg->axes[2] * 500);
    rcCommand[YAW] = (-msg->axes[3] * 500);
    int heading = (int) (attitude[YAW] / 10.0f);
    if(rcCommand[THROTTLE] < 1050){
        headFreeModeHold = heading;
    }

     
    //ROS_INFO("RC-Raw %i [%i,%i,%i,%i] ",heading - headFreeModeHold, rcCommand[THROTTLE],rcCommand[ROLL],rcCommand[PITCH],rcCommand[YAW]);
    float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);

    int rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
    rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "gateway_node");
    ros::NodeHandle nh_;
    ros::Publisher ctrl_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber tof_sub_;
    std::string odometry_topic = "/hummingbird/odometry_sensor1/odometry";
    std::string tof_topic = "/hummingbird/ground_truth/pose";
    
    integral = 0;
    last_error = 0;
    current_distance = 0;
    last_cycle_time = ros::Time::now();
    rcCommand[THROTTLE] = 1000;
    rcCommand[ROLL] = 0;
    rcCommand[PITCH] = 0;
    rcCommand[YAW] = 0;
    
    control_msg_.roll = 0;
    control_msg_.pitch = 0;
    control_msg_.yaw_rate = 0;
    control_msg_.thrust.x = 0;
    control_msg_.thrust.y = 0;
    control_msg_.thrust.z = 0;
    
    
    joy_sub_ = nh_.subscribe("joy", 1, &joyCallback);
    odometry_sub_ = nh_.subscribe(odometry_topic,1,odometryCallback);
    tof_sub_ = nh_.subscribe(tof_topic,1,tofCallback);
    
    ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);

    

    ros::Rate r(250);
    while (ros::ok())
    {
        int cycleTime = (ros::Time::now().nsec - last_cycle_time.nsec) / 1000;
        if(cycleTime == 0 || cycleTime > 10000)
        {
            ROS_INFO("cycleTime out of scope %i ", cycleTime);
            last_cycle_time = ros::Time::now();
            
        } else {
            int thrust_alt = getAltitudeThrottle(current_distance, hoverat,cycleTime);
            altHoldThrottle = constrain(rcCommand[THROTTLE] + thrust_alt, 1050, 1800);
            ROS_INFO("Thr %i %i %i %i ",thrust_alt, rcCommand[THROTTLE],rcCommand[THROTTLE] + thrust_alt, altHoldThrottle);

            control_msg_.roll = rcCommand[ROLL];
            control_msg_.pitch = rcCommand[PITCH];
            control_msg_.yaw_rate = rcCommand[YAW];
            control_msg_.thrust.z = altHoldThrottle;
            ros::Time update_time = ros::Time::now();
            control_msg_.header.stamp = update_time;
            control_msg_.header.frame_id = "rotors_joy_frame";
            ctrl_pub_.publish(control_msg_);
        }
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}


