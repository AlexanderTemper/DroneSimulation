#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy current_joy_;
mav_msgs::RollPitchYawrateThrust control_msg_;

void joyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  control_msg_.roll = -msg->axes[1] * (30.0 * M_PI / 180.0);
  control_msg_.pitch = -msg->axes[2] * (30.0 * M_PI / 180.0);
  control_msg_.thrust.z = (-msg->axes[0] + 1) *30 / 2.0;
  control_msg_.yaw_rate = -msg->axes[3] * (2*M_PI);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "gateway_node");
    ROS_INFO("%s", "Geht");

    ros::NodeHandle nh_;
    ros::Publisher ctrl_pub_;
    ros::Subscriber joy_sub_;
    
    control_msg_.roll = 0;
    control_msg_.pitch = 0;
    control_msg_.yaw_rate = 0;
    control_msg_.thrust.x = 0;
    control_msg_.thrust.y = 0;
    control_msg_.thrust.z = 5;
    
    joy_sub_ = nh_.subscribe("joy", 10, &joyCallback);
    
    ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

    

    ros::Rate r(1000); // 1000 hz
    while (ros::ok())
    {
        ros::Time update_time = ros::Time::now();
        control_msg_.header.stamp = update_time;
        control_msg_.header.frame_id = "rotors_joy_frame";
        ctrl_pub_.publish(control_msg_);
        
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}


