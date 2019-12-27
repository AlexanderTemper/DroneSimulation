#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>

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

// rosrun baseflight_controller baseflight_controller_node _imu_topic:=/iris/imu
int main(int argc, char** argv) {
    ros::init(argc, argv, "baseflight_controller_node");
    ros::Subscriber imu_sub_;
    ros::Publisher actuators_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh("~");
    std::string imu_sub_topic;
    std::string actuators_pub_topic;

    pnh.param("imu_topic", imu_sub_topic, std::string(mav_msgs::default_topics::IMU));    
    pnh.param("actuators_pub_topic", actuators_pub_topic, std::string("/iris/command/motor_speed"));
    
    ROS_INFO("Topic %s , %s ", imu_sub_topic.c_str(),actuators_pub_topic.c_str());

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(imu_sub_topic,10,imuCallback);

    actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);
    
    ros::Rate r(100); // 1000 hz
    while (ros::ok())
    {
        ROS_INFO("Gyro [%f,%f,%f] Acc [%f,%f,%f]",gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z);
        

        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

        actuator_msg->angular_velocities.clear();
        actuator_msg->angular_velocities.push_back(100);
        actuator_msg->angular_velocities.push_back(0);
        actuator_msg->angular_velocities.push_back(0);
        actuator_msg->angular_velocities.push_back(0);

    
        
        ros::Time current_time = ros::Time::now();
        
        actuator_msg->header.stamp.sec = current_time.sec;
        actuator_msg->header.stamp.nsec = current_time.nsec;

        actuators_pub_.publish(actuator_msg);
        
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}




