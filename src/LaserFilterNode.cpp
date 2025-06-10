#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class LaserFilterNode {
public:
    LaserFilterNode() {
        // Publisher TF statiche
        

        // Subscriber
        clean_back_sub_ = nh_.subscribe("/scan_back", 10, &LaserFilterNode::cleanBackCallback, this);
        clean_front_sub_ = nh_.subscribe("/scan_front", 10, &LaserFilterNode::cleanFrontCallback, this);

        // Publisher
        pub_back_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_back_clean", 1);
        pub_front_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_front_clean", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber clean_back_sub_, clean_front_sub_;
    ros::Publisher pub_back_, pub_front_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    

    void cleanBackCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (msg->ranges.size() < 675) {
            ROS_WARN("Scan back troppo corto: %lu elementi", msg->ranges.size());
            return;
        }

        sensor_msgs::LaserScan clean_msg = *msg;
        clean_msg.header.frame_id = "laser_back";  // Imposta il frame corretto
        

        if (!msg->intensities.empty() && msg->intensities.size() >= 675) {
            clean_msg.intensities.assign(msg->intensities.begin() + 135, msg->intensities.begin() + 675);
        }


        clean_msg.angle_min = msg->angle_min + msg->angle_increment * 135;
        clean_msg.angle_max = clean_msg.angle_min + msg->angle_increment * (675-135-1);
        clean_msg.ranges.assign(msg->ranges.begin() + 90, msg->ranges.begin() + 675);


        pub_back_.publish(clean_msg);
    }

    void cleanFrontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (msg->ranges.size() < 711) {
            ROS_WARN("Scan front troppo corto: %lu elementi", msg->ranges.size());
            return;
        }

        sensor_msgs::LaserScan clean_msg = *msg;
        clean_msg.header.frame_id = "laser_front";  // Imposta il frame corretto
        

        if (!msg->intensities.empty() && msg->intensities.size() >= 675) {
            clean_msg.intensities.assign(msg->intensities.begin() + 135, msg->intensities.begin() + 675);
        }


        clean_msg.angle_min = msg->angle_min + msg->angle_increment * 135;
        clean_msg.angle_max = clean_msg.angle_min + msg->angle_increment * (675-135-1 );
        clean_msg.ranges.assign(msg->ranges.begin() + 90, msg->ranges.begin() + 675);
        pub_front_.publish(clean_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laserfilternode");
    LaserFilterNode node;

    ROS_INFO("Laser filter node avviato con TF statici.");


    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<geometry_msgs::TransformStamped> transforms;


    // TF da base_link a laser_back 
    geometry_msgs::TransformStamped back_tf;
    back_tf.header.stamp = ros::Time::now();
    back_tf.header.frame_id = "base_link";
    back_tf.child_frame_id = "laser_back";
    back_tf.transform.translation.x = -0.3;
    back_tf.transform.translation.y = 0.0;
    back_tf.transform.translation.z = -0.115;

    tf2::Quaternion q_back;
    q_back.setRPY(-3.142, 0.002, 3.142); 
    back_tf.transform.rotation.x = q_back.x();
    back_tf.transform.rotation.y = q_back.y();
    back_tf.transform.rotation.z = q_back.z();
    back_tf.transform.rotation.w = q_back.w();

    // TF da base_link a laser_front 
    geometry_msgs::TransformStamped front_tf;
    front_tf.header.stamp = ros::Time::now();
    front_tf.header.frame_id = "base_link";
    front_tf.child_frame_id = "laser_front";
    front_tf.transform.translation.x = 0.300;
    front_tf.transform.translation.y = 0.0;
    front_tf.transform.translation.z = -0.115;

    tf2::Quaternion q_front;
    q_front.setRPY(-3.142, 0.002, -0.002);
    front_tf.transform.rotation.x = q_front.x();
    front_tf.transform.rotation.y = q_front.y();
    front_tf.transform.rotation.z = q_front.z();
    front_tf.transform.rotation.w = q_front.w();

    // Add to vector and broadcast
    transforms.push_back(back_tf);
    transforms.push_back(front_tf);
    static_broadcaster.sendTransform(transforms);

    
    ros::spin();
    return 0;
}
