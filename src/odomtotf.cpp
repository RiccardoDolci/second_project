#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class OdomToTF {
public:
    OdomToTF() {
        odom_sub_ = nh_.subscribe("/odometry", 10, &OdomToTF::odomCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    tf::TransformBroadcaster tf_broadcaster_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = msg->header.stamp;
        odom_tf.header.frame_id = "odom";       // or "world" or your fixed frame
        odom_tf.child_frame_id = "base_link";   // or "robot", whatever your robot's frame is

        odom_tf.transform.translation.x = msg->pose.pose.position.x;
        odom_tf.transform.translation.y = msg->pose.pose.position.y;
        odom_tf.transform.translation.z = msg->pose.pose.position.z;

        odom_tf.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_.sendTransform(odom_tf);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf");
    OdomToTF node;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<geometry_msgs::TransformStamped> transforms;

    // First static transform: base_link -> body_link
    geometry_msgs::TransformStamped tf1;
    tf1.header.stamp = ros::Time::now();
    tf1.header.frame_id = "base_link";
    tf1.child_frame_id = "body_link";
    tf1.transform.translation.x = 0.0;
    tf1.transform.translation.y = 0.0;
    tf1.transform.translation.z = 0.0;

    tf2::Quaternion q1;
    q1.setRPY(0, 0, 0);
    tf1.transform.rotation.x = q1.x();
    tf1.transform.rotation.y = q1.y();
    tf1.transform.rotation.z = q1.z();
    tf1.transform.rotation.w = q1.w();

    // Second static transform: base_link -> sick_back
    geometry_msgs::TransformStamped tf2;
    tf2.header.stamp = ros::Time::now();
    tf2.header.frame_id = "base_link";
    tf2.child_frame_id = "sick_back";
    tf2.transform.translation.x = -0.3;
    tf2.transform.translation.y = 0.0;
    tf2.transform.translation.z = -0.115;

    tf2::Quaternion q2;
    q2.setRPY(-3.142, 0, 3.14159); // 180 degrees in radians
    tf2.transform.rotation.x = q2.x();
    tf2.transform.rotation.y = q2.y();
    tf2.transform.rotation.z = q2.z();
    tf2.transform.rotation.w = q2.w();

    // Add to vector and broadcast
    transforms.push_back(tf1);
    transforms.push_back(tf2);
    static_broadcaster.sendTransform(transforms);

    ROS_INFO("TF broadcaster from odometry started.");
    ros::spin();
    return 0;
}
