#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

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
    ROS_INFO("TF broadcaster from odometry started.");
    ros::spin();
    return 0;
}
