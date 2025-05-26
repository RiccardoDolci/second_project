#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

class BicycleOdometryNode {
public:
    BicycleOdometryNode() : x_(0.0), y_(0.0), theta_(84 * M_PI / 180.0), initialized_(false) {
        ros::NodeHandle private_nh("~");
        private_nh.param("wheelbase", wheelbase_, 1.765);
        private_nh.param("steering_factor", steering_factor_, 32.0);

        speedsteer_sub_ = nh_.subscribe("/speedsteer", 1, &BicycleOdometryNode::speedsteerCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber speedsteer_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;

    double wheelbase_;
    double steering_factor_;
    double x_, y_, theta_;
    ros::Time last_time_;
    bool initialized_;

    void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        //ROS_INFO("Callback attivato! steer: %f, speed: %f", msg->point.x, msg->point.y);

        if (!initialized_) {
            last_time_ = msg->header.stamp;
            initialized_ = true;
            return;
        }

        double steer_deg = msg->point.x + 7.2;  // angolo di sterzo in gradi
        double v_kmh = msg->point.y;           // velocità in km/h

        double v = v_kmh / 3.6;  // km/h → m/s
        double alpha = (steer_deg / steering_factor_) * M_PI / 180.0;

        ros::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        double b = 1.3;  // offset laterale
        double R = wheelbase_ / tan(alpha) + b;
        double omega = v / R;

        const double EPSILON = 1e-11;
        if (std::abs(omega) < EPSILON) {
            ROS_INFO("Runge-Kutta");
            double theta_mid = theta_ + (omega * dt) / 2.0;
            x_ += v * dt * std::cos(theta_mid);
            y_ += v * dt * std::sin(theta_mid);
            theta_ += omega * dt;
        } else {
            ROS_INFO("Correct Integration");
            double theta_new = theta_ + omega * dt;
            x_ += (v / omega) * (std::sin(theta_new) - std::sin(theta_));
            y_ += -(v / omega) * (std::cos(theta_new) - std::cos(theta_));
            theta_ = theta_new;
        }

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";
        odom.child_frame_id = "vehicle";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = omega;

        odom_pub_.publish(odom);

        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "world";
        odom_tf.child_frame_id = "vehicle";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_quat;

        odom_broadcaster_.sendTransform(odom_tf);
    } 
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer");
    BicycleOdometryNode node;
    ROS_INFO("Nodo avviato correttamente!");

    ros::spin();
    return 0;
}
