#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Pose2D {
    double x;
    double y;
    double theta;
};

std::vector<Pose2D> readGoalsFromCSV(const std::string& filename) {
    std::vector<Pose2D> goals;
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return goals;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        Pose2D pose;

        if (!std::getline(ss, item, ',')) continue;
        pose.x = std::stod(item);

        if (!std::getline(ss, item, ',')) continue;
        pose.y = std::stod(item);

        if (!std::getline(ss, item, ',')) continue;
        pose.theta = std::stod(item);

        goals.push_back(pose);
    }
    file.close();
    return goals;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher_node");
    ros::NodeHandle nh;

    // CSV file path
    std::string csv_path = "second_project/csv/goals.csv";

    // Read goals from CSV
    std::vector<Pose2D> goals = readGoalsFromCSV(csv_path);
    if (goals.empty()) {
        ROS_ERROR("No goals loaded from file.");
        return 1;
    }
    ROS_INFO("Loaded %lu goals from %s", goals.size(), csv_path.c_str());

    MoveBaseClient ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();

    for (size_t i = 0; i < goals.size(); ++i) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goals[i].x;
        goal.target_pose.pose.position.y = goals[i].y;
        goal.target_pose.pose.position.z = 0.0;

        // Convert yaw (theta) to quaternion
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goals[i].theta);

        ROS_INFO("Sending goal %lu: x=%.2f y=%.2f theta=%.2f", i, goals[i].x, goals[i].y, goals[i].theta);
        ac.sendGoal(goal);

        // Wait indefinitely for the result
        ac.waitForResult();

        auto state = ac.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal %lu reached.", i);
        } else {
            ROS_WARN("Goal %lu aborted or failed with state: %s", i, state.toString().c_str());
            // Decide here: break or continue to next goal anyway
            // For now, continue to next goal
        }
    }

    ROS_INFO("All goals processed.");
    return 0;
}
