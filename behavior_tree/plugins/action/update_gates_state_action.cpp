#include "ros/ros.h"
#include "asv_utils/geometry_utils.h"

#include "behavior_tree/plugins/action/update_gates_state_action.hpp"

const std::string totem_string = "totem";
const std::string surmark_string = "surmark";
const std::string red_totem_string = "red_totem";
const std::string red_surmark_string = "surmark950410";

bool is_cylinder(const std::string name) {
    return (name.find(totem_string) != std::string::npos) || 
           (name.find(surmark_string) != std::string::npos);
}

bool is_red(const std::string name) {
    return (name == red_totem_string) || (name == red_surmark_string);
}

std::string get_name(const geometry_msgs::PoseStamped object) {
    return object.header.frame_id;
}

bool is_valid_gate_pair(const geometry_msgs::PoseStamped gate_1, const geometry_msgs::PoseStamped gate_2) {
    return ((gate_1.pose.position.y > gate_2.pose.position.y) && is_red(get_name(gate_1)) && !is_red(get_name(gate_2))) ||
           ((gate_1.pose.position.y < gate_2.pose.position.y) && !is_red(get_name(gate_1)) && is_red(get_name(gate_2)));
}

namespace behavior_tree
{
UpdateGatesStateAction::UpdateGatesStateAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(action_name, conf)
{}

BT::NodeStatus UpdateGatesStateAction::tick() 
{
    std::vector<asv_msgs::DetectedObject> new_detected_objects;
    std::vector<geometry_msgs::PoseStamped> new_cylinder_objects;
    geometry_msgs::Pose new_robot_pose;

    geometry_msgs::PoseStamped new_gate_1;
    geometry_msgs::PoseStamped new_gate_2;

    std::string state;
    
    if (!getInput("detected_objects", new_detected_objects)) {
        ROS_ERROR("[UpdateGatesState] Unable to obtain detected objects from blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("pose", new_robot_pose)) {
        ROS_ERROR("[UpdateGatesState] Unable to obtain robot pose from blackboard");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::Pose zero_pose;
    zero_pose.position.x = 0.0;
    zero_pose.position.y = 0.0;

    for (auto object : new_detected_objects) {
        std::string object_name = object.name;
        if (is_cylinder(object_name)) {
            geometry_msgs::PoseStamped cylinder;
            cylinder.header.frame_id = object_name;
            cylinder.pose.position.x = object.rel_coords[0];
            cylinder.pose.position.y = object.rel_coords[1];
            cylinder.pose.position.z = object.rel_coords[2];

            if (cylinder.pose.position.x > 2.0) {
                new_cylinder_objects.push_back(cylinder);
            }
        }
    }

    if (new_cylinder_objects.empty()) {
        ROS_WARN("No gates detected");
        state = "noGate";

    } else if (new_cylinder_objects.size() == 1) {
        if (is_red(get_name(new_cylinder_objects[0]))) {
            state = "redGate";
        } else {
            state = "nonRedGate";
        }

        new_gate_1 = new_cylinder_objects[0];

    } else {
        int gate_1_index = -1;
        int gate_2_index = -1;

        double gate_1_distance = -1;
        double gate_2_distance = -1;

        for (int i = 0; i < new_cylinder_objects.size(); i++) {
            geometry_msgs::Pose object_pose = new_cylinder_objects[i].pose;
            double dist = geometry_utils::euclidean_distance(zero_pose, object_pose);

            if (gate_1_index == -1) {
                gate_1_index = i;
                gate_1_distance = dist;

            } else if (gate_2_index == -1) {
                if (dist < gate_1_distance) {
                    gate_2_index = gate_1_index;
                    gate_2_distance = gate_1_distance;

                    gate_1_index = i;
                    gate_1_distance = dist;

                } else {
                    gate_2_index = i;
                    gate_2_distance = dist;
                }

            } else if (dist < gate_1_distance) {
                gate_2_index = gate_1_index;
                gate_2_distance = gate_1_distance;

                gate_1_index = i;
                gate_1_distance = dist;

            } else if (dist < gate_2_distance) {
                gate_2_index = i;
                gate_2_distance = dist;
            }
        }

        new_gate_1 = new_cylinder_objects[gate_1_index];
        new_gate_2 = new_cylinder_objects[gate_2_index];

        if (is_valid_gate_pair(new_gate_1, new_gate_2)) {
            state = "gatePair";
        } else {
            if (is_red(get_name(new_gate_1))) {
                state = "redGate";
            } else {
                state = "nonRedGate";
            }
        }
    }

    // Update states, previous states still saved at this point in case we wanna do anything fancy
    gates_state_ = state;
    gate_1_ = new_gate_1;
    gate_2_ = new_gate_2;
    robot_pose_ = new_robot_pose;
    cylinder_objects_ = new_cylinder_objects;

    // Output everything
    setOutput("gates_state", gates_state_);
    setOutput("gate_1", gate_1_);
    setOutput("gate_2", gate_2_);

    // ROS_INFO("Current gate state: %s", gates_state_.c_str());

    return BT::NodeStatus::SUCCESS;
}

} // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<behavior_tree::UpdateGatesStateAction>("UpdateGatesState");
}