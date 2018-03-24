// this node will publish the topic "cylinder_blocks_poses"
// including all current cylinder blocks, pose is 3-D position

// ros communication:
    // subscribe to topic "/current_cylinder_blocks"
    // subscribe to topic "/gazebo/model_states"
    // publish the topic "/cylinder_blocks_poses"

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Int8MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <ur5_notebook/blocks_poses.h>

// global variables
int g_quantity;
std::vector<int8_t> g_current_blocks;
std::vector<double> g_x;
std::vector<double> g_y;
std::vector<double> g_z;
bool g_current_callback_started = false;
bool g_poses_updated = false;  // act as frequency control of publish loop

std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

void currentCallback(const std_msgs::Int8MultiArray& current_blocks) {
    // this topic contains information of what cylinder blocks have been spawned
    if (!g_current_callback_started) {
        // set first time started flag to true
        g_current_callback_started = true;
        ROS_INFO("current callback has been invoked first time");
    }
    g_quantity = current_blocks.data.size();
    g_current_blocks.resize(g_quantity);
    g_current_blocks = current_blocks.data;
}

void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) {
    // this callback update global values of cylinder positions
    if (g_current_callback_started) {
        // only update when currentCylinderCallback has been invoked the first time
        // get cylinder blocks positions according to settings in g_current_cylinder_blocks
        std::vector<double> box_x;
        std::vector<double> box_y;
        std::vector<double> box_z;
        box_x.resize(g_quantity);
        box_y.resize(g_quantity);
        box_z.resize(g_quantity);
        // find position of all current cylinders in topic message
        bool poses_completed = true;
        for (int i=0; i<g_quantity; i++) {
            // get index of ith cylinder
            std::string indexed_model_name;
            if (g_current_blocks[i] == 0) {
                indexed_model_name = "red_cylinder_" + intToString(i);
            }
            else {
                indexed_model_name = "blue_cylinder_" + intToString(i);
            }
            int index = -1;
            int model_quantity = current_model_states.name.size();  // number of models measured
            for (int j=0; j<model_quantity; j++) {
                if (current_model_states.name[j] == indexed_model_name) {
                    index = j; break;
                }
            }
            if (index != -1) {
                // this model name has been successfully indexed
                box_x[i] = current_model_states.pose[index].position.x;
                box_y[i] = current_model_states.pose[index].position.y;
                box_z[i] = current_model_states.pose[index].position.z;
            }
            else {
                // ROS_ERROR("fail to find model name in the model_states topic");
                // in the test run, there is chance that the last cylinder is not in the topic message
                // and g_cylinder_quantity (fron spawner node) is larger than the cylinder quantity here
                // because /gazebo/model_states are sampled at a high rate of about 1000Hz
                // so the position data should be aborted if fail to find the last cylinder
                poses_completed = false;
            }
        }
        if (poses_completed) {
            // only pass data to globals when they are completed
            g_x.resize(g_quantity);
            g_y.resize(g_quantity);
            g_z.resize(g_quantity);
            g_x = box_x;
            g_y = box_y;
            g_z = box_z;
            if (!g_poses_updated) {
                // reset flag to true, after updating global value of g_x, g_y, g_z
                g_poses_updated = true;
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "blocks_poses_publisher");
    ros::NodeHandle nh;

    // initialize subscribers for "/current_cylinder_blocks" and "/gazebo/model_states"
    ros::Subscriber current_subscriber = nh.subscribe("/current_blocks"
        , 1, currentCallback);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // initialize publisher for "/cylinder_blocks_poses"
    ros::Publisher poses_publisher
        = nh.advertise<ur5_notebook::blocks_poses>("blocks_poses", 1);
    ur5_notebook::blocks_poses current_poses_msg;

    // publishing loop
    while (ros::ok()) {
        if (g_poses_updated) {
            // only publish when cylinder positions are updated
            // no need to publish repeated data
            g_poses_updated = false;  // set flag to false
            // there is tiny possibility that g_x is not in the length of g_cylinder_quantity
            int local_quantity = g_x.size();  // so get length of g_x
            current_poses_msg.x.resize(local_quantity);
            current_poses_msg.y.resize(local_quantity);
            current_poses_msg.z.resize(local_quantity);
            current_poses_msg.x = g_x;
            current_poses_msg.y = g_y;
            current_poses_msg.z = g_z;
            poses_publisher.publish(current_poses_msg);
        }
        ros::spinOnce();
    }
    return 0;
}
