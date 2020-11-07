#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <string>

enum class TrinaState {
    WAITING,
    COMMANDED,
    PICKING,
    PLACING,
    FAILED
};

class TrinaSM
{

private:
    TrinaState curr_state;
    ros::Subscriber sub;
    ros::Publisher pub;

public:


    TrinaSM(ros::NodeHandle *nh)
    {
        curr_state = TrinaState::WAITING;
        sub = nh->subscribe("buttons", 1000, &TrinaSM::callback, this);
        pub = nh->advertise<std_msgs::String>("state_messages", 1);
    }
    inline TrinaState getCurrentState() const { return curr_state; }
    void callback(const std_msgs::Int8::ConstPtr &msg)
    {
        switch (curr_state)
        {
        case TrinaState::WAITING:
            ROS_INFO("Current state: %s", "Waiting");
            break;
        case TrinaState::COMMANDED:
            ROS_INFO("Current state: %s", "Commanded");
            break;
        case TrinaState::PICKING:
            ROS_INFO("Current state: %s", "Picking");
            break;
        case TrinaState::PLACING:
            ROS_INFO("Current state: %s", "Placing");
            break;
        case TrinaState::FAILED:
            ROS_INFO("Current state: %s", "Failed");
        }
    
        int temp = msg->data;
        curr_state = ((0 <= temp) && (temp <= 4)) ? static_cast<TrinaState>(temp) : TrinaState::FAILED;
        std_msgs::String output;
        output.data = "changed state";
        pub.publish(output);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trina_SM");
    ros::NodeHandle nh;

    TrinaSM trina_1 = TrinaSM(&nh);

    ros::spin();

    return 0;
}