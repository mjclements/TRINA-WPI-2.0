#include <ros/ros.h>

#include "pick_and_place/PickSrv.h"
#include "pick_and_place/PlaceSrv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place_demo");

    ros::NodeHandle nh;
    ros::ServiceClient pick_client = nh.serviceClient<pick_and_place::PickSrv>("/trina2_1/trina_pick");
    ros::ServiceClient place_client = nh.serviceClient<pick_and_place::PlaceSrv>("/trina2_1/trina_place");

    pick_and_place::PickSrv pick_srv;
    pick_and_place::PlaceSrv place_srv;

    std::string object;
    place_srv.request.x = 0.7;
    place_srv.request.y = 0.1;

    for (int i = 0; i < 3; i++)
    {
        object = "unit_box_" + std::to_string(i);
        pick_srv.request.pick_obj = object;

        // If pick service can be fixed, use this version with error checking:
        // if (pick_client.call(pick_srv))
        // {
        //     ROS_INFO("Picked up block %i", i);
        //     if (place_client.call(place_srv)) 
        //     { 
        //         ROS_INFO("Placed block");
        //     }   
        //     else {
        //         ROS_ERROR("Failed to place, exiting");
        //         return 1;
        //     }         
        // }
        // else {
        //     ROS_INFO("Continuing, failed to pick up block %i", i);
        // }
        pick_client.call(pick_srv);
        place_client.call(place_srv);
    }
   return 0;
}