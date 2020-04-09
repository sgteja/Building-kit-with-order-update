#include <osrf_gear/ConveyorBeltControl.h>
#include <ros/ros.h>

class AriacConveyorManager {
public:
    AriacConveyorManager();
    ~AriacConveyorManager();

 	void setConveyorPower(double);
    

private:
    ros::NodeHandle conveyor_nh_;
   	ros::ServiceClient conveyor_client_;
   	osrf_gear::ConveyorBeltControl beltControl_;
};