
//--Created by Gnyana Teja.S 03/09/20

#include "conveyor_controller.h" 

//todo RWA-3 remove this later, stopping conveyer belt    
    
AriacConveyorManager::AriacConveyorManager() {
    ROS_INFO_STREAM(">>>>>>>>>> Connecting to Conveyor Client>>>>>>>>>>>");
    conveyor_client_ = 
            conveyor_nh_.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
}

AriacConveyorManager::~AriacConveyorManager() {}

void AriacConveyorManager::setConveyorPower(double power){

    if (!conveyor_client_.exists()){
        conveyor_client_.waitForExistence();
    }
    
    beltControl_.request.power = power;
    conveyor_client_.call(beltControl_);
    if (!beltControl_.response.success) {  // If not successful, print out why.
        ROS_ERROR_STREAM("Failed to control conveyer: " << beltControl_.response.success);
    } else {
        ROS_INFO("Conveyor controlled!");
    }

}