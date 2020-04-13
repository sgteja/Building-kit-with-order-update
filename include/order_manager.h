#pragma once

#include <list>
#include <map>
#include <string>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor.h"
#include "robot_controller.h"
#include "conveyor_controller.h"

class AriacOrderManager {
public:
    AriacOrderManager();
    ~AriacOrderManager();
    void OrderCallback(const osrf_gear::Order::ConstPtr& order_msg);
    void ExecuteOrder();
    std::string GetProductFrame(std::string product_type, bool conv, bool check);
    std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> GetOrder();
    std::string PickAndPlace(std::pair<std::string,geometry_msgs::Pose> object_prop,int agvnum);
    bool PickAndPlaceFromConv(std::pair<std::string,geometry_msgs::Pose> object_prop,int agvnum);
    void SubmitAGV(int num, std::string shipmentID);
    bool RemoveFailureParts(int sensor_num, std::string product_type);
    geometry_msgs::Pose PickUp(std::string product_type, std::string product_frame, int agv_id);
    bool CheckOrderUpdate(int current_order_count, std::string orderID);
    void ClearTray(int agv_id);

private:
    ros::NodeHandle order_manager_nh_;
    ros::Subscriber order_subscriber_;
    std::vector<osrf_gear::Order> received_orders_;
    AriacSensorManager camera_;
    AriacConveyorManager convBelt_;
    RobotController arm1_;
    RobotController arm2_;
    tf::TransformListener part_tf_listener_;
    std::pair<std::string, geometry_msgs::Pose> product_type_pose_;
    std::string object;
    std::map<std::string, std::vector<std::string>> product_frame_list_;
    std::map<std::string, std::vector<std::string>> product_frame_list_conv_;
    osrf_gear::Order order_;
    std::vector<std::pair<std::string, geometry_msgs::Pose>> conveyer_list_;
    std::vector<std::pair<std::string, geometry_msgs::Pose>> bin_list_;
    std::vector<std::string> parts_list_kit_1_;
    std::vector<std::string> parts_list_kit_2_;

};

