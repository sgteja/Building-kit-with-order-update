//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



// AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);

}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type, bool conv=false, bool check=true) {
    //--Grab the last one from the list then remove it
    if (conv){
        if (!product_frame_list_conv_.empty() && product_frame_list_conv_.find(product_type)!=product_frame_list_conv_.end()) {
            std::string frame = product_frame_list_conv_[product_type].back();
            ROS_INFO_STREAM("Frame >>>> " << frame);
            product_frame_list_conv_[product_type].pop_back();
            return frame;
        } else {
            return "-1";
        }
    }
    else{
        if (!product_frame_list_.empty() && product_frame_list_.find(product_type)!=product_frame_list_.end()) {
            std::string frame = product_frame_list_[product_type].back();
            ROS_INFO_STREAM("Frame >>>> " << frame);
            if (!check){
                product_frame_list_[product_type].pop_back();
            }
            return frame;
        } else {
            return "-1";
        }
    }
}

geometry_msgs::Pose AriacOrderManager::PickUp(std::string product_type, std::string product_frame, int agv_id){

    std::string product_part_frame = product_frame;
    std::string delimiter = "_";
    unsigned int pos = 0;
    unsigned int count = 0;
    std::string token;
    int binNumber;
    while ((pos = product_part_frame.find(delimiter)) != std::string::npos) {
        token = product_part_frame.substr(0, pos);
        count++;
        product_part_frame.erase(0, pos + 1);
        if (count == 3){
            break;
        }
    }
    int bin_number = std::stoi(token,nullptr,0);
    auto part_pose = camera_.GetPartPose("/world", product_frame);
    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
    double offset = part_pose.position.z*0.1;
    bool failed_pick;
    geometry_msgs::Pose rackDrop;
    rackDrop.position.x = 0.3;
    rackDrop.position.y = 0.0;
    rackDrop.position.z = 1;
    rackDrop.orientation.x = 0;
    rackDrop.orientation.y = 0;
    rackDrop.orientation.z = 0;
    rackDrop.orientation.z = 1;
    if (agv_id ==1){
        
        if (bin_number<=2){
            bool failed_pick = arm2_.PickPart(part_pose);
            arm1_.SendRobotHome("arm1_exch");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                auto part_pose = camera_.GetPartPose("/world", product_frame);
                failed_pick = arm2_.PickPart(part_pose);
            }
            arm2_.SendRobotHome("arm2_exch");
            bool drop = arm2_.DropPart(rackDrop,false);
            arm2_.SendRobotHome("bin");
            rackDrop.position.z = 0.955;
            
            double r=0, p=0, y=-1.0;
            tf2::Quaternion q_rot, q_new, q_old;
            q_rot.setRPY(r,p,y);
            q_old[0] = part_pose.orientation.x;
            q_old[1] = part_pose.orientation.y;
            q_old[2] = part_pose.orientation.z;
            q_old[3] = part_pose.orientation.w;
            q_new = q_rot * q_old;
            q_new.normalize();
            rackDrop.orientation.x = q_new[0];
            rackDrop.orientation.y = q_new[1];
            rackDrop.orientation.z = q_new[2];
            rackDrop.orientation.w = q_new[3];

            failed_pick = arm1_.PickPart(rackDrop);
            while(!failed_pick){
                failed_pick = arm1_.PickPart(rackDrop);
            }
        }

        else{
            bool failed_pick = arm1_.PickPart(part_pose);
            while(!failed_pick){
                failed_pick = arm1_.PickPart(part_pose);
            }
            rackDrop = part_pose;
        }
    }

    else {
        if (bin_number>=5){
            bool failed_pick = arm1_.PickPart(part_pose);
            arm2_.SendRobotHome("arm2_exch");
            ROS_WARN_STREAM("Picking up state "<< failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                auto part_pose = camera_.GetPartPose("/world", product_frame);
                failed_pick = arm1_.PickPart(part_pose);
            }
            arm1_.SendRobotHome("arm1_exch");
            bool drop = arm1_.DropPart(rackDrop,false);
            arm1_.SendRobotHome("bin");
            rackDrop.position.z = 0.955;
           
            double r=0, p=0, y=1.57;
            tf2::Quaternion q_rot, q_new, q_old;
            q_rot.setRPY(r,p,y);
            q_old[0] = part_pose.orientation.x;
            q_old[1] = part_pose.orientation.y;
            q_old[2] = part_pose.orientation.z;
            q_old[3] = part_pose.orientation.w;
            q_new = q_rot * q_old;
            q_new.normalize();
            rackDrop.orientation.x = q_new[0];
            rackDrop.orientation.y = q_new[1];
            rackDrop.orientation.z = q_new[2];
            rackDrop.orientation.w = q_new[3];
            
            failed_pick = arm2_.PickPart(rackDrop);
            while(!failed_pick){
                failed_pick = arm2_.PickPart(rackDrop);
            }
        }

        else{
            bool failed_pick = arm2_.PickPart(part_pose);
            while(!failed_pick){
                failed_pick = arm2_.PickPart(part_pose);
            }
            rackDrop = part_pose;
        }
    }

    return rackDrop;

}

std::string AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    
    arm1_.SendRobotHome("bin");
    arm2_.SendRobotHome("bin");

    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = this->GetProductFrame(product_type, false,false);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    
    auto part_pose = camera_.GetPartPose("/world",product_frame);

    part_pose = this->PickUp(product_type, product_frame, agv_id);
    

    
    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        arm1_.SendRobotHome("kit1");
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y += 0.05;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.orientation.x <<","<< StampedPose_out.pose.orientation.y 
                        << "," << StampedPose_out.pose.orientation.z<< "," << StampedPose_out.pose.orientation.w<<")");
        auto temp_frame = product_frame;
        temp_frame[15] = '8';
        parts_list_kit_1_.push_back(temp_frame);
    }
    else{
        arm2_.SendRobotHome("kit2");
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y -= 0.05;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
        auto temp_frame = product_frame;
        temp_frame[15] = '9';
        parts_list_kit_2_.push_back(temp_frame);
    }
 
    if (agv_id==1){
        auto result = arm1_.DropPart(StampedPose_out.pose, true, part_pose);
    }
    else{
        auto result = arm2_.DropPart(StampedPose_out.pose, true, part_pose);
    }
    // ros::Duration(10.0).sleep();


    // return result;
    return product_frame;
}


bool AriacOrderManager::PickAndPlaceFromConv(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    
    if (agv_id==1){
        arm1_.SendRobotHome("conv");
    }
    else{
        arm2_.SendRobotHome("conv");
    }

    int i = 0;
    geometry_msgs::Pose part_pose;
    while(true){
        std::string product_type = product_type_pose.first;
        ROS_WARN_STREAM("Product type >>>> " << product_type);
        int part_num{0};
        while(this->GetProductFrame(product_type, true)=="-1"){
            ros::spinOnce();
            ros::Duration(1).sleep();
            ROS_WARN_STREAM_THROTTLE(20,"Waiting for product to arrive");
            product_frame_list_conv_ = camera_.get_product_frame_list_conv();    
        }
            
        std::string product_frame = this->GetProductFrame(product_type, true);
        part_num = camera_.get_break_beam_trig_counter(2);
        ROS_WARN_STREAM("Product frame >>>> " << product_frame);
        if (i==0){
            part_pose = camera_.GetPartPoseFromConv("/world",product_frame);
        }
        auto temp_pose = part_pose;
        if(product_type == "pulley_part")
                temp_pose.position.z += 0.08;
        temp_pose.position.y = 1.05;
        //--task the robot to pick up this part
        ROS_INFO_STREAM("Moving to part...");
        temp_pose.position.z += 0.03;
        if (agv_id==1){
            arm1_.GoToTarget(temp_pose);
        }
        else{
            arm2_.GoToTarget(temp_pose);
        }
        while(camera_.get_break_beam_trig_counter(1)!=part_num){
            ros::spinOnce();
        }

        temp_pose.position.z -= 0.015;

        bool failed_pick;
        if (agv_id==1){
            arm1_.GoToTarget(temp_pose);
            failed_pick = arm1_.PickPartFromConv(temp_pose);
        }
        else{
            arm1_.GoToTarget(temp_pose);
            failed_pick = arm1_.PickPartFromConv(temp_pose);
        }
        ROS_WARN_STREAM("Picking up state " << failed_pick);

        if (failed_pick){
            break;
        }

        if (agv_id==1){
            arm1_.SendRobotHome("conv");
        }
        else{
            arm2_.SendRobotHome("conv");
        }
        
        i++;
    }
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y -= 0.1;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        StampedPose_out.pose.position.y += 0.1;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
    // ros::Duration(5.0).sleep();
    // ros::spinOnce();
    bool result;
    if (agv_id==1){
        arm1_.ChangeOrientation(StampedPose_out.pose.orientation, part_pose.orientation);
        result = arm1_.DropPart(StampedPose_out.pose,false);
    }
    else {
        arm2_.ChangeOrientation(StampedPose_out.pose.orientation, part_pose.orientation);
        result = arm2_.DropPart(StampedPose_out.pose,false);
    }   
    // ros::Duration(10.0).sleep();

    // arm1_.SendRobotHome(0);

    return result;
}

bool AriacOrderManager::CheckOrderUpdate(int current_order_count, std::string orderID){

    ROS_WARN_STREAM("received_orders size "<< received_orders_.size());
    
    if (current_order_count+1 == received_orders_.size()){
        return false;
    }

    else {
        ROS_WARN_STREAM("received_orders last id "<< received_orders_[current_order_count+1].order_id <<" , "
        << "current order id "<< orderID);
        if(orderID[6] == received_orders_[current_order_count+1].order_id[6]){
            return true;
        }
    }

    return false;

}

void AriacOrderManager::ClearTray(int agv_id){

    if (agv_id == 1){
        while(parts_list_kit_1_.size()!=0){
            std::string frame = parts_list_kit_1_.back();
            std::string product_type = frame.substr(17,6);
            auto part_pose = camera_.GetPartPose("/world",frame);
            if(product_type == "pulley")
                part_pose.position.z += 0.08;
            //--task the robot to pick up this part
            bool failed_pick;
            failed_pick = arm1_.PickPart(part_pose);
            ROS_WARN_STREAM("Picking up state " << failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                auto part_pose = camera_.GetPartPose("/world",frame);
                failed_pick = arm1_.PickPart(part_pose);
            }
            part_pose.position.z += 0.3;
            part_pose.position.x -= 0.1;
            part_pose.position.y -= 0.8;
            bool result;
            result = arm1_.DropPart(part_pose, false);
            parts_list_kit_1_.pop_back();
        }
    }
    else{
        while(parts_list_kit_2_.size()!=0){
            std::string frame = parts_list_kit_2_.back();
            std::string product_type = frame.substr(17,6);
            auto part_pose = camera_.GetPartPose("/world",frame);
            if(product_type == "pulley")
                part_pose.position.z += 0.08;
            //--task the robot to pick up this part
            bool failed_pick;
            failed_pick = arm2_.PickPart(part_pose);
            ROS_WARN_STREAM("Picking up state " << failed_pick);
            ros::Duration(1).sleep();
            while(!failed_pick){
                auto part_pose = camera_.GetPartPose("/world",frame);
                failed_pick = arm2_.PickPart(part_pose);
            }
            part_pose.position.z += 0.3;
            part_pose.position.y += 0.8;
            part_pose.position.x -= 0.1;
            bool result;
            result = arm2_.DropPart(part_pose, false);
            parts_list_kit_2_.pop_back();
        }
    }

}


void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};
    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    product_frame_list_ = camera_.get_product_frame_list();
    int current_order_count = 0;
    // Reading the order
    // for (const auto &order:received_orders_){
    while(received_orders_.size()!=current_order_count){
        auto order = received_orders_[current_order_count];
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            for (const auto &product: products){
                ros::spinOnce();
                product_type_pose_.first = product.type;
                ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
                if (this->GetProductFrame(product_type_pose_.first)=="-1"){
                    // pick_n_place_success =  PickAndPlaceFromConv(product_type_pose_, agv_id);
                    conveyer_list_.push_back(product_type_pose_);
                }
                else{
                    // pick_n_place_success = PickAndPlace(product_type_pose_, agv_id);
                    bin_list_.push_back(product_type_pose_);
                }
                
            }
                // --todo: What do we do if pick and place fails?
            for (const auto &conveyer_part: conveyer_list_){
                pick_n_place_success =  PickAndPlaceFromConv(conveyer_part, agv_id);
            }
            for (const auto &bin_part: bin_list_){
                bool result = true;
                while(result){
                    std::string picked_part_frame =  PickAndPlace(bin_part, agv_id);
                    ros::Duration(1).sleep();
                    ros::spinOnce();
                    result = RemoveFailureParts(agv_id, bin_part.first);
                }
                if (CheckOrderUpdate(current_order_count, order_id)){
                    ROS_WARN_STREAM(">>>>>>>>Order Update is triggered<<<<<<<<<");
                    ClearTray(agv_id);
                    break;
                }
            }
            conveyer_list_.clear();
            bin_list_.clear();
            if(agv_id==1){
                arm1_.SendRobotHome("arm1");
            }
            else {
                arm2_.SendRobotHome("arm2");
            }

            //-- Check for order update
            if (!CheckOrderUpdate(current_order_count, order_id)){
                ros::Duration(6).sleep();
                ros::spinOnce();
            }
            if (CheckOrderUpdate(current_order_count, order_id)){
                ROS_WARN_STREAM(">>>>>>>>Order Update is triggered<<<<<<<<<");
                if(agv_id==1){
                    if (parts_list_kit_1_.size()>0){
                        ClearTray(agv_id);
                    }
                }
                else{
                    if (parts_list_kit_2_.size()>0){
                        ClearTray(agv_id);
                    }
                }
                break;
            }
            else{
                ROS_INFO_STREAM("Shipment id: "<< shipment_type);
                SubmitAGV(agv_id, shipment_type);
                ROS_INFO_STREAM("Submitting AGV 1");
                if(agv_id==1){
                    parts_list_kit_1_.clear();
                }
                else{
                    parts_list_kit_2_.clear();
                }
            }
        }
        current_order_count ++;
    }
}

bool AriacOrderManager::RemoveFailureParts(int sensor_num, std::string product_type){

    if (camera_.get_faulty_parts_num(sensor_num)>0){
        std::string product_part_frame;
        if(sensor_num==1){
            product_part_frame = parts_list_kit_1_.back();
            parts_list_kit_1_.pop_back();
        }
        else{
            product_part_frame = parts_list_kit_2_.back();
            parts_list_kit_2_.pop_back();
        }
        std::string delimiter = "_";
        unsigned int pos = 0;
        unsigned int count = 0;
        std::string token;
        while ((pos = product_part_frame.find(delimiter)) != std::string::npos) {
            token = product_part_frame.substr(0, pos);
            count++;
            product_part_frame.erase(0, pos + 1);
            if (count == 6){
                break;
            }
        }
        std::string product_frame = "quality_control_sensor_"+std::to_string(sensor_num)+
                                    "_model_"+token+"_frame";
        ROS_INFO_STREAM(">>>>>> Faulty part frame : "<< product_frame);

        auto part_pose = camera_.GetPartPose("/world",product_frame);


        if(product_type == "pulley_part")
            part_pose.position.z += 0.08;
        //--task the robot to pick up this part
        bool failed_pick;
        if (sensor_num==1){
            failed_pick = arm1_.PickPart(part_pose);
        }
        else {
            failed_pick = arm2_.PickPart(part_pose);
        }
        ROS_WARN_STREAM("Picking up state " << failed_pick);
        ros::Duration(1).sleep();

        while(!failed_pick){
            auto part_pose = camera_.GetPartPose("/world",product_frame);
            if (sensor_num==1){
                failed_pick = arm1_.PickPart(part_pose);
            }
            else {
                failed_pick = arm2_.PickPart(part_pose);
            }
        }

        

        bool result;
        if (sensor_num==1){
            part_pose.position.z += 0.3;
            part_pose.position.x -= 0.1;
            part_pose.position.y -= 0.8;
            result = arm1_.DropPart(part_pose, false);

        }
        else{
            part_pose.position.z += 0.3;
            part_pose.position.x -= 0.1;
            part_pose.position.y += 0.8;
            result = arm2_.DropPart(part_pose, false);
        }


        return true;

    }
    else{
        return false;
    }

}

void AriacOrderManager::SubmitAGV(int num, std::string shipmentID) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    srv.request.shipment_type = shipmentID;
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}
