//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id, int bin) :
robot_controller_nh_("/ariac/"+arm_id),
robot_controller_options("manipulator",
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
robot_move_group_(robot_controller_options) {
    ROS_WARN(">>>>> RobotController");

    robot_move_group_.setPlanningTime(10);
    robot_move_group_.setNumPlanningAttempts(3);
    robot_move_group_.setPlannerId("TRRTkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.7);
    robot_move_group_.setMaxAccelerationScalingFactor(0.7);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);

    //-- The order of joints positions is as follows
    // ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 
    // 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    //--These are joint positions used for the home position to pick from bin
    home_joint_pose_bin_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};

    //-- The joint positions for the home position to pick from the conveyer belt
    home_joint_pose_conv_ = {0, 3.27, -2.38, -1.76, -0.57, -4.70, 0};

    joint_names_ = {"linear_arm_actuator_joint",  "shoulder_pan_joint", "shoulder_lift_joint", 
    "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    home_joint_pose_kit1_ = {1.16, 1.51, -1.26, 1.88, 4.02, -1.51, -2.03};


    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.025;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/arm1/gripper/state", 10, &RobotController::GripperCallback, this);

    // SendRobotHome(bin);

    if (bin==1) {
        end_position_ = home_joint_pose_bin_;
    }
    else {
        end_position_ = home_joint_pose_conv_;
    }

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/arm1/gripper/control");
    drop_flag_ = false;
}

RobotController::~RobotController() {}

/**
 *
 * @return
 */
bool RobotController::Planner() {
    ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
}

void RobotController::ChangeOrientation(geometry_msgs::Quaternion orientation){

    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::vector<double> joint_values = robot_move_group_.getCurrentJointValues();
    tf::Quaternion Q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(orientation,Q);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
    ROS_INFO_STREAM(">>>>> Rotation :"<< yaw);
    joint_values[6] += yaw;

    robot_move_group_.setJointValueTarget(joint_values);
    
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.5).sleep();
    }

    // ros::Duration(2.0).sleep();

    robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);

    ros::Duration(2.0).sleep();

}

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) {
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    ros::Duration(0.5).sleep();

    robot_planner_.trajectory_ = traj;


    robot_move_group_.execute(robot_planner_);
    ros::Duration(0.5).sleep();
}

void RobotController::SendRobotHome(int bin) {
    if (bin==1) {
        robot_move_group_.setJointValueTarget(home_joint_pose_bin_);
    }
    else if (bin==2){
        unsigned int i = 0;
        for (const auto &joint_name: joint_names_){
            robot_move_group_.setJointValueTarget(joint_name, home_joint_pose_kit1_[i]);
            i++;
        }
    }
    else {
        robot_move_group_.setJointValueTarget(home_joint_pose_conv_);
    }
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.5).sleep();
    }

    // ros::Duration(2.0).sleep();

    robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
                                            ros::Time(0), ros::Duration(10));
    robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
                                           ros::Time(0), robot_tf_transform_);


    fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    fixed_orientation_.w = robot_tf_transform_.getRotation().w();

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);
    ros::Duration(2.0).sleep();

}

void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

bool RobotController::DropPart(geometry_msgs::Pose part_pose, bool change_orient) {

    drop_flag_ = true;

    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){
        ROS_INFO_STREAM("Moving towards AGV1...");

       auto temp_pose = part_pose;
       temp_pose.position.z += 0.5;
       this->GoToTarget({temp_pose, part_pose});
       ros::Duration(2).sleep();
       ros::spinOnce();
       if (change_orient){
            ChangeOrientation(part_pose.orientation);
       }
       
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(false);
    

    }

    drop_flag_ = false;
    return gripper_state_;
}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {


    ROS_INFO_STREAM("Moving to part...");
    part_pose.position.z = part_pose.position.z + offset_;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.3;

    this->GoToTarget({temp_pose_1, part_pose});

    ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.01;
        this->GoToTarget({temp_pose_1, part_pose});
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }

    ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget(temp_pose_1);
    return gripper_state_;
}

bool RobotController::PickPartFromConv(geometry_msgs::Pose& part_pose) {
    ROS_INFO_STREAM("Moving to part...");
    ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();

    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.3;
    ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget(temp_pose_1);
    return gripper_state_;
}