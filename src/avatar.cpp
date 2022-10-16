#include "avatar.h"
#include <fstream>
using namespace TOCABI;

// ofstream MJ_graph("/home/dyros/data/myeongju/MJ_graph.txt");
// ofstream MJ_graph1("/home/dyros/data/myeongju/MJ_graph1.txt");
// ofstream MJ_joint1("/home/dyros/data/myeongju/MJ_joint1.txt");
// ofstream MJ_joint2("/home/dyros/data/myeongju/MJ_joint2.txt");

// ofstream MJ_graph("/home/dyros_rm/MJ/data/myeongju/MJ_graph.txt");
// ofstream MJ_graph1("/home/dyros_rm/MJ/data/myeongju/MJ_graph1.txt");
// ofstream MJ_joint1("/home/dyros_rm/MJ/data/myeongju/MJ_joint1.txt");
// ofstream MJ_joint2("/home/dyros_rm/MJ/data/myeongju/MJ_joint2.txt");

AvatarController::AvatarController(RobotData &rd) : rd_(rd)
{
    nh_avatar_.setCallbackQueue(&queue_avatar_);

    upperbodymode_sub = nh_avatar_.subscribe("/tocabi/avatar/upperbodymodecommand", 100, &AvatarController::UpperbodyModeCallback, this);

    arm_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/armpdgain", 100, &AvatarController::ArmJointGainCallback, this);
    waist_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/waistpdgain", 100, &AvatarController::WaistJointGainCallback, this);

    // hmd_posture_sub = nh_avatar_.subscribe("/HMD", 100, &AvatarController::HmdCallback, this);

    // pelvis_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER0", 100, &AvatarController::PelvisTrackerCallback, this);
    // chest_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER1", 100, &AvatarController::ChestTrackerCallback, this);
    // lelbow_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER2", 100, &AvatarController::LeftElbowTrackerCallback, this);
    // lhand_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER3", 100, &AvatarController::LeftHandTrackerCallback, this);
    // relbow_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER4", 100, &AvatarController::RightElbowTrackerCallback, this);
    // rhand_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER5", 100, &AvatarController::RightHandTrackerCallback, this);
    
    //subscribers
    tracker_status_sub = nh_avatar_.subscribe("/TRACKERSTATUS", 100, &AvatarController::TrackerStatusCallback, this);
    tracker_pose_sub = nh_avatar_.subscribe("/tracker_pose", 1, &AvatarController::TrackerPoseCallback, this, ros::TransportHints().tcpNoDelay(true));
    master_pose_sub = nh_avatar_.subscribe("/tocabi/haptic_poses", 1, &AvatarController::MasterPoseCallback, this, ros::TransportHints().tcpNoDelay(true));

    vive_tracker_pose_calibration_sub = nh_avatar_.subscribe("/tocabi/avatar/pose_calibration_flag", 100, &AvatarController::PoseCalibrationCallback, this);

    // pedal_command = nh_avatar_.subscribe("/tocabi/pedalcommand", 100, &AvatarController::PedalCommandCallback, this); // MJ

    robot_hand_pos_mapping_scale_sub = nh_avatar_.subscribe("/tocabi/avatar/hand_pos_mapping_sclae", 100, &AvatarController::HandPosMappingScaleCallback, this);

    //publishers
    calibration_state_pub = nh_avatar_.advertise<std_msgs::String>("/tocabi_status", 5);
    calibration_state_gui_log_pub = nh_avatar_.advertise<std_msgs::String>("/tocabi/guilog", 100);
    upperbodymode_pub = nh_avatar_.advertise<std_msgs::Int8>("/tocabi/avatar/upperbodymodecurrent", 5); 
    avatar_warning_pub = nh_avatar_.advertise<std_msgs::Int8>("/tocabi/avatar/warningmsg", 5); 

    haptic_force_pub = nh_avatar_.advertise<std_msgs::Float32MultiArray>("/tocabi/hand_ftsensors", 5);

    bool urdfmode = false;
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    // if (urdfmode)
    // {
    //     urdf_path = desc_package_path + "/dyros_tocabi_ankleRollDamping.urdf";
    // }
    // else
    // {
    //     urdf_path = desc_package_path + "/dyros_tocabi.urdf";
    // }

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_d_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_C_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_global_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_local_, true, false);

    for (int i = 0; i < FILE_CNT; i++)
    {
        file[i].open(FILE_NAMES[i]);
    }

    setGains();
    setNeuralNetworks();
    first_loop_larm_ = true;
    first_loop_rarm_ = true;
    first_loop_upperbody_ = true;
    first_loop_hqpik_ = true;
    first_loop_hqpik2_ = true;
    first_loop_qp_retargeting_ = true;
}

void AvatarController::setGains()
{

    // support_foot_damping_gain_.setZero();
    // support_foot_damping_gain_(0, 0) = 0.01;
    // support_foot_damping_gain_(1, 1) = 0.01;
    // support_foot_damping_gain_(2, 2) = 0;

    ////////////////////////////////////////////////////

    /////////Torque Limit///////////
    torque_task_min_(0) = -300;
    torque_task_min_(1) = -300;
    torque_task_min_(2) = -300;
    torque_task_min_(3) = -300;
    torque_task_min_(4) = -300;
    torque_task_min_(5) = -300;

    torque_task_min_(6) = -300;
    torque_task_min_(7) = -300;
    torque_task_min_(8) = -300;
    torque_task_min_(9) = -300;
    torque_task_min_(10) = -300;
    torque_task_min_(11) = -300;

    torque_task_min_(12) = -300;
    torque_task_min_(13) = -300;
    torque_task_min_(14) = -300;

    torque_task_min_(15) = -300;
    torque_task_min_(16) = -300;
    torque_task_min_(17) = -300;
    torque_task_min_(18) = -300;
    torque_task_min_(19) = -300;
    torque_task_min_(20) = -300;
    torque_task_min_(21) = -100;
    torque_task_min_(22) = -100;

    torque_task_min_(23) = -100;
    torque_task_min_(24) = -100;

    torque_task_min_(25) = -300;
    torque_task_min_(26) = -300;
    torque_task_min_(27) = -300;
    torque_task_min_(28) = -300;
    torque_task_min_(29) = -300;
    torque_task_min_(30) = -300;
    torque_task_min_(31) = -100;
    torque_task_min_(32) = -100;

    torque_task_max_(0) = 300;
    torque_task_max_(1) = 300;
    torque_task_max_(2) = 300;
    torque_task_max_(3) = 300;
    torque_task_max_(4) = 300;
    torque_task_max_(5) = 300;

    torque_task_max_(6) = 300;
    torque_task_max_(7) = 300;
    torque_task_max_(8) = 300;
    torque_task_max_(9) = 300;
    torque_task_max_(10) = 300;
    torque_task_max_(11) = 300;

    torque_task_max_(12) = 300;
    torque_task_max_(13) = 300;
    torque_task_max_(14) = 300;

    torque_task_max_(15) = 100;
    torque_task_max_(16) = 300;
    torque_task_max_(17) = 300;
    torque_task_max_(18) = 300;
    torque_task_max_(19) = 300;
    torque_task_max_(20) = 300;
    torque_task_max_(21) = 100;
    torque_task_max_(22) = 100;

    torque_task_max_(23) = 100;
    torque_task_max_(24) = 100;

    torque_task_max_(25) = 100;
    torque_task_max_(26) = 300;
    torque_task_max_(27) = 300;
    torque_task_max_(28) = 300;
    torque_task_max_(29) = 300;
    torque_task_max_(30) = 300;
    torque_task_max_(31) = 100;
    torque_task_max_(32) = 100;
    ////////////////////////////////

    //////////Joint PD Gain/////////
    /// For Simulation
    // for (int i = 0; i < MODEL_DOF; i++)
    // {
    // 	kp_joint_(i) = 100; 		//(tune)
    // 	kv_joint_(i) = 20;		//(tune)
    // }

    // //Waist Joint Gains
    // for (int i = 0; i < 3; i++)
    // {
    // 	kp_joint_(12 + i) = 900;
    // 	kv_joint_(12 + i) = 60;
    // }
    // kp_joint_(12) = 2500;
    // kp_joint_(13) = 900;
    // kp_joint_(14) = 900;

    // kv_joint_(12) = 100;
    // kv_joint_(13) = 60;
    // kv_joint_(14) = 60;

    // kp_joint_(20) = 64;	//forearm
    // kp_joint_(21) = 64;	//wrist1
    // kp_joint_(22) = 64;	//wrist2
    // kv_joint_(20) = 10;
    // kv_joint_(21) = 10;
    // kv_joint_(22) = 10;

    // kp_joint_(30) = 64;
    // kp_joint_(31) = 64;
    // kp_joint_(32) = 64;
    // kv_joint_(30) = 10;
    // kv_joint_(31) = 10;
    // kv_joint_(32) = 10;

    // kp_joint_(23) = 49;	//head
    // kp_joint_(24) = 49;
    // kv_joint_(23) = 14;	//head
    // kv_joint_(24) = 14;

    // //stiff	//(tune)
    // kp_stiff_joint_(0) = 3600; //R hip yaw joint gain
    // kv_stiff_joint_(0) = 120;
    // kp_stiff_joint_(1) = 4900; //L hip roll joint gain
    // kv_stiff_joint_(1) = 140;
    // kp_stiff_joint_(2) = 4900; //L hip pitch joint gain
    // kv_stiff_joint_(2) = 140;

    // kp_stiff_joint_(3) = 1600; //L knee joint gain
    // kv_stiff_joint_(3) = 80;

    // kp_stiff_joint_(4) = 400; //L ankle pitch joint gain
    // kv_stiff_joint_(4) = 40;
    // kp_stiff_joint_(5) = 400; //L ankle roll joint gain
    // kv_stiff_joint_(5) = 40;

    // kp_stiff_joint_(6) = 3600; //R hip yaw joint gain
    // kv_stiff_joint_(6) = 120;
    // kp_stiff_joint_(7) = 4900; //R hip roll joint gain
    // kv_stiff_joint_(7) = 140;
    // kp_stiff_joint_(8) = 4900; //R hip pitch joint gain
    // kv_stiff_joint_(8) = 140;

    // kp_stiff_joint_(9) = 1600; //R knee joint gain
    // kv_stiff_joint_(9) = 80;

    // kp_stiff_joint_(10) = 400; //R ankle pitch joint gain
    // kv_stiff_joint_(10) = 40;
    // kp_stiff_joint_(11) = 400; //R ankle roll joint gain
    // kv_stiff_joint_(11) = 40;

    // //soft	//(tune)
    // kp_soft_joint_(0) = 3600; //L hip yaw joint gain
    // kv_soft_joint_(0) = 120;
    // kp_soft_joint_(1) = 400; //L hip roll joint gain
    // kv_soft_joint_(1) = 40;
    // kp_soft_joint_(2) = 400; //L hip pitch joint gain
    // kv_soft_joint_(2) = 40;

    // kp_soft_joint_(3) = 100; //L knee joint gain
    // kv_soft_joint_(3) = 20;

    // kp_soft_joint_(4) = 25; //L ankle pitch joint gain
    // kv_soft_joint_(4) = 10;
    // kp_soft_joint_(5) = 25; //L ankle roll joint gain
    // kv_soft_joint_(5) = 10;

    // kp_soft_joint_(6) = 3600; //R hip yaw joint gain
    // kv_soft_joint_(6) = 120;
    // kp_soft_joint_(7) = 400; //R hip roll joint gain
    // kv_soft_joint_(7) = 40;
    // kp_soft_joint_(8) = 400; //R hip pitch joint gain
    // kv_soft_joint_(8) = 40;

    // kp_soft_joint_(9) = 100; //R knee joint gain
    // kv_soft_joint_(9) = 20;

    // kp_soft_joint_(10) = 25; //R ankle pitch joint gain
    // kv_soft_joint_(10) = 10;
    // kp_soft_joint_(11) = 25; //R ankle roll joint gain
    // kv_soft_joint_(11) = 10;

    // for (int i = 0; i < 12; i++) //Leg
    // {
    // 	kp_joint_(i) = kp_stiff_joint_(i);
    // 	kv_joint_(i) = kv_stiff_joint_(i);
    // }
    /////////////////

    /// For Real Robot
    kp_stiff_joint_(0) = 2000; // right leg
    kp_stiff_joint_(1) = 5000;
    kp_stiff_joint_(2) = 4000;
    kp_stiff_joint_(3) = 3700;
    kp_stiff_joint_(4) = 5000;
    kp_stiff_joint_(5) = 5000;
    kp_stiff_joint_(6) = 2000; // left leg
    kp_stiff_joint_(7) = 5000;
    kp_stiff_joint_(8) = 4000;
    kp_stiff_joint_(9) = 3700;
    kp_stiff_joint_(10) = 5000;
    kp_stiff_joint_(11) = 5000;
    kp_stiff_joint_(12) = 6000; // waist
    kp_stiff_joint_(13) = 10000;
    kp_stiff_joint_(14) = 10000;
    kp_stiff_joint_(15) = 200; // left arm
    kp_stiff_joint_(16) = 400;
    kp_stiff_joint_(17) = 200;
    kp_stiff_joint_(18) = 200;
    kp_stiff_joint_(19) = 125;
    kp_stiff_joint_(20) = 125;
    kp_stiff_joint_(21) = 25;
    kp_stiff_joint_(22) = 25;
    kp_stiff_joint_(23) = 50; // head
    kp_stiff_joint_(24) = 50;
    kp_stiff_joint_(25) = 200; // right arm
    kp_stiff_joint_(26) = 400;
    kp_stiff_joint_(27) = 200;
    kp_stiff_joint_(28) = 200;
    kp_stiff_joint_(29) = 125;
    kp_stiff_joint_(30) = 125;
    kp_stiff_joint_(31) = 25;
    kp_stiff_joint_(32) = 25;

    kv_stiff_joint_(0) = 15; // right leg
    kv_stiff_joint_(1) = 50;
    kv_stiff_joint_(2) = 20;
    kv_stiff_joint_(3) = 25;
    kv_stiff_joint_(4) = 30;
    kv_stiff_joint_(5) = 30;
    kv_stiff_joint_(6) = 15; // left leg
    kv_stiff_joint_(7) = 50;
    kv_stiff_joint_(8) = 20;
    kv_stiff_joint_(9) = 25;
    kv_stiff_joint_(10) = 30;
    kv_stiff_joint_(11) = 30;
    kv_stiff_joint_(12) = 200; // waist
    kv_stiff_joint_(13) = 100;
    kv_stiff_joint_(14) = 100;
    kv_stiff_joint_(15) = 7; // left arm
    kv_stiff_joint_(16) = 5;
    kv_stiff_joint_(17) = 2.5;
    kv_stiff_joint_(18) = 2.5;
    kv_stiff_joint_(19) = 2.5;
    kv_stiff_joint_(20) = 2;
    kv_stiff_joint_(21) = 2;
    kv_stiff_joint_(22) = 2;
    kv_stiff_joint_(23) = 2; // head
    kv_stiff_joint_(24) = 2;
    kv_stiff_joint_(25) = 7; // right arm
    kv_stiff_joint_(26) = 5;
    kv_stiff_joint_(27) = 2.5;
    kv_stiff_joint_(28) = 2.5;
    kv_stiff_joint_(29) = 2.5;
    kv_stiff_joint_(30) = 2;
    kv_stiff_joint_(31) = 2;
    kv_stiff_joint_(32) = 2;

    kp_soft_joint_(0) = 2000; // right leg
    kp_soft_joint_(1) = 5000;
    kp_soft_joint_(2) = 4000;
    kp_soft_joint_(3) = 3700;
    kp_soft_joint_(4) = 5000;
    kp_soft_joint_(5) = 5000;
    kp_soft_joint_(6) = 2000; // left leg
    kp_soft_joint_(7) = 5000;
    kp_soft_joint_(8) = 4000;
    kp_soft_joint_(9) = 3700;
    kp_soft_joint_(10) = 5000;
    kp_soft_joint_(11) = 5000;
    kp_soft_joint_(12) = 6000; // waist
    kp_soft_joint_(13) = 10000;
    kp_soft_joint_(14) = 10000;
    kp_soft_joint_(15) = 200; // left arm
    kp_soft_joint_(16) = 80;
    kp_soft_joint_(17) = 60;
    kp_soft_joint_(18) = 60;
    kp_soft_joint_(19) = 60;
    kp_soft_joint_(20) = 60;
    kp_soft_joint_(21) = 20;
    kp_soft_joint_(22) = 20;
    kp_soft_joint_(23) = 50; // head
    kp_soft_joint_(24) = 50;
    kp_soft_joint_(25) = 200; // right arm
    kp_soft_joint_(26) = 80;
    kp_soft_joint_(27) = 60;
    kp_soft_joint_(28) = 60;
    kp_soft_joint_(29) = 60;
    kp_soft_joint_(30) = 60;
    kp_soft_joint_(31) = 20;
    kp_soft_joint_(32) = 20;

    kv_soft_joint_(0) = 15; // right leg
    kv_soft_joint_(1) = 50;
    kv_soft_joint_(2) = 20;
    kv_soft_joint_(3) = 25;
    kv_soft_joint_(4) = 30;
    kv_soft_joint_(5) = 30;
    kv_soft_joint_(6) = 15; // left leg
    kv_soft_joint_(7) = 50;
    kv_soft_joint_(8) = 20;
    kv_soft_joint_(9) = 25;
    kv_soft_joint_(10) = 30;
    kv_soft_joint_(11) = 30;
    kv_soft_joint_(12) = 200; // waist
    kv_soft_joint_(13) = 100;
    kv_soft_joint_(14) = 100;
    kv_soft_joint_(15) = 7; // left arm
    kv_soft_joint_(16) = 5;
    kv_soft_joint_(17) = 2.5;
    kv_soft_joint_(18) = 2.5;
    kv_soft_joint_(19) = 2.5;
    kv_soft_joint_(20) = 2;
    kv_soft_joint_(21) = 2;
    kv_soft_joint_(22) = 2;
    kv_soft_joint_(23) = 2; // head
    kv_soft_joint_(24) = 2;
    kv_soft_joint_(25) = 7; // right arm
    kv_soft_joint_(26) = 5;
    kv_soft_joint_(27) = 2.5;
    kv_soft_joint_(28) = 2.5;
    kv_soft_joint_(29) = 2.5;
    kv_soft_joint_(30) = 2;
    kv_soft_joint_(31) = 2;
    kv_soft_joint_(32) = 2;
    // for (int i = 0; i < MODEL_DOF; i++)
    // {
    //     kp_soft_joint_(i) = kp_stiff_joint_(i) / 4;
    //     kp_soft_joint_(i) = kv_stiff_joint_(i) / 2;
    // }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        kp_joint_(i) = kp_stiff_joint_(i);
        kv_joint_(i) = kv_stiff_joint_(i);
    }
    ///////////////

    ///////////////////////////////

    // arm controller
    joint_limit_l_.resize(33);
    joint_limit_h_.resize(33);
    joint_vel_limit_l_.resize(33);
    joint_vel_limit_h_.resize(33);

    // LEG
    for (int i = 0; i < 12; i++)
    {
        joint_limit_l_(i) = -180 * DEG2RAD;
        joint_limit_h_(i) = 180 * DEG2RAD;
    }

    // WAIST
    joint_limit_l_(12) = -30 * DEG2RAD;
    joint_limit_h_(12) = 30 * DEG2RAD;
    joint_limit_l_(13) = -15 * DEG2RAD;
    joint_limit_h_(13) = 30 * DEG2RAD;
    joint_limit_l_(14) = -15 * DEG2RAD;
    joint_limit_h_(14) = 15 * DEG2RAD;
    // LEFT ARM
    joint_limit_l_(15) = -30 * DEG2RAD;
    joint_limit_h_(15) = 30 * DEG2RAD;
    joint_limit_l_(16) = -160 * DEG2RAD;
    joint_limit_h_(16) = 90 * DEG2RAD;
    joint_limit_l_(17) = -95 * DEG2RAD;
    joint_limit_h_(17) = 95 * DEG2RAD;
    joint_limit_l_(18) = -180 * DEG2RAD;
    joint_limit_h_(18) = 180 * DEG2RAD;
    joint_limit_l_(19) = -150 * DEG2RAD;
    joint_limit_h_(19) = -15 * DEG2RAD;
    joint_limit_l_(20) = -180 * DEG2RAD;
    joint_limit_h_(20) = 180 * DEG2RAD;
    joint_limit_l_(21) = -70 * DEG2RAD;
    joint_limit_h_(21) = 70 * DEG2RAD;
    joint_limit_l_(22) = -60 * DEG2RAD;
    joint_limit_h_(22) = 60 * DEG2RAD;
    // HEAD
    joint_limit_l_(23) = -80 * DEG2RAD;
    joint_limit_h_(23) = 80 * DEG2RAD;
    joint_limit_l_(24) = -40 * DEG2RAD;
    joint_limit_h_(24) = 18 * DEG2RAD;
    // RIGHT ARM
    joint_limit_l_(25) = -30 * DEG2RAD;
    joint_limit_h_(25) = 30 * DEG2RAD;
    joint_limit_l_(26) = -90 * DEG2RAD;
    joint_limit_h_(26) = 160 * DEG2RAD;
    joint_limit_l_(27) = -95 * DEG2RAD;
    joint_limit_h_(27) = 95 * DEG2RAD;
    joint_limit_l_(28) = -180 * DEG2RAD;
    joint_limit_h_(28) = 180 * DEG2RAD;
    joint_limit_l_(29) = 15 * DEG2RAD;
    joint_limit_h_(29) = 150 * DEG2RAD;
    joint_limit_l_(30) = -180 * DEG2RAD;
    joint_limit_h_(30) = 180 * DEG2RAD;
    joint_limit_l_(31) = -70 * DEG2RAD;
    joint_limit_h_(31) = 70 * DEG2RAD;
    joint_limit_l_(32) = -60 * DEG2RAD;
    joint_limit_h_(32) = 60 * DEG2RAD;

    // LEG
    for (int i = 0; i < 12; i++)
    {
        joint_vel_limit_l_(i) = -2 * M_PI;
        joint_vel_limit_h_(i) = 2 * M_PI;
    }

    // UPPERBODY
    for (int i = 12; i < 33; i++)
    {
        joint_vel_limit_l_(i) = -M_PI;
        joint_vel_limit_h_(i) = M_PI;
    }

    // 1st arm joint vel limit
    joint_vel_limit_l_(15) = -M_PI / 3;
    joint_vel_limit_h_(15) = M_PI / 3;

    joint_vel_limit_l_(25) = -M_PI / 3;
    joint_vel_limit_h_(25) = M_PI / 3;

    // Head joint vel limit
    joint_vel_limit_l_(23) = -2 * M_PI;
    joint_vel_limit_h_(23) = 2 * M_PI;
    joint_vel_limit_l_(24) = -2 * M_PI;
    joint_vel_limit_h_(24) = 2 * M_PI;

    // forearm joint vel limit
    joint_vel_limit_l_(20) = -2 * M_PI;
    joint_vel_limit_h_(20) = 2 * M_PI;
    joint_vel_limit_l_(30) = -2 * M_PI;
    joint_vel_limit_h_(30) = 2 * M_PI;
}
void AvatarController::setNeuralNetworks()
{
    ///// Between Left Arm and Upperbody & Head Collision Detection Network /////
    Eigen::VectorXd n_hidden, q_to_input_mapping_vector;
    n_hidden.resize(6);
    q_to_input_mapping_vector.resize(13);
    n_hidden << 120, 100, 80, 60, 40, 20;
    q_to_input_mapping_vector << 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24;
    initializeScaMlp(larm_upperbody_sca_mlp_, 13, 2, n_hidden, q_to_input_mapping_vector);
    loadScaNetwork(larm_upperbody_sca_mlp_, "/home/dyros/catkin_ws/src/tocabi_avatar/sca_mlp/larm_upperbody/");
    //////////////////////////////////////////////////////////////////////////////

    ///// Between Right Arm and Upperbody & Head Collision Detection Network /////
    n_hidden << 120, 100, 80, 60, 40, 20;
    q_to_input_mapping_vector << 12, 13, 14, 25, 26, 27, 28, 29, 30, 31, 32, 23, 24;
    initializeScaMlp(rarm_upperbody_sca_mlp_, 13, 2, n_hidden, q_to_input_mapping_vector);
    loadScaNetwork(rarm_upperbody_sca_mlp_, "/home/dyros/catkin_ws/src/tocabi_avatar/sca_mlp/rarm_upperbody/");
    //////////////////////////////////////////////////////////////////////////////

    ///// Between Arms Collision Detection Network /////
    // q_to_input_mapping_vector.resize(16);
    // n_hidden << 120, 100, 80, 60, 40, 20;
    // q_to_input_mapping_vector << 15, 16, 17, 18, 19, 20, 21, 22, 25, 26, 27, 28, 29, 30, 31, 32;
    // initializeScaMlp(btw_arms_sca_mlp_, 16, 2, n_hidden, q_to_input_mapping_vector);
    // loadScaNetwork(btw_arms_sca_mlp_, "/home/dyros/catkin_ws/src/tocabi_avatar/sca_mlp/btw_arms/");
    //////////////////////////////////////////////////////////////////////////////
}
Eigen::VectorQd AvatarController::getControl()
{
    return rd_.torque_desired;
}

void AvatarController::computeSlow()
{
    queue_avatar_.callAvailable(ros::WallDuration());

    if (rd_.tc_.mode == 10)
    {

    }
    else if (rd_.tc_.mode == 11)
    {

    }
    else if (rd_.tc_.mode == 12)
    {
        if (initial_flag == 0)
        {
            Joint_gain_set_MJ();
            walking_enable_ = false;
            ref_q_ = rd_.q_;

            for (int i = 0; i < 12; i++)
            {
                Initial_ref_q_(i) = ref_q_(i);
            }
            init_leg_time_ = (double)rd_.control_time_us_/ 1000000.0;
            desired_q_fast_ = rd_.q_;
            desired_q_dot_fast_.setZero();

            initial_flag = 1;
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            mode_12_count_ = 0;
            joy_input_enable_ = true;

            chair_mode_ = false; /// avatar semifinals //1025

            parameterSetting();
            initWalkingParameter();
            updateInitialStateJoy();
            cout << "mode = 12 : Pedal Init" << endl;
            cout << "chair_mode_: " << chair_mode_ << endl;
            WBC::SetContact(rd_, 1, 1);
            Gravity_MJ_ = WBC::GravityCompensationTorque(rd_);
            atb_grav_update_ = false;
        }

        if (atb_grav_update_ == false && (initial_flag == 2) )
        {
            atb_grav_update_ = true;
            Gravity_MJ_fast_ = Gravity_MJ_;
            atb_grav_update_ = false;
        }

        for (int i = 0; i < MODEL_DOF; i++)
        {
            rd_.torque_desired(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_(i);
        }

        // for chair mode
        if (chair_mode_)
        {
            for (int i = 0; i < 12; i++)
            {
                // rd_.torque_desired(i) = DyrosMath::cubic(rd_.control_time_, init_leg_time_, init_leg_time_+3.0, - Kd(i) * rd_.q_dot_(i), 0, 0, 0);
                rd_.torque_desired(i) = 0;
            }
        }

        mode_12_count_ ++;

        if( (initial_flag == 2) && (mode_12_count_ > 1000) )
        {
            rd_.tc_.mode = 13;
            rd_.tc_init = true;
            mode_12_count_ = 0;
        }
    }
    else if (rd_.tc_.mode == 13) //2KHZ STRICT
    {
        if (walking_end_flag == 0)
        {
            parameterSetting(); // Don't delete this!!
            updateInitialStateJoy();
            // updateInitialState();
            getRobotState();
            cout << "Control mode 13 is initilized" << endl;
            for (int i = 0; i < 12; i++)
            {
                Initial_ref_q_(i) = ref_q_(i);
                Initial_current_q_(i) = rd_.q_(i);
            }
            initial_flag = 0;
            init_leg_time_ = (double)rd_.control_time_us_ / 1000000.0;
            walking_end_flag = 1;

            if(rd_.avatar_reboot_signal)
            {
                rd_.avatar_reboot_signal = false;
                
                hmd_check_pose_calibration_[0] = true;
                hmd_check_pose_calibration_[1] = true;
                hmd_check_pose_calibration_[2] = true;
                hmd_check_pose_calibration_[4] = true;

                still_pose_cali_flag_ = true;
                t_pose_cali_flag_ = true;
                forward_pose_cali_flag_ = true;
                cout << "Reading Calibration Log File..." << endl;

                std_msgs::String msg;
                std::stringstream log_load;
                log_load << "Calibration Pose Data is Loaded";
                msg.data = log_load.str();
                calibration_state_pub.publish(msg);
                calibration_state_gui_log_pub.publish(msg);
    

                cout << "Calibration Status: [" << hmd_check_pose_calibration_[0] << ", " << hmd_check_pose_calibration_[1] << ", " << hmd_check_pose_calibration_[2] << "]" << endl;
            }
        }
        else
        {
            getRobotState();

            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_MJ_fast_ = Gravity_MJ_;
                atb_grav_update_ = false;
            }
        }

        torque_lower_.setZero();

        /////////////////////////////////////////////////////////////////////////////////////////

        if (atb_upper_update_ == false)
        {
            atb_upper_update_ = true;
            desired_q_fast_ = desired_q_slow_;
            desired_q_dot_fast_ = desired_q_dot_slow_;
            atb_upper_update_ = false;
        }

        torque_upper_.setZero();
        for (int i = 12; i < MODEL_DOF; i++)
        {
            torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - current_q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            // torque_upper_(i) = Gravity_MJ_fast_(i);
            rd_.q_desired(i) = desired_q_fast_(i);  // for logging
            rd_.q_dot_desired(i) = desired_q_dot_fast_(i);  // for logging
            // torque_upper_(i) = torque_upper_(i) * pd_control_mask_(i); // masking for joint pd control
        }

        // printOutTextFile();
        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = torque_lower_ + torque_upper_;
        ////////////////////////////////////////////////////////////////////////////////
    }
    else if (rd_.tc_.mode == 14)
    {
    }
}

void AvatarController::computeFast()
{
    if (rd_.tc_.mode == 10)
    {

    }
    else if (rd_.tc_.mode == 11)
    {

    }
    else if (rd_.tc_.mode == 12)
    {
        if (initial_flag == 1)
        {
            WBC::SetContact(rd_, 1, 1);
            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }

            cout << "comutefast tc.mode = 12 is initialized" << endl;
            initial_flag = 2;
        }
    }
    else if (rd_.tc_.mode == 13)
    {
        
        VectorQd Gravity_MJ_local;
        
        Gravity_MJ_local = floatGravityTorque(rd_.q_virtual_);

        if (atb_grav_update_ == false)
        {
            atb_grav_update_ = true;
            Gravity_MJ_ = Gravity_MJ_local;
            atb_grav_update_ = false;
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////

        if (rd_.tc_init == true)
        {
            initWalkingParameter();
            rd_.tc_init = false;
        }


        // data process//
        getRobotData(); // 47~64us
        getProcessedRobotData(); // <<1us
        // motion planing and control//

        // Self-Collision-Avoidance Network Inferences
        calculateScaMlpOutput(larm_upperbody_sca_mlp_);
        calculateScaMlpOutput(rarm_upperbody_sca_mlp_);
        // calculateScaMlpOutput(btw_arms_sca_mlp_);

        // avatar mode pedal
        avatarModeStateMachine();
        
        // if (current_q_(24) > 5 * DEG2RAD)
        // {
        //     if (abs(current_q_(23)) > 18 * DEG2RAD)
        //     {
        //         joint_limit_h_(24) = 10 * DEG2RAD;
        //         joint_limit_h_(23) = 80 * DEG2RAD;
        //         joint_limit_l_(23) = -80 * DEG2RAD;
        //     }
        //     else
        //     {
        //         joint_limit_h_(24) = 30 * DEG2RAD;
        //         joint_limit_h_(23) = 13 * DEG2RAD;
        //         joint_limit_l_(23) = -13 * DEG2RAD;
        //     }
        // }
        // else
        // {
        //     joint_limit_h_(24) = 10 * DEG2RAD;
        //     joint_limit_h_(23) = 80 * DEG2RAD;
        //     joint_limit_l_(23) = -80 * DEG2RAD;
        // }

        // if( abs(current_q_(23)) < 20*DEG2RAD)
        // {
        //     joint_limit_h_(24) = 35*DEG2RAD;
        //     joint_limit_l_(24) = -40*DEG2RAD;
        // }
        // else
        // {
        //     joint_limit_h_(24) = 10*DEG2RAD;
        //     joint_limit_l_(24) = -40*DEG2RAD;
        // }

        motionGenerator(); // 140~240us(HQPIK)



        for (int i = 12; i < MODEL_DOF; i++)
        {
            desired_q_(i) = motion_q_(i);
            desired_q_dot_(i) = motion_q_dot_(i);
            // desired_q_dot_(i) = 0;
        }

        if (atb_upper_update_ == false)
        {
            atb_upper_update_ = true;
            desired_q_slow_ = desired_q_;
            desired_q_dot_slow_ = desired_q_dot_;
            atb_upper_update_ = false;
        }

        savePreData();

        // printOutTextFile();
    }
    else if (rd_.tc_.mode == 14)
    {

    }
}

void AvatarController::computeThread3()
{
    
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AvatarController::initWalkingParameter()
{
    walking_mode_on_ = true;
    upper_body_mode_ = 3;
    upper_body_mode_raw_ = 3;

    upperbody_mode_recieved_ = true;



    jac_rhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_rfoot_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lfoot_.setZero(6, MODEL_DOF_VIRTUAL);

    // set init pre data


    pre_time_ = rd_.control_time_ - 0.001;
    pre_desired_q_ = rd_.q_;
    last_desired_q_ = rd_.q_;
    pre_desired_q_dot_.setZero();

    init_q_ = rd_.q_;
    zero_q_ = init_q_;
    desired_q_ = init_q_;
    desired_q_dot_.setZero();
    desired_q_ddot_.setZero();

    A_mat_ = rd_.A_;
    A_mat_pre_ = rd_.A_;

    mob_residual_wholebody_.setZero(MODEL_DOF_VIRTUAL);
    mob_integral_wholebody_.setZero(MODEL_DOF_VIRTUAL);

    mob_integral_.setZero();
    mob_residual_.setZero();

    lh_ft_.setZero();
    rh_ft_.setZero();
    lh_ft_wo_hw_.setZero();
    rh_ft_wo_hw_.setZero();
    lh_ft_wo_hw_lpf_.setZero();
    rh_ft_wo_hw_lpf_.setZero();

    lh_ft_wo_hw_global_.setZero();
    rh_ft_wo_hw_global_.setZero();

    torque_from_lh_ft_.setZero();
    torque_from_rh_ft_.setZero();
    torque_from_lh_ft_lpf_.setZero();
    torque_from_rh_ft_lpf_.setZero();

    motion_q_pre_ = init_q_;
    motion_q_dot_pre_.setZero();


    lhand_control_point_offset_.setZero();
    rhand_control_point_offset_.setZero();
    // lhand_control_point_offset_(2) = -0.13;
    // rhand_control_point_offset_(2) = -0.13;
    lhand_control_point_offset_(2) = -0.13; // without FT
    rhand_control_point_offset_(2) = -0.15; // with FT

    lfoot_ft_sensor_offset_.setZero();
    rfoot_ft_sensor_offset_.setZero();
    lhand_ft_sensor_offset_.setZero();
    rhand_ft_sensor_offset_.setZero();

    lfoot_ft_sensor_offset_(2) = -0.09;
    rfoot_ft_sensor_offset_(2) = -0.09;

    lhand_ft_sensor_offset_(2) = -0.035;
    rhand_ft_sensor_offset_(2) = -0.035;

    robot_shoulder_width_ = 0.6;

    robot_upperarm_max_l_ = 0.3376 * 1.0;
    robot_lowerarm_max_l_ = 0.31967530867;
    // robot_arm_max_l_ = 0.98*sqrt(robot_upperarm_max_l_*robot_upperarm_max_l_ + robot_lowerarm_max_l_*robot_lowerarm_max_l_ + 2*robot_upperarm_max_l_*robot_lowerarm_max_l_*cos( -joint_limit_h_(19)) );
    robot_arm_max_l_ = (robot_upperarm_max_l_ + robot_lowerarm_max_l_) * 0.95 + lhand_control_point_offset_.norm();

    hmd_check_pose_calibration_[0] = false;
    hmd_check_pose_calibration_[1] = false;
    hmd_check_pose_calibration_[2] = false;
    hmd_check_pose_calibration_[3] = false;
    hmd_check_pose_calibration_[4] = false;

    still_pose_cali_flag_ = false;
    t_pose_cali_flag_ = false;
    forward_pose_cali_flag_ = false;
    read_cali_log_flag_ = false;

    hmd_larm_max_l_ = 0.45;
    hmd_rarm_max_l_ = 0.45;
    hmd_shoulder_width_ = 0.5;

    hmd_pelv_pose_.setIdentity();
    hmd_lshoulder_pose_.setIdentity();
    hmd_lhand_pose_.setIdentity();
    hmd_rshoulder_pose_.setIdentity();
    hmd_rupperarm_pose_.setIdentity();
    hmd_rhand_pose_.setIdentity();
    hmd_chest_pose_.setIdentity();

    hmd_pelv_pose_raw_.setIdentity();
    hmd_lshoulder_pose_raw_.setIdentity();
    hmd_lhand_pose_raw_.setIdentity();
    hmd_rshoulder_pose_raw_.setIdentity();
    hmd_rupperarm_pose_raw_.setIdentity();
    hmd_rhand_pose_raw_.setIdentity();
    hmd_chest_pose_raw_.setIdentity();

    hmd_head_pose_raw_last_.setIdentity();
    hmd_pelv_pose_raw_last_.setIdentity();
    hmd_lshoulder_pose_raw_last_.setIdentity();
    hmd_lupperarm_pose_raw_last_.setIdentity();
    hmd_lhand_pose_raw_last_.setIdentity();
    hmd_rshoulder_pose_raw_last_.setIdentity();
    hmd_rupperarm_pose_raw_last_.setIdentity();
    hmd_rhand_pose_raw_last_.setIdentity();
    hmd_chest_pose_raw_last_.setIdentity();
    

    hmd_head_pose_pre_.setIdentity();
    hmd_lshoulder_pose_pre_.setIdentity();
    hmd_lupperarm_pose_pre_.setIdentity();
    hmd_lhand_pose_pre_.setIdentity();
    hmd_rshoulder_pose_pre_.setIdentity();
    hmd_rupperarm_pose_pre_.setIdentity();
    hmd_rhand_pose_pre_.setIdentity();
    hmd_chest_pose_pre_.setIdentity();
    hmd_pelv_pose_pre_.setIdentity();

    hmd_pelv_pose_init_.setIdentity();
    tracker_status_changed_time_ = current_time_;
    hmd_tracker_status_ = false;
    hmd_tracker_status_raw_ = false;
    hmd_tracker_status_pre_ = false;

    // hmd_tracker_status_ = true;
    // hmd_tracker_status_raw_ = true;f
    // hmd_tracker_status_pre_ = true;

    hmd_head_abrupt_motion_count_ = 0;
    hmd_lupperarm_abrupt_motion_count_ = 0;
    hmd_lhand_abrupt_motion_count_ = 0;
    hmd_rupperarm_abrupt_motion_count_ = 0;
    hmd_rhand_abrupt_motion_count_ = 0;
    hmd_chest_abrupt_motion_count_ = 0;
    hmd_pelv_abrupt_motion_count_ = 0;

    last_solved_hierarchy_num_ = hierarchy_num_hqpik_ - 1;
}

void AvatarController::getRobotData()
{
    current_time_ = (double)rd_.control_time_us_ / 1000000.0;

    if (current_time_ != pre_time_)
    {
        dt_ = current_time_ - pre_time_;
    }

    if(dt_ < 0)
    {
        // cout<< cred << "WARNING: 'dt' is negative in thread2: "<< dt_ << creset << endl;
        current_time_ = pre_time_;
    }
    else if(dt_ > 0.002)
    {
        // cout<< cred <<"WARNING: 'dt' is too large in thread2: "<< dt_<< creset << endl;
    }

    dt_ = DyrosMath::minmax_cut(dt_, 0.0005, 0.002);

    current_q_ = __q_virtual.segment(6,MODEL_DOF);
    current_q_dot_ = __q_dot_virtual.segment(6,MODEL_DOF);
    current_q_ddot_ = __q_ddot_virtual.segment(6, MODEL_DOF);
    pelv_pos_current_ = link_avatar_[Pelvis].xpos;
    pelv_vel_current_.segment(0, 3) = link_avatar_[Pelvis].v;
    pelv_vel_current_.segment(3, 3) = link_avatar_[Pelvis].w;

    pelv_rot_current_ = link_avatar_[Pelvis].rotm;
    pelv_rpy_current_ = DyrosMath::rot2Euler(pelv_rot_current_); // ZYX multiply
    // pelv_rpy_current_ = (pelv_rot_current_).eulerAngles(2, 1, 0);
    // pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    pelv_yaw_rot_current_from_global_ = pelv_rot_current_;
    pelv_rot_current_yaw_aline_ = pelv_yaw_rot_current_from_global_.transpose() * pelv_rot_current_;
    // pelv_pos_current_ = pelv_yaw_rot_current_from_global_.transpose() * pelv_pos_current_;

    pelv_transform_current_from_global_.translation().setZero();
    pelv_transform_current_from_global_.linear() = pelv_rot_current_yaw_aline_;

    pelv_angvel_current_ = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Pelvis].w;

    com_mass_ = link_avatar_[COM_id].mass;

    /////////////////////////Feet Transformation and Velocity/////////////////////
    lfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Foot].xpos - pelv_pos_current_);
    lfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Foot].rotm;
    rfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Foot].xpos - pelv_pos_current_);
    rfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Foot].rotm;

    lfoot_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Foot].v;
    lfoot_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Foot].w;
    rfoot_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Foot].v;
    rfoot_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Foot].w;
    ///////////////////////////////////////////////////////////////////////////////

    ////////////////////////Knee Trnasformation and Velocity///////////////////////
    lknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Foot - 2].xpos - pelv_pos_current_);
    lknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Foot - 2].rotm;
    rknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Foot - 2].xpos - pelv_pos_current_);
    rknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Foot - 2].rotm;
    ///////////////////////////////////////////////////////////////////////////////

    ////////////////////////Hand Trnasformation and Velocity///////////////////////
    lhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Hand].xpos - pelv_pos_current_);
    lhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand].rotm;
    rhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Hand].xpos - pelv_pos_current_);
    rhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand].rotm;

    lhand_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand].v;
    lhand_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand].w;
    rhand_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand].v;
    rhand_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand].w;
    ///////////////////////////////////////////////////////////////////////////////

    ////////////////////////Elbow Trnasformation and Velocity///////////////////////
    lelbow_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Hand - 3].xpos - pelv_pos_current_);
    lelbow_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 3].rotm;
    relbow_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Hand - 3].xpos - pelv_pos_current_);
    relbow_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 3].rotm;

    lelbow_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 3].v;
    lelbow_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 3].w;
    relbow_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 3].v;
    relbow_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 3].w;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Upper Arm Trnasformation and Velocity////////////////////
    lupperarm_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Hand - 4].xpos - pelv_pos_current_);
    lupperarm_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 4].rotm;
    rupperarm_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Hand - 4].xpos - pelv_pos_current_);
    rupperarm_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 4].rotm;

    lupperarm_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 4].v;
    lupperarm_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 4].w;
    rupperarm_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 4].v;
    rupperarm_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 4].w;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Shoulder Trnasformation and Velocity////////////////////
    lshoulder_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Hand - 5].xpos - pelv_pos_current_);
    lshoulder_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 5].rotm;
    rshoulder_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Hand - 5].xpos - pelv_pos_current_);
    rshoulder_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 5].rotm;

    lshoulder_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 5].v;
    lshoulder_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 5].w;
    rshoulder_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 5].v;
    rshoulder_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 5].w;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Acromion Trnasformation and Velocity////////////////////
    lacromion_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Hand - 6].xpos - pelv_pos_current_);
    lacromion_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 6].rotm;
    racromion_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Hand - 6].xpos - pelv_pos_current_);
    racromion_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 6].rotm;

    lacromion_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 6].v;
    lacromion_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 6].w;
    racromion_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 6].v;
    racromion_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 6].w;
    ////////////////////////////////////////////////////////////////////////////////

    ///////////////////////Armbase Trasformation and ///////////////////////////////
    larmbase_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Left_Hand - 7].xpos - pelv_pos_current_);
    larmbase_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Left_Hand - 7].rotm;
    rarmbase_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Right_Hand - 7].xpos - pelv_pos_current_);
    rarmbase_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Right_Hand - 7].rotm;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Head & Upperbody Trnasformation ////////////////////////
    head_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Head].xpos - pelv_pos_current_);
    head_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Head].rotm;

    upperbody_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (link_avatar_[Upper_Body].xpos - pelv_pos_current_);
    upperbody_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[Upper_Body].rotm;
    ////////////////////////////////////////////////////////////////////////////////

    ///////////////////////Rotation Euler Angles////////////////////////////////////
    lhand_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lhand_transform_current_from_global_.linear());
    rhand_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(rhand_transform_current_from_global_.linear());
    lelbow_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lelbow_transform_current_from_global_.linear());
    relbow_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(relbow_transform_current_from_global_.linear());
    lupperarm_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lshoulder_transform_current_from_global_.linear());
    rupperarm_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(rupperarm_transform_current_from_global_.linear());
    lshoulder_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lupperarm_transform_current_from_global_.linear());
    rshoulder_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(rshoulder_transform_current_from_global_.linear());
    lacromion_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lacromion_transform_current_from_global_.linear());
    racromion_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(racromion_transform_current_from_global_.linear());
    ////////////////////////////////////////////////////////////////////////////////

    ///////////////////////Variables Updated from desired joint position////////////////////////////
    pre_desired_q_qvqd_;
    Quaterniond q(pelv_rot_current_yaw_aline_); // conversion error

    pre_desired_q_qvqd_.setZero();
    // pre_desired_q_qvqd_(3) = q.x(); 	//x y z
    // pre_desired_q_qvqd_(4) = q.y(); 	//x y z
    // pre_desired_q_qvqd_(5) = q.z(); 	//x y z
    // pre_desired_q_qvqd_(39) = q.w();						//w
    pre_desired_q_qvqd_(39) = 1;
    pre_desired_q_qvqd_.segment(6, MODEL_DOF) = pre_desired_q_;

    pre_desired_q_dot_vqd_.setZero();
    pre_desired_q_dot_vqd_.segment(0, 6) = pelv_vel_current_;
    pre_desired_q_dot_vqd_.segment(6, MODEL_DOF) = pre_desired_q_dot_;

    pre_desired_q_ddot_vqd_.setZero();

    VectorXd q_ddot_virtual, q_dot_virtual, q_virtual;
    q_virtual = pre_desired_q_qvqd_;
    q_dot_virtual = pre_desired_q_dot_vqd_;
    q_ddot_virtual = pre_desired_q_ddot_vqd_;
    RigidBodyDynamics::UpdateKinematicsCustom(model_d_, &q_virtual, &q_dot_virtual, &q_ddot_virtual);

    lfoot_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Foot].id, Eigen::Vector3d::Zero(), false);
    lfoot_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Foot].id, false)).transpose();

    lhand_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand].id, lhand_control_point_offset_, false);
    lhand_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand].id, false)).transpose();

    lelbow_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 3].id, Eigen::Vector3d::Zero(), false);
    lelbow_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 3].id, false).transpose();

    lupperarm_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 4].id, Eigen::Vector3d::Zero(), false);
    lupperarm_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 4].id, false).transpose();

    lshoulder_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 5].id, Eigen::Vector3d::Zero(), false);
    lshoulder_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 5].id, false).transpose();

    lacromion_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 6].id, Eigen::Vector3d::Zero(), false);
    lacromion_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 6].id, false).transpose();

    larmbase_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 7].id, Eigen::Vector3d::Zero(), false);
    larmbase_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 7].id, false).transpose();

    rhand_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand].id, rhand_control_point_offset_, false);
    rhand_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand].id, false)).transpose();

    relbow_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 3].id, Eigen::Vector3d::Zero(), false);
    relbow_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 3].id, false).transpose();

    rupperarm_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 4].id, Eigen::Vector3d::Zero(), false);
    rupperarm_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 4].id, false).transpose();

    rshoulder_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 5].id, Eigen::Vector3d::Zero(), false);
    rshoulder_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 5].id, false).transpose();

    racromion_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 6].id, Eigen::Vector3d::Zero(), false);
    racromion_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 6].id, false).transpose();

    rarmbase_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 7].id, Eigen::Vector3d::Zero(), false);
    rarmbase_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 7].id, false).transpose();

    head_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Head].id, Eigen::Vector3d::Zero(), false);
    head_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Head].id, false).transpose();

    upperbody_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, link_avatar_[Upper_Body].id, Eigen::Vector3d::Zero(), false);
    upperbody_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, link_avatar_[Upper_Body].id, false).transpose();

    RigidBodyDynamics::Math::Vector3d com_pos_temp, com_vel_temp, com_accel_temp, com_ang_momentum_temp, com_ang_moment_temp;

    RigidBodyDynamics::Utils::CalcCenterOfMass(model_d_, q_virtual, q_dot_virtual, &q_ddot_virtual, com_mass_, com_pos_temp, &com_vel_temp, &com_accel_temp, &com_ang_momentum_temp, &com_ang_moment_temp, false);
    ///////////////////////////////////////////////////////////////////////////////////////////

    Matrix6d R_R;
    R_R.setZero();
    R_R.block(0, 0, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
    R_R.block(3, 3, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
    // R_R.setIdentity();

    jac_com_ = R_R * link_avatar_[COM_id].jac.cast<double>();
    jac_com_pos_ = pelv_yaw_rot_current_from_global_.transpose() * link_avatar_[COM_id].jac_com.cast<double>().topRows(3);
    

}

void AvatarController::getProcessedRobotData()
{

    if (walking_mode_on_) // command on
    {
        start_time_ = current_time_;
        program_start_time_ = current_time_;

        init_q_ = current_q_;
        last_desired_q_ = current_q_;

        walking_mode_on_ = false;


        pelv_transform_init_from_support_ = pelv_transform_current_from_support_;
        pelv_transform_start_from_support_ = pelv_transform_current_from_support_;
        lfoot_transform_start_from_support_ = lfoot_transform_current_from_support_;
        rfoot_transform_start_from_support_ = rfoot_transform_current_from_support_;

        lfoot_transform_desired_ = lfoot_transform_current_from_support_;
        rfoot_transform_desired_ = rfoot_transform_current_from_support_;
        pelv_transform_desired_ = pelv_transform_current_from_support_;

        lfoot_transform_desired_last_ = lfoot_transform_current_from_support_;
        rfoot_transform_desired_last_ = rfoot_transform_current_from_support_;
        pelv_transform_desired_last_ = pelv_transform_current_from_support_;

        pelv_transform_start_from_global_.translation() = pelv_pos_current_;
        pelv_transform_start_from_global_.linear() = pelv_rot_current_yaw_aline_;
        lfoot_transform_start_from_global_ = lfoot_transform_current_from_global_;
        rfoot_transform_start_from_global_ = rfoot_transform_current_from_global_;

        lfoot_transform_init_from_global_ = lfoot_transform_current_from_global_;
        rfoot_transform_init_from_global_ = rfoot_transform_current_from_global_;

        lhand_transform_init_from_global_ = lhand_transform_current_from_global_;
        rhand_transform_init_from_global_ = rhand_transform_current_from_global_;

        lelbow_transform_init_from_global_ = lelbow_transform_current_from_global_;
        relbow_transform_init_from_global_ = relbow_transform_current_from_global_;

        lupperarm_transform_init_from_global_ = lupperarm_transform_current_from_global_;
        rupperarm_transform_init_from_global_ = rupperarm_transform_current_from_global_;

        lshoulder_transform_init_from_global_ = lshoulder_transform_current_from_global_;
        rshoulder_transform_init_from_global_ = rshoulder_transform_current_from_global_;

        lacromion_transform_init_from_global_ = lacromion_transform_current_from_global_;
        racromion_transform_init_from_global_ = racromion_transform_current_from_global_;

        larmbase_transform_init_from_global_ = larmbase_transform_current_from_global_;
        rarmbase_transform_init_from_global_ = rarmbase_transform_current_from_global_;

        head_transform_init_from_global_ = head_transform_current_from_global_;
        upperbody_transform_init_from_global_ = upperbody_transform_current_from_global_;

        master_lhand_pose_raw_pre_ = lhand_transform_pre_desired_from_;
        master_rhand_pose_raw_pre_ = rhand_transform_pre_desired_from_;
        master_lelbow_pose_raw_pre_ = lupperarm_transform_pre_desired_from_;
        master_relbow_pose_raw_pre_ = rupperarm_transform_pre_desired_from_;
        master_lshoulder_pose_raw_pre_ = lacromion_transform_pre_desired_from_;
        master_rshoulder_pose_raw_pre_ = racromion_transform_pre_desired_from_;
        master_head_pose_raw_pre_ = head_transform_pre_desired_from_;
        master_upperbody_pose_raw_pre_ = upperbody_transform_pre_desired_from_;

        master_lhand_pose_raw_ppre_ = lhand_transform_pre_desired_from_;
        master_rhand_pose_raw_ppre_ = rhand_transform_pre_desired_from_;
        master_head_pose_raw_ppre_ = head_transform_pre_desired_from_;
        master_lelbow_pose_raw_ppre_ = lupperarm_transform_pre_desired_from_;
        master_relbow_pose_raw_ppre_ = rupperarm_transform_pre_desired_from_;
        master_lshoulder_pose_raw_ppre_ = lacromion_transform_pre_desired_from_;
        master_rshoulder_pose_raw_ppre_ = racromion_transform_pre_desired_from_;
        master_upperbody_pose_raw_ppre_ = upperbody_transform_pre_desired_from_;

        master_lhand_pose_pre_ = lhand_transform_pre_desired_from_;
        master_rhand_pose_pre_ = rhand_transform_pre_desired_from_;
        master_lelbow_pose_pre_ = lupperarm_transform_pre_desired_from_;
        master_relbow_pose_pre_ = rupperarm_transform_pre_desired_from_;
        master_lshoulder_pose_pre_ = lacromion_transform_pre_desired_from_;
        master_rshoulder_pose_pre_ = racromion_transform_pre_desired_from_;
        master_head_pose_pre_ = head_transform_pre_desired_from_;
        master_upperbody_pose_pre_ = upperbody_transform_pre_desired_from_;

        master_lhand_pose_ppre_ = lhand_transform_pre_desired_from_;
        master_rhand_pose_ppre_ = rhand_transform_pre_desired_from_;
        master_head_pose_ppre_ = head_transform_pre_desired_from_;
        master_lelbow_pose_ppre_ = lupperarm_transform_pre_desired_from_;
        master_relbow_pose_ppre_ = rupperarm_transform_pre_desired_from_;
        master_lshoulder_pose_ppre_ = lacromion_transform_pre_desired_from_;
        master_rshoulder_pose_ppre_ = racromion_transform_pre_desired_from_;
        master_upperbody_pose_ppre_ = upperbody_transform_pre_desired_from_;

        master_relative_lhand_pos_pre_ = lhand_transform_current_from_global_.translation() - rhand_transform_current_from_global_.translation();
        master_relative_rhand_pos_pre_ = rhand_transform_current_from_global_.translation() - lhand_transform_current_from_global_.translation();

        master_lhand_vel_.setZero();
        master_rhand_vel_.setZero();
        master_lelbow_vel_.setZero();
        master_relbow_vel_.setZero();
        master_lshoulder_vel_.setZero();
        master_rshoulder_vel_.setZero();

        master_lhand_rqy_.setZero();
        master_rhand_rqy_.setZero();
        master_lelbow_rqy_.setZero();
        master_relbow_rqy_.setZero();
        master_lshoulder_rqy_.setZero();
        master_rshoulder_rqy_.setZero();
        master_head_rqy_.setZero();

        lhand_vel_error_.setZero();
        rhand_vel_error_.setZero();
        lelbow_vel_error_.setZero();
        relbow_vel_error_.setZero();
        lacromion_vel_error_.setZero();
        racromion_vel_error_.setZero();

        pelv_pos_init_ = pelv_pos_current_;
        pelv_vel_init_ = pelv_vel_current_;
        pelv_rot_init_ = pelv_rot_current_;
        pelv_rpy_init_ = pelv_rpy_current_;
        pelv_rot_init_yaw_aline_ = pelv_rot_current_yaw_aline_;
        pelv_transform_init_from_global_ = pelv_transform_current_from_global_;

        lfoot_transform_init_from_global_ = lfoot_transform_current_from_global_;
        rfoot_transform_init_from_global_ = rfoot_transform_current_from_global_;
        // lfoot_transform_init_from_global_ = lfoot_transform_pre_desired_from_;
        // rfoot_transform_init_from_global_ = rfoot_transform_pre_desired_from_;


        // init_q_ = current_q_;
        last_desired_q_ = desired_q_;


        lfoot_transform_init_from_support_ = lfoot_transform_current_from_support_;
        rfoot_transform_init_from_support_ = rfoot_transform_current_from_support_;
        pelv_transform_init_from_support_ = pelv_transform_current_from_support_;
        pelv_rpy_init_from_support_ = DyrosMath::rot2Euler(pelv_transform_init_from_support_.linear());
    }
}

void AvatarController::avatarModeStateMachine()
{
    //////CHECK SELF COLLISION///////////
    if(larm_upperbody_sca_mlp_.hx < 0.0)
    {
        larm_upperbody_sca_mlp_.self_collision_stop_cnt_ += 1;
    }
    else
    {
        larm_upperbody_sca_mlp_.self_collision_stop_cnt_ == 0;
    }

    if(rarm_upperbody_sca_mlp_.hx < 0.0)
    {
        rarm_upperbody_sca_mlp_.self_collision_stop_cnt_ += 1;
    }
    else
    {
        rarm_upperbody_sca_mlp_.self_collision_stop_cnt_ == 0;
    }

    if(upper_body_mode_ != 3)
    {
        if(larm_upperbody_sca_mlp_.self_collision_stop_cnt_ > 100 && current_time_ > upperbody_command_time_ + 3.0)
        {
            avatarUpperbodyModeUpdate(3);

            larm_upperbody_sca_mlp_.self_collision_stop_cnt_ = 0;
            cout<< cred << "WARNING: Self Collision is Detected btw Left Arm - Body" << creset << endl;

            std_msgs::Int8 warning_msg_1;
            warning_msg_1.data = 1;
            avatar_warning_pub.publish(warning_msg_1);

            std_msgs::String msg;
            std::stringstream larm_selfcol;
            larm_selfcol << "Self Collision (Left Arm)";
            msg.data = larm_selfcol.str();
            calibration_state_gui_log_pub.publish(msg);
        }
        if(rarm_upperbody_sca_mlp_.self_collision_stop_cnt_ > 100 && current_time_ > upperbody_command_time_ + 3.0)
        {
            avatarUpperbodyModeUpdate(3);

            rarm_upperbody_sca_mlp_.self_collision_stop_cnt_ = 0;
            cout<< cred << "WARNING: Self Collision is Detected btw Right Arm - Body" << creset << endl;

            std_msgs::Int8 warning_msg_2;
            warning_msg_2.data = 2;
            avatar_warning_pub.publish(warning_msg_2);

            std_msgs::String msg;
            std::stringstream rarm_selfcol;
            rarm_selfcol << "Self Collision (Right Arm)";
            msg.data = rarm_selfcol.str();
            calibration_state_gui_log_pub.publish(msg);
        }
    }

    //test
    // if( int(rd_.control_time_*2000)%1000 == 0)
    // {
    //     cout<<"larm hx: "<<larm_upperbody_sca_mlp_.hx << endl;
    //     cout<<"rarm hx: "<<rarm_upperbody_sca_mlp_.hx << endl;
    // }
    //////////////////////////////////////////////////////

    /// @brief masterarm haptic feedback publihser
    if(real_robot_mode_ == true)
    {
        lh_ft_feedback_ = lh_ft_wo_hw_global_lpf_;
        rh_ft_feedback_ = rh_ft_wo_hw_global_lpf_;
    }
    else
    {
        lh_ft_feedback_ = -lh_ft_wo_hw_global_lpf_;
        rh_ft_feedback_ = -rh_ft_wo_hw_global_lpf_;
    }

    double time_smooting = 3.0;
    
    if( upper_body_mode_ < 6)
    {
        if( current_time_ > upperbody_command_time_+time_smooting)
        {
            lh_ft_feedback_.setZero();
            rh_ft_feedback_.setZero();
        }
        else
        {
            double linear_spline;
            linear_spline = DyrosMath::minmax_cut( 
                1-(current_time_ - upperbody_command_time_)/time_smooting, 0.0, 1.0 );
            
            lh_ft_feedback_ = linear_spline*lh_ft_feedback_;
            rh_ft_feedback_ = linear_spline*rh_ft_feedback_;
        }
    }
    else
    {
        if( current_time_ <= upperbody_command_time_+time_smooting)
        {
            double linear_spline;
            linear_spline = DyrosMath::minmax_cut( 
                (current_time_ - upperbody_command_time_)/time_smooting, 0.0, 1.0 );
            
            lh_ft_feedback_ = linear_spline*lh_ft_feedback_;
            rh_ft_feedback_ = linear_spline*rh_ft_feedback_;
        }
    }
    
    // std_msgs::Float32MultiArray hand_ft_msg;
    // hand_ft_msg.data.resize(12);
    // for(int i=0; i<6; i++)
    // {
    //     hand_ft_msg.data[i] = lh_ft_feedback_(i);
    //     hand_ft_msg.data[i+6] = rh_ft_feedback_(i);
    // }
    // haptic_force_pub.publish(hand_ft_msg);
    //////////////////////////////////////////////

    upper_body_mode_ = upper_body_mode_raw_;

    /// @brief upper body mode publisher for GUI
    std_msgs::Int8 msg;
    msg.data = upper_body_mode_;
    upperbodymode_pub.publish(msg);
}
void AvatarController::avatarUpperbodyModeUpdate(int mode_input)
{
    upper_body_mode_raw_ = mode_input;
    upperbody_mode_recieved_ = true;
    // upperbody_command_time_ = current_time_;
    // upperbody_mode_q_init_ = motion_q_pre_;
}
void AvatarController::motionGenerator()
{
    motion_q_dot_.setZero();
    motion_q_.setZero();
    pd_control_mask_.setZero();

    ///////////////////////LEG/////////////////////////
    //////LEFT LEG///////0 0 0.02 0.15 -0.17 0
    motion_q_(0) = 0;
    motion_q_(1) = 0;
    motion_q_(2) = 0.02;
    motion_q_(3) = 0.6;
    motion_q_(4) = -0.12;
    motion_q_(5) = 0;
    pd_control_mask_(0) = 1;
    pd_control_mask_(1) = 0;
    pd_control_mask_(2) = 0;
    pd_control_mask_(3) = 1;
    pd_control_mask_(4) = 1;
    pd_control_mask_(5) = 1;
    //////////////////////
    /////RIFHT LEG////////0 0 0.02 0.15 -0.17 0
    motion_q_(6) = 0;
    motion_q_(7) = 0;
    motion_q_(8) = 0.02;
    motion_q_(9) = 0.6;
    motion_q_(10) = -0.12;
    motion_q_(11) = 0;
    pd_control_mask_(6) = 1;
    pd_control_mask_(7) = 0;
    pd_control_mask_(8) = 0;
    pd_control_mask_(9) = 1;
    pd_control_mask_(10) = 1;
    pd_control_mask_(11) = 1;
    //////////////////////

    poseCalibration();

    if (upper_body_mode_ == 1) // init pose
    {
        if (upperbody_mode_recieved_ == true)
        {
            cout << "Upperbody Mode is Changed to #1" << endl;
            upperbody_mode_recieved_ = false;
            upperbody_command_time_ = current_time_;
            upperbody_mode_q_init_ = motion_q_pre_;
        }

        ///////////////////////WAIST/////////////////////////
        motion_q_(12) = 0;
        motion_q_(13) = 0; // pitch
        motion_q_(14) = 0; // roll
        pd_control_mask_(12) = 1;
        pd_control_mask_(13) = 1;
        pd_control_mask_(14) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////HEAD/////////////////////////
        motion_q_(23) = 0; // yaw
        motion_q_(24) = 0; // pitch
        pd_control_mask_(23) = 1;
        pd_control_mask_(24) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////ARM/////////////////////////
        //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
        motion_q_(15) = 0.3;
        motion_q_(16) = 0.3;
        motion_q_(17) = 1.5;
        motion_q_(18) = -1.27;
        motion_q_(19) = -1.0;
        motion_q_(20) = 0.0;
        motion_q_(21) = -1.0;
        motion_q_(22) = 0.0;

        pd_control_mask_(15) = 1;
        pd_control_mask_(16) = 1;
        pd_control_mask_(17) = 1;
        pd_control_mask_(18) = 1;
        pd_control_mask_(19) = 1;
        pd_control_mask_(20) = 1;
        pd_control_mask_(21) = 1;
        pd_control_mask_(22) = 1;
        //////////////////////
        /////RIFHT ARM////////-0.3 -0.3 -1.5 1.27 1 0 1 0
        motion_q_(25) = -0.3;
        motion_q_(26) = -0.3;
        motion_q_(27) = -1.5;
        motion_q_(28) = 1.27;
        motion_q_(29) = 1.0;
        motion_q_(30) = 0.0;
        motion_q_(31) = 1.0;
        motion_q_(32) = 0.0;

        pd_control_mask_(25) = 1;
        pd_control_mask_(26) = 1;
        pd_control_mask_(27) = 1;
        pd_control_mask_(28) = 1;
        pd_control_mask_(29) = 1;
        pd_control_mask_(30) = 1;
        pd_control_mask_(31) = 1;
        pd_control_mask_(32) = 1;
        /////////////////////////////////////////////////////

        for (int i = 12; i < MODEL_DOF; i++)
        {
            motion_q_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 4, upperbody_mode_q_init_(i), 0, 0, motion_q_(i), 0, 0)(0);
        }
    }
    else if (upper_body_mode_ == 2) // Zero pose
    {
        if (upperbody_mode_recieved_ == true)
        {
            cout << "Upperbody Mode is Changed to #2" << endl;
            upperbody_mode_recieved_ = false;
            upperbody_command_time_ = current_time_;
            upperbody_mode_q_init_ = motion_q_pre_;
        }
        ///////////////////////WAIST/////////////////////////
        motion_q_(12) = 0; // pitch
        motion_q_(13) = 0; // pitch
        motion_q_(14) = 0; // roll
        pd_control_mask_(12) = 1;
        pd_control_mask_(13) = 1;
        pd_control_mask_(14) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////HEAD/////////////////////////
        motion_q_(23) = 0; // yaw
        motion_q_(24) = 0; // pitch
        pd_control_mask_(23) = 1;
        pd_control_mask_(24) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////ARM/////////////////////////
        //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
        motion_q_(15) = 0.3;
        motion_q_(16) = 0.12;
        motion_q_(17) = 1.43;
        motion_q_(18) = -0.85;
        motion_q_(19) = -0.45; // elbow
        motion_q_(20) = 1;
        motion_q_(21) = 0.0;
        motion_q_(22) = 0.0;
        pd_control_mask_(15) = 1;
        pd_control_mask_(16) = 1;
        pd_control_mask_(17) = 1;
        pd_control_mask_(18) = 1;
        pd_control_mask_(19) = 1;
        pd_control_mask_(20) = 1;
        pd_control_mask_(21) = 1;
        pd_control_mask_(22) = 1;
        //////////////////////
        /////RIFHT ARM////////-0.3 -0.3 -1.5 1.27 1 0 1 0
        motion_q_(25) = -0.3;
        motion_q_(26) = -0.12;
        motion_q_(27) = -1.43;
        motion_q_(28) = 0.85;
        motion_q_(29) = 0.45; // elbow
        motion_q_(30) = -1;
        motion_q_(31) = 0.0;
        motion_q_(32) = 0.0;
        pd_control_mask_(25) = 1;
        pd_control_mask_(26) = 1;
        pd_control_mask_(27) = 1;
        pd_control_mask_(28) = 1;
        pd_control_mask_(29) = 1;
        pd_control_mask_(30) = 1;
        pd_control_mask_(31) = 1;
        pd_control_mask_(32) = 1;
        /////////////////////////////////////////////////////

        for (int i = 12; i < MODEL_DOF; i++)
        {
            motion_q_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 4, upperbody_mode_q_init_(i), 0, 0, motion_q_(i), 0, 0)(0);
        }
    }
    else if (upper_body_mode_ == 3) // Freezing
    {
        if (upperbody_mode_recieved_ == true)
        {
            cout << "Upperbody Mode is Changed to #3" << endl;
            cout << "----------Robot is Freezed---------" << endl;

            upperbody_mode_recieved_ = false;
            upperbody_mode_q_init_ = motion_q_pre_;

            std_msgs::String msg;
            std::stringstream upperbody_mode_ss;
            upperbody_mode_ss << "Robot is Freezed!";
            msg.data = upperbody_mode_ss.str();
            calibration_state_pub.publish(msg);
            calibration_state_gui_log_pub.publish(msg);
        }

        for (int i = 12; i < MODEL_DOF; i++)
        {
            motion_q_(i) = upperbody_mode_q_init_(i);
            pd_control_mask_(i) = 1;
        }
    }
    else if (upper_body_mode_ == 4) // READY pose
    {
        if (upperbody_mode_recieved_ == true)
        {
            cout << "Upperbody Mode is Changed to #4 (READY POSE)" << endl;
            upperbody_mode_recieved_ = false;
            upperbody_command_time_ = current_time_;
            upperbody_mode_q_init_ = motion_q_pre_;

            std_msgs::String msg;
            std::stringstream upperbody_mode_ss;
            upperbody_mode_ss << "Ready Pose is On!";
            msg.data = upperbody_mode_ss.str();
            calibration_state_pub.publish(msg);
            calibration_state_gui_log_pub.publish(msg);
        }
        ///////////////////////WAIST/////////////////////////
        motion_q_(12) = 0; // pitch
        motion_q_(13) = 0; // pitch
        motion_q_(14) = 0; // roll
        pd_control_mask_(12) = 1;
        pd_control_mask_(13) = 1;
        pd_control_mask_(14) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////HEAD/////////////////////////
        motion_q_(23) = 0; // yaw
        motion_q_(24) = 0; // pitch
        pd_control_mask_(23) = 1;
        pd_control_mask_(24) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////ARM/////////////////////////
        //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
        motion_q_(15) = 0.3;
        motion_q_(16) = -0.6;
        motion_q_(17) = 1.2;
        motion_q_(18) = -0.80;
        motion_q_(19) = -2.3; // elbow
        motion_q_(20) = 1.45;
        motion_q_(21) = 0.0;
        motion_q_(22) = 0.0;
        pd_control_mask_(15) = 1;
        pd_control_mask_(16) = 1;
        pd_control_mask_(17) = 1;
        pd_control_mask_(18) = 1;
        pd_control_mask_(19) = 1;
        pd_control_mask_(20) = 1;
        pd_control_mask_(21) = 1;
        pd_control_mask_(22) = 1;
        //////////////////////
        /////RIFHT ARM////////-0.3 -0.3 -1.5 1.27 1 0 1 0
        motion_q_(25) = -0.3;
        motion_q_(26) = 0.6;
        motion_q_(27) = -1.2;
        motion_q_(28) = 0.8;
        motion_q_(29) = 2.3; // elbow
        motion_q_(30) = -1.45;
        motion_q_(31) = 0.0;
        motion_q_(32) = 0.0;
        pd_control_mask_(25) = 1;
        pd_control_mask_(26) = 1;
        pd_control_mask_(27) = 1;
        pd_control_mask_(28) = 1;
        pd_control_mask_(29) = 1;
        pd_control_mask_(30) = 1;
        pd_control_mask_(31) = 1;
        pd_control_mask_(32) = 1;
        /////////////////////////////////////////////////////

        for (int i = 12; i < MODEL_DOF; i++)
        {
            motion_q_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 4, upperbody_mode_q_init_(i), 0, 0, motion_q_(i), 0, 0)(0);
        }
    }
    else if (upper_body_mode_ == 5) // HEAD ONLY
    {
        if (still_pose_cali_flag_ == false)
        {
            cout << cred << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << creset << endl;
            avatarUpperbodyModeUpdate(3);
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #5 (HEAD ONLY MODE)" << endl;

                upperbody_mode_q_init_ = motion_q_pre_;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "HEAD Only Tracking Contorol in On";
                msg.data = upperbody_mode_ss.str();
                calibration_state_pub.publish(msg);
                calibration_state_gui_log_pub.publish(msg);
            }

            for (int i = 12; i < MODEL_DOF; i++)
            {
                motion_q_(i) = upperbody_mode_q_init_(i);
                pd_control_mask_(i) = 1;
            }

            rawMasterPoseProcessing();
            ///////////////////////HEAD/////////////////////////
            Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
            error_w_head = head_transform_pre_desired_from_.linear().transpose() * error_w_head;
            error_w_head(0) = 0;
            error_w_head = head_transform_pre_desired_from_.linear() * error_w_head;

            MatrixXd J_temp, J_head, I3, J_inv_head;

            Vector3d u_dot_head = 200 * error_w_head;
            J_temp.setZero(6, MODEL_DOF_VIRTUAL);
            J_head.setZero(3, 2);
            I3.setIdentity(3, 3);

            RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Head].id, Eigen::Vector3d::Zero(), J_temp, false);
            J_head.block(0, 0, 3, 2) = J_temp.block(0, 29, 3, 2); // orientation
            J_inv_head = J_head.transpose() * (J_head * J_head.transpose() + I3 * 0.000001).inverse();

            for (int i = 0; i < 3; i++)
            {
                u_dot_head(i) = DyrosMath::minmax_cut(u_dot_head(i), -2.0, 2.0);
            }

            motion_q_dot_.segment(23, 2) = J_inv_head * u_dot_head;
            motion_q_dot_(23) = DyrosMath::minmax_cut(motion_q_dot_(23), joint_vel_limit_l_(23), joint_vel_limit_h_(23));
            motion_q_dot_(24) = DyrosMath::minmax_cut(motion_q_dot_(24), joint_vel_limit_l_(24), joint_vel_limit_h_(24));

            motion_q_.segment(23, 2) = motion_q_pre_.segment(23, 2) + motion_q_dot_.segment(23, 2) * dt_;
            motion_q_(23) = DyrosMath::minmax_cut(motion_q_(23), joint_limit_l_(23), joint_limit_h_(23));
            motion_q_(24) = DyrosMath::minmax_cut(motion_q_(24), joint_limit_l_(24), joint_limit_h_(24));

            // cout<<"master_head_pose_: \n"<<master_head_pose_.linear()<<endl;
            // motion_q_dot_.setZero();
        }
    }
    else if ((upper_body_mode_ == 6)) // HQPIK ver1
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << cred << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << creset << endl;
            avatarUpperbodyModeUpdate(3);
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #6 (HQPIK1-AVATAR XPRIZE SEMIFINALS VERSION)" << endl;

                first_loop_hqpik_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Motion Tracking Contorol in On (HQPIK1-AVATAR XPRIZE SEMIFINALS VERSION)";
                msg.data = upperbody_mode_ss.str();
                calibration_state_pub.publish(msg);
                calibration_state_gui_log_pub.publish(msg);
            }

            rawMasterPoseProcessing();
            motionRetargeting_HQPIK();
            // motionRetargeting_HQPIK_lexls();
            // motionRetargeting_QPIK_upperbody();
            // if (int(current_time_ * 10000) % 10000 == 0)
            // {
            //     cout<<"hqpik_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
            // }
        }
    }
    else if (upper_body_mode_ == 7) // HQPIK ver2
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << cred << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << creset << endl;
            avatarUpperbodyModeUpdate(3);
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #7 (HQPIK ver2)" << endl;

                first_loop_hqpik2_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Motion Tracking Contorol in On (HQPIK ver2)";
                msg.data = upperbody_mode_ss.str();
                calibration_state_pub.publish(msg);
                calibration_state_gui_log_pub.publish(msg);
            }

            rawMasterPoseProcessing();
            motionRetargeting_HQPIK();

            // if (int(current_time_ * 10000) % 10000 == 0)
            // {
            //     cout<<"hqpik_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
            // }
        }
    }
    else if (upper_body_mode_ == 8) // Absolute mapping
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << cred << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << creset << endl;
            avatarUpperbodyModeUpdate(3);
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #8 (ABSOLUTE HAND POS MAPPING)" << endl;

                first_loop_hqpik_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Upperbody Mode is Changed to #8 (ABSOLUTE HAND POS MAPPING)";
                msg.data = upperbody_mode_ss.str();
                calibration_state_pub.publish(msg);
                calibration_state_gui_log_pub.publish(msg);
            }

            rawMasterPoseProcessing();
            motionRetargeting_HQPIK();
            // motionRetargeting_HQPIK_lexls();
            // motionRetargeting_QPIK_upperbody();
            // if (int(current_time_ * 10000) % 10000 == 0)
            // {
            //     cout<<"hqpik_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
            // }
        }
    }
    else if (upper_body_mode_ == 9) // Propositional mapping
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << cred << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << creset << endl;
            avatarUpperbodyModeUpdate(3);
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #9 (PROPOSITIONAL HAND POS MAPPING)" << endl;

                first_loop_hqpik_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Upperbody Mode is Changed to #9 (PROPOSITIONAL HAND POS MAPPING)";
                msg.data = upperbody_mode_ss.str();
                calibration_state_pub.publish(msg);
                calibration_state_gui_log_pub.publish(msg);
            }

            rawMasterPoseProcessing();
            motionRetargeting_HQPIK();
            // motionRetargeting_HQPIK_lexls();
            // motionRetargeting_QPIK_upperbody();
            // if (int(current_time_ * 10000) % 10000 == 0)
            // {
            //     cout<<"hqpik_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
            // }
        }
    }
    else if (upper_body_mode_ == 10) // Cali Pose Direction Only
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << cred << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << creset << endl;
            avatarUpperbodyModeUpdate(3);
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #10 (3D Mouse Mode)" << endl;

                first_loop_hqpik_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Upperbody Mode is Changed to #10 (3D Mouse Mode)";
                msg.data = upperbody_mode_ss.str();
                calibration_state_pub.publish(msg);
                calibration_state_gui_log_pub.publish(msg);
            }

            rawMasterPoseProcessing();
            motionRetargeting_HQPIK();
            // motionRetargeting_HQPIK_lexls();
            // motionRetargeting_QPIK_upperbody();
            // if (int(current_time_ * 10000) % 10000 == 0)
            // {
            //     cout<<"hqpik_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
            // }
        }
    }

}

void AvatarController::motionRetargeting_QPIK_larm()
{

    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    /////////////////////////////////ARM///////////////////////////////////////////
    const int variable_size = 8;
    const int constraint_size1 = 8; //[lb <=	x	<= 	ub] form constraints
    const int constraint_size2 = 6; //[lb <=	Ax 	<=	ub] from constraints
    const int control_size_hand = 6;
    const int control_size_upperarm = 3;
    const int control_size_shoulder = 3;

    if (first_loop_larm_)
    {
        QP_qdot_larm.InitializeProblemSize(variable_size, constraint_size2);

        first_loop_larm_ = false;
    }

    double w1 = 2500;  // hand tracking
    double w2 = 0.002; // joint acc
    double w3 = 1;     // task space vel
    double w4 = 30;    // joint vel
    double w5 = 50;    // upperarm oriention tracking
    double w6 = 1;     // shoulder oriention tracking

    MatrixXd J_l_arm, J_l_upperarm, J_l_shoulder, J_temp1, J_temp2, J_temp3;
    J_l_arm.setZero(control_size_hand, variable_size);
    J_l_upperarm.setZero(control_size_upperarm, variable_size);
    J_l_shoulder.setZero(control_size_shoulder, variable_size);

    VectorXd u_dot_lhand, u_dot_lupperarm, u_dot_lshoulder;
    u_dot_lhand.setZero(control_size_hand);
    u_dot_lupperarm.setZero(control_size_upperarm);
    u_dot_lshoulder.setZero(control_size_shoulder);

    MatrixXd H, H1, H2, H3, H4, H5, H6, A;
    VectorXd g, g1, g2, g5, g6, ub, lb, ubA, lbA;

    MatrixXd I8;
    I8.setIdentity(variable_size, variable_size);

    H.setZero(variable_size, variable_size);
    H1.setZero(variable_size, variable_size);
    H2.setZero(variable_size, variable_size);
    H3.setZero(variable_size, variable_size);
    H4.setZero(variable_size, variable_size);
    H5.setZero(variable_size, variable_size);
    H6.setZero(variable_size, variable_size);

    A.setZero(constraint_size2, variable_size);

    g.setZero(variable_size);
    g1.setZero(variable_size);
    g2.setZero(variable_size);
    g5.setZero(variable_size);
    g6.setZero(variable_size);

    ub.setZero(constraint_size1);
    lb.setZero(constraint_size1);

    ubA.setZero(constraint_size2);
    lbA.setZero(constraint_size2);

    VectorXd qpres;

    // VectorQVQd q_desired_pre;
    // q_desired_pre.setZero();
    // q_desired_pre(39) = 1;
    // q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;

    Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
    Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());

    Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    error_w_lupperarm(0) = 0;
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear() * error_w_lupperarm;

    Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
    error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
    error_w_lshoulder(0) = 0;
    error_w_lshoulder = lacromion_transform_pre_desired_from_.linear() * error_w_lshoulder;

    for (int i = 0; i < 3; i++)
    {
        u_dot_lhand(i) = master_lhand_vel_(i) + 200 * error_v_lhand(i);
        u_dot_lhand(i + 3) = master_lhand_vel_(i + 3) + 100 * error_w_lhand(i);

        u_dot_lupperarm(i) = master_lelbow_vel_(i + 3) + 100 * error_w_lupperarm(i);
        u_dot_lshoulder(i) = master_lshoulder_vel_(i + 3) + 100 * error_w_lshoulder(i);
    }

    Vector3d zero3;
    zero3.setZero();
    J_temp1.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand].id, lhand_control_point_offset_, J_temp1, false);
    J_l_arm.block(0, 0, 3, 8) = J_temp1.block(3, 21, 3, 8); // position
    J_l_arm.block(3, 0, 3, 8) = J_temp1.block(0, 21, 3, 8); // orientation
    // J_l_arm.block(0, 0, 6, 1).setZero(); //penalize 1st joint for hand control

    J_temp2.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 4].id, zero3, J_temp2, false);
    J_l_upperarm.block(0, 0, 3, 8) = J_temp2.block(0, 21, 3, 8); // orientation
    // J_l_upperarm.block(0, 0, 3, 1).setZero(); //penalize 1st joint for hand control

    J_temp3.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 6].id, zero3, J_temp3, false);
    J_l_shoulder.block(0, 0, 3, 8) = J_temp3.block(0, 21, 3, 8); // orientation

    //// null space projection of hand and elbow////
    MatrixXd J1, J2, J3, N1, N2_aug, J2_aug, J2_aug_pinv, J1_pinv, J2_pinv, J3_pinv, J2N1, J2N1_pinv, J3N2_aug, J3N2_aug_pinv, I3, I6, I9;
    MatrixXd q1_dot_star, q2_dot_star;
    J1.setZero(control_size_hand, variable_size);
    J2.setZero(control_size_upperarm, variable_size);
    J3.setZero(control_size_shoulder, variable_size);

    N1.setZero(variable_size, variable_size);
    N2_aug.setZero(variable_size, variable_size);
    J2_aug.setZero(control_size_hand + control_size_upperarm, variable_size);
    J2_aug_pinv.setZero(variable_size, control_size_hand + control_size_upperarm);
    J2N1.setZero(control_size_upperarm, variable_size);
    J2N1_pinv.setZero(variable_size, control_size_upperarm);
    J3N2_aug.setZero(control_size_shoulder, variable_size);
    J3N2_aug_pinv.setZero(variable_size, control_size_shoulder);
    q1_dot_star.setZero(variable_size, 1);
    q2_dot_star.setZero(variable_size, 1);
    I3.setIdentity(3, 3);
    I6.setIdentity(6, 6);
    I9.setIdentity(9, 9);

    J1 = J_l_arm;
    J2 = J_l_upperarm;
    J3 = J_l_shoulder;
    J1_pinv = J1.transpose() * (J1 * J1.transpose() + I6 * 0.0001).inverse();
    N1 = I8 - (J1_pinv)*J1;
    // q1_dot_star = J1_pinv*u_dot_lhand;

    J2_pinv = J2.transpose() * (J2 * J2.transpose() + I3 * 0.0001).inverse();
    J2_aug.block(0, 0, control_size_hand, variable_size) = J1;
    J2_aug.block(control_size_hand, 0, control_size_upperarm, variable_size) = J2;

    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    J2_aug_pinv = J2_aug.transpose() * (J2_aug * J2_aug.transpose() + I9 * 0.0001).inverse();
    // std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    N2_aug = I8 - (J2_aug_pinv)*J2_aug;

    J2N1 = J2 * N1;
    J2N1_pinv = J2N1.transpose() * (J2N1 * J2N1.transpose() + I3 * 0.0001).inverse();
    // q2_dot_star = q1_dot_star + J2N1_pinv*(u_dot_lupperarm - J2*q1_dot_star);

    J3N2_aug = J3 * N2_aug;

    H1 = J1.transpose() * J1;
    H2 = I8 * (1 / dt_) * (1 / dt_);
    H3 = J1.transpose() * J1;
    H4 = A_mat_.block(21, 21, variable_size, variable_size);
    H5 = (J2N1).transpose() * (J2N1);
    H6 = (J3N2_aug).transpose() * (J3N2_aug);

    g1 = -J1.transpose() * u_dot_lhand;
    g2 = -motion_q_dot_pre_.segment(15, 8) * (1 / dt_) * (1 / dt_);
    g5 = -(J2N1).transpose() * (u_dot_lupperarm - J2 * motion_q_dot_pre_.segment(15, variable_size));
    g6 = -(J3N2_aug).transpose() * (u_dot_lshoulder - J3 * motion_q_dot_pre_.segment(15, variable_size));
    ////////////////////////////////////////////////////////////

    H = w1 * H1 + w2 * H2 + w3 * H3 + w4 * H4 + w5 * H5 + w6 * H6;
    g = w1 * g1 + w2 * g2 + w5 * g5 + w6 * g6;

    if (int(current_time_ * 1e4) % int(1e3) == 0)
    {
        // cout<<"g1: \n"<<g1<<endl;
        // cout<<"g2: \n"<<g2<<endl;
        // cout<<"g5: \n"<<g5<<endl;

        // cout<<"u_dot_lhand: \n"<<u_dot_lhand<<endl;
        // cout<<"u_dot_lelbow: \n"<<u_dot_lelbow<<endl;

        // cout<<"lelbow_transform_pre_desired_from: \n"<<lelbow_transform_pre_desired_from.translation()<<endl;

        // // cout<<"N1: \n"<<N1<<endl;
        // cout<<"J_l_arm: \n"<<J_l_arm<<endl;
    }

    double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

    for (int i = 0; i < constraint_size1; i++)
    {
        lb(i) = max(speed_reduce_rate * (joint_limit_l_(i + 15) - current_q_(i + 15)), joint_vel_limit_l_(i + 15));
        ub(i) = min(speed_reduce_rate * (joint_limit_h_(i + 15) - current_q_(i + 15)), joint_vel_limit_h_(i + 15));
        // lb(i) = joint_vel_limit_l_(i+1);
        // ub(i) = joint_vel_limit_h_(i+1);
    }

    A = J_l_arm;

    for (int i = 0; i < constraint_size2; i++)
    {
        for (int j = 0; j < variable_size; j++)
        {
            if (abs(A(i, j)) < 1e-3)
            {
                if (A(i, j) > 0)
                {
                    A(i, j) = 1e-3;
                }
                else
                {
                    A(i, j) = -1e-3;
                }
            }
        }
    }

    for (int i = 0; i < 3; i++) // position velocity limit
    {
        lbA(i) = -1;
        ubA(i) = 1;
    }

    for (int i = 3; i < 6; i++) // angular velocity limit
    {
        lbA(i) = -3;
        ubA(i) = 3;
    }

    QP_qdot_larm.EnableEqualityCondition(equality_condition_eps_);
    QP_qdot_larm.UpdateMinProblem(H, g);
    QP_qdot_larm.UpdateSubjectToAx(A, lbA, ubA);
    QP_qdot_larm.UpdateSubjectToX(lb, ub);

    VectorXd q_dot_larm;
    // std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    // std::chrono::steady_clock::time_point t5;

    if (QP_qdot_larm.SolveQPoases(100, qpres))
    {
        q_dot_larm = qpres.segment(0, variable_size);
        // t5 = std::chrono::steady_clock::now();
    }
    else
    {
        q_dot_larm.setZero(variable_size);
    }

    for (int i = 0; i < variable_size; i++)
    {
        motion_q_dot_(15 + i) = q_dot_larm(i);
        motion_q_(15 + i) = motion_q_pre_(15 + i) + motion_q_dot_(15 + i) * dt_;
        pd_control_mask_(15 + i) = 1;
    }

    if (int(current_time_ * 1e4) % int(1e3) == 0)
    {
        // cout<<"J1: \n"<<J_l_arm<<endl;
        // cout<<"N1: \n"<<N1<<endl;
        // cout<<"N2: \n"<<N_2<<endl;
        // cout<<"J2N1: \n"<<J2N1<<endl;
        // cout<<"J2N1_pinv: \n"<<J2N1_pinv<<endl;
        // cout<<"J3N1N2: \n"<<J3N1N2<<endl;
        // // cout<<"J3N1N2_pinv: \n"<<J3N1N2_pinv<<endl;
        // cout<<"left 1st task error: \n"<< J_l_arm*q_dot_larm - u_dot_lhand<<endl;
        // cout<<"left 2nd task error: \n"<< J_l_upperarm*q_dot_larm - u_dot_lupperarm<<endl;
        // cout<<"left 3rd task error: \n"<< J_l_shoulder*q_dot_larm - u_dot_lshoulder<<endl;
        // cout<<"qdot2: \n"<<(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
        // cout<<"N1 X qdot2: \n"<<N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
        // cout<<"J2 X N1 X qdot2: \n"<<J_l_upperarm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
        // cout<<"J1 X N1 X qdot2: \n"<<J_l_arm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;

        // cout<<"t2 - t1: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
        // cout<<"t3 - t2: "<< std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() <<endl;
        // cout<<"t4 - t3: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() <<endl;
        // cout<<"t5 - t4: "<< std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() <<endl;
        // cout<<"t5 - t1: "<< std::chrono::duration_cast<std::chrono::microseconds>(t5 - t1).count() <<endl;
        // cout<<"q1_dot_star: "<< q1_dot_star <<endl;
        // cout<<"q2_dot_star: "<< q2_dot_star <<endl;
    }

    lhand_vel_error_ = J_l_arm * q_dot_larm - u_dot_lhand;
    lelbow_vel_error_ = J_l_upperarm * q_dot_larm - u_dot_lupperarm;
    lacromion_vel_error_ = J_l_shoulder * q_dot_larm - u_dot_lshoulder;

    // motion_q_(15) = 0.3;
    // pd_control_mask_(15) = 1;

    // motion_q_(19) = DyrosMath::minmax_cut(motion_q_(19), joint_limit_l_(4), joint_limit_h_(4));		//elbow
    // motion_q_(29) = DyrosMath::minmax_cut(motion_q_(29), joint_limit_l_(12), joint_limit_h_(12));
    /////////////////////////////////////////////////////////////////////////////////////////////
}

void AvatarController::motionRetargeting_QPIK_rarm()
{
    /////////////////////////////////ARM///////////////////////////////////////////
    const int variable_size = 8;
    const int constraint_size1 = 8; //[lb <=	x	<= 	ub] form constraints
    const int constraint_size2 = 6; //[lb <=	Ax 	<=	ub] from constraints
    const int control_size_hand = 6;
    const int control_size_upperarm = 3;
    const int control_size_shoulder = 3;

    if (first_loop_rarm_)
    {
        QP_qdot_rarm.InitializeProblemSize(variable_size, constraint_size2);

        first_loop_rarm_ = false;
    }

    double w1 = 2500;  // hand tracking
    double w2 = 0.002; // joint acc
    double w3 = 1;     // task space vel
    double w4 = 30;    // joint vel
    double w5 = 50;    // elbow position tracking
    double w6 = 1;     // shoulder oriention tracking

    MatrixXd J_r_arm, J_r_upperarm, J_r_shoulder, J_temp1, J_temp2, J_temp3;
    J_r_arm.setZero(control_size_hand, variable_size);
    J_r_upperarm.setZero(control_size_upperarm, variable_size);
    J_r_shoulder.setZero(control_size_shoulder, variable_size);

    VectorXd u_dot_rhand, u_dot_rupperarm, u_dot_rshoulder;
    u_dot_rhand.setZero(control_size_hand);
    u_dot_rupperarm.setZero(control_size_upperarm);
    u_dot_rshoulder.setZero(control_size_shoulder);

    MatrixXd H, H1, H2, H3, H4, H5, H6, A;
    VectorXd g, g1, g2, g5, g6, ub, lb, ubA, lbA;

    MatrixXd I8;
    I8.setIdentity(variable_size, variable_size);

    H.setZero(variable_size, variable_size);
    H1.setZero(variable_size, variable_size);
    H2.setZero(variable_size, variable_size);
    H3.setZero(variable_size, variable_size);
    H4.setZero(variable_size, variable_size);
    H5.setZero(variable_size, variable_size);
    H6.setZero(variable_size, variable_size);

    A.setZero(constraint_size2, variable_size);

    g.setZero(variable_size);
    g1.setZero(variable_size);
    g2.setZero(variable_size);
    g5.setZero(variable_size);
    g6.setZero(variable_size);

    ub.setZero(constraint_size1);
    lb.setZero(constraint_size1);

    ubA.setZero(constraint_size2);
    lbA.setZero(constraint_size2);

    VectorXd qpres;

    // VectorQVQd q_desired_pre;
    // q_desired_pre.setZero();
    // q_desired_pre(39) = 1;
    // q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;

    Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
    Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());

    Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    error_w_rupperarm(0) = 0;
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear() * error_w_rupperarm;

    Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
    error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
    error_w_rshoulder(0) = 0;
    error_w_rshoulder = racromion_transform_pre_desired_from_.linear() * error_w_rshoulder;

    double hand_relative_p_gain = DyrosMath::cubic(master_relative_rhand_pos_.norm(), 0.4, robot_shoulder_width_, 1, 0, 0, 0);

    for (int i = 0; i < 3; i++)
    {
        u_dot_rhand(i) = master_rhand_vel_(i) + 200 * error_v_rhand(i);
        u_dot_rhand(i + 3) = master_rhand_vel_(i + 3) + 100 * error_w_rhand(i);

        u_dot_rupperarm(i) = master_relbow_vel_(i + 3) + 100 * error_w_rupperarm(i);
        u_dot_rshoulder(i) = master_rshoulder_vel_(i + 3) + 100 * error_w_rshoulder(i);
    }

    Vector3d zero3;
    zero3.setZero();
    J_temp1.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand].id, rhand_control_point_offset_, J_temp1, false);
    J_r_arm.block(0, 0, 3, 8) = J_temp1.block(3, 31, 3, 8); // position
    J_r_arm.block(3, 0, 3, 8) = J_temp1.block(0, 31, 3, 8); // orientation
    // J_r_arm.block(0, 0, 6, 1).setZero(); //penalize 1st joint for hand control

    J_temp2.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 4].id, zero3, J_temp2, false);
    J_r_upperarm.block(0, 0, 3, 8) = J_temp2.block(0, 31, 3, 8); // orientation
    // J_r_upperarm.block(0, 0, 3, 1).setZero(); //penalize 1st joint for hand control

    J_temp3.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 6].id, zero3, J_temp3, false);
    J_r_shoulder.block(0, 0, 3, 8) = J_temp3.block(0, 31, 3, 8); // orientation

    //// null space projection of hand and elbow////
    MatrixXd J1, J2, J3, N1, N2_aug, J2_aug, J2_aug_pinv, J1_pinv, J2_pinv, J3_pinv, J2N1, J2N1_pinv, J3N2_aug, J3N2_aug_pinv, I3, I6, I9;
    MatrixXd q1_dot_star, q2_dot_star;
    J1.setZero(control_size_hand, variable_size);
    J2.setZero(control_size_upperarm, variable_size);
    J3.setZero(control_size_shoulder, variable_size);

    N1.setZero(variable_size, variable_size);
    N2_aug.setZero(variable_size, variable_size);
    J2_aug.setZero(control_size_hand + control_size_upperarm, variable_size);
    J2_aug_pinv.setZero(variable_size, control_size_hand + control_size_upperarm);
    J2N1.setZero(control_size_upperarm, variable_size);
    J2N1_pinv.setZero(variable_size, control_size_upperarm);
    J3N2_aug.setZero(control_size_shoulder, variable_size);
    J3N2_aug_pinv.setZero(variable_size, control_size_shoulder);
    q1_dot_star.setZero(variable_size, 1);
    q2_dot_star.setZero(variable_size, 1);
    I3.setIdentity(3, 3);
    I6.setIdentity(6, 6);
    I9.setIdentity(9, 9);

    J1 = J_r_arm;
    J2 = J_r_upperarm;
    J3 = J_r_shoulder;
    J1_pinv = J1.transpose() * (J1 * J1.transpose() + I6 * 0.0001).inverse();
    N1 = I8 - (J1_pinv)*J1;
    // q1_dot_star = J1_pinv*u_dot_rhand;

    J2_pinv = J2.transpose() * (J2 * J2.transpose() + I3 * 0.0001).inverse();
    J2_aug.block(0, 0, control_size_hand, variable_size) = J1;
    J2_aug.block(control_size_hand, 0, control_size_upperarm, variable_size) = J2;

    J2_aug_pinv = J2_aug.transpose() * (J2_aug * J2_aug.transpose() + I9 * 0.0001).inverse();

    N2_aug = I8 - (J2_aug_pinv)*J2_aug;

    J2N1 = J2 * N1;
    J2N1_pinv = J2N1.transpose() * (J2N1 * J2N1.transpose() + I3 * 0.0001).inverse();
    // q2_dot_star = q1_dot_star + J2N1_pinv*(u_dot_rupperarm - J2*q1_dot_star);

    J3N2_aug = J3 * N2_aug;

    H1 = J1.transpose() * J1;
    H2 = I8 * (1 / dt_) * (1 / dt_);
    H3 = J1.transpose() * J1;
    H4 = A_mat_.block(31, 31, variable_size, variable_size);
    H5 = (J2N1).transpose() * (J2N1);
    H6 = (J3N2_aug).transpose() * (J3N2_aug);

    g1 = -J1.transpose() * u_dot_rhand;
    g2 = -motion_q_dot_pre_.segment(25, 8) * (1 / dt_) * (1 / dt_);
    g5 = -(J2N1).transpose() * (u_dot_rupperarm - J2 * motion_q_dot_pre_.segment(25, variable_size));
    g6 = -(J3N2_aug).transpose() * (u_dot_rshoulder - J3 * motion_q_dot_pre_.segment(25, variable_size));
    ////////////////////////////////////////////////////////////

    H = w1 * H1 + w2 * H2 + w3 * H3 + w4 * H4 + w5 * H5 + w6 * H6;
    g = w1 * g1 + w2 * g2 + w5 * g5 + w6 * g6;

    if (int(current_time_ * 1e4) % int(1e3) == 0)
    {
        // cout<<"g1: \n"<<g1<<endl;
        // cout<<"g2: \n"<<g2<<endl;
        // cout<<"g5: \n"<<g5<<endl;

        // cout<<"u_dot_rhand: \n"<<u_dot_rhand<<endl;
        // cout<<"u_dot_rupperarm: \n"<<u_dot_rupperarm<<endl;

        // cout<<"racromion_transform_pre_desired_from: \n"<<racromion_transform_pre_desired_from_.translation()<<endl;
        // cout<<"rarmbase_transform_pre_desired_from: \n"<<rarmbase_transform_pre_desired_from_.translation()<<endl;

        // cout<<"N1: \n"<<N1<<endl;
        // cout<<"J_r_arm: \n"<<J_r_arm<<endl;
    }

    double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

    for (int i = 0; i < constraint_size1; i++)
    {
        lb(i) = max(speed_reduce_rate * (joint_limit_l_(i + 25) - current_q_(i + 25)), joint_vel_limit_l_(i + 25));
        ub(i) = min(speed_reduce_rate * (joint_limit_h_(i + 25) - current_q_(i + 25)), joint_vel_limit_h_(i + 25));
    }

    A = J_r_arm;

    for (int i = 0; i < constraint_size2; i++)
    {
        for (int j = 0; j < variable_size; j++)
        {
            if (abs(A(i, j)) < 1e-3)
            {
                if (A(i, j) > 0)
                {
                    A(i, j) = 1e-3;
                }
                else
                {
                    A(i, j) = -1e-3;
                }
            }
        }
    }

    for (int i = 0; i < 3; i++) // position velocity limit
    {
        lbA(i) = -1;
        ubA(i) = 1;
    }

    for (int i = 3; i < 6; i++) // angular velocity limit
    {
        lbA(i) = -3;
        ubA(i) = 3;
    }

    QP_qdot_rarm.EnableEqualityCondition(equality_condition_eps_);
    QP_qdot_rarm.UpdateMinProblem(H, g);
    QP_qdot_rarm.UpdateSubjectToAx(A, lbA, ubA);
    QP_qdot_rarm.UpdateSubjectToX(lb, ub);

    VectorXd q_dot_rarm;

    if (QP_qdot_rarm.SolveQPoases(100, qpres))
    {
        q_dot_rarm = qpres.segment(0, variable_size);
    }
    else
    {
        q_dot_rarm.setZero(variable_size);
    }

    for (int i = 0; i < variable_size; i++)
    {
        motion_q_dot_(25 + i) = q_dot_rarm(i);
        motion_q_(25 + i) = motion_q_pre_(25 + i) + motion_q_dot_(25 + i) * dt_;
        pd_control_mask_(25 + i) = 1;
    }

    if (int(current_time_ * 1e4) % int(1e3) == 0)
    {
        // cout<<"J1: \n"<<J_l_arm<<endl;
        // cout<<"N1: \n"<<N1<<endl;
        // cout<<"N2: \n"<<N_2<<endl;
        // cout<<"J2N1: \n"<<J2N1<<endl;
        // cout<<"J2N1_pinv: \n"<<J2N1_pinv<<endl;
        // cout<<"J3N1N2: \n"<<J3N1N2<<endl;
        // cout<<"J3N1N2_pinv: \n"<<J3N1N2_pinv<<endl;
        // cout<<"right 1st task error: \n"<< J_r_arm*q_dot_rarm - u_dot_rhand<<endl;
        // cout<<"right 2nd task error: \n"<< J_r_upperarm*q_dot_rarm - u_dot_rupperarm<<endl;
        // cout<<"right 3rd task error: \n"<< J_r_shoulder*q_dot_rarm - u_dot_rshoulder<<endl;
        // cout<<"qdot2: \n"<<(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
        // cout<<"N1 X qdot2: \n"<<N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
        // cout<<"J2 X N1 X qdot2: \n"<<J_l_upperarm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
        // cout<<"J1 X N1 X qdot2: \n"<<J_l_arm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
    }

    rhand_vel_error_ = J_r_arm * q_dot_rarm - u_dot_rhand;
    relbow_vel_error_ = J_r_upperarm * q_dot_rarm - u_dot_rupperarm;
    racromion_vel_error_ = J_r_shoulder * q_dot_rarm - u_dot_rshoulder;

    // motion_q_(25) = -0.3;
    // pd_control_mask_(25) = 1;

    // motion_q_(19) = DyrosMath::minmax_cut(motion_q_(19), joint_limit_l_(4), joint_limit_h_(4));		//elbow
    // motion_q_(29) = DyrosMath::minmax_cut(motion_q_(29), joint_limit_l_(12), joint_limit_h_(12));
    /////////////////////////////////////////////////////////////////////////////////////////////
}

void AvatarController::motionRetargeting_QPIK_upperbody()
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (first_loop_upperbody_)
    {
        QP_qdot_upperbody_.InitializeProblemSize(variable_size_upperbody_, constraint_size2_upperbody_);

        for (int i = 0; i < hierarchy_num_upperbody_; i++)
        {
            J_upperbody_[i].setZero(control_size_upperbody_[i], variable_size_upperbody_);
            u_dot_upperbody_[i].setZero(control_size_upperbody_[i]);
        }

        ubA_upperbody_.setZero(constraint_size2_upperbody_);
        lbA_upperbody_.setZero(constraint_size2_upperbody_);

        H_upperbody_.setZero(variable_size_upperbody_, variable_size_upperbody_);
        g_upperbody_.setZero(variable_size_upperbody_);

        ub_upperbody_.setZero(constraint_size1_upperbody_);
        lb_upperbody_.setZero(constraint_size1_upperbody_);

        q_dot_upperbody_.setZero(variable_size_upperbody_);

        first_loop_upperbody_ = false;

        w1_upperbody_ = 2500;  // upperbody tracking(5000)
        w2_upperbody_ = 2500;  // hand & head(2500)
        w3_upperbody_ = 2500;  // upperarm(50)
        w4_upperbody_ = 2500;  // shoulder(1)
        w5_upperbody_ = 0;     // kinematic energy(50)
        w6_upperbody_ = 0.000; // acceleration(0.002)

        N1_upperbody_.setZero(variable_size_upperbody_, variable_size_upperbody_);
        N2_aug_upperbody_.setZero(variable_size_upperbody_, variable_size_upperbody_);
        N3_aug_upperbody_.setZero(variable_size_upperbody_, variable_size_upperbody_);
        J2_aug_upperbody_.setZero(control_size_upperbody_[0] + control_size_upperbody_[1], variable_size_upperbody_);
        J2_aug_pinv_upperbody_.setZero(variable_size_upperbody_, control_size_upperbody_[0] + control_size_upperbody_[1]);
        J3_aug_upperbody_.setZero(control_size_upperbody_[0] + control_size_upperbody_[1] + control_size_upperbody_[2], variable_size_upperbody_);
        J3_aug_pinv_upperbody_.setZero(variable_size_upperbody_, control_size_upperbody_[0] + control_size_upperbody_[1] + control_size_upperbody_[2]);
        J2N1_upperbody_.setZero(control_size_upperbody_[1], variable_size_upperbody_);
        J3N2_aug_upperbody_.setZero(control_size_upperbody_[2], variable_size_upperbody_);
        J4N3_aug_upperbody_.setZero(control_size_upperbody_[3], variable_size_upperbody_);

        I3_upperbody_.setIdentity(3, 3);
        I6_upperbody_.setIdentity(6, 6);
        I15_upperbody_.setIdentity(15, 15);
        I21_upperbody_.setIdentity(21, 21);
    }

    VectorXd qpres;

    // VectorQVQd q_desired_pre;
    // q_desired_pre.setZero();
    // q_desired_pre(39) = 1;
    // q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;

    // Upperbody
    Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());

    // Hand error
    Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
    Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
    Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
    Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());

    // Head error
    Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
    error_w_head = head_transform_pre_desired_from_.linear().transpose() * error_w_head;
    error_w_head(0) = 0;
    error_w_head = head_transform_pre_desired_from_.linear() * error_w_head;

    // Upperarm error
    Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    error_w_lupperarm(0) = 0;
    Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    error_w_rupperarm(0) = 0;

    // Shoulder error
    Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
    error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
    error_w_lshoulder(0) = 0;
    Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
    error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
    error_w_rshoulder(0) = 0;

    u_dot_upperbody_[0] = 100 * error_w_upperbody;

    u_dot_upperbody_[1].segment(0, 3) = 200 * error_v_lhand;
    u_dot_upperbody_[1].segment(3, 3) = 100 * error_w_lhand;
    u_dot_upperbody_[1].segment(6, 3) = 200 * error_v_rhand;
    u_dot_upperbody_[1].segment(9, 3) = 100 * error_w_rhand;
    u_dot_upperbody_[1].segment(12, 2) = 100 * error_w_head.segment(1, 2);

    u_dot_upperbody_[2].segment(0, 2) = 100 * error_w_lupperarm.segment(1, 2);
    u_dot_upperbody_[2].segment(2, 2) = 100 * error_w_rupperarm.segment(1, 2);

    u_dot_upperbody_[3].segment(0, 2) = 50 * error_w_lshoulder.segment(1, 2);
    u_dot_upperbody_[3].segment(2, 2) = 50 * error_w_rshoulder.segment(1, 2);

    Vector3d zero3;
    zero3.setZero();
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);

    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Upper_Body].id, zero3, J_temp_, false);
    J_upperbody_[0].block(0, 0, 3, variable_size_upperbody_) = J_temp_.block(0, 18, 3, variable_size_upperbody_); // orientation

    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand].id, lhand_control_point_offset_, J_temp_, false);
    J_upperbody_[1].block(0, 0, 3, variable_size_upperbody_) = J_temp_.block(3, 18, 3, variable_size_upperbody_); // position
    J_upperbody_[1].block(3, 0, 3, variable_size_upperbody_) = J_temp_.block(0, 18, 3, variable_size_upperbody_); // orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand].id, rhand_control_point_offset_, J_temp_, false);
    J_upperbody_[1].block(6, 0, 3, variable_size_upperbody_) = J_temp_.block(3, 18, 3, variable_size_upperbody_); // position
    J_upperbody_[1].block(9, 0, 3, variable_size_upperbody_) = J_temp_.block(0, 18, 3, variable_size_upperbody_); // orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Head].id, zero3, J_temp_, false);
    J_upperbody_[1].block(12, 0, 2, variable_size_upperbody_) = (head_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); // orientation

    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 4].id, zero3, J_temp_, false);
    J_upperbody_[2].block(0, 0, 2, variable_size_upperbody_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); // orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 4].id, zero3, J_temp_, false);
    J_upperbody_[2].block(2, 0, 2, variable_size_upperbody_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); // orientation

    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 6].id, zero3, J_temp_, false);
    J_upperbody_[3].block(0, 0, 2, variable_size_upperbody_) = (lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); // orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 6].id, zero3, J_temp_, false);
    J_upperbody_[3].block(2, 0, 2, variable_size_upperbody_) = (racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); // orientation

    //// null space projection of tasks for strict hierarchy////

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    J1_pinv_upperbody_ = J_upperbody_[0].transpose() * (J_upperbody_[0] * J_upperbody_[0].transpose() + I3_upperbody_ * damped_puedoinverse_eps_).inverse();
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    N1_upperbody_ = I21_upperbody_ - (J1_pinv_upperbody_)*J_upperbody_[0];

    J2_aug_upperbody_.block(0, 0, control_size_upperbody_[0], variable_size_upperbody_) = J_upperbody_[0];
    J2_aug_upperbody_.block(control_size_upperbody_[0], 0, control_size_upperbody_[1], variable_size_upperbody_) = J_upperbody_[1];

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    J2_aug_pinv_upperbody_ = J2_aug_upperbody_.transpose() * (J2_aug_upperbody_ * J2_aug_upperbody_.transpose() + Eigen::MatrixXd::Identity(17, 17) * damped_puedoinverse_eps_).inverse();
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    N2_aug_upperbody_ = I21_upperbody_ - (J2_aug_pinv_upperbody_)*J2_aug_upperbody_;

    J3_aug_upperbody_.block(0, 0, control_size_upperbody_[0] + control_size_upperbody_[1], variable_size_upperbody_) = J2_aug_upperbody_;
    J3_aug_upperbody_.block(control_size_upperbody_[0] + control_size_upperbody_[1], 0, control_size_upperbody_[2], variable_size_upperbody_) = J_upperbody_[2];
    std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
    J3_aug_pinv_upperbody_ = J3_aug_upperbody_.transpose() * (J3_aug_upperbody_ * J3_aug_upperbody_.transpose() + Eigen::MatrixXd::Identity(21, 21) * damped_puedoinverse_eps_).inverse();
    std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
    N3_aug_upperbody_ = I21_upperbody_ - (J3_aug_pinv_upperbody_)*J3_aug_upperbody_;

    J2N1_upperbody_ = J_upperbody_[1] * N1_upperbody_;
    J3N2_aug_upperbody_ = J_upperbody_[2] * N2_aug_upperbody_;
    J4N3_aug_upperbody_ = J_upperbody_[3] * N3_aug_upperbody_;

    MatrixXd H1, H2, H3, H4, H5, H6;
    VectorXd g1, g2, g3, g4, g5, g6;

    H1 = J_upperbody_[0].transpose() * J_upperbody_[0];
    H2 = (J2N1_upperbody_).transpose() * (J2N1_upperbody_);
    H3 = (J3N2_aug_upperbody_).transpose() * (J3N2_aug_upperbody_);
    H4 = (J4N3_aug_upperbody_).transpose() * (J4N3_aug_upperbody_);
    H5 = A_mat_.block(18, 18, variable_size_upperbody_, variable_size_upperbody_);
    H6 = I21_upperbody_ * (1 / dt_) * (1 / dt_);

    g1 = -J_upperbody_[0].transpose() * u_dot_upperbody_[0];
    g2 = -(J2N1_upperbody_).transpose() * (u_dot_upperbody_[1] - J_upperbody_[1] * motion_q_dot_pre_.segment(12, variable_size_upperbody_));
    g3 = -(J3N2_aug_upperbody_).transpose() * (u_dot_upperbody_[2] - J_upperbody_[2] * motion_q_dot_pre_.segment(12, variable_size_upperbody_));
    g4 = -(J4N3_aug_upperbody_).transpose() * (u_dot_upperbody_[3] - J_upperbody_[3] * motion_q_dot_pre_.segment(12, variable_size_upperbody_));
    g5.setZero(variable_size_upperbody_, 1);
    g6 = -motion_q_dot_pre_.segment(12, variable_size_upperbody_) * (1 / dt_) * (1 / dt_);

    H_upperbody_ = w1_upperbody_ * H1 + w2_upperbody_ * H2 + w3_upperbody_ * H3 + w4_upperbody_ * H4 + w5_upperbody_ * H5 + w6_upperbody_ * H6;
    g_upperbody_ = w1_upperbody_ * g1 + w2_upperbody_ * g2 + w3_upperbody_ * g3 + w4_upperbody_ * g4 + w5_upperbody_ * g5 + w6_upperbody_ * g6;

    double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

    for (int i = 0; i < constraint_size1_upperbody_; i++)
    {
        lb_upperbody_(i) = max(speed_reduce_rate * (joint_limit_l_(i + 12) - current_q_(i + 12)), joint_vel_limit_l_(i + 12));
        ub_upperbody_(i) = min(speed_reduce_rate * (joint_limit_h_(i + 12) - current_q_(i + 12)), joint_vel_limit_h_(i + 12));
    }

    A_upperbody_ = J_upperbody_[1].block(0, 0, constraint_size2_upperbody_, variable_size_upperbody_);

    // for(int i = 0; i<constraint_size2_upperbody_; i++)
    // {
    // 	for(int j=0 ;j<variable_size; j++)
    // 	{
    // 		if( abs(A(i, j)) < 1e-3)
    // 		{
    // 			if(A(i,j) > 0)
    // 			{
    // 				A(i, j) = 1e-3;
    // 			}
    // 			else
    // 			{
    // 				A(i, j) = -1e-3;
    // 			}
    // 		}
    // 	}
    // }

    for (int i = 0; i < 3; i++) // linear velocity limit
    {
        lbA_upperbody_(i) = -2;
        ubA_upperbody_(i) = 2;
        lbA_upperbody_(i + 6) = -2;
        ubA_upperbody_(i + 6) = 2;
    }

    for (int i = 0; i < 3; i++) // angular velocity limit
    {
        lbA_upperbody_(i + 3) = -3;
        ubA_upperbody_(i + 3) = 3;
        lbA_upperbody_(i + 9) = -3;
        ubA_upperbody_(i + 9) = 3;
    }

    QP_qdot_upperbody_.EnableEqualityCondition(equality_condition_eps_);
    QP_qdot_upperbody_.UpdateMinProblem(H_upperbody_, g_upperbody_);
    QP_qdot_upperbody_.UpdateSubjectToAx(A_upperbody_, lbA_upperbody_, ubA_upperbody_);
    QP_qdot_upperbody_.UpdateSubjectToX(lb_upperbody_, ub_upperbody_);

    VectorXd q_dot_upperbody;
    std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
    if (QP_qdot_upperbody_.SolveQPoases(100, qpres))
    {
        q_dot_upperbody = qpres.segment(0, variable_size_upperbody_);
    }
    else
    {
        QP_qdot_upperbody_.InitializeProblemSize(variable_size_upperbody_, constraint_size2_upperbody_);
        q_dot_upperbody.setZero(variable_size_upperbody_);

        if (int(current_time_ * 10000) % 1000 == 0)
            cout << "qpik_upperarm is not solved!!" << endl;
    }
    std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();
    for (int i = 0; i < variable_size_upperbody_; i++)
    {
        motion_q_dot_(12 + i) = q_dot_upperbody(i);
        motion_q_(12 + i) = motion_q_pre_(12 + i) + motion_q_dot_(12 + i) * dt_;
        pd_control_mask_(12 + i) = 1;
    }

    std::chrono::steady_clock::time_point t10 = std::chrono::steady_clock::now();

    lhand_vel_error_ = J_upperbody_[1].block(0, 0, 6, variable_size_upperbody_) * q_dot_upperbody - u_dot_upperbody_[1].segment(0, 6);
    lelbow_vel_error_.segment(1, 2) = J_upperbody_[2].block(0, 0, 2, variable_size_upperbody_) * q_dot_upperbody - u_dot_upperbody_[2].segment(0, 2);
    lacromion_vel_error_.segment(1, 2) = J_upperbody_[3].block(0, 0, 2, variable_size_upperbody_) * q_dot_upperbody - u_dot_upperbody_[3].segment(0, 2);

    rhand_vel_error_ = J_upperbody_[1].block(6, 0, 6, variable_size_upperbody_) * q_dot_upperbody - u_dot_upperbody_[1].segment(6, 6);
    relbow_vel_error_.segment(1, 2) = J_upperbody_[2].block(2, 0, 2, variable_size_upperbody_) * q_dot_upperbody - u_dot_upperbody_[2].segment(2, 2);
    racromion_vel_error_.segment(1, 2) = J_upperbody_[3].block(2, 0, 2, variable_size_upperbody_) * q_dot_upperbody - u_dot_upperbody_[3].segment(2, 2);

    lhand_pos_error_ = master_lhand_pose_pre_.translation() - lhand_transform_pre_desired_from_.translation();
    rhand_pos_error_ = master_rhand_pose_pre_.translation() - rhand_transform_pre_desired_from_.translation();

    Eigen::AngleAxisd lhand_pos_error_aa(master_lhand_pose_pre_.linear() * lhand_transform_pre_desired_from_.linear().transpose());
    lhand_ori_error_ = lhand_pos_error_aa.axis() * lhand_pos_error_aa.angle();
    Eigen::AngleAxisd rhand_pos_error_aa(master_rhand_pose_pre_.linear() * rhand_transform_pre_desired_from_.linear().transpose());
    rhand_ori_error_ = rhand_pos_error_aa.axis() * rhand_pos_error_aa.angle();

    Eigen::AngleAxisd lelbow_ori_error_aa(master_lelbow_pose_pre_.linear() * lupperarm_transform_pre_desired_from_.linear().transpose());
    lelbow_ori_error_ = lelbow_ori_error_aa.axis() * lelbow_ori_error_aa.angle();
    Eigen::AngleAxisd relbow_ori_error_aa(master_relbow_pose_pre_.linear() * rupperarm_transform_pre_desired_from_.linear().transpose());
    relbow_ori_error_ = relbow_ori_error_aa.axis() * relbow_ori_error_aa.angle();

    Eigen::AngleAxisd lshoulder_ori_error_aa(master_lshoulder_pose_pre_.linear() * lacromion_transform_pre_desired_from_.linear().transpose());
    lshoulder_ori_error_ = lshoulder_ori_error_aa.axis() * lshoulder_ori_error_aa.angle();

    Eigen::AngleAxisd rshoulder_ori_error_aa(master_rshoulder_pose_pre_.linear() * racromion_transform_pre_desired_from_.linear().transpose());
    rshoulder_ori_error_ = rshoulder_ori_error_aa.axis() * rshoulder_ori_error_aa.angle();
    if (int(current_time_ * 1e4) % int(1e4) == 0)
    {
        // 	cout<<"t2 - t1: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
        // 	cout<<"t3 - t2: "<< std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() <<endl;
        // 	cout<<"t4 - t3: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() <<endl;
        // 	cout<<"t5 - t4: "<< std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() <<endl;
        // 	cout<<"t6 - t5: "<< std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() <<endl;
        // 	cout<<"t7 - t6: "<< std::chrono::duration_cast<std::chrono::microseconds>(t7 - t6).count() <<endl;
        // 	cout<<"t8 - t7: "<< std::chrono::duration_cast<std::chrono::microseconds>(t8 - t7).count() <<endl;
        // 	cout<<"t9 - t8: "<< std::chrono::duration_cast<std::chrono::microseconds>(t9 - t8).count() <<endl;
        cout << "motionRetargeting_QPIK_upperbody computing time: " << std::chrono::duration_cast<std::chrono::microseconds>(t10 - t1).count() << endl;
    }
}

// void AvatarController::motionRetargeting_QPIK_wholebody()
// {
//     const int variable_size = 33;
//     const int constraint_size1 = 33;
// }

void AvatarController::motionRetargeting_HQPIK()
{
    // const unsigned int hierarchy_num_hqpik_ = 3;
    // const unsigned int variable_size_hqpik_ = 21;
	// const unsigned int constraint_size1_hqpik_ = 21;	//[lb <=	x	<= 	ub] form constraints
	// const unsigned int constraint_size2_hqpik_[3] = {12, 15, 17};	//[lb <=	Ax 	<=	ub] or [Ax = b]
	// const unsigned int control_size_hqpik_[3] = {3, 14, 8};		//1: upperbody, 2: head + hand, 3: upperarm + shoulder AAC

    if (first_loop_hqpik_)
    {
        for (int i = 0; i < hierarchy_num_hqpik_; i++)
        {
            QP_qdot_hqpik_.resize(hierarchy_num_hqpik_);
            QP_qdot_hqpik_[i].InitializeProblemSize(variable_size_hqpik_, constraint_size2_hqpik_[i]);
            J_hqpik_[i].setZero(control_size_hqpik_[i], variable_size_hqpik_);
            u_dot_hqpik_[i].setZero(control_size_hqpik_[i]);

            ubA_hqpik_[i].setZero(constraint_size2_hqpik_[i]);
            lbA_hqpik_[i].setZero(constraint_size2_hqpik_[i]);

            H_hqpik_[i].setZero(variable_size_hqpik_, variable_size_hqpik_);
            g_hqpik_[i].setZero(variable_size_hqpik_);

            ub_hqpik_[i].setZero(constraint_size1_hqpik_);
            lb_hqpik_[i].setZero(constraint_size1_hqpik_);

            q_dot_hqpik_[i].setZero(variable_size_hqpik_);

            w1_hqpik_[i] = 2500;  // upperbody tracking (2500)
            w2_hqpik_[i] = 50;    // kinetic energy (50)
            w3_hqpik_[i] = 0.000; // acceleration (0.000)
        }

        // upper arm & shoulder orientation control gain
        w1_hqpik_[2] = 250;   // upperbody tracking (250)
        w2_hqpik_[2] = 50;    // kinetic energy (50)
        w3_hqpik_[2] = 0.001; // acceleration (0.002)

        // shoulder orientation control gain
        // w1_hqpik_[3] = 250;   // upperbody tracking (250)
        // w2_hqpik_[3] = 50;    // kinetic energy (50)
        // w3_hqpik_[3] = 0.001; // acceleration (0.002)

        last_solved_hierarchy_num_ = 2;

        first_loop_hqpik_ = false;
    }

    // VectorQVQd q_desired_pre;
    // q_desired_pre.setZero();
    // q_desired_pre(39) = 1;
    // q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;
    Vector3d zero3;
    zero3.setZero();
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);

    ////1st Task
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Upper_Body].id, zero3, J_temp_, true);
    J_hqpik_[0].block(0, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); // orientation

    Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());
    u_dot_hqpik_[0] = 100 * error_w_upperbody;

    ////2nd Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand].id, lhand_control_point_offset_, J_temp_, false);
    J_hqpik_[1].block(0, 0, 3, variable_size_hqpik_) = J_temp_.block(3, 18, 3, variable_size_hqpik_); // position
    J_hqpik_[1].block(3, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); // orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand].id, rhand_control_point_offset_, J_temp_, false);
    J_hqpik_[1].block(6, 0, 3, variable_size_hqpik_) = J_temp_.block(3, 18, 3, variable_size_hqpik_); // position
    J_hqpik_[1].block(9, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); // orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Head].id, zero3, J_temp_, false);
    J_hqpik_[1].block(12, 0, 2, variable_size_hqpik_) = (head_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); // orientation

    // Hand error
    Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
    Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
    Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
    Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());

    // Head error
    Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
    error_w_head = head_transform_pre_desired_from_.linear().transpose() * error_w_head;
    error_w_head(0) = 0;

    u_dot_hqpik_[1].segment(0, 3) = 200 * error_v_lhand;
    u_dot_hqpik_[1].segment(3, 3) = 100 * error_w_lhand;
    u_dot_hqpik_[1].segment(6, 3) = 200 * error_v_rhand;
    u_dot_hqpik_[1].segment(9, 3) = 100 * error_w_rhand;
    u_dot_hqpik_[1].segment(12, 2) = 200 * error_w_head.segment(1, 2);

    ////3rd Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik_[2].block(0, 0, 2, variable_size_hqpik_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); // orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);

    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik_[2].block(2, 0, 2, variable_size_hqpik_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); // orientation

    // Upperarm error
    Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    error_w_lupperarm(0) = 0;

    Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    error_w_rupperarm(0) = 0;

    u_dot_hqpik_[2].segment(0, 2) = 100 * error_w_lupperarm.segment(1, 2);
    u_dot_hqpik_[2].segment(2, 2) = 100 * error_w_rupperarm.segment(1, 2);


    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik_[2].block(4, 0, 2, variable_size_hqpik_) = (lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); // orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik_[2].block(6, 0, 2, variable_size_hqpik_) = (racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); // orientation

    // Shoulder error
    Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
    error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
    error_w_lshoulder(0) = 0;

    Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
    error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
    error_w_rshoulder(0) = 0;

    u_dot_hqpik_[2].segment(4, 2) = 100 * error_w_lshoulder.segment(1, 2);
    u_dot_hqpik_[2].segment(6, 2) = 100 * error_w_rshoulder.segment(1, 2);

    for (int i = 0; i < hierarchy_num_hqpik_; i++)
    {
        if (i > last_solved_hierarchy_num_)
        {
            QP_qdot_hqpik_[i].InitializeProblemSize(variable_size_hqpik_, constraint_size2_hqpik_[i]);
        }

        if(last_solved_hierarchy_num_ == 0)
        {
            QP_qdot_hqpik_[0].InitializeProblemSize(variable_size_hqpik_, constraint_size2_hqpik_[0]);
        }
    }

    last_solved_hierarchy_num_ = -1;

    for (int i = 0; i < hierarchy_num_hqpik_; i++)
    {
        MatrixXd H1, H2, H3;
        VectorXd g1, g2, g3;

        H1 = J_hqpik_[i].transpose() * J_hqpik_[i];
        H2 = Eigen::MatrixXd::Identity(variable_size_hqpik_, variable_size_hqpik_);
        // H2 = A_mat_.block(18, 18, variable_size_hqpik_, variable_size_hqpik_) + Eigen::MatrixXd::Identity(variable_size_hqpik_, variable_size_hqpik_) * (2e-2);
        H2(3, 3) += 10;   // left arm 1st joint
        H2(13, 13) += 10; // right arm 1st joint
        H3 = Eigen::MatrixXd::Identity(variable_size_hqpik_, variable_size_hqpik_) * (2000) * (2000);

        g1 = -J_hqpik_[i].transpose() * u_dot_hqpik_[i];
        g2.setZero(variable_size_hqpik_);
        g3 = -motion_q_dot_pre_.segment(12, variable_size_hqpik_) * (2000) * (2000);

        if (i >= 2)
        {
        }

        H_hqpik_[i] = w1_hqpik_[i] * H1 + w2_hqpik_[i] * H2 + w3_hqpik_[i] * H3;
        g_hqpik_[i] = w1_hqpik_[i] * g1 + w2_hqpik_[i] * g2 + w3_hqpik_[i] * g3;

        double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

        for (int j = 0; j < constraint_size1_hqpik_; j++)
        {
            lb_hqpik_[i](j) = min(max(speed_reduce_rate * (joint_limit_l_(j + 12) - motion_q_pre_(j + 12)), joint_vel_limit_l_(j + 12)), joint_vel_limit_h_(j + 12));
            ub_hqpik_[i](j) = max(min(speed_reduce_rate * (joint_limit_h_(j + 12) - motion_q_pre_(j + 12)), joint_vel_limit_h_(j + 12)), joint_vel_limit_l_(j + 12));
        }

        A_hqpik_[i].setZero(constraint_size2_hqpik_[i], variable_size_hqpik_);

        int higher_task_equality_num = 0;
        for (int h = 0; h < i; h++)
        {
            A_hqpik_[i].block(higher_task_equality_num, 0, control_size_hqpik_[h], variable_size_hqpik_) = J_hqpik_[h];
            // ubA_hqpik_[i].segment(higher_task_equality_num, control_size_hqpik_[h]) = J_hqpik_[h] * q_dot_hqpik_[h];
            // lbA_hqpik_[i].segment(higher_task_equality_num, control_size_hqpik_[h]) = J_hqpik_[h] * q_dot_hqpik_[h];
            ubA_hqpik_[i].segment(higher_task_equality_num, control_size_hqpik_[h]) = J_hqpik_[h] * q_dot_hqpik_[i - 1];
            lbA_hqpik_[i].segment(higher_task_equality_num, control_size_hqpik_[h]) = J_hqpik_[h] * q_dot_hqpik_[i - 1];
            higher_task_equality_num += control_size_hqpik_[h];
        }

        // hand velocity constraints
        if (i < 2)
        {
            A_hqpik_[i].block(higher_task_equality_num, 0, 12, variable_size_hqpik_) = J_hqpik_[1].block(0, 0, 12, variable_size_hqpik_);

            for (int j = 0; j < 3; j++)
            {
                // linear velocity limit
                lbA_hqpik_[i](higher_task_equality_num + j) = -0.8;
                ubA_hqpik_[i](higher_task_equality_num + j) = 0.8;
                lbA_hqpik_[i](higher_task_equality_num + j + 6) = -0.8;
                ubA_hqpik_[i](higher_task_equality_num + j + 6) = 0.8;

                // angular velocity limit
                lbA_hqpik_[i](higher_task_equality_num + j + 3) = -2*M_PI;
                ubA_hqpik_[i](higher_task_equality_num + j + 3) = 2*M_PI;
                lbA_hqpik_[i](higher_task_equality_num + j + 9) = -2*M_PI;
                ubA_hqpik_[i](higher_task_equality_num + j + 9) = 2*M_PI;
            }
        }

        // QP_qdot_hqpik_[i].SetPrintLevel(PL_NONE);
        QP_qdot_hqpik_[i].EnableEqualityCondition(equality_condition_eps_);
        QP_qdot_hqpik_[i].UpdateMinProblem(H_hqpik_[i], g_hqpik_[i]);
        QP_qdot_hqpik_[i].UpdateSubjectToAx(A_hqpik_[i], lbA_hqpik_[i], ubA_hqpik_[i]);
        QP_qdot_hqpik_[i].UpdateSubjectToX(lb_hqpik_[i], ub_hqpik_[i]);

        // cout<<"test8"<<endl;
        if (QP_qdot_hqpik_[i].SolveQPoases(200, qpres_hqpik_))
        {
            q_dot_hqpik_[i] = qpres_hqpik_.segment(0, variable_size_hqpik_);

            last_solved_hierarchy_num_ = i;

            // if(i == 3)
            // {
            //     if (int(current_time_ * 10000) % 1000 == 0)
            //         std::cout << "4th HQPIK(shoulder) is solved" << std::endl;
            // }
        }
        else
        {
            q_dot_hqpik_[i].setZero();

            // last_solved_hierarchy_num_ = max(i-1, 0);

            last_solved_hierarchy_num_ = DyrosMath::minmax_cut(last_solved_hierarchy_num_, 0, hierarchy_num_hqpik_-1);
            
            if (i == 0)
            {
                if(true)
                {
                    std::cout << "Error hierarchy: " << 0 << std::endl;
                    std::cout << "last solved q_dot: " << q_dot_hqpik_[0].transpose() << std::endl;
                }
            }
            else
            {
                if(last_solved_hierarchy_num_ < 0)
                {
                    std::cout<<"last_solved_hierarchy_num_ is negative!! "<< std::endl;
                }
                // if (int(current_time_ * 2000) % 1000 == 0)
                if(true)
                {
                    std::cout << "Error hierarchy: " << i << std::endl;
                    std::cout << "last solved q_dot: " << q_dot_hqpik_[last_solved_hierarchy_num_].transpose() << std::endl;
                }
            }
            // cout<<"Error qpres_: \n"<< qpres_ << endl;
            break;
        }
    }
    
    if(last_solved_hierarchy_num_ < 0)
    {
        std::cout<<"last_solved_hierarchy_num_ is negative!! "<< std::endl;
    }

    for (int i = 0; i < variable_size_hqpik_; i++)
    {
        motion_q_dot_(12 + i) = q_dot_hqpik_[last_solved_hierarchy_num_](i);
        motion_q_(12 + i) = motion_q_pre_(12 + i) + motion_q_dot_(12 + i) * dt_;
        pd_control_mask_(12 + i) = 1;
    }
}

void AvatarController::motionRetargeting_HQPIK2()
{
    // const int hierarchy_num_hqpik2_ = 5;
    // const int variable_size_hqpik2_ = 21;
    // const int constraint_size1_hqpik2_ = 21;	//[lb <=	x	<= 	ub] form constraints
    // const int constraint_size2_hqpik2_[5] = {12, 15, 17, 21};	//[lb <=	Ax 	<=	ub] or [Ax = b]
    // const int control_size_hqpik2_[5] = {4, 3, 12, 4, 4};		//1: head ori(2)+pos(2), 2: upper body ori, 3: hand, 4: upper arm ori(2) 4: shoulder ori(2)

    if (first_loop_hqpik2_)
    {
        for (int i = 0; i < hierarchy_num_hqpik2_; i++)
        {
            QP_qdot_hqpik2_.resize(hierarchy_num_hqpik2_);
            QP_qdot_hqpik2_[i].InitializeProblemSize(variable_size_hqpik2_, constraint_size2_hqpik2_[i]);
            J_hqpik2_[i].setZero(control_size_hqpik2_[i], variable_size_hqpik2_);
            u_dot_hqpik2_[i].setZero(control_size_hqpik2_[i]);

            ubA_hqpik2_[i].setZero(constraint_size2_hqpik2_[i]);
            lbA_hqpik2_[i].setZero(constraint_size2_hqpik2_[i]);

            H_hqpik2_[i].setZero(variable_size_hqpik2_, variable_size_hqpik2_);
            g_hqpik2_[i].setZero(variable_size_hqpik2_);

            ub_hqpik2_[i].setZero(constraint_size1_hqpik2_);
            lb_hqpik2_[i].setZero(constraint_size1_hqpik2_);

            q_dot_hqpik2_[i].setZero(variable_size_hqpik2_);

            w1_hqpik2_[i] = 2500;  // upperbody tracking (2500)
            w2_hqpik2_[i] = 50;    // kinetic energy (50)
            w3_hqpik2_[i] = 0.000; // acceleration ()

            
        }

        // upper arm orientation control gain
        w1_hqpik_[2] = 250;   // upperbody tracking (250)
        w2_hqpik_[2] = 50;    // kinetic energy (50)
        w3_hqpik_[2] = 0.001; // acceleration (0.002)

        // // upper arm orientation control gain
        // w1_hqpik2_[3] = 250;   // upperbody tracking (2500)
        // w2_hqpik2_[3] = 50;    // kinetic energy (50)
        // w3_hqpik2_[3] = 0.002; // acceleration ()

        // // shoulder orientation control gain
        // w1_hqpik2_[4] = 250;   // upperbody tracking (2500)
        // w2_hqpik2_[4] = 50;    // kinetic energy (50)
        // w3_hqpik2_[4] = 0.002; // acceleration ()

        last_solved_hierarchy_num_ = -1;

        first_loop_hqpik2_ = false;

        // nominal_q_pose_.setZero();
        // // left arm zero pose
        
        // nominal_q_pose_(15) = 0.3;
        // nominal_q_pose_(16) = 0.12;
        // nominal_q_pose_(17) = 1.43;
        // nominal_q_pose_(18) = -0.85;
        // nominal_q_pose_(19) = -0.45; // elbow
        // nominal_q_pose_(20) = 1.0;
        // nominal_q_pose_(21) = 0.0;
        // nominal_q_pose_(22) = 0.0;

        // nominal_q_pose_(25) = -0.3;
        // nominal_q_pose_(26) = -0.12;
        // nominal_q_pose_(27) = -1.43;
        // nominal_q_pose_(28) = 0.85;
        // nominal_q_pose_(29) = 0.45; // elbow
        // nominal_q_pose_(30) = -1.0;
        // nominal_q_pose_(31) = 0.0;
        // nominal_q_pose_(32) = 0.0;
    }
    // VectorQVQd q_desired_pre;
    // q_desired_pre.setZero();
    // q_desired_pre(39) = 1;
    // q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;
    Vector3d zero3;
    zero3.setZero();

    ////1st Task
    J_hqpik2_[0].setZero();
    u_dot_hqpik2_[0].setZero();

    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Head].id, zero3, J_temp_, false);
    J_hqpik2_[0].block(2, 0, 2, variable_size_hqpik2_) = J_temp_.block(3, 18, 2, variable_size_hqpik2_);                                                                                                 // x, y position
    J_hqpik2_[0].block(0, 0, 2, variable_size_hqpik2_) = (head_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // y, z orientation
    //waist
    // J_hqpik2_[0](2, 0) = 1.0;
    // J_hqpik2_[0](3, 1) = 1.0;
    // J_hqpik2_[0](4, 2) = 1.0;

    // Head error
    Vector3d error_v_head = master_head_pose_.translation() - head_transform_pre_desired_from_.translation();
    Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
    error_w_head = head_transform_pre_desired_from_.linear().transpose() * error_w_head;
    error_w_head(0) = 0;
    u_dot_hqpik2_[0].segment(2, 2) = 100 * error_v_head.segment(0, 2);
    u_dot_hqpik2_[0].segment(0, 2) = 100 * error_w_head.segment(1, 2);
    
    // u_dot_hqpik2_[0](2) = 100*(0.0 - pre_desired_q_qvqd_(18));
    // u_dot_hqpik2_[0](3) = 100*(0.0 - pre_desired_q_qvqd_(19));
    // u_dot_hqpik2_[0](4) = 100*(0.0 - pre_desired_q_qvqd_(20));
    ////

    //// 2nd Task
    J_hqpik2_[1].setZero();
    u_dot_hqpik2_[1].setZero();

    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand].id, lhand_control_point_offset_, J_temp_, false);
    J_hqpik2_[1].block(0, 0, 3, variable_size_hqpik2_) = J_temp_.block(3, 18, 3, variable_size_hqpik2_); // position
    J_hqpik2_[1].block(3, 0, 3, variable_size_hqpik2_) = J_temp_.block(0, 18, 3, variable_size_hqpik2_); // orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand].id, rhand_control_point_offset_, J_temp_, false);
    J_hqpik2_[1].block(6, 0, 3, variable_size_hqpik2_) = J_temp_.block(3, 18, 3, variable_size_hqpik2_); // position
    J_hqpik2_[1].block(9, 0, 3, variable_size_hqpik2_) = J_temp_.block(0, 18, 3, variable_size_hqpik2_); // orientation
    // Hand error
    Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
    Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
    Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
    Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());
    u_dot_hqpik2_[1].segment(0, 3) = 100 * error_v_lhand;
    u_dot_hqpik2_[1].segment(3, 3) = 100 * error_w_lhand;
    u_dot_hqpik2_[1].segment(6, 3) = 100 * error_v_rhand;
    u_dot_hqpik2_[1].segment(9, 3) = 100 * error_w_rhand;
    ////

    //// 3rd Task
    J_hqpik2_[2].setZero();
    u_dot_hqpik2_[2].setZero();

    // J_hqpik2_[2].setIdentity();
    // for(int i = 0; i < control_size_hqpik2_[2]; i++)
    // {
    //     u_dot_hqpik2_[2](i) = 10*(nominal_q_pose_(12+i) - motion_q_pre_(12+i));
    //     u_dot_hqpik2_[2](i) = DyrosMath::minmax_cut(u_dot_hqpik2_[2](i), -M_PI/1, M_PI/1);
    // }
    
    // upperarm
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik2_[2].block(0, 0, 2, variable_size_hqpik2_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik2_[2].block(2, 0, 2, variable_size_hqpik2_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    // Upper arm error
    Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    error_w_lupperarm(0) = 0;
    Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    error_w_rupperarm(0) = 0;
    u_dot_hqpik2_[2].segment(0, 2) = 100 * error_w_lupperarm.segment(1, 2);
    u_dot_hqpik2_[2].segment(2, 2) = 100 * error_w_rupperarm.segment(1, 2);

    double shoulder_task_w = 1.0;

    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik2_[2].block(4, 0, 2, variable_size_hqpik2_) = shoulder_task_w*(lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik2_[2].block(6, 0, 2, variable_size_hqpik2_) = shoulder_task_w*(racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    // Shoulder error
    Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
    error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
    error_w_lshoulder(0) = 0;
    Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
    error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
    error_w_rshoulder(0) = 0;
    u_dot_hqpik2_[2].segment(4, 2) = 100 * shoulder_task_w*error_w_lshoulder.segment(1, 2);
    u_dot_hqpik2_[2].segment(6, 2) = 100 * shoulder_task_w*error_w_rshoulder.segment(1, 2);
    ////

    // ////3rd Task
    // J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    // RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Upper_Body].id, zero3, J_temp_, true);
    // J_hqpik2_[1].block(0, 0, 3, variable_size_hqpik2_) = J_temp_.block(0, 18, 3, variable_size_hqpik2_); // orientation
    // // upper body error
    // Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());
    // u_dot_hqpik2_[1] = 100 * error_w_upperbody;


    // ////4th Task
    // J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    // RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Left_Hand - 4].id, zero3, J_temp_, false);
    // J_hqpik2_[3].block(0, 0, 2, variable_size_hqpik2_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    // J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    // RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, link_avatar_[Right_Hand - 4].id, zero3, J_temp_, false);
    // J_hqpik2_[3].block(2, 0, 2, variable_size_hqpik2_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    // // Upper arm error
    // Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    // error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    // error_w_lupperarm(0) = 0;
    // Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    // error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    // error_w_rupperarm(0) = 0;
    // u_dot_hqpik2_[3].segment(0, 2) = 100 * error_w_lupperarm.segment(1, 2);
    // u_dot_hqpik2_[3].segment(2, 2) = 100 * error_w_rupperarm.segment(1, 2);

    // ////5th Task
    // J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    // RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, zero3, J_temp_, false);
    // J_hqpik2_[4].block(0, 0, 2, variable_size_hqpik2_) = (lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    // J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    // RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, zero3, J_temp_, false);
    // J_hqpik2_[4].block(2, 0, 2, variable_size_hqpik2_) = (racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); // orientation
    // // Shoulder error
    // Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
    // error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
    // error_w_lshoulder(0) = 0;
    // Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
    // error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
    // error_w_rshoulder(0) = 0;
    // u_dot_hqpik2_[4].segment(0, 2) = 100 * error_w_lshoulder.segment(1, 2);
    // u_dot_hqpik2_[4].segment(2, 2) = 100 * error_w_rshoulder.segment(1, 2);

    for (int i = 0; i < hierarchy_num_hqpik2_; i++)
    {
        if (i > last_solved_hierarchy_num_)
        {
            QP_qdot_hqpik2_[i].InitializeProblemSize(variable_size_hqpik2_, constraint_size2_hqpik2_[i]);
        }
    }

    last_solved_hierarchy_num_ = -1;
    for (int i = 0; i < hierarchy_num_hqpik2_; i++)
    {

        MatrixXd H1, H2, H3;
        VectorXd g1, g2, g3;

        H1 = J_hqpik2_[i].transpose() * J_hqpik2_[i];
        H2 = Eigen::MatrixXd::Identity(variable_size_hqpik2_, variable_size_hqpik2_);
        // H2 = A_mat_.block(18, 18, variable_size_hqpik2_, variable_size_hqpik2_) + Eigen::MatrixXd::Identity(variable_size_hqpik2_, variable_size_hqpik2_) * (2e-2);
        H2(3, 3) += 10;   // left arm 1st joint
        H2(13, 13) += 10; // right arm 1st joint
        H3 = Eigen::MatrixXd::Identity(variable_size_hqpik2_, variable_size_hqpik2_) * (2000) * (2000);

        g1 = -J_hqpik2_[i].transpose() * u_dot_hqpik2_[i];
        g2.setZero(variable_size_hqpik2_);
        g3 = -motion_q_dot_pre_.segment(12, variable_size_hqpik2_) * (2000) * (2000);

        if (i >= 2)
        {
        }

        H_hqpik2_[i] = w1_hqpik2_[i] * H1 + w2_hqpik2_[i] * H2 + w3_hqpik2_[i] * H3;
        g_hqpik2_[i] = w1_hqpik2_[i] * g1 + w2_hqpik2_[i] * g2 + w3_hqpik2_[i] * g3;

        double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

        for (int j = 0; j < constraint_size1_hqpik2_; j++)
        {
            lb_hqpik2_[i](j) = max(speed_reduce_rate * (joint_limit_l_(j + 12) - motion_q_pre_(j + 12)), joint_vel_limit_l_(j + 12));
            ub_hqpik2_[i](j) = min(speed_reduce_rate * (joint_limit_h_(j + 12) - motion_q_pre_(j + 12)), joint_vel_limit_h_(j + 12));
        }

        A_hqpik2_[i].setZero(constraint_size2_hqpik2_[i], variable_size_hqpik2_);

        int higher_task_equality_num = 0;
        for (int h = 0; h < i; h++)
        {
            A_hqpik2_[i].block(higher_task_equality_num, 0, control_size_hqpik2_[h], variable_size_hqpik2_) = J_hqpik2_[h];
            // ubA_hqpik2_[i].segment(higher_task_equality_num, control_size_hqpik2_[h]) = J_hqpik2_[h] * q_dot_hqpik2_[h];
            // lbA_hqpik2_[i].segment(higher_task_equality_num, control_size_hqpik2_[h]) = J_hqpik2_[h] * q_dot_hqpik2_[h];
            ubA_hqpik2_[i].segment(higher_task_equality_num, control_size_hqpik2_[h]) = J_hqpik2_[h] * q_dot_hqpik2_[i - 1];
            lbA_hqpik2_[i].segment(higher_task_equality_num, control_size_hqpik2_[h]) = J_hqpik2_[h] * q_dot_hqpik2_[i - 1];
            higher_task_equality_num += control_size_hqpik2_[h];
        }
        // hand velocity constraints
        if (i < 2)
        {
            A_hqpik2_[i].block(higher_task_equality_num, 0, 12, variable_size_hqpik2_) = J_hqpik2_[2].block(0, 0, 12, variable_size_hqpik2_);

            for (int j = 0; j < 3; j++)
            {
                double lin_vel_max = 0.8;
                double ang_vel_max = 2*M_PI;
                // linear velocity limit
                lbA_hqpik2_[i](higher_task_equality_num + j) = -lin_vel_max;
                ubA_hqpik2_[i](higher_task_equality_num + j) = lin_vel_max;
                lbA_hqpik2_[i](higher_task_equality_num + j + 6) = -lin_vel_max;
                ubA_hqpik2_[i](higher_task_equality_num + j + 6) = lin_vel_max;

                // angular velocity limit
                lbA_hqpik2_[i](higher_task_equality_num + j + 3) = -ang_vel_max;
                ubA_hqpik2_[i](higher_task_equality_num + j + 3) = ang_vel_max;
                lbA_hqpik2_[i](higher_task_equality_num + j + 9) = -ang_vel_max;
                ubA_hqpik2_[i](higher_task_equality_num + j + 9) = ang_vel_max;
            }
        }

        QP_qdot_hqpik2_[i].EnableEqualityCondition(equality_condition_eps_);
        QP_qdot_hqpik2_[i].UpdateMinProblem(H_hqpik2_[i], g_hqpik2_[i]);
        QP_qdot_hqpik2_[i].UpdateSubjectToAx(A_hqpik2_[i], lbA_hqpik2_[i], ubA_hqpik2_[i]);
        QP_qdot_hqpik2_[i].UpdateSubjectToX(lb_hqpik2_[i], ub_hqpik2_[i]);

        if (QP_qdot_hqpik2_[i].SolveQPoases(200, qpres_hqpik2_))
        {
            q_dot_hqpik2_[i] = qpres_hqpik2_.segment(0, variable_size_hqpik2_);

            last_solved_hierarchy_num_ = i;

            // if(i == 3)
            // {
            //     if (int(current_time_ * 10000) % 1000 == 0)
            //         std::cout << "4th HQPIK(shoulder) is solved" << std::endl;
            // }
        }
        else
        {
            q_dot_hqpik2_[i].setZero();

            // last_solved_hierarchy_num_ = max(i-1, 0);
            // if (i < 5)
            // {
            if (int(current_time_ * 10000) % 1000 == 0)
                std::cout << "Error hierarchy: " << i << std::endl;
            // }
            // cout<<"Error qpres_: \n"<< qpres_ << endl;
            break;
        }
        // cout<<"ubA_[0]: " << ubA_[0]<<endl;
    }
    if (int(current_time_ * 10000) % 2000 == 0)
    {
        // cout<<"u_dot_[0]- J_hqpik2_[0]*q_dot_hqpik2_[0]: \n" << u_dot_hqpik2_[0] - J_hqpik2_[0]*q_dot_hqpik2_[0] << endl;
        // cout<<"u_dot_[0]- J_hqpik2_[0]*q_dot_hqpik2_[1]: \n" << u_dot_hqpik2_[0] - J_hqpik2_[0]*q_dot_hqpik2_[1] << endl;
        // cout<<"u_dot_[0]- J_hqpik2_[0]*q_dot_hqpik2_[2]: \n" << u_dot_hqpik2_[0] - J_hqpik2_[0]*q_dot_hqpik2_[2] << endl;
        // cout<<"u_dot_[0]- J_hqpik2_[0]*q_dot_hqpik2_[3]: \n" << u_dot_hqpik2_[0] - J_hqpik2_[0]*q_dot_hqpik2_[3] << endl;

        // cout<<"u_dot_[1]- J_hqpik2_[1]*q_dot_hqpik2_[1]: \n" << u_dot_hqpik2_[1] - J_hqpik2_[1]*q_dot_hqpik2_[1] << endl;
        // cout<<"u_dot_[2]- J_hqpik2_[2]*q_dot_hqpik2_[2]: \n" << u_dot_hqpik2_[2] - J_hqpik2_[2]*q_dot_hqpik2_[2] << endl;

        // cout<<"J_hqpik2_[1]: \n" << J_hqpik2_[1]<<endl;
        // cout<<"u_dot_[0]: \n" << u_dot_[0]<<endl;
        // cout<<"u_dot_[1]: \n" << u_dot_[1]<<endl;
        // cout<<"u_dot_[2]: \n" << u_dot_[2]<<endl;
    }

    // cout<<"J_hqpik2_[0]: \n"<< J_hqpik2_[0]<<endl;
    for (int i = 0; i < variable_size_hqpik2_; i++)
    {
        motion_q_dot_(12 + i) = q_dot_hqpik2_[last_solved_hierarchy_num_](i);
        motion_q_(12 + i) = motion_q_pre_(12 + i) + motion_q_dot_(12 + i) * dt_;
        pd_control_mask_(12 + i) = 1;
    }
}
//////////Self Collision Avoidance Network////////////////
void AvatarController::initializeScaMlp(MLP &mlp, int n_input, int n_output, Eigen::VectorXd n_hidden, Eigen::VectorXd q_to_input_mapping_vector)
{
    mlp.n_input = n_input;
    mlp.n_output = n_output;
    mlp.n_hidden = n_hidden;
    mlp.n_layer = n_hidden.rows()+1; // hiden layers + output layer
    mlp.q_to_input_mapping_vector = q_to_input_mapping_vector;

    mlp.weight.resize(mlp.n_layer);
    mlp.bias.resize(mlp.n_layer);
    mlp.hidden.resize(mlp.n_layer-1);
    mlp.hidden_derivative.resize(mlp.n_layer-1);

    mlp.w_path.resize(mlp.n_layer);
    mlp.b_path.resize(mlp.n_layer);

    mlp.weight_files.resize(mlp.n_layer);
    mlp.bias_files.resize(mlp.n_layer);
    //parameters resize
    for (int i = 0; i < mlp.n_layer; i++)
    {   
        
        if(i == 0)
        {
            mlp.weight[i].setZero(mlp.n_hidden(i), mlp.n_input);
            mlp.bias[i].setZero(mlp.n_hidden(i));
            mlp.hidden[i].setZero(mlp.n_hidden(i));
            mlp.hidden_derivative[i].setZero(mlp.n_hidden(i), mlp.n_input);
        }
        else if(i == mlp.n_layer - 1)
        {
            mlp.weight[i].setZero(mlp.n_output, mlp.n_hidden(i-1));
            mlp.bias[i].setZero(mlp.n_output);
        }
        else
        {
            mlp.weight[i].setZero(mlp.n_hidden(i), mlp.n_hidden(i-1));
            mlp.bias[i].setZero(mlp.n_hidden(i));
            mlp.hidden[i].setZero(mlp.n_hidden(i));
            mlp.hidden_derivative[i].setZero(mlp.n_hidden(i), mlp.n_hidden(i-1));
        }
    }

    //input output resize
    mlp.input_slow.setZero(mlp.n_input);
    mlp.input_fast.setZero(mlp.n_input);
    mlp.input_thread.setZero(mlp.n_input);

    mlp.output_slow.setZero(mlp.n_output);
    mlp.output_fast.setZero(mlp.n_output);
    mlp.output_thread.setZero(mlp.n_output);

    mlp.output_derivative_fast.setZero(mlp.n_output, mlp.n_input);
    mlp.hx_gradient_fast.setZero(mlp.n_input);
    mlp.hx_gradient_fast_lpf.setZero(mlp.n_input);
    mlp.hx_gradient_fast_pre.setZero(mlp.n_input);

    mlp.self_collision_stop_cnt_ = 0;
}
void AvatarController::loadScaNetwork(MLP &mlp, std::string folder_path)
{
    for(int i =0; i<mlp.n_layer; i++)
    {
        mlp.w_path[i] = folder_path + "weight_" + std::to_string(i) + ".txt";
        mlp.b_path[i] = folder_path + "bias_" + std::to_string(i) + ".txt";

        mlp.weight_files[i].open(mlp.w_path[i], ios::in);
        mlp.bias_files[i].open(mlp.b_path[i], ios::in);

        readWeightFile(mlp, i);
        readBiasFile(mlp, i);
    }
}
void AvatarController::readWeightFile(MLP &mlp, int weight_num)
{
    if (!mlp.weight_files[weight_num].is_open())
    {
        std::cout << "Can not find the file: " << mlp.w_path[weight_num] << std::endl;
    }
    for(int i = 0; i<mlp.weight[weight_num].rows() ; i++)
    {
        for(int j = 0; j<mlp.weight[weight_num].cols() ; j++)
        {
            mlp.weight_files[weight_num] >> mlp.weight[weight_num](i, j);
        }
    }
    mlp.weight_files[weight_num].close();

    if(mlp.loadweightfile_verbose == true)
    {
        cout<<"weight_"<<weight_num<<": \n"<< mlp.weight[weight_num] <<endl;
    }
}
void AvatarController::readBiasFile(MLP &mlp, int bias_num)
{
    if (!mlp.bias_files[bias_num].is_open())
    {
        std::cout << "Can not find the file: " << mlp.b_path[bias_num] << std::endl;
    }
    for(int i = 0; i<mlp.bias[bias_num].rows() ; i++)
    {
        mlp.bias_files[bias_num] >> mlp.bias[bias_num](i);
    }
    mlp.bias_files[bias_num].close();

    if(mlp.loadbiasfile_verbose == true)
    {
        cout<<"bias_"<<bias_num - mlp.n_layer<< ": \n"<< mlp.bias[bias_num] <<endl;
    }
}
void AvatarController::calculateScaMlpInput(MLP &mlp)
{
    for(int i = 0; i<mlp.n_input; i++)
    {
        // mlp.input_slow(i) = rd_.q_(mlp.q_to_input_mapping_vector(i));
        // mlp.input_slow(i) = desired_q_fast_(mlp.q_to_input_mapping_vector(i));
        if(sca_dynamic_version_)
        {
            mlp.input_slow(i) = q_braking_stop_(mlp.q_to_input_mapping_vector(i));
        }
        else
        {
            mlp.input_slow(i) = rd_.q_(mlp.q_to_input_mapping_vector(i));
            // mlp.input_slow(i) = desired_q_fast_(mlp.q_to_input_mapping_vector(i));
        }
    }
    
    if(atb_mlp_input_update_ == false)
    {
        atb_mlp_input_update_ = true;
        mlp.input_thread = mlp.input_slow;
        q_ddot_max_thread_ = q_ddot_max_slow_;
        atb_mlp_input_update_ = false;
    }
}
void AvatarController::calculateScaMlpOutput(MLP &mlp)
{
    if(atb_mlp_input_update_ == false)
    {
        atb_mlp_input_update_ = true;
        mlp.input_fast = mlp.input_thread;
        q_ddot_max_fast_ = q_ddot_max_thread_;
        atb_mlp_input_update_ = false;
    }
    MatrixXd temp_derivative_pi; 
    for(int layer = 0; layer < mlp.n_layer; layer++)
    {
        if(layer == 0)  // input layer
        {
            mlp.hidden[0] = mlp.weight[0]*mlp.input_fast + mlp.bias[0];
            for(int h=0; h<mlp.n_hidden(layer); h++)
            {
                mlp.hidden[0](h) = std::tanh(mlp.hidden[0](h));   //activation function
                mlp.hidden_derivative[0].row(h) = (1-(mlp.hidden[0](h)*mlp.hidden[0](h)))*mlp.weight[0].row(h); //derivative wrt input
            }
            temp_derivative_pi = mlp.hidden_derivative[0];
        }
        else if(layer == mlp.n_layer - 1)   // output layer
        {
            mlp.output_fast = mlp.weight[layer]*mlp.hidden[layer-1] + mlp.bias[layer];
            mlp.output_derivative_fast = mlp.weight[layer]*temp_derivative_pi;
        }
        else    // hidden layers
        {
            mlp.hidden[layer] = mlp.weight[layer]*mlp.hidden[layer-1] + mlp.bias[layer];
            for(int h=0; h<mlp.n_hidden(layer); h++)
            {
                mlp.hidden[layer](h) = std::tanh(mlp.hidden[layer](h));   //activation function
                mlp.hidden_derivative[layer].row(h) = (1-(mlp.hidden[layer](h)*mlp.hidden[layer](h)))*mlp.weight[layer].row(h); //derivative wrt input
            }
            temp_derivative_pi =  mlp.hidden_derivative[layer]*temp_derivative_pi;
        }
    }

    if(sca_dynamic_version_)
    {
        // for(int i=0; i<mlp.n_input; i++)
        // {
            // if(q_ddot_max_fast_(mlp.q_to_input_mapping_vector(i)) !=0 )
            // {
            //     mlp.output_derivative_fast.col(i) = mlp.output_derivative_fast.col(i)*
            //     ( 1 - desired_q_ddot_(mlp.q_to_input_mapping_vector(i))/q_ddot_max_fast_(mlp.q_to_input_mapping_vector(i)) );
            // }
        // }
    }

    mlp.hx_gradient_fast_pre = mlp.hx_gradient_fast;
    mlp.hx_gradient_fast = (mlp.output_derivative_fast.row(1) - mlp.output_derivative_fast.row(0)).transpose();
    for(int i=0; i<mlp.n_input; i++)
    {
        mlp.hx_gradient_fast_lpf(i) = DyrosMath::lpf(mlp.hx_gradient_fast(i), mlp.hx_gradient_fast_pre(i), 1/dt_, 10.0);
    }

    mlp.hx = mlp.output_fast(1) - mlp.output_fast(0);


    // if(atb_mlp_output_update_ == false)
    // {
    //     atb_mlp_output_update_ = true;
    //     mlp.output_thread = mlp.output_fast;
    //     atb_mlp_output_update_ = false;
    // }
}
////////////////////////////////////////////////////////////////////////////////////////////
void AvatarController::poseCalibration()
{
    hmd_tracker_status_ = hmd_tracker_status_raw_;

    if (hmd_tracker_status_ == true)
    {
        if (hmd_tracker_status_pre_ == false)
        {
            tracker_status_changed_time_ = current_time_;
            cout << "tracker is attatched" << endl;

            std_msgs::String msg;
            std::stringstream upperbody_mode_ss;
            upperbody_mode_ss << "tracker is attatched";
            msg.data = upperbody_mode_ss.str();
            calibration_state_pub.publish(msg);
            calibration_state_gui_log_pub.publish(msg);
        }

        hmd_head_pose_ = hmd_head_pose_raw_;
        hmd_lshoulder_pose_ = hmd_lshoulder_pose_raw_;
        hmd_lupperarm_pose_ = hmd_lupperarm_pose_raw_;
        hmd_lhand_pose_ = hmd_lhand_pose_raw_;
        hmd_rshoulder_pose_ = hmd_rshoulder_pose_raw_;
        hmd_rupperarm_pose_ = hmd_rupperarm_pose_raw_;
        hmd_rhand_pose_ = hmd_rhand_pose_raw_;
        hmd_chest_pose_ = hmd_chest_pose_raw_;
        hmd_pelv_pose_ = hmd_pelv_pose_raw_;

        if (current_time_ - tracker_status_changed_time_ <= 3)
        {
            // double w = DyrosMath::cubic(current_time_, tracker_status_changed_time_, tracker_status_changed_time_+5, 0, 1, 0, 0);
            double w = (current_time_ - tracker_status_changed_time_) / 3;
            w = DyrosMath::minmax_cut(w, 0.0, 1.0);

            hmd_head_pose_.translation() = w * hmd_head_pose_raw_.translation() + (1 - w) * hmd_head_pose_raw_last_.translation();
            hmd_lupperarm_pose_.translation() = w * hmd_lupperarm_pose_raw_.translation() + (1 - w) * hmd_lupperarm_pose_raw_last_.translation();
            hmd_lhand_pose_.translation() = w * hmd_lhand_pose_raw_.translation() + (1 - w) * hmd_lhand_pose_raw_last_.translation();
            hmd_rupperarm_pose_.translation() = w * hmd_rupperarm_pose_raw_.translation() + (1 - w) * hmd_rupperarm_pose_raw_last_.translation();
            hmd_rhand_pose_.translation() = w * hmd_rhand_pose_raw_.translation() + (1 - w) * hmd_rhand_pose_raw_last_.translation();
            hmd_chest_pose_.translation() = w * hmd_chest_pose_raw_.translation() + (1 - w) * hmd_chest_pose_raw_last_.translation();
            hmd_pelv_pose_.translation() = w * hmd_pelv_pose_raw_.translation() + (1 - w) * hmd_pelv_pose_raw_last_.translation();

            Eigen::AngleAxisd head_ang_diff(hmd_head_pose_raw_.linear() * hmd_head_pose_raw_last_.linear().transpose());
            Eigen::AngleAxisd lelbow_ang_diff(hmd_lupperarm_pose_raw_.linear() * hmd_lupperarm_pose_raw_last_.linear().transpose());
            Eigen::AngleAxisd lhand_ang_diff(hmd_lhand_pose_raw_.linear() * hmd_lhand_pose_raw_last_.linear().transpose());
            Eigen::AngleAxisd relbow_ang_diff(hmd_rupperarm_pose_raw_.linear() * hmd_rupperarm_pose_raw_last_.linear().transpose());
            Eigen::AngleAxisd rhand_ang_diff(hmd_rhand_pose_raw_.linear() * hmd_rhand_pose_raw_last_.linear().transpose());
            Eigen::AngleAxisd upperbody_ang_diff(hmd_chest_pose_raw_.linear() * hmd_chest_pose_raw_last_.linear().transpose());
            Eigen::AngleAxisd pelv_ang_diff(hmd_pelv_pose_raw_.linear() * hmd_pelv_pose_raw_last_.linear().transpose());

            Eigen::Matrix3d lhand_diff_m, rhand_diff_m, lelbow_diff_m, relbow_diff_m, head_diff_m, upperbody_diff_m, pelv_diff_m;
            lhand_diff_m = Eigen::AngleAxisd(lhand_ang_diff.angle() * w, lhand_ang_diff.axis());
            rhand_diff_m = Eigen::AngleAxisd(rhand_ang_diff.angle() * w, rhand_ang_diff.axis());
            lelbow_diff_m = Eigen::AngleAxisd(lelbow_ang_diff.angle() * w, lelbow_ang_diff.axis());
            relbow_diff_m = Eigen::AngleAxisd(relbow_ang_diff.angle() * w, relbow_ang_diff.axis());
            head_diff_m = Eigen::AngleAxisd(head_ang_diff.angle() * w, head_ang_diff.axis());
            upperbody_diff_m = Eigen::AngleAxisd(upperbody_ang_diff.angle() * w, upperbody_ang_diff.axis());
            pelv_diff_m = Eigen::AngleAxisd(pelv_ang_diff.angle() * w, pelv_ang_diff.axis());

            hmd_lupperarm_pose_.linear() = lelbow_diff_m * hmd_lupperarm_pose_raw_last_.linear();
            hmd_lhand_pose_.linear() = lhand_diff_m * hmd_lhand_pose_raw_last_.linear();
            hmd_rupperarm_pose_.linear() = relbow_diff_m * hmd_rupperarm_pose_raw_last_.linear();
            hmd_rhand_pose_.linear() = rhand_diff_m * hmd_rhand_pose_raw_last_.linear();
            hmd_head_pose_.linear() = head_diff_m * hmd_head_pose_raw_last_.linear();
            hmd_chest_pose_.linear() = upperbody_diff_m * hmd_chest_pose_raw_last_.linear();
            hmd_pelv_pose_.linear() = pelv_diff_m * hmd_pelv_pose_raw_last_.linear();

            if (int((current_time_ - tracker_status_changed_time_) * 2000) % 1000 == 0)
                cout << "Motion Tracking Resume!" << int((current_time_ - tracker_status_changed_time_) / 3 * 100) << "%" << endl;
        }
        else
        {
        }
    }
    else // false
    {
        if (hmd_tracker_status_pre_ == true)
        {
            tracker_status_changed_time_ = current_time_;
            cout << cred << "tracker is detatched" << creset << endl;

            std_msgs::String msg;
            std::stringstream upperbody_mode_ss;
            upperbody_mode_ss << "tracker is detatched";
            msg.data = upperbody_mode_ss.str();
            calibration_state_pub.publish(msg);
            calibration_state_gui_log_pub.publish(msg);

            hmd_head_pose_raw_last_ = hmd_head_pose_raw_;
            hmd_lupperarm_pose_raw_last_ = hmd_lupperarm_pose_raw_;
            hmd_lhand_pose_raw_last_ = hmd_lhand_pose_raw_;
            hmd_rupperarm_pose_raw_last_ = hmd_rupperarm_pose_raw_;
            hmd_rhand_pose_raw_last_ = hmd_rhand_pose_raw_;
            hmd_chest_pose_raw_last_ = hmd_chest_pose_raw_;
            hmd_pelv_pose_raw_last_ = hmd_pelv_pose_raw_;
        }

        hmd_head_pose_ = hmd_head_pose_raw_last_;
        hmd_lupperarm_pose_ = hmd_lupperarm_pose_raw_last_;
        hmd_lhand_pose_ = hmd_lhand_pose_raw_last_;
        hmd_rupperarm_pose_ = hmd_rupperarm_pose_raw_last_;
        hmd_rhand_pose_ = hmd_rhand_pose_raw_last_;
        hmd_chest_pose_ = hmd_chest_pose_raw_last_;
        hmd_pelv_pose_ = hmd_pelv_pose_raw_last_;
    }


    hmd_pelv_vel_.segment(0, 3) = (hmd_pelv_pose_.translation() - hmd_pelv_pose_pre_.translation()) / dt_;
    Eigen::AngleAxisd ang_temp(hmd_pelv_pose_.linear() * hmd_pelv_pose_pre_.linear().transpose());
    hmd_pelv_vel_.segment(3, 3) = ang_temp.axis() * ang_temp.angle() / dt_;

    bool fast_pelv_move = false;
    bool far_pelv_move = false;
    int maximum_data_cut_num = 200;
    double pelv_max_vel = 5;
    double pelv_pos_boundary = 0.3;

    if ((hmd_check_pose_calibration_[3] == true) && (hmd_tracker_status_ == true) && (current_time_ - tracker_status_changed_time_ > 5))
    {
        // hmd_pelv_pose_ = velocityFilter(hmd_pelv_pose_, hmd_pelv_pose_pre_, hmd_pelv_vel_, pelv_max_vel, hmd_pelv_abrupt_motion_count_, maximum_data_cut_num, fast_pelv_move);

        // if (fast_pelv_move)
        // {
        //     cout << "Fast Pelvis Movement is Detected! (" << hmd_pelv_abrupt_motion_count_ << ")" << endl;
        // }

        // if (far_pelv_move)
        // {
        //     cout << "WARNING: The Operator Has Left The Init Position!! \n"
        //          << "Robot is stopped (upperbody mode #3)" << endl;
        //     upper_body_mode_ = 3;
        // }
    }

    Eigen::Vector3d hmd_pelv_rpy;
    Eigen::Matrix3d hmd_pelv_yaw_rot;
    Eigen::Isometry3d hmd_pelv_pose_yaw_only;

    if ((hmd_check_pose_calibration_[0] == true) && (still_pose_cali_flag_ == false))
    {
        hmd_pelv_pose_yaw_only.translation() = hmd_pelv_pose_.translation();
        hmd_pelv_rpy = DyrosMath::rot2Euler(hmd_pelv_pose_.linear());
        hmd_pelv_yaw_rot = DyrosMath::rotateWithZ(hmd_pelv_rpy(2));
        hmd_pelv_pose_yaw_only.linear() = hmd_pelv_yaw_rot;
    }
    else
    {
        hmd_pelv_pose_yaw_only.translation() = hmd_pelv_pose_init_.translation();
        hmd_pelv_rpy = DyrosMath::rot2Euler(hmd_pelv_pose_init_.linear());
        hmd_pelv_yaw_rot = DyrosMath::rotateWithZ(hmd_pelv_rpy(2));
        hmd_pelv_pose_yaw_only.linear() = hmd_pelv_yaw_rot;
    }

    // coordinate conversion
    if(master_arm_mode_ == false)
    {
        hmd_head_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_head_pose_;
        hmd_lupperarm_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_lupperarm_pose_;
        hmd_lhand_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_lhand_pose_;
        hmd_rupperarm_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_rupperarm_pose_;
        hmd_rhand_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_rhand_pose_;
        hmd_chest_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_chest_pose_;
        // hmd_pelv_pose_.linear().setIdentity();
    }
    else
    {
        hmd_head_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_head_pose_;
        hmd_lupperarm_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_lupperarm_pose_;
        // hmd_lhand_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_lhand_pose_;
        hmd_rupperarm_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_rupperarm_pose_;
        // hmd_rhand_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_rhand_pose_;
        hmd_chest_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_chest_pose_;
        // orientation offset
        hmd_lhand_pose_.linear() = hmd_lhand_pose_.linear()*DyrosMath::rotateWithY(-90*DEG2RAD);
        hmd_rhand_pose_.linear() = hmd_rhand_pose_.linear()*DyrosMath::rotateWithY(-90*DEG2RAD);
    }

    Eigen::Vector3d tracker_offset;
    // tracker_offset << -0.08, 0, 0; // bebop
    tracker_offset << -0.08, 0, -0.04; //senseglove

    hmd_lhand_pose_.translation() += hmd_lhand_pose_.linear() * tracker_offset;
    hmd_rhand_pose_.translation() += hmd_rhand_pose_.linear() * tracker_offset;

    if ((hmd_check_pose_calibration_[0] == true) && (still_pose_cali_flag_ == false))
    {
        // hmd_still_cali_lhand_pos_ = hmd_lhand_pose_.translation() - hmd_chest_pose_.translation();
        // hmd_still_cali_rhand_pos_ = hmd_rhand_pose_.translation() - hmd_chest_pose_.translation();
        hmd_still_cali_lhand_pos_ = hmd_lhand_pose_.translation();
        hmd_still_cali_rhand_pos_ = hmd_rhand_pose_.translation();

        // hmd_still_cali_lhand_pos_ <<  -0.13517, 0.289845, -0.259223;
        // hmd_still_cali_rhand_pos_ <<   -0.0650479, -0.324795, -0.255538;

        std_msgs::String msg;
        std::stringstream still_cali_data;
        still_cali_data << "still_L : " << (hmd_still_cali_lhand_pos_(0)) << ", "
                        << (hmd_still_cali_lhand_pos_(1)) << ", "
                        << (hmd_still_cali_lhand_pos_(2)) << std::endl
                        << "still_R : " << (hmd_still_cali_rhand_pos_(0)) << ", "
                        << (hmd_still_cali_rhand_pos_(1)) << ", "
                        << (hmd_still_cali_rhand_pos_(2)) << std::endl;
        msg.data = still_cali_data.str();
        calibration_state_pub.publish(msg);
        calibration_state_gui_log_pub.publish(msg);

        cout << "hmd_still_cali_lhand_pos_: " << hmd_still_cali_lhand_pos_ << endl;
        cout << "hmd_still_cali_rhand_pos_: " << hmd_still_cali_rhand_pos_ << endl;

        calibration_log_file_ofstream_[0].open(calibration_folder_dir_ + "/still_pose_.txt");
        calibration_log_file_ofstream_[0] << hmd_still_cali_lhand_pos_ << endl;
        calibration_log_file_ofstream_[0] << hmd_still_cali_rhand_pos_ << endl;
        calibration_log_file_ofstream_[0].close();

        hmd_head_pose_init_ = hmd_head_pose_;
        hmd_lupperarm_pose_init_ = hmd_lupperarm_pose_;
        hmd_lhand_pose_init_ = hmd_lhand_pose_;
        hmd_rupperarm_pose_init_ = hmd_rupperarm_pose_;
        hmd_rhand_pose_init_ = hmd_rhand_pose_;
        hmd_pelv_pose_init_ = hmd_pelv_pose_;
        hmd_chest_pose_init_ = hmd_chest_pose_;

        // hmd_head_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
        // hmd_lupperarm_pose_init_<<  -0.13517, 0.289845, -0.259223;
        // hmd_lhand_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
        // hmd_rupperarm_pose_init_<<  -0.13517, 0.289845, -0.259223;
        // hmd_rhand_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
        // hmd_pelv_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
        // hmd_chest_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;

        cout << "hmd_head_pose_init_: " << hmd_head_pose_init_.translation() << endl;
        cout << "hmd_lupperarm_pose_init_: " << hmd_lupperarm_pose_init_.translation() << endl;
        cout << "hmd_lhand_pose_init_: " << hmd_lhand_pose_init_.translation() << endl;
        cout << "hmd_rupperarm_pose_init_: " << hmd_rupperarm_pose_init_.translation() << endl;
        cout << "hmd_rhand_pose_init_: " << hmd_rhand_pose_init_.translation() << endl;
        cout << "hmd_pelv_pose_init_: " << hmd_pelv_pose_init_.translation() << endl;
        cout << "hmd_chest_pose_init_: " << hmd_chest_pose_init_.translation() << endl;

        calibration_log_file_ofstream_[3].open(calibration_folder_dir_ + "/hmd_pose_init_.txt");
        calibration_log_file_ofstream_[3] << hmd_head_pose_init_.translation() << "\n"
                                          << hmd_head_pose_init_.linear() << endl;
        calibration_log_file_ofstream_[3] << hmd_lupperarm_pose_init_.translation() << "\n"
                                          << hmd_lupperarm_pose_init_.linear() << endl;
        calibration_log_file_ofstream_[3] << hmd_lhand_pose_init_.translation() << "\n"
                                          << hmd_lhand_pose_init_.linear() << endl;
        calibration_log_file_ofstream_[3] << hmd_rupperarm_pose_init_.translation() << "\n"
                                          << hmd_rupperarm_pose_init_.linear() << endl;
        calibration_log_file_ofstream_[3] << hmd_rhand_pose_init_.translation() << "\n"
                                          << hmd_rhand_pose_init_.linear() << endl;
        calibration_log_file_ofstream_[3] << hmd_pelv_pose_init_.translation() << "\n"
                                          << hmd_pelv_pose_init_.linear() << endl;
        calibration_log_file_ofstream_[3] << hmd_chest_pose_init_.translation() << "\n"
                                          << hmd_chest_pose_init_.linear() << endl;
        calibration_log_file_ofstream_[3].close();
        still_pose_cali_flag_ = true;
    }

    if ((hmd_check_pose_calibration_[1] == true) && (t_pose_cali_flag_ == false))
    {
        // hmd_tpose_cali_lhand_pos_ = hmd_lhand_pose_.translation() - hmd_chest_pose_.translation();
        // hmd_tpose_cali_rhand_pos_ = hmd_rhand_pose_.translation() - hmd_chest_pose_.translation();
        hmd_tpose_cali_lhand_pos_ = hmd_lhand_pose_.translation();
        hmd_tpose_cali_rhand_pos_ = hmd_rhand_pose_.translation();
        // 20210209
        //  		hmd_tpose_cali_lhand_pos_: -0.284813
        //   0.803326
        //   0.356484
        //  hmd_tpose_cali_rhand_pos_: -0.146704
        //  -0.806712
        //   0.357469
        //  hmd_tpose_cali_lhand_pos_ <<  -0.284813, 0.803326, 0.356484;
        //  hmd_tpose_cali_rhand_pos_ <<  -0.146704, -0.806712, 0.357469;

        std_msgs::String msg;
        std::stringstream tpose_cali_data;
        tpose_cali_data << "T_L : " << (hmd_tpose_cali_lhand_pos_(0)) << ", "
                        << (hmd_tpose_cali_lhand_pos_(1)) << ", "
                        << (hmd_tpose_cali_lhand_pos_(2)) << std::endl
                        << "T_R : " << (hmd_tpose_cali_rhand_pos_(0)) << ", "
                        << (hmd_tpose_cali_rhand_pos_(1)) << ", "
                        << (hmd_tpose_cali_rhand_pos_(2));
        msg.data = tpose_cali_data.str();
        calibration_state_pub.publish(msg);
        calibration_state_gui_log_pub.publish(msg);

        cout << "hmd_tpose_cali_lhand_pos_: " << hmd_tpose_cali_lhand_pos_ << endl;
        cout << "hmd_tpose_cali_rhand_pos_: " << hmd_tpose_cali_rhand_pos_ << endl;

        calibration_log_file_ofstream_[1].open(calibration_folder_dir_ + "/t_pose_.txt");
        calibration_log_file_ofstream_[1] << hmd_tpose_cali_lhand_pos_ << endl;
        calibration_log_file_ofstream_[1] << hmd_tpose_cali_rhand_pos_ << endl;
        calibration_log_file_ofstream_[1].close();
        t_pose_cali_flag_ = true;
    }

    if ((hmd_check_pose_calibration_[2] == true) && (forward_pose_cali_flag_ == false))
    {
        // hmd_forward_cali_lhand_pos_ = hmd_lhand_pose_.translation() - hmd_chest_pose_.translation();
        // hmd_forward_cali_rhand_pos_ = hmd_rhand_pose_.translation() - hmd_chest_pose_.translation();
        hmd_forward_cali_lhand_pos_ = hmd_lhand_pose_.translation();
        hmd_forward_cali_rhand_pos_ = hmd_rhand_pose_.translation();

        // 20210209

        // 0.430654
        // 0.263043
        //  0.36878
        // hmd_forward_cali_rhand_pos_:  0.463099
        // -0.198173
        //  0.373117

        // hmd_forward_cali_lhand_pos_ <<  0.430654, 0.263043, 0.36878;
        // hmd_forward_cali_rhand_pos_ <<  0.463099, -0.198173, 0.373117;

        std_msgs::String msg;
        std::stringstream forward_cali_data;
        forward_cali_data << "Foward_L : " << hmd_forward_cali_lhand_pos_(0) << ", "
                          << (hmd_forward_cali_lhand_pos_(1)) << ", "
                          << (hmd_forward_cali_lhand_pos_(2)) << std::endl
                          << "Forward_R : " << (hmd_forward_cali_rhand_pos_(0)) << ", "
                          << (hmd_forward_cali_rhand_pos_(1)) << ", "
                          << (hmd_forward_cali_rhand_pos_(2));
        msg.data = forward_cali_data.str();
        calibration_state_pub.publish(msg);
        calibration_state_gui_log_pub.publish(msg);

        cout << "hmd_forward_cali_lhand_pos_: " << hmd_forward_cali_lhand_pos_ << endl;
        cout << "hmd_forward_cali_rhand_pos_: " << hmd_forward_cali_rhand_pos_ << endl;

        calibration_log_file_ofstream_[2].open(calibration_folder_dir_ + "/forward_pose_.txt");
        calibration_log_file_ofstream_[2] << hmd_forward_cali_lhand_pos_ << endl;
        calibration_log_file_ofstream_[2] << hmd_forward_cali_rhand_pos_ << endl;
        calibration_log_file_ofstream_[2].close();
        forward_pose_cali_flag_ = true;
    }

    if ((hmd_check_pose_calibration_[4] == true) && (read_cali_log_flag_ == false))
    {
        //////////read calibration log file/////////////////
        calibration_log_file_ifstream_[0].open(calibration_folder_dir_ + "/still_pose_.txt");
        calibration_log_file_ifstream_[1].open(calibration_folder_dir_ + "/t_pose_.txt");
        calibration_log_file_ifstream_[2].open(calibration_folder_dir_ + "/forward_pose_.txt");
        calibration_log_file_ifstream_[3].open(calibration_folder_dir_ + "/hmd_pose_init_.txt");

        if (calibration_log_file_ifstream_[0].is_open())
        {
            getTranslationDataFromText(calibration_log_file_ifstream_[0], hmd_still_cali_lhand_pos_);
            getTranslationDataFromText(calibration_log_file_ifstream_[0], hmd_still_cali_rhand_pos_);
            cout << "Still Pose is Uploaded: [(" << hmd_still_cali_lhand_pos_.transpose() << "), (" << hmd_still_cali_rhand_pos_.transpose() << ")]" << endl;
            calibration_log_file_ifstream_[0].close();
        }
        else
        {
            cout << "Still Pose Calibration File Is Not Opened!" << endl;
        }

        if (calibration_log_file_ifstream_[1].is_open())
        {
            getTranslationDataFromText(calibration_log_file_ifstream_[1], hmd_tpose_cali_lhand_pos_);
            getTranslationDataFromText(calibration_log_file_ifstream_[1], hmd_tpose_cali_rhand_pos_);
            cout << "T Pose is Uploaded: [(" << hmd_tpose_cali_lhand_pos_.transpose() << "), (" << hmd_tpose_cali_rhand_pos_.transpose() << ")]" << endl;
            calibration_log_file_ifstream_[1].close();
        }
        else
        {
            cout << "T Pose Calibration File Is Not Opened!" << endl;
        }

        if (calibration_log_file_ifstream_[2].is_open())
        {
            getTranslationDataFromText(calibration_log_file_ifstream_[2], hmd_forward_cali_lhand_pos_);
            getTranslationDataFromText(calibration_log_file_ifstream_[2], hmd_forward_cali_rhand_pos_);
            cout << "Forward Pose is Uploaded: [(" << hmd_forward_cali_lhand_pos_.transpose() << "), (" << hmd_forward_cali_rhand_pos_.transpose() << ")]" << endl;
            calibration_log_file_ifstream_[2].close();
        }
        else
        {
            cout << "Forward Pose Calibration File Is Not Opened!" << endl;
        }

        if (calibration_log_file_ifstream_[3].is_open())
        {
            getIsometry3dDataFromText(calibration_log_file_ifstream_[3], hmd_head_pose_init_);
            getIsometry3dDataFromText(calibration_log_file_ifstream_[3], hmd_lupperarm_pose_init_);
            getIsometry3dDataFromText(calibration_log_file_ifstream_[3], hmd_lhand_pose_init_);
            getIsometry3dDataFromText(calibration_log_file_ifstream_[3], hmd_rupperarm_pose_init_);
            getIsometry3dDataFromText(calibration_log_file_ifstream_[3], hmd_rhand_pose_init_);
            getIsometry3dDataFromText(calibration_log_file_ifstream_[3], hmd_pelv_pose_init_);
            getIsometry3dDataFromText(calibration_log_file_ifstream_[3], hmd_chest_pose_init_);
            calibration_log_file_ifstream_[3].close();

            cout << "hmd_head_pose_init_: " << hmd_head_pose_init_.translation().transpose() << endl;
            cout << "hmd_lupperarm_pose_init_: " << hmd_lupperarm_pose_init_.translation().transpose() << endl;
            cout << "hmd_lhand_pose_init_: " << hmd_lhand_pose_init_.translation().transpose() << endl;
            cout << "hmd_rupperarm_pose_init_: " << hmd_rupperarm_pose_init_.translation().transpose() << endl;
            cout << "hmd_rhand_pose_init_: " << hmd_rhand_pose_init_.translation().transpose() << endl;
            cout << "hmd_pelv_pose_init_: " << hmd_pelv_pose_init_.translation().transpose() << endl;
            cout << "hmd_chest_pose_init_: " << hmd_chest_pose_init_.translation().transpose() << endl;
            cout << "hmd_chest_pose_init_.linear(): " << hmd_chest_pose_init_.linear() << endl;
        }
        else
        {
            cout << "HMD Init Pose File Is NOT Opened!" << endl;
        }

        hmd_check_pose_calibration_[3] = false;

        read_cali_log_flag_ = true;
    }

    if ((hmd_check_pose_calibration_[3] == false) && (still_pose_cali_flag_ * t_pose_cali_flag_ * forward_pose_cali_flag_ == true))
    {
        // hmd_lshoulder_center_pos_(0) = (hmd_still_cali_lhand_pos_(0) + hmd_tpose_cali_lhand_pos_(0)) / 2;
        // // hmd_lshoulder_center_pos_(1) = (hmd_still_cali_lhand_pos_(1) + hmd_forward_cali_lhand_pos_(1))/2;
        // hmd_lshoulder_center_pos_(1) = hmd_still_cali_lhand_pos_(1);
        // hmd_lshoulder_center_pos_(2) = (hmd_tpose_cali_lhand_pos_(2) + hmd_forward_cali_lhand_pos_(2)) / 2;

        // hmd_rshoulder_center_pos_(0) = (hmd_still_cali_rhand_pos_(0) + hmd_tpose_cali_rhand_pos_(0)) / 2;
        // // hmd_rshoulder_center_pos_(1) = (hmd_still_cali_rhand_pos_(1) + hmd_forward_cali_rhand_pos_(1))/2;
        // hmd_rshoulder_center_pos_(1) = hmd_still_cali_rhand_pos_(1);
        // hmd_rshoulder_center_pos_(2) = (hmd_tpose_cali_rhand_pos_(2) + hmd_forward_cali_rhand_pos_(2)) / 2;

        //// Geometric Shoulder Calculation ///////////////////////////
        getCenterOfShoulderCali(hmd_still_cali_lhand_pos_, hmd_tpose_cali_lhand_pos_, hmd_forward_cali_lhand_pos_, hmd_lshoulder_center_pos_);
        getCenterOfShoulderCali(hmd_still_cali_rhand_pos_, hmd_tpose_cali_rhand_pos_, hmd_forward_cali_rhand_pos_, hmd_rshoulder_center_pos_);

        hmd_larm_max_l_ = 0;
        hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_still_cali_lhand_pos_).norm();
        hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_tpose_cali_lhand_pos_).norm();
        hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_forward_cali_lhand_pos_).norm();
        hmd_larm_max_l_ /= 3;
        // hmd_larm_max_l_ = 0.54;

        hmd_rarm_max_l_ = 0;
        hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_still_cali_rhand_pos_).norm();
        hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_tpose_cali_rhand_pos_).norm();
        hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_forward_cali_rhand_pos_).norm();
        hmd_rarm_max_l_ /= 3;

        hmd_shoulder_width_ = (hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_).norm();
        // hmd_shoulder_width_ = 0.521045;

        // hmd_rarm_max_l_ = 0.54;
        Eigen::Vector3d l_still_basis = hmd_still_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        Eigen::Vector3d l_tpose_basis = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        Eigen::Vector3d l_forward_basis = hmd_forward_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        Eigen::Vector3d l_angle_btw_bases;
        l_angle_btw_bases(0) = (l_still_basis.dot(l_tpose_basis)) / (l_still_basis.norm() * l_tpose_basis.norm());
        l_angle_btw_bases(0) = DyrosMath::minmax_cut(l_angle_btw_bases(0), -1.0, 1.0);
        l_angle_btw_bases(0) = acos(l_angle_btw_bases(0)) * RAD2DEG;

        l_angle_btw_bases(1) = (l_tpose_basis.dot(l_forward_basis)) / (l_tpose_basis.norm() * l_forward_basis.norm());
        l_angle_btw_bases(1) = DyrosMath::minmax_cut(l_angle_btw_bases(1), -1.0, 1.0);
        l_angle_btw_bases(1) = acos(l_angle_btw_bases(1)) * RAD2DEG;

        l_angle_btw_bases(2) = (l_forward_basis.dot(l_still_basis)) / (l_forward_basis.norm() * l_still_basis.norm());
        l_angle_btw_bases(2) = DyrosMath::minmax_cut(l_angle_btw_bases(2), -1.0, 1.0);
        l_angle_btw_bases(2) = acos(l_angle_btw_bases(2)) * RAD2DEG;

        Eigen::Vector3d r_still_basis = hmd_still_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        Eigen::Vector3d r_tpose_basis = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        Eigen::Vector3d r_forward_basis = hmd_forward_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        Eigen::Vector3d r_angle_btw_bases;
        r_angle_btw_bases(0) = (r_still_basis.dot(r_tpose_basis)) / (r_still_basis.norm() * r_tpose_basis.norm());
        r_angle_btw_bases(0) = DyrosMath::minmax_cut(r_angle_btw_bases(0), -1.0, 1.0);
        r_angle_btw_bases(0) = acos(r_angle_btw_bases(0)) * RAD2DEG;

        r_angle_btw_bases(1) = (r_tpose_basis.dot(r_forward_basis)) / (r_tpose_basis.norm() * r_forward_basis.norm());
        r_angle_btw_bases(1) = DyrosMath::minmax_cut(r_angle_btw_bases(1), -1.0, 1.0);
        r_angle_btw_bases(1) = acos(r_angle_btw_bases(1)) * RAD2DEG;

        r_angle_btw_bases(2) = (r_forward_basis.dot(r_still_basis)) / (r_forward_basis.norm() * r_still_basis.norm());
        r_angle_btw_bases(2) = DyrosMath::minmax_cut(r_angle_btw_bases(2), -1.0, 1.0);
        r_angle_btw_bases(2) = acos(r_angle_btw_bases(2)) * RAD2DEG;

        std_msgs::String msg;
        std::stringstream arm_length_data;
        arm_length_data << "Left Arm Length : " << hmd_larm_max_l_ << ", "
                        << "Left Arm Length : " << hmd_rarm_max_l_ << "\n"
                        << "hmd_lshoulder_center_pos_: " << hmd_lshoulder_center_pos_.transpose() << "\n"
                        << "hmd_rshoulder_center_pos_: " << hmd_rshoulder_center_pos_.transpose() << "\n"
                        << "l_still_basis: " << l_still_basis.transpose() << "\n"
                        << "l_tpose_basis: " << l_tpose_basis.transpose() << "\n"
                        << "l_forward_basis: " << l_forward_basis.transpose() << "\n"
                        << "r_still_basis: " << r_still_basis.transpose() << "\n"
                        << "r_tpose_basis: " << r_tpose_basis.transpose() << "\n"
                        << "r_forward_basis: " << r_forward_basis.transpose() << "\n"
                        <<  "l_angle_btw_bases: " << l_angle_btw_bases.transpose() << "\n"
                        << "r_angle_btw_bases: " << r_angle_btw_bases.transpose() << endl;

        msg.data = arm_length_data.str();
        calibration_state_pub.publish(msg);
        calibration_state_gui_log_pub.publish(msg);

        cout << "hmd_lshoulder_center_pos_: " << hmd_lshoulder_center_pos_.transpose() << endl;
        cout << "hmd_rshoulder_center_pos_: " << hmd_rshoulder_center_pos_.transpose() << endl;
        cout << cblue << "hmd_larm_max_l_: " << hmd_larm_max_l_ << endl;
        cout << "hmd_rarm_max_l_: " << hmd_rarm_max_l_ << creset << endl;
        cout << "hmd_shoulder_width_: " << hmd_shoulder_width_ << endl;
        hmd_check_pose_calibration_[3] = true;

        hmd_chest_2_lshoulder_center_pos_ = hmd_lshoulder_center_pos_ - hmd_chest_pose_init_.translation();
        hmd_chest_2_rshoulder_center_pos_ = hmd_rshoulder_center_pos_ - hmd_chest_pose_init_.translation();

        cout << "hmd_chest_2_lshoulder_center_pos_: " << hmd_chest_2_lshoulder_center_pos_.transpose() << endl;
        cout << "hmd_chest_2_rshoulder_center_pos_: " << hmd_chest_2_rshoulder_center_pos_.transpose() << endl;

        cout << "l_still_basis: " << l_still_basis.transpose() << ", norm: " << l_still_basis.norm() << endl;
        cout << "l_tpose_basis: " << l_tpose_basis.transpose() << ", norm: " << l_tpose_basis.norm() << endl;
        cout << "l_forward_basis: " << l_forward_basis.transpose() << ", norm: " << l_forward_basis.norm() << endl;

        cout << "r_still_basis: " << r_still_basis.transpose() << ", norm: " << r_still_basis.norm() << endl;
        cout << "r_tpose_basis: " << r_tpose_basis.transpose() << ", norm: " << r_tpose_basis.norm() << endl;
        cout << "r_forward_basis: " << r_forward_basis.transpose() << ", norm: " << r_forward_basis.norm() << endl;

        cout << cblue << "l_angles_btw_bases(degree); should be near 90degrees: " << l_angle_btw_bases.transpose() << endl;
        cout << "r_angles_btw_bases(degree); should be near 90degrees: " << r_angle_btw_bases.transpose() << creset << endl;

        hmd_lshoulder_pose_init_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_lshoulder_center_pos_ + hmd_chest_pose_init_.translation();
        hmd_lshoulder_pose_init_.linear() = hmd_chest_pose_init_.linear();
        hmd_rshoulder_pose_init_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_rshoulder_center_pos_ + hmd_chest_pose_init_.translation();
        hmd_rshoulder_pose_init_.linear() = hmd_chest_pose_init_.linear();
    }

    // Shoulder Data
    hmd_lshoulder_pose_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_lshoulder_center_pos_ + hmd_chest_pose_.translation();
    hmd_lshoulder_pose_.linear() = hmd_chest_pose_.linear();
    hmd_rshoulder_pose_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_rshoulder_center_pos_ + hmd_chest_pose_.translation();
    hmd_rshoulder_pose_.linear() = hmd_chest_pose_.linear();

    // HMD Velocity
    double tracker_hz = 130;

    hmd_head_vel_.segment(0, 3) = (hmd_head_pose_.translation() - hmd_head_pose_pre_.translation()) * tracker_hz;
    hmd_lshoulder_vel_.segment(0, 3) = (hmd_lshoulder_pose_.translation() - hmd_lshoulder_pose_pre_.translation()) * tracker_hz;
    hmd_lupperarm_vel_.segment(0, 3) = (hmd_lupperarm_pose_.translation() - hmd_lupperarm_pose_pre_.translation()) * tracker_hz;
    hmd_lhand_vel_.segment(0, 3) = (hmd_lhand_pose_.translation() - hmd_lhand_pose_pre_.translation()) * tracker_hz;
    hmd_rshoulder_vel_.segment(0, 3) = (hmd_rshoulder_pose_.translation() - hmd_rshoulder_pose_pre_.translation()) * tracker_hz;
    hmd_rupperarm_vel_.segment(0, 3) = (hmd_rupperarm_pose_.translation() - hmd_rupperarm_pose_pre_.translation()) * tracker_hz;
    hmd_rhand_vel_.segment(0, 3) = (hmd_rhand_pose_.translation() - hmd_rhand_pose_pre_.translation()) * tracker_hz;
    hmd_chest_vel_.segment(0, 3) = (hmd_chest_pose_.translation() - hmd_chest_pose_pre_.translation()) * tracker_hz;

    Eigen::AngleAxisd ang_temp_1(hmd_head_pose_.linear() * hmd_head_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_2(hmd_lshoulder_pose_.linear() * hmd_lshoulder_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_3(hmd_lupperarm_pose_.linear() * hmd_lupperarm_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_4(hmd_lhand_pose_.linear() * hmd_lhand_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_5(hmd_rshoulder_pose_.linear() * hmd_rshoulder_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_6(hmd_rupperarm_pose_.linear() * hmd_rupperarm_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_7(hmd_rhand_pose_.linear() * hmd_rhand_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_8(hmd_chest_pose_.linear() * hmd_chest_pose_pre_.linear().transpose());

    hmd_head_vel_.segment(3, 3) = ang_temp_1.axis() * ang_temp_1.angle() * tracker_hz;
    hmd_lshoulder_vel_.segment(3, 3) = ang_temp_2.axis() * ang_temp_2.angle() * tracker_hz;
    hmd_lupperarm_vel_.segment(3, 3) = ang_temp_3.axis() * ang_temp_3.angle() * tracker_hz;
    hmd_lhand_vel_.segment(3, 3) = ang_temp_4.axis() * ang_temp_4.angle() * tracker_hz;
    hmd_rshoulder_vel_.segment(3, 3) = ang_temp_5.axis() * ang_temp_5.angle() * tracker_hz;
    hmd_rupperarm_vel_.segment(3, 3) = ang_temp_6.axis() * ang_temp_6.angle() * tracker_hz;
    hmd_rhand_vel_.segment(3, 3) = ang_temp_7.axis() * ang_temp_7.angle() * tracker_hz;
    hmd_chest_vel_.segment(3, 3) = ang_temp_8.axis() * ang_temp_8.angle() * tracker_hz;

    double check_val = 0;

    check_val = hmd_lhand_vel_.segment(0, 3).norm();
    // abrupt motion check and stop

    double limit_val = 4.0;

    // if (upper_body_mode_>=5)
    if(false)
    {
        if (check_val > limit_val)
        {
            cout << cred << "WARNING: left hand linear velocity is over the 2.0m/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_lhand_vel_.segment(3, 3).norm();
        if ((check_val > limit_val * M_PI))
        {
            cout << cred <<"WARNING: left hand angular velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_rhand_vel_.segment(0, 3).norm();
        if ((check_val > limit_val))
        {
            cout << cred <<"WARNING: right hand linear velocity is over the 2.0m/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_rhand_vel_.segment(3, 3).norm();
        if ((check_val > limit_val * M_PI))
        {
            cout << cred <<"WARNING: right hand angular velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_lupperarm_vel_.segment(0, 3).norm();
        if ((check_val > limit_val))
        {
            cout << cred <<"WARNING: hmd_lupperarm_vel_ linear velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_lupperarm_vel_.segment(3, 3).norm();
        if ((check_val > limit_val * M_PI))
        {
            cout << cred <<"WARNING: hmd_lupperarm_vel_ angular velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_rupperarm_vel_.segment(0, 3).norm();
        if ((check_val > limit_val))
        {
            cout << cred <<"WARNING: hmd_rupperarm_vel_ linear velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_rupperarm_vel_.segment(3, 3).norm();
        if ((check_val > limit_val * M_PI))
        {
            cout << cred <<"WARNING: hmd_rupperarm_vel_ angular velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_head_vel_.segment(3, 3).norm();
        if ((check_val > limit_val * M_PI))
        {
            cout << cred <<"WARNING: Head angular velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }

        check_val = hmd_chest_vel_.segment(3, 3).norm();
        if ((check_val > limit_val * M_PI))
        {
            cout << cred <<"WARNING: Chest angular velocity is over the 360 degree/s limit" << check_val << creset << endl;
            avatarUpperbodyModeUpdate(3);
        }
    }
}
void AvatarController::getCenterOfShoulderCali(Eigen::Vector3d Still_pose_cali, Eigen::Vector3d T_pose_cali, Eigen::Vector3d Forward_pose_cali, Eigen::Vector3d &CenterOfShoulder_cali)
{
    Eigen::Matrix3d temp_mat;
    Eigen::Vector3d one3, normal_to_cali_plane, p1_p2, p2_p3, p3_p1, u12, v23, center_of_cali_plane1, center_of_cali_plane2;
    temp_mat << Still_pose_cali.transpose(), T_pose_cali.transpose(), Forward_pose_cali.transpose();
    one3(0) = 1;
    one3(1) = 1;
    one3(2) = 1;

    normal_to_cali_plane = (temp_mat.inverse()) * one3;

    p1_p2 = T_pose_cali - Still_pose_cali;
    p2_p3 = Forward_pose_cali - T_pose_cali;
    p3_p1 = Still_pose_cali - Forward_pose_cali;

    u12 = normal_to_cali_plane.cross(p1_p2);
    v23 = normal_to_cali_plane.cross(p2_p3);

    Eigen::MatrixXd temp_mat2, temp_vec2, ts;
    temp_mat2.resize(3, 2);
    temp_vec2.resize(3, 1);
    ts.resize(2, 1);

    temp_mat2 << u12, -v23;

    temp_vec2 = (Forward_pose_cali - Still_pose_cali) / 2;
    ts = (temp_mat2.transpose() * temp_mat2).inverse() * temp_mat2.transpose() * temp_vec2;

    center_of_cali_plane1 = (Still_pose_cali + T_pose_cali) / 2 + u12 * ts(0);
    center_of_cali_plane2 = (T_pose_cali + Forward_pose_cali) / 2 + v23 * ts(1);

    double r = (Still_pose_cali - center_of_cali_plane1).norm();

    double k1 = ((p1_p2.norm() * p1_p2.norm()) / 2 - r * r);
    k1 = sqrt(k1);
    double k2 = ((p2_p3.norm() * p2_p3.norm()) / 2 - r * r);
    k2 = sqrt(k2);
    double k3 = ((p3_p1.norm() * p3_p1.norm()) / 2 - r * r);
    k3 = sqrt(k3);
    double k_star = (k1 + k2 + k3) / 3;

    double k_threshold = 0.1;
    if ((abs(k1 - k2) > k_threshold) || (abs(k2 - k3) > k_threshold) || (abs(k1 - k3) > k_threshold))
    {
        cout << cred << "WARNING: Re-Calibration is REQUIRED!" << creset << endl;
    }
    CenterOfShoulder_cali = center_of_cali_plane1 - normal_to_cali_plane.normalized() * k_star;
}


void AvatarController::getTranslationDataFromText(std::ifstream &text_file, Eigen::Vector3d &trans)
{
    for (int i = 0; i < 3; i++)
    {
        string data;
        text_file >> data;
        trans(i) = atof(data.c_str());
    }
}
void AvatarController::getMatrix3dDataFromText(std::ifstream &text_file, Eigen::Matrix3d &mat)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            string data;
            text_file >> data;
            mat(i, j) = atof(data.c_str());
        }
    }
}

void AvatarController::getIsometry3dDataFromText(std::ifstream &text_file, Eigen::Isometry3d &isom)
{
    Vector3d trans;
    Matrix3d mat;
    getTranslationDataFromText(text_file, trans);
    getMatrix3dDataFromText(text_file, mat);
    isom.translation() = trans;
    isom.linear() = mat;
}

void AvatarController::rawMasterPoseProcessing()
{
    if (upperbody_mode_recieved_ == true)
    {
        upperbody_command_time_ = current_time_;
        upperbody_mode_q_init_ = motion_q_pre_;

        master_lhand_pose_ = lhand_transform_current_from_global_;
        master_rhand_pose_ = rhand_transform_current_from_global_;

        master_lhand_pose_pre_ = lhand_transform_pre_desired_from_;
        master_rhand_pose_pre_ = rhand_transform_pre_desired_from_;
        master_lelbow_pose_pre_ = lupperarm_transform_pre_desired_from_;
        master_relbow_pose_pre_ = rupperarm_transform_pre_desired_from_;
        master_lshoulder_pose_pre_ = lacromion_transform_pre_desired_from_;
        master_rshoulder_pose_pre_ = racromion_transform_pre_desired_from_;
        master_head_pose_pre_ = head_transform_pre_desired_from_;
        master_upperbody_pose_pre_ = upperbody_transform_pre_desired_from_;

        master_lhand_pose_ppre_ = lhand_transform_pre_desired_from_;
        master_rhand_pose_ppre_ = rhand_transform_pre_desired_from_;
        master_head_pose_ppre_ = head_transform_pre_desired_from_;
        master_lelbow_pose_ppre_ = lupperarm_transform_pre_desired_from_;
        master_relbow_pose_ppre_ = rupperarm_transform_pre_desired_from_;
        master_lshoulder_pose_ppre_ = lacromion_transform_pre_desired_from_;
        master_rshoulder_pose_ppre_ = racromion_transform_pre_desired_from_;
        master_upperbody_pose_ppre_ = upperbody_transform_pre_desired_from_;

        master_relative_lhand_pos_pre_ = lhand_transform_current_from_global_.translation() - rhand_transform_current_from_global_.translation();
        master_relative_rhand_pos_pre_ = rhand_transform_current_from_global_.translation() - lhand_transform_current_from_global_.translation();


        // save start variables for 3D mosue mode
        master_lhand_pose_start_ = lhand_transform_pre_desired_from_;
        master_rhand_pose_start_ = rhand_transform_pre_desired_from_;
        hmd_lhand_pose_start_ = hmd_lhand_pose_;
        hmd_rhand_pose_start_ = hmd_rhand_pose_;

        upperbody_mode_recieved_ = false;
        // hmd_shoulder_width_ = (hmd_lupperarm_pose_.translation() - hmd_rupperarm_pose_.translation()).norm();
    }

    // abruptMotionFilter();
    // hmdRawDataProcessing();
    if( upper_body_mode_ == 6  && upper_body_mode_ == 7)
    {
        handPositionRetargeting();
    }
    else if (upper_body_mode_ == 8)
    {
        /////Absolute hand position mapping //////
        Vector3d hand_offset;
        if(master_arm_mode_)
        {
            hand_offset << 0.0, 0, 0.0;
        }
        else
        {
            hand_offset << 0.0, 0.0, 0.15;
        }
        master_lhand_pose_raw_.translation() = hmd_lhand_pose_.translation() + hand_offset;
        master_rhand_pose_raw_.translation() = hmd_rhand_pose_.translation() + hand_offset;
        ///////////////////////////////////////////
    }
    else if (upper_body_mode_ == 9)
    {
        ///////Propotional hand position mapping////////////
        Vector3d hand_offset;
        if(master_arm_mode_)
        {
            hand_offset << 0.0, 0, 0.0;
        }
        else
        {
            hand_offset << 0.0, 0.0, 0.15;
        }
    
        master_lhand_pose_raw_.translation() = hand_pos_mapping_scale_raw_ * 1.3 * hmd_lhand_pose_.translation() + hand_offset;
        master_rhand_pose_raw_.translation() = hand_pos_mapping_scale_raw_ * 1.3 * hmd_rhand_pose_.translation() + hand_offset;

        //dg self collision test
        // master_lhand_pose_raw_.translation()(0) = 0.4 + 0.15*std::sin(current_time_*2*M_PI/4);
        // master_lhand_pose_raw_.translation()(1) = 0.05;
        // master_lhand_pose_raw_.translation()(2) = 0.2;
    }
    else if (upper_body_mode_ == 10)
    {
        ///////3D Mouse Mode////////////
        master_lhand_pose_raw_.translation() = master_lhand_pose_start_.translation() + 
                                                hand_pos_mapping_scale_raw_ * 1.0 * (hmd_lhand_pose_.translation() - hmd_lhand_pose_start_.translation());

        master_rhand_pose_raw_.translation() = master_rhand_pose_start_.translation() + 
                                                hand_pos_mapping_scale_raw_ * 1.0 * (hmd_rhand_pose_.translation() - hmd_rhand_pose_start_.translation());
        ////////////////////////////////////////////////////
    }

    orientationRetargeting();
    //////////////1025////////////////////////
    // master_lhand_pose_raw_ = master_lhand_pose_pre_;
    // master_rhand_pose_raw_ = master_rhand_pose_pre_;
    // master_lelbow_pose_raw_ = master_lelbow_pose_pre_;
    // master_relbow_pose_raw_ = master_relbow_pose_pre_;
    // master_lshoulder_pose_raw_= master_lshoulder_pose_pre_;
    // master_rshoulder_pose_raw_= master_rshoulder_pose_pre_;
    // master_head_pose_raw_= master_head_pose_pre_;
    // master_upperbody_pose_raw_= master_upperbody_pose_pre_;

    // master_lhand_pose_raw_.translation()(2) = DyrosMath::cubic(current_time_, upperbody_command_time_ +10, upperbody_command_time_+13, 0.2, 0.4, 0.0, 0.0);
    // master_lhand_pose_raw_.translation()(0) = 0.4;
    // master_lhand_pose_raw_.translation()(1) = 0.3;

    // master_rhand_pose_raw_.translation()(2) = DyrosMath::cubic(current_time_, upperbody_command_time_ +10, upperbody_command_time_+13, 0.2, 0.4, 0.0, 0.0);
    // master_rhand_pose_raw_.translation()(0) = 0.4;
    // master_rhand_pose_raw_.translation()(1) = -0.3;
    //////////////////////////////////////////

    double fc_filter = 10.0; // hz
    double spline_time = 3.0; // second
    if (current_time_ <= upperbody_command_time_ + spline_time)
    {
        for (int i = 0; i < 3; i++)
        {
            master_lhand_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, lhand_transform_pre_desired_from_.translation()(i), 0, 0, master_lhand_pose_raw_.translation()(i), 0, 0)(0);
            master_rhand_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, rhand_transform_pre_desired_from_.translation()(i), 0, 0, master_rhand_pose_raw_.translation()(i), 0, 0)(0);

            master_lelbow_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, lupperarm_transform_pre_desired_from_.translation()(i), 0, 0, master_lelbow_pose_raw_.translation()(i), 0, 0)(0);
            master_relbow_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, rupperarm_transform_pre_desired_from_.translation()(i), 0, 0, master_relbow_pose_raw_.translation()(i), 0, 0)(0);

            master_lshoulder_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, lacromion_transform_pre_desired_from_.translation()(i), 0, 0, master_lshoulder_pose_raw_.translation()(i), 0, 0)(0);
            master_rshoulder_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, racromion_transform_pre_desired_from_.translation()(i), 0, 0, master_rshoulder_pose_raw_.translation()(i), 0, 0)(0);

            master_head_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, head_transform_pre_desired_from_.translation()(i), 0, 0, master_head_pose_raw_.translation()(i), 0, 0)(0);

            master_relative_lhand_pos_raw_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, lhand_transform_pre_desired_from_.translation()(i) - rhand_transform_pre_desired_from_.translation()(i), 0, 0, master_relative_lhand_pos_raw_(i), 0, 0)(0);
            master_relative_rhand_pos_raw_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, rhand_transform_pre_desired_from_.translation()(i) - lhand_transform_pre_desired_from_.translation()(i), 0, 0, master_relative_rhand_pos_raw_(i), 0, 0)(0);
        }

        master_lhand_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, lhand_transform_pre_desired_from_.linear(), master_lhand_pose_raw_.linear());
        master_rhand_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, rhand_transform_pre_desired_from_.linear(), master_rhand_pose_raw_.linear());
        master_lelbow_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_raw_.linear());
        master_relbow_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_raw_.linear());
        master_lshoulder_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_raw_.linear());
        master_rshoulder_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_raw_.linear());
        master_head_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, head_transform_pre_desired_from_.linear(), master_head_pose_raw_.linear());
        master_upperbody_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + spline_time, upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_raw_.linear());
    }

    // master_lhand_pose_.translation() = DyrosMath::lpf<3>(master_lhand_pose_raw_.translation(), master_lhand_pose_pre_.translation(), 1 / dt_, fc_filter);
    // master_rhand_pose_.translation() = DyrosMath::lpf<3>(master_rhand_pose_raw_.translation(), master_rhand_pose_pre_.translation(), 1 / dt_, fc_filter);
    // master_lelbow_pose_.translation() = DyrosMath::lpf<3>(master_lelbow_pose_raw_.translation(), master_lelbow_pose_pre_.translation(), 1 / dt_, fc_filter);
    // master_relbow_pose_.translation() = DyrosMath::lpf<3>(master_relbow_pose_raw_.translation(), master_relbow_pose_pre_.translation(), 1 / dt_, fc_filter);
    // master_lshoulder_pose_.translation() = DyrosMath::lpf<3>(master_lshoulder_pose_raw_.translation(), master_lshoulder_pose_pre_.translation(), 1 / dt_, fc_filter);
    // master_rshoulder_pose_.translation() = DyrosMath::lpf<3>(master_rshoulder_pose_raw_.translation(), master_rshoulder_pose_pre_.translation(), 1 / dt_, fc_filter);
    // master_head_pose_.translation() = DyrosMath::lpf<3>(master_head_pose_raw_.translation(), master_head_pose_pre_.translation(), 1 / dt_, fc_filter);
    // master_upperbody_pose_.translation() = DyrosMath::lpf<3>(master_upperbody_pose_raw_.translation(), master_upperbody_pose_pre_.translation(), 1 / dt_, fc_filter);

    master_lhand_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_lhand_pose_raw_.translation(), master_lhand_pose_raw_pre_.translation(), master_lhand_pose_raw_ppre_.translation(), master_lhand_pose_pre_.translation(), master_lhand_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);
    master_rhand_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_rhand_pose_raw_.translation(), master_rhand_pose_raw_pre_.translation(), master_rhand_pose_raw_ppre_.translation(), master_rhand_pose_pre_.translation(), master_rhand_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);
    master_lelbow_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_lelbow_pose_raw_.translation(), master_lelbow_pose_raw_pre_.translation(), master_lelbow_pose_raw_ppre_.translation(), master_lelbow_pose_pre_.translation(), master_lelbow_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);
    master_relbow_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_relbow_pose_raw_.translation(), master_relbow_pose_raw_pre_.translation(), master_relbow_pose_raw_ppre_.translation(), master_relbow_pose_pre_.translation(), master_relbow_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);
    master_lshoulder_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_lshoulder_pose_raw_.translation(), master_lshoulder_pose_raw_pre_.translation(), master_lshoulder_pose_raw_ppre_.translation(), master_lshoulder_pose_pre_.translation(), master_lshoulder_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);
    master_rshoulder_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_rshoulder_pose_raw_.translation(), master_rshoulder_pose_raw_pre_.translation(), master_rshoulder_pose_raw_ppre_.translation(), master_rshoulder_pose_pre_.translation(), master_rshoulder_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);
    master_head_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_head_pose_raw_.translation(), master_head_pose_raw_pre_.translation(), master_head_pose_raw_ppre_.translation(), master_head_pose_pre_.translation(), master_head_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);
    master_upperbody_pose_.translation() = DyrosMath::secondOrderLowPassFilter<3>(master_upperbody_pose_raw_.translation(), master_upperbody_pose_raw_pre_.translation(), master_upperbody_pose_raw_ppre_.translation(), master_upperbody_pose_pre_.translation(), master_upperbody_pose_ppre_.translation(), fc_filter, 1, 1 / dt_);

    master_relative_lhand_pos_ = DyrosMath::lpf<3>(master_relative_lhand_pos_raw_, master_relative_lhand_pos_pre_, 1 / dt_, fc_filter);
    master_relative_rhand_pos_ = DyrosMath::lpf<3>(master_relative_rhand_pos_raw_, master_relative_rhand_pos_pre_, 1 / dt_, fc_filter);

    Eigen::AngleAxisd lhand_ang_diff(master_lhand_pose_raw_.linear() * master_lhand_pose_pre_.linear().transpose());
    Eigen::AngleAxisd rhand_ang_diff(master_rhand_pose_raw_.linear() * master_rhand_pose_pre_.linear().transpose());
    Eigen::AngleAxisd lelbow_ang_diff(master_lelbow_pose_raw_.linear() * master_lelbow_pose_pre_.linear().transpose());
    Eigen::AngleAxisd relbow_ang_diff(master_relbow_pose_raw_.linear() * master_relbow_pose_pre_.linear().transpose());
    Eigen::AngleAxisd lshoulder_ang_diff(master_lshoulder_pose_raw_.linear() * master_lshoulder_pose_pre_.linear().transpose());
    Eigen::AngleAxisd rshoulder_ang_diff(master_rshoulder_pose_raw_.linear() * master_rshoulder_pose_pre_.linear().transpose());
    Eigen::AngleAxisd head_ang_diff(master_head_pose_raw_.linear() * master_head_pose_pre_.linear().transpose());
    Eigen::AngleAxisd upperbody_ang_diff(master_upperbody_pose_raw_.linear() * master_upperbody_pose_pre_.linear().transpose());

    Eigen::Matrix3d lhand_diff_m, rhand_diff_m, lelbow_diff_m, relbow_diff_m, lshoulder_diff_m, rshoulder_diff_m, head_diff_m, upperbody_diff_m;
    double temp_ang_diff_filtered;
    // lhand_ang_diff.angle() = DyrosMath::lpf(lhand_ang_diff.angle(), 0, 2000, fc_filter);
    // lhand_diff_m = Eigen::AngleAxisd( temp_ang_diff_filtered, fc_filter), lhand_ang_diff.axis());
    lhand_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(lhand_ang_diff.angle(), 0, 1 / dt_, fc_filter), lhand_ang_diff.axis());
    rhand_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(rhand_ang_diff.angle(), 0, 1 / dt_, fc_filter), rhand_ang_diff.axis());
    lelbow_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(lelbow_ang_diff.angle(), 0, 1 / dt_, fc_filter), lelbow_ang_diff.axis());
    relbow_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(relbow_ang_diff.angle(), 0, 1 / dt_, fc_filter), relbow_ang_diff.axis());
    lshoulder_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(lshoulder_ang_diff.angle(), 0, 1 / dt_, fc_filter), lshoulder_ang_diff.axis());
    rshoulder_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(rshoulder_ang_diff.angle(), 0, 1 / dt_, fc_filter), rshoulder_ang_diff.axis());
    head_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(head_ang_diff.angle(), 0, 1 / dt_, fc_filter), head_ang_diff.axis());
    upperbody_diff_m = Eigen::AngleAxisd(DyrosMath::lpf(upperbody_ang_diff.angle(), 0, 1 / dt_, fc_filter), upperbody_ang_diff.axis());

    master_lhand_pose_.linear() = lhand_diff_m * master_lhand_pose_pre_.linear();
    master_rhand_pose_.linear() = rhand_diff_m * master_rhand_pose_pre_.linear();
    master_lelbow_pose_.linear() = lelbow_diff_m * master_lelbow_pose_pre_.linear();
    master_relbow_pose_.linear() = relbow_diff_m * master_relbow_pose_pre_.linear();
    master_lshoulder_pose_.linear() = lshoulder_diff_m * master_lshoulder_pose_pre_.linear();
    master_rshoulder_pose_.linear() = rshoulder_diff_m * master_rshoulder_pose_pre_.linear();
    master_head_pose_.linear() = head_diff_m * master_head_pose_pre_.linear();
    master_upperbody_pose_.linear() = upperbody_diff_m * master_upperbody_pose_pre_.linear();

    // for print
    master_lhand_rqy_ = DyrosMath::rot2Euler_tf(master_lhand_pose_.linear());
    master_rhand_rqy_ = DyrosMath::rot2Euler_tf(master_rhand_pose_.linear());

    master_lelbow_rqy_ = DyrosMath::rot2Euler_tf(master_lelbow_pose_.linear());
    master_relbow_rqy_ = DyrosMath::rot2Euler_tf(master_relbow_pose_.linear());

    master_lshoulder_rqy_ = DyrosMath::rot2Euler_tf(master_lshoulder_pose_.linear());
    master_rshoulder_rqy_ = DyrosMath::rot2Euler_tf(master_rshoulder_pose_.linear());

    master_head_rqy_ = DyrosMath::rot2Euler_tf(master_head_pose_.linear());

    // if( int(current_time_*10000)%1000 == 0)
    // {
    // 	cout<<"master_lelbow_rqy_: "<<master_lelbow_rqy_<<endl;
    // 	cout<<"master_relbow_rqy_: "<<master_relbow_rqy_<<endl;
    // }
    // master_lhand_pose_.linear() = lhand_transform_init_from_global_.linear();
    // master_rhand_pose_.linear() = rhand_transform_init_from_global_.linear();

    // double arm_len_max = 0.95;

    // if( master_lhand_pose_.translation().norm() > arm_len_max)
    // {
    // 	master_lhand_pose_.translation() = master_lhand_pose_.translation().normalized() * arm_len_max;
    // }

    // if( master_rhand_pose_.translation().norm() > arm_len_max)
    // {
    // 	master_rhand_pose_.translation() = master_rhand_pose_.translation().normalized() * arm_len_max;
    // }

    // for(int i = 0; i<3; i++)
    // {
    // 	master_lhand_vel_(i) = (master_lhand_pose_.translation()(i) - master_lhand_pose_pre_.translation()(i))/dt_;
    // 	master_rhand_vel_(i) = (master_rhand_pose_.translation()(i) - master_rhand_pose_pre_.translation()(i))/dt_;
    // }

    master_lhand_vel_.setZero();
    master_rhand_vel_.setZero();

    master_lelbow_vel_.setZero();
    master_relbow_vel_.setZero();

    master_lshoulder_vel_.setZero();
    master_rshoulder_vel_.setZero();

    master_head_vel_.setZero();
    master_upperbody_vel_.setZero();
}
void AvatarController::handPositionRetargeting()
{
    ///////////////////////////////////////////////HQP MOTION RETARGETING////////////////////////////////////////////
    if (first_loop_qp_retargeting_)
    {
        lhand_master_ref_stack_.setZero(3, 3);
        lhand_robot_ref_stack_.setZero(3, 3);
        rhand_master_ref_stack_.setZero(3, 3);
        rhand_robot_ref_stack_.setZero(3, 3);

        robot_still_pose_lhand_.setZero();
        robot_t_pose_lhand_.setZero();
        robot_forward_pose_lhand_.setZero();
        robot_still_pose_rhand_.setZero();
        robot_t_pose_rhand_.setZero();
        robot_forward_pose_rhand_.setZero();

        lhand_mapping_vector_.setZero();
        rhand_mapping_vector_.setZero();

        robot_still_pose_lhand_(2) += -(robot_arm_max_l_);
        robot_t_pose_lhand_(1) += (robot_arm_max_l_);
        robot_forward_pose_lhand_(0) += (robot_arm_max_l_);

        robot_still_pose_rhand_(2) += -(robot_arm_max_l_);
        robot_t_pose_rhand_(1) += -(robot_arm_max_l_);
        robot_forward_pose_rhand_(0) += (robot_arm_max_l_);

        lhand_master_ref_stack_.block(0, 0, 3, 1) = hmd_still_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        lhand_master_ref_stack_.block(0, 1, 3, 1) = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        lhand_master_ref_stack_.block(0, 2, 3, 1) = hmd_forward_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        // lhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        // lhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_rshoulder_center_pos_ - hmd_lshoulder_center_pos_;

        lhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_lhand_;
        lhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_lhand_;
        lhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_lhand_;
        // lhand_robot_ref_stack_.block(0, 3, 3, 1) = robot_t_pose_lhand_;
        // lhand_robot_ref_stack_(1, 3) = -robot_shoulder_width_;

        rhand_master_ref_stack_.block(0, 0, 3, 1) = hmd_still_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        rhand_master_ref_stack_.block(0, 1, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        rhand_master_ref_stack_.block(0, 2, 3, 1) = hmd_forward_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        // rhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        // rhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_;

        rhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_rhand_;
        rhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_rhand_;
        rhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_rhand_;
        // rhand_robot_ref_stack_.block(0, 3, 3, 1) = robot_t_pose_rhand_;
        // rhand_robot_ref_stack_(1, 3) = robot_shoulder_width_;

        E1_.setZero(control_size_retargeting_[0], variable_size_retargeting_);
        E2_.setZero(control_size_retargeting_[1], variable_size_retargeting_);
        E3_.setZero(control_size_retargeting_[2], variable_size_retargeting_);
        H_retargeting_.setZero(variable_size_retargeting_, variable_size_retargeting_);
        g_retargeting_.setZero(variable_size_retargeting_);
        u1_.setZero(control_size_retargeting_[0]);
        u2_.setZero(control_size_retargeting_[1]);
        u3_.setZero(control_size_retargeting_[2]);

        ub_retargeting_.setZero(constraint_size1_retargeting_);
        lb_retargeting_.setZero(constraint_size1_retargeting_);

        E1_.block(0, 0, 3, 3) = lhand_master_ref_stack_;
        E2_.block(0, 3, 3, 3) = rhand_master_ref_stack_;
        E3_.block(0, 0, 3, 3) = lhand_robot_ref_stack_;
        E3_.block(0, 3, 3, 3) = -rhand_robot_ref_stack_;

        for (int i = 0; i < constraint_size1_retargeting_; i++)
        {
            ub_retargeting_(i) = w_dot_max_;
            lb_retargeting_(i) = w_dot_min_;
        }

        w1_retargeting_ = 1;
        w2_retargeting_ = 1;
        w3_retargeting_ = 1;
        human_shoulder_width_ = (hmd_rshoulder_center_pos_ - hmd_lshoulder_center_pos_).norm();

        Eigen::MatrixXd lhand_master_ref_stack_pinverse_ = lhand_master_ref_stack_.transpose() * (lhand_master_ref_stack_ * lhand_master_ref_stack_.transpose() + damped_puedoinverse_eps_ * Eigen::Matrix3d::Identity()).inverse();
        lhand_mapping_vector_pre_ = lhand_master_ref_stack_pinverse_ * hmd_lshoulder_pose_init_.linear() * hmd_lshoulder_pose_.linear().transpose() * (hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());

        Eigen::MatrixXd rhand_master_ref_stack_pinverse_ = rhand_master_ref_stack_.transpose() * (rhand_master_ref_stack_ * rhand_master_ref_stack_.transpose() + damped_puedoinverse_eps_ * Eigen::Matrix3d::Identity()).inverse();
        rhand_mapping_vector_pre_ = rhand_master_ref_stack_pinverse_ * hmd_rshoulder_pose_init_.linear() * hmd_rshoulder_pose_.linear().transpose() * (hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

        h_pre_lhand_ = (hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());
        h_pre_rhand_ = (hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

        for (int i = 0; i < 3; i++)
        {
            QP_motion_retargeting_[i].InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_[i]);

            A_retargeting_[i].setZero(constraint_size2_retargeting_[i], variable_size_retargeting_);
            ubA_retargeting_[i].setZero(constraint_size2_retargeting_[i]);
            lbA_retargeting_[i].setZero(constraint_size2_retargeting_[i]);

            A_retargeting_[i].block(0, 0, 3, 3) = lhand_master_ref_stack_;
            A_retargeting_[i].block(3, 3, 3, 3) = rhand_master_ref_stack_;

            for (int j = 0; j < 6; j++)
            {
                ubA_retargeting_[i](j) = human_vel_max_;
                lbA_retargeting_[i](j) = human_vel_min_;
            }

            qpres_retargeting_[i].setZero(variable_size_retargeting_);
        }

        first_loop_qp_retargeting_ = false;
    }
    else
    {
        double speed_reduce_rate = 20;

        ub_retargeting_(0) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(0)), w_dot_max_);
        ub_retargeting_(1) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(1)), w_dot_max_);
        ub_retargeting_(2) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(2)), w_dot_max_);
        // ub_retargeting_(3) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(3)), w_dot_max_);
        ub_retargeting_(3) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(0)), w_dot_max_);
        ub_retargeting_(4) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(1)), w_dot_max_);
        ub_retargeting_(5) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(2)), w_dot_max_);
        // ub_retargeting_(7) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(3)), w_dot_max_);

        lb_retargeting_(0) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(0)), w_dot_min_);
        lb_retargeting_(1) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(1)), w_dot_min_);
        lb_retargeting_(2) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(2)), w_dot_min_);
        // lb_retargeting_(3) = max(speed_reduce_rate * (-1.0 - lhand_mapping_vector_pre_(3)), w_dot_min_);
        lb_retargeting_(3) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(0)), w_dot_min_);
        lb_retargeting_(4) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(1)), w_dot_min_);
        lb_retargeting_(5) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(2)), w_dot_min_);
        // lb_retargeting_(7) = max(speed_reduce_rate * (-1.0 - rhand_mapping_vector_pre_(3)), w_dot_min_);

        h_pre_lhand_ = lhand_master_ref_stack_ * lhand_mapping_vector_pre_;
        h_pre_rhand_ = rhand_master_ref_stack_ * rhand_mapping_vector_pre_;
        r_pre_lhand_ = lhand_robot_ref_stack_ * lhand_mapping_vector_pre_;
        r_pre_rhand_ = rhand_robot_ref_stack_ * rhand_mapping_vector_pre_;
    }


    qpRetargeting_1(); // calc lhand_mapping_vector_, rhand_mapping_vector_ //1025



    hmd2robot_lhand_pos_mapping_ = lhand_robot_ref_stack_ * lhand_mapping_vector_;
    hmd2robot_rhand_pos_mapping_ = rhand_robot_ref_stack_ * rhand_mapping_vector_;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (hmd2robot_lhand_pos_mapping_.norm() > robot_arm_max_l_)
    {
        hmd2robot_lhand_pos_mapping_ = hmd2robot_lhand_pos_mapping_.normalized() * robot_arm_max_l_;
    }

    if (hmd2robot_rhand_pos_mapping_.norm() > robot_arm_max_l_)
    {
        hmd2robot_rhand_pos_mapping_ = hmd2robot_rhand_pos_mapping_.normalized() * robot_arm_max_l_;
    }

    if (hmd2robot_lhand_pos_mapping_.norm() < 0.1)
    {
        hmd2robot_lhand_pos_mapping_ = hmd2robot_lhand_pos_mapping_.normalized() * 0.1;
    }

    if (hmd2robot_rhand_pos_mapping_.norm() < 0.1)
    {
        hmd2robot_rhand_pos_mapping_ = hmd2robot_rhand_pos_mapping_.normalized() * 0.1;
    }


    Vector3d robot_init_hand_pos, robot_init_lshoulder_pos, robot_init_rshoulder_pos, delta_hmd2robot_lhand_pos_maping, delta_hmd2robot_rhand_pos_maping, delta_hmd2robot_lelbow_pos_maping, delta_hmd2robot_relbow_pos_maping;
    robot_init_hand_pos << 0, 0, -(robot_arm_max_l_);
    robot_init_lshoulder_pos << 0, 0.1491, 0.065;
    robot_init_rshoulder_pos << 0, -0.1491, 0.065;

    master_lhand_pose_raw_.translation() = larmbase_transform_pre_desired_from_.translation() + upperbody_transform_pre_desired_from_.linear() * (robot_init_lshoulder_pos + hmd2robot_lhand_pos_mapping_);
    master_rhand_pose_raw_.translation() = rarmbase_transform_pre_desired_from_.translation() + upperbody_transform_pre_desired_from_.linear() * (robot_init_rshoulder_pos + hmd2robot_rhand_pos_mapping_);
}
void AvatarController::orientationRetargeting()
{
     Matrix3d robot_lhand_ori_init, robot_rhand_ori_init, robot_lelbow_ori_init, robot_relbow_ori_init, robot_lshoulder_ori_init, robot_rshoulder_ori_init, robot_head_ori_init, robot_upperbody_ori_init;
    robot_lhand_ori_init = DyrosMath::rotateWithZ(-90 * DEG2RAD);
    robot_rhand_ori_init = DyrosMath::rotateWithZ(90 * DEG2RAD);
    // robot_lshoulder_ori_init = DyrosMath::rotateWithZ(-0.3);
    robot_lshoulder_ori_init.setIdentity();
    // robot_rshoulder_ori_init = DyrosMath::rotateWithZ(0.3);
    robot_rshoulder_ori_init.setIdentity();
    robot_head_ori_init.setIdentity();

    robot_head_ori_init = DyrosMath::rotateWithY(-10 * DEG2RAD);
    
    robot_upperbody_ori_init.setIdentity();

    // robot_lelbow_ori_init << 0, 0, -1, 1, 0, 0, 0, -1, 0;
    robot_lelbow_ori_init.setZero();
    robot_lelbow_ori_init(0, 2) = -1;
    robot_lelbow_ori_init(1, 0) = 1;
    robot_lelbow_ori_init(2, 1) = -1;
    robot_lelbow_ori_init = DyrosMath::rotateWithZ(-0 * DEG2RAD) * robot_lelbow_ori_init;

    robot_relbow_ori_init.setZero();
    robot_relbow_ori_init(0, 2) = -1;
    robot_relbow_ori_init(1, 0) = -1;
    robot_relbow_ori_init(2, 1) = 1;
    robot_relbow_ori_init = DyrosMath::rotateWithZ(0 * DEG2RAD) * robot_relbow_ori_init;


    master_upperbody_pose_raw_.translation().setZero();
    Eigen::Matrix3d chest_diff_m, shoulder_diff_m;
    Eigen::AngleAxisd chest_ang_diff(hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose());
    chest_diff_m = Eigen::AngleAxisd(chest_ang_diff.angle() * 1.0, chest_ang_diff.axis());
    // master_upperbody_pose_raw_.linear() = chest_diff_m * robot_upperbody_ori_init;
    master_upperbody_pose_raw_.linear() = hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose()*robot_upperbody_ori_init;

    master_lhand_pose_raw_.linear() = hmd_lhand_pose_.linear() * DyrosMath::rotateWithZ(M_PI / 2); // absolute orientation

    master_rhand_pose_raw_.linear() = hmd_rhand_pose_.linear() * DyrosMath::rotateWithZ(-M_PI / 2); // absolute orientation

    master_lelbow_pose_raw_.translation().setZero();
    master_lelbow_pose_raw_.linear() = hmd_lupperarm_pose_.linear() * hmd_lupperarm_pose_init_.linear().transpose() * robot_lelbow_ori_init;

    master_relbow_pose_raw_.translation().setZero();
    master_relbow_pose_raw_.linear() = hmd_rupperarm_pose_.linear() * hmd_rupperarm_pose_init_.linear().transpose() * robot_relbow_ori_init;

    master_lshoulder_pose_raw_.translation().setZero();
    master_lshoulder_pose_raw_.linear() = hmd_lshoulder_pose_.linear() * hmd_lshoulder_pose_init_.linear().transpose() * robot_lshoulder_ori_init;

    master_rshoulder_pose_raw_.translation().setZero();
    master_rshoulder_pose_raw_.linear() = hmd_rshoulder_pose_.linear() * hmd_rshoulder_pose_init_.linear().transpose() * robot_rshoulder_ori_init;

    Vector3d hmd_head_displacement = hmd_head_pose_.translation() - hmd_head_pose_init_.translation();
    hmd_head_displacement(0) = DyrosMath::minmax_cut(hmd_head_displacement(0), -0.10, +0.10);
    hmd_head_displacement(1) = DyrosMath::minmax_cut(hmd_head_displacement(1), -0.15, +0.15);

    master_head_pose_raw_.translation() = hmd_head_displacement;
    master_head_pose_raw_.translation()(0) += 0.10;
    master_head_pose_raw_.linear() = hmd_head_pose_.linear() * hmd_head_pose_init_.linear().transpose() * robot_head_ori_init;

    // master_head_pose_raw_.linear() = hmd_head_pose_.linear();

    shoulder_diff_m = Eigen::AngleAxisd(chest_ang_diff.angle() * 1.0, chest_ang_diff.axis());
    master_lshoulder_pose_raw_.linear() = shoulder_diff_m * robot_upperbody_ori_init;
    master_rshoulder_pose_raw_.linear() = shoulder_diff_m * robot_upperbody_ori_init;
}
void AvatarController::hmdRawDataProcessing()
{
    ///////////////////////////////////MAPPING///////////////////////////////
    // Vector3d robot_still_pose_lhand, robot_t_pose_lhand, robot_forward_pose_lhand, robot_still_pose_rhand, robot_t_pose_rhand, robot_forward_pose_rhand;
    // Vector4d lhand_mapping_vector, rhand_mapping_vector;
    // MatrixXd lhand_master_ref_stack, lhand_robot_ref_stack, rhand_master_ref_stack, rhand_robot_ref_stack, lhand_master_ref_stack_pinverse, rhand_master_ref_stack_pinverse;
    // lhand_master_ref_stack.setZero(3, 4);
    // lhand_robot_ref_stack.setZero(3, 4);
    // lhand_master_ref_stack_pinverse.setZero(4, 3);
    // rhand_master_ref_stack.setZero(3, 4);
    // rhand_robot_ref_stack.setZero(3, 4);
    // rhand_master_ref_stack_pinverse.setZero(4, 3);

    // robot_still_pose_lhand.setZero();
    // robot_t_pose_lhand.setZero();
    // robot_forward_pose_lhand.setZero();
    // robot_still_pose_rhand.setZero();
    // robot_t_pose_rhand.setZero();
    // robot_forward_pose_rhand.setZero();

    // robot_still_pose_lhand(2) += -(robot_arm_max_l_);
    // robot_t_pose_lhand(1) += (robot_arm_max_l_);
    // robot_forward_pose_lhand(0) += (robot_arm_max_l_);

    // robot_still_pose_rhand(2) += -(robot_arm_max_l_);
    // robot_t_pose_rhand(1) += -(robot_arm_max_l_);
    // robot_forward_pose_rhand(0) += (robot_arm_max_l_);

    // lhand_master_ref_stack.block(0, 0, 3, 1) = hmd_still_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
    // lhand_master_ref_stack.block(0, 1, 3, 1) = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
    // lhand_master_ref_stack.block(0, 2, 3, 1) = hmd_forward_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
    // lhand_master_ref_stack.block(0, 3, 3, 1) = hmd_rshoulder_center_pos_ - hmd_lshoulder_center_pos_;

    // lhand_robot_ref_stack.block(0, 0, 3, 1) = robot_still_pose_lhand;
    // lhand_robot_ref_stack.block(0, 1, 3, 1) = robot_t_pose_lhand;
    // lhand_robot_ref_stack.block(0, 2, 3, 1) = robot_forward_pose_lhand;
    // lhand_robot_ref_stack(1, 3) = -robot_shoulder_width_;

    // lhand_master_ref_stack_pinverse = lhand_master_ref_stack.transpose() * (lhand_master_ref_stack * lhand_master_ref_stack.transpose() + damped_puedoinverse_eps_ * Eigen::Matrix3d::Identity()).inverse();
    // lhand_mapping_vector = lhand_master_ref_stack_pinverse * hmd_lshoulder_pose_init_.linear() * hmd_lshoulder_pose_.linear().transpose() *(hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());

    // hmd2robot_lhand_pos_mapping_ = lhand_robot_ref_stack * lhand_mapping_vector;

    // rhand_master_ref_stack.block(0, 0, 3, 1) = hmd_still_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
    // rhand_master_ref_stack.block(0, 1, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
    // rhand_master_ref_stack.block(0, 2, 3, 1) = hmd_forward_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
    // rhand_master_ref_stack.block(0, 3, 3, 1) = hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_;

    // rhand_robot_ref_stack.block(0, 0, 3, 1) = robot_still_pose_rhand;
    // rhand_robot_ref_stack.block(0, 1, 3, 1) = robot_t_pose_rhand;
    // rhand_robot_ref_stack.block(0, 2, 3, 1) = robot_forward_pose_rhand;
    // rhand_robot_ref_stack(1, 3) = robot_shoulder_width_;

    // rhand_master_ref_stack_pinverse = rhand_master_ref_stack.transpose() * (rhand_master_ref_stack * rhand_master_ref_stack.transpose() + damped_puedoinverse_eps_ * Eigen::Matrix3d::Identity()).inverse();
    // rhand_mapping_vector = rhand_master_ref_stack_pinverse * hmd_rshoulder_pose_init_.linear() * hmd_rshoulder_pose_.linear().transpose() *(hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

    // hmd2robot_rhand_pos_mapping_ = rhand_robot_ref_stack * rhand_mapping_vector;

    // if ((int(current_time_ * 2000) % int(1000) == 0))
    // {
    //     cout<<"lhand_mapping_vector: "<<lhand_mapping_vector.transpose()<<endl;
    //     cout<<"rhand_mapping_vector: "<<rhand_mapping_vector.transpose()<<endl;
    //     cout<<"E1_*lhand_mapping_vector_ - h_d_lhand_ "<<lhand_master_ref_stack*lhand_mapping_vector - hmd_lshoulder_pose_init_.linear() * hmd_lshoulder_pose_.linear().transpose() *(hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation()) <<endl;
    //     cout<<"E2_*rhand_mapping_vector_ - h_d_rhand_ "<<rhand_master_ref_stack*rhand_mapping_vector - hmd_rshoulder_pose_init_.linear() * hmd_rshoulder_pose_.linear().transpose() *(hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation()) <<endl;
    // }

    ///////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////QP MOTION RETARGETING///////////////////////////////////
    // if (first_loop_qp_retargeting_)
    // {
    //     QP_motion_retargeting_lhand_.InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_);
    //     QP_motion_retargeting_rhand_.InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_);

    //     lhand_master_ref_stack_.setZero(3, 4);
    //     lhand_robot_ref_stack_.setZero(3, 4);
    //     lhand_master_ref_stack_pinverse_.setZero(4, 3);
    //     rhand_master_ref_stack_.setZero(3, 4);
    //     rhand_robot_ref_stack_.setZero(3, 4);
    //     rhand_master_ref_stack_pinverse_.setZero(4, 3);

    //     robot_still_pose_lhand_.setZero();
    //     robot_t_pose_lhand_.setZero();
    //     robot_forward_pose_lhand_.setZero();
    //     robot_still_pose_rhand_.setZero();
    //     robot_t_pose_rhand_.setZero();
    //     robot_forward_pose_rhand_.setZero();

    //     lhand_mapping_vector_.setZero();
    //     rhand_mapping_vector_.setZero();

    //     robot_still_pose_lhand_(2) += -(robot_arm_max_l_);
    //     robot_t_pose_lhand_(1) += (robot_arm_max_l_);
    //     robot_forward_pose_lhand_(0) += (robot_arm_max_l_);

    //     robot_still_pose_rhand_(2) += -(robot_arm_max_l_);
    //     robot_t_pose_rhand_(1) += -(robot_arm_max_l_);
    //     robot_forward_pose_rhand_(0) += (robot_arm_max_l_);

    //     lhand_master_ref_stack_.block(0, 0, 3, 1) = hmd_still_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
    //     lhand_master_ref_stack_.block(0, 1, 3, 1) = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
    //     lhand_master_ref_stack_.block(0, 2, 3, 1) = hmd_forward_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
    //     lhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_rshoulder_center_pos_ - hmd_lshoulder_center_pos_;

    //     lhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_lhand_;
    //     lhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_lhand_;
    //     lhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_lhand_;
    //     lhand_robot_ref_stack_(1, 3) = -robot_shoulder_width_;

    //     rhand_master_ref_stack_.block(0, 0, 3, 1) = hmd_still_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
    //     rhand_master_ref_stack_.block(0, 1, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
    //     rhand_master_ref_stack_.block(0, 2, 3, 1) = hmd_forward_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
    //     rhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_;

    //     rhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_rhand_;
    //     rhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_rhand_;
    //     rhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_rhand_;
    //     rhand_robot_ref_stack_(1, 3) = robot_shoulder_width_;

    //     H_retargeting_lhand_.setZero(variable_size_retargeting_, variable_size_retargeting_);
    //     H_retargeting_rhand_.setZero(variable_size_retargeting_, variable_size_retargeting_);
    //     g_retargeting_lhand_.setZero(variable_size_retargeting_);
    //     g_retargeting_rhand_.setZero(variable_size_retargeting_);
    //     A_retargeting_lhand_.setZero(constraint_size2_retargeting_,variable_size_retargeting_);
    //     A_retargeting_rhand_.setZero(constraint_size2_retargeting_,variable_size_retargeting_);

    //     qpres_retargeting_.setZero(variable_size_retargeting_);
    //     ub_retargeting_.setZero(constraint_size1_retargeting_);
    //     lb_retargeting_.setZero(constraint_size1_retargeting_);
    //     ubA_retargeting_.setZero(constraint_size2_retargeting_);
    //     lbA_retargeting_.setZero(constraint_size2_retargeting_);

    //     first_loop_qp_retargeting_ = false;
    // }

    // lb_retargeting_(0) = -1;
    // lb_retargeting_(1) = 0;
    // lb_retargeting_(2) = -1;
    // lb_retargeting_(3) = 0;

    // ub_retargeting_(0) = 1;
    // ub_retargeting_(1) = 1;
    // ub_retargeting_(2) = 1;
    // ub_retargeting_(3) = 1;

    // lbA_retargeting_(0) = -100;
    // ubA_retargeting_(0) = 100;
    // //LEFT HAND
    // H_retargeting_lhand_ = lhand_master_ref_stack_.transpose()*lhand_master_ref_stack_;
    // H_retargeting_lhand_ += Eigen::MatrixXd::Identity(variable_size_retargeting_, variable_size_retargeting_)*1e-6;
    // g_retargeting_lhand_ = -lhand_master_ref_stack_.transpose() * hmd_lshoulder_pose_init_.linear() * hmd_lshoulder_pose_.linear().transpose() *(hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());

    // QP_motion_retargeting_lhand_.EnableEqualityCondition(equality_condition_eps_);
    // QP_motion_retargeting_lhand_.UpdateMinProblem(H_retargeting_lhand_, g_retargeting_lhand_);
    // QP_motion_retargeting_lhand_.UpdateSubjectToAx(A_retargeting_lhand_, lbA_retargeting_, ubA_retargeting_);
    // // QP_motion_retargeting_lhand_.DeleteSubjectToAx();
    // QP_motion_retargeting_lhand_.UpdateSubjectToX(lb_retargeting_, ub_retargeting_);

    // if (QP_motion_retargeting_lhand_.SolveQPoases(200, qpres_retargeting_))
    // {
    //     lhand_mapping_vector_ = qpres_retargeting_.segment(0, variable_size_retargeting_);
    // }
    // else
    // {
    //     QP_motion_retargeting_lhand_.InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_);
    //     // lhand_mapping_vector_.setZero(variable_size_retargeting_);

    //     if (int(current_time_ * 10000) % 1000 == 0)
    //         cout << "QP motion retargetng for left hand is not solved!!" << endl;
    // }

    // hmd2robot_lhand_pos_mapping_ = lhand_robot_ref_stack_ * lhand_mapping_vector_;

    // //RIGHT HAND
    // H_retargeting_rhand_ = rhand_master_ref_stack_.transpose()*rhand_master_ref_stack_;
    // H_retargeting_rhand_ += Eigen::MatrixXd::Identity(variable_size_retargeting_, variable_size_retargeting_)*1e-6;
    // g_retargeting_rhand_ = -rhand_master_ref_stack_.transpose() * hmd_rshoulder_pose_init_.linear() * hmd_rshoulder_pose_.linear().transpose() *(hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

    // QP_motion_retargeting_rhand_.EnableEqualityCondition(equality_condition_eps_);
    // QP_motion_retargeting_rhand_.UpdateMinProblem(H_retargeting_rhand_, g_retargeting_rhand_);
    // QP_motion_retargeting_rhand_.UpdateSubjectToAx(A_retargeting_rhand_, lbA_retargeting_, ubA_retargeting_);
    // // QP_motion_retargeting_rhand_.DeleteSubjectToAx();
    // QP_motion_retargeting_rhand_.UpdateSubjectToX(lb_retargeting_, ub_retargeting_);

    // if (QP_motion_retargeting_rhand_.SolveQPoases(200, qpres_retargeting_))
    // {
    //     rhand_mapping_vector_ = qpres_retargeting_.segment(0, variable_size_retargeting_);
    // }
    // else
    // {
    //     QP_motion_retargeting_rhand_.InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_);
    //     // rhand_mapping_vector_.setZero(variable_size_retargeting_);

    //     if (int(current_time_ * 10000) % 1000 == 0)
    //         cout << "QP motion retargetng for right hand is not solved!!" << endl;
    // }

    // hmd2robot_rhand_pos_mapping_ = rhand_robot_ref_stack_ * rhand_mapping_vector_;

    // if (int(current_time_ * 10000) % 1000 == 0)
    // {
    //     cout << "lhand_mapping_vector_:" << lhand_mapping_vector_ << endl;
    //     cout << "rhand_mapping_vector_:" << rhand_mapping_vector_ << endl;
    // }

    ///////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////HQP MOTION RETARGETING////////////////////////////////////////////
    if (first_loop_qp_retargeting_)
    {
        lhand_master_ref_stack_.setZero(3, 3);
        lhand_robot_ref_stack_.setZero(3, 3);
        rhand_master_ref_stack_.setZero(3, 3);
        rhand_robot_ref_stack_.setZero(3, 3);

        robot_still_pose_lhand_.setZero();
        robot_t_pose_lhand_.setZero();
        robot_forward_pose_lhand_.setZero();
        robot_still_pose_rhand_.setZero();
        robot_t_pose_rhand_.setZero();
        robot_forward_pose_rhand_.setZero();

        lhand_mapping_vector_.setZero();
        rhand_mapping_vector_.setZero();

        robot_still_pose_lhand_(2) += -(robot_arm_max_l_);
        robot_t_pose_lhand_(1) += (robot_arm_max_l_);
        robot_forward_pose_lhand_(0) += (robot_arm_max_l_);

        robot_still_pose_rhand_(2) += -(robot_arm_max_l_);
        robot_t_pose_rhand_(1) += -(robot_arm_max_l_);
        robot_forward_pose_rhand_(0) += (robot_arm_max_l_);

        lhand_master_ref_stack_.block(0, 0, 3, 1) = hmd_still_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        lhand_master_ref_stack_.block(0, 1, 3, 1) = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        lhand_master_ref_stack_.block(0, 2, 3, 1) = hmd_forward_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        // lhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
        // lhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_rshoulder_center_pos_ - hmd_lshoulder_center_pos_;

        lhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_lhand_;
        lhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_lhand_;
        lhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_lhand_;
        // lhand_robot_ref_stack_.block(0, 3, 3, 1) = robot_t_pose_lhand_;
        // lhand_robot_ref_stack_(1, 3) = -robot_shoulder_width_;

        rhand_master_ref_stack_.block(0, 0, 3, 1) = hmd_still_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        rhand_master_ref_stack_.block(0, 1, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        rhand_master_ref_stack_.block(0, 2, 3, 1) = hmd_forward_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        // rhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        // rhand_master_ref_stack_.block(0, 3, 3, 1) = hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_;

        rhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_rhand_;
        rhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_rhand_;
        rhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_rhand_;
        // rhand_robot_ref_stack_.block(0, 3, 3, 1) = robot_t_pose_rhand_;
        // rhand_robot_ref_stack_(1, 3) = robot_shoulder_width_;

        E1_.setZero(control_size_retargeting_[0], variable_size_retargeting_);
        E2_.setZero(control_size_retargeting_[1], variable_size_retargeting_);
        E3_.setZero(control_size_retargeting_[2], variable_size_retargeting_);
        H_retargeting_.setZero(variable_size_retargeting_, variable_size_retargeting_);
        g_retargeting_.setZero(variable_size_retargeting_);
        u1_.setZero(control_size_retargeting_[0]);
        u2_.setZero(control_size_retargeting_[1]);
        u3_.setZero(control_size_retargeting_[2]);

        ub_retargeting_.setZero(constraint_size1_retargeting_);
        lb_retargeting_.setZero(constraint_size1_retargeting_);

        E1_.block(0, 0, 3, 3) = lhand_master_ref_stack_;
        E2_.block(0, 3, 3, 3) = rhand_master_ref_stack_;
        E3_.block(0, 0, 3, 3) = lhand_robot_ref_stack_;
        E3_.block(0, 3, 3, 3) = -rhand_robot_ref_stack_;

        for (int i = 0; i < constraint_size1_retargeting_; i++)
        {
            ub_retargeting_(i) = w_dot_max_;
            lb_retargeting_(i) = w_dot_min_;
        }

        w1_retargeting_ = 1;
        w2_retargeting_ = 1;
        w3_retargeting_ = 1;
        human_shoulder_width_ = (hmd_rshoulder_center_pos_ - hmd_lshoulder_center_pos_).norm();

        Eigen::MatrixXd lhand_master_ref_stack_pinverse_ = lhand_master_ref_stack_.transpose() * (lhand_master_ref_stack_ * lhand_master_ref_stack_.transpose() + damped_puedoinverse_eps_ * Eigen::Matrix3d::Identity()).inverse();
        lhand_mapping_vector_pre_ = lhand_master_ref_stack_pinverse_ * hmd_lshoulder_pose_init_.linear() * hmd_lshoulder_pose_.linear().transpose() * (hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());

        Eigen::MatrixXd rhand_master_ref_stack_pinverse_ = rhand_master_ref_stack_.transpose() * (rhand_master_ref_stack_ * rhand_master_ref_stack_.transpose() + damped_puedoinverse_eps_ * Eigen::Matrix3d::Identity()).inverse();
        rhand_mapping_vector_pre_ = rhand_master_ref_stack_pinverse_ * hmd_rshoulder_pose_init_.linear() * hmd_rshoulder_pose_.linear().transpose() * (hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

        h_pre_lhand_ = (hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());
        h_pre_rhand_ = (hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

        for (int i = 0; i < 3; i++)
        {
            QP_motion_retargeting_[i].InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_[i]);

            A_retargeting_[i].setZero(constraint_size2_retargeting_[i], variable_size_retargeting_);
            ubA_retargeting_[i].setZero(constraint_size2_retargeting_[i]);
            lbA_retargeting_[i].setZero(constraint_size2_retargeting_[i]);

            A_retargeting_[i].block(0, 0, 3, 3) = lhand_master_ref_stack_;
            A_retargeting_[i].block(3, 3, 3, 3) = rhand_master_ref_stack_;

            for (int j = 0; j < 6; j++)
            {
                ubA_retargeting_[i](j) = human_vel_max_;
                lbA_retargeting_[i](j) = human_vel_min_;
            }

            qpres_retargeting_[i].setZero(variable_size_retargeting_);
        }

        first_loop_qp_retargeting_ = false;
    }
    else
    {
        double speed_reduce_rate = 20;

        ub_retargeting_(0) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(0)), w_dot_max_);
        ub_retargeting_(1) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(1)), w_dot_max_);
        ub_retargeting_(2) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(2)), w_dot_max_);
        // ub_retargeting_(3) = min(speed_reduce_rate * (1.0 - lhand_mapping_vector_pre_(3)), w_dot_max_);
        ub_retargeting_(3) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(0)), w_dot_max_);
        ub_retargeting_(4) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(1)), w_dot_max_);
        ub_retargeting_(5) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(2)), w_dot_max_);
        // ub_retargeting_(7) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(3)), w_dot_max_);

        lb_retargeting_(0) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(0)), w_dot_min_);
        lb_retargeting_(1) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(1)), w_dot_min_);
        lb_retargeting_(2) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(2)), w_dot_min_);
        // lb_retargeting_(3) = max(speed_reduce_rate * (-1.0 - lhand_mapping_vector_pre_(3)), w_dot_min_);
        lb_retargeting_(3) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(0)), w_dot_min_);
        lb_retargeting_(4) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(1)), w_dot_min_);
        lb_retargeting_(5) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(2)), w_dot_min_);
        // lb_retargeting_(7) = max(speed_reduce_rate * (-1.0 - rhand_mapping_vector_pre_(3)), w_dot_min_);

        h_pre_lhand_ = lhand_master_ref_stack_ * lhand_mapping_vector_pre_;
        h_pre_rhand_ = rhand_master_ref_stack_ * rhand_mapping_vector_pre_;
        r_pre_lhand_ = lhand_robot_ref_stack_ * lhand_mapping_vector_pre_;
        r_pre_rhand_ = rhand_robot_ref_stack_ * rhand_mapping_vector_pre_;
    }

    double hand_d = (hmd_lhand_pose_.translation() - hmd_rhand_pose_.translation()).norm();
    // double beta = DyrosMath::cubic(hand_d, human_shoulder_width_+0.2, human_shoulder_width_-0.1, 1, 0, 0, 0);    // cubic transition
    double beta = 0;
    // double beta = DyrosMath::minmax_cut( (hand_d - human_shoulder_width_) / (-0.2), 0.0, 1.0);  //linear transition

    if (beta == 0)
    {
        qpRetargeting_1(); // calc lhand_mapping_vector_, rhand_mapping_vector_ //1025
        // if ((int(current_time_ * 1e4) % int(1e4) == 0))
        // {
        //     cout << "beta0: " << beta << endl;
        // }
    }
    else if (beta == 1)
    {
        qpRetargeting_21();
        // if ((int(current_time_ * 1e4) % int(1e4) == 0))
        // {
        //     cout << "beta1: " << beta << endl;
        // }
    }
    else // transition
    {
        qpRetargeting_1();
        qpRetargeting_21Transition(beta); // qpRetargeting_1() must be preceded
        // if ((int(current_time_ * 1e4) % int(1e4) == 0))
        // {
        //     cout << "beta0~1: " << beta << endl;
        // }
    }

    // if ((int(current_time_ * 2000) % int(1000) == 0))
    // {
    //     cout<<"lhand_mapping_vector_: "<<lhand_mapping_vector_.transpose()<<endl;
    //     cout<<"rhand_mapping_vector_: "<<rhand_mapping_vector_.transpose()<<endl;
    //     cout<<"E1_*lhand_mapping_vector_ - h_d_lhand_ "<<E1_.block(0, 0, 3, 3)*lhand_mapping_vector_ - h_d_lhand_ <<endl;
    //     cout<<"E2_*rhand_mapping_vector_ - h_d_rhand_ "<<E2_.block(0, 3, 3, 3)*rhand_mapping_vector_ - h_d_rhand_ <<endl;
    // }

    hmd2robot_lhand_pos_mapping_ = lhand_robot_ref_stack_ * lhand_mapping_vector_;
    hmd2robot_rhand_pos_mapping_ = rhand_robot_ref_stack_ * rhand_mapping_vector_;

    // if (upper_body_mode_ == 10)
    // {
    //     hmd2robot_lhand_pos_mapping_ *= hmd_larm_max_l_ / robot_arm_max_l_;
    //     hmd2robot_rhand_pos_mapping_ *= hmd_rarm_max_l_ / robot_arm_max_l_;
    // }

    // if( int(current_time_ *100000)%1000 == 0)
    // {
    //     cout<<" lhand_mapping_vector_"<< lhand_mapping_vector_.transpose() << endl;
    //     cout<<" Rhand_mapping_vector_"<< rhand_mapping_vector_.transpose() << endl;
    // }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (hmd2robot_lhand_pos_mapping_.norm() > robot_arm_max_l_)
    {
        hmd2robot_lhand_pos_mapping_ = hmd2robot_lhand_pos_mapping_.normalized() * robot_arm_max_l_;
    }

    if (hmd2robot_rhand_pos_mapping_.norm() > robot_arm_max_l_)
    {
        hmd2robot_rhand_pos_mapping_ = hmd2robot_rhand_pos_mapping_.normalized() * robot_arm_max_l_;
    }

    if (hmd2robot_lhand_pos_mapping_.norm() < 0.1)
    {
        hmd2robot_lhand_pos_mapping_ = hmd2robot_lhand_pos_mapping_.normalized() * 0.1;
    }

    if (hmd2robot_rhand_pos_mapping_.norm() < 0.1)
    {
        hmd2robot_rhand_pos_mapping_ = hmd2robot_rhand_pos_mapping_.normalized() * 0.1;
    }



    Vector3d robot_init_hand_pos, robot_init_lshoulder_pos, robot_init_rshoulder_pos, delta_hmd2robot_lhand_pos_maping, delta_hmd2robot_rhand_pos_maping, delta_hmd2robot_lelbow_pos_maping, delta_hmd2robot_relbow_pos_maping;
    robot_init_hand_pos << 0, 0, -(robot_arm_max_l_);
    robot_init_lshoulder_pos << 0, 0.1491, 0.065;
    robot_init_rshoulder_pos << 0, -0.1491, 0.065;
    Matrix3d robot_lhand_ori_init, robot_rhand_ori_init, robot_lelbow_ori_init, robot_relbow_ori_init, robot_lshoulder_ori_init, robot_rshoulder_ori_init, robot_head_ori_init, robot_upperbody_ori_init;
    robot_lhand_ori_init = DyrosMath::rotateWithZ(-90 * DEG2RAD);
    robot_rhand_ori_init = DyrosMath::rotateWithZ(90 * DEG2RAD);
    // robot_lshoulder_ori_init = DyrosMath::rotateWithZ(-0.3);
    robot_lshoulder_ori_init.setIdentity();
    // robot_rshoulder_ori_init = DyrosMath::rotateWithZ(0.3);
    robot_rshoulder_ori_init.setIdentity();
    robot_head_ori_init.setIdentity();
    robot_upperbody_ori_init.setIdentity();

    // robot_lelbow_ori_init << 0, 0, -1, 1, 0, 0, 0, -1, 0;
    robot_lelbow_ori_init.setZero();
    robot_lelbow_ori_init(0, 2) = -1;
    robot_lelbow_ori_init(1, 0) = 1;
    robot_lelbow_ori_init(2, 1) = -1;
    robot_lelbow_ori_init = DyrosMath::rotateWithZ(-0 * DEG2RAD) * robot_lelbow_ori_init;

    robot_relbow_ori_init.setZero();
    robot_relbow_ori_init(0, 2) = -1;
    robot_relbow_ori_init(1, 0) = -1;
    robot_relbow_ori_init(2, 1) = 1;
    robot_relbow_ori_init = DyrosMath::rotateWithZ(0 * DEG2RAD) * robot_relbow_ori_init;

    // delta_hmd2robot_lhand_pos_maping = hmd2robot_lhand_pos_mapping_ - hmd2robot_lhand_pos_mapping_init_;
    // delta_hmd2robot_rhand_pos_maping = hmd2robot_rhand_pos_mapping_ - hmd2robot_rhand_pos_mapping_init_;

    // hmd2robot_lhand_pos_mapping_ = robot_init_hand_pos + delta_hmd2robot_lhand_pos_maping;
    // hmd2robot_rhand_pos_mapping_ = robot_init_hand_pos + delta_hmd2robot_rhand_pos_maping;

    master_upperbody_pose_raw_.translation().setZero();
    Eigen::AngleAxisd chest_ang_diff(hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose());
    Eigen::Matrix3d chest_diff_m, shoulder_diff_m;
    chest_diff_m = Eigen::AngleAxisd(chest_ang_diff.angle() * 1.0, chest_ang_diff.axis());
    master_upperbody_pose_raw_.linear() = chest_diff_m * robot_upperbody_ori_init;
    // master_upperbody_pose_raw_.linear() = hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose()*robot_upperbody_ori_init;

    master_lhand_pose_raw_.translation() = larmbase_transform_pre_desired_from_.translation() + upperbody_transform_pre_desired_from_.linear() * (robot_init_lshoulder_pos + hmd2robot_lhand_pos_mapping_);
    // master_lhand_pose_raw_.linear() = hmd_lhand_pose_.linear()*hmd_lhand_pose_init_.linear().transpose()*robot_lhand_ori_init;	//relative orientation
    master_lhand_pose_raw_.linear() = hmd_lhand_pose_.linear() * DyrosMath::rotateWithZ(M_PI / 2); // absolute orientation

    master_rhand_pose_raw_.translation() = rarmbase_transform_pre_desired_from_.translation() + upperbody_transform_pre_desired_from_.linear() * (robot_init_rshoulder_pos + hmd2robot_rhand_pos_mapping_);
    // master_rhand_pose_raw_.linear() = hmd_rhand_pose_.linear()*hmd_rhand_pose_init_.linear().transpose()*robot_rhand_ori_init;	//relative orientation
    master_rhand_pose_raw_.linear() = hmd_rhand_pose_.linear() * DyrosMath::rotateWithZ(-M_PI / 2); // absolute orientation

    // master_lelbow_pose_raw_.translation() = lshoulder_transform_pre_desired_from_.translation() + hmd2robot_lelbow_pos_mapping_;
    master_lelbow_pose_raw_.translation().setZero();
    master_lelbow_pose_raw_.linear() = hmd_lupperarm_pose_.linear() * hmd_lupperarm_pose_init_.linear().transpose() * robot_lelbow_ori_init;

    // master_relbow_pose_raw_.translation() = rshoulder_transform_pre_desired_from_.translation() + hmd2robot_relbow_pos_mapping_;
    master_relbow_pose_raw_.translation().setZero();
    master_relbow_pose_raw_.linear() = hmd_rupperarm_pose_.linear() * hmd_rupperarm_pose_init_.linear().transpose() * robot_relbow_ori_init;

    master_lshoulder_pose_raw_.translation().setZero();
    master_lshoulder_pose_raw_.linear() = hmd_lshoulder_pose_.linear() * hmd_lshoulder_pose_init_.linear().transpose() * robot_lshoulder_ori_init;

    master_rshoulder_pose_raw_.translation().setZero();
    master_rshoulder_pose_raw_.linear() = hmd_rshoulder_pose_.linear() * hmd_rshoulder_pose_init_.linear().transpose() * robot_rshoulder_ori_init;

    Vector3d hmd_head_displacement = hmd_head_pose_.translation() - hmd_head_pose_init_.translation();
    hmd_head_displacement(0) = DyrosMath::minmax_cut(hmd_head_displacement(0), -0.10, +0.10);
    hmd_head_displacement(1) = DyrosMath::minmax_cut(hmd_head_displacement(1), -0.15, +0.15);

    master_head_pose_raw_.translation() = hmd_head_displacement;
    master_head_pose_raw_.translation()(0) += 0.10;
    master_head_pose_raw_.linear() = hmd_head_pose_.linear() * hmd_head_pose_init_.linear().transpose() * robot_head_ori_init;

    // master_head_pose_raw_.linear() = hmd_head_pose_.linear();

    shoulder_diff_m = Eigen::AngleAxisd(chest_ang_diff.angle() * 1.0, chest_ang_diff.axis());
    master_lshoulder_pose_raw_.linear() = shoulder_diff_m * robot_upperbody_ori_init;
    master_rshoulder_pose_raw_.linear() = shoulder_diff_m * robot_upperbody_ori_init;

    master_relative_lhand_pos_raw_ = hmd_lhand_pose_.translation() - hmd_rhand_pose_.translation();
    master_relative_rhand_pos_raw_ = hmd_rhand_pose_.translation() - hmd_lhand_pose_.translation();
    master_relative_lhand_pos_raw_ = master_relative_lhand_pos_raw_ * (robot_shoulder_width_) / (hmd_shoulder_width_);
    master_relative_rhand_pos_raw_ = master_relative_lhand_pos_raw_ * (robot_shoulder_width_) / (hmd_shoulder_width_);

    if (int(current_time_ * 1e4) % int(2e4) == 0)
    {
        // cout<<"master_lelbow_pose_raw_.linear(): \n"<<master_lelbow_pose_raw_.linear()<<endl;
        // cout<<"master_lelbow_pose_.linear(): \n"<<master_lelbow_pose_.linear()<<endl;
        // cout<<"master_relbow_pose_raw_.linear(): \n"<<master_relbow_pose_raw_.linear()<<endl;
        // cout<<"master_relbow_pose_.linear(): \n"<<master_relbow_pose_.linear()<<endl;
        // cout<<"master_lhand_pose_raw_.translation(): \n"<<master_lhand_pose_raw_.translation()<<endl;

        // cout<<"master_lelbow_pose_raw_.linear(): \n"<<master_lelbow_pose_raw_.linear()<<endl;
        // cout<<"master_relbow_pose_raw_.linear(): \n"<<master_relbow_pose_raw_.linear()<<endl;
        // cout<<"left hand ori mat: \n"<<lhand_transform_current_from_global_.linear()<<endl;
        // cout<<"command arm length: "<<(lshoulder_transform_current_from_global_.translation() - master_lhand_pose_.translation()).norm() <<endl;
        // cout<<"lhand_mapping_vector: "<<lhand_mapping_vector<<endl;
        // cout<<"rhand_mapping_vector: "<<rhand_mapping_vector<<endl;
    }
}

void AvatarController::qpRetargeting_1()
{
    h_d_lhand_ = hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());
    h_d_rhand_ = hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

    u1_ = control_gain_retargeting_ * (h_d_lhand_ - h_pre_lhand_);
    u2_ = control_gain_retargeting_ * (h_d_rhand_ - h_pre_rhand_);

    H_retargeting_ = w1_retargeting_ * E1_.transpose() * E1_ + w2_retargeting_ * E2_.transpose() * E2_ + Eigen::MatrixXd::Identity(6, 6) * damped_puedoinverse_eps_;
    g_retargeting_ = -w1_retargeting_ * E1_.transpose() * u1_ - w2_retargeting_ * E2_.transpose() * u2_;

    QP_motion_retargeting_[0].EnableEqualityCondition(equality_condition_eps_);
    QP_motion_retargeting_[0].UpdateMinProblem(H_retargeting_, g_retargeting_);
    QP_motion_retargeting_[0].UpdateSubjectToAx(A_retargeting_[0], lbA_retargeting_[0], ubA_retargeting_[0]);
    QP_motion_retargeting_[0].UpdateSubjectToX(lb_retargeting_, ub_retargeting_);

    // if (int(current_time_ * 10000) % 1000 == 0)
    // {
    //     cout << "lb_retargeting_: " << lb_retargeting_.transpose() << endl;
    //     cout << "ub_retargeting_: " << ub_retargeting_.transpose() << endl;
    // }

    if (QP_motion_retargeting_[0].SolveQPoases(200, qpres_retargeting_[0]))
    {
        lhand_mapping_vector_dot_ = qpres_retargeting_[0].segment(0, 3);
        rhand_mapping_vector_dot_ = qpres_retargeting_[0].segment(3, 3);

        lhand_mapping_vector_ = lhand_mapping_vector_pre_ + lhand_mapping_vector_dot_ * dt_;
        rhand_mapping_vector_ = rhand_mapping_vector_pre_ + rhand_mapping_vector_dot_ * dt_;
    }
    else
    {
        QP_motion_retargeting_[0].InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_[0]);
        lhand_mapping_vector_ = lhand_mapping_vector_pre_;
        rhand_mapping_vector_ = rhand_mapping_vector_pre_;
        if (int(current_time_ * 2000) % 1000 == 0)
            cout << "QP motion retargetng is not solved!! (beta == 0)" << endl;
    }
}
void AvatarController::qpRetargeting_21()
{
    h_d_lhand_ = hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());
    h_d_rhand_ = hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

    Vector3d r2l_robot_shoulder;
    r2l_robot_shoulder.setZero();
    r2l_robot_shoulder(1) = robot_shoulder_width_;

    u1_ = control_gain_retargeting_ * (h_d_lhand_ - h_pre_lhand_);
    u2_ = control_gain_retargeting_ * (h_d_rhand_ - h_pre_rhand_);
    u3_ = control_gain_retargeting_ * (robot_shoulder_width_ / human_shoulder_width_ * hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_lhand_pose_.translation() - hmd_rhand_pose_.translation()) - (r_pre_lhand_ + r2l_robot_shoulder - r_pre_rhand_));

    H_retargeting_ = w3_retargeting_ * E3_.transpose() * E3_ + Eigen::MatrixXd::Identity(variable_size_retargeting_, variable_size_retargeting_) * damped_puedoinverse_eps_;
    g_retargeting_ = -w3_retargeting_ * E3_.transpose() * u3_;

    QP_motion_retargeting_[1].EnableEqualityCondition(equality_condition_eps_);
    QP_motion_retargeting_[1].UpdateMinProblem(H_retargeting_, g_retargeting_);
    QP_motion_retargeting_[1].UpdateSubjectToAx(A_retargeting_[1], lbA_retargeting_[1], ubA_retargeting_[1]);
    QP_motion_retargeting_[1].UpdateSubjectToX(lb_retargeting_, ub_retargeting_);

    if (QP_motion_retargeting_[1].SolveQPoases(200, qpres_retargeting_[1]))
    {
        H_retargeting_ = w1_retargeting_ * E1_.transpose() * E1_ + w2_retargeting_ * E2_.transpose() * E2_ + Eigen::MatrixXd::Identity(variable_size_retargeting_, variable_size_retargeting_) * damped_puedoinverse_eps_;
        g_retargeting_ = -w1_retargeting_ * E1_.transpose() * u1_ - w2_retargeting_ * E2_.transpose() * u2_;

        A_retargeting_[2].block(6, 0, 3, variable_size_retargeting_) = E3_;
        lbA_retargeting_[2].segment(6, 3) = E3_ * qpres_retargeting_[1];
        ubA_retargeting_[2].segment(6, 3) = E3_ * qpres_retargeting_[1];

        QP_motion_retargeting_[2].EnableEqualityCondition(equality_condition_eps_);
        QP_motion_retargeting_[2].UpdateMinProblem(H_retargeting_, g_retargeting_);
        QP_motion_retargeting_[2].UpdateSubjectToAx(A_retargeting_[2], lbA_retargeting_[2], ubA_retargeting_[2]);
        QP_motion_retargeting_[2].UpdateSubjectToX(lb_retargeting_, ub_retargeting_);

        if (QP_motion_retargeting_[2].SolveQPoases(200, qpres_retargeting_[2]))
        {
            lhand_mapping_vector_dot_ = qpres_retargeting_[2].segment(0, 3);
            rhand_mapping_vector_dot_ = qpres_retargeting_[2].segment(3, 3);

            lhand_mapping_vector_ = lhand_mapping_vector_pre_ + lhand_mapping_vector_dot_ * dt_;
            rhand_mapping_vector_ = rhand_mapping_vector_pre_ + rhand_mapping_vector_dot_ * dt_;
        }
        else
        {
            QP_motion_retargeting_[2].InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_[2]);

            lhand_mapping_vector_dot_ = qpres_retargeting_[1].segment(0, 3);
            rhand_mapping_vector_dot_ = qpres_retargeting_[1].segment(3, 3);

            lhand_mapping_vector_ = lhand_mapping_vector_pre_ + lhand_mapping_vector_dot_ * dt_;
            rhand_mapping_vector_ = rhand_mapping_vector_pre_ + rhand_mapping_vector_dot_ * dt_;
            if (int(current_time_ * 2000) % 1000 == 0)
                cout << "QP motion retargetng is not solved!! (beta == 1, second qp)" << endl;
        }
    }
    else
    {
        QP_motion_retargeting_[1].InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_[1]);

        if (int(current_time_ * 2000) % 1000 == 0)
            cout << "QP motion retargetng is not solved!! (beta == 1, first qp)" << endl;
    }
}
void AvatarController::qpRetargeting_21Transition(double beta)
{
    h_d_lhand_ = hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());
    h_d_rhand_ = hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

    Vector3d r2l_robot_shoulder;
    r2l_robot_shoulder.setZero();
    r2l_robot_shoulder(1) = robot_shoulder_width_;

    u1_ = control_gain_retargeting_ * (h_d_lhand_ - h_pre_lhand_);
    u2_ = control_gain_retargeting_ * (h_d_rhand_ - h_pre_rhand_);
    u3_ = control_gain_retargeting_ * (robot_shoulder_width_ / human_shoulder_width_ * hmd_chest_pose_init_.linear() * hmd_chest_pose_.linear().transpose() * (hmd_lhand_pose_.translation() - hmd_rhand_pose_.translation()) - (r_pre_lhand_ + r2l_robot_shoulder - r_pre_rhand_));

    H_retargeting_ = w3_retargeting_ * E3_.transpose() * E3_ + Eigen::MatrixXd::Identity(variable_size_retargeting_, variable_size_retargeting_) * damped_puedoinverse_eps_;
    g_retargeting_ = -w3_retargeting_ * E3_.transpose() * u3_;

    QP_motion_retargeting_[1].EnableEqualityCondition(equality_condition_eps_);
    QP_motion_retargeting_[1].UpdateMinProblem(H_retargeting_, g_retargeting_);
    QP_motion_retargeting_[1].UpdateSubjectToAx(A_retargeting_[1], lbA_retargeting_[1], ubA_retargeting_[1]);
    QP_motion_retargeting_[1].UpdateSubjectToX(lb_retargeting_, ub_retargeting_);

    if (QP_motion_retargeting_[1].SolveQPoases(200, qpres_retargeting_[1]))
    {
        H_retargeting_ = w1_retargeting_ * E1_.transpose() * E1_ + w2_retargeting_ * E2_.transpose() * E2_ + Eigen::MatrixXd::Identity(variable_size_retargeting_, variable_size_retargeting_) * damped_puedoinverse_eps_;
        g_retargeting_ = -w1_retargeting_ * E1_.transpose() * u1_ - w2_retargeting_ * E2_.transpose() * u2_;

        A_retargeting_[2].block(6, 0, 3, variable_size_retargeting_) = E3_;
        lbA_retargeting_[2].segment(6, 3) = beta * (E3_ * qpres_retargeting_[1]) + (1 - beta) * (E3_ * qpres_retargeting_[0]);
        ubA_retargeting_[2].segment(6, 3) = beta * (E3_ * qpres_retargeting_[1]) + (1 - beta) * (E3_ * qpres_retargeting_[0]);

        QP_motion_retargeting_[2].EnableEqualityCondition(equality_condition_eps_);
        QP_motion_retargeting_[2].UpdateMinProblem(H_retargeting_, g_retargeting_);
        QP_motion_retargeting_[2].UpdateSubjectToAx(A_retargeting_[2], lbA_retargeting_[2], ubA_retargeting_[2]);
        QP_motion_retargeting_[2].UpdateSubjectToX(lb_retargeting_, ub_retargeting_);

        if (QP_motion_retargeting_[2].SolveQPoases(200, qpres_retargeting_[2]))
        {
            lhand_mapping_vector_dot_ = qpres_retargeting_[2].segment(0, 3);
            rhand_mapping_vector_dot_ = qpres_retargeting_[2].segment(3, 3);

            lhand_mapping_vector_ = lhand_mapping_vector_pre_ + lhand_mapping_vector_dot_ * dt_;
            rhand_mapping_vector_ = rhand_mapping_vector_pre_ + rhand_mapping_vector_dot_ * dt_;
        }
        else
        {
            QP_motion_retargeting_[2].InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_[2]);

            lhand_mapping_vector_dot_ = qpres_retargeting_[1].segment(0, 3);
            rhand_mapping_vector_dot_ = qpres_retargeting_[1].segment(3, 3);

            lhand_mapping_vector_ = lhand_mapping_vector_pre_ + lhand_mapping_vector_dot_ * dt_;
            rhand_mapping_vector_ = rhand_mapping_vector_pre_ + rhand_mapping_vector_dot_ * dt_;

            if (int(current_time_ * 2000) % 1000 == 0)
                cout << "QP motion retargetng is not solved!! (transition, second qp)" << endl;
        }
    }
    else
    {
        QP_motion_retargeting_[1].InitializeProblemSize(variable_size_retargeting_, constraint_size2_retargeting_[1]);

        if (int(current_time_ * 2000) % 1000 == 0)
            cout << "QP motion retargetng is not solved!! (transition, first qp)" << endl;
    }
}
void AvatarController::floatingBaseMOB()
{
    Eigen::VectorXd nonlinear_torque_temp;
    nonlinear_torque_temp.setZero(MODEL_DOF_VIRTUAL, 1);

    A_mat_pre_ = A_mat_;

    Eigen::MatrixXd A_temp;
    A_temp.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_global_, q_virtual_Xd_global_, A_temp, false);

    A_mat_ = A_temp;

    A_dot_mat_ = (A_mat_ - A_mat_pre_) * hz_;

    RigidBodyDynamics::NonlinearEffects(model_global_, q_virtual_Xd_global_, q_dot_virtual_Xd_global_, nonlinear_torque_temp);

    nonlinear_torque_ = nonlinear_torque_temp;

    Eigen::VectorXd current_torque, mob_residual_pre_wholebody;

    current_torque = torque_lower_ + torque_upper_;
    
    mob_residual_pre_wholebody = mob_residual_wholebody_;

    VectorXd momentum_virtual, current_torque_virtual, nonlinear_term_virtual;
    momentum_virtual.setZero(MODEL_DOF_VIRTUAL);
    momentum_virtual = A_mat_ * q_dot_virtual_Xd_global_;
    current_torque_virtual.setZero(MODEL_DOF_VIRTUAL);
    current_torque_virtual.segment(6, MODEL_DOF) = current_torque.segment(0, MODEL_DOF);
    nonlinear_term_virtual.setZero(MODEL_DOF_VIRTUAL);
    nonlinear_term_virtual = A_dot_mat_ * q_dot_virtual_Xd_global_ - nonlinear_torque_;

    mob_residual_wholebody_ = momentumObserverCore(momentum_virtual, current_torque_virtual, nonlinear_term_virtual, mob_residual_pre_wholebody, mob_integral_wholebody_, 1 / hz_, 100);

}
Eigen::VectorXd AvatarController::momentumObserverCore(VectorXd current_momentum, VectorXd current_torque, VectorXd nonlinear_term, VectorXd mob_residual_pre, VectorXd &mob_residual_integral, double dt, double k)
{
    // input: current_momentum, current_torque, nonlinear term, mob_residual_pre, mob_integral, dt, k
    // output: mob_residual
    const int dof = current_momentum.size();
    Eigen::VectorXd mob_residual(dof);

    mob_residual_integral = mob_residual_integral + dt * (current_torque + nonlinear_term + mob_residual_pre);
    mob_residual = k * (current_momentum - mob_residual_integral);

    return mob_residual;
}
MatrixXd AvatarController::getCMatrix(VectorXd q, VectorXd qdot)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double h = 1e-16;
    const int dof = 39;

    Eigen::VectorXd q_new = q;
    Eigen::MatrixXd C(dof, dof);
    Eigen::MatrixXd C1(dof, dof);
    C1.setZero();
    Eigen::MatrixXd C2(dof, dof);
    C2.setZero();
    Eigen::MatrixXd H_origin(dof, dof), H_new(dof, dof);
    Eigen::MatrixXd m[dof];
    double b[dof][dof][dof];
    H_origin.setZero();
    H_origin = A_mat_;
    H_new.setZero();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    for (int i = 0; i < dof; i++)
    {
        if (i >= 3 & i < 6) // quaternion exeption
        {
            Quaterniond quat(q(39), q(3), q(4), q(5));
            Eigen::Matrix3d rotm;
            rotm = quat.normalized().toRotationMatrix();
            if (i == 3)
            {
                rotm = rotm * DyrosMath::rotateWithX(h);
                // rotm = DyrosMath::rotateWithX(h)*rotm;
            }
            else if (i == 4)
            {
                rotm = rotm * DyrosMath::rotateWithY(h);
                // rotm = DyrosMath::rotateWithY(h)*rotm;
            }
            else if (i == 5)
            {
                rotm = rotm * DyrosMath::rotateWithZ(h);
                // rotm = DyrosMath::rotateWithZ(h)*rotm;
            }

            Quaterniond quat_new(rotm);
            q_new = q;
            q_new(3) = quat_new.x();
            q_new(4) = quat_new.y();
            q_new(5) = quat_new.z();
            q_new(39) = quat_new.w();
        }
        else
        {
            q_new = q;
            q_new(i) += h;
        }

        // RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_C_, q, H_origin, true);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_C_, q_new, H_new, true);
        m[i].resize(dof, dof);
        m[i] = (H_new - H_origin) / h;

        // cout<<"m["<<i<<"]: \n" << (H_new - H_origin)<< endl;
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    for (int i = 0; i < dof; i++)
        for (int j = 0; j < dof; j++)
            for (int k = 0; k < dof; k++)
                b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    C.setZero();

    for (int i = 0; i < dof; i++)
        for (int j = 0; j < dof; j++)
            C1(i, j) = b[i][j][j] * qdot(j);
    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    for (int k = 0; k < dof; k++)
        for (int j = 0; j < dof; j++)
            for (int i = 1 + j; i < dof; i++)
                C2(k, j) += 2.0 * b[k][j][i] * qdot(i);
    std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
    C = C1 + C2;

    // cout<<"getCMatrix #1: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() <<endl;
    // cout<<"getCMatrix #2: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count() <<endl;
    // cout<<"getCMatrix #3: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count() <<endl;
    // cout<<"getCMatrix #4: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t5 - t4).count() <<endl;
    // cout<<"getCMatrix #5: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t6 - t5).count() <<endl;

    return C;
}

void AvatarController::savePreData()
{
    pre_time_ = current_time_;
    pre_q_ = rd_.q_;
    pre_desired_q_ = desired_q_;
    pre_desired_q_dot_ = desired_q_dot_;
    motion_q_pre_ = motion_q_;
    motion_q_dot_pre_ = motion_q_dot_;


    master_lhand_pose_raw_ppre_ = master_lhand_pose_raw_pre_;
    master_lhand_pose_raw_ppre_ = master_lhand_pose_raw_pre_;
    master_rhand_pose_raw_ppre_ = master_rhand_pose_raw_pre_;
    master_head_pose_raw_ppre_ = master_head_pose_raw_pre_;
    master_lelbow_pose_raw_ppre_ = master_lelbow_pose_raw_pre_;
    master_relbow_pose_raw_ppre_ = master_relbow_pose_raw_pre_;
    master_lshoulder_pose_raw_ppre_ = master_lshoulder_pose_raw_pre_;
    master_rshoulder_pose_raw_ppre_ = master_rshoulder_pose_raw_pre_;
    master_upperbody_pose_raw_ppre_ = master_upperbody_pose_raw_pre_;

    master_lhand_pose_raw_pre_ = master_lhand_pose_raw_;
    master_rhand_pose_raw_pre_ = master_rhand_pose_raw_;
    master_head_pose_raw_pre_ = master_head_pose_raw_;
    master_lelbow_pose_raw_pre_ = master_lelbow_pose_raw_;
    master_relbow_pose_raw_pre_ = master_relbow_pose_raw_;
    master_lshoulder_pose_raw_pre_ = master_lshoulder_pose_raw_;
    master_rshoulder_pose_raw_pre_ = master_rshoulder_pose_raw_;
    master_upperbody_pose_raw_pre_ = master_upperbody_pose_raw_;

    master_lhand_pose_ppre_ = master_lhand_pose_pre_;
    master_rhand_pose_ppre_ = master_rhand_pose_pre_;
    master_head_pose_ppre_ = master_head_pose_pre_;
    master_lelbow_pose_ppre_ = master_lelbow_pose_pre_;
    master_relbow_pose_ppre_ = master_relbow_pose_pre_;
    master_lshoulder_pose_ppre_ = master_lshoulder_pose_pre_;
    master_rshoulder_pose_ppre_ = master_rshoulder_pose_pre_;
    master_upperbody_pose_ppre_ = master_upperbody_pose_pre_;

    master_lhand_pose_pre_ = master_lhand_pose_;
    master_rhand_pose_pre_ = master_rhand_pose_;
    master_lelbow_pose_pre_ = master_lelbow_pose_;
    master_relbow_pose_pre_ = master_relbow_pose_;
    master_lshoulder_pose_pre_ = master_lshoulder_pose_;
    master_rshoulder_pose_pre_ = master_rshoulder_pose_;
    master_head_pose_pre_ = master_head_pose_;
    master_upperbody_pose_pre_ = master_upperbody_pose_;

    master_relative_lhand_pos_pre_ = master_relative_lhand_pos_;
    master_relative_rhand_pos_pre_ = master_relative_rhand_pos_;

    hmd_tracker_status_pre_ = hmd_tracker_status_;

    hmd_head_pose_pre_ = hmd_head_pose_;
    hmd_lshoulder_pose_pre_ = hmd_lshoulder_pose_;
    hmd_lupperarm_pose_pre_ = hmd_lupperarm_pose_;
    hmd_lhand_pose_pre_ = hmd_lhand_pose_;
    hmd_rshoulder_pose_pre_ = hmd_rshoulder_pose_;
    hmd_rupperarm_pose_pre_ = hmd_rupperarm_pose_;
    hmd_rhand_pose_pre_ = hmd_rhand_pose_;
    hmd_chest_pose_pre_ = hmd_chest_pose_;
    hmd_pelv_pose_pre_ = hmd_pelv_pose_;

    lhand_mapping_vector_pre_ = lhand_mapping_vector_;
    rhand_mapping_vector_pre_ = rhand_mapping_vector_;

    avatar_op_pedal_pre_ = avatar_op_pedal_;
}

void AvatarController::UpperbodyModeCallback(const std_msgs::Int8 &msg)
{
    // avatarUpperbodyModeUpdate(msg.data);
    upper_body_mode_raw_ = msg.data;
    upperbody_mode_recieved_ = true;
}


void AvatarController::ArmJointGainCallback(const std_msgs::Float32MultiArray &msg)
{
    // left arm kp
    kp_joint_(15) = msg.data[0];
    kp_joint_(16) = msg.data[1];
    kp_joint_(17) = msg.data[2];
    kp_joint_(18) = msg.data[3];
    kp_joint_(19) = msg.data[4];
    kp_joint_(20) = msg.data[5];
    kp_joint_(21) = msg.data[6];
    kp_joint_(22) = msg.data[7];
    // right arm kp
    kp_joint_(25) = msg.data[0];
    kp_joint_(26) = msg.data[1];
    kp_joint_(27) = msg.data[2];
    kp_joint_(28) = msg.data[3];
    kp_joint_(29) = msg.data[4];
    kp_joint_(30) = msg.data[5];
    kp_joint_(31) = msg.data[6];
    kp_joint_(32) = msg.data[7];

    // left arm kd
    kv_joint_(15) = msg.data[8];
    kv_joint_(16) = msg.data[9];
    kv_joint_(17) = msg.data[10];
    kv_joint_(18) = msg.data[11];
    kv_joint_(19) = msg.data[12];
    kv_joint_(20) = msg.data[13];
    kv_joint_(21) = msg.data[14];
    kv_joint_(22) = msg.data[15];
    // right arm kd
    kv_joint_(25) = msg.data[8];
    kv_joint_(26) = msg.data[9];
    kv_joint_(27) = msg.data[10];
    kv_joint_(28) = msg.data[11];
    kv_joint_(29) = msg.data[12];
    kv_joint_(30) = msg.data[13];
    kv_joint_(31) = msg.data[14];
    kv_joint_(32) = msg.data[15];
}

void AvatarController::WaistJointGainCallback(const std_msgs::Float32MultiArray &msg)
{
    kp_joint_(12) = msg.data[0];
    kp_joint_(13) = msg.data[1];
    kp_joint_(14) = msg.data[2];

    kv_joint_(12) = msg.data[3];
    kv_joint_(13) = msg.data[4];
    kv_joint_(14) = msg.data[5];
}

// void AvatarController::LeftControllerCallback(const tocabi_msgs::matrix_3_4 &msg)
// {
// 	master_lhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
// 	master_lhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
// 	master_lhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

// 	master_lhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
// 	master_lhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
// 	master_lhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

// 	master_lhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
// 	master_lhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
// 	master_lhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

// 	master_lhand_pose_raw_.translation()(0) = msg.firstRow[3];
// 	master_lhand_pose_raw_.translation()(1) = msg.secondRow[3];
// 	master_lhand_pose_raw_.translation()(2) = msg.thirdRow[3];
// }

// void AvatarController::RightControllerCallback(const tocabi_msgs::matrix_3_4 &msg)
// {
// 	master_rhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
// 	master_rhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
// 	master_rhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

// 	master_rhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
// 	master_rhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
// 	master_rhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

// 	master_rhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
// 	master_rhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
// 	master_rhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

// 	master_rhand_pose_raw_.translation()(0) = msg.firstRow[3];
// 	master_rhand_pose_raw_.translation()(1) = msg.secondRow[3];
// 	master_rhand_pose_raw_.translation()(2) = msg.thirdRow[3];
// }

void AvatarController::HmdCallback(const tocabi_msgs::matrix_3_4 &msg)
{
    hmd_head_pose_raw_.linear()(0, 0) = msg.firstRow[0];
    hmd_head_pose_raw_.linear()(0, 1) = msg.firstRow[1];
    hmd_head_pose_raw_.linear()(0, 2) = msg.firstRow[2];

    hmd_head_pose_raw_.linear()(1, 0) = msg.secondRow[0];
    hmd_head_pose_raw_.linear()(1, 1) = msg.secondRow[1];
    hmd_head_pose_raw_.linear()(1, 2) = msg.secondRow[2];

    hmd_head_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
    hmd_head_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
    hmd_head_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

    hmd_head_pose_raw_.translation()(0) = msg.firstRow[3];
    hmd_head_pose_raw_.translation()(1) = msg.secondRow[3];
    hmd_head_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void AvatarController::PoseCalibrationCallback(const std_msgs::Int8 &msg)
{
    if (msg.data == 1) // still pose
    {
        hmd_check_pose_calibration_[0] = true;
        cout << "Still Pose Calibration is On." << endl;
    }
    else if (msg.data == 2) // T pose
    {
        hmd_check_pose_calibration_[1] = true;
        cout << "T Pose Calibration is On." << endl;
    }
    else if (msg.data == 3) // forward stretch
    {
        hmd_check_pose_calibration_[2] = true;
        cout << "Forward Stretch Pose Calibration is On." << endl;
    }
    else if (msg.data == 4) // reset callibration
    {
        for (int i = 0; i < 5; i++)
        {
            hmd_check_pose_calibration_[i] = false;
        }
        still_pose_cali_flag_ = false;
        t_pose_cali_flag_ = false;
        forward_pose_cali_flag_ = false;
        read_cali_log_flag_ = false;
        hmd_check_pose_calibration_[3] = false;
        cout << "Pose Calibration is Reset." << endl;

        std_msgs::String msg;
        std::stringstream reset;
        reset << "RESET POSE CALIBRATION";
        msg.data = reset.str();
        calibration_state_pub.publish(msg);
        calibration_state_gui_log_pub.publish(msg);
    }
    else if (msg.data == 5)
    {
        hmd_check_pose_calibration_[0] = true;
        hmd_check_pose_calibration_[1] = true;
        hmd_check_pose_calibration_[2] = true;
        hmd_check_pose_calibration_[4] = true;

        still_pose_cali_flag_ = true;
        t_pose_cali_flag_ = true;
        forward_pose_cali_flag_ = true;
        cout << "Reading Calibration Log File..." << endl;

        std_msgs::String msg;
        std::stringstream log_load;
        log_load << "Calibration Pose Data is Loaded";
        msg.data = log_load.str();
        calibration_state_pub.publish(msg);
        calibration_state_gui_log_pub.publish(msg);
    }

    cout << "Calibration Status: [" << hmd_check_pose_calibration_[0] << ", " << hmd_check_pose_calibration_[1] << ", " << hmd_check_pose_calibration_[2] << "]" << endl;
}

void AvatarController::LeftHandTrackerCallback(const tocabi_msgs::matrix_3_4 &msg)
{
    hmd_lhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
    hmd_lhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
    hmd_lhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

    hmd_lhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
    hmd_lhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
    hmd_lhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

    hmd_lhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
    hmd_lhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
    hmd_lhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

    hmd_lhand_pose_raw_.translation()(0) = msg.firstRow[3];
    hmd_lhand_pose_raw_.translation()(1) = msg.secondRow[3];
    hmd_lhand_pose_raw_.translation()(2) = msg.thirdRow[3];

    // std::cout<<"left hand callback test"<<std::endl;
}

void AvatarController::RightHandTrackerCallback(const tocabi_msgs::matrix_3_4 &msg)
{
    hmd_rhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
    hmd_rhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
    hmd_rhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

    hmd_rhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
    hmd_rhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
    hmd_rhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

    hmd_rhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
    hmd_rhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
    hmd_rhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

    hmd_rhand_pose_raw_.translation()(0) = msg.firstRow[3];
    hmd_rhand_pose_raw_.translation()(1) = msg.secondRow[3];
    hmd_rhand_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void AvatarController::LeftElbowTrackerCallback(const tocabi_msgs::matrix_3_4 &msg)
{
    hmd_lupperarm_pose_raw_.linear()(0, 0) = msg.firstRow[0];
    hmd_lupperarm_pose_raw_.linear()(0, 1) = msg.firstRow[1];
    hmd_lupperarm_pose_raw_.linear()(0, 2) = msg.firstRow[2];

    hmd_lupperarm_pose_raw_.linear()(1, 0) = msg.secondRow[0];
    hmd_lupperarm_pose_raw_.linear()(1, 1) = msg.secondRow[1];
    hmd_lupperarm_pose_raw_.linear()(1, 2) = msg.secondRow[2];

    hmd_lupperarm_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
    hmd_lupperarm_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
    hmd_lupperarm_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

    hmd_lupperarm_pose_raw_.translation()(0) = msg.firstRow[3];
    hmd_lupperarm_pose_raw_.translation()(1) = msg.secondRow[3];
    hmd_lupperarm_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void AvatarController::RightElbowTrackerCallback(const tocabi_msgs::matrix_3_4 &msg)
{
    hmd_rupperarm_pose_raw_.linear()(0, 0) = msg.firstRow[0];
    hmd_rupperarm_pose_raw_.linear()(0, 1) = msg.firstRow[1];
    hmd_rupperarm_pose_raw_.linear()(0, 2) = msg.firstRow[2];

    hmd_rupperarm_pose_raw_.linear()(1, 0) = msg.secondRow[0];
    hmd_rupperarm_pose_raw_.linear()(1, 1) = msg.secondRow[1];
    hmd_rupperarm_pose_raw_.linear()(1, 2) = msg.secondRow[2];

    hmd_rupperarm_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
    hmd_rupperarm_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
    hmd_rupperarm_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

    hmd_rupperarm_pose_raw_.translation()(0) = msg.firstRow[3];
    hmd_rupperarm_pose_raw_.translation()(1) = msg.secondRow[3];
    hmd_rupperarm_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void AvatarController::ChestTrackerCallback(const tocabi_msgs::matrix_3_4 &msg)
{
    hmd_chest_pose_raw_.linear()(0, 0) = msg.firstRow[0];
    hmd_chest_pose_raw_.linear()(0, 1) = msg.firstRow[1];
    hmd_chest_pose_raw_.linear()(0, 2) = msg.firstRow[2];

    hmd_chest_pose_raw_.linear()(1, 0) = msg.secondRow[0];
    hmd_chest_pose_raw_.linear()(1, 1) = msg.secondRow[1];
    hmd_chest_pose_raw_.linear()(1, 2) = msg.secondRow[2];

    hmd_chest_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
    hmd_chest_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
    hmd_chest_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

    hmd_chest_pose_raw_.translation()(0) = msg.firstRow[3];
    hmd_chest_pose_raw_.translation()(1) = msg.secondRow[3];
    hmd_chest_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void AvatarController::PelvisTrackerCallback(const tocabi_msgs::matrix_3_4 &msg)
{
    hmd_pelv_pose_raw_.linear()(0, 0) = msg.firstRow[0];
    hmd_pelv_pose_raw_.linear()(0, 1) = msg.firstRow[1];
    hmd_pelv_pose_raw_.linear()(0, 2) = msg.firstRow[2];

    hmd_pelv_pose_raw_.linear()(1, 0) = msg.secondRow[0];
    hmd_pelv_pose_raw_.linear()(1, 1) = msg.secondRow[1];
    hmd_pelv_pose_raw_.linear()(1, 2) = msg.secondRow[2];

    hmd_pelv_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
    hmd_pelv_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
    hmd_pelv_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

    hmd_pelv_pose_raw_.translation()(0) = msg.firstRow[3];
    hmd_pelv_pose_raw_.translation()(1) = msg.secondRow[3];
    hmd_pelv_pose_raw_.translation()(2) = msg.thirdRow[3];

    hmd_pelv_pose_raw_.linear() = hmd_pelv_pose_raw_.linear() * DyrosMath::rotateWithZ(M_PI); // tracker is behind the chair
}

void AvatarController::TrackerStatusCallback(const std_msgs::Bool &msg)
{
    hmd_tracker_status_raw_ = msg.data;
}

void AvatarController::TrackerPoseCallback(const geometry_msgs::PoseArray &msg)
{
    // msg.poses[0];

    if(master_arm_mode_ == true)
    {
        tf::poseMsgToEigen(msg.poses[0],hmd_pelv_pose_raw_);
    
        hmd_pelv_pose_raw_.linear() = hmd_pelv_pose_raw_.linear() * DyrosMath::rotateWithZ(M_PI); // tracker is behind the chair

        tf::poseMsgToEigen(msg.poses[1],hmd_chest_pose_raw_);
        tf::poseMsgToEigen(msg.poses[2],hmd_lupperarm_pose_raw_);
        tf::poseMsgToEigen(msg.poses[3],hmd_rupperarm_pose_raw_);
        tf::poseMsgToEigen(msg.poses[4],hmd_head_pose_raw_);
    }
    else
    {
        tf::poseMsgToEigen(msg.poses[0],hmd_pelv_pose_raw_);
    
        hmd_pelv_pose_raw_.linear() = hmd_pelv_pose_raw_.linear() * DyrosMath::rotateWithZ(M_PI); // tracker is behind the chair

        tf::poseMsgToEigen(msg.poses[1],hmd_chest_pose_raw_);
        tf::poseMsgToEigen(msg.poses[2],hmd_lupperarm_pose_raw_);
        tf::poseMsgToEigen(msg.poses[3],hmd_lhand_pose_raw_);
        tf::poseMsgToEigen(msg.poses[4],hmd_rupperarm_pose_raw_);
        tf::poseMsgToEigen(msg.poses[5],hmd_rhand_pose_raw_);
        tf::poseMsgToEigen(msg.poses[6],hmd_head_pose_raw_);       
    }
}
void AvatarController::MasterPoseCallback(const geometry_msgs::PoseArray &msg)
{
    if(master_arm_mode_ == true)
    {
        tf::poseMsgToEigen(msg.poses[0], hmd_lhand_pose_raw_);
        tf::poseMsgToEigen(msg.poses[1], hmd_rhand_pose_raw_);
    }
}
void AvatarController::HandPosMappingScaleCallback(const std_msgs::Float32 &msg)
{
    hand_pos_mapping_scale_raw_ = msg.data;
}
double AvatarController::bandBlock(double value, double max, double min)
{
    double y;

    if (value <= min)
    {
        y = -(value - min) * (value - min);
    }
    else if (value >= max)
    {
        y = (value - max) * (value - max);
    }
    else
    {
        y = 0;
    }

    return y;
}

void AvatarController::printOutTextFile()
{
    if (printout_cnt_ % 20 == 0)
    // if(true)
    {
        // if (printout_cnt_ <= 100 * 60 * 60 * 1) // 1h
        if (true)
        {
            file[0] << rd_.control_time_ << "\t" ;
            // <<rd_.torque_desired (0)<<"\t"<<rd_.torque_desired (1)<<"\t"<<rd_.torque_desired (2)<<"\t"<<rd_.torque_desired (3)<<"\t"<<rd_.torque_desired (4)<<"\t"<<rd_.torque_desired (5)<<"\t"<<rd_.torque_desired (6)<<"\t"<<rd_.torque_desired (7)<<"\t"<<rd_.torque_desired (8)<<"\t"<<rd_.torque_desired (9)<<"\t"<<rd_.torque_desired (10)<<"\t"<<rd_.torque_desired (11)<<"\t"<<rd_.torque_desired (12)<<"\t"<<rd_.torque_desired (13)<<"\t"<<rd_.torque_desired (14)<<"\t"<<rd_.torque_desired (15)<<"\t"<<rd_.torque_desired (16)<<"\t"<<rd_.torque_desired (17)<<"\t"<<rd_.torque_desired (18)<<"\t"<<rd_.torque_desired (19)<<"\t"<<rd_.torque_desired (20)<<"\t"<<rd_.torque_desired (21)<<"\t"<<rd_.torque_desired (22)<<"\t"<<rd_.torque_desired (23)<<"\t"<<rd_.torque_desired (24)<<"\t"<<rd_.torque_desired (25)<<"\t"<<rd_.torque_desired (26)<<"\t"<<rd_.torque_desired (27)<<"\t"<<rd_.torque_desired (28)<<"\t"<<rd_.torque_desired (29)<<"\t"<<rd_.torque_desired (30)<<"\t"<<rd_.torque_desired (31)<<"\t"<<rd_.torque_desired (32)<<endl;
            // file[4] << torque_grav_(0) << "\t" << torque_grav_(1) << "\t" << torque_grav_(2) << "\t" << torque_grav_(3) << "\t" << torque_grav_(4) << "\t" << torque_grav_(5) << "\t" << torque_grav_(6) << "\t" << torque_grav_(7) << "\t" << torque_grav_(8) << "\t" << torque_grav_(9) << "\t" << torque_grav_(10) << "\t" << torque_grav_(11) << endl;
            for (int i = 0; i < 3; i++)            //pelv rot 1st column
            {
                file[0] << rd_.link_[Pelvis].rotm(i, 0) << "\t";
            }
            for (int i = 0; i < 3; i++) //pelv rot 2nd column
            {
                file[0] << rd_.link_[Pelvis].rotm(i, 1) << "\t";
            }
            for (int i = 18; i < 39; i++) //q
            {
                file[0] << rd_.q_virtual_(i) << "\t";
            }
            for (int i = 18; i < 39; i++) //qdot
            {
                file[0] << rd_.q_dot_virtual_(i) << "\t";
            }
            for (int i = 18; i < 39; i++) //qdot pre
            {
                file[0] << q_dot_virtual_Xd_global_pre_(i) << "\t";
            }
            for (int i = 3; i < 6; i++) //ang vel
            {
                file[0] << rd_.q_dot_virtual_(i) << "\t";
            }
            for (int i = 0; i < 3; i++) // lin acc
            {
                file[0] << rd_.q_ddot_virtual_(i) << "\t";
            }
            for (int i = 12; i < 33; i++)
            {
                file[0] << desired_q_slow_(i) << "\t";
            }
            for (int i = 12; i < 33; i++)
            {
                file[0] << rd_.torque_desired(i) << "\t";
            }
            for (int i = 18; i < 39; i++) // left + right leg
            {
                file[0] << mob_residual_wholebody_(i) << "\t";
            }
            for (int i = 12; i < 33; i++)
            {
                file[0] << torque_current_elmo_(i) << "\t";
            }
            for (int i = 0; i < 6; i++)
            {
                file[0] << lh_ft_wo_hw_global_lpf_(i) << "\t";
            }
            for (int i = 0; i < 6; i++)
            {
                file[0] << rh_ft_wo_hw_global_lpf_(i) << "\t";
            }
            for (int i = 12; i < 15; i++)
            {
                file[0] << torque_from_lh_ft_(i) << "\t";
            }
            for (int i = 15; i < 23; i++)
            {
                file[0] << torque_from_lh_ft_(i) << "\t";
            }
            for (int i = 12; i < 15; i++)
            {
                file[0] << torque_from_rh_ft_(6 + i) << "\t";
            }
            for (int i = 25; i < 33; i++)
            {
                file[0] << torque_from_rh_ft_(6 + i) << "\t";
            }
            for (int i = 12; i < 15; i++)
            {
                file[0] << torque_from_lh_ft_lpf_(i) << "\t";
            }
            for (int i = 15; i < 23; i++)
            {
                file[0] << torque_from_lh_ft_lpf_(i) << "\t";
            }
            for (int i = 12; i < 15; i++)
            {
                file[0] << torque_from_rh_ft_lpf_(6 + i) << "\t";
            }
            for (int i = 25; i < 33; i++)
            {
                file[0] << torque_from_rh_ft_lpf_(6 + i) << "\t";
            }
            file[0] << endl;

            for (int i = 0; i < 6; i++)
            {
                file[1] << lh_ft_(i) << "\t";
            }
            for (int i = 0; i < 6; i++)
            {
                file[1] << rh_ft_(i) << "\t";
            }
            file[1] << endl;
        }
        else
        {
            // cout << "WARNING: Logging for 1h" << endl;
        }
    }

    printout_cnt_ += 1;
}

//////////////////////////////MJ's Functions////////////////////
void AvatarController::PedalCommandCallback(const tocabi_msgs::WalkingCommandConstPtr &msg)
{
    if (joy_input_enable_ == true)
    {
        joystick_input(0) = DyrosMath::minmax_cut(2 * (msg->step_length_x), 0.0, 2.0) - 1.0; // FW
        joystick_input(2) = DyrosMath::minmax_cut(2 * (msg->theta) - DyrosMath::sign(msg->theta), -0.5 + 0.5 * DyrosMath::sign(msg->theta), 0.5 + 0.5 * DyrosMath::sign(msg->theta));
        // joystick_input(2) = msg->theta;
        joystick_input(3) = DyrosMath::minmax_cut(2 * (msg->z), 0.0, 2.0) - 1.0; // BW
        joystick_input(1) = (joystick_input(0) + 1) / 2 + abs(joystick_input(2)) + (joystick_input(3) + 1) / 2;
    }
    else
    {
        joystick_input(1) = -1.0;
    }

    if (joystick_input(1) > 0.0001)
    {
        walking_enable_ = true;
        walking_end_flag = 1;
        // cout<<"walking triggered!!"<<endl;
    }
}

void AvatarController::updateInitialState()
{
    if (walking_tick_mj == 0)
    {
        // calculateFootStepTotal();
        calculateFootStepTotal_MJ();

        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); // ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        // pelv_float_init_.translation()(0) += 0.11;

        pelv_float_init_.translation()(0) = 0;
        pelv_float_init_.translation()(1) = 0;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        lfoot_float_init_.translation()(0) = 0;
        lfoot_float_init_.translation()(1) = 0.1225;

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        rfoot_float_init_.translation()(0) = 0;
        rfoot_float_init_.translation()(1) = -0.1225;

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        com_float_init_(0) = 0;
        com_float_init_(1) = 0;

        if (aa == 0)
        {
            lfoot_float_init_.translation()(1) = 0.1025;
            rfoot_float_init_.translation()(1) = -0.1025;
            aa = 1;
        }
        cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;

        Eigen::Isometry3d ref_frame;

        if (foot_step_(0, 6) == 0) // right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(0, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        supportfoot_float_init_.setZero();
        swingfoot_float_init_.setZero();

        if (foot_step_(0, 6) == 1) // left suppport foot
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }
        else
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }

        pelv_support_start_ = pelv_support_init_;
        // cout<<"pelv_support_start_.translation()(2): "<<pelv_support_start_.translation()(2);
        total_step_num_ = foot_step_.col(1).size();

        xi_mj_ = com_support_init_(0); // preview parameter
        yi_mj_ = com_support_init_(1);
        zc_mj_ = com_support_init_(2);
    }
    else if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {
        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); // ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        rfoot_rpy_current_.setZero();
        lfoot_rpy_current_.setZero();
        rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
        lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

        rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
        lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
        rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
        lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        // lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        // rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        Eigen::Isometry3d ref_frame;

        if (foot_step_(current_step_num_, 6) == 0) // right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        //////dg edit
        Eigen::Isometry3d ref_frame_yaw_only;
        ref_frame_yaw_only.translation() = ref_frame.translation();
        Eigen::Vector3d ref_frame_rpy;
        ref_frame_rpy = DyrosMath::rot2Euler(ref_frame.linear());
        ref_frame_yaw_only.linear() = DyrosMath::rotateWithZ(ref_frame_rpy(2));

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame_yaw_only) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), com_float_init_);
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), rfoot_float_init_);
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
        ///////////////

        // pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        // com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);
        // pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        // lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        // rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        // rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        // lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
    }
}

void AvatarController::getRobotState()
{
    //GET DATA FROM rd_ START
    // static VectorVQd __q_dot_virtual;
    // static VectorQVQd __q_virtual;
    // static VectorVQd __q_ddot_virtual;
    // static LinkData link_avatar_[LINK_NUMBER + 1];

    __q_dot_virtual = rd_.q_dot_virtual_;
    __q_virtual = rd_.q_virtual_;
    __q_ddot_virtual = rd_.q_ddot_virtual_;
    for (int i=0;i<LINK_NUMBER + 1; i++)
    {
        link_avatar_[i] = rd_.link_[i];
    }
    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    lh_ft_ = rd_.LH_FT;
    rh_ft_ = rd_.RH_FT;
    // elmo current torque
    for (int i = 0; i < MODEL_DOF; i++)
    {
        torque_current_elmo_(i) = rd_.torque_elmo_(i); // motor current torque
    }
    //GET DATA FROM rd_ END


    //////////// model_global_ UPDATE///////////////
    // Base frame is global
    if (walking_tick_mj == 0)
        q_dot_virtual_Xd_global_pre_ = __q_dot_virtual;
    q_dot_virtual_Xd_global_ = __q_dot_virtual;


    q_virtual_Xd_global_ = __q_virtual;
    q_ddot_virtual_Xd_global_ = (q_dot_virtual_Xd_global_ - q_dot_virtual_Xd_global_pre_) * hz_;
    q_ddot_virtual_Xd_global_.segment(0, 6) = __q_ddot_virtual.segment(0, 6);

    RigidBodyDynamics::UpdateKinematicsCustom(model_global_, &q_virtual_Xd_global_, &q_dot_virtual_Xd_global_, &q_ddot_virtual_Xd_global_);
    //////////////////////////////////////////////

    pelv_rpy_current_mj_.setZero();
    pelv_rpy_current_mj_ = DyrosMath::rot2Euler(link_avatar_[Pelvis].rotm); // ZYX multiply

    R_angle = pelv_rpy_current_mj_(0);
    P_angle = pelv_rpy_current_mj_(1);
    pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(link_avatar_[Right_Foot].rotm);
    lfoot_rpy_current_ = DyrosMath::rot2Euler(link_avatar_[Left_Foot].rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

    pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * link_avatar_[Pelvis].rotm;

    pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), link_avatar_[Pelvis].xpos);

    lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * link_avatar_[Left_Foot].rotm;
    // lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * link_avatar_[Left_Foot].rotm;
    lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), link_avatar_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

    rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * link_avatar_[Right_Foot].rotm;
    // rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * link_avatar_[Right_Foot].rotm;
    rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), link_avatar_[Right_Foot].xpos); // 지면에서 Ankle frame

    com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), link_avatar_[COM_id].xpos); // 지면에서 CoM 위치
    com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), link_avatar_[COM_id].v);

    if (walking_tick_mj == 0)
    {
        com_float_current_dot_LPF = com_float_current_dot;
        com_float_current_dot_prev = com_float_current_dot;
    }

    com_float_current_dot_prev = com_float_current_dot;
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * com_float_current_dot;

    if (walking_tick_mj == 0)
    {
        com_float_current_LPF = com_float_current_;
    }

    com_float_current_LPF = 1 / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_LPF + (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_;

    double support_foot_flag = foot_step_(current_step_num_, 6);
    if (support_foot_flag == 0)
    {
        supportfoot_float_current_ = rfoot_float_current_;
    }
    else if (support_foot_flag == 1)
    {
        supportfoot_float_current_ = lfoot_float_current_;
    }


    ///////////dg edit
    Eigen::Isometry3d supportfoot_float_current_yaw_only;
    supportfoot_float_current_yaw_only.translation() = supportfoot_float_current_.translation();
    Eigen::Vector3d support_foot_current_rpy;
    support_foot_current_rpy = DyrosMath::rot2Euler(supportfoot_float_current_.linear());
    supportfoot_float_current_yaw_only.linear() = DyrosMath::rotateWithZ(support_foot_current_rpy(2));

    pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * pelv_float_current_;
    lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * lfoot_float_current_;
    rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * rfoot_float_current_;
    ////////////////////

    // pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * pelv_float_current_;
    // lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * lfoot_float_current_;
    // rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * rfoot_float_current_;

    // cout << "L : " << lfoot_float_current_.linear() << "\n" <<  "R : " << rfoot_float_current_.linear() << endl;
    com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);
    


    if(real_robot_mode_ == true)
    {
        // FT local frame rotation
        lh_ft_(0) *= -1;
        lh_ft_(2) *= -1;
        lh_ft_(3) *= -1;
        lh_ft_(5) *= -1;

        rh_ft_(1) *= -1;
        rh_ft_(2) *= -1;
        rh_ft_(4) *= -1;
        rh_ft_(5) *= -1;
    }

    if (walking_tick_mj == 0)
    {
        l_ft_LPF = l_ft_;
        r_ft_LPF = r_ft_;
    }

    l_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_;
    r_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_;


    double tocabi_lhand_mass = 0.8;
    Vector6d wrench_lhand;
    wrench_lhand.setZero();
    wrench_lhand(2) = -tocabi_lhand_mass*9.81;
    Vector3d lh_com(0.0, 0.0, -0.1542);
    Vector3d lh_ft_point(0.0, 0.0, -0.1028);
    Vector3d lh_com2cp = lh_ft_point - lh_com; 
    Matrix6d adt_lh;
    adt_lh.setIdentity();
    adt_lh.block(3, 0, 3, 3) = DyrosMath::skm(-lh_com2cp) * Matrix3d::Identity();
    Matrix6d rotlh;
    rotlh.setZero();
    rotlh.block(0, 0, 3, 3) = pelv_yaw_rot_current_from_global_mj_.linear().transpose()*link_avatar_[Left_Hand].rotm;
    rotlh.block(3, 3, 3, 3) = pelv_yaw_rot_current_from_global_mj_.linear().transpose()*link_avatar_[Left_Hand].rotm;
    
    lh_ft_wo_hw_ = lh_ft_ + adt_lh * (rotlh.transpose() * wrench_lhand);
    lh_ft_wo_hw_lpf_ = DyrosMath::lpf<6>(lh_ft_wo_hw_, lh_ft_wo_hw_lpf_, 2000, 100 / (2 * M_PI));

    lh_ft_wo_hw_global_ = rotlh * lh_ft_wo_hw_;
    lh_ft_wo_hw_global_lpf_ = DyrosMath::lpf<6>(lh_ft_wo_hw_global_, lh_ft_wo_hw_global_lpf_, 2000, 10.0);

    double tocabi_rhand_mass = 0.8;
    Vector6d wrench_rhand;
    wrench_rhand.setZero();
    wrench_rhand(2) = -tocabi_rhand_mass*9.81;
    Vector3d rh_com(0.0, 0.0, -0.1542);
    Vector3d rh_ft_point(0.0, 0.0, -0.1028);
    Vector3d rh_com2cp = rh_ft_point - rh_com; 
    Matrix6d adt_rh;
    adt_rh.setIdentity();
    adt_rh.block(3, 0, 3, 3) = DyrosMath::skm(-rh_com2cp) * Matrix3d::Identity();
    Matrix6d rotrh;
    rotrh.setZero();
    rotrh.block(0, 0, 3, 3) = pelv_yaw_rot_current_from_global_mj_.linear().transpose()*link_avatar_[Right_Hand].rotm;
    rotrh.block(3, 3, 3, 3) = pelv_yaw_rot_current_from_global_mj_.linear().transpose()*link_avatar_[Right_Hand].rotm;

    rh_ft_wo_hw_ = rh_ft_ + adt_rh * (rotrh.transpose() * wrench_rhand);
    rh_ft_wo_hw_lpf_ = DyrosMath::lpf<6>(rh_ft_wo_hw_, rh_ft_wo_hw_lpf_, 2000, 100 / (2 * M_PI));

    rh_ft_wo_hw_global_ = rotrh * rh_ft_wo_hw_;
    rh_ft_wo_hw_global_lpf_ = DyrosMath::lpf<6>(rh_ft_wo_hw_global_, rh_ft_wo_hw_global_lpf_, 2000, 10.0);
    

    Eigen::MatrixXd J_temp, R_lh, R_rh;
    J_temp.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_global_, q_virtual_Xd_global_, link_avatar_[Left_Foot].id, lfoot_ft_sensor_offset_, J_temp, false);
    jac_lfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL); // position
    jac_lfoot_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL); // orientation
    J_temp.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_global_, q_virtual_Xd_global_, link_avatar_[Right_Foot].id, rfoot_ft_sensor_offset_, J_temp, false);
    jac_rfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL); // position
    jac_rfoot_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL); // orientation

    J_temp.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_global_, q_virtual_Xd_global_, link_avatar_[Left_Hand].id, lhand_ft_sensor_offset_, J_temp, false);
    jac_lhand_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL); // position
    jac_lhand_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL); // orientation

    J_temp.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_global_, q_virtual_Xd_global_, link_avatar_[Right_Hand].id, rhand_ft_sensor_offset_, J_temp, false);
    jac_rhand_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(3, 0, 3, MODEL_DOF_VIRTUAL); // position
    jac_rhand_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = J_temp.block(0, 0, 3, MODEL_DOF_VIRTUAL); // orientation
    
    R_lh.setZero(6, 6);
    R_lh.block(0, 0, 3, 3) = link_avatar_[Left_Hand].rotm;
    R_lh.block(3, 3, 3, 3) = link_avatar_[Left_Hand].rotm;

    torque_from_lh_ft_ = jac_lhand_.block(0, 6, 6, MODEL_DOF).transpose() * R_lh * (-lh_ft_wo_hw_);
    torque_from_lh_ft_lpf_ = DyrosMath::lpf<33>(torque_from_lh_ft_, torque_from_lh_ft_lpf_, 2000, 100 / (2 * M_PI));

    R_rh.setZero(6, 6);
    R_rh.block(0, 0, 3, 3) = link_avatar_[Right_Hand].rotm;
    R_rh.block(3, 3, 3, 3) = link_avatar_[Right_Hand].rotm;

    torque_from_rh_ft_ = jac_rhand_.block(0, 6, 6, MODEL_DOF).transpose() * R_rh * (-rh_ft_wo_hw_);
    torque_from_rh_ft_lpf_ = DyrosMath::lpf<33>(torque_from_rh_ft_, torque_from_rh_ft_lpf_, 2000, 100 / (2 * M_PI));


    // MJ_graph << l_ft_LPF(2) << "," << l_ft_LPF(3) << "," << l_ft_LPF(4) << "," << r_ft_LPF(2) << "," << r_ft_LPF(3) << "," << r_ft_LPF(4) << endl;
    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);

    zmp_measured_mj_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP X
    zmp_measured_mj_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y

    wn = sqrt(GRAVITY / zc_mj_);

    if (walking_tick_mj == 0)
    {
        zmp_measured_LPF_.setZero();
    }

    zmp_measured_LPF_ = (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 8.0 * del_t) * zmp_measured_LPF_;

    floatingBaseMOB();

    ////////// prepare sca mlp input vectors ////////////
    calculateScaMlpInput(larm_upperbody_sca_mlp_);
    calculateScaMlpInput(rarm_upperbody_sca_mlp_);
    // calculateScaMlpInput(btw_arms_sca_mlp_);
    /////////////////////////////////////////////////////

}

void AvatarController::calculateFootStepTotal()
{
    double initial_rot = 0.0;
    double final_rot = 0.0;
    double initial_drot = 0.0;
    double final_drot = 0.0;

    initial_rot = atan2(target_y_, target_x_);

    if (initial_rot > 0.0)
        initial_drot = 5 * DEG2RAD;
    else
        initial_drot = -5 * DEG2RAD;

    unsigned int initial_total_step_number = initial_rot / initial_drot;
    double initial_residual_angle = initial_rot - initial_total_step_number * initial_drot;

    final_rot = target_theta_ - initial_rot;
    if (final_rot > 0.0)
        final_drot = 5 * DEG2RAD;
    else
        final_drot = -5 * DEG2RAD;

    unsigned int final_total_step_number = final_rot / final_drot;
    double final_residual_angle = final_rot - final_total_step_number * final_drot;
    double length_to_target = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    double dlength = step_length_x_;
    unsigned int middle_total_step_number = length_to_target / dlength;
    double middle_residual_length = length_to_target - middle_total_step_number * dlength;

    if (length_to_target == 0)
    {
        middle_total_step_number = 30; //
        dlength = 0;
    }

    unsigned int number_of_foot_step;

    int del_size;

    del_size = 1;
    number_of_foot_step = initial_total_step_number * del_size + middle_total_step_number * del_size + final_total_step_number * del_size;

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
    {
        if (initial_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(initial_residual_angle) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        if (middle_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(middle_residual_length) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        if (abs(final_residual_angle) >= 0.0001)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
            number_of_foot_step = number_of_foot_step + del_size;
    }

    foot_step_.resize(number_of_foot_step, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(number_of_foot_step, 7);
    foot_step_support_frame_.setZero();

    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing_ == true)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right;
    temp2 = -is_right;
    temp3 = -is_right;

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
    {
        for (int i = 0; i < initial_total_step_number; i++)
        {
            temp *= -1;
            foot_step_(index, 0) = temp * 0.1025 * sin((i + 1) * initial_drot);
            foot_step_(index, 1) = -temp * 0.1025 * cos((i + 1) * initial_drot);
            foot_step_(index, 5) = (i + 1) * initial_drot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }

        if (temp == is_right)
        {
            if (abs(initial_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
            else
            {
                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
        }
        else if (temp == -is_right)
        {
            temp *= -1;

            foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;

            temp *= -1;

            foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        for (int i = 0; i < middle_total_step_number; i++)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (i + 1)) + temp2 * sin(initial_rot) * (0.1025);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (i + 1)) - temp2 * cos(initial_rot) * (0.1025);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }

        if (temp2 == is_right)
        {
            if (abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;

                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
        }
        else if (temp2 == -is_right)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;

            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }
    }

    double final_position_x = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);
    double final_position_y = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < final_total_step_number; i++)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin((i + 1) * final_drot + initial_rot);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos((i + 1) * final_drot + initial_rot);
            foot_step_(index, 5) = (i + 1) * final_drot + initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }

        if (abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;

            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
    }
}

void AvatarController::calculateFootStepTotal_MJ()
{
    double initial_rot = 0.0;
    double final_rot = 0.0;
    double initial_drot = 0.0;
    double final_drot = 0.0;

    initial_rot = atan2(target_y_, target_x_);

    if (initial_rot > 0.0)
        initial_drot = 20 * DEG2RAD;
    else
        initial_drot = -20 * DEG2RAD;

    unsigned int initial_total_step_number = initial_rot / initial_drot;
    double initial_residual_angle = initial_rot - initial_total_step_number * initial_drot;

    final_rot = target_theta_ - initial_rot;
    if (final_rot > 0.0)
        final_drot = 20 * DEG2RAD;
    else
        final_drot = -20 * DEG2RAD;

    unsigned int final_total_step_number = final_rot / final_drot;
    double final_residual_angle = final_rot - final_total_step_number * final_drot;
    double length_to_target = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    double dlength = step_length_x_;
    unsigned int middle_total_step_number = length_to_target / dlength;
    double middle_residual_length = length_to_target - middle_total_step_number * dlength;

    double step_width_init;
    double step_width;

    step_width_init = 0.01;
    step_width = 0.02;

    if (length_to_target == 0.0)
    {
        middle_total_step_number = 4; // total foot step number
        dlength = 0;
    }

    unsigned int number_of_foot_step;

    int del_size;

    del_size = 1;
    number_of_foot_step = 2 + initial_total_step_number * del_size + middle_total_step_number * del_size + final_total_step_number * del_size;

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
    {
        if (initial_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(initial_residual_angle) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        if (middle_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(middle_residual_length) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        if (abs(final_residual_angle) >= 0.0001)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
            number_of_foot_step = number_of_foot_step + del_size;
    }

    foot_step_.resize(number_of_foot_step, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(number_of_foot_step, 7);
    foot_step_support_frame_.setZero();

    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing_ == true)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right;
    temp2 = -is_right;
    temp3 = -is_right;

    int temp0;
    temp0 = -is_right;

    double initial_dir = 0.0;

    if (aa == 0)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            if (i == 0)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width_init * (i + 1));
            }
            else if (i == 1)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width_init * (i + 1));
            }

            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }
    else if (aa == 1)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
    {
        for (int i = 0; i < initial_total_step_number; i++)
        {
            temp *= -1;
            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((i + 1) * initial_drot);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((i + 1) * initial_drot);
            foot_step_(index, 5) = (i + 1) * initial_drot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }

        if (temp == is_right)
        {
            if (abs(initial_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
            else
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
        }
        else if (temp == -is_right)
        {
            temp *= -1;

            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;

            temp *= -1;

            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001) // 직진, 제자리 보행
    {

        for (int i = 0; i < middle_total_step_number; i++)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (i + 1)) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (i + 1)) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }

        if (temp2 == is_right)
        {
            if (abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;

                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
        }
        else if (temp2 == -is_right)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;

            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }
    }

    double final_position_x = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);
    double final_position_y = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < final_total_step_number; i++)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin((i + 1) * final_drot + initial_rot);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos((i + 1) * final_drot + initial_rot);
            foot_step_(index, 5) = (i + 1) * final_drot + initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }

        if (abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;

            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
    }
}

void AvatarController::floatToSupportFootstep()
{
    Eigen::Isometry3d reference;

    if (current_step_num_ == 0)
    {
        if (foot_step_(0, 6) == 0)
        {
            reference.translation() = rfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
        else
        {
            reference.translation() = lfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_ - 1, 5));
        for (int i = 0; i < 3; i++)
        {
            reference.translation()(i) = foot_step_(current_step_num_ - 1, i);
        }
    }

    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    for (int i = 0; i < total_step_num_; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            temp_global_position(j) = foot_step_(i, j);
        }

        temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

        for (int j = 0; j < 3; j++)
        {
            foot_step_support_frame_(i, j) = temp_local_position(j);
        }

        foot_step_support_frame_(i, 3) = foot_step_(i, 3);
        foot_step_support_frame_(i, 4) = foot_step_(i, 4);
        if (current_step_num_ == 0)
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - supportfoot_float_init_(5);
        }
        else
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - foot_step_(current_step_num_ - 1, 5);
        }
    }

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = swingfoot_float_init_(j); // swingfoot_float_init_은 Pelvis에서 본 Swing 발의 Position, orientation.

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        swingfoot_support_init_(j) = temp_local_position(j);

    swingfoot_support_init_(3) = swingfoot_float_init_(3);
    swingfoot_support_init_(4) = swingfoot_float_init_(4);

    if (current_step_num_ == 0)
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
    else
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if (current_step_num_ == 0)
        supportfoot_support_init_(5) = 0;
    else
        supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);
}

void AvatarController::Joint_gain_set_MJ()
{
    // simulation gains
    //  Kp(0) = 1800.0; Kd(0) = 70.0; // Left Hip yaw
    //  Kp(1) = 2100.0; Kd(1) = 90.0;// Left Hip roll
    //  Kp(2) = 2100.0; Kd(2) = 90.0;// Left Hip pitch
    //  Kp(3) = 2100.0; Kd(3) = 90.0;// Left Knee pitch
    //  Kp(4) = 900.0; Kd(4) = 40.0;// Left Ankle pitch
    //  Kp(5) = 900.0; Kd(5) = 40.0;// Left Ankle roll

    // Kp(6) = 1800.0; Kd(6) = 70.0;// Right Hip yaw
    // Kp(7) = 2100.0; Kd(7) = 90.0;// Right Hip roll
    // Kp(8) = 2100.0; Kd(8) = 90.0;// Right Hip pitch
    // Kp(9) = 2100.0; Kd(9) = 90.0;// Right Knee pitch
    // Kp(10) = 900.0; Kd(10) = 40.0;// Right Ankle pitch
    // Kp(11) = 900.0; Kd(11) = 40.0;// Right Ankle roll

    // Kp(12) = 2200.0; Kd(12) = 90.0;// Waist yaw
    // Kp(13) = 2200.0; Kd(13) = 90.0;// Waist pitch
    // Kp(14) = 2200.0; Kd(14) = 90.0;// Waist roll

    // Kp(15) = 400.0; Kd(15) = 10.0;
    // Kp(16) = 800.0; Kd(16) = 10.0;
    // Kp(17) = 400.0; Kd(17) = 10.0;
    // Kp(18) = 400.0; Kd(18) = 10.0;
    // Kp(19) = 250.0; Kd(19) = 2.5;
    // Kp(20) = 250.0; Kd(20) = 2.0;
    // Kp(21) = 50.0; Kd(21) = 2.0; // Left Wrist
    // Kp(22) = 50.0; Kd(22) = 2.0; // Left Wrist

    // Kp(23) = 50.0; Kd(23) = 2.0; // Neck
    // Kp(24) = 50.0; Kd(24) = 2.0; // Neck

    // Kp(25) = 400.0; Kd(25) = 10.0;
    // Kp(26) = 800.0; Kd(26) = 10.0;
    // Kp(27) = 400.0; Kd(27) = 10.0;
    // Kp(28) = 400.0; Kd(28) = 10.0;
    // Kp(29) = 250.0; Kd(29) = 2.5;
    // Kp(30) = 250.0; Kd(30) = 2.0;
    // Kp(31) = 50.0; Kd(31) = 2.0; // Right Wrist
    // Kp(32) = 50.0; Kd(32) = 2.0; // Right Wrist

    Kp(0) = 2000.0;
    Kd(0) = 20.0; // Left Hip yaw
    Kp(1) = 5000.0;
    Kd(1) = 55.0; // Left Hip roll //55
    Kp(2) = 4000.0;
    Kd(2) = 45.0; // Left Hip pitch
    Kp(3) = 3700.0;
    Kd(3) = 40.0;   // Left Knee pitch
    Kp(4) = 4000.0; // 5000
    Kd(4) = 65.0;   // Left Ankle pitch /5000 / 30  //55
    Kp(5) = 4000.0; // 5000
    Kd(5) = 65.0;   // Left Ankle roll /5000 / 30 //55

    Kp(6) = 2000.0;
    Kd(6) = 20.0; // Right Hip yaw
    Kp(7) = 5000.0;
    Kd(7) = 55.0; // Right Hip roll  //55
    Kp(8) = 4000.0;
    Kd(8) = 45.0; // Right Hip pitch
    Kp(9) = 3700.0;
    Kd(9) = 40.0;    // Right Knee pitch
    Kp(10) = 4000.0; // 5000
    Kd(10) = 65.0;   // Right Ankle pitch //55
    Kp(11) = 4000.0; // 5000
    Kd(11) = 65.0;   // Right Ankle roll //55

    Kp(12) = 6000.0;
    Kd(12) = 200.0; // Waist yaw
    Kp(13) = 10000.0;
    Kd(13) = 100.0; // Waist pitch
    Kp(14) = 10000.0;
    Kd(14) = 100.0; // Waist roll

    Kp(15) = 400.0;
    Kd(15) = 10.0;
    Kp(16) = 800.0;
    Kd(16) = 10.0;
    Kp(17) = 400.0;
    Kd(17) = 10.0;
    Kp(18) = 400.0;
    Kd(18) = 10.0;
    Kp(19) = 250.0;
    Kd(19) = 2.5;
    Kp(20) = 250.0;
    Kd(20) = 2.0;
    Kp(21) = 50.0;
    Kd(21) = 2.0; // Left Wrist
    Kp(22) = 50.0;
    Kd(22) = 2.0; // Left Wrist

    Kp(23) = 50.0;
    Kd(23) = 2.0; // Neck
    Kp(24) = 50.0;
    Kd(24) = 2.0; // Neck

    Kp(25) = 400.0;
    Kd(25) = 10.0;
    Kp(26) = 800.0;
    Kd(26) = 10.0;
    Kp(27) = 400.0;
    Kd(27) = 10.0;
    Kp(28) = 400.0;
    Kd(28) = 10.0;
    Kp(29) = 250.0;
    Kd(29) = 2.5;
    Kp(30) = 250.0;
    Kd(30) = 2.0;
    Kp(31) = 50.0;
    Kd(31) = 2.0; // Right Wrist
    Kp(32) = 50.0;
    Kd(32) = 2.0; // Right Wrist
}

void AvatarController::addZmpOffset()
{
    double lfoot_zmp_offset_, rfoot_zmp_offset_;

    // lfoot_zmp_offset_ = -0.025; // 0.9 초
    // rfoot_zmp_offset_ = 0.025;

    lfoot_zmp_offset_ = -0.02; // 1.1 초
    rfoot_zmp_offset_ = 0.02;

    foot_step_support_frame_offset_ = foot_step_support_frame_;

    supportfoot_support_init_offset_ = supportfoot_support_init_;

    if (foot_step_(0, 6) == 0) // right support foot
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
        // swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
    }
    else
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
        // swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
    }

    for (int i = 0; i < total_step_num_; i++)
    {
        if (foot_step_(i, 6) == 0) // right support, left swing
        {
            foot_step_support_frame_offset_(i, 1) += lfoot_zmp_offset_;
        }
        else
        {
            foot_step_support_frame_offset_(i, 1) += rfoot_zmp_offset_;
        }
    }
}

void AvatarController::getZmpTrajectory()
{
    unsigned int planning_step_number = 3;
    unsigned int norm_size = 0;

    if (current_step_num_ >= total_step_num_ - planning_step_number)
        norm_size = (t_last_ - t_start_ + 1) * (total_step_num_ - current_step_num_) + 3.0 * hz_;
    else
        norm_size = (t_last_ - t_start_ + 1) * (planning_step_number);
    if (current_step_num_ == 0)
        norm_size = norm_size + t_temp_ + 1;
    addZmpOffset();
    zmpGenerator(norm_size, planning_step_number);
}

void AvatarController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
    ref_zmp_mj_.resize(norm_size, 2);
    Eigen::VectorXd temp_px;
    Eigen::VectorXd temp_py;

    unsigned int index = 0;
    // 매 tick 마다 zmp가 3발 앞까지 계산 된다.

    if (current_step_num_ == 0) // Walking을 수행 할 때, 정지 상태 일때 3초 동안 Ref X ZMP를 0으로 보냄. Y ZMP는 제자리 유지.
    {
        for (int i = 0; i <= t_temp_; i++) // 600 tick
        {
            if (i < 1.0 * hz_)
            {
                ref_zmp_mj_(i, 0) = com_support_init_(0);
                ref_zmp_mj_(i, 1) = com_support_init_(1);
            }
            else if (i < 2.0 * hz_)
            {
                double del_x = i - 1.0 * hz_;
                ref_zmp_mj_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (1.0 * hz_);
                ref_zmp_mj_(i, 1) = com_support_init_(1);
            }
            else
            {
                ref_zmp_mj_(i, 0) = 0.0;
                ref_zmp_mj_(i, 1) = com_support_init_(1);
            }
            index++;
        }
    }
    /////////////////////////////////////////////////////////////////////
    if (current_step_num_ >= total_step_num_ - planning_step_num)
    {
        for (unsigned int i = current_step_num_; i < total_step_num_; i++)
        {
            onestepZmp(i, temp_px, temp_py);

            for (unsigned int j = 0; j < t_total_; j++)
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);
            }
            index = index + t_total_;
        }

        for (unsigned int j = 0; j < 3.0 * hz_; j++)
        {
            ref_zmp_mj_(index + j, 0) = ref_zmp_mj_(index - 1, 0);
            ref_zmp_mj_(index + j, 1) = ref_zmp_mj_(index - 1, 1);
        }
        index = index + 3.0 * hz_;
    }
    else // 보행 중 사용 하는 Ref ZMP
    {
        for (unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)
        {
            onestepZmp(i, temp_px, temp_py);
            for (unsigned int j = 0; j < t_total_; j++) // 1 step 보행은 1.2초, 240 tick
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);
            }
            index = index + t_total_; // 참조 zmp가 이만큼 쌓였다.
                                      // 결국 실제 로봇 1Hz마다 720개의 ref_zmp를 생성함. 3.6초
        }
    }
}

void AvatarController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py)
{
    temp_px.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
    temp_py.resize(t_total_);
    temp_px.setZero();
    temp_py.setZero();

    double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0, Kx2 = 0, Ky2 = 0;
    if (current_step_number == 0)
    {
        Kx = 0;
        Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
        Kx2 = foot_step_support_frame_offset_(current_step_number, 0) / 2 - supportfoot_support_init_offset_(0);
        Ky2 = foot_step_support_frame_offset_(current_step_number, 1) / 2 - supportfoot_support_init_offset_(1);

        for (int i = 0; i < t_total_; i++)
        {
            if (i >= 0 && i < t_rest_init_ + t_double1_) // 0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = Kx / (t_double1_ + t_rest_init_) * (i + 1);
                temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) // 0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = 0;
                temp_py(i) = supportfoot_support_init_offset_(1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) // 1.05 ~ 1.15초 , 210 ~ 230 tick
            {
                temp_px(i) = 0 + Kx2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = supportfoot_support_init_offset_(1) + Ky2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
    else if (current_step_number == 1)
    {
        Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2;
        Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2;
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

        for (int i = 0; i < t_total_; i++)
        {
            if (i < t_rest_init_ + t_double1_) // 0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) // 0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) // 1.05 ~ 1.2초 , 210 ~ 240 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
    else
    {
        Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - ((foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2);
        Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - ((foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2);
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

        for (int i = 0; i < t_total_; i++)
        {
            if (i < t_rest_init_ + t_double1_) // 0 ~ 0.15초 , 0 ~ 30 tick
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) // 0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) // 1.05 ~ 1.2초 , 210 ~ 240 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
}

void AvatarController::getFootTrajectory()
{
    Eigen::Vector6d target_swing_foot;

    for (int i = 0; i < 6; i++)
    {
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i);
    }

    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            lfoot_trajectory_support_.translation().setZero();
            lfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_support_.translation()(2) = 0;
            rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            rfoot_trajectory_support_.translation().setZero();
            rfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_support_.translation()(2) = 0;
            lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        double t_rest_temp = 0.00 * hz_;

        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            // lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            }

            rfoot_trajectory_euler_support_(0) = 0;
            rfoot_trajectory_euler_support_(1) = 0;
            rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            }

            lfoot_trajectory_euler_support_(0) = 0;
            lfoot_trajectory_euler_support_(1) = 0;
            lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            // lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }
    else
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_euler_support_.setZero();
            // lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            for (int i = 0; i < 3; i++)
            {
                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            // rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            // rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

            for (int i = 0; i < 3; i++)
            {
                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            // lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }
}

void AvatarController::preview_Parameter(double dt, int NL, Eigen::MatrixXd &Gi, Eigen::VectorXd &Gd, Eigen::MatrixXd &Gx, Eigen::MatrixXd &A, Eigen::VectorXd &B, Eigen::MatrixXd &C)
{
    A.resize(3, 3);
    A(0, 0) = 1.0;
    A(0, 1) = dt;
    A(0, 2) = dt * dt * 0.5;
    A(1, 0) = 0;
    A(1, 1) = 1.0;
    A(1, 2) = dt;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    B.resize(3);
    B(0) = dt * dt * dt / 6;
    B(1) = dt * dt / 2;
    B(2) = dt;

    C.resize(1, 3);
    C(0, 0) = 1;
    C(0, 1) = 0;
    C(0, 2) = -0.71 / 9.81;

    Eigen::MatrixXd A_bar;
    Eigen::VectorXd B_bar;

    B_bar.resize(4);
    B_bar.segment(0, 1) = C * B;
    B_bar.segment(1, 3) = B;

    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();

    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.resize(4, 4);
    I_bar.resize(4, 1);
    F_bar.resize(4, 3);
    F_bar.setZero();

    F_bar.block<1, 3>(0, 0) = C * A;
    F_bar.block<3, 3>(1, 0) = A;

    I_bar.setZero();
    I_bar(0, 0) = 1.0;

    A_bar.block<4, 1>(0, 0) = I_bar;
    A_bar.block<4, 3>(0, 1) = F_bar;

    Eigen::MatrixXd Qe;
    Qe.resize(1, 1);
    Qe(0, 0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1, 1);
    R(0, 0) = 0.000001;

    Eigen::MatrixXd Qx;
    Qx.resize(3, 3);
    Qx.setZero();

    Eigen::MatrixXd Q_bar;
    Q_bar.resize(3, 3);
    Q_bar.setZero();
    Q_bar(0, 0) = Qe(0, 0);

    Eigen::Matrix4d K;

    K(0, 0) = 1083.572780788710;
    K(0, 1) = 586523.188429418020;
    K(0, 2) = 157943.283121116518;
    K(0, 3) = 41.206077691894;
    K(1, 0) = 586523.188429418020;
    K(1, 1) = 319653984.254277825356;
    K(1, 2) = 86082274.531361579895;
    K(1, 3) = 23397.754069026785;
    K(2, 0) = 157943.283121116518;
    K(2, 1) = 86082274.531361579895;
    K(2, 2) = 23181823.112113621086;
    K(2, 3) = 6304.466397614751;
    K(3, 0) = 41.206077691894;
    K(3, 1) = 23397.754069026785;
    K(3, 2) = 6304.466397614751;
    K(3, 3) = 2.659250532188;

    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.resize(1, 1);
    Temp_mat.setZero();
    Temp_mat_inv.resize(1, 1);
    Temp_mat_inv.setZero();
    Ac_bar.setZero();
    Ac_bar.resize(4, 4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();

    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;

    Eigen::MatrixXd Ac_bar_tran(4, 4);
    Ac_bar_tran = Ac_bar.transpose();

    Gi.resize(1, 1);
    Gx.resize(1, 3);
    Gi(0, 0) = 872.3477; // Temp_mat_inv * B_bar_tran * K * I_bar ;
    // Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;
    Gx(0, 0) = 945252.1760702;
    Gx(0, 1) = 256298.6905049;
    Gx(0, 2) = 542.0544196;
    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.resize(4, NL);
    X_bar.setZero();
    X_bar_col.setZero();
    X_bar_col = -Ac_bar_tran * K * I_bar;

    for (int i = 0; i < NL; i++)
    {
        X_bar.block<4, 1>(0, i) = X_bar_col;
        X_bar_col = Ac_bar_tran * X_bar_col;
    }

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0, 0);

    for (int i = 0; i < NL; i++)
    {
        Gd.segment(i, 1) = Gd_col;
        Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i);
    }
}

void AvatarController::previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double &UX, double &UY,
                                         Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{

    int zmp_size;
    zmp_size = ref_zmp_mj_.col(1).size();
    Eigen::VectorXd px_ref, py_ref;
    px_ref.resize(zmp_size);
    py_ref.resize(zmp_size);

    for (int i = 0; i < zmp_size; i++)
    {
        px_ref(i) = ref_zmp_mj_(i, 0);
        py_ref(i) = ref_zmp_mj_(i, 1);
    }

    ZMP_X_REF = px_ref(tick);
    ZMP_Y_REF = py_ref(tick);

    Eigen::VectorXd px, py;
    px.resize(1);
    py.resize(1);

    if (tick == 0 && current_step_num_ == 0)
    {
        preview_x_b_mj.setZero();
        preview_y_b_mj.setZero();
        preview_x_mj.setZero();
        preview_y_mj.setZero();
        preview_x_b_mj(0) = x_i;
        preview_y_b_mj(0) = y_i;
        preview_x_mj(0) = x_i;
        preview_y_mj(0) = y_i;
        UX = 0;
        UY = 0;
        cout << "preview X state : " << preview_x_mj(0) << "," << preview_x_mj(1) << "," << preview_x_mj(2) << endl;
        cout << "preview Y state : " << preview_y_mj(0) << "," << preview_y_mj(1) << "," << preview_y_mj(2) << endl;
    }
    else
    {
        preview_x_mj = xs;
        preview_y_mj = ys;

        preview_x_b_mj(0) = preview_x_mj(0) - preview_x_mj(1) * 0.0005;
        preview_y_b_mj(0) = preview_y_mj(0) - preview_y_mj(1) * 0.0005;
        preview_x_b_mj(1) = preview_x_mj(1) - preview_x_mj(2) * 0.0005;
        preview_y_b_mj(1) = preview_y_mj(1) - preview_y_mj(2) * 0.0005;
        preview_x_b_mj(2) = preview_x_mj(2) - UX * 0.0005;
        preview_y_b_mj(2) = preview_y_mj(2) - UY * 0.0005;
    }
    px = C * preview_x_mj;
    py = C * preview_y_mj;

    double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

    for (int i = 0; i < NL; i++)
    {
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (px_ref(tick + 1 + i) - px_ref(tick + i));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (py_ref(tick + 1 + i) - py_ref(tick + i));
    }

    Eigen::MatrixXd del_ux(1, 1);
    Eigen::MatrixXd del_uy(1, 1);
    del_ux.setZero();
    del_uy.setZero();

    Eigen::VectorXd GX_X(1);
    GX_X = Gx * (preview_x_mj - preview_x_b_mj);
    Eigen::VectorXd GX_Y(1);
    GX_Y = Gx * (preview_y_mj - preview_y_b_mj);

    if (walking_tick_mj == 0)
    {
        del_zmp.setZero();
        cout << "del_zmp : " << del_zmp(0) << "," << del_zmp(1) << endl;
    }

    del_ux(0, 0) = -(px(0) - px_ref(tick)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - py_ref(tick)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x_mj + B * UX;
    YD = A * preview_y_mj + B * UY;
    // SC_err_compen(XD(0), YD(0));
    if (walking_tick_mj == 0)
    {
        zmp_err_(0) = 0;
        zmp_err_(1) = 0;
    }
    else
    {
        zmp_err_(0) = zmp_err_(0) + (px_ref(tick) - zmp_measured_LPF_(0)) * 0.0005;
        zmp_err_(1) = zmp_err_(1) + (py_ref(tick) - zmp_measured_LPF_(1)) * 0.0005;
    }

    cp_desired_(0) = XD(0) + XD(1) / wn;
    cp_desired_(1) = YD(0) + YD(1) / wn;

    SC_err_compen(com_support_current_(0), com_support_current_(1));

    cp_measured_(0) = com_support_cp_(0) + com_float_current_dot_LPF(0) / wn;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;

    del_zmp(0) = 1.4 * (cp_measured_(0) - cp_desired_(0));
    del_zmp(1) = 1.3 * (cp_measured_(1) - cp_desired_(1));

    CLIPM_ZMP_compen_MJ(del_zmp(0), del_zmp(1));
    // cout  << px_ref(tick) << "," << py_ref(tick) << "," << XD(0) << "," << YD(0) << endl;
    // MJ_graph << px_ref(tick) << "," << py_ref(tick) << "," << XD(0) << "," << YD(0) << endl;
    // MJ_graph << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << endl;
}

void AvatarController::SC_err_compen(double x_des, double y_des)
{
    if (walking_tick_mj == 0)
    {
        SC_com.setZero();
    }
    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {
        sc_err_before.setZero();
        sc_err_before(0) = com_support_current_(0) - foot_step_support_frame_(current_step_num_, 0); // 1.3으로 할꺼면 마지막에 더해야됨. SC_com을 이 함수보다 나중에 더하기 때문에
                                                                                                     // sc_err_before(1) = y_des - com_support_current_(1);
    }

    if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {
        sc_err_after.setZero();
        sc_err_after(0) = com_support_current_(0);
        // sc_err_after(1) = y_des - com_support_current_(1);
        sc_err = sc_err_after - sc_err_before;
    }

    if (current_step_num_ != 0)
    {
        SC_com(0) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05 * hz_, sc_err(0), 0, 0.0, 0.0);
        SC_com(1) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05 * hz_, sc_err(1), 0, 0.0, 0.0);
    }

    if (current_step_num_ != total_step_num_ - 1)
    {
        if (current_step_num_ != 0 && walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_total_)
        {
            com_support_cp_(0) = com_support_current_(0) - SC_com(0);
        }
        else
        {
            com_support_cp_(0) = com_support_current_(0);
        }
    }
    else if (current_step_num_ == total_step_num_ - 1)
    {
        if (walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 2 * t_total_)
        {
            com_support_cp_(0) = com_support_current_(0) - SC_com(0);
        }
    }
}

void AvatarController::getPelvTrajectory()
{
    // double pelv_offset = -0.00; // DG
    // double pelv_transition_time = 1.0;
    // if (walking_enable_ == true)
    // {
    //     pelv_height_offset_ = DyrosMath::cubic(walking_tick_mj, 0, pelv_transition_time * hz_, pelv_support_init_.translation()(2) - com_desired_(2), 0.0, 0.0, 0.0);
    // }
    // else
    // {
    //     pelv_height_offset_ = DyrosMath::cubic(rd_.control_time_, init_leg_time_, init_leg_time_ + 5.0, pelv_support_init_.translation()(2) - com_desired_(2), pelv_offset, 0.0, 0.0);
    // }

    double pelv_transition_time = 2.0;
    double pelv_height_offset_ = 0.0;
    if (walking_enable_ == true)
    {
        pelv_height_offset_ = DyrosMath::cubic(walking_tick_mj, 0, pelv_transition_time * hz_, 0.0, 0.05, 0.0, 0.0);
    }

    double z_rot = foot_step_support_frame_(current_step_num_, 5);

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - 0 * 0.15 * damping_x - com_support_current_(0)); //- 0.01 * zmp_err_(0) * 0;
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7 * (com_desired_(1) - 0 * 0.6 * damping_y - com_support_current_(1));  //- 0.01 * zmp_err_(1) * 0;
    // pelv_trajectory_support_.translation()(2) = com_desired_(2) + pelv_height_offset_; //DG
    pelv_trajectory_support_.translation()(2) = com_desired_(2) - 0 * pelv_height_offset_;

    // MJ_graph << com_desired_(0) << "," << com_support_current_(0) << "," << com_desired_(1) << "," << com_support_current_(1) << endl;
    Eigen::Vector3d Trunk_trajectory_euler;
    Trunk_trajectory_euler.setZero();

    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_double2_ - t_rest_last_, pelv_support_euler_init_(2), z_rot / 2.0, 0.0, 0.0);
    }
    else
    {
        Trunk_trajectory_euler(2) = z_rot / 2.0;
    }

    // P_angle_i = P_angle_i + (0 - P_angle)*del_t;
    // Trunk_trajectory_euler(1) = 0.05*(0.0 - P_angle) + 1.5*P_angle_i;
    if (aa == 0 && walking_tick_mj == 0 && (walking_enable_ == true))
    {
        P_angle_input = 0;
        R_angle_input = 0;
    }

    P_angle_input_dot = 1.5 * (0.0 - P_angle) - 0.01 * P_angle_input;
    R_angle_input_dot = 2.0 * (0.0 - R_angle) - 0.01 * R_angle_input;

    P_angle_input = P_angle_input + P_angle_input_dot * del_t;
    R_angle_input = R_angle_input + R_angle_input_dot * del_t;

    if (R_angle_input > 3 * DEG2RAD) // 1.5 degree
    {
        R_angle_input = 3 * DEG2RAD;
    }
    else if (R_angle_input < -3 * DEG2RAD)
    {
        R_angle_input = -3 * DEG2RAD;
    }

    if (P_angle_input > 5 * DEG2RAD) // 5 degree
    {
        P_angle_input = 5 * DEG2RAD;
        // cout << "a" << endl;
    }
    else if (P_angle_input < -5 * DEG2RAD)
    {
        P_angle_input = -5 * DEG2RAD;
        // cout << "b" << endl;
    }
    // Trunk_trajectory_euler(0) = R_angle_input;
    Trunk_trajectory_euler(1) = P_angle_input;

    // MJ_graph << P_angle * 180 / 3.141592 << "," << Trunk_trajectory_euler(1) << endl;

    pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2)) * DyrosMath::rotateWithY(Trunk_trajectory_euler(1)) * DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void AvatarController::supportToFloatPattern()
{
    // lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
    // rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
    pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;

    rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
    lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
}

void AvatarController::getComTrajectory()
{
    if (walking_tick_mj == 0)
    {
        Gi_mj_.setZero();
        Gx_mj_.setZero();
        Gd_mj_.setZero();
        preview_Parameter(1.0 / hz_, 16 * hz_ / 10, Gi_mj_, Gd_mj_, Gx_mj_, A_mj_, B_mj_, C_mj_);
        xs_mj_(0) = xi_mj_;
        xs_mj_(1) = 0;
        xs_mj_(2) = 0;
        ys_mj_(0) = yi_mj_;
        ys_mj_(1) = 0;
        xs_mj_(2) = 0;
        UX_mj_ = 0;
        UY_mj_ = 0;
        xd_mj_ = xs_mj_;
    }

    if (current_step_num_ == 0)
    {
        zmp_start_time_mj_ = 0.0;
    }
    else
    {
        zmp_start_time_mj_ = t_start_;
    }

    previewcontroller(0.0005, 3200, walking_tick_mj - zmp_start_time_mj_, xi_mj_, yi_mj_, xs_mj_, ys_mj_, UX_mj_, UY_mj_, Gi_mj_, Gd_mj_, Gx_mj_, A_mj_, B_mj_, C_mj_, xd_mj_, yd_mj_);

    xs_mj_ = xd_mj_;
    ys_mj_ = yd_mj_;

    com_desired_(0) = xd_mj_(0);
    com_desired_(1) = yd_mj_(0);
    // com_desired_(2) = pelv_support_start_.translation()(2);
    com_desired_(2) = 0.77172;

    // SC_err_compen(com_desired_(0), com_desired_(1));

    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1)
    {
        Eigen::Vector3d com_pos_prev;
        Eigen::Vector3d com_pos;
        Eigen::Vector3d com_vel_prev;
        Eigen::Vector3d com_vel;
        Eigen::Vector3d com_acc_prev;
        Eigen::Vector3d com_acc;
        Eigen::Matrix3d temp_rot;
        Eigen::Vector3d temp_pos;

        temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        for (int i = 0; i < 3; i++)
            temp_pos(i) = foot_step_support_frame_(current_step_num_, i);

        com_pos_prev(0) = xs_mj_(0);
        com_pos_prev(1) = ys_mj_(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = xs_mj_(1);
        com_vel_prev(1) = ys_mj_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = xs_mj_(2);
        com_acc_prev(1) = ys_mj_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        xs_mj_(0) = com_pos(0);
        ys_mj_(0) = com_pos(1);
        xs_mj_(1) = com_vel(0);
        ys_mj_(1) = com_vel(1);
        xs_mj_(2) = com_acc(0);
        ys_mj_(2) = com_acc(1);
    }
}

void AvatarController::computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d &q_des)
{
    Eigen::Vector3d R_r, R_D, L_r, L_D;

    L_D << 0.11, +0.1025, -0.1025;
    R_D << 0.11, -0.1025, -0.1025;

    L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * L_D - float_lleg_transform.translation());
    R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * R_D - float_rleg_transform.translation());

    double R_C = 0, L_C = 0, L_upper = 0.351, L_lower = 0.351, R_alpha = 0, L_alpha = 0;

    L_C = sqrt(pow(L_r(0), 2) + pow(L_r(1), 2) + pow(L_r(2), 2));
    R_C = sqrt(pow(R_r(0), 2) + pow(R_r(1), 2) + pow(R_r(2), 2));

    q_des(3) = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2)) / (2 * L_upper * L_lower)) + M_PI);
    q_des(9) = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2)) / (2 * L_upper * L_lower)) + M_PI);
    L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
    R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));

    q_des(4) = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2))) - L_alpha;
    q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2))) - R_alpha;

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
    Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

    L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3) - q_des(4));
    L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
    R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9) - q_des(10));
    R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11));

    L_Hip_rot_mat.setZero();
    R_Hip_rot_mat.setZero();

    L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
    R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

    q_des(0) = atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1));                                                       // Hip yaw
    q_des(1) = atan2(L_Hip_rot_mat(2, 1), -L_Hip_rot_mat(0, 1) * sin(q_des(0)) + L_Hip_rot_mat(1, 1) * cos(q_des(0))); // Hip roll
    q_des(2) = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2));                                                       // Hip pitch
    q_des(3) = q_des(3);                                                                                               // Knee pitch
    q_des(4) = q_des(4);                                                                                               // Ankle pitch
    q_des(5) = atan2(L_r(1), L_r(2));                                                                                  // Ankle roll

    q_des(6) = atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    q_des(7) = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_des(6)) + R_Hip_rot_mat(1, 1) * cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = atan2(R_r(1), R_r(2));

    if (walking_tick_mj == 0)
    {
        sc_joint_err.setZero();
    }

    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {                                                                                           // 5.3, 0
        sc_joint_before.setZero();
        sc_joint_before = q_des;
    }
    if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {                                                          // 5.3005, 1
        sc_joint_after.setZero();
        sc_joint_after = q_des;

        sc_joint_err = sc_joint_after - sc_joint_before;
    }
    if (current_step_num_ != 0)
    {
        for (int i = 0; i < 12; i++)
        {
            SC_joint(i) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.005 * hz_, sc_joint_err(i), 0.0, 0.0, 0.0);
        }

        if (walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 0.005 * hz_)
        {
            q_des = q_des - SC_joint;
        }
    }
}

void AvatarController::GravityCalculate_MJ()
{
    double contact_gain = 0.0;
    double eta = 0.9;
    VectorQd grav_;

    if (walking_tick_mj < t_start_ + t_rest_init_)
    {
        WBC::SetContact(rd_, 1, 1);
        Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
        Gravity_SSP_.setZero();
        contact_gain = 1.0;
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_) // 0.03 s
    {
        contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_, t_start_ + t_rest_init_ + t_double1_, 1.0, 0.0, 0.0, 0.0);

        WBC::SetContact(rd_, 1, 1);
        Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
        Gravity_SSP_.setZero();
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }

    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) // SSP
    {
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            WBC::SetContact(rd_, 1, 0);
            Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_SSP_(1) = 1.0 * Gravity_SSP_(1);
            Gravity_SSP_(5) = 1.0 * Gravity_SSP_(5);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            WBC::SetContact(rd_, 0, 1);
            Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_SSP_(7) = 1.0 * Gravity_SSP_(7);
            Gravity_SSP_(11) = 1.0 * Gravity_SSP_(11);
        }
        Gravity_DSP_.setZero();
        contact_torque_MJ.setZero();
    }

    else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_)
    {
        contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_, t_start_ + t_total_ - t_rest_last_, 0.0, 1.0, 0.0, 0.0);
        Gravity_SSP_.setZero();
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            WBC::SetContact(rd_, 1, 1);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            WBC::SetContact(rd_, 1, 1);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }
    else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
    {
        contact_gain = 1.0;

        WBC::SetContact(rd_, 1, 1);
        Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);

        Gravity_SSP_.setZero();
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }

    if (atb_grav_update_ == false)
    {
        atb_grav_update_ = true;
        Gravity_MJ_ = Gravity_DSP_ + Gravity_SSP_; // + contact_torque_MJ;
        atb_grav_update_ = false;
    }
    // return grav_;
}
Eigen::VectorQd AvatarController::floatGravityTorque(Eigen::VectorQVQd q)
{
    // old version test
    Eigen::VectorQd gravity_torque;

    Eigen::VectorVQd G_mat;

    G_mat.setZero();
    for (int i = 0; i < MODEL_DOF + 1; i++)
        G_mat -= link_avatar_[i].jac_com.cast<double>().topRows(3).transpose() * link_avatar_[i].mass * rd_.grav_ref;

    gravity_torque = G_mat.segment(6, MODEL_DOF);
    return gravity_torque;
}
void AvatarController::parameterSetting()
{
    target_x_ = 0.0;
    target_y_ = 0.0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0.0;
    step_length_x_ = 0.1;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;

    // t_rest_init_ = 0.27*hz_;
    // t_rest_last_ = 0.27*hz_;
    // t_double1_ = 0.03*hz_;
    // t_double2_ = 0.03*hz_;
    // t_total_= 1.3*hz_;

    // t_rest_init_ = 0.12 * hz_; // Slack, 0.9 step time
    // t_rest_last_ = 0.12 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 0.9 * hz_;

    t_rest_init_ = 0.2 * hz_; // slack
    t_rest_last_ = 0.2 * hz_;
    t_double1_ = 0.03 * hz_;
    t_double2_ = 0.03 * hz_;
    t_total_ = 1.1 * hz_;

    t_temp_ = 4.0 * hz_;
    t_last_ = t_total_ + t_temp_;
    t_start_ = t_temp_ + 1;
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
    foot_height_ = 0.055;      // 0.9 sec 0.05
    pelv_height_offset_ = 0.0; // change pelvis height for manipulation when the robot stop walking
}

void AvatarController::updateNextStepTime()
{
    if (walking_tick_mj == t_last_)
    {
        if (current_step_num_ != total_step_num_ - 1)
        {
            t_start_ = t_last_ + 1;
            t_start_real_ = t_start_ + t_rest_init_;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;
        }
    }
    if (current_step_num_ == total_step_num_ - 1 && walking_tick_mj >= t_last_ + t_total_)
    {
        // walking_enable_ = false;
        // cout << "Last " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;
    }
    else
    {
        walking_tick_mj++;
    }
}

void AvatarController::CLIPM_ZMP_compen_MJ(double XZMP_ref, double YZMP_ref)
{
    double Kp_x_ssp, Kv_x_ssp;
    double Kp_y_ssp, Kv_y_ssp;
    Kp_x_ssp = 30;
    Kv_x_ssp = 2;
    Kp_y_ssp = 40;
    Kv_y_ssp = 2;
    double del_t = 0.0005;

    if (walking_tick_mj == 0)
    {
        A_x_ssp.resize(2, 2);
        B_x_ssp.resize(2, 1);
        Ad_x_ssp.resize(2, 2);
        Bd_x_ssp.resize(2, 1);
        C_x_ssp.resize(1, 2);
        D_x_ssp.resize(1, 1);

        A_y_ssp.resize(2, 2);
        B_y_ssp.resize(2, 1);
        Ad_y_ssp.resize(2, 2);
        Bd_y_ssp.resize(2, 1);
        C_y_ssp.resize(1, 2);
        D_y_ssp.resize(1, 1);

        ff_gain_x_ssp.resize(1, 1);
        ff_gain_y_ssp.resize(1, 1);

        K_x_ssp.resize(1, 2);
        K_y_ssp.resize(1, 2);

        X_x_ssp.setZero();
        Y_x_ssp.resize(1, 1);

        X_y_ssp.setZero();
        Y_y_ssp.resize(1, 1);

        K_x_ssp(0, 0) = 0.0083;
        K_x_ssp(0, 1) = 0.19;
        // Control pole : -5 , damping : 0.7 (실제 로봇) // Control pole : -7 , damping : 0.9 (시뮬레이션)
        K_y_ssp(0, 0) = -0.375;
        K_y_ssp(0, 1) = 0.125;

        // Define the state space equation
        A_x_ssp(0, 0) = 0;
        A_x_ssp(0, 1) = 1;
        A_x_ssp(1, 0) = -Kp_x_ssp;
        A_x_ssp(1, 1) = -Kv_x_ssp;

        B_x_ssp(0, 0) = 0;
        B_x_ssp(1, 0) = Kp_x_ssp;

        Ad_x_ssp(0, 0) = 1 - 0.5 * Kp_x_ssp * del_t * del_t;
        Ad_x_ssp(0, 1) = del_t - 0.5 * Kv_x_ssp * del_t * del_t;
        Ad_x_ssp(1, 0) = -Kp_x_ssp * del_t;
        Ad_x_ssp(1, 1) = 1 - Kv_x_ssp * del_t;

        Bd_x_ssp(0, 0) = 0.5 * Kp_x_ssp * del_t * del_t;
        Bd_x_ssp(1, 0) = Kp_x_ssp * del_t;

        C_x_ssp(0, 0) = 1 + zc_mj_ / GRAVITY * Kp_x_ssp;
        C_x_ssp(0, 1) = zc_mj_ / GRAVITY * Kv_x_ssp;

        D_x_ssp(0, 0) = -zc_mj_ / GRAVITY * Kp_x_ssp;

        ff_gain_x_ssp = (-(C_x_ssp - D_x_ssp * K_x_ssp) * ((A_x_ssp - B_x_ssp * K_x_ssp).inverse()) * B_x_ssp + D_x_ssp).inverse();

        A_y_ssp(0, 0) = 0;
        A_y_ssp(0, 1) = 1;
        A_y_ssp(1, 0) = -Kp_y_ssp;
        A_y_ssp(1, 1) = -Kv_y_ssp;

        B_y_ssp(0, 0) = 0;
        B_y_ssp(1, 0) = Kp_y_ssp;

        Ad_y_ssp(0, 0) = 1 - 0.5 * Kp_y_ssp * del_t * del_t;
        Ad_y_ssp(0, 1) = del_t - 0.5 * Kv_y_ssp * del_t * del_t;
        Ad_y_ssp(1, 0) = -Kp_y_ssp * del_t;
        Ad_y_ssp(1, 1) = 1 - Kv_y_ssp * del_t;

        Bd_y_ssp(0, 0) = 0.5 * Kp_y_ssp * del_t * del_t;
        Bd_y_ssp(1, 0) = Kp_y_ssp * del_t;

        C_y_ssp(0, 0) = 1 + zc_mj_ / GRAVITY * Kp_y_ssp;
        C_y_ssp(0, 1) = zc_mj_ / GRAVITY * Kv_y_ssp;

        D_y_ssp(0, 0) = -zc_mj_ / GRAVITY * Kp_y_ssp;

        ff_gain_y_ssp = (-(C_y_ssp - D_y_ssp * K_y_ssp) * ((A_y_ssp - B_y_ssp * K_y_ssp).inverse()) * B_y_ssp + D_y_ssp).inverse();
    }

    // X_x_ssp(0) = com_float_current_(0);
    X_x_ssp(0) = com_support_current_(0);

    if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
    {
        X_y_ssp(0) = com_support_current_(1) - rfoot_support_current_.translation()(1) * 0.5;
    }
    else if (foot_step_(current_step_num_, 6) == 0)
    {
        X_y_ssp(0) = com_support_current_(1) - lfoot_support_current_.translation()(1) * 0.5;
    }

    U_ZMP_x_ssp = -(K_x_ssp(0, 0) * X_x_ssp(0) + K_x_ssp(0, 1) * preview_x_mj(1)) + XZMP_ref * ff_gain_x_ssp(0, 0);
    U_ZMP_y_ssp = -(K_y_ssp(0, 0) * X_y_ssp(0) + K_y_ssp(0, 1) * preview_y_mj(1)) + YZMP_ref * ff_gain_y_ssp(0, 0);

    U_ZMP_x_ssp_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * U_ZMP_x_ssp_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * U_ZMP_x_ssp;
    U_ZMP_y_ssp_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * U_ZMP_y_ssp_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * U_ZMP_y_ssp;
    if (walking_tick_mj == 0)
    {
        U_ZMP_x_ssp_LPF = U_ZMP_x_ssp;
        U_ZMP_y_ssp_LPF = U_ZMP_y_ssp;
    }

    damping_x = U_ZMP_x_ssp_LPF;
    damping_y = U_ZMP_y_ssp_LPF;

    if (damping_x > 0.02)
    {
        damping_x = 0.02;
    }
    else if (damping_x < -0.02)
    {
        damping_x = -0.02;
    }

    if (damping_y > 0.03) // 로봇 0.03, 시뮬 0.02
    {
        damping_y = 0.03;
    }
    else if (damping_y < -0.03)
    {
        damping_y = -0.03;
    }

    // MJ_graph << cp_desired_(1) << "," << cp_measured_(1) << "," << com_float_current_(1) << "," << X_y_ssp(0) << "," << damping_y << endl;
}

void AvatarController::hip_compensator()
{
    double left_hip_roll = -0.2 * DEG2RAD, right_hip_roll = -0.2 * DEG2RAD, left_hip_roll_first = -0.20 * DEG2RAD, right_hip_roll_first = -0.20 * DEG2RAD, //실험, 제자리 0.6, 0.4
        left_hip_pitch = 0.4 * DEG2RAD, right_hip_pitch = 0.4 * DEG2RAD, left_hip_pitch_first = 0.40 * DEG2RAD, right_hip_pitch_first = 0.40 * DEG2RAD,    // 실험 , 제자리 0.75deg
        left_ank_pitch = 0.0 * DEG2RAD, right_ank_pitch = 0.0 * DEG2RAD, left_ank_pitch_first = 0.0 * DEG2RAD, right_ank_pitch_first = 0.0 * DEG2RAD,
           left_hip_roll_temp = 0.0, right_hip_roll_temp = 0.0, left_hip_pitch_temp = 0.0, right_hip_pitch_temp = 0.0, left_ank_pitch_temp = 0.0, right_ank_pitch_temp = 0.0, temp_time = 0.05 * hz_;

    if (current_step_num_ == 0)
    {
        if (foot_step_(current_step_num_, 6) == 1) // left support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_hip_roll_first, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_hip_pitch_first, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, left_ank_pitch_first, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_hip_roll_first, 0.0, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_hip_pitch_first, 0.0, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_ank_pitch_first, 0.0, 0.0, 0.0);
            }
            else
            {
                left_hip_roll_temp = 0.0;
                left_hip_pitch_temp = 0.0;
                left_ank_pitch_temp = 0.0;
            }
        }
        else if (foot_step_(current_step_num_, 6) == 0) // right support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_hip_roll_first, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_hip_pitch_first, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0.0, right_ank_pitch_first, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_hip_roll_first, 0.0, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_hip_pitch_first, 0.0, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_ank_pitch_first, 0.0, 0.0, 0.0);
            }
            else
            {
                right_hip_roll_temp = 0.0;
                right_hip_pitch_temp = 0.0;
                right_ank_pitch_temp = 0.0;
            }
        }
    }
    else
    {
        if (foot_step_(current_step_num_, 6) == 1) // left support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_roll, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_pitch, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_ank_pitch, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_hip_roll, 0.0, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_hip_pitch, 0.0, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_ank_pitch, 0.0, 0.0, 0.0);
            }
            else
            {
                left_hip_roll_temp = 0;
                left_hip_pitch_temp = 0;
                left_ank_pitch_temp = 0;
            }
        }
        else if (foot_step_(current_step_num_, 6) == 0) // right support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_roll, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_pitch, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_ank_pitch, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_hip_roll, 0.0, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_hip_pitch, 0.0, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_ank_pitch, 0.0, 0.0, 0.0);
            }
            else
            {
                right_hip_roll_temp = 0;
                right_hip_pitch_temp = 0;
                right_ank_pitch_temp = 0.0;
            }
        }
    }

    ref_q_(1) = ref_q_(1) - left_hip_roll_temp;
    ref_q_(7) = ref_q_(7) + right_hip_roll_temp;
    ref_q_(2) = ref_q_(2) - left_hip_pitch_temp;
    ref_q_(8) = ref_q_(8) - right_hip_pitch_temp;
    ref_q_(4) = ref_q_(4) - left_ank_pitch_temp;
    ref_q_(10) = ref_q_(10) - right_ank_pitch_temp;
}

void AvatarController::Compliant_control(Eigen::Vector12d desired_leg_q)
{
    Eigen::Vector12d current_u;
    double del_t = 0.0, Kp = 0.0;
    del_t = 1 / hz_;
    Kp = 100.0; // 실험
                //   Kp = 20.0; // 시뮬

    if (walking_tick_mj == 0)
    {
        for (int i = 0; i < 12; i++)
        {
            DOB_IK_output_b_(i) = rd_.q_(i);
            DOB_IK_output_(i) = rd_.q_(i);
            current_u(i) = rd_.q_(i);
        }
    }

    if (walking_tick_mj > 0)
    {
        for (int i = 0; i < 12; i++)
        {
            current_u(i) = (rd_.q_(i) - (1 - Kp * del_t) * q_prev_MJ_(i)) / (Kp * del_t);
        }
    }

    Eigen::Vector12d d_hat;
    d_hat = current_u - DOB_IK_output_b_;

    if (walking_tick_mj == 0)
        d_hat_b = d_hat;

    d_hat = (2 * M_PI * 5.0 * del_t) / (1 + 2 * M_PI * 5.0 * del_t) * d_hat + 1 / (1 + 2 * M_PI * 5.0 * del_t) * d_hat_b;

    double default_gain = 0.0;
    double compliant_gain = 0.0;
    double compliant_tick = 0.1 * hz_;
    double gain_temp = 0.0;
    for (int i = 0; i < 12; i++)
    {
        if (i < 6)
        {
            gain_temp = default_gain;

            if (foot_step_(current_step_num_, 6) == 0)
            {
                if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
                {
                    gain_temp = default_gain;
                }
                else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick)
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, default_gain, compliant_gain, 0.0, 0.0);
                }
                else
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
                }
            }
            else
            {
                gain_temp = default_gain;
            }

            DOB_IK_output_(i) = desired_leg_q(i) + gain_temp * d_hat(i);
        }
        else
        {
            gain_temp = default_gain;

            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지 상태
            {
                if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
                {
                    gain_temp = default_gain;
                }
                else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick)
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, default_gain, compliant_gain, 0.0, 0.0);
                }
                else
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
                }
            }
            else // 오른발 지지 상태
            {
                gain_temp = default_gain;
            }

            DOB_IK_output_(i) = desired_leg_q(i) + gain_temp * d_hat(i);
        }
    }

    d_hat_b = d_hat;
    DOB_IK_output_b_ = DOB_IK_output_;
    // MJ_graph << DOB_IK_output_(1) << "," << desired_leg_q(1) << "," << DOB_IK_output_(2) << "," << desired_leg_q(2) << "," << DOB_IK_output_(3) << "," << desired_leg_q(3) << "," << DOB_IK_output_(4) << "," << desired_leg_q(4) << "," << DOB_IK_output_(5) << "," << desired_leg_q(5) << endl;
    // MJ_graph1 << DOB_IK_output_(7) << "," << desired_leg_q(7) << "," << DOB_IK_output_(8) << "," << desired_leg_q(8) << "," << DOB_IK_output_(9) << "," << desired_leg_q(9) << "," << DOB_IK_output_(10) << "," << desired_leg_q(10) << "," << DOB_IK_output_(11) << "," << desired_leg_q(11) << endl;
}

void AvatarController::CP_compen_MJ()
{
    double alpha = 0;
    double F_R = 0, F_L = 0;

    // Tau_R.setZero(); Tau_L.setZero();

    Tau_CP.setZero();

    alpha = (com_float_current_(1) - rfoot_float_current_.translation()(1)) / (lfoot_float_current_.translation()(1) - rfoot_float_current_.translation()(1));

    if (alpha > 1)
    {
        alpha = 1;
    }
    else if (alpha < 0)
    {
        alpha = 0;
    }

    F_R = (1 - alpha) * link_avatar_[COM_id].mass * GRAVITY;
    F_L = alpha * link_avatar_[COM_id].mass * GRAVITY;

    Tau_CP(4) = F_L * del_zmp(0);  // L pitch
    Tau_CP(10) = F_R * del_zmp(0); // R pitch

    Tau_CP(5) = -F_L * del_zmp(1);  // L roll
    Tau_CP(11) = -F_R * del_zmp(1); // R roll
}

void AvatarController::CP_compen_MJ_FT()
{ // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
    double alpha = 0;
    double F_R = 0, F_L = 0;
    double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
    double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;
    double zmp_offset = 0, ZMP_Y_REF_alpha = 0;
    double alpha_new = 0;

    //   zmp_offset = 0.025; // 0.9초
    zmp_offset = 0.02; // 1.1초
    // Preview를 이용한 COM 생성시 ZMP offset을 2cm 안쪽으로 넣었지만, alpha 계산은 2cm 넣으면 안되기 때문에 조정해주는 코드
    // 어떻게 보면 COM, CP 궤적은 ZMP offset이 반영되었고, CP 제어기는 반영안시킨게 안맞는거 같기도함
    if (walking_tick_mj > t_temp_)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha = ZMP_Y_REF + zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
            else
            {
                ZMP_Y_REF_alpha = ZMP_Y_REF - zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha = ZMP_Y_REF + zmp_offset;
            }
            else
            {
                ZMP_Y_REF_alpha = ZMP_Y_REF - zmp_offset;
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha = ZMP_Y_REF + zmp_offset - zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
            else
            {
                ZMP_Y_REF_alpha = ZMP_Y_REF - zmp_offset + zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
        }
        else
        {
            ZMP_Y_REF_alpha = ZMP_Y_REF;
        }
    }
    else
    {
        ZMP_Y_REF_alpha = ZMP_Y_REF;
    }

    ////////////////////////
    //   double A = 0, B = 0, d = 0, X1 = 0, Y1 = 0, e_2 = 0, L = 0, l = 0;
    //   A =  (lfoot_support_current_.translation()(0) - rfoot_support_current_.translation()(0));
    //   B = -(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    //   X1 = ZMP_Y_REF_alpha + 0*del_zmp(1) - rfoot_support_current_.translation()(1);
    //   Y1 = ZMP_X_REF + 0*del_zmp(0) - rfoot_support_current_.translation()(0);
    //   L = sqrt(A*A + B*B);
    //   d = abs(A*X1 + B*Y1) / L;
    //   e_2 = X1*X1 + Y1*Y1;
    //   l = sqrt(e_2 - d*d);
    //   alpha_new = l/L;
    alpha = (ZMP_Y_REF_alpha + 0 * del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    // cout << alpha << "," << ZMP_Y_REF << "," << rfoot_support_current_.translation()(1) << "," << lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1) << endl;
    //  로봇에서 구현할때 alpha가 0~1로 나오는지 확인, ZMP offset 0으로 해야됨.
    if (alpha > 1)
    {
        alpha = 1;
    } // 왼발 지지때 alpha = 1
    else if (alpha < 0)
    {
        alpha = 0;
    }
    //   if(alpha_new > 1)
    //   { alpha_new = 1; } // 왼발 지지때 alpha = 1
    //   else if(alpha_new < 0)
    //   { alpha_new = 0; }

    double real_robot_mass_offset_ = 42; // 42

    F_R = -(1 - alpha) * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ + 15);
    F_L = -alpha * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ - 15); // alpha가 0~1이 아니면 desired force가 로봇 무게보다 계속 작게나와서 지면 반발력을 줄이기위해 다리길이를 줄임.

    if (walking_tick_mj == 0)
    {
        F_F_input = 0.0;
        F_T_L_x_input = 0.0;
        F_T_R_x_input = 0.0;
        F_T_L_y_input = 0.0;
        F_T_R_y_input = 0.0;
    }

    //////////// Force
    F_F_input_dot = 0.0001 * ((l_ft_LPF(2) - r_ft_LPF(2)) - (F_L - F_R)) - 3.0 * F_F_input; // 0.9초 0.0001/ 3.0

    F_F_input = F_F_input + F_F_input_dot * del_t;

    if (F_F_input >= 0.02) // 1.1초 0.02
    {
        F_F_input = 0.02;
    }
    else if (F_F_input <= -0.02)
    {
        F_F_input = -0.02;
    }
    //   if(F_F_input >= 0.01) // 0.9초 0.01
    //   {
    //     F_F_input = 0.01;
    //   }
    //   else if(F_F_input <= -0.01)
    //   {
    //     F_F_input = -0.01;
    //   }
    // MJ_graph << ZMP_Y_REF << "," << ZMP_Y_REF_alpha << "," << alpha << "," << F_L << "," << F_R << "," << F_F_input << "," << l_ft_LPF(2) << "," << r_ft_LPF(2) <<  endl;

    //////////// Torque
    // X,Y 축을 X,Y 방향으로 헷갈렸었고, 위치 명령을 발목 IK각도에 바로 넣었었음.
    Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha + del_zmp(1))) * F_L);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_L);

    if (Tau_all_x > 100)
    {
        Tau_all_x = 100;
    }
    else if (Tau_all_x < -100)
    {
        Tau_all_x = -100;
    }

    if (Tau_all_y > 100)
    {
        Tau_all_y = 100;
    }
    else if (Tau_all_y < -100)
    {
        Tau_all_y = -100;
    }

    Tau_R_x = (1 - alpha) * Tau_all_x;
    Tau_L_x = (alpha)*Tau_all_x;

    Tau_L_y = -alpha * Tau_all_y;
    Tau_R_y = -(1 - alpha) * Tau_all_y;

    double Kr_roll = 0.0, Kl_roll = 0.0;
    double Kr_pitch = 0.0, Kl_pitch = 0.0;

    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        Kr_roll = 30.0; // 20
        Kl_roll = 30.0;
        Kr_pitch = 30.0;
        Kl_pitch = 30.0;
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        if (alpha == 1) // 왼발 지지
        {
            Kl_roll = 30.0;
            Kr_roll = 50.0;
            Kl_pitch = 30.0;
            Kr_pitch = 50.0;
        }
        if (alpha == 0) // 오른발 지지
        {
            Kl_roll = 50.0;
            Kr_roll = 30.0;
            Kl_pitch = 50.0;
            Kr_pitch = 30.0;
        }
    }
    else
    {
        Kr_roll = 30.0;
        Kl_roll = 30.0;
        Kr_pitch = 30.0;
        Kl_pitch = 30.0;
    }

    // Roll 방향 (-0.02/-30 0.9초)
    F_T_L_x_input_dot = -0.01 * (Tau_L_x - l_ft_LPF(3)) - Kl_roll * F_T_L_x_input;
    //   F_T_L_x_input_dot = -0.02*(Tau_L_x - l_ft_LPF(3)) - 30.0*F_T_L_x_input;
    F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
    // F_T_L_x_input = 0;
    F_T_R_x_input_dot = -0.01 * (Tau_R_x - r_ft_LPF(3)) - Kr_roll * F_T_R_x_input;
    //   F_T_R_x_input_dot = -0.02*(Tau_R_x - r_ft_LPF(3)) - 30.0*F_T_R_x_input;
    F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
    // F_T_R_x_input = 0;

    // Pitch 방향  (0.005/-30 0.9초)
    F_T_L_y_input_dot = 0.005 * (Tau_L_y - l_ft_LPF(4)) - Kl_pitch * F_T_L_y_input;
    //   F_T_L_y_input_dot = 0.005*(Tau_L_y - l_ft_LPF(4)) - 30.0*F_T_L_y_input;
    F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
    // F_T_L_y_input = 0;
    F_T_R_y_input_dot = 0.005 * (Tau_R_y - r_ft_LPF(4)) - Kr_pitch * F_T_R_y_input;
    //   F_T_R_y_input_dot = 0.005*(Tau_R_y - r_ft_LPF(4)) - 30.0*F_T_R_y_input;
    F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;
    // F_T_R_y_input = 0;
    // MJ_graph << l_ft_LPF(2) - r_ft_LPF(2) << "," <<  (F_L - F_R) << "," << F_F_input << endl;
    // MJ_graph << F_T_L_x_input << "," << F_T_R_x_input << "," <<  F_T_L_y_input << "," <<  F_T_R_y_input << "," << F_F_input << "," << cp_measured_(1) << "," << cp_desired_(1) << endl;
    if (F_T_L_x_input >= 0.1) // 5 deg limit
    {
        F_T_L_x_input = 0.1;
    }
    else if (F_T_L_x_input < -0.1)
    {
        F_T_L_x_input = -0.1;
    }

    if (F_T_R_x_input >= 0.1) // 5 deg limit
    {
        F_T_R_x_input = 0.1;
    }
    else if (F_T_R_x_input < -0.1)
    {
        F_T_R_x_input = -0.1;
    }

    if (F_T_L_y_input >= 0.1) // 5 deg limit
    {
        F_T_L_y_input = 0.1;
    }
    else if (F_T_L_y_input < -0.1)
    {
        F_T_L_y_input = -0.1;
    }

    if (F_T_R_y_input >= 0.1) // 5 deg limit
    {
        F_T_R_y_input = 0.1;
    }
    else if (F_T_R_y_input < -0.1)
    {
        F_T_R_y_input = -0.1;
    }

    // MJ_joint1 << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << "," << ZMP_Y_REF << "," << ZMP_Y_REF_alpha << "," << l_ft_LPF(2) << "," << r_ft_LPF(2) << endl;
    // cout << F_T_R_x_input*180/3.141592 << "," << F_T_L_x_input*180/3.141592 << "," << Tau_R_x << "," << Tau_L_x << "," << r_ft_(3) << "," << l_ft_(3) << endl;
    // MJ_graph << alpha << "," << alpha_new << endl;
    // MJ_graph << rfoot_support_current_.translation()(1) << "," << lfoot_support_current_.translation()(1) << "," << ZMP_Y_REF << "," << Tau_R_y << "," << Tau_L_y << endl;
    // MJ_graph << Tau_L_x << "," << Tau_R_x << "," << l_ft_LPF(3) << "," << r_ft_LPF(3) << "," << Tau_L_y << "," << Tau_R_y << "," << l_ft_LPF(4) << "," << r_ft_LPF(4) << "," << F_F_input << endl;
    // MJ_graph << ZMP_Y_REF << "," << alpha << "," << ZMP_Y_REF_alpha << endl;
    // MJ_graph << Tau_all_y << "," << Tau_L_y << "," << Tau_R_y << "," << l_ft_(4) << "," << r_ft_(4) << "," << cp_measured_(0) << "," << cp_desired_(0) << endl;
}

void AvatarController::updateInitialStateJoy()
{
    if (walking_tick_mj == 0)
    {
        calculateFootStepTotal_MJoy(); // joystick&pedal Footstep
        joy_enable_ = true;
        std::cout << "step_length : " << joystick_input_(0) << " trigger(z) : " << joystick_input_(1) << " theta : " << joystick_input_(2) << std::endl;

        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); // ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        // pelv_float_init_.translation()(0) += 0.11;

        pelv_float_init_.translation()(0) = 0;
        pelv_float_init_.translation()(1) = 0;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        lfoot_float_init_.translation()(0) = 0;
        lfoot_float_init_.translation()(1) = 0.1225;

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        rfoot_float_init_.translation()(0) = 0;
        rfoot_float_init_.translation()(1) = -0.1225;

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        com_float_init_(0) = 0;
        com_float_init_(1) = 0;

        if (aa == 0)
        {
            lfoot_float_init_.translation()(1) = 0.1025;
            rfoot_float_init_.translation()(1) = -0.1025;
            // t_temp_ = 4.0*hz_
            if (walking_enable_ == true)
            {
                aa = 1;
            }
        }
        cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;

        Eigen::Isometry3d ref_frame;

        if (foot_step_(0, 6) == 0) // right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(0, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        supportfoot_float_init_.setZero();
        swingfoot_float_init_.setZero();

        if (foot_step_(0, 6) == 1) // left suppport foot
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }
        else
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }

        pelv_support_start_ = pelv_support_init_;
        total_step_num_ = foot_step_.col(1).size();

        xi_mj_ = com_support_init_(0); // preview parameter
        yi_mj_ = com_support_init_(1);
        zc_mj_ = com_support_init_(2);
    }
    else if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {
        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); // ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        // pelv_float_init_.translation()(0) += 0.11;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        Eigen::Isometry3d ref_frame;

        if (foot_step_(current_step_num_, 6) == 0) // right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
    }
    if (walking_tick_mj == t_start_)
    {
        if (joy_input_enable_ == true)
        {
            joystick_input_(0) = (joystick_input(0) + 1) / 2; // FW
            joystick_input_(3) = (joystick_input(3) + 1) / 2; // BW
            // joystick_input_(1) = joystick_input(1);
            joystick_input_(2) = -joystick_input(2);
            joystick_input_(1) = joystick_input_(0) + abs(joystick_input_(2)) + joystick_input_(3);
        }

        if (joystick_input_(1) > 0)
        {
            calculateFootStepTotal_MJoy();
            total_step_num_ = foot_step_.col(1).size();
            joy_enable_ = true;
            std::cout << "step_length : " << joystick_input_(0) << " trigger(z) : " << joystick_input_(1) << " theta : " << joystick_input_(2) << std::endl;
        }
        else if (joy_enable_ == true)
        {
            calculateFootStepTotal_MJoy_End();
            total_step_num_ = foot_step_.col(1).size();
            joy_enable_ = false;
            joy_input_enable_ = false;
            joystick_input_(1) = -1.0;
        }
    }
}

void AvatarController::calculateFootStepTotal_MJoy()
{
    double width = 0.1225;
    double length = 0.00;
    double lengthb = 0.00;
    double theta = 10 * DEG2RAD;
    double width_buffer = 0.0;
    double temp;
    int temp2;
    int index = 3;

    double length_total = joystick_input_(0) * length - joystick_input_(3) * lengthb;

    joy_index_++;
    foot_step_.resize(joy_index_ + index, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(joy_index_ + index, 7);
    foot_step_support_frame_.setZero();

    if (walking_tick_mj != 0)
    {
        // for(int i=0; i<joy_index_ + 2 ; i++){
        for (int i = 0; i < foot_step_joy_temp_.col(1).size(); i++)
        {
            foot_step_(i, 0) = foot_step_joy_temp_(i, 0);
            foot_step_(i, 1) = foot_step_joy_temp_(i, 1);
            foot_step_(i, 5) = foot_step_joy_temp_(i, 5);
            foot_step_(i, 6) = foot_step_joy_temp_(i, 6);
        }
    }

    foot_step_(1, 6) = 0;
    if (aa == 0)
    {
        width_buffer = 0.01;
        joystick_input_(0) = 0;
        joystick_input_(2) = 0;
        joystick_input_(3) = 0;
    }

    if (joy_index_ < 3)
    {
        temp = 1;
        temp2 = 0;

        foot_step_(0, 5) = temp2 * joystick_input_(2) * theta;                                                            // 0.0;
        foot_step_(0, 0) = (width - width_buffer) * sin(foot_step_(0, 5)) + temp2 * length_total * cos(foot_step_(0, 5)); // 0.0;
        foot_step_(0, 1) = -(width - width_buffer) * cos(foot_step_(0, 5)) + temp2 * length_total * sin(foot_step_(0, 5));
        foot_step_(0, 6) = 1.0;
        temp2++;

        foot_step_(1, 5) = temp2 * joystick_input_(2) * theta; // 0.0;
        foot_step_(1, 0) = -width * sin(foot_step_(1, 5)) + temp2 * length_total * cos(foot_step_(1, 5));
        foot_step_(1, 1) = width * cos(foot_step_(1, 5)) + temp2 * length_total * sin(foot_step_(1, 5));
        foot_step_(1, 6) = 0.0;
        temp2++;

        foot_step_(2, 5) = temp2 * joystick_input_(2) * theta;
        foot_step_(2, 0) = width * sin(foot_step_(2, 5)) + temp2 * length_total * cos(foot_step_(2, 5));
        foot_step_(2, 1) = -width * cos(foot_step_(2, 5)) + temp2 * length_total * sin(foot_step_(2, 5));
        foot_step_(2, 6) = 1.0;
        temp2++;

        foot_step_(3, 5) = temp2 * joystick_input_(2) * theta;
        foot_step_(3, 0) = -width * sin(foot_step_(3, 5)) + temp2 * length_total * cos(foot_step_(3, 5));
        foot_step_(3, 1) = width * cos(foot_step_(3, 5)) + temp2 * length_total * sin(foot_step_(3, 5));
        foot_step_(3, 6) = 0.0;
    }
    else
    {
        if (foot_step_(joy_index_, 6) == 1)
            temp = 1;
        else if (foot_step_(joy_index_, 6) == 0)
            temp = -1;

        for (int i = -1; i < index; i++)
        {
            temp *= -1;

            foot_step_(joy_index_ + i, 5) = foot_step_(joy_index_ + i - 1, 5) + joystick_input_(2) * theta;
            foot_step_(joy_index_ + i, 0) = foot_step_(joy_index_ + i - 1, 0) + temp * width * (sin(foot_step_(joy_index_ + i - 1, 5)) + sin(foot_step_(joy_index_ + i, 5))) + length_total * cos(foot_step_(joy_index_ + i, 5));
            foot_step_(joy_index_ + i, 1) = foot_step_(joy_index_ + i - 1, 1) - temp * width * (cos(foot_step_(joy_index_ + i - 1, 5)) + cos(foot_step_(joy_index_ + i, 5))) + length_total * sin(foot_step_(joy_index_ + i, 5));
            foot_step_(joy_index_ + i, 6) = 0.5 + 0.5 * temp;
        }
    }

    foot_step_joy_temp_.resize(joy_index_ + index, 7);
    foot_step_joy_temp_.setZero();
    foot_step_joy_temp_ = foot_step_;
}

void AvatarController::calculateFootStepTotal_MJoy_End()
{
    double width = 0.1225;
    double temp;
    int index = 1;

    joy_index_++;
    foot_step_.resize(joy_index_ + index, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(joy_index_ + index, 7);
    foot_step_support_frame_.setZero();

    if (walking_tick_mj != 0)
    {
        for (int i = 0; i < joy_index_ + 1; i++)
        {
            foot_step_(i, 0) = foot_step_joy_temp_(i, 0);
            foot_step_(i, 1) = foot_step_joy_temp_(i, 1);
            foot_step_(i, 5) = foot_step_joy_temp_(i, 5);
            foot_step_(i, 6) = foot_step_joy_temp_(i, 6);
        }
    }

    if (foot_step_(joy_index_, 6) == 1)
        temp = 1;
    else if (foot_step_(joy_index_, 6) == 0)
        temp = -1;

    for (int i = -1; i < index; i++)
    {
        temp *= -1;
        foot_step_(joy_index_ + i, 5) = foot_step_(joy_index_ + i - 1, 5);
        foot_step_(joy_index_ + i, 0) = foot_step_(joy_index_ + i - 1, 0) + temp * 2 * width * sin(foot_step_(joy_index_ + i, 5));
        foot_step_(joy_index_ + i, 1) = foot_step_(joy_index_ + i - 1, 1) - temp * 2 * width * cos(foot_step_(joy_index_ + i, 5));
        foot_step_(joy_index_ + i, 6) = 0.5 + 0.5 * temp;
    }

    cout << "-----footstep-position-----" << endl;
    for (int i = 0; i < joy_index_ + index; i++)
    {
        cout << i << " : " << foot_step_(i, 6) << " : " << foot_step_(i, 5) << " : " << foot_step_(i, 0) << " , " << foot_step_(i, 1) << endl;
    }
    cout << "-----footstep-planning-----" << endl;
}
void AvatarController::updateNextStepTimeJoy()
{
    if (walking_tick_mj == t_last_)
    {
        if (current_step_num_ != total_step_num_ - 1)
        {
            t_start_ = t_last_ + 1;
            t_start_real_ = t_start_ + t_rest_init_;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;
        }
    }

    walking_tick_mj++;

    if (current_step_num_ == total_step_num_ - 1 && walking_tick_mj >= t_last_ + t_total_ + 1)
    {
        walking_enable_ = false;
        walking_tick_mj = 0;
        joy_index_ = 0;
        ref_q_ = rd_.q_;
        for (int i = 0; i < 12; i++)
        {
            Initial_ref_q_(i) = ref_q_(i);
        }
        cout << "            end" << endl;
        cout << "___________________________" << endl;
        joy_input_enable_ = true;
        walking_end_flag = 0;
    }
}

void AvatarController::computePlanner()
{
}

void AvatarController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}