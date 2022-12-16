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

ofstream MJ_graph("/home/dg/data/walking_baseline/MJ_graph.txt");
ofstream MJ_graph1("/home/dg/data/walking_baseline/MJ_graph1.txt");
ofstream MJ_graph2("/home/dg/data/walking_baseline/MJ_graph2.txt");
ofstream MJ_q_("/home/dg/data/walking_baseline/MJ_q_.txt");
ofstream MJ_q_dot_("/home/dg/data/walking_baseline/MJ_q_dot_.txt");
ofstream MJ_CAM_("/home/dg/data/walking_baseline/MJ_CAM_.txt"); 
ofstream MJ_CP_ZMP("/home/dg/data/walking_baseline/MJ_CP_ZMP.txt");

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
    mujoco_ext_force_apply_pub = nh_avatar_.advertise<std_msgs::Float32MultiArray>("/tocabi_avatar/applied_ext_force", 10);
    mujoco_applied_ext_force_.data.resize(7);

    pedal_command = nh_avatar_.subscribe("/tocabi/pedalcommand", 100, &AvatarController::PedalCommandCallback, this); //MJ
    joystick_command = nh_avatar_.subscribe("joy", 100, &AvatarController::JoystickCommandCallback, this); //DG

    joystick_tocabi_command_pub = nh_avatar_.advertise<std_msgs::String>("/tocabi/command", 5);
    //opto_ftsensor_sub = nh_avatar_.subscribe("/atiforce/ftsensor", 100, &AvatarController::OptoforceFTCallback, this); // real robot experiment

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
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_MJ_, true, false);

    for (int i = 0; i < FILE_CNT; i++)
    {
        file[i].open(FILE_NAMES[i]);
    }

    setGains();
    // setNeuralNetworks(); // deleted by MJ/1209
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
        joint_vel_limit_l_(i) = -M_PI * 2;
        joint_vel_limit_h_(i) = M_PI * 2;
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
    loadScaNetwork(larm_upperbody_sca_mlp_, "/home/dyros_rm/catkin_ws/src/tocabi_avatar/sca_mlp/larm_upperbody/");
    //////////////////////////////////////////////////////////////////////////////

    ///// Between Right Arm and Upperbody & Head Collision Detection Network /////
    n_hidden << 120, 100, 80, 60, 40, 20;
    q_to_input_mapping_vector << 12, 13, 14, 25, 26, 27, 28, 29, 30, 31, 32, 23, 24;
    initializeScaMlp(rarm_upperbody_sca_mlp_, 13, 2, n_hidden, q_to_input_mapping_vector);
    loadScaNetwork(rarm_upperbody_sca_mlp_, "/home/dyros_rm/catkin_ws/src/tocabi_avatar/sca_mlp/rarm_upperbody/");
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
        if (initial_flag == 0)
        {
            Joint_gain_set_MJ();
            walking_enable_ = true;
            // Initial pose
            ref_q_ = rd_.q_;
            for (int i = 0; i < 12; i++)
            {
                Initial_ref_q_(i) = ref_q_(i);
            }
            
            // Saving for initial upper body pose
            // edited by MJ (Initial upper body trajectory generation for CAM control /220110)
            CAM_upper_init_q_.setZero();
            Initial_ref_upper_q_.setZero();
            for (int i = 12; i < MODEL_DOF; i++)
            {
                Initial_ref_upper_q_(i) = ref_q_(i);
            }
            // CAM_upper_init_q_(16) = +10.0 * DEG2RAD;
            // CAM_upper_init_q_(26) = -10.0 * DEG2RAD;
            // CAM_upper_init_q_(17) = +70.0 * DEG2RAD; // Rolling dist +70.0 deg
            // CAM_upper_init_q_(27) = -70.0 * DEG2RAD; // Rolling dist -70.0 deg
            // CAM_upper_init_q_(19) = -90.0 * DEG2RAD; //-90.0
            // CAM_upper_init_q_(29) = +90.0 * DEG2RAD; //+90.0            
            
            q_prev_MJ_ = rd_.q_;
            desired_q_slow_ = rd_.q_;
            desired_q_fast_ = rd_.q_;
            desired_q_dot_.setZero();
            desired_q_dot_fast_.setZero();

            walking_tick_mj = 0;
            walking_end_flag = 0;
            parameterSetting();
            initWalkingParameter();
            cout << "computeslow mode = 10 is initialized" << endl;
            cout << "time: "<<rd_.control_time_ << endl; //dg add

            WBC::SetContact(rd_, 1, 1);
            Gravity_MJ_ = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);
            //Gravity_MJ_.setZero();
            atb_grav_update_ = false;
            initial_flag = 1;
        }

        if (atb_grav_update_ == false)
        {
            atb_grav_update_ = true;
            Gravity_MJ_fast_ = Gravity_MJ_;
            atb_grav_update_ = false;
        }  

        // edited by MJ (Initial upper body trajectory generation for CAM control /220110)
        // if(initial_tick_mj <= 2.0 * hz_)
        // {
        //     ref_q_(16) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(16), CAM_upper_init_q_(16), 0.0, 0.0);
        //     ref_q_(17) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(17), CAM_upper_init_q_(17), 0.0, 0.0);
        //     ref_q_(19) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(19), CAM_upper_init_q_(19), 0.0, 0.0);
        //     ref_q_(26) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(26), CAM_upper_init_q_(26), 0.0, 0.0);
        //     ref_q_(27) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(27), CAM_upper_init_q_(27), 0.0, 0.0);
        //     ref_q_(29) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(29), CAM_upper_init_q_(29), 0.0, 0.0);

        //     initial_tick_mj ++;         
        // }
        
        for (int i = 0; i < MODEL_DOF; i++)
        {
            rd_.torque_desired(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
        }        

        //l_ft_ = rd_.LF_FT;  //Fz: generated by robot,  Fx:   , Fy:    Mx: ,   My: , Mz:
        //r_ft_ = rd_.RF_FT;

        //if( int(rd_.control_time_*2000)%1000 == 0)
        //    cout << "l_ft: "<<l_ft_.transpose() << "\n r_ft_: " << r_ft_.transpose() << endl; 
    }
    else if (rd_.tc_.mode == 11)
    {
        ////////////////////////////////////////////////////////////////////////////
        /////////////////// Biped Walking Controller made by MJ ////////////////////
        ////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////
        if (walking_enable_ == true)
        {
            if (walking_tick_mj == 0)
            {
                parameterSetting();
                initial_flag = 0;

                atb_grav_update_ = false;
                atb_desired_q_update_ = false;
                atb_walking_traj_update_ = false;
                torque_upper_fast_.setZero();
                torque_upper_fast_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);
                torque_upper_.setZero();
                torque_upper_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);

                cout << "parameter setting OK" << endl;
                cout << "mode = 11" << endl;
            }
            
            updateInitialState();
            getRobotState();
            floatToSupportFootstep();

            if (current_step_num_ < total_step_num_)
            {            
                getZmpTrajectory();
                // getComTrajectory(); // 조현민꺼에서 프리뷰에서 CP 궤적을 생성하기 때문에 필요  
                getComTrajectory_mpc(); // working with thread3 (MPC thread)   
                CPMPC_bolt_Controller_MJ(); 
                // BoltController_MJ(); // Stepping Controller for DCM eos                
                // MJDG CMP control
                CentroidalMomentCalculator(); // working with computefast() (CAM controller)

                // getFootTrajectory();
                getFootTrajectory_stepping(); // working with BoltController_MJ()  
                getPelvTrajectory();
                supportToFloatPattern();
                
                // STEP1: send desired AM to the slow thread
                if (atb_walking_traj_update_ == false)
                {
                    atb_walking_traj_update_ = true;
                    del_ang_momentum_fast_ = del_ang_momentum_;
                    atb_walking_traj_update_ = false;
                }
                
                computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);
                // Compliant_control(q_des_);
                for (int i = 0; i < 12; i++)
                {
                    ref_q_(i) = q_des_(i);
                    // ref_q_(i) = DOB_IK_output_(i);
                }
                //hip_compensator();
                //GravityCalculate_MJ();

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    Gravity_MJ_fast_ = Gravity_MJ_;
                    atb_grav_update_ = false;
                }

                if (walking_tick_mj < 1.0 * hz_)
                {
                    //for leg
                    for (int i = 0; i < 12; i++)
                    {
                        ref_q_(i) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(i), q_des_(i), 0.0, 0.0);
                    }
                    //for waist
                    // ref_q_(13) = Initial_ref_q_(13);
                    // ref_q_(14) = Initial_ref_q_(14);
                    // //for arm
                    // ref_q_(16) = Initial_ref_q_(16);
                    // ref_q_(26) = Initial_ref_q_(26);
                    // ref_q_(17) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(17), 50.0 * DEG2RAD, 0.0, 0.0);  // + direction angle makes the left arm down.
                    // ref_q_(27) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(27), -50.0 * DEG2RAD, 0.0, 0.0); // - direction angle makes the right arm down.
                }
                
                CP_compen_MJ();
                CP_compen_MJ_FT();                

                torque_lower_.setZero();
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + Tau_CP(i) + Gravity_MJ_fast_(i);
                    // 4 (Ankle_pitch_L), 5 (Ankle_roll_L), 10 (Ankle_pitch_R),11 (Ankle_roll_R)
                }
                // printOutTextFile();

                updateNextStepTime();
                q_prev_MJ_ = rd_.q_;
          
                //if((walking_tick_mj >= 5.88*hz_)&&(walking_tick_mj < 6.083*hz_)) 
                // if(current_step_num_ == 4 && (walking_tick_mj >= t_start_ + 0.15*hz_ + 0.6*0.2*hz_)  && (walking_tick_mj < t_start_ + 0.15*hz_ + 0.6*0.2*hz_ + 0.2*hz_))
                // { // -170,175 // -350 7.5 - 7.6 //  67% 
                //     // cout << current_step_num_ << "," << t_start_ << "," << walking_tick_mj << endl;
                //     mujoco_applied_ext_force_.data[0] = -200;//0*312;//-232.0;//x-axis linear force // 
                //     mujoco_applied_ext_force_.data[1] = 0;//+126.0;  //y-axis linear force  
                //     mujoco_applied_ext_force_.data[2] = 0.0;  //z-axis linear force
                //     mujoco_applied_ext_force_.data[3] = 0.0;  //x-axis angular moment
                //     mujoco_applied_ext_force_.data[4] = 0.0;  //y-axis angular moment
                //     mujoco_applied_ext_force_.data[5] = 0.0;  //z-axis angular moment

                //     mujoco_applied_ext_force_.data[6] = 1; //link idx; 1:pelvis

                //     mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);                    
                // } 
                // else
                // {
                //     mujoco_applied_ext_force_.data[0] = 0; //x-axis linear force
                //     mujoco_applied_ext_force_.data[1] = 0; //y-axis linear force
                //     mujoco_applied_ext_force_.data[2] = 0; //z-axis linear force
                //     mujoco_applied_ext_force_.data[3] = 0; //x-axis angular moment
                //     mujoco_applied_ext_force_.data[4] = 0; //y-axis angular moment
                //     mujoco_applied_ext_force_.data[5] = 0; //z-axis angular moment

                //     mujoco_applied_ext_force_.data[6] = 1; //link idx; 1:pelvis

                //     mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);
                // }
            }
        }
        else
        {
            if (walking_end_flag == 0)
            {
                cout << "walking finish" << endl;
                walking_end_flag = 1;
                initial_flag = 0;
            }

            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_MJ_fast_ = Gravity_MJ_;
                atb_grav_update_ = false;
            }

            torque_lower_.setZero();
            for (int i = 0; i < 12; i++)
            {
                torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
            }
        }
        
        /////////////////////////////////////////////////////////////////////////////////////////
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_fast_ = desired_q_slow_;
            desired_q_dot_fast_ = desired_q_dot_slow_;
            atb_desired_q_update_ = false;
        }

        torque_upper_.setZero();
        for (int i = 12; i < MODEL_DOF; i++)
        {
            torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - rd_.q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - rd_.q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            //Is there any problem if i write the code as below? - myeongju-
            // torque_upper_(i) = (Kp(i) * (ref_q_(i) - del_cmm_q_(i) - rd_.q_(i)) + Kd(i) * (0.0 - rd_.q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            // torque_upper_(i) = torque_upper_(i) * pd_control_mask_(i); // masking for joint pd control
        }

        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = torque_lower_ + torque_upper_;
        ///////////////////////////////////////////////////////////////////////////////
    }
    else if (rd_.tc_.mode == 12)
    {
        if (initial_flag == 0)
        {
            Joint_gain_set_MJ();
            walking_enable_ = false;
            walking_stop_flag_ = true;
            stopping_step_planning_trigger_ = true;

            ref_q_ = rd_.q_;

            for (int i = 0; i < 12; i++)
            {
                Initial_ref_q_(i) = ref_q_(i);
            }
            init_leg_time_ = (double)rd_.control_time_us_/ 1000000.0;
            desired_q_slow_ = rd_.q_;
            desired_q_fast_ = rd_.q_;
            desired_q_dot_.setZero();
            desired_q_dot_fast_.setZero();

            
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            mode_12_count_ = 0;
            joy_input_enable_ = true;

            chair_mode_ = false; /// avatar semifinals //1025

            parameterSetting();
            initWalkingParameter();
            // loadCollisionThreshold("/home/dyros_rm/catkin_ws/src/tocabi_avatar/config/"); // deleted by MJ/1209
            cout << "mode = 12 : Pedal Init" << endl;
            cout << "chair_mode_: " << chair_mode_ << endl;
            // cout << "ref_q_: "<<ref_q_.transpose() << endl;
            WBC::SetContact(rd_, 1, 1);
            Gravity_MJ_ = WBC::GravityCompensationTorque(rd_);
            atb_grav_update_ = false;
            initial_flag = 1;
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

        // mode_12_count_ ++;

        // if( (initial_flag == 2) && (mode_12_count_ > 1000) )
        // {
        //     rd_.tc_.mode = 13;
        //     rd_.tc_init = true;
        //     mode_12_count_ = 0;
        // }
    }
    else if (rd_.tc_.mode == 13) //2KHZ STRICT
    {
        getJoystickCommand();

        if (walking_enable_ == true)
        {   
            if (walking_tick_mj == 0)
            {
                parameterSetting();
                initial_flag = 0;

                atb_grav_update_ = false;
                atb_desired_q_update_ = false;
                torque_upper_fast_.setZero();
                torque_upper_fast_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);
                torque_upper_.setZero();
                torque_upper_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);

                // pelv_trajectory_support_init_ = pelv_trajectory_support_;
                for (int i = 0; i < 12; i++)
                {
                    Initial_ref_q_(i) = ref_q_(i);
                }

                cout << "\n\n\n\n"
                     << endl;
                cout << "___________________________ " << endl;
                cout << "\n           Start " << endl;
                cout << "parameter setting OK" << endl;
                cout << "mode = 13" << endl;
            }

            updateInitialStateJoy();
            getRobotState();
            floatToSupportFootstep();
            if (current_step_num_ < total_step_num_)
            {   
                getZmpTrajectory();
                // getComTrajectory();
                getComTrajectory_mpc(); // working with thread3 (MPC thread)  
                CPMPC_bolt_Controller_MJ();
                CentroidalMomentCalculator();
                // getFootTrajectory();
                getFootTrajectory_stepping(); // working with BoltController_MJ()  
                getPelvTrajectory();
                supportToFloatPattern();

                computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);

                // Compliant_control(q_des_);
                for (int i = 0; i < 12; i++)
                {
                    ref_q_(i) = q_des_(i);
                    // ref_q_(i) = DOB_IK_output_(i);

                    // ref_q_(i) = desired_q_fast_(i);
                    static bool ref_q_nan_flag = false;
                    if (ref_q_(i) != ref_q_(i))
                    {
                        if (ref_q_nan_flag == false)
                        {
                            cout << "WARNING: ref_q(" << i << ") is NAN!!" << endl;
                            cout << "lfoot_trajectory_float_: " << lfoot_trajectory_float_.translation().transpose() << endl;
                            cout << "rfoot_trajectory_float_: " << rfoot_trajectory_float_.translation().transpose() << endl;
                            cout << "pelv_trajectory_float_: " << pelv_trajectory_float_.translation().transpose() << endl;

                            cout << "lfoot_trajectory_float_: " << lfoot_trajectory_float_.linear() << endl;
                            cout << "rfoot_trajectory_float_: " << rfoot_trajectory_float_.linear() << endl;
                            cout << "pelv_trajectory_float_: " << pelv_trajectory_float_.linear() << endl;
                        }

                        ref_q_nan_flag = true;
                    }
                }
                // hip_compensator();

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    Gravity_MJ_fast_ = Gravity_MJ_;
                    atb_grav_update_ = false;
                }

                if (chair_mode_)
                {
                    ref_q_ = Initial_ref_q_;
                }

                if (walking_tick_mj < 1.0 * hz_)
                {
                    for (int i = 0; i < 12; i++)
                    {
                        ref_q_(i) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(i), q_des_(i), 0.0, 0.0);
                    }
                }

                if (!chair_mode_)
                {
                    CP_compen_MJ();
                    CP_compen_MJ_FT();
                }

                torque_lower_.setZero();
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i) + Tau_CP(i);
                    // 4 (Ankle_pitch_L), 5 (Ankle_roll_L), 10 (Ankle_pitch_R),11 (Ankle_roll_R)
                }
                
                // printOutTextFile();
                q_prev_MJ_ = rd_.q_;
                q_dot_virtual_Xd_global_pre_ = q_dot_virtual_Xd_global_;
                q_dot_virtual_Xd_local_pre_ = q_dot_virtual_Xd_local_;

                current_step_num_prev_ = current_step_num_;
                current_support_foot_is_left_prev_ = current_support_foot_is_left_;

                updateNextStepTimeJoy();
            }

            /*if(rd_.avatar_reboot_signal)
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
            }*/
        }
        else
        {
            if (walking_end_flag == 0)
            {
                cout << "com_desired_1: " << com_desired_ << endl;
                parameterSetting(); //Don't delete this!!
                updateInitialStateJoy();
                //updateInitialState();
                getRobotState();
                floatToSupportFootstep();
                getZmpTrajectory();
                getComTrajectory_mpc();
                CPMPC_bolt_Controller_MJ();
                CentroidalMomentCalculator();
                // getComTrajectory();
                // getFootTrajectory();
                getFootTrajectory_stepping(); // working with BoltController_MJ()  

                cout << "walking finish" << endl;
                cout << "com_desired_2: " << com_desired_ << endl;
                for (int i = 0; i < 12; i++)
                {
                    Initial_ref_q_(i) = ref_q_(i);
                    Initial_current_q_(i) = rd_.q_(i);
                }
                pelv_trajectory_support_init_ = pelv_trajectory_support_;
                com_desired_(0) = 0;
                initial_flag = 0;
                init_leg_time_ = (double)rd_.control_time_us_/ 1000000.0;
                walking_end_flag = 1;
                cout << "com_desired_3: " << com_desired_ << endl;
            }

            getRobotState();
            getPelvTrajectory();
            supportToFloatPattern();
            computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);

            for (int i = 0; i < 12; i++)
            {
                ref_q_(i) = q_des_(i);
                // ref_q_(i) = DOB_IK_output_(i);

                // ref_q_(i) = desired_q_fast_(i);
                static bool ref_q_nan_flag = false;
                if (ref_q_(i) != ref_q_(i))
                {
                    if (ref_q_nan_flag == false)
                    {
                        cout << "WARNING: ref_q(" << i << ") is NAN!!" << endl;
                        cout << "lfoot_trajectory_float_: " << lfoot_trajectory_float_.translation().transpose() << endl;
                        cout << "rfoot_trajectory_float_: " << rfoot_trajectory_float_.translation().transpose() << endl;
                        cout << "pelv_trajectory_float_: " << pelv_trajectory_float_.translation().transpose() << endl;

                        cout << "lfoot_trajectory_float_: " << lfoot_trajectory_float_.linear() << endl;
                        cout << "rfoot_trajectory_float_: " << rfoot_trajectory_float_.linear() << endl;
                        cout << "pelv_trajectory_float_: " << pelv_trajectory_float_.linear() << endl;
                    }

                    ref_q_nan_flag = true;
                }
            }

            // hip_compensator();

            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_MJ_fast_ = Gravity_MJ_;
                atb_grav_update_ = false;
            }

            if ((double)rd_.control_time_us_/ 1000000.0 <= init_leg_time_ + 2.0)
            {
                for (int i = 0; i < 12; i++)
                {
                    ref_q_(i) = DyrosMath::cubic( (double)rd_.control_time_us_/ 1000000.0, init_leg_time_, init_leg_time_ + 2.0, Initial_ref_q_(i), q_des_(i), 0.0, 0.0);
                }
            }

            if (chair_mode_)
            {
                for (int i = 0; i < 12; i++)
                {
                    ref_q_(i) = Initial_current_q_(i);
                    Gravity_MJ_fast_(i) = 0;
                }
            }

            torque_lower_.setZero();
            for (int i = 0; i < 12; i++)
            {
                torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////

        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_fast_ = desired_q_slow_;
            desired_q_dot_fast_ = desired_q_dot_slow_;
            atb_desired_q_update_ = false;
        }

        torque_upper_.setZero();
        for (int i = 12; i < MODEL_DOF; i++)
        {
            torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - rd_.q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - rd_.q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            // torque_upper_(i) = Gravity_MJ_fast_(i);
            rd_.q_desired(i) = desired_q_fast_(i);  // for logging
            rd_.q_dot_desired(i) = desired_q_dot_fast_(i);  // for logging
            // torque_upper_(i) = torque_upper_(i) * pd_control_mask_(i); // masking for joint pd control
        }
        // printOutTextFile();
        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = torque_lower_ + torque_upper_;
        ////////////////////////////////////////////////////////////////////////////////

        // printOutTextFile();
    }
    else if (rd_.tc_.mode == 14)
    {

    }
}

void AvatarController::computeFast()
{
    if (rd_.tc_.mode == 10)
    {
        if (initial_flag == 1)
        {
            WBC::SetContact(rd_, 1, 1);
            VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

            if (float_data_collect_mode_ == true)
            {
                Gravity_MJ_local = floatGravityTorque(rd_.q_virtual_);
            }

            if (atb_grav_update_ == false)
            {

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
                
                cout<<"comutefast tc.mode =10 is initialized"<<endl;
            }

            // // MOB-LSTM INFERENCE
            // initializeLegLSTM(left_leg_mob_lstm_);
            // loadLstmWeights(left_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/lstm_tocabi/weights/left_leg/left_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");
            // loadLstmMeanStd(left_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/lstm_tocabi/mean_std/left_leg/left_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");

            // initializeLegLSTM(right_leg_mob_lstm_);
            // loadLstmWeights(right_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/lstm_tocabi/weights/right_leg/right_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");
            // loadLstmMeanStd(right_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/lstm_tocabi/mean_std/right_leg/right_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");

            // // PETER GRU
            // initializeLegGRU(left_leg_peter_gru_, 24, 12, 150);
            // loadGruWeights(left_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/weights/left_leg/left_leg_tocabi_new_data3_jts_lpf_peter/");
            // loadGruMeanStd(left_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/mean_std/left_leg/left_leg_tocabi_new_data3_jts_lpf_peter/");
            
            // initializeLegGRU(right_leg_peter_gru_, 24, 12, 150);
            // loadGruWeights(right_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/weights/right_leg/right_leg_tocabi_new_data3_jts_lpf_peter/");
            // loadGruMeanStd(right_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/mean_std/right_leg/right_leg_tocabi_new_data3_jts_lpf_peter/");


            initial_flag = 2;
        }
    }
    else if (rd_.tc_.mode == 11)
    {
        ////////////////////////////////////////////////////////////////////////////
        /////////////////// Biped Walking Controller made by MJ ////////////////////
        ////////////////////////////////////////////////////////////////////////////
        if (walking_enable_ == true)
        {
            if (current_step_num_ < total_step_num_)
            {
                GravityCalculate_MJ();

                //STEP2: recieve desired AM
                if(walking_tick_mj >= 1)
                {
                    if (atb_walking_traj_update_ == false)
                    {
                        atb_walking_traj_update_ = true;
                        del_ang_momentum_slow_ = del_ang_momentum_fast_;
                        atb_walking_traj_update_ = false;
                    }
                }
            }
        }
        else
        {
            WBC::SetContact(rd_, 1, 1);
            int support_foot;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                support_foot = 1;
            }
            else
            {
                support_foot = 0;
            }
             
            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, support_foot);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////

        if (rd_.tc_init == true)
        {
            initWalkingParameter();
            rd_.tc_init = false;
        }

        //data process//
        getRobotData();
        getProcessedRobotData();

        //motion planing and control//
        motionGenerator();
        //STEP3: Compute q_dot for CAM control
        //computeCAMcontrol_HQP();


        // calculateLstmOutput(left_leg_mob_lstm_);  //20~25us
        // calculateLstmOutput(right_leg_mob_lstm_); //20~25us
        

        // calculateGruOutput(left_leg_peter_gru_);
        // calculateGruOutput(right_leg_peter_gru_);

        // // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // // cout<<"LSTM output calc time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
        // estimated_model_unct_torque_fast_.setZero();
        // estimated_model_unct_torque_fast_.segment(0, 6) = left_leg_mob_lstm_.real_output.segment(0, 6);
        // estimated_model_unct_torque_fast_.segment(6, 6) = right_leg_mob_lstm_.real_output.segment(0, 6);

        // estimated_external_torque_gru_fast_.setZero();
        // estimated_external_torque_gru_fast_.segment(0, 6) = left_leg_peter_gru_.real_output.segment(0, 6);
        // estimated_external_torque_gru_fast_.segment(6, 6) = right_leg_peter_gru_.real_output.segment(0, 6);

        // if (gaussian_mode_ == true)
        // {
        //     // gaussian model
        //     estimated_model_unct_torque_variance_fast_.setZero();
        //     estimated_model_unct_torque_variance_fast_.segment(0, 6) = left_leg_mob_lstm_.real_output.segment(6, 6);
        //     estimated_model_unct_torque_variance_fast_.segment(6, 6) = right_leg_mob_lstm_.real_output.segment(6, 6);
        // }
        // if(left_leg_peter_gru_.gaussian_mode == true)
        // {
        //     estimated_external_torque_variance_gru_fast_.setZero();
        //     estimated_external_torque_variance_gru_fast_.segment(0, 6) = left_leg_peter_gru_.real_output.segment(6, 6);
        //     estimated_external_torque_variance_gru_fast_.segment(6, 6) = right_leg_peter_gru_.real_output.segment(6, 6);
        // }

        // if (left_leg_mob_lstm_.atb_lstm_output_update_ == false)
        // {
        //     left_leg_mob_lstm_.atb_lstm_output_update_ = true;
        //     estimated_model_unct_torque_thread_ = estimated_model_unct_torque_fast_;
        //     estimated_model_unct_torque_variance_thread_ = estimated_model_unct_torque_variance_fast_;
        //     left_leg_mob_lstm_.atb_lstm_output_update_ = false;
        // }

        // if (left_leg_peter_gru_.atb_gru_output_update_ == false)
        // {
        //     left_leg_peter_gru_.atb_gru_output_update_ = true;
        //     estimated_external_torque_gru_thread_ = estimated_external_torque_gru_fast_;
        //     estimated_external_torque_variance_gru_thread_ = estimated_external_torque_variance_gru_fast_;
        //     left_leg_peter_gru_.atb_gru_output_update_ = false;
        // }

        for (int i = 12; i < MODEL_DOF; i++)
        {
            desired_q_(i) = motion_q_(i);
            desired_q_dot_(i) = motion_q_dot_(i); 
        }

        //STEP4: send desired q to the fast thread
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_slow_ = desired_q_;
            desired_q_dot_slow_ = desired_q_dot_;
            atb_desired_q_update_ = false;
        }
        // if (atb_desired_q_update_ == false)
        // {
        //     atb_desired_q_update_ = true;
        //     torque_upper_.setZero();
        //     for (int i = 12; i < MODEL_DOF; i++)
        //     {
        //         torque_upper_(i) = (kp_joint_(i) * (desired_q_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_(i) - current_q_dot_(i)) + 1.0 * Gravity_MJ_(i));
        //         torque_upper_(i) = torque_upper_(i) * pd_control_mask_(i); // masking for joint pd control
        //     }
        //     atb_desired_q_update_ = false;
        // }

        savePreData();

        // printOutTextFile();
    }
    else if (rd_.tc_.mode == 12)
    {
        if (initial_flag == 1)
        {
            WBC::SetContact(rd_, 1, 1);

            VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

            // // MOB-LSTM INFERENCE
            // initializeLegLSTM(left_leg_mob_lstm_);
            // loadLstmWeights(left_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/lstm_tocabi/weights/left_leg/left_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");
            // loadLstmMeanStd(left_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/lstm_tocabi/mean_std/left_leg/left_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");

            // initializeLegLSTM(right_leg_mob_lstm_);
            // loadLstmWeights(right_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/lstm_tocabi/weights/right_leg/right_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");
            // loadLstmMeanStd(right_leg_mob_lstm_, "/home/dyros/catkin_ws/src/tocabi_avatar/lstm_tocabi/mean_std/right_leg/right_leg_tocabi_model_vel_ft_wo_quat_only_ground_data/");

            // // PETER GRU
            // initializeLegGRU(left_leg_peter_gru_, 24, 12, 150);
            // loadGruWeights(left_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/weights/left_leg/left_leg_tocabi_new_data_jts_lpf_peter/");
            // loadGruMeanStd(left_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/mean_std/left_leg/left_leg_tocabi_new_data_jts_lpf_peter/");
            
            // initializeLegGRU(right_leg_peter_gru_, 24, 12, 150);
            // loadGruWeights(right_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/weights/right_leg/right_leg_tocabi_new_data_jts_lpf_peter/");
            // loadGruMeanStd(right_leg_peter_gru_, "/home/dyros/catkin_ws/src/tocabi_avatar/neural_networks/gru_tocabi/mean_std/right_leg/right_leg_tocabi_new_data_jts_lpf_peter/");

            if (atb_grav_update_ == false)
            {

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
        
        if (walking_enable_ == true)
        {   
            if (current_step_num_ < total_step_num_)
            {
                GravityCalculate_MJ(); // 90~160us
            }
        }
        else
        {    
            WBC::SetContact(rd_, 1, 1);
            int support_foot;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                support_foot = 1;
            }
            else
            {
                support_foot = 0;
            } 
            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, support_foot);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }  
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
        // calculateScaMlpOutput(larm_upperbody_sca_mlp_);
        // calculateScaMlpOutput(rarm_upperbody_sca_mlp_);
        // calculateScaMlpOutput(btw_arms_sca_mlp_);
        // avatar mode pedal
        avatarModeStateMachine();

        //motion planing and control//
        motionGenerator(); // 140~240us(HQPIK)
        //STEP3: Compute q_dot for CAM control
        //computeCAMcontrol_HQP();

        // calculateLstmOutput(left_leg_mob_lstm_);  //20~25us
        // calculateLstmOutput(right_leg_mob_lstm_); //20~25us
        // // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // // cout<<"LSTM output calc time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
        // estimated_model_unct_torque_fast_.setZero();
        // estimated_model_unct_torque_fast_.segment(0, 6) = left_leg_mob_lstm_.real_output.segment(0, 6);
        // estimated_model_unct_torque_fast_.segment(6, 6) = right_leg_mob_lstm_.real_output.segment(0, 6);

        // if (gaussian_mode_ == true)
        // {
        //     // gaussian model
        //     estimated_model_unct_torque_variance_fast_.setZero();
        //     estimated_model_unct_torque_variance_fast_.segment(0, 6) = left_leg_mob_lstm_.real_output.segment(6, 6);
        //     estimated_model_unct_torque_variance_fast_.segment(6, 6) = right_leg_mob_lstm_.real_output.segment(6, 6);
        // }

        // if (left_leg_mob_lstm_.atb_lstm_output_update_ == false)
        // {
        //     left_leg_mob_lstm_.atb_lstm_output_update_ = true;
        //     estimated_model_unct_torque_thread_ = estimated_model_unct_torque_fast_;
        //     estimated_model_unct_torque_variance_thread_ = estimated_model_unct_torque_variance_fast_;
        //     left_leg_mob_lstm_.atb_lstm_output_update_ = false;
        // }

        for (int i = 12; i < MODEL_DOF; i++)
        {
            desired_q_(i) = motion_q_(i);
            desired_q_dot_(i) = motion_q_dot_(i); 
        }

        //STEP4: send desired q to the fast thread
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_slow_ = desired_q_;
            desired_q_dot_slow_ = desired_q_dot_;
            atb_desired_q_update_ = false;
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
    thread3GetDataFromThread1();
    comGenerator_MPC_wieber(50.0, 1.0/50.0, 2.5, 2000/50.0); // Hz, T, Preview window
    cpcontroller_MPC_MJDG(50.0, 1.5); // Hz, Preview window
    // comGenerator_MPC_joe(50.0, 1.0/50.0, 1.5, 2000/50.0); // Hz, T, Preview window  
    thread3SendDataToThread1();
}
void AvatarController::thread3GetDataFromThread1()
{
    if(atb_thread3_input_data_update_ == false)  
    {
        atb_thread3_input_data_update_ = true;
        walking_tick_mj_mpc_ = walking_tick_mj_thread_;
        t_total_mpc_ = t_total_thread_;
        t_rest_init_mpc_ = t_rest_init_thread_;
        t_rest_last_mpc_ = t_rest_last_thread_;
        current_step_num_mpc_ = current_step_num_thread_;
        total_step_num_mpc_ = total_step_num_thread_; // only used in CP-MPC
        zmp_start_time_mj_mpc_ = zmp_start_time_mj_thread_;
        ref_zmp_mpc_ = ref_zmp_thread_;        
        ref_zmp_wo_offset_mpc_ = ref_zmp_wo_offset_thread_; // only used in CP-MPC   
        
        cp_measured_mpc_ = cp_measured_thread_;
        current_support_foot_is_left_mpc_ = current_support_foot_is_left_thread_;
        foot_step_support_frame_mpc_ = foot_step_support_frame_thread_;

        x_hat_ = x_hat_thread2_;
        y_hat_ = y_hat_thread2_;
        x_hat_p_ = x_hat_p_thread2_;
        y_hat_p_ = y_hat_p_thread2_;

        if( CP_MPC_first_loop_ && (current_support_foot_is_left_mpc_prev_!= current_support_foot_is_left_mpc_))
        {
            // update once when the support foot is changed
            cpmpc_deszmp_x_(0) = cpmpc_des_zmp_x_thread2_; // To apply step change desired ZMP (computeslow) for gradient vector (thread 3/MPC)       
            cpmpc_deszmp_y_(0) = cpmpc_des_zmp_y_thread2_; 
        }

        atb_thread3_input_data_update_ = false;
    }
}
void AvatarController::comGenerator_MPC_wieber(double MPC_freq, double T, double preview_window, int MPC_synchro_hz_)
{   //https://doi.org/10.1163/016918610X493552
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    
    int N = preview_window * MPC_freq; // N step horizon (2.7s x 50)
    int mpc_tick = walking_tick_mj_mpc_ - zmp_start_time_mj_mpc_;
    static int MPC_first_loop = 0;
    int state_num_ = 3;
    Eigen::MatrixXd O_N;
    Eigen::MatrixXd I_N(N,N);
    O_N.setZero(N,N); // N x N Zero matrix
    I_N.setIdentity(); // N x N Identity matrix
    //int MPC_synchro_hz_ = 40; // 40 = Control freq (2000) / MPC_freq (50)

    if(MPC_first_loop == 0)
    {
        // Define State matrix
        A_mpc_(0, 0) = 1.0;
        A_mpc_(0, 1) = T;
        A_mpc_(0, 2) = T * T * 0.5;
        A_mpc_(1, 0) = 0;
        A_mpc_(1, 1) = 1.0;
        A_mpc_(1, 2) = T;
        A_mpc_(2, 0) = 0;
        A_mpc_(2, 1) = 0;
        A_mpc_(2, 2) = 1.0;
        // Define Input matrix
        B_mpc_(0) = T * T * T / 6;
        B_mpc_(1) = T * T / 2;
        B_mpc_(2) = T;
        // Define Output matrix
        C_mpc_transpose_(0) = 1;
        C_mpc_transpose_(1) = 0;
        C_mpc_transpose_(2) = -0.71 / 9.81;                 
                
        W1_mpc_ = 0.000001; // The term alpha in wieber's paper
        W2_mpc_ = 1.0; // The term gamma in wieber's paper

        P_ps_mpc_.setZero(N,state_num_);
        P_pu_mpc_.setZero(N,N);
        P_vs_mpc_.setZero(N,state_num_);
        P_vu_mpc_.setZero(N,N);
        P_zs_mpc_.setZero(N,state_num_);
        P_zu_mpc_.setZero(N,N);

        x_com_pos_recur_.setZero(N);
        x_com_vel_recur_.setZero(N);
        x_zmp_recur_.setZero(N);

        y_com_pos_recur_.setZero(N);
        y_com_vel_recur_.setZero(N);
        y_zmp_recur_.setZero(N);

        for(int i = 0; i < N; i++)
        {   
            // Define reculsive state matrix for MPC P_zs (N x 3)
            P_ps_mpc_(i,0) = 1;
            P_ps_mpc_(i,1) = (i+1)*T;
            P_ps_mpc_(i,2) = ((i+1)*(i+1)*T*T)*0.5;

            P_vs_mpc_(i,0) = 0;
            P_vs_mpc_(i,1) = 1;
            P_vs_mpc_(i,2) = (i+1)*T;

            P_zs_mpc_(i,0) = 1;
            P_zs_mpc_(i,1) = (i+1)*T;
            P_zs_mpc_(i,2) = ((i+1)*(i+1)*T*T)*0.5 - 0.71/9.81;
            
            // Define reculsive input matrix for MPC P_zu (N x N / Lower Triangular Toeplitz matrix)
            for(int j = 0; j < N; j++)
            {
                if(j >= i)
                {
                    P_pu_mpc_(j,i) = (1 + 3*(j-i) + 3*(j-i)*(j-i))*(T*T*T)/6;
                    P_vu_mpc_(j,i) = (1 + 2*(j-i))*(T*T)/2;
                    P_zu_mpc_(j,i) = (1 + 3*(j-i) + 3*(j-i)*(j-i))*(T*T*T)/6 - T*0.71/9.81;
                }
            }
        }        

        Q_prime_.setZero(N,N);
        Q_prime_ = W1_mpc_*I_N + W2_mpc_*P_zu_mpc_.transpose()*P_zu_mpc_;

        QP_mpc_x_.InitializeProblemSize(N, N);
        QP_mpc_y_.InitializeProblemSize(N, N);
 
        MPC_first_loop = 1;
        cout << "Initialization of MPC parameters is complete." << endl;
    }

    Eigen::VectorXd p_x(N);
    Eigen::VectorXd p_y(N);
    // Eigen::VectorXd p(2*N);
    Z_x_ref_.setZero(N);
    Z_y_ref_.setZero(N);
    Z_x_ref_wo_offset_.setZero(75); // this shoud be same with CP-MPC's N_cp!!
    Z_y_ref_wo_offset_.setZero(75);

    for(int i = 0; i < N; i ++)
    {
        int index = mpc_tick + MPC_synchro_hz_*(i+1);
        Z_x_ref_(i) = ref_zmp_mpc_(index, 0); // 20 = Control freq (2000) / MPC_freq (100)
        Z_y_ref_(i) = ref_zmp_mpc_(index, 1);
    }    

    // this is for CP-MPC
    for(int i = 0; i < 75; i ++)
    {
        int index = mpc_tick + MPC_synchro_hz_*(i+1);
        Z_x_ref_wo_offset_(i) = ref_zmp_wo_offset_mpc_(index, 0); // 20 = Control freq (2000) / MPC_freq (100)
        Z_y_ref_wo_offset_(i) = ref_zmp_wo_offset_mpc_(index, 1);
    }  

    // MJ_graph2 <<Z_x_ref_(0)<<","<< Z_y_ref_(0) << ","<<mpc_tick <<","<< MPC_synchro_hz_<<","<<mpc_tick+MPC_synchro_hz_<<endl;

    //define cost functions
    p_x = W2_mpc_*P_zu_mpc_.transpose()*(P_zs_mpc_*x_hat_ - Z_x_ref_);
    p_y = W2_mpc_*P_zu_mpc_.transpose()*(P_zs_mpc_*y_hat_ - Z_y_ref_);
    // p.segment(0, N) = p_x;
    // p.segment(N, N) = p_y;  
    
    Eigen::VectorXd lb_b_x(N);
    Eigen::VectorXd ub_b_x(N);
    Eigen::VectorXd lb_b_y(N);
    Eigen::VectorXd ub_b_y(N);
    // Eigen::VectorXd lb_b(2*N);
    // Eigen::VectorXd ub_b(2*N);
    Eigen::VectorXd zmp_bound(N);

    for(int i = 0; i < N; i++)  
    {
        zmp_bound(i) = 0.1;
    }
    
    lb_b_x = Z_x_ref_ - zmp_bound * 1.3 - P_zs_mpc_*x_hat_;
    ub_b_x = Z_x_ref_ + zmp_bound * 1.7 - P_zs_mpc_*x_hat_;
    lb_b_y = Z_y_ref_ - zmp_bound - P_zs_mpc_*y_hat_;
    ub_b_y = Z_y_ref_ + zmp_bound - P_zs_mpc_*y_hat_;
    
    // QP_mpc_x_.InitializeProblemSize(N, N);
    QP_mpc_x_.EnableEqualityCondition(equality_condition_eps_);
    QP_mpc_x_.UpdateMinProblem(Q_prime_,p_x);
    QP_mpc_x_.DeleteSubjectToAx();      
    QP_mpc_x_.UpdateSubjectToAx(P_zu_mpc_, lb_b_x, ub_b_x);
    
    //U_x_mpc_.setZero(N);  
    if (QP_mpc_x_.SolveQPoases(200, MPC_input_x_))
    {   
        x_hat_p_ = x_hat_;            
        U_x_mpc_ = MPC_input_x_.segment(0, N);

        x_com_pos_recur_ = P_ps_mpc_ * x_hat_ + P_pu_mpc_* U_x_mpc_;
        x_com_vel_recur_ = P_vs_mpc_ * x_hat_ + P_vu_mpc_* U_x_mpc_;
        x_zmp_recur_ = P_zs_mpc_ * x_hat_ + P_zu_mpc_* U_x_mpc_;

        x_hat_ = A_mpc_ * x_hat_ + B_mpc_ * U_x_mpc_(0);
    }
        
    // QP_mpc_y_.InitializeProblemSize(N, N);
    QP_mpc_y_.EnableEqualityCondition(equality_condition_eps_);
    QP_mpc_y_.UpdateMinProblem(Q_prime_,p_y);
    QP_mpc_y_.DeleteSubjectToAx();      
    QP_mpc_y_.UpdateSubjectToAx(P_zu_mpc_, lb_b_y, ub_b_y);
    
   //U_y_mpc_.setZero(N); 
    if (QP_mpc_y_.SolveQPoases(200, MPC_input_y_))
    {             
        y_hat_p_ = y_hat_;

        U_y_mpc_ = MPC_input_y_.segment(0, N);       

        y_com_pos_recur_ = P_ps_mpc_ * y_hat_ + P_pu_mpc_* U_y_mpc_;
        y_com_vel_recur_ = P_vs_mpc_ * y_hat_ + P_vu_mpc_* U_y_mpc_;
        y_zmp_recur_ = P_zs_mpc_ * y_hat_ + P_zu_mpc_* U_y_mpc_;
        y_hat_ = A_mpc_ * y_hat_ + B_mpc_ * U_y_mpc_(0);

    }    

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    
    if((int)walking_tick_mj_mpc_ % 200 == 0)
    {   
       // cout<<"wieber mpc calculation time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << endl;
    }    
}

void AvatarController::cpcontroller_MPC_MJDG(double MPC_freq, double preview_window)
{
    ///////////////////////////////////////// CP control + stepping MPC ///////////////////////////////////////
    //// https://doi.org/10.3182/20120905-3-HR-2030.00165

    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();    

    int mpc_tick = walking_tick_mj_mpc_ - zmp_start_time_mj_mpc_;
    int N_cp = preview_window * MPC_freq; 
    // int footprint_num = ceil(preview_window*hz_/t_total_mpc_); // 0.9s walking->2, less than 0.75s walking -> 3
    int footprint_num = 2;
    double T = 1/MPC_freq;
    int MPC_synchro_hz = 2000.0 / MPC_freq;

    Eigen::VectorXd cp_x_ref(N_cp);
    Eigen::VectorXd cp_y_ref(N_cp);

    // Eigen::VectorXd Z_x_ref_wo_offset_(N_cp);
    // Eigen::VectorXd Z_y_ref_wo_offset_(N_cp);    

    Eigen::MatrixXd zeros_Ncp_x_f(N_cp, footprint_num);
    Eigen::MatrixXd zeros_Ncp_x_Ncp(N_cp, N_cp);
    Eigen::MatrixXd eyes_f(footprint_num, footprint_num);
    Eigen::MatrixXd P_sel(N_cp, footprint_num);

    zeros_Ncp_x_f.setZero();
    zeros_Ncp_x_Ncp.setZero();
    eyes_f.setIdentity();
    P_sel.setZero();

    // reference CP trajectory generation
    cp_x_ref = x_com_pos_recur_.segment(0, N_cp) + x_com_vel_recur_.segment(0, N_cp)/wn; // 1.5 s
    cp_y_ref = y_com_pos_recur_.segment(0, N_cp) + y_com_vel_recur_.segment(0, N_cp)/wn;
    
    // if(atb_cpmpc_rcv_update_ == false) // Receive datas from the compute slow thread 
    // {
    //     atb_cpmpc_rcv_update_ = true;
                
    //     if(current_step_num_thread_ == current_step_num_mpc_)
    //     {
                      
    //     }
    //     else
    //     {
    //         cout << "computeslow thread step num = " << current_step_num_thread_ << endl;
    //         cout << "MPC thread step num = " << current_step_num_mpc_ << endl;            
    //         cout << "stepchange was occured in only computeslow thread." << endl;
    //     }

    //     if(current_step_num_mpc_ != current_step_num_mpc_prev_) // receive step change control input (stepchange state in MPC thread)
    //     {      
            
    //     } // In this case, step change of the des.ZMP was occured in only computeslow and the step num is also increased in MPC, but MPC is not fully ended.

    //     atb_cpmpc_rcv_update_ = false;        
    // }  

    if(CP_MPC_first_loop_ == false)
    {
        // Define Input matrix
        Eigen::VectorXd B_cp_mpc(1);
        B_cp_mpc(0) = 1 - exp(wn*T); 
        
        // Define recursive state, input matrix
        F_cp_.setZero(N_cp, 1);
        F_zmp_.setZero(N_cp, N_cp);
        
        for(int i = 0; i < N_cp; i++)
        {   
            F_cp_(i,0) = exp(wn*T*i);

            for(int j = 0; j < N_cp; j++)
            {
                if(j >= i)
                {
                    F_zmp_(j,i) = exp(wn*T*(j-i))*B_cp_mpc(0);
                }
            }
        }        
        
        // Define diffence matrix
        diff_matrix_.setIdentity(N_cp, N_cp);

        for(int i = 0; i < N_cp-1; i ++)
        {
            diff_matrix_(i+1, i) = -1.0;
        }
        
        e1_cpmpc_.setZero(N_cp);
        e1_cpmpc_(0) = 1.0;
        
        weighting_cp_.setZero(N_cp, N_cp);
        weighting_zmp_diff_.setZero(N_cp, N_cp);
        double weighting_foot = 0.01;// 100.0;  //0.01;

        // Weighting parameter
        for(int i = 0; i < N_cp; i++) // N_cp = 75
        {
            if(i < 1)
            {
                weighting_cp_(i,i) = 1.0;
                weighting_zmp_diff_(i,i) = 0.2;
                // weighting_cp_(i,i) = 2.0;
                // weighting_zmp_diff_(i,i) = 0.2;
            }
            else if (i < 50)
            {
                weighting_cp_(i,i) = 1.0;
                weighting_zmp_diff_(i,i) = 10.0;                
            }
            else
            {
                weighting_cp_(i,i) = 100.0;
                weighting_zmp_diff_(i,i) = 0.10; 
            }            
        }

        // Hessian matrix
        H_cpmpc_.setZero(N_cp, N_cp);
        H_cpmpc_ = diff_matrix_.transpose()*weighting_zmp_diff_*diff_matrix_ + F_zmp_.transpose()*weighting_cp_*F_zmp_;        

        H_cpStepping_mpc_.setZero(N_cp + footprint_num, N_cp + footprint_num);
        H_cpStepping_mpc_.block(0, 0, N_cp, N_cp) = H_cpmpc_; // CP-MPC
        H_cpStepping_mpc_.block(N_cp, 0, footprint_num, N_cp) = zeros_Ncp_x_f.transpose();
        H_cpStepping_mpc_.block(0, N_cp, N_cp, footprint_num) = zeros_Ncp_x_f;
        H_cpStepping_mpc_.block(N_cp, N_cp, footprint_num, footprint_num) = weighting_foot*eyes_f; // Foot mpc

        // Control input (desired zmp) initinalization 
        cpmpc_deszmp_x_.setZero(N_cp + footprint_num);
        cpmpc_deszmp_x_(0) = x_hat_(0); // Position of the CoM
        
        cpmpc_deszmp_y_.setZero(N_cp + footprint_num);
        cpmpc_deszmp_y_(0) = y_hat_(0); // Position of the CoM
        
        QP_cpmpc_x_.InitializeProblemSize(N_cp + footprint_num, N_cp + footprint_num); // MPC variable : desired ZMP, foot position 
        QP_cpmpc_y_.InitializeProblemSize(N_cp + footprint_num, N_cp + footprint_num);

        CP_MPC_first_loop_ = true;
        cout << "Initialization of CP_MPC parameters is complete." << endl;
    }  
    // cout << "test 1" << endl;
    // For foot adjustment
    int swing_time_cur = 0, swing_time_next = 0, swing_time_n_next = 0, dsp_time1 = 0, dsp_time2 = 0;
    
    if(current_step_num_mpc_ > 0 && current_step_num_mpc_ != total_step_num_mpc_-1) // Define selection vector for swingfoot adjustment
    {
        swing_time_cur = (t_total_mpc_ - mpc_tick)/MPC_synchro_hz; // remaining sampling time in current foot. 

        if(N_cp - swing_time_cur >= t_total_mpc_/MPC_synchro_hz) // 3 footholds are included in N_cp step.(current, next, n_next foothold)
        {
            swing_time_next = t_total_mpc_/MPC_synchro_hz;
            swing_time_n_next = N_cp - (t_total_mpc_/MPC_synchro_hz + swing_time_cur);
            dsp_time1 = (t_rest_init_mpc_ + t_double1_)/MPC_synchro_hz;
            dsp_time2 = (t_rest_last_mpc_ + t_double2_)/MPC_synchro_hz;
        }
        else // 2 footholds are included in N_cp step.(current, next foothold)
        {
            swing_time_next = N_cp - swing_time_cur;
            swing_time_n_next = 0;
            dsp_time1 = (t_rest_init_mpc_ + t_double1_)/MPC_synchro_hz;
            dsp_time2 = 0;
        }

        Eigen::VectorXd sel_swingfoot(swing_time_cur); 
        Eigen::VectorXd sel_swingfoot_next(swing_time_next);  
        Eigen::VectorXd sel_swingfoot_n_next(swing_time_n_next);  
        
        sel_swingfoot.setZero();
        
        sel_swingfoot_next.setOnes();
        sel_swingfoot_next.segment(0, dsp_time1).setZero();
        sel_swingfoot_next.segment(swing_time_next - dsp_time2, dsp_time2).setZero();
        
        P_sel.block(0, 0, swing_time_cur, 1) = sel_swingfoot;        
        P_sel.block(swing_time_cur, 0, swing_time_next, 1) = sel_swingfoot_next;  

        if(swing_time_n_next != 0)
        {
            if(swing_time_n_next < dsp_time1)
            {
                sel_swingfoot_n_next.setZero();
            }
            else
            {
                int swing_last_time_n_next = DyrosMath::minmax_cut(swing_time_n_next-dsp_time1, 0, int(t_total_mpc_)-dsp_time1-dsp_time2);

                sel_swingfoot_n_next.setZero();
                sel_swingfoot_n_next.segment(dsp_time1, swing_last_time_n_next).setOnes();
            }
            P_sel.block(swing_time_cur + swing_time_next, 1, swing_time_n_next, 1) = sel_swingfoot_n_next;
        }
        // cout << P_sel << "," << swing_time_cur << "," << swing_time_next << "," << swing_time_n_next << "," << dsp_time1 << endl;
    }
    else
    {       
        P_sel.setZero(N_cp,footprint_num); 
    }
     
    Eigen::VectorXd g_cpmpc_x(N_cp);
    Eigen::VectorXd g_cpmpc_y(N_cp);
    Eigen::VectorXd g_cpStepping_mpc_x(N_cp + footprint_num);
    Eigen::VectorXd g_cpStepping_mpc_y(N_cp + footprint_num);
    
    // gradient vector
    g_cpmpc_x = F_zmp_.transpose()*weighting_cp_*(F_cp_*cp_measured_mpc_(0) - cp_x_ref) - diff_matrix_.transpose()*weighting_zmp_diff_*e1_cpmpc_*cpmpc_deszmp_x_(0);
    g_cpStepping_mpc_x.setZero(N_cp + footprint_num);
    g_cpStepping_mpc_x.segment(0, N_cp) = g_cpmpc_x;

    g_cpmpc_y = F_zmp_.transpose()*weighting_cp_*(F_cp_*cp_measured_mpc_(1) - cp_y_ref) - diff_matrix_.transpose()*weighting_zmp_diff_*e1_cpmpc_*cpmpc_deszmp_y_(0);
    g_cpStepping_mpc_y.setZero(N_cp + footprint_num);
    g_cpStepping_mpc_y.segment(0, N_cp) = g_cpmpc_y;
     
    // constraint formulation
    Eigen::MatrixXd A_cpStepping_mpc(N_cp+footprint_num, N_cp+footprint_num);
    Eigen::MatrixXd A_cp_mpc(N_cp, N_cp);
    A_cp_mpc.setIdentity();
    A_cpStepping_mpc.block(0, 0, N_cp, N_cp) = A_cp_mpc;  
    A_cpStepping_mpc.block(0, N_cp, N_cp, footprint_num) = -P_sel;
    A_cpStepping_mpc.block(N_cp, 0, footprint_num, N_cp) = zeros_Ncp_x_f.transpose();
    A_cpStepping_mpc.block(N_cp, N_cp, footprint_num, footprint_num) = eyes_f;

    Eigen::VectorXd ub_x_cp_mpc(N_cp);
    Eigen::VectorXd lb_x_cp_mpc(N_cp);
    Eigen::VectorXd ub_y_cp_mpc(N_cp);
    Eigen::VectorXd lb_y_cp_mpc(N_cp);    
     
    Eigen::VectorXd ub_x_foot_cp_mpc(footprint_num);
    Eigen::VectorXd lb_x_foot_cp_mpc(footprint_num);
    Eigen::VectorXd ub_y_foot_cp_mpc(footprint_num);
    Eigen::VectorXd lb_y_foot_cp_mpc(footprint_num);   

    Eigen::VectorXd ub_x_cpStepping_mpc(N_cp + footprint_num);
    Eigen::VectorXd lb_x_cpStepping_mpc(N_cp + footprint_num);
    Eigen::VectorXd ub_y_cpStepping_mpc(N_cp + footprint_num);
    Eigen::VectorXd lb_y_cpStepping_mpc(N_cp + footprint_num);    
    
    // for(int i = 0; i < N_cp; i ++)
    // {
    //     int index = mpc_tick + MPC_synchro_hz*(i+1);
    //     Z_x_ref_wo_offset_(i) = ref_zmp_mpc_(index, 0); // 20 = Control freq (2000) / MPC_freq (100)
    //     Z_y_ref_wo_offset_(i) = ref_zmp_mpc_(index, 1);
    // }  
    
    MJ_CP_ZMP <<Z_x_ref_wo_offset_(0)<<","<< Z_y_ref_wo_offset_(0) <<  ","<<mpc_tick <<","<< MPC_synchro_hz<< ","<<mpc_tick+MPC_synchro_hz<<endl;
    Eigen::VectorXd zmp_bound_x(N_cp);
    Eigen::VectorXd zmp_bound_y(N_cp);
     
    for(int i = 0; i < N_cp; i++)  
    {
        zmp_bound_x(i) = 0.1;
        zmp_bound_y(i) = 0.07;  
    }
     
    lb_x_cp_mpc = Z_x_ref_wo_offset_ - zmp_bound_x*1.2;
    ub_x_cp_mpc = Z_x_ref_wo_offset_ + zmp_bound_x*1.8;

    lb_y_cp_mpc = Z_y_ref_wo_offset_ - zmp_bound_y; // Z_y_ref_ is the trajectory considering the ZMP offset for COM planning.
    ub_y_cp_mpc = Z_y_ref_wo_offset_ + zmp_bound_y; // However, Ref. ZMP without ZMP offset is required for CP control.
     
    double del_F_y_rightswing_min = -0.08, del_F_y_rightswing_max = 0.03;
    double del_F_y_leftswing_min = -0.03, del_F_y_leftswing_max = 0.08;
    double rightswingfoot_y_min = -0.35, rightswingfoot_y_max = -0.20;
    double leftswingfoot_y_min = 0.20, leftswingfoot_y_max = 0.35;

    ub_x_foot_cp_mpc.setZero();   
    lb_x_foot_cp_mpc.setZero();
    
    ub_x_foot_cp_mpc(0) = min(+0.2, 0.3- foot_step_support_frame_mpc_(current_step_num_mpc_, 0));// - foot_step_support_frame_(current_step_num_, 0); // 제자리 테스트에서는 일단 0.1, max : 0.2
    lb_x_foot_cp_mpc(0) = max(-0.2, -0.3- foot_step_support_frame_mpc_(current_step_num_mpc_, 0));// - foot_step_support_frame_(current_step_num_, 0); // 제자리 테스트 일단 -0.1, min : -0.15
        
    for(int i = 1; i < footprint_num; i ++) // 다음 놓일 위치에서? 아니면 실시간 스윙발 위치에서?
    {
        // real robot experiment 0.2?
        ub_x_foot_cp_mpc(i) = 0.2; // 제자리 테스트에서는 일단 0.1, max : 0.2
        lb_x_foot_cp_mpc(i) = -0.2;// - foot_step_support_frame_(current_step_num_, 0); // 제자리 테스트 일단 -0.1, min : -0.15
        
        // if(alpha_step_mpc_ == 1) // left foot support
        // {
        //     ub_x_foot_cp_mpc(0) = 0.2 - rfoot_support_current_.translation()(0);
        //     lb_x_foot_cp_mpc(0) = -0.2 - rfoot_support_current_.translation()(0);
        // }
        // else if(alpha_step_mpc_ == -1) // right foot support
        // {
        //     ub_x_foot_cp_mpc(0) = 0.2 - lfoot_support_current_.translation()(0);
        //     lb_x_foot_cp_mpc(0) = -0.2 - lfoot_support_current_.translation()(0);
        // } 
    }   
    ub_y_foot_cp_mpc.setZero();   
    lb_y_foot_cp_mpc.setZero();
     
    if(current_support_foot_is_left_mpc_ == true)// left foot support
    {   
        ub_y_foot_cp_mpc(0) = rightswingfoot_y_max - foot_step_support_frame_mpc_(current_step_num_mpc_, 1);
        lb_y_foot_cp_mpc(0) = rightswingfoot_y_min - foot_step_support_frame_mpc_(current_step_num_mpc_, 1);
        
        ub_y_foot_cp_mpc(1) = del_F_y_leftswing_max;
        lb_y_foot_cp_mpc(1) = del_F_y_leftswing_min;          
    }
    else if(current_support_foot_is_left_mpc_ == false) // right foot support
    {
        // for(int i = 0; i < footprint_num; i ++)
        // {
        //     ub_y_foot_cp_mpc(i) = del_F_y_leftswing_max;
        //     lb_y_foot_cp_mpc(i) = del_F_y_leftswing_min;
        // }

        ub_y_foot_cp_mpc(0) = leftswingfoot_y_max - foot_step_support_frame_mpc_(current_step_num_mpc_, 1);
        lb_y_foot_cp_mpc(0) = leftswingfoot_y_min - foot_step_support_frame_mpc_(current_step_num_mpc_, 1);
        
        ub_y_foot_cp_mpc(1) = del_F_y_rightswing_max;
        lb_y_foot_cp_mpc(1) = del_F_y_rightswing_min;        
    }

    lb_x_cpStepping_mpc.segment(0, N_cp) = lb_x_cp_mpc;
    lb_x_cpStepping_mpc.segment(N_cp, footprint_num) = lb_x_foot_cp_mpc;
    ub_x_cpStepping_mpc.segment(0, N_cp) = ub_x_cp_mpc;
    ub_x_cpStepping_mpc.segment(N_cp, footprint_num) = ub_x_foot_cp_mpc; 

    lb_y_cpStepping_mpc.segment(0, N_cp) = lb_y_cp_mpc;
    lb_y_cpStepping_mpc.segment(N_cp, footprint_num) = lb_y_foot_cp_mpc;
    ub_y_cpStepping_mpc.segment(0, N_cp) = ub_y_cp_mpc;
    ub_y_cpStepping_mpc.segment(N_cp, footprint_num) = ub_y_foot_cp_mpc; 
    
    // Define QP problem for CP-MPC  
    QP_cpmpc_x_.EnableEqualityCondition(equality_condition_eps_);
    QP_cpmpc_x_.UpdateMinProblem(H_cpStepping_mpc_,g_cpStepping_mpc_x);
    QP_cpmpc_x_.DeleteSubjectToAx();      
    QP_cpmpc_x_.UpdateSubjectToAx(A_cpStepping_mpc, lb_x_cpStepping_mpc, ub_x_cpStepping_mpc);             
       
    if (QP_cpmpc_x_.SolveQPoases(200, cpmpc_input_x_))
    {          
        cpmpc_des_zmp_x_p_mpc_ = cpmpc_des_zmp_x_mpc_;      
        cpmpc_deszmp_x_ = cpmpc_input_x_.segment(0, N_cp + footprint_num);
        del_F_x_mpc_ = cpmpc_deszmp_x_(N_cp);
        cpmpc_des_zmp_x_mpc_ = cpmpc_deszmp_x_(0);
    }
    
    QP_cpmpc_y_.EnableEqualityCondition(equality_condition_eps_);
    QP_cpmpc_y_.UpdateMinProblem(H_cpStepping_mpc_,g_cpStepping_mpc_y);
    QP_cpmpc_y_.DeleteSubjectToAx();      
    QP_cpmpc_y_.UpdateSubjectToAx(A_cpStepping_mpc, lb_y_cpStepping_mpc, ub_y_cpStepping_mpc);    
     
    if (QP_cpmpc_y_.SolveQPoases(200, cpmpc_input_y_))
    {             
        cpmpc_des_zmp_y_p_mpc_ = cpmpc_des_zmp_y_mpc_; 
        cpmpc_deszmp_y_ = cpmpc_input_y_.segment(0, N_cp + footprint_num);
        del_F_y_mpc_ = cpmpc_deszmp_y_(N_cp);
        cpmpc_des_zmp_y_mpc_ = cpmpc_deszmp_y_(0);
    }

    
    // MJ_graph << cp_x_ref(0) << "," << cp_measured_mpc_(0) << "," << Z_x_ref_wo_offset_(0) << "," << cpmpc_deszmp_x_(0) << "," << cpmpc_deszmp_x_(N_cp) << endl; //"," << t_total_ << "," << cp_err_norm_x << "," << weighting_dsp << "," << cp_predicted_x(0) - cp_x_ref(0) << endl;
    // MJ_graph << cp_y_ref(0) << "," << cp_measured_mpc_(1) << "," << Z_y_ref_wo_offset_(0) << "," << cpmpc_deszmp_y_(0) << "," << cpmpc_deszmp_y_(N_cp) << endl; //"," << t_total_ << "," << cp_err_integ_y_ << "," << weighting_dsp <<  endl;
        
    current_step_num_mpc_prev_ = current_step_num_mpc_;
    current_support_foot_is_left_mpc_prev_ = current_support_foot_is_left_mpc_; 
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    // if((int)walking_tick_mj_mpc_ % 2 == 0)
    // {   
    //     cout<<"cp mpc calculation time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() << endl;
    // }    
}
void AvatarController::thread3SendDataToThread1()
{
    if(atb_thread3_output_data_update_ == false)
    {
        atb_thread3_output_data_update_ = true;
        // mpc x
        x_hat_p_thread_ = x_hat_p_;
        x_hat_thread_ = x_hat_;
        current_step_num_thread2_ = current_step_num_mpc_;

        // mpc y
        y_hat_p_thread_ = y_hat_p_;
        y_hat_thread_ = y_hat_;
        current_step_num_thread2_ = current_step_num_mpc_;

        // cpmpc x
        del_F_x_thread_ = del_F_x_mpc_;
        cpmpc_des_zmp_x_thread_ = cpmpc_des_zmp_x_mpc_;
        cpmpc_des_zmp_x_p_thread_ = cpmpc_des_zmp_x_p_mpc_;

        // cpmpc y
        del_F_y_thread_ = del_F_y_mpc_;
        cpmpc_des_zmp_y_thread_ = cpmpc_des_zmp_y_mpc_;
        cpmpc_des_zmp_y_p_thread_ = cpmpc_des_zmp_y_p_mpc_;

        current_support_foot_is_left_thread2_ = current_support_foot_is_left_mpc_;
        atb_thread3_output_data_update_ = false;
    }
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
    joy_command_buffer_.setZero();
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

void AvatarController::frictionTorqueCalculator(Eigen::VectorQd q_dot, Eigen::VectorQd q_dot_des, Eigen::VectorQd tau_m, Eigen::VectorQd &tau_f)
{
    int w1, w2, w3;
    double abs_q_dot_threshold = 0.01;
    tau_f.setZero();

    for (int j = 0; j < 12; j++) // only legs
    {
        if (abs(q_dot(j)) < abs_q_dot_threshold)
        {
            if (q_dot_des(j) == 0.0)
            {
                w1 = 1;
                w2 = 0;
            }
            else
            {
                w1 = 0;
                w2 = 1;
            }
            w3 = 0;
        }
        else
        {
            w1 = 0;
            w2 = 0;
            w3 = 1;
        }

        w_friction_.setZero(5);
        w_friction_(0) = w1 * DyrosMath::sign(tau_m(j)) + w3 * DyrosMath::sign(q_dot(j));
        w_friction_(1) = w2 * DyrosMath::sign(tau_m(j));
        w_friction_(2) = w3 * q_dot(j) * q_dot(j) * q_dot(j);
        w_friction_(3) = w3 * q_dot(j) * q_dot(j);
        w_friction_(4) = w3 * q_dot(j);

        tau_f(j) = w_friction_.transpose() * theta_joints_mat_leg_.col(j);
    }
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
        motion_q_(24) = 0.3; // pitch
        pd_control_mask_(23) = 1;
        pd_control_mask_(24) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////ARM/////////////////////////
        //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
        motion_q_(15) = 0.0;
        motion_q_(16) = -0.3;
        motion_q_(17) = 1.57;
        motion_q_(18) = -1.2;
        motion_q_(19) = -1.57; // elbow
        motion_q_(20) = 1.5;
        motion_q_(21) = 0.4;
        motion_q_(22) = -0.2;
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
        motion_q_(25) = 0.0;
        motion_q_(26) = 0.3;
        motion_q_(27) = -1.57;
        motion_q_(28) = 1.2;
        motion_q_(29) = 1.57; // elbow
        motion_q_(30) = -1.5;
        motion_q_(31) = -0.4;
        motion_q_(32) = 0.2;
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
                                                hand_pos_mapping_scale_raw_ * 1.3 * (hmd_lhand_pose_.translation() - hmd_lhand_pose_start_.translation());

        master_rhand_pose_raw_.translation() = master_rhand_pose_start_.translation() + 
                                                hand_pos_mapping_scale_raw_ * 1.3 * (hmd_rhand_pose_.translation() - hmd_rhand_pose_start_.translation());
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
void AvatarController::collisionEstimation()
{
    collisionIsolation();
    // collisionIdentification();
}
void AvatarController::collisionIsolation()
{
    left_leg_collision_detected_link_ = 0;
    right_leg_collision_detected_link_ = 0;

    int detection_tick_margin = 0.05 * hz_;
    unsigned int continuous_filter_window = 4;
    double compensation_gain = 1.0;
    double foot_normal_force_threshold = 10;

    VectorQd estimated_model_unct_torque_std;

    for (int i = 0; i < 12; i++)
    {
        estimated_model_unct_torque_std(i) = sqrt(estimated_external_torque_variance_gru_slow_(i));
    }

    VectorQd threshold_joint_torque_w_sigma = 1.1 * threshold_joint_torque_collision_ + 2.0 * estimated_model_unct_torque_std;

    ///////////////////////////////////////////

    // if(walking_tick_mj%200 == 0)
    // {
    //     cout<<"estimated_ext_torque_lstm_: "<<estimated_ext_torque_lstm_.segment(0, 6).transpose() <<endl;
    //     cout<<"estimated_ext_force_lfoot_lstm_: "<<estimated_ext_force_lfoot_lstm_.segment(0, 6).transpose() <<endl;
    // }

    ///////////////////////////LEFT LEG////////////////////////////////////
    for (int i = 0; i < 6; i++)
    {
        // threshold_joint_torque_collision_(i) = 30;
        // left leg
        if (abs(estimated_external_torque_gru_slow_(i)) > threshold_joint_torque_w_sigma(i))
        {
            left_leg_collision_detected_link_ = i + 1;
        }

        if ((left_leg_collision_detected_link_ >= 1) && (left_leg_collision_detected_link_ <= 3))
        {
            left_leg_collision_detected_link_ = 3;
        }
        else if ((left_leg_collision_detected_link_ >= 5) && (left_leg_collision_detected_link_ <= 6))
        {
            left_leg_collision_detected_link_ = 6;
        }
    }

    bool check_left_swing_foot_ = (walking_tick_mj >= t_rest_init_ + t_double1_ + t_start_ + detection_tick_margin) && (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - detection_tick_margin) && (foot_step_(current_step_num_, 6) == 0);
    bool check_left_early_contact = (estimated_ext_force_lfoot_gru_(2) > foot_normal_force_threshold);

    if (left_leg_collision_detected_link_ != 0 && check_left_swing_foot_ && !check_left_early_contact)
    {
        if (left_leg_collision_detected_link_ == 3)
        {
            left_leg_collision_cnt_[0] += 1;
        }
        else
        {
            left_leg_collision_cnt_[0] = 0;
        }

        if (left_leg_collision_detected_link_ == 4)
        {
            left_leg_collision_cnt_[1] += 1;
        }
        else
        {
            left_leg_collision_cnt_[1] = 0;
        }

        if (left_leg_collision_detected_link_ == 6)
        {
            left_leg_collision_cnt_[2] += 1;
        }
        else
        {
            left_leg_collision_cnt_[2] = 0;
        }

        // update max_collision_free_torque
        for (int i = 0; i < 6; i++)
        {
            if (maximum_collision_free_torque_(i) < estimated_external_torque_gru_slow_(i))
            {
                maximum_collision_free_torque_(i) = estimated_external_torque_gru_slow_(i);
            }
        }

        if (left_leg_collision_cnt_[0] >= continuous_filter_window || left_leg_collision_cnt_[1] >= continuous_filter_window || left_leg_collision_cnt_[2] >= continuous_filter_window)
        {

            // current_step_num_ = total_step_num_ - 3;
            if (collision_detection_flag_ == false)
            // if(true)
            {
                cout << "collision is detected on the " << left_leg_collision_detected_link_ << "th link of LEFT LEG" << endl;
                cout << "estimated_external_torque_gru_slow_: \n"
                     << estimated_external_torque_gru_slow_.segment(0, 6).transpose() << endl;
                cout << "estimated_ext_force_lfoot_gru_: \n"
                     << estimated_ext_force_lfoot_gru_.segment(0, 6).transpose() << endl;
                cout << "maximum_collision_free_torque_: \n"
                     << maximum_collision_free_torque_.segment(0, 6).transpose() << endl;

                // calculateFootStepTotal_reactive(lfoot_support_current_.translation(), estimated_ext_force_lfoot_gru_.segment(0, 3), false);
                collision_detection_flag_ = true;
            }

            for (int i = 0; i < 6; i++)
            {
                // if (estimated_ext_torque_lstm_(i) > 0)
                // {
                //     ext_torque_compensation_(i) = compensation_gain*(estimated_ext_torque_lstm_(i) - threshold_joint_torque_w_sigma(i));
                //     ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), 0.0, 100.0);
                // }
                // else
                // {
                //     ext_torque_compensation_(i) = compensation_gain*(estimated_ext_torque_lstm_(i) + threshold_joint_torque_w_sigma(i));
                //     ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), -100.0, 0.0);
                // }

                // ext_torque_compensation_(i) = compensation_gain*estimated_ext_torque_lstm_(i);
                // ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), -100.0, 100.0);
            }
        }

        for (int i = 0; i < 6; i++)
        {
            ext_torque_compensation_(i) = compensation_gain * estimated_external_torque_gru_slow_(i);
            ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), -100.0, 100.0);
        }
    }
    else
    {
        left_leg_collision_cnt_[0] = 0;
        left_leg_collision_cnt_[1] = 0;
        left_leg_collision_cnt_[2] = 0;

        for (int i = 0; i < 6; i++)
        {
            ext_torque_compensation_(i) *= 0.999;
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////RIGHT LEG////////////////////////////////////
    for (int i = 0; i < 6; i++)
    {
        // threshold_joint_torque_collision_(i) = 30;
        // left leg
        if (abs(estimated_external_torque_gru_slow_(i + 6)) > threshold_joint_torque_w_sigma(i + 6))
        {
            right_leg_collision_detected_link_ = i + 1;
        }

        if ((right_leg_collision_detected_link_ >= 1) && (right_leg_collision_detected_link_ <= 3))
        {
            right_leg_collision_detected_link_ = 3;
        }
        else if ((right_leg_collision_detected_link_ >= 5) && (right_leg_collision_detected_link_ <= 6))
        {
            right_leg_collision_detected_link_ = 6;
        }
    }

    bool check_right_swing_foot_ = (walking_tick_mj >= t_rest_init_ + t_double1_ + t_start_ + detection_tick_margin) && (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - detection_tick_margin) && (foot_step_(current_step_num_, 6) == 1);
    bool check_right_early_contact = (estimated_ext_force_rfoot_gru_(2) > foot_normal_force_threshold);

    if (right_leg_collision_detected_link_ != 0 && check_right_swing_foot_ && !check_right_early_contact)
    {
        if (right_leg_collision_detected_link_ == 3)
        {
            right_leg_collision_cnt_[0] += 1;
            // right_leg_collision_cnt_[1] = 0;
        }
        else
        {
            right_leg_collision_cnt_[0] = 0;
        }

        if (right_leg_collision_detected_link_ == 4)
        {
            right_leg_collision_cnt_[1] += 1;
            // right_leg_collision_cnt_[0] = 0;
        }
        else
        {
            right_leg_collision_cnt_[1] = 0;
        }

        if (right_leg_collision_detected_link_ == 6)
        {
            right_leg_collision_cnt_[2] += 1;
            // right_leg_collision_cnt_[0] = 0;
            // right_leg_collision_cnt_[1] = 0;
        }
        else
        {
            right_leg_collision_cnt_[2] = 0;
        }

        // update max_collision_free_torque
        for (int i = 6; i < 12; i++)
        {
            if (maximum_collision_free_torque_(i) < estimated_external_torque_gru_slow_(i))
            {
                maximum_collision_free_torque_(i) = estimated_external_torque_gru_slow_(i);
            }
        }

        if (right_leg_collision_cnt_[0] >= continuous_filter_window || right_leg_collision_cnt_[1] >= continuous_filter_window || right_leg_collision_cnt_[2] >= continuous_filter_window)
        {

            // current_step_num_ = total_step_num_ - 3;
            if (collision_detection_flag_ == false)
            // if(true)
            {
                cout << "collision is detected on the " << right_leg_collision_detected_link_ << "th link of RIGHT LEG" << endl;
                cout << "estimated_external_torque_gru_slow_: \n"
                     << estimated_external_torque_gru_slow_.segment(6, 6).transpose() << endl;
                cout << "estimated_ext_force_rfoot_gru_: \n"
                     << estimated_ext_force_rfoot_gru_.segment(0, 6).transpose() << endl;
                cout << "maximum_collision_free_torque_: \n"
                     << maximum_collision_free_torque_.segment(6, 6).transpose() << endl;
                // calculateFootStepTotal_reactive(rfoot_support_current_.translation(), estimated_ext_force_rfoot_gru_.segment(0, 3), true);
                collision_detection_flag_ = true;
            }

            for (int i = 6; i < 12; i++)
            {
                // if (estimated_ext_torque_lstm_(i) > 0)
                // {
                //     ext_torque_compensation_(i) = compensation_gain*(estimated_ext_torque_lstm_(i) - threshold_joint_torque_w_sigma(i));
                //     ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), 0.0, 100.0);
                // }
                // else
                // {
                //     ext_torque_compensation_(i) = compensation_gain*(estimated_ext_torque_lstm_(i) + threshold_joint_torque_w_sigma(i));
                //     ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), -100.0, 0.0);
                // }

                // ext_torque_compensation_(i) = compensation_gain*estimated_ext_torque_lstm_(i);
                // ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), -100.0, 100.0);
            }
        }

        // reflex torque control
        for (int i = 6; i < 12; i++)
        {
            ext_torque_compensation_(i) = compensation_gain * estimated_external_torque_gru_slow_(i);
            ext_torque_compensation_(i) = DyrosMath::minmax_cut(ext_torque_compensation_(i), -100.0, 100.0);
        }
    }
    else
    {
        right_leg_collision_cnt_[0] = 0;
        right_leg_collision_cnt_[1] = 0;
        right_leg_collision_cnt_[2] = 0;

        for (int i = 6; i < 12; i++)
        {
            ext_torque_compensation_(i) *= 0.999;
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////////
}
void AvatarController::collisionIdentification()
{
    if (collision_detection_flag_)
    {
        if (left_leg_collision_detected_link_ == 3)
        {
            Matrix6d left_leg_thigh_jac;
            MatrixXd J_temp, left_leg_thigh_jac_3_3, left_leg_thigh_jac_6_3;
            Vector3d l_thigh_offset_(0, 0, -0.175);
            J_temp.setZero(6, MODEL_DOF_VIRTUAL);
            RigidBodyDynamics::CalcPointJacobian6D(model_local_, q_virtual_Xd_local_, rd_.link_[Left_Foot - 3].id, l_thigh_offset_, J_temp, false);
            left_leg_thigh_jac.block(0, 0, 3, 6) = J_temp.block(3, 6, 3, 6); // position
            left_leg_thigh_jac.block(3, 0, 3, 6) = J_temp.block(0, 6, 3, 6); // orientation
            // cout << "left_leg_thigh_jac \n"
            //      << left_leg_thigh_jac << endl;
            left_leg_thigh_jac_3_3.setZero(3, 3);
            left_leg_thigh_jac_6_3.setZero(6, 3);
            left_leg_thigh_jac_3_3 = left_leg_thigh_jac.block(0, 0, 3, 3);
            left_leg_thigh_jac_6_3 = left_leg_thigh_jac.block(0, 0, 6, 3);

            Vector3d force_on_origin;
            Vector6d force_on_origin2;
            MatrixXd I3;
            MatrixXd I6;
            I3.setIdentity(3, 3);
            I6.setIdentity(6, 6);
            force_on_origin = left_leg_thigh_jac_3_3 * ((left_leg_thigh_jac_3_3.transpose() * left_leg_thigh_jac_3_3) + I3 * 0.000001).inverse() * estimated_ext_torque_lstm_.segment(0, 3);
            // force_on_origin = (left_leg_thigh_jac_3_3.transpose()).inverse()*estimated_ext_torque_lstm_.segment(0, 3);
            force_on_origin2 = left_leg_thigh_jac_6_3 * ((left_leg_thigh_jac_6_3.transpose() * left_leg_thigh_jac_6_3) + I3 * 0.000001).inverse() * estimated_ext_torque_lstm_.segment(0, 3);
            // cout << "estimated_ext_torque_lstm_.segment(0, 3) \n"
            //      << estimated_ext_torque_lstm_.segment(0, 3).transpose() << endl;
            cout << "force_on_origin1 \n"
                 << force_on_origin.transpose() << endl;
            cout << "force_on_origin2 \n"
                 << force_on_origin2.transpose() << endl;
            cout << "force on foot \n"
                 << estimated_ext_force_lfoot_lstm_.transpose() << endl;

            Vector3d r_origin_thigh, r_a_thigh;
            Matrix3d skm_r_o_T;

            // r_origin_thigh = rd_.link_[Left_Foot-3].xpos + rd_.link_[Left_Foot-3].rotm*l_thigh_offset_ - rd_.link_[Pelvis].xpos;
            // skm_r_o_T = DyrosMath::skm(wrench_on_origin.segment(0, 3)).transpose();
            // cout<<"skm_r_o_T \n"<<skm_r_o_T<<endl;
            // cout<<"skm_r_o_T inverse \n"<< (skm_r_o_T.transpose()*skm_r_o_T + I3*0.000001).inverse()*skm_r_o_T.transpose() <<endl;
            // r_a_thigh = (skm_r_o_T.transpose()*skm_r_o_T + I3*0.000001).inverse()*skm_r_o_T.transpose()*wrench_on_origin.segment(3, 3);
            // cout<<"r_a_thigh \n"<<r_a_thigh.transpose()<<endl;
        }

        if (left_leg_collision_detected_link_ == 4)
        {
        }

        if (left_leg_collision_detected_link_ == 6)
        {
        }
    }
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
void AvatarController::loadCollisionThreshold(std::string folder_path)
{
    std::string thr_path("collision_threshold.txt");

    thr_path = folder_path + thr_path;
    col_thr_file_.open(thr_path, ios::in);

    int index = 0;
    float temp;
    threshold_joint_torque_collision_.setZero();

    if (!col_thr_file_.is_open())
    {
        std::cout << "Can not find the Collision Threshold file" << std::endl;
    }
    else
    {
        while (!col_thr_file_.eof())
        {
            col_thr_file_ >> temp;

            if (temp == temp)
            {
                if (index < 12)
                {
                    threshold_joint_torque_collision_(index) = temp;
                    index++;
                    cout << "threshold: " << index << ", " << temp << endl;
                }
                else
                {
                    cout << "Collision Threshold file has more than 12 values" << endl;
                }
            }
            else
            {
                cout << "WARNING: collision_threshold has NaN value! (" << temp << ") at" + thr_path << endl;
            }
        }
    }

    if (index == 12)
    {
        cout << "Collision Threshold: [" << threshold_joint_torque_collision_.segment(0, 12).transpose() << "]" << endl;
    }
    col_thr_file_.close();

    maximum_collision_free_torque_ = threshold_joint_torque_collision_;
}

void AvatarController::initializeLegLSTM(LSTM &lstm)
{
    lstm.n_input = n_input_;
    lstm.n_sequence_length = n_sequence_length_;
    lstm.n_output = n_output_;
    lstm.n_hidden = n_hidden_;

    lstm.buffer_size = buffer_size_;
    lstm.nn_input_size = nn_input_size_;

    lstm.input_mode_idx = 19;
    lstm.output_mode_idx = 19;

    lstm.ring_buffer.setZero(buffer_size_);
    lstm.buffer_head = lstm.buffer_size - 1;
    lstm.buffer_tail = 0;
    lstm.input_slow.setZero(nn_input_size_, 20);
    lstm.input_fast.setZero(nn_input_size_, 20);
    lstm.input_thread.setZero(nn_input_size_, 20);
    lstm.input_mean.setZero(n_input_);
    lstm.input_std.setZero(n_input_);
    lstm.robot_input_data.setZero(n_input_);

    lstm.real_output.setZero(n_output_); // real out with physical dimension
    lstm.output_mean.setZero(n_output_);
    lstm.output_std.setZero(n_output_);

    lstm.W_ih.setZero(4 * n_hidden_ * n_input_);
    lstm.b_ih.setZero(4 * n_hidden_);
    lstm.W_hh.setZero(4 * n_hidden_ * n_hidden_);
    lstm.b_hh.setZero(4 * n_hidden_);
    lstm.W_linear.setZero(n_output_ * n_hidden_);
    lstm.b_linear.setZero(n_output_);

    lstm.hidden.setZero(n_hidden_, 20);
    lstm.cell.setZero(n_hidden_, 20);
    lstm.gates.setZero(4 * n_hidden_); // [input/update, forget, cell, output]

    lstm.input_gate.setZero(n_hidden_);
    lstm.forget_gate.setZero(n_hidden_);
    lstm.cell_gate.setZero(n_hidden_);
    lstm.output_gate.setZero(n_hidden_);
    lstm.output.setZero(n_output_);
}

void AvatarController::loadLstmWeights(LSTM &lstm, std::string folder_path)
{
    std::string W_ih_path("lstm_weight_ih_l0.txt");
    std::string W_hh_path("lstm_weight_hh_l0.txt");
    std::string b_ih_path("lstm_bias_ih_l0.txt");
    std::string b_hh_path("lstm_bias_hh_l0.txt");
    std::string W_linear_path("linear_weight.txt");
    std::string b_linear_path("linear_bias.txt");

    W_ih_path = folder_path + W_ih_path;
    W_hh_path = folder_path + W_hh_path;
    b_ih_path = folder_path + b_ih_path;
    b_hh_path = folder_path + b_hh_path;
    W_linear_path = folder_path + W_linear_path;
    b_linear_path = folder_path + b_linear_path;

    network_weights_file_[0].open(W_ih_path, ios::in);
    network_weights_file_[1].open(W_hh_path, ios::in);
    network_weights_file_[2].open(b_ih_path, ios::in);
    network_weights_file_[3].open(b_hh_path, ios::in);
    network_weights_file_[4].open(W_linear_path, ios::in);
    network_weights_file_[5].open(b_linear_path, ios::in);

    int index = 0;
    float temp;

    if (!network_weights_file_[0].is_open())
    {
        std::cout << "Can not find the W_ih file" << std::endl;
    }
    else
    {
        while (!network_weights_file_[0].eof())
        {
            network_weights_file_[0] >> temp;
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.W_ih(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: W_ih has NaN value! (" << temp << ") at" + W_ih_path << endl;
                }
            }
        }
        network_weights_file_[0].close();
    }

    if (!network_weights_file_[1].is_open())
    {
        std::cout << "Can not find the W_hh file" << std::endl;
    }
    else
    {
        index = 0;
        while (!network_weights_file_[1].eof())
        {
            network_weights_file_[1] >> temp;
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.W_hh(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: W_hh has NaN value!" << endl;
                }
            }
        }
        network_weights_file_[1].close();
    }

    if (!network_weights_file_[2].is_open())
    {
        std::cout << "Can not find the b_ih file" << std::endl;
    }
    else
    {
        index = 0;
        while (!network_weights_file_[2].eof())
        {
            network_weights_file_[2] >> temp;
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.b_ih(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: b_ih has NaN value!" << endl;
                }
            }
        }
        network_weights_file_[2].close();
    }

    if (!network_weights_file_[3].is_open())
    {
        std::cout << "Can not find the b_hh file" << std::endl;
    }
    else
    {
        index = 0;
        while (!network_weights_file_[3].eof())
        {
            network_weights_file_[3] >> temp;
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.b_hh(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: b_hh has NaN value!" << endl;
                }
            }
        }
        network_weights_file_[3].close();
    }

    if (!network_weights_file_[4].is_open())
    {
        std::cout << "Can not find the W_linear file" << std::endl;
    }
    else
    {
        index = 0;
        while (!network_weights_file_[4].eof())
        {
            network_weights_file_[4] >> temp;
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.W_linear(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: W_linear has NaN value!" << endl;
                }
            }
        }
        network_weights_file_[4].close();
    }

    if (!network_weights_file_[5].is_open())
    {
        std::cout << "Can not find the b_linear file" << std::endl;
    }
    else
    {
        index = 0;
        while (!network_weights_file_[5].eof())
        {
            network_weights_file_[5] >> temp;
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.b_linear(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: b_linear has NaN value!" << endl;
                }
            }
        }
        network_weights_file_[5].close();
    }

    // cout<<"lstm.b_ih: \n"<<lstm.b_ih<<endl;
    // cout<<"lstm.W_ih: \n"<<lstm.W_ih<<endl;
    // cout<<"lstm.b_hh: \n"<<lstm.b_hh<<endl;
    // cout<<"lstm.b_linear: \n"<<lstm.b_linear<<endl;
}
void AvatarController::loadLstmMeanStd(LSTM &lstm, std::string folder_path)
{
    std::string input_mean_path("input_mean.txt");
    std::string input_std_path("input_std.txt");
    std::string output_mean_path("output_mean.txt");
    std::string output_std_path("output_std.txt");

    input_mean_path = folder_path + input_mean_path;
    input_std_path = folder_path + input_std_path;
    output_mean_path = folder_path + output_mean_path;
    output_std_path = folder_path + output_std_path;

    mean_std_file_[0].open(input_mean_path, ios::in);
    mean_std_file_[1].open(input_std_path, ios::in);
    mean_std_file_[2].open(output_mean_path, ios::in);
    mean_std_file_[3].open(output_std_path, ios::in);

    int index = 0;
    float temp;

    if (mean_std_file_[0].is_open())
    {
        while (mean_std_file_[0] >> temp)
        {
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.input_mean(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: input_mean has NaN value!" << endl;
                }
            }
        }
    }
    else
    {
        std::cout << "Can not find the 'input_mean.txt' file" << std::endl;
    }
    mean_std_file_[0].close();

    index = 0;
    if (mean_std_file_[1].is_open())
    {
        while (mean_std_file_[1] >> temp)
        {
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.input_std(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: input_std has NaN value!" << endl;
                }
            }
        }
    }
    else
    {
        std::cout << "Can not find the 'input_std.txt' file" << std::endl;
    }
    mean_std_file_[1].close();

    index = 0;
    if (mean_std_file_[2].is_open())
    {
        while (mean_std_file_[2] >> temp)
        {
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.output_mean(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: output_mean has NaN value!" << endl;
                }
            }
        }
    }
    else
    {
        std::cout << "Can not find the 'output_mean.txt' file" << std::endl;
    }
    mean_std_file_[2].close();

    index = 0;
    if (mean_std_file_[3].is_open())
    {
        while (mean_std_file_[3] >> temp)
        {
            if (temp != '\n')
            {
                if (temp = temp)
                {
                    lstm.output_std(index) = temp;
                    index++;
                }
                else
                {
                    cout << "WARNING: output_std has NaN value!" << endl;
                }
            }
        }
    }
    else
    {
        std::cout << "Can not find the 'output_std.txt' file" << std::endl;
    }
    mean_std_file_[3].close();

    if (gaussian_mode_)
    {
        for (int i = int(lstm.n_output / 2); i < lstm.n_output; i++)
        {
            lstm.output_std(i) = 1.0;
            lstm.output_mean(i) = 0.0;
        }
    }
    // cout<<"lstm.input_mean: \n"<<lstm.input_mean<<endl;
    // cout<<"lstm.input_std: \n"<<lstm.input_std<<endl;
    cout << "lstm.output_mean: \n"
         << lstm.output_mean << endl;
    cout << "lstm.output_std: \n"
         << lstm.output_std << endl;
}
void AvatarController::collectRobotInputData_acc_version()
{
    /////////left leg mob lstm//////////////////
    // left_leg_mob_lstm_.robot_input_data(0) = q_virtual_Xd_global_noise_(39); // quat
    // left_leg_mob_lstm_.robot_input_data(1) = q_virtual_Xd_global_noise_(3);
    // left_leg_mob_lstm_.robot_input_data(2) = q_virtual_Xd_global_noise_(4);
    // left_leg_mob_lstm_.robot_input_data(3) = q_virtual_Xd_global_noise_(5);

    left_leg_mob_lstm_.robot_input_data(0) = q_virtual_Xd_global_noise_(6); // q
    left_leg_mob_lstm_.robot_input_data(1) = q_virtual_Xd_global_noise_(7);
    left_leg_mob_lstm_.robot_input_data(2) = q_virtual_Xd_global_noise_(8);
    left_leg_mob_lstm_.robot_input_data(3) = q_virtual_Xd_global_noise_(9);
    left_leg_mob_lstm_.robot_input_data(4) = q_virtual_Xd_global_noise_(10);
    left_leg_mob_lstm_.robot_input_data(5) = q_virtual_Xd_global_noise_(11);

    left_leg_mob_lstm_.robot_input_data(6) = q_dot_virtual_Xd_global_noise_(3); // pelv ang vel
    left_leg_mob_lstm_.robot_input_data(7) = q_dot_virtual_Xd_global_noise_(4);
    left_leg_mob_lstm_.robot_input_data(8) = q_dot_virtual_Xd_global_noise_(5);

    left_leg_mob_lstm_.robot_input_data(9) = q_dot_virtual_Xd_global_noise_(6); // qdot
    left_leg_mob_lstm_.robot_input_data(10) = q_dot_virtual_Xd_global_noise_(7);
    left_leg_mob_lstm_.robot_input_data(11) = q_dot_virtual_Xd_global_noise_(8);
    left_leg_mob_lstm_.robot_input_data(12) = q_dot_virtual_Xd_global_noise_(9);
    left_leg_mob_lstm_.robot_input_data(13) = q_dot_virtual_Xd_global_noise_(10);
    left_leg_mob_lstm_.robot_input_data(14) = q_dot_virtual_Xd_global_noise_(11);

    left_leg_mob_lstm_.robot_input_data(15) = rd_.torque_desired(0); // desired torque
    left_leg_mob_lstm_.robot_input_data(16) = rd_.torque_desired(1);
    left_leg_mob_lstm_.robot_input_data(17) = rd_.torque_desired(2);
    left_leg_mob_lstm_.robot_input_data(18) = rd_.torque_desired(3);
    left_leg_mob_lstm_.robot_input_data(19) = rd_.torque_desired(4);
    left_leg_mob_lstm_.robot_input_data(20) = rd_.torque_desired(5);

    left_leg_mob_lstm_.robot_input_data(21) = q_ddot_virtual_Xd_global_noise_(0); // lin acc
    left_leg_mob_lstm_.robot_input_data(22) = q_ddot_virtual_Xd_global_noise_(1);
    left_leg_mob_lstm_.robot_input_data(23) = q_ddot_virtual_Xd_global_noise_(2);

    /////////right leg mob lstm//////////////////
    // right_leg_mob_lstm_.robot_input_data(0) = rd_.q_virtual_(39); // quat
    // right_leg_mob_lstm_.robot_input_data(1) = rd_.q_virtual_(3);
    // right_leg_mob_lstm_.robot_input_data(2) = rd_.q_virtual_(4);
    // right_leg_mob_lstm_.robot_input_data(3) = rd_.q_virtual_(5);

    right_leg_mob_lstm_.robot_input_data(0) = q_virtual_Xd_global_noise_(12); // q
    right_leg_mob_lstm_.robot_input_data(1) = q_virtual_Xd_global_noise_(13);
    right_leg_mob_lstm_.robot_input_data(2) = q_virtual_Xd_global_noise_(14);
    right_leg_mob_lstm_.robot_input_data(3) = q_virtual_Xd_global_noise_(15);
    right_leg_mob_lstm_.robot_input_data(4) = q_virtual_Xd_global_noise_(16);
    right_leg_mob_lstm_.robot_input_data(5) = q_virtual_Xd_global_noise_(17);

    right_leg_mob_lstm_.robot_input_data(6) = q_dot_virtual_Xd_global_noise_(3); // pelv ang vel
    right_leg_mob_lstm_.robot_input_data(7) = q_dot_virtual_Xd_global_noise_(4);
    right_leg_mob_lstm_.robot_input_data(8) = q_dot_virtual_Xd_global_noise_(5);

    right_leg_mob_lstm_.robot_input_data(9) = q_dot_virtual_Xd_global_noise_(12); // qdot
    right_leg_mob_lstm_.robot_input_data(10) = q_dot_virtual_Xd_global_noise_(13);
    right_leg_mob_lstm_.robot_input_data(11) = q_dot_virtual_Xd_global_noise_(14);
    right_leg_mob_lstm_.robot_input_data(12) = q_dot_virtual_Xd_global_noise_(15);
    right_leg_mob_lstm_.robot_input_data(13) = q_dot_virtual_Xd_global_noise_(16);
    right_leg_mob_lstm_.robot_input_data(14) = q_dot_virtual_Xd_global_noise_(17);

    right_leg_mob_lstm_.robot_input_data(15) = rd_.torque_desired(6); // desired torque
    right_leg_mob_lstm_.robot_input_data(16) = rd_.torque_desired(7);
    right_leg_mob_lstm_.robot_input_data(17) = rd_.torque_desired(8);
    right_leg_mob_lstm_.robot_input_data(18) = rd_.torque_desired(9);
    right_leg_mob_lstm_.robot_input_data(19) = rd_.torque_desired(10);
    right_leg_mob_lstm_.robot_input_data(20) = rd_.torque_desired(11);

    right_leg_mob_lstm_.robot_input_data(21) = q_ddot_virtual_Xd_global_noise_(0); // lin acc
    right_leg_mob_lstm_.robot_input_data(22) = q_ddot_virtual_Xd_global_noise_(1);
    right_leg_mob_lstm_.robot_input_data(23) = q_ddot_virtual_Xd_global_noise_(2);

    /////// with q dot pre data
    // int tick_ago_head;
    // tick_ago_head = left_leg_mob_lstm_.buffer_head - 19*left_leg_mob_lstm_.n_input; //10ms ago tail for 100hz input

    // if(tick_ago_head < 0)
    // {
    //     tick_ago_head += left_leg_mob_lstm_.buffer_size;
    // }
    // left_leg_mob_lstm_.robot_input_data(19) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 10);    //pelv ang vel 10ms ago
    // left_leg_mob_lstm_.robot_input_data(20) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 11);
    // left_leg_mob_lstm_.robot_input_data(21) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 12);

    // left_leg_mob_lstm_.robot_input_data(22) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 13);       //qdot 10ms ago
    // left_leg_mob_lstm_.robot_input_data(23) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 14);
    // left_leg_mob_lstm_.robot_input_data(24) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 15);
    // left_leg_mob_lstm_.robot_input_data(25) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 16);
    // left_leg_mob_lstm_.robot_input_data(26) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 17);
    // left_leg_mob_lstm_.robot_input_data(27) = left_leg_mob_lstm_.ring_buffer(tick_ago_head -(left_leg_mob_lstm_.n_input-1) + 18);

    // left_leg_mob_lstm_.robot_input_data(28) = rd_.q_ddot_virtual_(0);    //lin acc
    // left_leg_mob_lstm_.robot_input_data(29) = rd_.q_ddot_virtual_(1);
    // left_leg_mob_lstm_.robot_input_data(30) = rd_.q_ddot_virtual_(2);
}
void AvatarController::calculateLstmInput(LSTM &lstm)
{
    // index update
    lstm.buffer_head += lstm.n_input;
    lstm.buffer_tail += lstm.n_input;
    lstm.buffer_head = lstm.buffer_head % lstm.buffer_size;
    lstm.buffer_tail = lstm.buffer_tail % lstm.buffer_size;

    lstm.input_mode_idx += 1;
    lstm.input_mode_idx = lstm.input_mode_idx % 20;

    int input_data_size = lstm.robot_input_data.size();

    // ring buffer update
    if (lstm.n_input == input_data_size)
    {
        for (int i = 0; i < lstm.n_input; i++)
        {
            lstm.ring_buffer(lstm.buffer_head - lstm.n_input + 1 + i) = lstm.robot_input_data(i);
        }
    }
    else
    {
        cout << "ERROR: input data to the NN has different size with the NN input size." << endl;
    }

    // nn input generation
    for (int i = 0; i < lstm.n_sequence_length; i++)
    {
        for (int j = 0; j < lstm.n_input; j++)
        {
            int buffer_idx = lstm.buffer_head - j - lstm.n_input * 20 * i;
            if (buffer_idx < 0)
                buffer_idx += buffer_size_;
            buffer_idx = buffer_idx % buffer_size_;

            lstm.input_slow(lstm.nn_input_size - j - i * lstm.n_input - 1, lstm.input_mode_idx) = (lstm.ring_buffer(buffer_idx) - lstm.input_mean(lstm.n_input - 1 - j)) / lstm.input_std(lstm.n_input - 1 - j);
        }
    }

    // if(walking_tick_mj%2000 == 0)
    // {
    //     cout<<"lstm.input_slow: "<<lstm.input_slow.transpose()<<endl;
    // }
    if (lstm.atb_lstm_input_update_ == false)
    {
        lstm.atb_lstm_input_update_ = true;
        lstm.input_thread = lstm.input_slow;
        lstm.atb_lstm_input_update_ = false;
    }
}
void AvatarController::calculateLstmOutput(LSTM &lstm)
{

    // get LSTM input from main thread
    if (lstm.atb_lstm_input_update_ == false)
    {
        lstm.atb_lstm_input_update_ = true;
        lstm.input_fast = lstm.input_thread;
        lstm.atb_lstm_input_update_ = false;
    }
    // lstm.hidden.setZero();
    // lstm.cell.setZero();

    while (lstm.input_mode_idx != lstm.output_mode_idx)
    {
        lstm.output_mode_idx += 1;
        lstm.output_mode_idx = lstm.output_mode_idx % 20;

        // LSTM layer
        for (int seq = 0; seq < lstm.n_sequence_length; seq++)
        {
            for (int row_idx = 0; row_idx < 4 * lstm.n_hidden; row_idx++)
            {
                lstm.gates(row_idx) = lstm.b_ih(row_idx) + lstm.b_hh(row_idx);
                for (int input_idx = 0; input_idx < lstm.n_input; input_idx++)
                    lstm.gates(row_idx) += lstm.W_ih(lstm.n_input * row_idx + input_idx) * lstm.input_fast(seq * lstm.n_input + input_idx, lstm.output_mode_idx);
                for (int hidden_idx = 0; hidden_idx < lstm.n_hidden; hidden_idx++)
                    lstm.gates(row_idx) += lstm.W_hh(lstm.n_hidden * row_idx + hidden_idx) * lstm.hidden(hidden_idx, lstm.output_mode_idx);
            }

            for (int row_idx = 0; row_idx < lstm.n_hidden; row_idx++)
            {
                lstm.input_gate(row_idx) = 1 / (1 + std::exp(-lstm.gates(row_idx)));
                lstm.forget_gate(row_idx) = 1 / (1 + std::exp(-lstm.gates(lstm.n_hidden + row_idx)));
                lstm.cell_gate(row_idx) = std::tanh(lstm.gates(2 * lstm.n_hidden + row_idx));
                lstm.output_gate(row_idx) = 1 / (1 + std::exp(-lstm.gates(3 * lstm.n_hidden + row_idx)));

                lstm.cell(row_idx, lstm.output_mode_idx) = lstm.forget_gate(row_idx) * lstm.cell(row_idx, lstm.output_mode_idx) + lstm.input_gate(row_idx) * lstm.cell_gate(row_idx);
                lstm.hidden(row_idx, lstm.output_mode_idx) = lstm.output_gate(row_idx) * std::tanh(lstm.cell(row_idx, lstm.output_mode_idx));
            }
        }
    }

    // Linear layer
    for (int joint = 0; joint < lstm.n_output; joint++)
    {
        lstm.output(joint) = lstm.b_linear(joint);
        for (int hidden_idx = 0; hidden_idx < lstm.n_hidden; hidden_idx++)
        {
            lstm.output(joint) += lstm.W_linear(joint * lstm.n_hidden + hidden_idx) * lstm.hidden(hidden_idx, lstm.output_mode_idx);
        }
    }

    // unnormalize
    for (int joint = 0; joint < lstm.n_output; joint++)
    {
        lstm.real_output(joint) = lstm.output(joint) * lstm.output_std(joint) + lstm.output_mean(joint);
    }

    // softplus
    if (gaussian_mode_)
    {
        double beta = 1;
        double threashold = 20;
        for (int i = int(lstm.n_output / 2); i < lstm.n_output; i++)
        {
            double input;
            input = beta * lstm.real_output(i);
            if (input > threashold)
            {
                lstm.real_output(i) = lstm.real_output(i); //*lstm.output_std(i-int(lstm.n_output/2))*lstm.output_std(i-int(lstm.n_output/2));
            }
            else
            {
                lstm.real_output(i) = std::log(1 + std::exp(input)) / beta; // softplus
                // lstm.real_output(i) = lstm.real_output(i)*lstm.output_std(i-int(lstm.n_output/2))*lstm.output_std(i-int(lstm.n_output/2));    // variance unnormalize
            }

            lstm.real_output(i) = lstm.real_output(i) * lstm.output_std(i-int(lstm.n_output / 2)) * lstm.output_std(i-int(lstm.n_output / 2));
        }
    }
}

void AvatarController::initializeLegGRU(GRU &gru, int n_input, int n_output, int n_hidden)
{
    // set network variables
    gru.n_input = n_input;
    gru.n_output = n_output;
    gru.n_hidden = n_hidden;
    //

    gru.buffer_size = gru.n_input * 20;

    gru.input_mode_idx = 19;
    gru.output_mode_idx = 19;

    gru.ring_buffer.setZero(gru.buffer_size);
    gru.buffer_head = gru.buffer_size - 1;
    gru.buffer_tail = 0;

    gru.input_slow.setZero(gru.n_input, 20);
    gru.input_fast.setZero(gru.n_input, 20);
    gru.input_thread.setZero(gru.n_input, 20);
    gru.input_mean.setZero(gru.n_input);
    gru.input_std.setZero(gru.n_input);

    gru.robot_input_data.setZero(gru.n_input);

    gru.output.setZero(gru.n_output);
    gru.real_output.setZero(gru.n_output); // real out with physical dimension
    gru.output_mean.setZero(gru.n_output);
    gru.output_std.setZero(gru.n_output);

    gru.W_ih.setZero(3 * gru.n_hidden, gru.n_input);
    gru.b_ih.setZero(3 * gru.n_hidden);
    gru.W_hh.setZero(3 * gru.n_hidden, gru.n_hidden);
    gru.b_hh.setZero(3 * gru.n_hidden);
    gru.W_linear.setZero(gru.n_output, gru.n_hidden);
    gru.b_linear.setZero(gru.n_output);

    gru.h_t.setZero(gru.n_hidden, 20);
    gru.r_t.setZero(gru.n_hidden);
    gru.z_t.setZero(gru.n_hidden);
    gru.n_t.setZero(gru.n_hidden);
}
void AvatarController::collectRobotInputData_peter_gru()
{
    left_leg_peter_gru_.robot_input_data(0) = rd_.q_virtual_(6); // q
    left_leg_peter_gru_.robot_input_data(1) = rd_.q_virtual_(7);
    left_leg_peter_gru_.robot_input_data(2) = rd_.q_virtual_(8);
    left_leg_peter_gru_.robot_input_data(3) = rd_.q_virtual_(9);
    left_leg_peter_gru_.robot_input_data(4) = rd_.q_virtual_(10);
    left_leg_peter_gru_.robot_input_data(5) = rd_.q_virtual_(11);

    left_leg_peter_gru_.robot_input_data(6) = rd_.q_dot_virtual_(3); // pelv ang vel
    left_leg_peter_gru_.robot_input_data(7) = rd_.q_dot_virtual_(4);
    left_leg_peter_gru_.robot_input_data(8) = rd_.q_dot_virtual_(5);

    left_leg_peter_gru_.robot_input_data(9) = rd_.q_dot_virtual_(6); // qdot
    left_leg_peter_gru_.robot_input_data(10) = rd_.q_dot_virtual_(7);
    left_leg_peter_gru_.robot_input_data(11) = rd_.q_dot_virtual_(8);
    left_leg_peter_gru_.robot_input_data(12) = rd_.q_dot_virtual_(9);
    left_leg_peter_gru_.robot_input_data(13) = rd_.q_dot_virtual_(10);
    left_leg_peter_gru_.robot_input_data(14) = rd_.q_dot_virtual_(11);

    left_leg_peter_gru_.robot_input_data(15) = rd_.torque_desired(0); // desired torque
    left_leg_peter_gru_.robot_input_data(16) = rd_.torque_desired(1);
    left_leg_peter_gru_.robot_input_data(17) = rd_.torque_desired(2);
    left_leg_peter_gru_.robot_input_data(18) = rd_.torque_desired(3);
    left_leg_peter_gru_.robot_input_data(19) = rd_.torque_desired(4);
    left_leg_peter_gru_.robot_input_data(20) = rd_.torque_desired(5);

    left_leg_peter_gru_.robot_input_data(21) = rd_.q_ddot_virtual_(0); // lin acc
    left_leg_peter_gru_.robot_input_data(22) = rd_.q_ddot_virtual_(1);
    left_leg_peter_gru_.robot_input_data(23) = rd_.q_ddot_virtual_(2);

    /////////right leg //////////////////

    right_leg_peter_gru_.robot_input_data(0) = rd_.q_virtual_(12); // q
    right_leg_peter_gru_.robot_input_data(1) = rd_.q_virtual_(13);
    right_leg_peter_gru_.robot_input_data(2) = rd_.q_virtual_(14);
    right_leg_peter_gru_.robot_input_data(3) = rd_.q_virtual_(15);
    right_leg_peter_gru_.robot_input_data(4) = rd_.q_virtual_(16);
    right_leg_peter_gru_.robot_input_data(5) = rd_.q_virtual_(17);

    right_leg_peter_gru_.robot_input_data(6) = rd_.q_dot_virtual_(3); // pelv ang vel
    right_leg_peter_gru_.robot_input_data(7) = rd_.q_dot_virtual_(4);
    right_leg_peter_gru_.robot_input_data(8) = rd_.q_dot_virtual_(5);

    right_leg_peter_gru_.robot_input_data(9) = rd_.q_dot_virtual_(12); // qdot
    right_leg_peter_gru_.robot_input_data(10) = rd_.q_dot_virtual_(13);
    right_leg_peter_gru_.robot_input_data(11) = rd_.q_dot_virtual_(14);
    right_leg_peter_gru_.robot_input_data(12) = rd_.q_dot_virtual_(15);
    right_leg_peter_gru_.robot_input_data(13) = rd_.q_dot_virtual_(16);
    right_leg_peter_gru_.robot_input_data(14) = rd_.q_dot_virtual_(17);

    right_leg_peter_gru_.robot_input_data(15) = rd_.torque_desired(6); // desired torque
    right_leg_peter_gru_.robot_input_data(16) = rd_.torque_desired(7);
    right_leg_peter_gru_.robot_input_data(17) = rd_.torque_desired(8);
    right_leg_peter_gru_.robot_input_data(18) = rd_.torque_desired(9);
    right_leg_peter_gru_.robot_input_data(19) = rd_.torque_desired(10);
    right_leg_peter_gru_.robot_input_data(20) = rd_.torque_desired(11);

    right_leg_peter_gru_.robot_input_data(21) = rd_.q_ddot_virtual_(0); // lin acc
    right_leg_peter_gru_.robot_input_data(22) = rd_.q_ddot_virtual_(1);
    right_leg_peter_gru_.robot_input_data(23) = rd_.q_ddot_virtual_(2);
}
void AvatarController::loadGruWeights(GRU &gru, std::string folder_path)
{
    std::string W_ih_path("gru_weight_ih_l0.txt");
    std::string W_hh_path("gru_weight_hh_l0.txt");
    std::string W_linear_path("linear_weight.txt");

    std::string b_ih_path("gru_bias_ih_l0.txt");
    std::string b_hh_path("gru_bias_hh_l0.txt");
    std::string b_linear_path("linear_bias.txt");

    W_ih_path = folder_path + W_ih_path;
    W_hh_path = folder_path + W_hh_path;
    W_linear_path = folder_path + W_linear_path;

    b_ih_path = folder_path + b_ih_path;
    b_hh_path = folder_path + b_hh_path;
    b_linear_path = folder_path + b_linear_path;

    gru.network_weights_files[0].open(W_ih_path, ios::in);
    gru.network_weights_files[1].open(W_hh_path, ios::in);
    gru.network_weights_files[2].open(W_linear_path, ios::in);

    gru.bias_files[0].open(b_ih_path, ios::in);
    gru.bias_files[1].open(b_hh_path, ios::in);
    gru.bias_files[2].open(b_linear_path, ios::in);

    // W_ih
    if (!gru.network_weights_files[0].is_open())
    {
        std::cout << "Can not find the file: " << W_ih_path << std::endl;
    }

    for (int i = 0; i < gru.W_ih.rows(); i++)
    {
        for (int j = 0; j < gru.W_ih.cols(); j++)
        {
            gru.network_weights_files[0] >> gru.W_ih(i, j);
        }
    }
    gru.network_weights_files[0].close();

    if (gru.loadweightfile_verbose == true)
    {
        cout << "weight_ih: \n"
             << gru.W_ih << endl;
    }

    // W_hh
    if (!gru.network_weights_files[1].is_open())
    {
        std::cout << "Can not find the file: " << W_hh_path << std::endl;
    }

    for (int i = 0; i < gru.W_hh.rows(); i++)
    {
        for (int j = 0; j < gru.W_hh.cols(); j++)
        {
            gru.network_weights_files[1] >> gru.W_hh(i, j);
        }
    }
    gru.network_weights_files[1].close();

    if (gru.loadweightfile_verbose == true)
    {
        cout << "weight_hh: \n"
             << gru.W_hh << endl;
    }

    // W_linear
    if (!gru.network_weights_files[2].is_open())
    {
        std::cout << "Can not find the file: " << W_linear_path << std::endl;
    }

    for (int i = 0; i < gru.W_linear.rows(); i++)
    {
        for (int j = 0; j < gru.W_linear.cols(); j++)
        {
            gru.network_weights_files[2] >> gru.W_linear(i, j);
        }
    }
    gru.network_weights_files[2].close();

    if (gru.loadweightfile_verbose == true)
    {
        cout << "weight_linear: \n"
             << gru.W_linear << endl;
    }

    // b_ih
    if (!gru.bias_files[0].is_open())
    {
        std::cout << "Can not find the file: " << b_ih_path << std::endl;
    }

    for (int i = 0; i < gru.b_ih.rows(); i++)
    {

        gru.bias_files[0] >> gru.b_ih(i);
    }
    gru.bias_files[0].close();

    if (gru.loadweightfile_verbose == true)
    {
        cout << "bias_ih: \n"
             << gru.b_ih.transpose() << endl;
    }

    // b_hh
    if (!gru.bias_files[1].is_open())
    {
        std::cout << "Can not find the file: " << b_hh_path << std::endl;
    }

    for (int i = 0; i < gru.b_hh.rows(); i++)
    {

        gru.bias_files[1] >> gru.b_hh(i);
    }
    gru.bias_files[1].close();

    if (gru.loadweightfile_verbose == true)
    {
        cout << "bias_hh: \n"
             << gru.b_hh.transpose() << endl;
    }

    // b_linear
    if (!gru.bias_files[2].is_open())
    {
        std::cout << "Can not find the file: " << b_linear_path << std::endl;
    }

    for (int i = 0; i < gru.b_linear.rows(); i++)
    {

        gru.bias_files[2] >> gru.b_linear(i);
    }
    gru.bias_files[2].close();

    if (gru.loadweightfile_verbose == true)
    {
        cout << "bias_linear: \n"
             << gru.b_linear.transpose() << endl;
    }
}
void AvatarController::loadGruMeanStd(GRU &gru, std::string folder_path)
{
    std::string input_mean_path("input_mean.txt");
    std::string input_std_path("input_std.txt");
    std::string output_mean_path("output_mean.txt");
    std::string output_std_path("output_std.txt");

    input_mean_path = folder_path + input_mean_path;
    input_std_path = folder_path + input_std_path;
    output_mean_path = folder_path + output_mean_path;
    output_std_path = folder_path + output_std_path;

    gru.mean_std_files[0].open(input_mean_path, ios::in);
    gru.mean_std_files[1].open(input_std_path, ios::in);
    gru.mean_std_files[2].open(output_mean_path, ios::in);
    gru.mean_std_files[3].open(output_std_path, ios::in);

    // input_mean
    if (!gru.mean_std_files[0].is_open())
    {
        std::cout << "Can not find the file: " << input_mean_path << std::endl;
    }

    for (int i = 0; i < gru.n_input; i++)
    {
        gru.mean_std_files[0] >> gru.input_mean(i);
    }
    gru.mean_std_files[0].close();

    

    // input_std
    if (!gru.mean_std_files[1].is_open())
    {
        std::cout << "Can not find the file: " << input_std_path << std::endl;
    }

    for (int i = 0; i < gru.n_input; i++)
    {
        gru.mean_std_files[1] >> gru.input_std(i);
    }
    gru.mean_std_files[1].close();


    // output_mean
    if (!gru.mean_std_files[2].is_open())
    {
        std::cout << "Can not find the file: " << output_mean_path << std::endl;
    }

    for (int i = 0; i < gru.n_output; i++)
    {
        gru.mean_std_files[2] >> gru.output_mean(i);
    }
    gru.mean_std_files[2].close();


    // input_std
    if (!gru.mean_std_files[3].is_open())
    {
        std::cout << "Can not find the file: " << output_std_path << std::endl;
    }

    for (int i = 0; i < gru.n_output; i++)
    {
        gru.mean_std_files[3] >> gru.output_std(i);
    }
    gru.mean_std_files[3].close();

    // gaussian mode
    if (gru.gaussian_mode)
    {
        for (int i = int(gru.n_output / 2); i < gru.n_output; i++)
        {
            gru.output_std(i) = 1.0;
            gru.output_mean(i) = 0.0;
        }
    }

    // print
    if (gru.loadmeanstdfile_verbose == true)
    {
        cout << "input_mean: \n"
             << gru.input_mean.transpose() << endl;

        cout << "input_std: \n"
             << gru.input_std.transpose() << endl;

        cout << "output_mean: \n"
             << gru.output_mean.transpose() << endl;

        cout << "output_std: \n"
             << gru.output_std.transpose() << endl;
    }
}
void AvatarController::calculateGruInput(GRU &gru)
{
    // index up{date
    gru.buffer_head += gru.n_input;
    gru.buffer_tail += gru.n_input;
    gru.buffer_head = gru.buffer_head % gru.buffer_size;
    gru.buffer_tail = gru.buffer_tail % gru.buffer_size;

    gru.input_mode_idx += 1;
    gru.input_mode_idx = gru.input_mode_idx % 20;

    int input_data_size = gru.robot_input_data.size();

    // ring buffer update
    if (gru.n_input == input_data_size)
    {
        for (int i = 0; i < gru.n_input; i++)
        {
            gru.ring_buffer(gru.buffer_head - gru.n_input + 1 + i) = gru.robot_input_data(i);
        }
    }
    else
    {
        cout << "ERROR: input data to the NN has different size with the NN input size." << endl;
    }

    // nn input generation

    for (int j = 0; j < gru.n_input; j++)
    {
        int buffer_idx = gru.buffer_head - j ;
        if (buffer_idx < 0)
            buffer_idx += gru.buffer_size;
        buffer_idx = buffer_idx % gru.buffer_size;

        gru.input_slow(gru.n_input - j - 1, gru.input_mode_idx) = (gru.ring_buffer(buffer_idx) - gru.input_mean(gru.n_input - 1 - j)) / gru.input_std(gru.n_input - 1 - j);
    }

    // if(walking_tick_mj%2000 == 0)
    // {
    //     cout<<"gru.input_slow: "<<gru.input_slow.transpose()<<endl;
    // }
    if (gru.atb_gru_input_update_ == false)
    {
        gru.atb_gru_input_update_ = true;
        gru.input_thread = gru.input_slow;
        gru.atb_gru_input_update_ = false;
    }
}
void AvatarController::calculateGruOutput(GRU &gru)
{
    // get GRU input from main thread
    if (gru.atb_gru_input_update_ == false)
    {
        gru.atb_gru_input_update_ = true;
        gru.input_fast = gru.input_thread;
        gru.atb_gru_input_update_ = false;
    }

    while (gru.input_mode_idx != gru.output_mode_idx)
    {
        gru.output_mode_idx += 1;
        gru.output_mode_idx = gru.output_mode_idx % 20;
        // GRU network
        gru.r_t = vecSigmoid( gru.W_ih.block(0, 0, gru.n_hidden, gru.n_input)*gru.input_fast.col(gru.output_mode_idx) + gru.b_ih.segment(0, gru.n_hidden)
                + gru.W_hh.block(0, 0, gru.n_hidden, gru.n_hidden)*gru.h_t.col(gru.output_mode_idx) + gru.b_hh.segment(0, gru.n_hidden) );

        gru.z_t = vecSigmoid( gru.W_ih.block(gru.n_hidden, 0, gru.n_hidden, gru.n_input)*gru.input_fast.col(gru.output_mode_idx) + gru.b_ih.segment(gru.n_hidden, gru.n_hidden)
                + gru.W_hh.block(gru.n_hidden, 0, gru.n_hidden, gru.n_hidden)*gru.h_t.col(gru.output_mode_idx) + gru.b_hh.segment(gru.n_hidden, gru.n_hidden) );
        VectorXd in_gate = gru.W_ih.block(2*gru.n_hidden, 0, gru.n_hidden, gru.n_input)*gru.input_fast.col(gru.output_mode_idx) + gru.b_ih.segment(2*gru.n_hidden, gru.n_hidden);
        VectorXd hn_gate = gru.W_hh.block(2*gru.n_hidden, 0, gru.n_hidden, gru.n_hidden)*gru.h_t.col(gru.output_mode_idx) + gru.b_hh.segment(2*gru.n_hidden, gru.n_hidden);
        VectorXd r_hn;
        r_hn.setZero(gru.n_hidden);
        for(int i=0; i<gru.n_hidden; i++)
        {
            r_hn(i) = gru.r_t(i)*hn_gate(i);
        }
        
        gru.n_t = vecTanh(in_gate + r_hn);
        for(int i=0; i<gru.n_hidden; i++)
        {
            gru.h_t(i, gru.output_mode_idx) = (1 - gru.z_t(i))*gru.n_t(i) + gru.z_t(i)*gru.h_t(i, gru.output_mode_idx);
        } 
    }

    // linear
    gru.output = gru.W_linear*gru.h_t.col(gru.output_mode_idx) + gru.b_linear;

    // unnormalize the mean
    for (int i = 0; i < gru.n_output; i++)
    {
        gru.real_output(i) = gru.output(i) * gru.output_std(i) + gru.output_mean(i);
    }
     
    // softplus
    if (gru.gaussian_mode)
    {
        double beta = 1;
        double threashold = 20;
        for (int i = int(gru.n_output / 2); i < gru.n_output; i++)
        {
            double input;
            input = beta * gru.real_output(i);
            if (input > threashold)
            {
                gru.real_output(i) = gru.real_output(i); 
            }
            else
            {
                gru.real_output(i) = std::log(1 + std::exp(input)) / beta; // softplus
            }

            gru.real_output(i) = gru.real_output(i) * gru.output_std(i-int(gru.n_output / 2)) * gru.output_std(i-int(gru.n_output / 2));
        }
    }
}
Eigen::VectorXd AvatarController::vecSigmoid(VectorXd input)
{
    int n = input.size();
    Eigen::VectorXd output;
    output.setZero(n);
 
    for(int i = 0; i < n; i++)
    {
        output(i) = 1 / ( 1 + std::exp( -input(i) ) );
    }

    return output;
}
Eigen::VectorXd AvatarController::vecTanh(VectorXd input)
{
    int n = input.size();
    Eigen::VectorXd output;
    output.setZero(n);

    for(int i = 0; i < n; i++)
    {
        output(i) = std::tanh(input(i));
    }

    return output;
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
void AvatarController::JoystickCommandCallback(const sensor_msgs::Joy &msg)
{
    joy_left_stick_(0) = msg.axes[0];   // horizontal
    joy_left_stick_(1) = msg.axes[1];   // vertical

    joy_right_stick_(0) = msg.axes[3];  // horizontal
    joy_right_stick_(1) = msg.axes[4];  // vertical

    for(int i = 0; i <11; i++)
    {
        joy_buttons_raw_(i) = msg.buttons[i];  //continue walking (A)
    }

    double max_step_l_x = 0.15;
    double max_step_l_y = 0.5;
    double max_yaw_angle = 10*DEG2RAD;

    double del_x = joy_left_stick_(1)*max_step_l_x;
    double del_y = joy_left_stick_(0)*max_step_l_y;
    double yaw_angle = joy_right_stick_(0)*max_yaw_angle;

    for(int i = 0; i <joy_command_buffer_size_-1; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            joy_command_buffer_(j, i+1) = joy_command_buffer_(j, i); //push back buffer data
        }
    }
    joy_command_buffer_(0, 0) = del_x;
    joy_command_buffer_(1, 0) = del_y;
    joy_command_buffer_(2, 0) = yaw_angle;

    del_x_command_ = joy_command_buffer_.row(0).mean();
    del_y_command_ = joy_command_buffer_.row(1).mean();
    yaw_angle_command_ = joy_command_buffer_.row(2).mean();

    bool any_command = abs(del_x_command_)+abs(del_y_command_)+abs(yaw_angle_command_);

    if (any_command && joy_input_enable_)
    {
        walking_enable_ = true;
        walking_stop_flag_ = false;
        walking_end_flag = 1;
        stopping_step_planning_trigger_ = true;
        // cout<<"walking triggered!!"<<endl;
    }
    else
    {
        if(!joy_continuous_walking_flag_)
        {
            walking_stop_flag_ =  true;
        }
    }
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
    double dead_zone_r1 = 0.15; // FW BW dead zone
    double dead_zone_r2 = 0.15; // theta zone

    if (joy_input_enable_ == true)
    {
        joystick_input(0) = DyrosMath::minmax_cut(1 / (1 - dead_zone_r1) * (msg->step_length_x) - dead_zone_r1 / (1 - dead_zone_r1), -1.0, 1.0); // FW
        joystick_input(2) = DyrosMath::minmax_cut(1 / (1 - dead_zone_r2) * (msg->theta) - DyrosMath::sign(msg->theta) * dead_zone_r2 / (1 - dead_zone_r2), -0.5 + 0.5 * DyrosMath::sign(msg->theta), 0.5 + 0.5 * DyrosMath::sign(msg->theta));
        // joystick_input(2) = msg->theta;
        joystick_input(3) = DyrosMath::minmax_cut(1 / (1 - dead_zone_r1) * (msg->z) - dead_zone_r1 / (1 - dead_zone_r1), -1.0, 1.0); // BW
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
        // cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;

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
    __q_dot_virtual = rd_.q_dot_virtual_;
    __q_virtual = rd_.q_virtual_;
    __q_ddot_virtual = rd_.q_ddot_virtual_;
    for (int i=0;i<LINK_NUMBER + 1; i++)
    {
        link_avatar_[i] = rd_.link_[i];
    }

    // elmo current torque
    for (int i = 0; i < MODEL_DOF; i++)
    {
        torque_current_elmo_(i) = rd_.torque_elmo_(i); // motor current torque
    }

    l_ft_ = rd_.LF_FT; // generated force by robot left foot
    r_ft_ = rd_.RF_FT; // generated force by robot right foot

    opto_ft_ = opto_ft_raw_; // collision test exp
    
    // deleted by MJ/1209
    // lh_ft_ = rd_.LH_FT;
    // rh_ft_ = rd_.RH_FT;

    //////////// model_global_ UPDATE///////////////

    // hmd_chest_vel_slow_(0) = DyrosMath::minmax_cut(hmd_chest_vel_slow_(0), -1.5, 1.5);
    // hmd_chest_vel_slow_(1) = DyrosMath::minmax_cut(hmd_chest_vel_slow_(1), -1.5, 1.5);
    // hmd_chest_vel_slow_(2) = DyrosMath::minmax_cut(hmd_chest_vel_slow_(2), -0.4, 0.4);

    // Base frame is global
    if (walking_tick_mj == 0)
        q_dot_virtual_Xd_global_pre_ = rd_.q_dot_virtual_;
    q_dot_virtual_Xd_global_ = rd_.q_dot_virtual_;
    // q_dot_virtual_Xd_global_.segment(0, 3) = hmd_chest_vel_slow_lpf_.segment(0, 3);  //tracker
    q_virtual_Xd_global_ = rd_.q_virtual_;
    q_ddot_virtual_Xd_global_ = (q_dot_virtual_Xd_global_ - q_dot_virtual_Xd_global_pre_) * hz_;
    q_ddot_virtual_Xd_global_.segment(0, 6) = rd_.q_ddot_virtual_.segment(0, 6);

    q_virtual_Xd_global_noise_ = q_virtual_Xd_global_;
    q_dot_virtual_Xd_global_noise_ = q_dot_virtual_Xd_global_;
    q_ddot_virtual_Xd_global_noise_ = q_ddot_virtual_Xd_global_;
    // std::mt19937 generator(std::random_device{}());
    // auto dist_1 = std::bind(std::normal_distribution<double>{0.0, 0.1}, std::mt19937(std::random_device{}()));
    // auto dist_2 = std::bind(std::normal_distribution<double>{0.0, 0.01}, std::mt19937(std::random_device{}()));
    // auto dist_3 = std::bind(std::normal_distribution<double>{0.0, 0.001}, std::mt19937(std::random_device{}()));
    // auto dist_4 = std::bind(std::normal_distribution<double>{0.0, 0.0001}, std::mt19937(std::random_device{}()));
    // auto dist_5 = std::bind(std::normal_distribution<double>{0.0, 0.00001}, std::mt19937(std::random_device{}()));

    // std::default_random_engine generator;
    // std::normal_distribution<double> dist(0.0, 0.1);

    // for (int i = 6; i<39; i++)
    // {
    //     q_virtual_Xd_global_noise_(i) = q_virtual_Xd_global_noise_(i) + dist_5();
    // }
    // // cout<<"q_noise - q = "<<(q_virtual_Xd_global_noise_ - q_virtual_Xd_global_).transpose() << endl;

    // for (int i = 0; i<3; i++)
    // {
    //     q_dot_virtual_Xd_global_noise_(i) = q_dot_virtual_Xd_global_noise_(i) + dist_2();
    // }
    // for (int i = 3; i<6; i++)
    // {
    //     q_dot_virtual_Xd_global_noise_(i) = q_dot_virtual_Xd_global_noise_(i) + dist_3();
    // }
    // for (int i = 6; i<MODEL_DOF_VIRTUAL; i++)
    // {
    //     q_dot_virtual_Xd_global_noise_(i) = q_dot_virtual_Xd_global_noise_(i) + dist_2();
    // }

    // for (int i = 0; i<3; i++)
    // {
    //     q_ddot_virtual_Xd_global_noise_(i) = q_ddot_virtual_Xd_global_noise_(i) + dist_4();
    // }

    RigidBodyDynamics::UpdateKinematicsCustom(model_global_, &q_virtual_Xd_global_, &q_dot_virtual_Xd_global_, &q_ddot_virtual_Xd_global_);
    //////////////////////////////////////////////
    ////////////model_local_ UPDATE///////////////
    // Base frame is attatced to the pelvis
    if (walking_tick_mj == 0)
    {
        q_dot_virtual_Xd_local_pre_ = rd_.q_dot_virtual_;
        q_dot_virtual_Xd_local_pre_.segment(0, 6).setZero();
    }

    q_dot_virtual_Xd_local_ = rd_.q_dot_virtual_;
    q_virtual_Xd_local_ = rd_.q_virtual_;

    q_dot_virtual_Xd_local_.segment(0, 6).setZero();
    q_virtual_Xd_local_.segment(0, 6).setZero();
    q_virtual_Xd_local_(39) = 1;
    q_ddot_virtual_Xd_local_ = (q_dot_virtual_Xd_local_ - q_dot_virtual_Xd_local_pre_) * hz_;

    RigidBodyDynamics::UpdateKinematicsCustom(model_local_, &q_virtual_Xd_local_, &q_dot_virtual_Xd_local_, &q_ddot_virtual_Xd_local_); // global frame is fixed to the pelvis frame


    /////////////////////////////////////////////

    pelv_rpy_current_mj_.setZero();
    pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); // ZYX multiply

    R_angle = pelv_rpy_current_mj_(0);
    P_angle = pelv_rpy_current_mj_(1);
    pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
    lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

    pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

    pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);

    lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
    // lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

    rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
    // rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

    com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치
    com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].v);

    if (walking_tick_mj == 0)
    {
        com_float_current_dot_LPF = com_float_current_dot;
        com_float_current_dot_prev = com_float_current_dot;
    }

    com_float_current_dot_prev = com_float_current_dot;
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_dot;
    // modified cut off freq. of CP error for joe's MPC
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

    SC_err_compen(com_support_current_(0), com_support_current_(1));

    wn = sqrt(GRAVITY / zc_mj_);

    cp_measured_(0) = com_support_cp_(0) + com_float_current_dot_LPF(0) / wn;
    // cp_measured_(0) = com_support_current_(0) + com_float_current_dot_LPF(0) / wn;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;



    double foot_plate_mass = 2.326; // urdf

    // double foot_plate_mass = 1.866; // bolt: 41g, black plate: 211g, red plate: 1614g
    Matrix6d adt;
    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(rd_.link_[Left_Foot].sensor_point) * Matrix3d::Identity();
    Matrix6d rotrf;
    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = rd_.link_[Left_Foot].rotm;
    rotrf.block(3, 3, 3, 3) = rd_.link_[Left_Foot].rotm;
    Vector3d LF_com(0.0162, 0.00008, -0.1209);

    Vector3d com2cp = rd_.link_[Left_Foot].sensor_point - LF_com;

    Matrix6d adt2;
    adt2.setIdentity();
    adt2.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();

    Vector6d Wrench_foot_plate;
    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = -foot_plate_mass * 9.81;

    Matrix6d inertia_foot_plate; // left and right are same
    inertia_foot_plate.setZero();
    inertia_foot_plate.block(0, 0, 3, 3) = foot_plate_mass * Matrix3d::Identity();
    inertia_foot_plate(3, 3) = 0.003386409;
    inertia_foot_plate(4, 3) = 1.1237E-05;
    inertia_foot_plate(5, 3) = -0.000526477;
    inertia_foot_plate(3, 4) = 1.1237E-05;
    inertia_foot_plate(4, 4) = 0.012600125;
    inertia_foot_plate(5, 4) = -1.976E-06;
    inertia_foot_plate(3, 5) = -0.000526477;
    inertia_foot_plate(4, 5) = -1.976E-06;
    inertia_foot_plate(5, 5) = 0.014296;

    Vector6d lfoot_acceleration_temp, lfoot_acceleration;
    lfoot_acceleration_temp = RigidBodyDynamics::CalcPointAcceleration6D(model_global_, q_virtual_Xd_global_, q_dot_virtual_Xd_global_, q_ddot_virtual_Xd_global_, rd_.link_[Left_Foot].id, LF_com, false);
    lfoot_acceleration.segment(0, 3) = lfoot_acceleration_temp.segment(3, 3); // translation
    lfoot_acceleration.segment(3, 3) = lfoot_acceleration_temp.segment(0, 3); // rotation
    // l_ft_wo_fw_ = l_ft_ + adt2 * (rotrf.transpose() * Wrench_foot_plate) - adt2.transpose() * inertia_foot_plate * (rotrf.transpose() * lfoot_acceleration);
    l_ft_wo_fw_ = l_ft_; // tocabi
    l_ft_wo_fw_lpf_ = DyrosMath::lpf<6>(l_ft_wo_fw_, l_ft_wo_fw_lpf_, 2000, 100 / (2 * M_PI));

    // l_ft_wo_fw_ = l_ft_ + adt2*(rotrf.transpose()*Wrench_foot_plate);

    // if( walking_tick_mj%200 == 0)
    //     cout<<"lfoot_acceleration: "<<lfoot_acceleration.transpose() <<endl;

    adt.setIdentity();
    adt.block(3, 0, 3, 3) = DyrosMath::skm(rd_.link_[Right_Foot].sensor_point) * Matrix3d::Identity();
    rotrf.setZero();
    rotrf.block(0, 0, 3, 3) = rd_.link_[Right_Foot].rotm;
    rotrf.block(3, 3, 3, 3) = rd_.link_[Right_Foot].rotm;
    Vector3d RF_com(0.0162, 0.00008, -0.1209);

    com2cp = rd_.link_[Right_Foot].sensor_point - RF_com;

    adt2.setIdentity();
    adt2.block(3, 0, 3, 3) = DyrosMath::skm(-com2cp) * Matrix3d::Identity();

    Wrench_foot_plate.setZero();
    Wrench_foot_plate(2) = -foot_plate_mass * 9.81;
    Vector6d rfoot_acceleration_temp, rfoot_acceleration;
    rfoot_acceleration_temp = RigidBodyDynamics::CalcPointAcceleration6D(model_global_, q_virtual_Xd_global_, q_dot_virtual_Xd_global_, q_ddot_virtual_Xd_global_, rd_.link_[Right_Foot].id, RF_com, false);
    rfoot_acceleration.segment(0, 3) = rfoot_acceleration_temp.segment(3, 3); // translation
    rfoot_acceleration.segment(3, 3) = rfoot_acceleration_temp.segment(0, 3); // rotation
    // r_ft_wo_fw_ = r_ft_ + adt2 * (rotrf.transpose() * Wrench_foot_plate) - adt2.transpose() * inertia_foot_plate * (rotrf.transpose() * rfoot_acceleration);
    r_ft_wo_fw_ = r_ft_; // tocabi
    r_ft_wo_fw_lpf_ = DyrosMath::lpf<6>(r_ft_wo_fw_, r_ft_wo_fw_lpf_, 2000, 100 / (2 * M_PI));

    if (walking_tick_mj == 0)
    {
        l_ft_LPF = l_ft_;
        r_ft_LPF = r_ft_;
    }

    l_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_;
    r_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_;

    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = -l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = -r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);

    zmp_measured_FT_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP X
    zmp_measured_FT_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y

    // current_time_computeslow_ = rd_.control_time_;

    // if (current_time_computeslow_ != pre_time_computeslow_)
    // {
    //     dt_computeslow_ = current_time_computeslow_ - pre_time_computeslow_;
    // }

    //friction torque calculation
    // frictionTorqueCalculator(rd_.q_dot_, desired_q_dot_fast_, torque_current_elmo_, friction_model_torque_);

    // collectRobotInputData_acc_version();     // 1us
    // calculateLstmInput(left_leg_mob_lstm_);  // 1us
    // calculateLstmInput(right_leg_mob_lstm_); // 1us

    // collectRobotInputData_peter_gru();
    // calculateGruInput(left_leg_peter_gru_);
    // calculateGruInput(right_leg_peter_gru_);

    floatingBaseMOB();                       // created by DG
    // collisionEstimation();
    // l_cf_ft_global_ = rd_.LF_CF_FT;
    // r_cf_ft_global_ = rd_.RF_CF_FT;

    // l_cf_ft_local_.segment(0, 3) = rd_.link_[Pelvis].rotm.transpose() * rd_.LF_CF_FT.segment(0, 3);
    // l_cf_ft_local_.segment(3, 3) = rd_.link_[Pelvis].rotm.transpose() * rd_.LF_CF_FT.segment(3, 3);

    // r_cf_ft_local_.segment(0, 3) = rd_.link_[Pelvis].rotm.transpose() * rd_.RF_CF_FT.segment(0, 3);
    // r_cf_ft_local_.segment(3, 3) = rd_.link_[Pelvis].rotm.transpose() * rd_.RF_CF_FT.segment(3, 3);
    // if( walking_tick_mj % 1000 == 0 )
    // {
    // cout<<"l_ft_: "<<l_ft_.transpose()<<endl;
    // cout<<"r_ft_: "<<r_ft_.transpose()<<endl;
    // cout<<"opto_ft_: "<<opto_ft_.transpose()<<endl;
    // }

    // MJ_graph << l_ft_LPF(2) << "," << l_ft_LPF(3) << "," << l_ft_LPF(4) << "," << r_ft_LPF(2) << "," << r_ft_LPF(3) << "," << r_ft_LPF(4) << endl;
    //  Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = -l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = -r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);
    zmp_measured_mj_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP X
    zmp_measured_mj_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y

    if (walking_tick_mj == 0)
    {
        zmp_measured_LPF_.setZero();
    }
    zmp_measured_LPF_ = (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 8.0 * del_t) * zmp_measured_LPF_;
    // dg test
    zmp_measured_LPF_ = zmp_measured_mj_;
    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // if(walking_tick_mj%2 == 0)
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
        middle_total_step_number = 10; //
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
    // foot_step_support_frame_thread_.setZero(number_of_foot_step, 7);
    // foot_step_support_frame_mpc_.setZero(number_of_foot_step, 7);

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
    unsigned int middle_total_step_number = length_to_target / abs(dlength);
    double middle_residual_length = (length_to_target - middle_total_step_number * abs(dlength)) * DyrosMath::sign(dlength);

    double step_width_init;
    double step_width;

    step_width_init = 0.01;
    step_width = 0.02;

    if (length_to_target == 0.0)
    {
        middle_total_step_number = 30; // total foot step number
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
    // foot_step_support_frame_thread_.setZero(number_of_foot_step, 7);
    // foot_step_support_frame_mpc_.setZero(number_of_foot_step, 7);
    modified_del_zmp_.setZero(number_of_foot_step, 2);
    m_del_zmp_x.setZero(number_of_foot_step, 2);
    m_del_zmp_y.setZero(number_of_foot_step, 2);

    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing_ == true)
    {
        is_right = 1;
        current_support_foot_is_left_ = true;
    }
    else
    {
        is_right = -1;
        current_support_foot_is_left_ = false;
    }

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
void AvatarController::calculateFootStepTotal_reactive(Eigen::Vector3d collision_position, Eigen::Vector3d external_force, bool is_right_foot_swing)
{
    double step_width_init;
    double step_width;

    step_width_init = 0.01;
    step_width = 0.02;

    int number_of_foot_step;

    int del_size;

    del_size = 1;
    number_of_foot_step = 4;

    current_step_num_ = 1;
    total_step_num_ = 4;

    foot_step_.resize(number_of_foot_step, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(number_of_foot_step, 7);
    foot_step_support_frame_.setZero();
    // foot_step_support_frame_thread_.setZero(number_of_foot_step, 7);
    // foot_step_support_frame_mpc_.setZero(number_of_foot_step, 7);
    modified_del_zmp_.setZero(number_of_foot_step, 2);
    m_del_zmp_x.setZero(number_of_foot_step, 2);
    m_del_zmp_y.setZero(number_of_foot_step, 2);

    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing == true)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right;
    temp2 = -is_right;
    temp3 = -is_right;

    int temp0;
    temp0 = -is_right;

    double initial_dir = 0.0;

    // double third_foot_step_x;
    // third_foot_step_x = DyrosMath::minmax_cut(target_x - 0.08, -2 * step_length_x_, 0.0);

    Vector2d first_foot_step = collision_position.segment(0, 2) + collision_position(2) * external_force.segment(0, 2).normalized();
    if (is_right_foot_swing)
    {
        first_foot_step(1) = DyrosMath::minmax_cut(first_foot_step(1), -0.5, -0.20);
    }
    else
    {
        first_foot_step(1) = DyrosMath::minmax_cut(first_foot_step(1), 0.20, 0.5);
    }

    Vector2d second_foot_step = first_foot_step;
    second_foot_step(1) += 2 * is_right * (0.1025 + step_width);
    second_foot_step += 0.1 * external_force.segment(0, 2).normalized();

    if (is_right_foot_swing)
    {
        second_foot_step(1) = DyrosMath::minmax_cut(second_foot_step(1), first_foot_step(1) + 0.20, first_foot_step(1) + 0.5);
    }
    else
    {
        second_foot_step(1) = DyrosMath::minmax_cut(second_foot_step(1), first_foot_step(1) - 0.5, first_foot_step(1) - 0.2);
    }

    // current support foot_step_
    foot_step_(0, 0) = 0.0;
    foot_step_(0, 1) = 0;
    foot_step_(0, 6) = 0.5 + 0.5 * (-is_right); //left foot support = 1, right foot support = 0

    // first landing step
    foot_step_(1, 0) = first_foot_step(0);
    foot_step_(1, 1) = first_foot_step(1);
    foot_step_(1, 6) = 0.5 + 0.5 * (is_right);

    // second step
    foot_step_(2, 0) = second_foot_step(0);
    foot_step_(2, 1) = second_foot_step(1);
    foot_step_(2, 6) = 0.5 + 0.5 * (-is_right);

    // third step
    foot_step_(3, 0) = second_foot_step(0);
    foot_step_(3, 1) = second_foot_step(1) - 2 * is_right * (0.1025 + step_width);
    foot_step_(3, 6) = 0.5 + 0.5 * (is_right);

    // fourth step
    // foot_step_(4, 0) = third_foot_step_x;
    // foot_step_(4, 1) = is_right * (0.1025 + step_width);
    // foot_step_(4, 6) = 0.5 + 0.5 * (-is_right);
    cout << "Reactive Foot Step\n"<< foot_step_ << endl;
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
    //  Kp(0) = 1800.0;
    //  Kd(0) = 70.0; // Left Hip yaw
    //  Kp(1) = 2100.0;
    //  Kd(1) = 90.0; // Left Hip roll
    //  Kp(2) = 2100.0;
    //  Kd(2) = 90.0; // Left Hip pitch
    //  Kp(3) = 2100.0;
    //  Kd(3) = 90.0; // Left Knee pitch
    //  Kp(4) = 2100.0;
    //  Kd(4) = 90.0; // Left Ankle pitch
    //  Kp(5) = 2100.0;
    //  Kd(5) = 90.0; // Left Ankle roll

    // Kp(6) = 1800.0;
    // Kd(6) = 70.0; // Right Hip yaw
    // Kp(7) = 2100.0;
    // Kd(7) = 90.0; // Right Hip roll
    // Kp(8) = 2100.0;
    // Kd(8) = 90.0; // Right Hip pitch
    // Kp(9) = 2100.0;
    // Kd(9) = 90.0; // Right Knee pitch
    // Kp(10) = 2100.0;
    // Kd(10) = 90.0; // Right Ankle pitch
    // Kp(11) = 2100.0;
    // Kd(11) = 90.0; // Right Ankle roll

    // Kp(12) = 2200.0;
    // Kd(12) = 90.0; // Waist yaw
    // Kp(13) = 2200.0;
    // Kd(13) = 90.0; // Waist pitch
    // Kp(14) = 2200.0;
    // Kd(14) = 90.0; // Waist roll

    // Kp(15) = 400.0;
    // Kd(15) = 10.0;
    // Kp(16) = 800.0;
    // Kd(16) = 10.0;
    // Kp(17) = 400.0;
    // Kd(17) = 10.0;
    // Kp(18) = 400.0;
    // Kd(18) = 10.0;
    // Kp(19) = 250.0;
    // Kd(19) = 2.5;
    // Kp(20) = 250.0;
    // Kd(20) = 2.0;
    // Kp(21) = 50.0;
    // Kd(21) = 2.0; // Left Wrist
    // Kp(22) = 50.0;
    // Kd(22) = 2.0; // Left Wrist

    // Kp(23) = 50.0;
    // Kd(23) = 2.0; // Neck
    // Kp(24) = 50.0;
    // Kd(24) = 2.0; // Neck

    // Kp(25) = 400.0;
    // Kd(25) = 10.0;
    // Kp(26) = 800.0;
    // Kd(26) = 10.0;
    // Kp(27) = 400.0;
    // Kd(27) = 10.0;
    // Kp(28) = 400.0;
    // Kd(28) = 10.0;
    // Kp(29) = 250.0;
    // Kd(29) = 2.5;
    // Kp(30) = 250.0;
    // Kd(30) = 2.0;
    // Kp(31) = 50.0;
    // Kd(31) = 2.0; // Right Wrist
    // Kp(32) = 50.0;
    // Kd(32) = 2.0; // Right Wrist

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

    // lfoot_zmp_offset_ = -0.015; // 0.9 초
    // rfoot_zmp_offset_ = 0.015;

    // lfoot_zmp_offset_ = -0.02; // 1.1 초
    // rfoot_zmp_offset_ = 0.02;

    // lfoot_zmp_offset_ = -0.015; // 1.3 초
    // rfoot_zmp_offset_ = 0.015;

    // lfoot_zmp_offset_ = -0.0; // simul 1.1 s
    // rfoot_zmp_offset_ = 0.0;

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
    }

    for (int i = 0; i < total_step_num_; i++)
    {
        if (foot_step_(i, 6) == 0) // left support foot
        {
            foot_step_support_frame_offset_(i, 1) += lfoot_zmp_offset_;
        }
        else // right support foot
        {
            foot_step_support_frame_offset_(i, 1) += rfoot_zmp_offset_;
        }
    }
}

void AvatarController::getZmpTrajectory()
{
    int planning_step_number = 6;
    int norm_size = 0;

    if (current_step_num_ >= total_step_num_ - planning_step_number)
        norm_size = (t_last_ - t_start_ + 1) * (total_step_num_ - current_step_num_) + 5.0 * hz_;
    else
        norm_size = (t_last_ - t_start_ + 1) * (planning_step_number);

    if (current_step_num_ == 0)
        norm_size = norm_size + t_temp_ + 1;
    addZmpOffset();
    zmpGenerator(norm_size, planning_step_number);
    ref_zmp_wo_offset_mpc_.resize(norm_size, 2);
    ref_zmp_mpc_.resize(norm_size, 2);
}

void AvatarController::zmpGenerator(int norm_size, int planning_step_num)
{
    ref_zmp_mj_.resize(norm_size, 2);
    ref_zmp_mj_wo_offset_.resize(norm_size, 2);
    Eigen::VectorXd temp_px;
    Eigen::VectorXd temp_py;
    Eigen::VectorXd temp_px_wo_offset;
    Eigen::VectorXd temp_py_wo_offset;

    unsigned int index = 0;
    

    // 매 tick 마다 zmp가 3발 앞까지 계산 된다.
    if (current_step_num_ == 0) // Walking을 수행 할 때, 정지 상태 일때 3초 동안 Ref X ZMP를 0으로 보냄. Y ZMP는 제자리 유지.
    {
        for (int i = 0; i <= t_temp_; i++) //600 tick
        {
            if (i < t_temp_*0.3)
            {
                ref_zmp_mj_(i, 0) = com_support_init_(0);

                ref_zmp_mj_wo_offset_(i, 0) = com_support_init_(0);
            }
            else if (i < t_temp_*0.7)
            {
                double del_x = i - t_temp_*0.3;
                ref_zmp_mj_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (t_temp_*0.4);

                ref_zmp_mj_wo_offset_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (t_temp_*0.4);
            }
            else
            {
                ref_zmp_mj_(i, 0) = 0.0;
                

                ref_zmp_mj_wo_offset_(i, 0) = 0.0;
                
            }
            // y-axis
            ref_zmp_mj_(i, 1) = com_support_init_(1);
            ref_zmp_mj_wo_offset_(i, 1) = com_support_init_(1);
            index++;
        }
    }
    /////////////////////////////////////////////////////////////////////.

    if (current_step_num_ >= total_step_num_ - planning_step_num)
    {
        for (unsigned int i = current_step_num_; i < total_step_num_; i++)
        {
            // onestepZmp(i, temp_px, temp_py);
            onestepZmp_wo_offset(i, temp_px, temp_py, temp_px_wo_offset, temp_py_wo_offset);
            for (unsigned int j = 0; j < t_total_; j++)
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);

                ref_zmp_mj_wo_offset_(index + j, 0) = temp_px_wo_offset(j);
                ref_zmp_mj_wo_offset_(index + j, 1) = temp_py_wo_offset(j);
            }
            index = index + t_total_;
        }

        for (unsigned int j = 0; j < 5.0 * hz_; j++)
        {
            ref_zmp_mj_(index + j, 0) = ref_zmp_mj_(index - 1, 0);
            ref_zmp_mj_(index + j, 1) = ref_zmp_mj_(index - 1, 1);

            ref_zmp_mj_wo_offset_(index + j, 0) = ref_zmp_mj_wo_offset_(index - 1, 0);
            ref_zmp_mj_wo_offset_(index + j, 1) = ref_zmp_mj_wo_offset_(index - 1, 1);
        }
        index = index + 5.0 * hz_;
    }
    else // 보행 중 사용 하는 Ref ZMP
    {           
        for (unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)
        {            
            // onestepZmp(i, temp_px, temp_py);
            onestepZmp_wo_offset(i, temp_px, temp_py, temp_px_wo_offset, temp_py_wo_offset);
            for (unsigned int j = 0; j < t_total_; j++)  
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);

                ref_zmp_mj_wo_offset_(index + j, 0) = temp_px_wo_offset(j);
                ref_zmp_mj_wo_offset_(index + j, 1) = temp_py_wo_offset(j);
            }
            index = index + t_total_;                                         
        }
    }
}

void AvatarController::onestepZmp(int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py)
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
            if (i >= 0 && i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = Kx / (t_double1_ + t_rest_init_) * (i + 1);
                temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = 0;
                temp_py(i) = supportfoot_support_init_offset_(1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick
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
            if (i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick
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
            int current_stepping_flag_ = 0;
            int prev_stepping_flag_ = 0;
            if(foot_step_(current_step_num_, 6) == 1) // left foot support
            {
                prev_stepping_flag_ = 0; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
                current_stepping_flag_ = 1; // Current first DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
            }
            else // right foot support
            {
                prev_stepping_flag_ = 1; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
                current_stepping_flag_ = 0; // First DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
            }

            if (i < t_rest_init_ + t_double1_)  
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 2, 0) - m_del_zmp_x(current_step_number -1,current_stepping_flag_) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 + (Kx + m_del_zmp_x(current_step_number -1, current_stepping_flag_)/2) / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 2, 1) - m_del_zmp_y(current_step_number -1,current_stepping_flag_) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 + (Ky + m_del_zmp_y(current_step_number -1, current_stepping_flag_)/2) / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_)  
            {   
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_)  
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + (Kx2 + m_del_zmp_x(current_step_number, prev_stepping_flag_)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + (Ky2 + m_del_zmp_y(current_step_number, prev_stepping_flag_)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
}

void AvatarController::onestepZmp_wo_offset(int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py, Eigen::VectorXd &temp_px_wo_offset, Eigen::VectorXd &temp_py_wo_offset)
{
    temp_px.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
    temp_py.resize(t_total_);
    temp_px.setZero();
    temp_py.setZero();
    temp_px_wo_offset.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
    temp_py_wo_offset.resize(t_total_);
    temp_px_wo_offset.setZero();
    temp_py_wo_offset.setZero();
    
    
    double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0, Kx2 = 0, Ky2 = 0;
    double Kx_wo_offset = 0, Ky_wo_offset = 0, Kx2_wo_offset = 0, Ky2_wo_offset = 0;
    if (current_step_number == 0)
    {
        Kx = 0;
        Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) - supportfoot_support_init_offset_(0))/2;
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) - supportfoot_support_init_offset_(1))/2;
        Kx_wo_offset = 0;
        Ky_wo_offset = supportfoot_support_init_(1) - com_support_init_(1);
        Kx2_wo_offset = (foot_step_support_frame_(current_step_number, 0) - supportfoot_support_init_(0))/2;
        Ky2_wo_offset = (foot_step_support_frame_(current_step_number, 1) - supportfoot_support_init_(1))/2;

        for (int i = 0; i < t_total_; i++)
        {
            if (i >= 0 && i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = 0;
                temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i + 1);
                
                temp_px_wo_offset(i) = Kx_wo_offset / (t_double1_ + t_rest_init_) * (i + 1);
                temp_py_wo_offset(i) = com_support_init_(1) + Ky_wo_offset / (t_double1_ + t_rest_init_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = 0;
                temp_py(i) = supportfoot_support_init_offset_(1);
                
                temp_px_wo_offset(i) = 0;
                temp_py_wo_offset(i) = supportfoot_support_init_(1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick
            {
                temp_px(i) = 0 + Kx2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = supportfoot_support_init_offset_(1) + Ky2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                
                temp_px_wo_offset(i) = 0 + Kx2_wo_offset / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py_wo_offset(i) = supportfoot_support_init_(1) + Ky2_wo_offset / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
    else if (current_step_number == 1)
    { 
        Kx = (foot_step_support_frame_offset_(current_step_number - 1, 0) - supportfoot_support_init_(0)) / 2;
        Ky = (foot_step_support_frame_offset_(current_step_number - 1, 1) - supportfoot_support_init_(1)) / 2;
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) - foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2;
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) - foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2;

        Kx_wo_offset = (foot_step_support_frame_(current_step_number - 1, 0) - supportfoot_support_init_(0)) / 2;
        Ky_wo_offset = (foot_step_support_frame_(current_step_number - 1, 1) - supportfoot_support_init_(1)) / 2;
        Kx2_wo_offset = (foot_step_support_frame_(current_step_number, 0) - foot_step_support_frame_(current_step_number - 1, 0)) / 2;
        Ky2_wo_offset = (foot_step_support_frame_(current_step_number, 1) - foot_step_support_frame_(current_step_number - 1, 1)) / 2;

        for (int i = 0; i < t_total_; i++)
        {
            if (i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) -Kx + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) -Ky + Ky / (t_rest_init_ + t_double1_) * (i + 1);
                
                temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0) -Kx_wo_offset + Kx_wo_offset / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1) -Ky_wo_offset + Ky_wo_offset / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);

                temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0);
                temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));

                temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0) + Kx2_wo_offset / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1) + Ky2_wo_offset / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
    else
    { 
        Kx = (foot_step_support_frame_offset_(current_step_number - 1, 0) - foot_step_support_frame_offset_(current_step_number - 2, 0)) / 2;
        Ky = (foot_step_support_frame_offset_(current_step_number - 1, 1) - foot_step_support_frame_offset_(current_step_number - 2, 1)) / 2;
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) - foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2;
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) - foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2;

        Kx_wo_offset = (foot_step_support_frame_(current_step_number - 1, 0) - foot_step_support_frame_(current_step_number - 2, 0))/ 2;
        Ky_wo_offset = (foot_step_support_frame_(current_step_number - 1, 1) - foot_step_support_frame_(current_step_number - 2, 1))/ 2;
        Kx2_wo_offset = (foot_step_support_frame_(current_step_number, 0) - foot_step_support_frame_(current_step_number - 1, 0)) / 2;
        Ky2_wo_offset = (foot_step_support_frame_(current_step_number, 1) - foot_step_support_frame_(current_step_number - 1, 1)) / 2;

        for (int i = 0; i < t_total_; i++)
        {   
            int current_stepping_flag_ = 0;
            int prev_stepping_flag_ = 0;
            if(foot_step_(current_step_num_, 6) == 1) // left foot support
            {
                prev_stepping_flag_ = 0; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
                current_stepping_flag_ = 1; // Current first DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
            }
            else // right foot support
            {
                prev_stepping_flag_ = 1; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
                current_stepping_flag_ = 0; // First DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
            }

            if (i < t_rest_init_ + t_double1_)  
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + (-Kx + m_del_zmp_x(current_step_number -1,current_stepping_flag_)/2) + (Kx - m_del_zmp_x(current_step_number -1, current_stepping_flag_)/2) / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + (-Ky + m_del_zmp_y(current_step_number -1,current_stepping_flag_)/2) + (Ky - m_del_zmp_y(current_step_number -1, current_stepping_flag_)/2) / (t_rest_init_ + t_double1_) * (i + 1);

                temp_px_wo_offset(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + (-Kx_wo_offset + m_del_zmp_x(current_step_number -1,current_stepping_flag_)/2) + (Kx_wo_offset - m_del_zmp_x(current_step_number -1, current_stepping_flag_)/2) / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py_wo_offset(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + (-Ky_wo_offset + m_del_zmp_y(current_step_number -1,current_stepping_flag_)/2) + (Ky_wo_offset - m_del_zmp_y(current_step_number -1, current_stepping_flag_)/2) / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_)  
            {   
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);

                temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0);
                temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_)  
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + (Kx2 + m_del_zmp_x(current_step_number, prev_stepping_flag_)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + (Ky2 + m_del_zmp_y(current_step_number, prev_stepping_flag_)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));

                temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0) + (Kx2_wo_offset + m_del_zmp_x(current_step_number, prev_stepping_flag_)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1) + (Ky2_wo_offset + m_del_zmp_y(current_step_number, prev_stepping_flag_)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
}

void AvatarController::getFootTrajectory()
{
    if (walking_tick_mj == 0)
    {
        target_swing_foot_.setZero();
        target_swing_foot_pre_.setZero();
    }

    target_swing_foot_pre_ = target_swing_foot_;
    if (true)
    {
        for (int i = 0; i < 6; i++)
        {
            target_swing_foot_(i) = DyrosMath::lpf(foot_step_support_frame_(current_step_num_, i), target_swing_foot_pre_(i), 2000, 2);
        }

        // for (int i = 3; i < 6; i++)
        // {
        //     target_swing_foot_(i) = foot_step_support_frame_(current_step_num_, i);
        // }
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            target_swing_foot_(i) = foot_step_support_frame_(current_step_num_, i);
        }
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
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot_(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), target_swing_foot_(i), 0.0, 0.0);
            }

            rfoot_trajectory_euler_support_(0) = 0;
            rfoot_trajectory_euler_support_(1) = 0;
            rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_euler_init_(2), target_swing_foot_(5), 0.0, 0.0);
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
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot_(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), target_swing_foot_(i), 0.0, 0.0);
            }

            lfoot_trajectory_euler_support_(0) = 0;
            lfoot_trajectory_euler_support_(1) = 0;
            lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_euler_init_(2), target_swing_foot_(5), 0.0, 0.0);
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
                rfoot_trajectory_support_.translation()(i) = target_swing_foot_(i);
                rfoot_trajectory_euler_support_(i) = target_swing_foot_(i + 3);
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
                lfoot_trajectory_support_.translation()(i) = target_swing_foot_(i);
                lfoot_trajectory_euler_support_(i) = target_swing_foot_(i + 3);
            }
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            // lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }
}

void AvatarController::getFootTrajectory_stepping()
{   
    // Eigen::Vector6d target_swing_foot;
    // Eigen::Vector6d desired_swing_foot;
    // Eigen::Vector6d fixed_swing_foot;
    double ssp_flag = 0;
    if(walking_tick_mj == 0)
    {
        desired_swing_foot.setZero();
        fixed_swing_foot.setZero();
        fixed_swing_foot_del_F_.setZero();
        target_swing_foot.setZero();    
        opt_F_.setZero();
    }
    
    for (int i = 0; i < 6; i++)
    {
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i);        
    }

    if(walking_tick_mj == t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_) // 조현민 처럼 Step으로 zmp를 변경하는게 아니라 부드럽게 바꿔줘도 좋을듯 / SSP 끝나기 0.1초 전 스윙 발 X,Y 고정
    {
        fixed_swing_foot(0) = desired_swing_foot(0); 
        fixed_swing_foot(1) = desired_swing_foot(1);  
        modified_del_zmp_(current_step_num_,0) = opt_F_(0) - target_swing_foot(0); //del_F_x_;   //opt_F_(0) - target_swing_foot(0);
        modified_del_zmp_(current_step_num_,1) = opt_F_(1) - target_swing_foot(1); //del_F_y_;   //opt_F_(1);        
    }

    if(current_step_num_ > 0)
    {
        if(foot_step_(current_step_num_, 6) == 1) // left support foot
        {    
            m_del_zmp_x(current_step_num_,0) = modified_del_zmp_(current_step_num_,0); // 왼발 지지때 추가로 생성된 오른쪽 스윙 발 변위만큼 오른발의 참조 ZMP를 수정해주기 위해 저장한 val
            m_del_zmp_y(current_step_num_,0) = modified_del_zmp_(current_step_num_,1);      
        }
        else // right support foot
        {              
            m_del_zmp_x(current_step_num_,1) = modified_del_zmp_(current_step_num_,0); // 오른발 지지때 추가로 생성된 왼쪽 스윙 발 변위만큼 왼발의 참조 ZMP를 수정해주기 위해 저장한 val
            m_del_zmp_y(current_step_num_,1) = modified_del_zmp_(current_step_num_,1); 
        }
    } 
    
    Eigen::Vector2d stepping_foot_init_pos;
    stepping_foot_init_pos.setZero();    

    if(foot_step_(current_step_num_,6) == 1) // left foot support
    {
        stepping_foot_init_pos(0) = rfoot_support_init_.translation()(0); //rfoot_trajectory_support_.translation()(0);
        stepping_foot_init_pos(1) = rfoot_support_init_.translation()(1); //rfoot_trajectory_support_.translation()(1);
    }
    else // right foot support
    {
        stepping_foot_init_pos(0) = lfoot_support_init_.translation()(0); //lfoot_trajectory_support_.translation()(0);
        stepping_foot_init_pos(1) = lfoot_support_init_.translation()(1); //lfoot_trajectory_support_.translation()(1);
    }
    
    if (walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_)
    {
        desired_swing_foot(0) = opt_F_(0); //target_swing_foot(0) + del_F_x_; //target_swing_foot(0) + opt_F_(0); // opt_F_ is optimized by target_swing_foot(0) + del_F_x
        desired_swing_foot(1) = opt_F_(1); // target_swing_foot(1) + del_F_y_; //target_swing_foot(1) + opt_F_(1); 
    }
    else
    {
        desired_swing_foot(0) = fixed_swing_foot(0);
        desired_swing_foot(1) = fixed_swing_foot(1);
    }    
     
    // real robot experiment 
    desired_swing_foot_LPF_(0) = 1 / (1 + 2 * M_PI * 2.0 * del_t) * desired_swing_foot_LPF_(0) + (2 * M_PI * 2.0 * del_t) / (1 + 2 * M_PI * 2.0 * del_t) * desired_swing_foot(0);
    // desired_swing_foot_LPF_(1) = 1 / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot_LPF_(1) + (2 * M_PI * 1.0 * del_t) / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot(1);
    // del_F_LPF_(1) = 1 / (1 + 2 * M_PI * 1.0 * del_t) * del_F_LPF_(1) + (2 * M_PI * 1.0 * del_t) / (1 + 2 * M_PI * 1.0 * del_t) * opt_F_(1);
 
    
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
    {   ssp_flag = 0.1;
        double t_rest_temp = 0.00 * hz_;

        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            // for (int i = 0; i < 2; i++)
            // {
            //     rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            // }

            // 220422
            rfoot_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(0), desired_swing_foot_LPF_(0), 0.0, 0.0);
            rfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    
            
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

            // for (int i = 0; i < 2; i++)
            // {
            //     lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            // }
            // 220422
            lfoot_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(0), desired_swing_foot_LPF_(0), 0.0, 0.0);
            lfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    
            
            lfoot_trajectory_euler_support_(0) = 0;
            lfoot_trajectory_euler_support_(1) = 0;
            lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }
    else
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_euler_support_.setZero();
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            for (int i = 0; i < 3; i++)
            {
                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }
            // 220422
            rfoot_trajectory_support_.translation()(0) =  desired_swing_foot_LPF_(0);
            rfoot_trajectory_support_.translation()(1) =  desired_swing_foot(1);

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

            for (int i = 0; i < 3; i++)
            {
                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }
            // 220422
            lfoot_trajectory_support_.translation()(0) =  desired_swing_foot_LPF_(0); 
            lfoot_trajectory_support_.translation()(1) =  desired_swing_foot(1);    
            
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }
    // MJ_graph1 << lfoot_trajectory_support_.translation()(0) << "," << rfoot_trajectory_support_.translation()(0) << "," << opt_F_(0) << "," << desired_swing_foot(0) << "," << target_swing_foot(0) << endl;
    // MJ_graph1 << lfoot_trajectory_support_.translation()(1) << "," << rfoot_trajectory_support_.translation()(1) << "," << target_swing_foot(1) + opt_F_(1) << "," << desired_swing_foot(1) << "," << ssp_flag << endl;
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
    ZMP_X_REF = ref_zmp_mj_(tick, 0);
    ZMP_Y_REF = ref_zmp_mj_(tick, 1);

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
        // cout << "preview X state : " << preview_x_mj(0) << "," << preview_x_mj(1) << "," << preview_x_mj(2) << endl;
        // cout << "preview Y state : " << preview_y_mj(0) << "," << preview_y_mj(1) << "," << preview_y_mj(2) << endl;
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
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (ref_zmp_mj_(tick + 1 + i, 0) - ref_zmp_mj_(tick + i, 0));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (ref_zmp_mj_(tick + 1 + i, 1) - ref_zmp_mj_(tick + i, 1));
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
        // cout << "del_zmp : " << del_zmp(0) << "," << del_zmp(1) << endl;
    }

    del_ux(0, 0) = -(px(0) - ref_zmp_mj_(tick, 0)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - ref_zmp_mj_(tick, 1)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x_mj + B * UX;
    YD = A * preview_y_mj + B * UY;

    cp_desired_(0) = XD(0) + XD(1) / wn;
    cp_desired_(1) = YD(0) + YD(1) / wn;

    // MJ_graph << XD(0) << "," << YD(0) << "," << ZMP_X_REF << "," << ZMP_Y_REF << "," << cp_desired_(0) << "," << cp_desired_(1) << endl;
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
    double pelv_height_offset_ = 0.05;
    if (walking_enable_ == true)
    {
        pelv_height_offset_ = DyrosMath::cubic(walking_tick_mj, 0, pelv_transition_time * hz_, 0.0, 0.05, 0.0, 0.0);
    }

    double z_rot = foot_step_support_frame_(current_step_num_, 5);

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - com_support_current_(0));
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7 * (com_desired_(1) - com_support_current_(1));
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

    P_angle_input_dot = 1.5 * (0.0 - P_angle);
    R_angle_input_dot = 2.0 * (0.0 - R_angle);

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

    if (float_data_collect_mode_ == false)
    {
        Trunk_trajectory_euler(0) = R_angle_input;
        Trunk_trajectory_euler(1) = P_angle_input;
    }

    pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2)) * DyrosMath::rotateWithY(Trunk_trajectory_euler(1)) * DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void AvatarController::supportToFloatPattern()
{
    // lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
    // rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
    //  pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
    pelv_trajectory_float_.setIdentity();
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;

    rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
    lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
}

void AvatarController::getComTrajectory_mpc()
{
    if (walking_tick_mj == 0)
    {       
        U_x_mpc_.setZero(135);
        U_y_mpc_.setZero(135);

        x_hat_.setZero();
        x_hat_(0) = xi_mj_;
        y_hat_.setZero();
        y_hat_(0) = yi_mj_;

        x_hat_thread_.setZero();
        x_hat_thread_(0) = xi_mj_;
        y_hat_thread_.setZero();
        y_hat_thread_(0) = yi_mj_;
        x_hat_p_thread_.setZero();
        x_hat_p_thread_(0) = xi_mj_;
        y_hat_p_thread_.setZero();
        y_hat_p_thread_(0) = yi_mj_;

        x_hat_thread2_.setZero();
        x_hat_thread2_(0) = xi_mj_;
        y_hat_thread2_.setZero();
        y_hat_thread2_(0) = yi_mj_;
        x_hat_p_thread2_.setZero();
        x_hat_p_thread2_(0) = xi_mj_;
        y_hat_p_thread2_.setZero();
        y_hat_p_thread2_(0) = yi_mj_;

        x_hat_r_.setZero();
        x_hat_r_p_.setZero();
        x_hat_r_(0) = xi_mj_;
        x_hat_r_p_(0) = xi_mj_;
        x_mpc_i_(0) = xi_mj_;

        y_hat_r_.setZero();
        y_hat_r_p_.setZero();
        y_hat_r_(0) = yi_mj_;
        y_hat_r_p_(0) = yi_mj_;
        y_mpc_i_(0) = yi_mj_;     

        cp_des_zmp_x_prev_ = xi_mj_;    
        cp_des_zmp_x_ = xi_mj_;
        
        cpmpc_des_zmp_x_thread_ = xi_mj_;
        cpmpc_des_zmp_x_thread2_ = xi_mj_;

        cp_des_zmp_y_prev_ = yi_mj_;    
        cp_des_zmp_y_ = yi_mj_;
        
        cpmpc_des_zmp_y_thread_ = yi_mj_;
        cpmpc_des_zmp_y_thread2_ = yi_mj_;

        cpmpc_des_zmp_x_p_thread_ = xi_mj_;
        cpmpc_des_zmp_y_p_thread_ = yi_mj_;

        cp_measured_thread_(0) = xi_mj_;
        cp_measured_thread_(1) = yi_mj_;

        des_zmp_interpol_.setZero();
        cpmpc_diff_.setZero();
        cpStepping_diff_.setZero();         
    }

    if (current_step_num_ == 0)
    {
        zmp_start_time_mj_ = 0.0;
    }
    else
    {
        zmp_start_time_mj_ = t_start_;
    }    

    ZMP_X_REF = ref_zmp_mj_(walking_tick_mj - zmp_start_time_mj_,0);
    ZMP_Y_REF = ref_zmp_mj_(walking_tick_mj - zmp_start_time_mj_,1); 

    // State variables x_hat_ and Control input U_mpc are updated with every MPC frequency.
    

    int alpha_step = 0;

    if (foot_step_(current_step_num_, 6) == 1)  // left foot support
    {
        alpha_step = 1;
    }
    else
    {
        alpha_step = -1;
    }        

    // support foot change           
    if(walking_tick_mj == t_start_ && current_step_num_ > 0)
    {
        x_hat_r_ = x_hat_r_sc_;
        x_hat_r_p_ = x_hat_r_p_sc_;

        y_hat_r_ = y_hat_r_sc_;
        y_hat_r_p_ = y_hat_r_p_sc_; 
        
        cp_des_zmp_x_ = des_zmp_x_stepchange_;
        cp_des_zmp_x_prev_ = des_zmp_x_prev_stepchange_;

        cp_des_zmp_y_ = des_zmp_y_stepchange_;
        cp_des_zmp_y_prev_ = des_zmp_y_prev_stepchange_;
    }

    // send vaiables to the mpc in the thread3
    if(atb_thread3_input_data_update_ == false)  
    {
        atb_thread3_input_data_update_ = true;
        walking_tick_mj_thread_ = walking_tick_mj;
        t_total_thread_ = t_total_;
        t_rest_init_thread_ = t_rest_init_;
        t_rest_last_thread_ = t_rest_last_;
        current_step_num_thread_ = current_step_num_;
        total_step_num_thread_ = total_step_num_;
        zmp_start_time_mj_thread_ = zmp_start_time_mj_;
        ref_zmp_thread_ = ref_zmp_mj_; 
        ref_zmp_wo_offset_thread_ = ref_zmp_mj_wo_offset_;     

        cp_measured_thread_ = cp_measured_;
        current_support_foot_is_left_thread_ = current_support_foot_is_left_;
        foot_step_support_frame_thread_ = foot_step_support_frame_;

        x_hat_p_thread2_ = x_hat_r_p_;
        y_hat_p_thread2_ = y_hat_r_p_; 
        x_hat_thread2_ = x_hat_r_;
        y_hat_thread2_ = y_hat_r_; // 이것만 쓰면 mpc안에서 덮어 씌워져버려서 따로 Step change 변수 만들어야함.

        if(walking_tick_mj == t_start_ && current_step_num_ > 0)
        {   
            cpmpc_des_zmp_x_thread2_ = des_zmp_x_stepchange_;
            cpmpc_des_zmp_y_thread2_ = des_zmp_y_stepchange_;
            cpmpc_interpol_cnt_x_ = 1;
            cpmpc_interpol_cnt_y_ = 1;
        }
        
        atb_thread3_input_data_update_ = false;
    }  

    // get the mpc result from thread3
    if(atb_thread3_output_data_update_ == false)
    {
        atb_thread3_output_data_update_ = true;
        if(current_support_foot_is_left_thread2_ == current_support_foot_is_left_ )
        {
            x_hat_r_ = x_hat_thread_;
            x_hat_r_p_ = x_hat_p_thread_;                
            wieber_interpol_cnt_x_ = 1;

            y_hat_r_ = y_hat_thread_;
            y_hat_r_p_ = y_hat_p_thread_;
            wieber_interpol_cnt_y_ = 1;

            // cp_des_zmp_x_prev_ = cp_des_zmp_x_;
            cp_des_zmp_x_prev_ =  cpmpc_des_zmp_x_p_thread_;
            cp_des_zmp_x_ = cpmpc_des_zmp_x_thread_;
            del_F_x_ = del_F_x_thread_;
            // del_F_x_next_ = del_F_x_thread_;
            cpmpc_interpol_cnt_x_ = 1;
            

            // cp_des_zmp_y_prev_ = cp_des_zmp_y_; //cpmpc_des_zmp_y_prev_thread_; 
            cp_des_zmp_y_prev_ =  cpmpc_des_zmp_y_p_thread_;
            cp_des_zmp_y_ = cpmpc_des_zmp_y_thread_;
            del_F_y_ = del_F_y_thread_;
            // del_F_y_next_ = del_F_y_thread_;
            cpmpc_interpol_cnt_y_ = 1;
            
        }
        else
        {
            del_F_x_ = 0;
            del_F_y_ = 0;
            // cout<<"Thread 3 output is ignored"<<endl;
        }
        atb_thread3_output_data_update_ = false;
    }

    x_diff_ = x_hat_r_ - x_hat_r_p_; 
    y_diff_ = y_hat_r_ - y_hat_r_p_;
    cpmpc_diff_(0) = cp_des_zmp_x_ - cp_des_zmp_x_prev_;
    cpmpc_diff_(1) = cp_des_zmp_y_ - cp_des_zmp_y_prev_;

    double thread_freq = 50.0;

    double x_com_lin_spline = (thread_freq/hz_)*wieber_interpol_cnt_x_;
    double y_com_lin_spline = (thread_freq/hz_)*wieber_interpol_cnt_y_;

    x_com_lin_spline = DyrosMath::minmax_cut(x_com_lin_spline, 0.0, 1.0);
    y_com_lin_spline = DyrosMath::minmax_cut(y_com_lin_spline, 0.0, 1.0);

    // x_mpc_i_ = x_com_lin_spline*x_diff_ + x_hat_r_p_; // 50.0 = MPC freq.
    // y_mpc_i_ = y_com_lin_spline*y_diff_ + y_hat_r_p_;

    x_mpc_i_ = x_hat_r_;
    y_mpc_i_ = y_hat_r_;

    wieber_interpol_cnt_x_ ++;
    wieber_interpol_cnt_y_ ++;

    wieber_interpol_cnt_x_ = DyrosMath::minmax_cut(wieber_interpol_cnt_x_, 1, int(hz_/thread_freq*1.5));
    wieber_interpol_cnt_y_ = DyrosMath::minmax_cut(wieber_interpol_cnt_y_, 1, int(hz_/thread_freq*1.5));

    double x_cpmpc_lin_spline = (thread_freq/hz_)*cpmpc_interpol_cnt_x_;
    double y_cpmpc_lin_spline = (thread_freq/hz_)*cpmpc_interpol_cnt_y_;

    x_cpmpc_lin_spline = DyrosMath::minmax_cut(x_cpmpc_lin_spline, 0.0, 1.0);
    y_cpmpc_lin_spline = DyrosMath::minmax_cut(y_cpmpc_lin_spline, 0.0, 1.0);
    
    // des_zmp_interpol_(0) = x_cpmpc_lin_spline*cpmpc_diff_(0) + cp_des_zmp_x_prev_;
    // des_zmp_interpol_(1) = y_cpmpc_lin_spline*cpmpc_diff_(1) + cp_des_zmp_y_prev_;

    des_zmp_interpol_(0) = cp_des_zmp_x_;
    des_zmp_interpol_(1) = cp_des_zmp_y_;

    // opt_F_(0) = del_F_x_;
    // opt_F_(1) = del_F_y_;
    // cout << del_F_x_ << endl;
    // MJ_graph << cp_des_zmp_y_prev_ << "," << cp_des_zmp_y_ << "," << del_F_x_ << "," << opt_F_(0) << "," << des_zmp_interpol_(1) << endl;
    
    cpmpc_interpol_cnt_x_ ++;
    cpmpc_interpol_cnt_y_ ++;
    
    // Reference COM, CP position // CPMPC로 대체하면 필요 X
    cp_desired_(0) = x_mpc_i_(0) + x_mpc_i_(1) / wn;
    cp_desired_(1) = y_mpc_i_(0) + y_mpc_i_(1) / wn;

    com_desired_(0) = x_mpc_i_(0);
    com_desired_(1) = y_mpc_i_(0);
    com_desired_(2) = 0.77172;     
              
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

        x_hat_r_p_sc_ = x_hat_r_p_;
        x_hat_r_sc_ = x_hat_r_;
        y_hat_r_p_sc_ = y_hat_r_p_;
        y_hat_r_sc_ = y_hat_r_; 

        des_zmp_x_prev_stepchange_ = cp_des_zmp_x_prev_;
        des_zmp_x_stepchange_ = cp_des_zmp_x_;
        des_zmp_y_prev_stepchange_ = cp_des_zmp_y_prev_;
        des_zmp_y_stepchange_ = cp_des_zmp_y_;

        temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        for (int i = 0; i < 3; i++)
            temp_pos(i) = foot_step_support_frame_(current_step_num_, i);        
       
        temp_pos(0) = temp_pos(0) + modified_del_zmp_(current_step_num_,0);
        temp_pos(1) = temp_pos(1) + modified_del_zmp_(current_step_num_,1);
         
        com_pos_prev(0) = x_hat_r_(0);
        com_pos_prev(1) = y_hat_r_(0);

        com_pos = temp_rot * (com_pos_prev - temp_pos); 

        com_vel_prev(0) = x_hat_r_(1);
        com_vel_prev(1) = y_hat_r_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_hat_r_(2);
        com_acc_prev(1) = y_hat_r_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_hat_r_sc_(0) = com_pos(0);
        y_hat_r_sc_(0) = com_pos(1);
        x_hat_r_sc_(1) = com_vel(0);
        y_hat_r_sc_(1) = com_vel(1);
        x_hat_r_sc_(2) = com_acc(0);
        y_hat_r_sc_(2) = com_acc(1);        

        com_pos_prev(0) = x_hat_r_p_(0);
        com_pos_prev(1) = y_hat_r_p_(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = x_hat_r_p_(1);
        com_vel_prev(1) = y_hat_r_p_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_hat_r_p_(2);
        com_acc_prev(1) = y_hat_r_p_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_hat_r_p_sc_(0) = com_pos(0);
        y_hat_r_p_sc_(0) = com_pos(1);
        x_hat_r_p_sc_(1) = com_vel(0);
        y_hat_r_p_sc_(1) = com_vel(1); 
        x_hat_r_p_sc_(2) = com_acc(0);
        y_hat_r_p_sc_(2) = com_acc(1);

        Eigen::Vector3d des_zmp_prev;
        Eigen::Vector3d des_zmp;
        des_zmp_prev.setZero();
        des_zmp.setZero();
        //com_pos_prev(0) = x_hat_r_p_sc_(0);
        des_zmp_prev(0) = cp_des_zmp_x_;
        des_zmp_prev(1) = cp_des_zmp_y_;
        des_zmp = temp_rot * (des_zmp_prev - temp_pos);        

        des_zmp_x_stepchange_ = des_zmp(0);
        des_zmp_y_stepchange_ = des_zmp(1); // step change 1 tick 전 desired ZMP (MPC output) step change    

        des_zmp_prev(0) = cp_des_zmp_x_prev_;
        des_zmp_prev(1) = cp_des_zmp_y_prev_;
        des_zmp = temp_rot * (des_zmp_prev - temp_pos);        

        des_zmp_x_prev_stepchange_ = des_zmp(0);
        des_zmp_y_prev_stepchange_ = des_zmp(1); // step change 1 tick 전 desired ZMP (MPC output) step change          
    }   

    // MJ_graph1 << ZMP_X_REF << "," << ZMP_Y_REF << "," << com_desired_(0) << "," << com_desired_(1) << "," << cp_desired_(0) << ","  << cp_desired_(1) << "," << com_support_current_(0) << "," << com_support_current_(1) << "," << des_zmp_interpol_(0) << "," << des_zmp_interpol_(1) << "," << cp_measured_(0) << "," << cp_measured_(1) << endl;  
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
    // cout << pelv_support_start_.translation()(2) << endl;

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

        temp_pos(0) = temp_pos(0) + modified_del_zmp_(current_step_num_, 0); // 왼발 오른발 나눌까?
        temp_pos(1) = temp_pos(1) + modified_del_zmp_(current_step_num_, 1);

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
     
    double knee_acos_var_L = 0;
    double knee_acos_var_R = 0;

    knee_acos_var_L = (pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2))/ (2 * L_upper * L_lower);
    knee_acos_var_R = (pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2))/ (2 * L_upper * L_lower);

    knee_acos_var_L = DyrosMath::minmax_cut(knee_acos_var_L, -0.99, + 0.99);
    knee_acos_var_R = DyrosMath::minmax_cut(knee_acos_var_R, -0.99, + 0.99);

    q_des(3) = (-acos(knee_acos_var_L) + M_PI);  
    q_des(9) = (-acos(knee_acos_var_R) + M_PI);
    
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
    q_des(2) = DyrosMath::minmax_cut(q_des(2), -90*DEG2RAD, - 5*DEG2RAD);
    q_des(3) = q_des(3);                                                                                               // Knee pitch
    q_des(4) = q_des(4);                                                                                               // Ankle pitch
    q_des(5) = atan2(L_r(1), L_r(2));                                                                                  // Ankle roll

    q_des(6) = atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    q_des(7) = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_des(6)) + R_Hip_rot_mat(1, 1) * cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(8) = DyrosMath::minmax_cut(q_des(8), -90*DEG2RAD, - 5*DEG2RAD);
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = atan2(R_r(1), R_r(2));

    if (walking_tick_mj == 0)
    {
        sc_joint_err.setZero();
    }

    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {                                                                                           //5.3, 0
        sc_joint_before.setZero();
        sc_joint_before = q_des;
    }
    if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {                                                          //5.3005, 1
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

void AvatarController::CPMPC_bolt_Controller_MJ()
{   
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //Setting up nominal values w.r.t current footstep
 
    // W : 현재 지지발 기준에서 plan된 y방향 발 위치에서 추가로 얼마나 더 step했는지 / 여기서 step width라고 부름/ 로봇이 얼마나 옆으로 이동했는지를 보여주는 value

    // L : del_F_x , W : del_F_y 
    
    double L_nom = 0;
    double L_min = 0; // min value of del_F_x
    double L_max = 0; // max value of del_F_x

    double W_nom = 0;
    double W_min = -0.1;
    double W_max = +0.1;

    double T_nom = 0;
    double T_min = 0; 
    double T_max = 0;
    double tau_nom = 0;
    double T_gap = 0;
        
    double w1_step = 500.0, w2_step = 1.0, w3_step = 1000.0; // simulation 
    // double w1_step = 1.0, w2_step = 0.02, w3_step = 3.0; // real robot experiment
    
    double u0_x = 0; 
    double u0_y = 0;    
    double b_nom_x = 0; 
    double b_nom_y = 0; 
    double l_p = 0;

    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        del_F_x_ = 0;
        del_F_y_ = 0;
    }
    else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
    {
        del_F_x_ = 0;
        del_F_y_ = 0;
    }

    L_nom = foot_step_support_frame_(current_step_num_, 0) + del_F_x_; // foot_step_support_frame_(current_step_num_, 0); 
    W_nom = foot_step_support_frame_(current_step_num_, 1) + del_F_y_; // 0;
    L_min = L_nom - 0.05;
    L_max = L_nom + 0.05;
    W_min = W_nom - 0.03;
    W_max = W_nom + 0.03;
    l_p = foot_step_support_frame_(current_step_num_, 1);    

    u0_x = 0.0;
    u0_y = 0.0; 
    // if(current_step_num_ != 0)
    // {
    //     u0_x = foot_step_support_frame_(current_step_num_-1, 0); 
    //     u0_y = foot_step_support_frame_(current_step_num_-1, 1); 
    // }
    // else
    // {
    //     u0_x = 0.0;
    //     u0_y = 0.0; 
    // }    

    T_gap = 0.05*hz_;
    T_nom = 0.6; // 0.6하면 370 못버팀.
    T_min = T_nom - 0.15; //(t_rest_last_ + t_double2_ + 0.1)/hz_ + 0.01; // DSP가 고정이라는 가정하에 써야됨.
    T_max = T_nom + 0.15;
    tau_nom = exp(wn*T_nom); 

    b_nom_x = L_nom/(exp(wn*T_nom)-1);
    b_nom_y = l_p/(1 + exp(wn*T_nom)) - W_nom/(1 - exp(wn*T_nom));
    
    Eigen::MatrixXd H_step;
    Eigen::VectorXd g_step; 
    
    H_step.setZero(5,5);
    H_step(0,0) = w1_step; // U_T,x (step position in x-direction)
    H_step(1,1) = w1_step; // w1_step; // U_T,y (step position in y-direction)
    H_step(2,2) = w2_step; // tau (step timing)
    H_step(3,3) = w3_step; // DCM offset in x
    H_step(4,4) = w3_step; // w3_step; // DCM offset in y
    
    g_step.setZero(5);
    g_step(0) = -w1_step * (u0_x + L_nom);
    g_step(1) = -w1_step * (u0_y + W_nom); // -w1_step * (u0_y + W_nom); // -200
    g_step(2) = -w2_step * tau_nom;
    g_step(3) = -w3_step * b_nom_x;  
    g_step(4) = -w3_step * b_nom_y;  // -w3_step * b_nom_y; // 0.01

    Eigen::VectorXd lb_step;
    Eigen::VectorXd ub_step;
    Eigen::MatrixXd A_step;         

    double stepping_start_time = 0;
    
    if (current_step_num_ == 0)
    {
        stepping_start_time = 0.0;
    }
    else
    {
        stepping_start_time = t_start_;
    }    
    
    A_step.setZero(7,5);

    A_step(0,0) = 1; // U_T,x
    A_step(0,1) = 0; // U_T,y
    A_step(0,2) = -(cp_measured_(0)-u0_x)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
    A_step(0,3) = 1; // b_x
    A_step(0,4) = 0; // b_y

    A_step(1,0) = 0; // U_T,x
    A_step(1,1) = 1; // U_T,y 
    A_step(1,2) = -(cp_measured_(1)-u0_y)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
    A_step(1,3) = 0; // b_x
    A_step(1,4) = 1; // b_y

    A_step(2,0) = 1; // U_T,x

    A_step(3,1) = 1; // U_T,y

    A_step(4,2) = 1; // tau

    A_step(5,3) = 1; // b_x
    A_step(6,4) = 1; // b_x

    lb_step.setZero(7);
    ub_step.setZero(7);

    lb_step(0) = u0_x;
    lb_step(1) = u0_y;
    lb_step(2) = u0_x + L_min;
    lb_step(3) = u0_y + W_min;
    lb_step(4) = exp(wn*T_min);
    lb_step(5) = b_nom_x - 0.1; 
    lb_step(6) = b_nom_y - 0.1;
    
    ub_step(0) = u0_x;
    ub_step(1) = u0_y;
    ub_step(2) = u0_x + L_max;
    ub_step(3) = u0_y + W_max;
    ub_step(4) = exp(wn*T_max);
    ub_step(5) = b_nom_x + 0.1;
    ub_step(6) = b_nom_y + 0.1;    
    
    if(walking_tick_mj == 0)
    {
        stepping_input_.setZero(5);
    }    

    
    if(current_step_num_ > 0 && (current_step_num_ != total_step_num_-1))
    {   // Solving the QP during only SSP
        if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            QP_stepping_.InitializeProblemSize(5, 7);
            QP_stepping_.EnableEqualityCondition(equality_condition_eps_);
            QP_stepping_.UpdateMinProblem(H_step, g_step);
            QP_stepping_.DeleteSubjectToAx();      
            QP_stepping_.UpdateSubjectToAx(A_step, lb_step, ub_step);
        
            if(QP_stepping_.SolveQPoases(200, stepping_input))
            {   
                stepping_input_ = stepping_input.segment(0, 5);
            }
        }

        if(stepping_input_(2) != 0)
        {
            if(walking_tick_mj - stepping_start_time < t_rest_init_ + t_double1_ + round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ - zmp_modif_time_margin_ - 1 )
            {           
                // t_total_ = round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ + t_rest_init_ + t_double1_ + t_rest_last_ + t_double2_;
                // t_total_ = DyrosMath::minmax_cut(t_total_, 0.75*hz_, 1.05*hz_);
                // t_last_ = t_start_ + t_total_ - 1;
            }            
        }
    }  
        
    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        stepping_input_(0) = foot_step_support_frame_(current_step_num_, 0) + del_F_x_;
        stepping_input_(1) = foot_step_support_frame_(current_step_num_, 1) + del_F_y_;
    }
    else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
    {
        // stepping_input_(0) = opt_F_(0);
        // stepping_input_(1) = opt_F_(1);
    }
    
    opt_F_(0) = L_nom;
    opt_F_(1) = W_nom;
    // opt_F_(0) = stepping_input_(0);
    // opt_F_(1) = stepping_input_(1);
    // cout << opt_F_(0) << "," << t_total_*0.0005 << endl;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // Log 함수 쓸때 주의 -> log(0) -> inf  
}


void AvatarController::BoltController_MJ()
{   
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //Setting up nominal values w.r.t current footstep
 
    // W : 현재 지지발 기준에서 plan된 y방향 발 위치에서 추가로 얼마나 더 step했는지 / 여기서 step width라고 부름/ 로봇이 얼마나 옆으로 이동했는지를 보여주는 value

    // L : del_F_x , W : del_F_y 

    double L_nom = 0;
    double L_min = -0.2; // min value of del_F_x
    double L_max = +0.2; // max value of del_F_x

    if(foot_step_(current_step_num_, 6) == 1) // left foot support
    { 
        L_max = 0.2 - rfoot_support_current_.translation()(0);
        L_min = -0.2 - rfoot_support_current_.translation()(0);
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right foot support
    {
        L_max = 0.2 - lfoot_support_current_.translation()(0);
        L_min = -0.2 - lfoot_support_current_.translation()(0);
    } 

    double W_nom = 0;
    double W_min = -0.08;
    double W_max = +0.08;

    double T_nom = 0;
    double T_min = 0;
    double T_max = 0;
    double tau_nom = 0;
    double T_gap = 0;
    
    double w1_step = 1.0, w2_step = 5.0, w3_step = 1000.0;
    // double w1_step = 1.0, w2_step = 0.0050, w3_step = 10.0;
    
    double u0_x = 0; 
    double u0_y = 0;    
    double b_nom_x = 0; 
    double b_nom_y = 0; 
    double l_p = 0;

    L_nom = foot_step_support_frame_(current_step_num_, 0); 
    W_nom = 0;
    l_p = foot_step_support_frame_(current_step_num_, 1);    

    if(current_step_num_ != 0)
    {
        u0_x = foot_step_support_frame_(current_step_num_-1, 0); 
        u0_y = foot_step_support_frame_(current_step_num_-1, 1); 
    }
    else
    {
        u0_x = 0.0;
        u0_y = 0.0; 
    }    

    T_gap = 0.05*hz_;
    T_nom = 0.6; // 0.6하면 370 못버팀.
    T_min = T_nom - 0.15; //(t_rest_last_ + t_double2_ + 0.1)/hz_ + 0.01; 
    T_max = T_nom + 0.15;
    tau_nom = exp(wn*T_nom); 

    b_nom_x = L_nom/(exp(wn*T_nom)-1);
    b_nom_y = l_p/(1 + exp(wn*T_nom)) - W_nom/(1 - exp(wn*T_nom));

    
    Eigen::MatrixXd H_step;
    Eigen::VectorXd g_step; 
    
    H_step.setZero(5,5);
    H_step(0,0) = w1_step; // U_T,x (step position in x-direction)
    H_step(1,1) = 200.0*w1_step; // U_T,y (step position in y-direction)
    H_step(2,2) = w2_step; // tau (step timing)
    H_step(3,3) = w3_step; // DCM offset in x
    H_step(4,4) = 0.01*w3_step; // DCM offset in y
    
    g_step.setZero(5);
    g_step(0) = -w1_step * (u0_x + L_nom);
    g_step(1) = -200.0*w1_step * (u0_y + W_nom);
    g_step(2) = -w2_step * tau_nom;
    g_step(3) = -w3_step * b_nom_x;  
    g_step(4) = -0.01*w3_step * b_nom_y;  

    Eigen::VectorXd lb_step;
    Eigen::VectorXd ub_step;
    Eigen::MatrixXd A_step;         

    double stepping_start_time = 0;
    
    if (current_step_num_ == 0)
    {
        stepping_start_time = 0.0;
    }
    else
    {
        stepping_start_time = t_start_;
    }    

    A_step.setZero(7,5);

    A_step(0,0) = 1; // U_T,x
    A_step(0,1) = 0; // U_T,y
    A_step(0,2) = -(cp_measured_(0)-u0_x)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
    A_step(0,3) = 1; // b_x
    A_step(0,4) = 0; // b_y

    A_step(1,0) = 0; // U_T,x
    A_step(1,1) = 1; // U_T,y 
    A_step(1,2) = -(cp_measured_(1)-u0_y)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
    A_step(1,3) = 0; // b_x
    A_step(1,4) = 1; // b_y

    A_step(2,0) = 1; // U_T,x

    A_step(3,1) = 1; // U_T,y

    A_step(4,2) = 1; // tau

    A_step(5,3) = 1; // b_x
    A_step(6,4) = 1; // b_x

    lb_step.setZero(7);
    ub_step.setZero(7);

    lb_step(0) = u0_x;
    lb_step(1) = u0_y;
    lb_step(2) = u0_x + L_min;
    lb_step(3) = u0_y + W_min;
    lb_step(4) = exp(wn*T_min);
    lb_step(5) = b_nom_x - 0.02;
    lb_step(6) = b_nom_y - 0.1;
    
    ub_step(0) = u0_x;
    ub_step(1) = u0_y;
    ub_step(2) = u0_x + L_max;
    ub_step(3) = u0_y + W_max;
    ub_step(4) = exp(wn*T_max);
    ub_step(5) = b_nom_x + 0.02;
    ub_step(6) = b_nom_y + 0.1;    

    if(walking_tick_mj == 0)
    {
        stepping_input_.setZero(5);
    }    

    if(current_step_num_ > 0 && (current_step_num_ != total_step_num_-1))
    {   // Solving the QP during only SSP
        if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            QP_stepping_.InitializeProblemSize(5, 7);
            QP_stepping_.EnableEqualityCondition(equality_condition_eps_);
            QP_stepping_.UpdateMinProblem(H_step, g_step);
            QP_stepping_.DeleteSubjectToAx();      
            QP_stepping_.UpdateSubjectToAx(A_step, lb_step, ub_step);
        
            if(QP_stepping_.SolveQPoases(200, stepping_input))
            {   
                stepping_input_ = stepping_input.segment(0, 5);
            }
        }

        if(stepping_input_(2) != 0)
        {
            if(walking_tick_mj - stepping_start_time < t_rest_init_ + t_double1_ + round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ - zmp_modif_time_margin_ - 1 )
            {           
                t_total_ = round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ + t_rest_init_ + t_double1_ + t_rest_last_ + t_double2_;
                t_total_ = DyrosMath::minmax_cut(t_total_, 0.75*hz_, 1.05*hz_);
                t_last_ = t_start_ + t_total_ - 1;
            }            
        }
    }     
 
    opt_F_(0) = stepping_input_(0);
    opt_F_(1) = stepping_input_(1);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // Log 함수 쓸때 주의 -> log(0) -> inf  
    // MJ_graph << stepping_input_(0) << "," << stepping_input_(1) << "," << t_total_ / hz_ << del_zmp(1) << "," << ZMP_Y_REF_alpha__ << endl;
     
}
 

void AvatarController::GravityCalculate_MJ()
{
    double contact_gain = 0.0;
    double eta = 0.9;
    VectorQd grav_;

    if (float_data_collect_mode_ == false)
    {
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
                // Gravity_SSP_(1) = 1.0 * Gravity_SSP_(1);
                // Gravity_SSP_(5) = 1.0 * Gravity_SSP_(5);
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 0, 1);
                Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
                // Gravity_SSP_(7) = 1.0 * Gravity_SSP_(7);
                // Gravity_SSP_(11) = 1.0 * Gravity_SSP_(11);
            }
            Gravity_DSP_.setZero();
            contact_torque_MJ.setZero();
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_)
        {
            contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_, t_start_ + t_total_ - t_rest_last_, 0.0, 1.0, 0.0, 0.0);
            WBC::SetContact(rd_, 1, 1);
            Gravity_SSP_.setZero();
            
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
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
    }
    else
    {
        // Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
        Gravity_DSP_ = floatGravityTorque(q_virtual_Xd_global_);

        Gravity_SSP_.setZero();
    }

    if (atb_grav_update_ == false)
    {
        atb_grav_update_ = true;
        Gravity_MJ_ = Gravity_DSP_ + Gravity_SSP_; // + contact_torque_MJ;
        atb_grav_update_ = false;
    }
}
Eigen::VectorQd AvatarController::floatGravityTorque(Eigen::VectorQVQd q)
{
    // Eigen::VectorXd gravity_torque_temp, q_virtual, q_dot_virtual, qdot_zero;
    // gravity_torque_temp.setZero(MODEL_DOF_VIRTUAL, 1);
    // qdot_zero.setZero(MODEL_DOF_VIRTUAL, 1);
    // q_virtual.setZero(MODEL_DOF_QVIRTUAL, 1);
    // q_virtual = q;
    // q_dot_virtual = rd_.q_dot_virtual_;
    // RigidBodyDynamics::UpdateKinematicsCustom(model_global_, &q_virtual, &qdot_zero, &qdot_zero);
    // RigidBodyDynamics::NonlinearEffects(model_global_, q, qdot_zero, gravity_torque_temp);
    // RigidBodyDynamics::UpdateKinematicsCustom(model_global_, &q_virtual, &q_dot_virtual, &qdot_zero);

    // Eigen::VectorQd gravity_torque = gravity_torque_temp.segment(6, MODEL_DOF);

    // old version test
    Eigen::VectorQd gravity_torque;
    WBC::SetContact(rd_, 1, 1);
    gravity_torque = rd_.G.segment(6, MODEL_DOF);
    return gravity_torque;
}

void AvatarController::parameterSetting()
{
    std::srand(std::time(nullptr)); // use current time as seed for random generator

    ////// random walking setting///////
    // target_z_ = 0.0;
    // com_height_ = 0.71;
    // target_theta_ = (float(std::rand()) / float(RAND_MAX) * 90.0 - 45.0) * DEG2RAD;
    // step_length_x_ = (std::rand() % 301) * 0.001 - 0.15;
    // step_length_y_ = 0.0;
    // is_right_foot_swing_ = 1;
    // target_x_ = step_length_x_ * 10 * sin(target_theta_);
    // target_y_ = step_length_x_ * 10 * cos(target_theta_);

    // t_rest_init_ = 0.02 * hz_; // Slack, 0.9 step time
    // t_rest_last_ = 0.02 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 0.6 * hz_ + (std::rand() % 61) * 0.01 * hz_;

    // // t_rest_init_ = 0.27*hz_;
    // // t_rest_last_ = 0.27*hz_;
    // // t_double1_ = 0.03*hz_;
    // // t_double2_ = 0.03*hz_;
    // // t_total_= 1.3*hz_;
    // foot_height_ = float(std::rand()) / float(RAND_MAX) * 0.05 + 0.03; // 0.9 sec 0.05
    ///////////////////////////////////////////////

    //// Normal walking setting ////
    //// 1.1s walking
    // target_x_ = 0.0;
    // target_y_ = 0;
    // target_z_ = 0.0;
    // com_height_ = 0.71;
    // target_theta_ = 0*DEG2RAD;
    // step_length_x_ = 0.10;
    // step_length_y_ = 0.0;
    // is_right_foot_swing_ = 1;

    // t_rest_init_ = 0.2*hz_;
    // t_rest_last_ = 0.2*hz_;
    // t_double1_ = 0.03*hz_;
    // t_double2_ = 0.03*hz_;
    // t_total_= 1.1*hz_;
    // foot_height_ = 0.070;      // 0.9 sec 0.05

    //// 0.9s walking
    // target_x_ = 1.0;
    // target_y_ = 0;
    // target_z_ = 0.0;
    // com_height_ = 0.71;
    // target_theta_ = 0 * DEG2RAD;
    // step_length_x_ = 0.10;
    // step_length_y_ = 0.0;
    // is_right_foot_swing_ = true;

    // t_rest_init_ = 0.12 * hz_; // Slack, 0.9 step time
    // t_rest_last_ = 0.12 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 0.9 * hz_;
    // foot_height_ = 0.055;      // 0.9 sec 0.05

    //// 0.7s walking
    // target_x_ = 1.0;
    // target_y_ = 0;
    // target_z_ = 0.0;
    // com_height_ = 0.71;
    // target_theta_ = 0*DEG2RAD;
    // step_length_x_ = 0.10;
    // step_length_y_ = 0.0;
    // is_right_foot_swing_ = 1;

    // t_rest_init_ = 0.06*hz_;
    // t_rest_last_ = 0.06*hz_;
    // t_double1_ = 0.03*hz_;
    // t_double2_ = 0.03*hz_;
    // t_total_= 0.7*hz_;
    // foot_height_ = 0.055;      // 0.9 sec 0.05

    //// 0.6s walking
    target_x_ = 0.0;
    target_y_ = 0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0*DEG2RAD;
    step_length_x_ = 0.10;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;

    t_rest_init_ = 0.04*hz_;
    t_rest_last_ = 0.04*hz_;
    t_double1_ = 0.03*hz_;
    t_double2_ = 0.03*hz_;
    t_total_= 0.6*hz_;
    foot_height_ = 0.050;      // 0.9 sec 0.05
    /////////////////////////////////

    // t_total_ = 0.5 * hz_;

    // t_rest_init_ = 0.08 * hz_; // slack
    // t_rest_last_ = 0.08 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 0.7 * hz_;

    // t_rest_init_ = 0.2 * hz_; // slack
    // t_rest_last_ = 0.2 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
    // t_total_ = 1.1 * hz_;

    t_temp_ = 1.0 * hz_;
    t_last_ = t_total_ + t_temp_;
    t_start_ = t_temp_ + 1;
    t_start_real_ = t_start_ + t_rest_init_;

    zmp_modif_time_margin_ = 0.1*hz_;

    current_step_num_ = 0;
    pelv_height_offset_ = 0.0; // change pelvis height for manipulation when the robot stop walking
    
    // current_time_computeslow_ = rd_.control_time_;
    // pre_time_computeslow_ = rd_.control_time_ - 1 / hz_;

    cout << "========= Walking Parameter Information ========" << endl;
    cout << "target_x_: " << target_x_ << endl;
    cout << "target_y_: " << target_y_ << endl;
    cout << "target_theta_: " << target_theta_ << endl;
    cout << "step_length_x_: " << step_length_x_ << endl;
    cout << "t_total_: " << t_total_ << endl;
    cout << "foot_height_: " << foot_height_ << endl;
    cout << "total_step_num_: " << total_step_num_ << endl;
    cout << "================================================" << endl;
}

void AvatarController::updateNextStepTime()
{
    if (walking_tick_mj >= t_last_)
    {
        if (current_step_num_ != total_step_num_ - 1)
        {
            // t_total_ = t_total_ + 0.05*hz_;
            t_start_ = t_last_ + 1;
            t_start_real_ = t_start_ + t_rest_init_;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;

            current_support_foot_is_left_ = !current_support_foot_is_left_;
        }
    }

    if (current_step_num_ == total_step_num_ - 1 && walking_tick_mj >= t_last_ + t_total_)
    {
        walking_enable_ = false;
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
}

void AvatarController::computeCAMcontrol_HQP()
{
    // const int hierarchy_num_camhqp_ = 2;
    // const int variable_size_camhqp_ = 8; 
    // const int constraint_size1_camhqp_ = 8; //[lb <=	x	<= 	ub] form constraints
    // const int constraint_size2_camhqp_[hierarchy_num_camhqp_] = {0, 3};	//[lb <=	Ax 	<=	ub] or [Ax = b]
    // const int control_size_camhqp_[hierarchy_num_camhqp_] = {3, 8}; //1: CAM control, 2: init pose
    if (first_loop_camhqp_)
    {
        for (int i = 0; i < hierarchy_num_camhqp_; i++)
        {   
            QP_cam_hqp_.resize(hierarchy_num_camhqp_);
            QP_cam_hqp_[i].InitializeProblemSize(variable_size_camhqp_, constraint_size2_camhqp_[i]);
            J_camhqp_[i].setZero(control_size_camhqp_[i], variable_size_camhqp_);
            u_dot_camhqp_[i].setZero(control_size_camhqp_[i]);

            ubA_camhqp_[i].setZero(constraint_size2_camhqp_[i]);
            lbA_camhqp_[i].setZero(constraint_size2_camhqp_[i]);

            H_camhqp_[i].setZero(variable_size_camhqp_, variable_size_camhqp_);
            g_camhqp_[i].setZero(variable_size_camhqp_);

            ub_camhqp_[i].setZero(constraint_size1_camhqp_);
            lb_camhqp_[i].setZero(constraint_size1_camhqp_);

            q_dot_camhqp_[i].setZero(variable_size_camhqp_);

            w1_camhqp_[0] = 2500; // |A*qdot - h|
            w2_camhqp_[0] = 50.000; // |q_dot|
            w3_camhqp_[0] = 0.0; //acceleration (0.000)  
            
            w1_camhqp_[1] = 2500; // |q_dot - q_dot_zero|
            w2_camhqp_[1] = 0.000; // |q_dot|
            w3_camhqp_[1] = 0.0; //acceleration (0.000)
        }
         
        control_joint_idx_camhqp_[0] = 13; // waist pitch
        control_joint_idx_camhqp_[1] = 14; // waist roll

        control_joint_idx_camhqp_[2] = 16; // left shoulder pitch
        control_joint_idx_camhqp_[3] = 17; // left shoulder roll
        control_joint_idx_camhqp_[4] = 19; // left elbow pitch

        control_joint_idx_camhqp_[5] = 26; // right shoulder pitch
        control_joint_idx_camhqp_[6] = 27; // right shoulder roll        
        control_joint_idx_camhqp_[7] = 29; // right elbow pitch

        last_solved_hierarchy_num_camhqp_ = -1;

        first_loop_camhqp_ = false;
    }

    Eigen::VectorXd q_test, q_dot_test;
    q_test = rd_.q_virtual_;
    // modify the virtual joint value from the value expressed in global frame to be the value expressed in the base frame
    // Eigen::Vector6d base_velocity;
    // base_velocity = rd_.q_dot_virtual_.segment(0, 6);
    // base_velocity.segment(0, 3) = rd_.link_[Pelvis].rotm.transpose() * base_velocity.segment(0, 3); // virtual joint velocity of the robot w.r.t pelvis frame
    // base_velocity.segment(3, 3) = rd_.link_[Pelvis].rotm.transpose() * base_velocity.segment(3, 3);

    q_test.segment(0, 6).setZero();
    q_test(39) = 1;
    // q_dot_test = rd_.q_dot_virtual_;
    // q_dot_test.segment(0, 6) = base_velocity;

    Eigen::MatrixXd mass_matrix_temp;
    Eigen::MatrixXd cmm;
    Eigen::MatrixXd cmm_support;
    Eigen::MatrixXd cmm_support1;
    Eigen::MatrixXd sel_matrix;
    Eigen::MatrixXd cmm_selected;
    Eigen::Vector2d eps;
    mass_matrix_temp.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    cmm.setZero(3, MODEL_DOF);
    cmm_support1.setZero(3, MODEL_DOF);
    cmm_support.setZero(2, MODEL_DOF);
    // sel_matrix.setZero(MODEL_DOF, MODEL_DOF);
    sel_matrix.setZero(MODEL_DOF, variable_size_camhqp_);
    cmm_selected.setZero(2, variable_size_camhqp_);
    // Defined the selection matrix //
    for(int i=0; i < variable_size_camhqp_; i++) // eight joints in upper body for CMP control
    {
        sel_matrix(control_joint_idx_camhqp_[i], i) = 1.0;  
    } 
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_MJ_, q_test, mass_matrix_temp, true);

    getCentroidalMomentumMatrix(mass_matrix_temp, cmm);
    cmm_support1 = pelv_support_current_.linear() * cmm; // considering the gravity direction
    cmm_support = cmm_support1.block<2, MODEL_DOF>(0,0);
    
    cmm_selected = cmm_support * sel_matrix;
    // for(int i = 0; i < 8; i ++)
    // {
    //     cmm_selected(2,i) = 0;
        
    // }
    // cmm_selected(2,0) = 2; 
    Eigen::Vector2d del_ang_momentum_slow_2;
    del_ang_momentum_slow_2 = del_ang_momentum_slow_.segment(0, 2); 
    J_camhqp_[0] = cmm_selected;
    //u_dot_camhqp_[0] = del_ang_momentum_slow_;
    u_dot_camhqp_[0] = del_ang_momentum_slow_2;
    J_camhqp_[1].setIdentity(8, 8);
    u_dot_camhqp_[1].setZero(control_size_camhqp_[1]); 
    
    for(int i =0; i < control_size_camhqp_[1]; i++) 
    {
        // recovery strategy
        u_dot_camhqp_[1](i) = 20*(CAM_upper_init_q_(control_joint_idx_camhqp_[i]) - motion_q_pre_(control_joint_idx_camhqp_[i]));        
    }

    for (int i = 0; i < hierarchy_num_camhqp_; i++)
    {
        if (i > last_solved_hierarchy_num_camhqp_)
        {
            QP_cam_hqp_[i].InitializeProblemSize(variable_size_camhqp_, constraint_size2_camhqp_[i]);
        }
    }
    //cout << u_dot_camhqp_[0] << "," << J_camhqp_[0] << endl;
    last_solved_hierarchy_num_camhqp_ = -1;
    for (int i = 0; i < hierarchy_num_camhqp_; i++)
    {
        MatrixXd H1, H2, H3;
        VectorXd g1, g2, g3;

        H1 = J_camhqp_[i].transpose() * J_camhqp_[i];
        H2 = Eigen::MatrixXd::Identity(variable_size_camhqp_, variable_size_camhqp_);
        //H2.resize(variable_size_camhqp_,variable_size_camhqp_);        
        //H2.setZero();// = Eigen::MatrixXd::Identity(variable_size_camhqp_, variable_size_camhqp_);
        //H2(0,0) = 0.1;
        H3 = Eigen::MatrixXd::Identity(variable_size_camhqp_, variable_size_camhqp_) * (1 / dt_) * (1 / dt_);
        //H3 = J_camhqp_[1].transpose() * J_camhqp_[1];
        g1 = -J_camhqp_[i].transpose() * u_dot_camhqp_[i]; // (variable_size_camhqp_ x 1 (i.e. 6x1))
        g2.setZero(variable_size_camhqp_); 
        g3.setZero(variable_size_camhqp_);  
        //g3 = -J_camhqp_[1].transpose() * u_dot_camhqp_[1];
        for(int j=0; j < variable_size_camhqp_; j++)
        {
            g3(j) = -motion_q_dot_pre_(control_joint_idx_camhqp_[i]) * (1 / dt_) * (1 / dt_);
        }
        
        H_camhqp_[i] = w1_camhqp_[i] * H1 + w2_camhqp_[i] * H2 + w3_camhqp_[i] * H3;
        g_camhqp_[i] = w1_camhqp_[i] * g1 + w2_camhqp_[i] * g2 + w3_camhqp_[i] * g3;

        double speed_reduce_rate = 40; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

        // for (int j = 0; j < constraint_size1_camhqp_; j++)
        // {
        //     lb_camhqp_[i](j) = min(max(speed_reduce_rate * (joint_limit_l_(control_joint_idx_camhqp_[j]) - current_q_(control_joint_idx_camhqp_[j])), joint_vel_limit_l_(control_joint_idx_camhqp_[j])), joint_vel_limit_h_(control_joint_idx_camhqp_[j]));
        //     ub_camhqp_[i](j) = max(min(speed_reduce_rate * (joint_limit_h_(control_joint_idx_camhqp_[j]) - current_q_(control_joint_idx_camhqp_[j])), joint_vel_limit_h_(control_joint_idx_camhqp_[j])), joint_vel_limit_l_(control_joint_idx_camhqp_[j]));
        // }
  
        // MJ's joint limit 
        lb_camhqp_[i](0) = min(max(speed_reduce_rate * (-20.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[0])), joint_vel_limit_l_(control_joint_idx_camhqp_[0])), joint_vel_limit_h_(control_joint_idx_camhqp_[0]));
        ub_camhqp_[i](0) = max(min(speed_reduce_rate * (20.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[0])), joint_vel_limit_h_(control_joint_idx_camhqp_[0])), joint_vel_limit_l_(control_joint_idx_camhqp_[0]));
        lb_camhqp_[i](1) = min(max(speed_reduce_rate * (-20.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[1])), joint_vel_limit_l_(control_joint_idx_camhqp_[1])), joint_vel_limit_h_(control_joint_idx_camhqp_[1]));
        ub_camhqp_[i](1) = max(min(speed_reduce_rate * (20.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[1])), joint_vel_limit_h_(control_joint_idx_camhqp_[1])), joint_vel_limit_l_(control_joint_idx_camhqp_[1]));
        // Left Shoulder pitch
        lb_camhqp_[i](2) = min(max(speed_reduce_rate * (-40*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[2])), joint_vel_limit_l_(control_joint_idx_camhqp_[2])), joint_vel_limit_h_(control_joint_idx_camhqp_[2]));
        ub_camhqp_[i](2) = max(min(speed_reduce_rate * (40*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[2])), joint_vel_limit_h_(control_joint_idx_camhqp_[2])), joint_vel_limit_l_(control_joint_idx_camhqp_[2]));
        // Left Shoulder roll // pitch에서 외란에서 롤을 생각보다 많이 써서 50 -> 30
        lb_camhqp_[i](3) = min(max(speed_reduce_rate * (30*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[3])), joint_vel_limit_l_(control_joint_idx_camhqp_[3])), joint_vel_limit_h_(control_joint_idx_camhqp_[3]));
        ub_camhqp_[i](3) = max(min(speed_reduce_rate * (80*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[3])), joint_vel_limit_h_(control_joint_idx_camhqp_[3])), joint_vel_limit_l_(control_joint_idx_camhqp_[3]));
        // Left Elbow pitch
        lb_camhqp_[i](4) = min(max(speed_reduce_rate * (-110*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[4])), joint_vel_limit_l_(control_joint_idx_camhqp_[4])), joint_vel_limit_h_(control_joint_idx_camhqp_[4]));
        ub_camhqp_[i](4) = max(min(speed_reduce_rate * (40*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[4])), joint_vel_limit_h_(control_joint_idx_camhqp_[4])), joint_vel_limit_l_(control_joint_idx_camhqp_[4]));
        // Right Shoulder pitch
        lb_camhqp_[i](5) = min(max(speed_reduce_rate * (-40*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[5])), joint_vel_limit_l_(control_joint_idx_camhqp_[5])), joint_vel_limit_h_(control_joint_idx_camhqp_[4]));
        ub_camhqp_[i](5) = max(min(speed_reduce_rate * (40*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[5])), joint_vel_limit_h_(control_joint_idx_camhqp_[5])), joint_vel_limit_l_(control_joint_idx_camhqp_[4]));
        // Right Shoulder roll
        lb_camhqp_[i](6) = min(max(speed_reduce_rate * (-80*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[6])), joint_vel_limit_l_(control_joint_idx_camhqp_[6])), joint_vel_limit_h_(control_joint_idx_camhqp_[5]));
        ub_camhqp_[i](6) = max(min(speed_reduce_rate * (-30*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[6])), joint_vel_limit_h_(control_joint_idx_camhqp_[6])), joint_vel_limit_l_(control_joint_idx_camhqp_[5]));
        // Right Elbow pitch
        lb_camhqp_[i](7) = min(max(speed_reduce_rate * (-40*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[7])), joint_vel_limit_l_(control_joint_idx_camhqp_[7])), joint_vel_limit_h_(control_joint_idx_camhqp_[5]));
        ub_camhqp_[i](7) = max(min(speed_reduce_rate * (110*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[7])), joint_vel_limit_h_(control_joint_idx_camhqp_[7])), joint_vel_limit_l_(control_joint_idx_camhqp_[5]));


        int higher_task_equality_num = 0;        
     
        for(int k = 0; k < constraint_size2_camhqp_[1]; k++)
        {
            eps(k) = DyrosMath::cubic(del_ang_momentum_slow_2.norm(), 1, 3, 0.4, 0.0, 0.0, 0.0);
        }

        for (int h = 0; h < i; h++)
        {
            if (constraint_size2_camhqp_[i] == 0)
            {
                break;
            }

            A_camhqp_[i].setZero(constraint_size2_camhqp_[i], variable_size_camhqp_);
            A_camhqp_[i].block(higher_task_equality_num, 0, control_size_camhqp_[h], variable_size_camhqp_) = J_camhqp_[h];
            ubA_camhqp_[i].segment(higher_task_equality_num, control_size_camhqp_[h]) = J_camhqp_[h] * q_dot_camhqp_[i-1] + eps;
            lbA_camhqp_[i].segment(higher_task_equality_num, control_size_camhqp_[h]) = J_camhqp_[h] * q_dot_camhqp_[i-1] - eps;
            higher_task_equality_num += control_size_camhqp_[h];
            //cout << J_camhqp_[h] * q_dot_camhqp_[i - 1] << "," <<  endl;
        }

        // QP_cam_hqp_[i].SetPrintLevel(PL_NONE);
        QP_cam_hqp_[i].EnableEqualityCondition(equality_condition_eps_);
        QP_cam_hqp_[i].UpdateMinProblem(H_camhqp_[i], g_camhqp_[i]);
        if(constraint_size2_camhqp_[i] == 0)
        {             
            QP_cam_hqp_[i].DeleteSubjectToAx();
        }
        else
        {
            QP_cam_hqp_[i].UpdateSubjectToAx(A_camhqp_[i], lbA_camhqp_[i], ubA_camhqp_[i]);
        }
        
        if(constraint_size1_camhqp_ == 0)
        {
            QP_cam_hqp_[i].DeleteSubjectToX();
        }
        else
        {
            QP_cam_hqp_[i].UpdateSubjectToX(lb_camhqp_[i], ub_camhqp_[i]);
        }        

        if (QP_cam_hqp_[i].SolveQPoases(200, qpres_camhqp_))
        {
            q_dot_camhqp_[i] = qpres_camhqp_.segment(0, variable_size_camhqp_);

            last_solved_hierarchy_num_camhqp_ = i;
        }
        else
        {
            q_dot_camhqp_[i].setZero();

            // if (int(current_time_ * 2000) % 1000 == 0)
            {
                std::cout << "Error hierarchy: " << i << std::endl;
                std::cout << "last solved q_dot: " << q_dot_camhqp_[last_solved_hierarchy_num_camhqp_].transpose() << std::endl;
            }
            
            break;
        }
    }
        
    for (int i = 0; i < variable_size_camhqp_; i++)
    {
        //motion_q_dot_(control_joint_idx_camhqp_[i]) = q_dot_camhqp_[0](i); // first hierarchy solution
        motion_q_dot_(control_joint_idx_camhqp_[i]) = q_dot_camhqp_[last_solved_hierarchy_num_camhqp_](i);
        motion_q_(control_joint_idx_camhqp_[i]) = motion_q_pre_(control_joint_idx_camhqp_[i]) + motion_q_dot_(control_joint_idx_camhqp_[i]) * dt_;
        pd_control_mask_(control_joint_idx_camhqp_[i]) = 1;
    }

    // CAM calculation based on actual joint angular velocity    
    Eigen::VectorXd cam_a;
    Eigen::VectorXd real_q_dot_hqp_; 
    cam_a.setZero(3);
    real_q_dot_hqp_.setZero(8);
    
    for(int i = 0; i < variable_size_camhqp_; i++)
    {
      real_q_dot_hqp_(i) = rd_.q_dot_(control_joint_idx_camhqp_[i]);  
    }
    cam_a = -J_camhqp_[0]*real_q_dot_hqp_;

    // // CAM calculation based on commanded joint angular velocity    
    Eigen::VectorXd cam_c;
    Eigen::VectorXd comm_q_dot_hqp_;
    cam_c.setZero(3);
    comm_q_dot_hqp_.setZero(8);

    for(int i = 0; i < variable_size_camhqp_; i++)
    {
      comm_q_dot_hqp_(i) = motion_q_dot_(control_joint_idx_camhqp_[i]);  
    }
    cam_c = -J_camhqp_[0]*comm_q_dot_hqp_;
   
    //if((walking_tick_mj % 100) == 0 )
    // {
    //     MJ_q_ << motion_q_(13) << "," << motion_q_(14) << "," << motion_q_(16) << "," << motion_q_(17) << "," << motion_q_(19) << "," << motion_q_(26) << "," << motion_q_(27) << "," << motion_q_(29) << endl;
    //     MJ_q_dot_ << motion_q_dot_(13) << "," << motion_q_dot_(14) << "," << motion_q_dot_(16) << "," << motion_q_dot_(17) << "," << motion_q_dot_(19) << "," << motion_q_dot_(26) << "," << motion_q_dot_(27) << "," << motion_q_dot_(29) << endl;
    //     MJ_CAM_ << cam_a(0) << "," << cam_a(1) << "," << cam_c(0) << "," << cam_c(1) << "," << del_ang_momentum_slow_(0) << "," << del_ang_momentum_slow_(1) << "," << del_ang_momentum_slow_2.norm() << "," << del_tau_(1) << "," << eps(0) << endl; 
    //     MJ_CP_ZMP << ZMP_X_REF << "," << ZMP_Y_REF_alpha_ << "," << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << "," << cp_desired_(0) << "," << cp_desired_(1) << "," << cp_measured_(0) << "," << cp_measured_(1) << endl;
    // }        
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
    double compliant_tick = 0.0 * hz_;
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

    F_R = (1 - alpha) * rd_.link_[COM_id].mass * GRAVITY;
    F_L = alpha * rd_.link_[COM_id].mass * GRAVITY;

    Tau_CP(4) = F_L * del_zmp(0);  // L pitch
    Tau_CP(10) = F_R * del_zmp(0); // R pitch

    Tau_CP(5) = -F_L * del_zmp(1);  // L roll
    Tau_CP(11) = -F_R * del_zmp(1); // R roll
}

// void AvatarController::CP_compen_MJ_FT()
// { // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
//     double alpha = 0;
//     double F_R = 0, F_L = 0;
//     double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
//     double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;
//     double zmp_offset = 0;
//     double alpha_new = 0;

//     zmp_offset = 0.01; // zmp_offset 함수 참고
            
//     // Preview를 이용한 COM 생성시 ZMP offset을 x cm 안쪽으로 넣었지만, alpha 계산은 x cm 넣으면 안되기 때문에 조정해주는 코드
//     // 어떻게 보면 COM, CP 궤적은 ZMP offset이 반영되었고, CP 제어기는 반영안시킨게 안맞는거 같기도함
//     if (walking_tick_mj > t_temp_)
//     {
//         if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
//         {
//             if (foot_step_(current_step_num_, 6) == 1)
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF + zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
//             }
//             else
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF - zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
//             }
//         }
//         else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
//         {
//             if (foot_step_(current_step_num_, 6) == 1)
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF + zmp_offset;
//             }
//             else
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF - zmp_offset;
//             }
//         }
//         else if (walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
//         {
//             if (foot_step_(current_step_num_, 6) == 1)
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF + zmp_offset - zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
//             }
//             else
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF - zmp_offset + zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
//             }
//         }
//         else
//         {
//             ZMP_Y_REF_alpha_ = ZMP_Y_REF;
//         }
//     }
//     else
//     {
//         ZMP_Y_REF_alpha_ = ZMP_Y_REF;
//     }
     
//     del_zmp(0) = des_zmp_interpol_(0) - ZMP_X_REF;
//     del_zmp(1) = des_zmp_interpol_(1) - ZMP_Y_REF_alpha_;
  
//     // del_zmp(0) = DyrosMath::minmax_cut(del_zmp(0), -0.1, 0.1);
//     // del_zmp(1) = DyrosMath::minmax_cut(del_zmp(1), -0.07, 0.07); 

//     ////////////////////////
//     // double A = 0, B = 0, d = 0, X1 = 0, Y1 = 0, e_2 = 0, L = 0, l = 0;
//     // A = (lfoot_support_current_.translation()(0) - rfoot_support_current_.translation()(0));
//     // B = -(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
//     // X1 = ZMP_Y_REF_alpha_ + 0 * del_zmp(1) - rfoot_support_current_.translation()(1);
//     // Y1 = ZMP_X_REF + 0 * del_zmp(0) - rfoot_support_current_.translation()(0);
//     // L = sqrt(A * A + B * B);
//     // d = abs(A * X1 + B * Y1) / L;
//     // e_2 = X1 * X1 + Y1 * Y1;
//     // l = sqrt(e_2 - d * d);
//     // alpha_new = l / L;
//     alpha = (ZMP_Y_REF_alpha_ + del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
//     if(walking_tick_mj == 0)
//     {
//         alpha_lpf_ = alpha;
//     }
//     alpha_lpf_ = 1 / (1 + 2 * M_PI * 6.0 * del_t) * alpha_lpf_ + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * alpha;
//     //cout << alpha << "," << ZMP_Y_REF << "," << rfoot_support_current_.translation()(1) << "," << lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1) << endl;
//     // 로봇에서 구현할때 alpha가 0~1로 나오는지 확인, ZMP offset 0으로 해야됨.
//     if (alpha > 1)
//     {
//         alpha = 1;
//     } // 왼발 지지때 alpha = 1
//     else if (alpha < 0)
//     {
//         alpha = 0;
//     }
//     if (alpha_new > 1)
//     {
//         alpha_new = 1;
//     } // 왼발 지지때 alpha = 1
//     else if (alpha_new < 0)
//     {
//         alpha_new = 0;
//     }
//     if (alpha_lpf_ > 1)
//     {
//         alpha_lpf_ = 1;
//     } // 왼발 지지때 alpha = 1
//     else if (alpha_lpf_ < 0)
//     {
//         alpha_lpf_ = 0;
//     }

//     F_R = -(1 - alpha_lpf_) * rd_.link_[COM_id].mass * GRAVITY;
//     F_L = -alpha_lpf_ * rd_.link_[COM_id].mass * GRAVITY; // alpha가 0~1이 아니면 desired force가 로봇 무게보다 계속 작게나와서 지면 반발력을 줄이기위해 다리길이를 줄임.

//     if (walking_tick_mj == 0)
//     {
//         F_F_input = 0.0;
//         F_T_L_x_input = 0.0;
//         F_T_R_x_input = 0.0;
//         F_T_L_y_input = 0.0;
//         F_T_R_y_input = 0.0;
//     }
//     //////////// Force
//     F_F_input_dot = 0.0005 * ((l_ft_(2) - r_ft_(2)) - (F_L - F_R)) - 3.0 * F_F_input; // F_F_input이 크면 다리를 원래대로 빨리줄인다. 이정도 게인 적당한듯0.001/0.00001 // SSP, DSP 게인값 바꿔야?
//     F_F_input = F_F_input + F_F_input_dot * del_t;

//     if (F_F_input >= 0.02)
//     {
//         F_F_input = 0.02;
//     }
//     else if (F_F_input <= -0.02)
//     {
//         F_F_input = -0.02;
//     }

//     //////////// Torque
//     // X,Y 축을 X,Y 방향으로 헷갈렸었고, 위치 명령을 발목 IK각도에 바로 넣었었음.
//     Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_L);
//     Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_L);

//     if (Tau_all_x > 100)
//     {
//         Tau_all_x = 100;
//     }
//     else if (Tau_all_x < -100)
//     {
//         Tau_all_x = -100;
//     }

//     if (Tau_all_y > 100)
//     {
//         Tau_all_y = 100;
//     }
//     else if (Tau_all_y < -100)
//     {
//         Tau_all_y = -100;
//     }

//     Tau_R_x = (1 - alpha) * Tau_all_x;
//     Tau_L_x = (alpha)*Tau_all_x;

//     Tau_L_y = -alpha * Tau_all_y;
//     Tau_R_y = -(1 - alpha) * Tau_all_y;

//     double Kr_roll = 0.0, Kl_roll = 0.0;
//     double Kr_pitch = 0.0, Kl_pitch = 0.0;

//     if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
//     {
//         Kr_roll = 10.0;
//         Kl_roll = 10.0;
//         Kr_pitch = 10.0;
//         Kl_pitch = 10.0;
//     }
//     else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
//     {
//         if (alpha == 1) // 왼발 지지
//         {
//             Kl_roll = 10.0;
//             Kr_roll = 50.0;
//             Kl_pitch = 10.0;
//             Kr_pitch = 50.0;
//         }
//         if (alpha == 0) // 오른발 지지
//         {
//             Kl_roll = 50.0;
//             Kr_roll = 10.0;
//             Kl_pitch = 50.0;
//             Kr_pitch = 10.0;
//         }
//     }
//     else
//     {
//         Kr_roll = 10.0;
//         Kl_roll = 10.0;
//         Kr_pitch = 10.0;
//         Kl_pitch = 10.0;
//     }

//     //Roll 방향 -0.3,50 -> High performance , -0.1, 50 평지 보행 적당
//     F_T_L_x_input_dot = -0.1 * (Tau_L_x - l_ft_LPF(3)) - 50.0 * F_T_L_x_input;
//     F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
//     //F_T_L_x_input = 0;
//     F_T_R_x_input_dot = -0.1 * (Tau_R_x - r_ft_LPF(3)) - 50.0 * F_T_R_x_input;
//     F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
//     //F_T_R_x_input = 0;

//     //Pitch 방향
//     F_T_L_y_input_dot = 0.1 * (Tau_L_y - l_ft_LPF(4)) - 50.0 * F_T_L_y_input;
//     F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
//     //F_T_L_y_input = 0;
//     F_T_R_y_input_dot = 0.1 * (Tau_R_y - r_ft_LPF(4)) - 50.0 * F_T_R_y_input;
//     F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t; 
//     //F_T_R_y_input = 0;

//     if (F_T_L_x_input >= 0.15) // 8.5 deg limit
//     {
//         F_T_L_x_input = 0.15;
//     }
//     else if (F_T_L_x_input < -0.15)
//     {
//         F_T_L_x_input = -0.15;
//     }

//     if (F_T_R_x_input >= 0.15) // 8.5 deg limit
//     {
//         F_T_R_x_input = 0.15;
//     }
//     else if (F_T_R_x_input < -0.15)
//     {
//         F_T_R_x_input = -0.15;
//     }

//     if (F_T_L_y_input >= 0.15) // 8.5 deg limit
//     {
//         F_T_L_y_input = 0.15;
//     }
//     else if (F_T_L_y_input < -0.15)
//     {
//         F_T_L_y_input = -0.15;
//     }

//     if (F_T_R_y_input >= 0.15) // 8.5 deg limit
//     {
//         F_T_R_y_input = 0.15;
//     }
//     else if (F_T_R_y_input < -0.15)
//     {
//         F_T_R_y_input = -0.15;
//     }    

//     // Tau_CP(4) = F_L * del_zmp(0);  // L pitch
//     // Tau_CP(10) = F_R * del_zmp(0); // R pitch

//     // Tau_CP(5) = -F_L * del_zmp(1);  // L roll
//     // Tau_CP(11) = -F_R * del_zmp(1); // R roll

//     // MJ_graph << stepping_input_(0) << "," << stepping_input_(1) << "," << t_total_ / hz_ << "," << ZMP_Y_REF_alpha_ + del_zmp(1) << "," << ZMP_Y_REF_alpha_ << endl;
// }

void AvatarController::CP_compen_MJ_FT()
{ 
    // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
    double alpha = 0;
    double F_R = 0, F_L = 0;
    double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
    double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;
    double zmp_offset = 0;
    double alpha_new = 0;

    // zmp_offset = 0.015; // 0.9초
    // zmp_offset = 0.02; // 1.1초
    // zmp_offset = 0.015; // 1.3초

    // Preview를 이용한 COM 생성시 ZMP offset을 2cm 안쪽으로 넣었지만, alpha 계산은 2cm 넣으면 안되기 때문에 조정해주는 코드
    // 어떻게 보면 COM, CP 궤적은 ZMP offset이 반영되었고, CP 제어기는 반영안시킨게 안맞는거 같기도함
    if (walking_tick_mj > t_temp_)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF + zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF - zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF + zmp_offset;
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF - zmp_offset;
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF + zmp_offset - zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF - zmp_offset + zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
        }
        else
        {
            ZMP_Y_REF_alpha_ = ZMP_Y_REF;
        }
    }
    else
    {
        ZMP_Y_REF_alpha_ = ZMP_Y_REF;
    }

    del_zmp(0) = des_zmp_interpol_(0) - ZMP_X_REF;
    del_zmp(1) = des_zmp_interpol_(1) - ZMP_Y_REF_alpha_;

    ////////////////////////
    //   double A = 0, B = 0, d = 0, X1 = 0, Y1 = 0, e_2 = 0, L = 0, l = 0;
    //   A =  (lfoot_support_current_.translation()(0) - rfoot_support_current_.translation()(0));
    //   B = -(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    //   X1 = ZMP_Y_REF_alpha_ + 0*del_zmp(1) - rfoot_support_current_.translation()(1);
    //   Y1 = ZMP_X_REF + 0*del_zmp(0) - rfoot_support_current_.translation()(0);
    //   L = sqrt(A*A + B*B);
    //   d = abs(A*X1 + B*Y1) / L;
    //   e_2 = X1*X1 + Y1*Y1;
    //   l = sqrt(e_2 - d*d);
    //   alpha_new = l/L;
    alpha = (ZMP_Y_REF_alpha_ + del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
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

    // if( walking_tick_mj%500 == 0)
    // {
    //     cout <<"alpha: "<< alpha << endl;
    // }
    //   if(alpha_new > 1)
    //   { alpha_new = 1; } // 왼발 지지때 alpha = 1
    //   else if(alpha_new < 0)
    //   { alpha_new = 0; }

    double real_robot_mass_offset_ = 0; // 81: no baterry, no hands, w/ avatar head and backpack, 129: w battery
    double right_left_force_diff = -0.0; // heavy foot: 15, small foot: -15

    F_R = -(1 - alpha) * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ + right_left_force_diff);
    F_L = -alpha * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ - right_left_force_diff); // alpha가 0~1이 아니면 desired force가 로봇 무게보다 계속 작게나와서 지면 반발력을 줄이기위해 다리길이를 줄임.

    if (walking_tick_mj == 0)
    {
        F_F_input = 0.0;
        F_T_L_x_input = 0.0;
        F_T_R_x_input = 0.0;
        F_T_L_y_input = 0.0;
        F_T_R_y_input = 0.0;
    }

    //////////// Force
    F_F_input_dot = 0.00010 * ((l_ft_LPF(2) - r_ft_LPF(2)) - (F_L - F_R)) - 3.0 * F_F_input; // 0.9초 0.0001/ 3.0

    F_F_input = F_F_input + F_F_input_dot * del_t;

    if (F_F_input >= 0.02) // 1.1초 0.02
    {
        F_F_input = 0.03;
        // cout << "F_F_input max" << endl;
    }
    else if (F_F_input <= -0.02)
    {
        F_F_input = -0.03;
        // cout << "F_F_input min" << endl;
    }
    //   if(F_F_input >= 0.01) // 0.9초 0.01
    //   {
    //     F_F_input = 0.01;
    //   }
    //   else if(F_F_input <= -0.01)
    //   {
    //     F_F_input = -0.01;
    //   }
    // MJ_graph << ZMP_Y_REF << "," << ZMP_Y_REF_alpha_ << "," << alpha << "," << F_L << "," << F_R << "," << F_F_input << "," << l_ft_LPF(2) << "," << r_ft_LPF(2) <<  endl;

    //////////// Torque
    // X,Y 축을 X,Y 방향으로 헷갈렸었고, 위치 명령을 발목 IK각도에 바로 넣었었음.
    Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_L);
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
        Kr_roll = 30.0;
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

    // Roll 방향 (-0.02/-30 0.9초) large foot(blue pad): 0.05/50 / small foot(orange pad): 0.07/50
    //   F_T_L_x_input_dot = -0.015*(Tau_L_x - l_ft_LPF(3)) - Kl_roll*F_T_L_x_input;
    F_T_L_x_input_dot = -0.04 * (Tau_L_x - l_ft_LPF(3)) - 40.0 * F_T_L_x_input;
    F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
    //   F_T_L_x_input = 0;
    //   F_T_R_x_input_dot = -0.015*(Tau_R_x - r_ft_LPF(3)) - Kr_roll*F_T_R_x_input;
    F_T_R_x_input_dot = -0.04 * (Tau_R_x - r_ft_LPF(3)) - 40.0 * F_T_R_x_input;
    F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
    //   F_T_R_x_input = 0;

    // Pitch 방향  (0.005/-30 0.9초) large foot(blue pad): 0.04/50 small foot(orange pad): 0.06/50
    //   F_T_L_y_input_dot = 0.005*(Tau_L_y - l_ft_LPF(4)) - Kl_pitch*F_T_L_y_input;
    F_T_L_y_input_dot = 0.040 * (Tau_L_y - l_ft_LPF(4)) - 40.0 * F_T_L_y_input;
    F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
    //   F_T_L_y_input = 0;
    //   F_T_R_y_input_dot = 0.005*(Tau_R_y - r_ft_LPF(4)) - Kr_pitch*F_T_R_y_input;
    F_T_R_y_input_dot = 0.040 * (Tau_R_y - r_ft_LPF(4)) - 40.0 * F_T_R_y_input;
    F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;
    //   F_T_R_y_input = 0;
    // MJ_graph << l_ft_LPF(2) - r_ft_LPF(2) << "," <<  (F_L - F_R) << "," << F_F_input << endl;
    // MJ_graph << F_T_L_x_input << "," << F_T_R_x_input << "," <<  F_T_L_y_input << "," <<  F_T_R_y_input << "," << F_F_input << "," << cp_measured_(1) << "," << cp_desired_(1) << endl;
    if (F_T_L_x_input >= 0.2) // 5 deg limit
    {
        F_T_L_x_input = 0.2;
        // cout << "F_T_L_x_input max" << endl;
    }
    else if (F_T_L_x_input < -0.2)
    {
        F_T_L_x_input = -0.2;
        // cout << "F_T_L_x_input min" << endl;
    }

    if (F_T_R_x_input >= 0.2) // 5 deg limit
    {
        F_T_R_x_input = 0.2;
        // cout << "F_T_R_x_input max" << endl;
    }
    else if (F_T_R_x_input < -0.2)
    {
        F_T_R_x_input = -0.2;
        // cout << "F_T_R_x_input min" << endl;
    }

    if (F_T_L_y_input >= 0.2) // 5 deg limit
    {
        F_T_L_y_input = 0.2;
    }
    else if (F_T_L_y_input < -0.2)
    {
        F_T_L_y_input = -0.2;
    }

    if (F_T_R_y_input >= 0.2) // 5 deg limit
    {
        F_T_R_y_input = 0.2;
    }
    else if (F_T_R_y_input < -0.2)
    {
        F_T_R_y_input = -0.2;
    }

    // MJ_joint1 << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << "," << ZMP_Y_REF << "," << ZMP_Y_REF_alpha_ << "," << l_ft_LPF(2) << "," << r_ft_LPF(2) << endl;
    // cout << F_T_R_x_input*180/3.141592 << "," << F_T_L_x_input*180/3.141592 << "," << Tau_R_x << "," << Tau_L_x << "," << r_ft_(3) << "," << l_ft_(3) << endl;
    // MJ_graph << alpha << "," << alpha_new << endl;
    // MJ_graph << rfoot_support_current_.translation()(1) << "," << lfoot_support_current_.translation()(1) << "," << ZMP_Y_REF << "," << Tau_R_y << "," << Tau_L_y << endl;
    // MJ_graph << Tau_L_x << "," << Tau_R_x << "," << l_ft_LPF(3) << "," << r_ft_LPF(3) << "," << Tau_L_y << "," << Tau_R_y << "," << l_ft_LPF(4) << "," << r_ft_LPF(4) << "," << F_F_input << endl;
    // MJ_graph << ZMP_Y_REF << "," << alpha << "," << ZMP_Y_REF_alpha_ << endl;
    // MJ_graph << Tau_all_y << "," << Tau_L_y << "," << Tau_R_y << "," << l_ft_(4) << "," << r_ft_(4) << "," << cp_measured_(0) << "," << cp_desired_(0) << endl;
}

void AvatarController::CentroidalMomentCalculator()
{
    del_cmp(0) = 1.4 * (cp_measured_(0) - cp_desired_(0));
    del_cmp(1) = 1.3 * (cp_measured_(1) - cp_desired_(1));
    double support_ratio = 0.0;
    double M_G = rd_.link_[COM_id].mass * GRAVITY;
    support_ratio = (ZMP_Y_REF_alpha_ - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));

    if (support_ratio > 1) // 왼발 지지때 alpha = 1
    {
        support_ratio = 1;
    }
    else if (support_ratio < 0)
    {
        support_ratio = 0;
    }

    Eigen::Vector2d cmp_limit_;
    Eigen::Vector2d del_tau_limit_;

    double foot_width_x_front = 0.180; // margin = 0.8 / original foot width in urdf -> 0.18/0.12, small foot: 0.16/0.10
    double foot_width_x_back = 0.120;
    double foot_width_y = 0.065; // margin = 0.8 / original foot width in urdf -> 0.065, small foot: 0.05
    double support_margin = 1.0;
    // del tau output limitation (220118/ DLR's CAM output is an approximately 20 Nm (maximum) and TORO has a weight of 79.2 kg)
    del_tau_limit_(0) = 20.0;
    del_tau_limit_(1) = 20.0;

    del_cmp(0) = DyrosMath::minmax_cut(del_cmp(0), -(foot_width_x_back * support_margin + del_tau_limit_(0) / M_G), foot_width_x_front * support_margin + del_tau_limit_(0) / M_G);
    del_cmp(1) = DyrosMath::minmax_cut(del_cmp(1), -(foot_width_y * support_margin + del_tau_limit_(1) / M_G), foot_width_y * support_margin + del_tau_limit_(1) / M_G);

    if (walking_tick_mj == 0)
    {
        del_tau_.setZero();
        del_ang_momentum_.setZero();
        del_ang_momentum_prev_.setZero();
    }

    del_ang_momentum_prev_ = del_ang_momentum_;

    double rbs_ratio = 1.0;        // recovery bondary safety ratio
    double recovery_damping = 7.0; // damping 20 is equivalent to 0,99 exp gain // 2정도 하면 반대방향으로 치는게 15Nm, 20하면 150Nm

    // X direction CP control
    if (del_cmp(0) > foot_width_x_front * support_margin)
    {
        del_zmp(0) = foot_width_x_front * support_margin;
        del_tau_(1) = (del_cmp(0) - del_zmp(0)) * M_G; // Y axis delta angular moment , X direction CP control
        torque_flag_x = 0;
    }
    else if (del_cmp(0) <= foot_width_x_front * support_margin && del_cmp(0) > -foot_width_x_back * support_margin)
    {
        del_zmp(0) = del_cmp(0);
        del_tau_(1) = -recovery_damping * del_ang_momentum_(1);
        del_tau_(1) = DyrosMath::minmax_cut(del_tau_(1), (-foot_width_x_back * support_margin * rbs_ratio - zmp_measured_mj_(0)) * M_G, (foot_width_x_front * support_margin * rbs_ratio - zmp_measured_mj_(0)) * M_G);
        torque_flag_x = 1;
    }
    else if (del_cmp(0) < -foot_width_x_back * support_margin)
    {
        del_zmp(0) = -foot_width_x_back * support_margin;
        del_tau_(1) = (del_cmp(0) - del_zmp(0)) * M_G;
        torque_flag_x = 0;
    }

    // Y direction CP control
    if (del_cmp(1) > foot_width_y * support_margin)
    {
        del_zmp(1) = foot_width_y * support_margin;
        del_tau_(0) = -(del_cmp(1) - del_zmp(1)) * M_G;
        torque_flag_y = 0;
    }
    else if (del_cmp(1) <= foot_width_y * support_margin && del_cmp(1) > -foot_width_y * support_margin)
    {
        torque_flag_y = 1;
        del_zmp(1) = del_cmp(1);
        del_tau_(0) = -recovery_damping * del_ang_momentum_(0);
        // del_tau_(0) = DyrosMath::minmax_cut( del_tau_(0), -(foot_width_y*rbs_ratio - zmp_measured_mj_(1))*M_G, -(-foot_width_y*rbs_ratio - zmp_measured_mj_(1))*M_G );

        // if(support_ratio == 1) // SSP 왼발 지지
        //  if (foot_step_(current_step_num_, 6) == 1) // 문제는 DSP때 CAM을 0으로 보내서 토크 리밋이 별 소용이없음.
        //  {
        //      if(del_tau_(0) < -(foot_width_y*rbs_ratio - zmp_measured_mj_(1))*M_G)
        //      {
        //          del_tau_(0) = -(foot_width_y*rbs_ratio - zmp_measured_mj_(1))*M_G;
        //      }
        //  }
        //  else
        //  {
        //      if(del_tau_(0) > -(-foot_width_y*rbs_ratio - zmp_measured_mj_(1))*M_G )
        //      {
        //          del_tau_(0) = -(-foot_width_y*rbs_ratio - zmp_measured_mj_(1))*M_G ;
        //      }
        //  }
    }
    else if (del_cmp(1) < -foot_width_y * support_margin)
    {
        torque_flag_y = 0;
        del_zmp(1) = -foot_width_y * support_margin;
        del_tau_(0) = -(del_cmp(1) - del_zmp(1)) * M_G;
    }

    // if(del_tau_(0) > del_tau_limit_(0))
    // {
    //     del_tau_(0) = del_tau_limit_(0);
    // }
    // else if(del_tau_(0) < -del_tau_limit_(0))
    // {
    //     del_tau_(0) = -del_tau_limit_(0);
    // }
    // if(del_tau_(1) > del_tau_limit_(1))
    // {
    //     del_tau_(1) = del_tau_limit_(1);
    // }
    // else if(del_tau_(1) < -del_tau_limit_(1))
    // {
    //     del_tau_(1) = -del_tau_limit_(1);
    // }

    //// Integrate Centroidal Moment
    del_ang_momentum_(1) = del_ang_momentum_prev_(1) + del_t * del_tau_(1);
    del_ang_momentum_(0) = del_ang_momentum_prev_(0) + del_t * del_tau_(0);

    // del CAM output limitation (220118/ DLR's CAM output is an approximately 4 Nms and TORO has a weight of 79.2 kg)
    double A_limit = 15.0;

    if (del_ang_momentum_(0) > A_limit)
    {
        del_ang_momentum_(0) = A_limit;
    }
    else if (del_ang_momentum_(0) < -A_limit)
    {
        del_ang_momentum_(0) = -A_limit;
    }

    if (del_ang_momentum_(1) > A_limit)
    {
        del_ang_momentum_(1) = A_limit;
    }
    else if (del_ang_momentum_(1) < -A_limit)
    {
        del_ang_momentum_(1) = -A_limit;
    }

    CLIPM_ZMP_compen_MJ(del_zmp(0), del_zmp(1)); // 원래 previewcontroller 자리에 있던 함수.
}

void AvatarController::getCentroidalMomentumMatrix(MatrixXd mass_matrix, MatrixXd &CMM)
{ // 1. CMM 계산 함수.
    // reference: Cavenago, et.al "Contact force observer for space robots." 2019 IEEE 58th Conference on Decision and Control (CDC). IEEE, 2019.
    //  mass_matrix: inertia matrix expressed in the base frame
    //  Using this CMM, calculation error of the angular velocity occurs in second decimal place with respect to the rbdl CalcCenterOfMass() function when tocabi is walking in place.
    //  Calculation Time : 3~4us
    const int joint_dof = MODEL_DOF;
    double mass = mass_matrix(0, 0);
    MatrixXd M_r = mass_matrix.block(3, 3, 3, 3);
    MatrixXd M_t = mass_matrix.block(0, 0, 3, 3);
    MatrixXd M_tr = mass_matrix.block(0, 3, 3, 3);
    MatrixXd M_rt = mass_matrix.block(3, 0, 3, 3);
    MatrixXd M_rm = mass_matrix.block(3, 6, 3, MODEL_DOF);
    MatrixXd M_tm = mass_matrix.block(0, 6, 3, MODEL_DOF);
    MatrixXd J_w;
    J_w.setZero(3, MODEL_DOF);
    J_w = (M_r - (M_rt * M_tr) / mass).inverse() * (M_rm - (M_rt * M_tm) / mass);
    // MatrixXd CMM;
    CMM.setZero(3, MODEL_DOF);
    CMM = M_r * J_w;

    // return CMM;
}

void AvatarController::updateInitialStateJoy()
{
    //joystick footstep Calculation

    if(walking_stop_flag_ == true)
    {   
        if(stopping_step_planning_trigger_ == true)
        {
            if(walking_tick_mj != 0)
            {
                joy_input_enable_ = false;
            }

            calculateFootStepTotalOmniEnd(current_support_foot_is_left_);

            stopping_step_planning_trigger_ = false;
        }
    }
    else
    {
        calculateFootStepTotalOmni(del_x_command_, del_y_command_, yaw_angle_command_, current_support_foot_is_left_);
    }

    if (walking_tick_mj == 0)
    {
        // calculateFootStepTotal_MJoy(); // joystick&pedal Footstep
        
        // cout<<"current_step_num_: "<<current_step_num_<<endl;
        // cout<<"current_support_foot_is_left_: "<<current_support_foot_is_left_<<endl;
        // cout<<"foot_step_: \n"<<foot_step_<<endl;        

        joy_enable_ = true;
        joy_input_enable_ = true;
        
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

        //for resize
        foot_step_support_frame_mpc_ = foot_step_support_frame_;
        foot_step_support_frame_thread_ = foot_step_support_frame_;
    }
    else if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {
        // cout<<"current_step_num_: "<<current_step_num_<<endl;
        // cout<<"current_support_foot_is_left_: "<<current_support_foot_is_left_<<endl;
        // cout<<"foot_step_: \n"<<foot_step_<<endl;

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

    // if (walking_tick_mj == t_start_)
    // {
    //     if (joy_input_enable_ == true)
    //     {
    //         joystick_input_(0) = (joystick_input(0) + 1) / 2; // FW
    //         joystick_input_(3) = (joystick_input(3) + 1) / 2; // BW
    //         // joystick_input_(1) = joystick_input(1);
    //         joystick_input_(2) = -joystick_input(2);
    //         joystick_input_(1) = joystick_input_(0) + abs(joystick_input_(2)) + joystick_input_(3);
    //     }

    //     if (joystick_input_(1) > 0)
    //     {
    //         calculateFootStepTotal_MJoy();
    //         total_step_num_ = foot_step_.col(1).size();
    //         joy_enable_ = true;
    //         std::cout << "step_length : " << joystick_input_(0) << " trigger(z) : " << joystick_input_(1) << " theta : " << joystick_input_(2) << std::endl;
    //     }
    //     else if (joy_enable_ == true)
    //     {
    //         calculateFootStepTotal_MJoy_End();
    //         total_step_num_ = foot_step_.col(1).size();
    //         joy_enable_ = false;
    //         joy_input_enable_ = false;
    //         joystick_input_(1) = -1.0;
    //     }
    // }
}

void AvatarController::calculateFootStepTotal_MJoy()
{
    double width = 0.1225;
    double length = 0.15;
    double lengthb = 0.1;
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
    // foot_step_support_frame_thread_.setZero(joy_index_ + index, 7);
    // foot_step_support_frame_mpc_.setZero(joy_index_ + index, 7);

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

    // mpc footstep adjustment
    modified_del_zmp_.setZero(joy_index_ + index, 2);
    m_del_zmp_x.setZero(joy_index_ + index, 2);
    m_del_zmp_y.setZero(joy_index_ + index, 2);
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
    // foot_step_support_frame_thread_.setZero(joy_index_ + index, 7);
    // foot_step_support_frame_mpc_.setZero(joy_index_ + index, 7);

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
void AvatarController::getJoystickCommand()
{
    joy_buttons_pre_ = joy_buttons_;
    for(int i = 0; i < 11; i++)
    {
        joy_buttons_(i) = joy_buttons_raw_(i);

        if( joy_buttons_(i) == 1 && joy_buttons_pre_(i) == 0)   // high signal
        {
            joy_buttons_clicked_(i) = 1;
        }
        else
        {
            joy_buttons_clicked_(i) = 0;
        }
    }

    // Button (A)
    if(joy_buttons_clicked_(0))
    {
        joy_continuous_walking_flag_ = !joy_continuous_walking_flag_;

        std_msgs::String msg;
        std::stringstream cont_walk_mode;
        if(joy_continuous_walking_flag_)
        {
            cont_walk_mode << "Continuous Walking Mode is ON";
            cout<< cont_walk_mode.str() <<endl;
        }
        else
        {
            cont_walk_mode << "Continuous Walking Mode is OFF";
            cout<< cont_walk_mode.str() <<endl;
        }
        msg.data = cont_walk_mode.str();
        calibration_state_gui_log_pub.publish(msg);        
    }
    
    if(joy_buttons_clicked_(1))
    {
        std_msgs::String positioncontrol_msg;
        std::stringstream position_control;
        position_control << "positioncontrol";
        positioncontrol_msg.data = position_control.str();
        joystick_tocabi_command_pub.publish(positioncontrol_msg);
    }
}
void AvatarController::updateNextStepTimeJoy()
{
    if (walking_tick_mj >= t_last_)
    {
        if (current_step_num_ != total_step_num_ - 1)
        {
            t_start_ = t_last_ + 1;
            t_start_real_ = t_start_ + t_rest_init_;
            t_last_ = t_start_ + t_total_ - 1;
            // current_step_num_++;
            
            //joystick
            current_support_foot_is_left_ = !current_support_foot_is_left_;

            current_step_num_++;
            if(walking_stop_flag_)
            {
                
            }
            else
            {
                current_step_num_ = DyrosMath::minmax_cut(current_step_num_, 0, 2);
            }

            // cout<<"current_step_num_: "<<current_step_num_<<endl;
            // cout<<"current_support_foot_is_left_: "<<current_support_foot_is_left_<<endl;
            // cout<<"t_start_: "<<t_start_<<endl;
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
        current_support_foot_is_left_ = !current_support_foot_is_left_;
        joy_input_enable_ = true;
        walking_end_flag = 0;
    }
}
void AvatarController::calculateFootStepTotalOmni(double del_x, double del_y, double del_yaw, bool current_support_foot_is_left)
{
    Eigen::Isometry3d left_to_right_step;
    Eigen::Isometry3d right_to_left_step;

    int total_foot_step_planning_num = 6;
    total_step_num_ = total_foot_step_planning_num;
    double nominal_step_width = 0.1225*2;
    bool debugging_mode = false;

    right_to_left_step = oneStepPlanner(del_x, del_y, del_yaw, 1);
    left_to_right_step = oneStepPlanner(del_x, del_y, del_yaw, 0);

    foot_step_.resize(total_foot_step_planning_num , 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(total_foot_step_planning_num , 7);
    foot_step_support_frame_.setZero();
    // foot_step_support_frame_thread_.setZero(total_foot_step_planning_num, 7);
    // foot_step_support_frame_mpc_.setZero(total_foot_step_planning_num, 7);

    // current support foot_step_
    Eigen::Isometry3d next_step_observed_in_first_step;
    int first_support_foot_is_left;
    if(current_step_num_%2 == 0)
    {
        first_support_foot_is_left = current_support_foot_is_left;
    }
    else
    {
        first_support_foot_is_left = !current_support_foot_is_left;
    }
    
    next_step_observed_in_first_step.setIdentity();
    // next_step_observed_in_first_step = oneStepPlanner(del_x, del_y, del_yaw, is_support_foot_left_temp);
    next_step_observed_in_first_step.translation()(1) = -(2*!first_support_foot_is_left - 1)*nominal_step_width/2; 
    
    // foot_step_(0, 0) = next_step_observed_in_first_step.translation()(0);
    // foot_step_(0, 1) = next_step_observed_in_first_step.translation()(1);
    // foot_step_(0, 6) = first_support_foot_is_left;

    bool is_support_foot_left_temp;
    int foot_step_next_planning_idx;

    if(walking_tick_mj < t_start_ + t_total_*0.3)
    {   
        foot_step_next_planning_idx = DyrosMath::minmax_cut(current_step_num_, 0, total_foot_step_planning_num);
        is_support_foot_left_temp = current_support_foot_is_left;
    }
    else
    {
        foot_step_next_planning_idx = DyrosMath::minmax_cut(current_step_num_+1, 0, total_foot_step_planning_num);
        is_support_foot_left_temp = !current_support_foot_is_left;
    }
    


    if(walking_tick_mj == t_start_ && current_step_num_prev_ == current_step_num_)
    {
        for(int i = 1; i <= foot_step_next_planning_idx; i++)
        {   
            foot_step_.row(i-1) = foot_step_joy_temp_.row(i);
        }
    }
    else
    {
        for(int i = 0; i < foot_step_next_planning_idx; i++)
        {   
            foot_step_.row(i) = foot_step_joy_temp_.row(i);
        }
    }
    
    

    if(foot_step_next_planning_idx >=  1)
    {
        for(int i = 0; i < 3; i++)
        {
            next_step_observed_in_first_step.translation()(i) = foot_step_(foot_step_next_planning_idx-1, i);
        }
        next_step_observed_in_first_step.linear() = DyrosMath::rotateWithZ(foot_step_(foot_step_next_planning_idx-1, 5));
    }

    for(int i = foot_step_next_planning_idx; i < total_foot_step_planning_num; i++)
    {
        next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(del_x, del_y, del_yaw, is_support_foot_left_temp);
        foot_step_(i, 0) = next_step_observed_in_first_step.translation()(0);
        foot_step_(i, 1) = next_step_observed_in_first_step.translation()(1);
        
        // int before_planning_step = DyrosMath::(i-1, 0, total_foot_step_planning_num-2);
        foot_step_(i, 5) = DyrosMath::rot2Euler(next_step_observed_in_first_step.linear())(2);
        foot_step_(i, 6) = is_support_foot_left_temp;
        is_support_foot_left_temp = !is_support_foot_left_temp;
    }

    foot_step_joy_temp_.resize(total_foot_step_planning_num, 7);
    foot_step_joy_temp_.setZero();
    foot_step_joy_temp_ = foot_step_;

    // mpc footstep adjustment  
    modified_del_zmp_.setZero(total_foot_step_planning_num, 2);
    m_del_zmp_x.setZero(total_foot_step_planning_num, 2);
    m_del_zmp_y.setZero(total_foot_step_planning_num, 2);

    // 
    if(debugging_mode)
    {   
        if(walking_tick_mj%100 == 0)
        {
            cout<<"-------- COMMAND STEP ----------"<<endl;
            cout<<"walking_tick: "<<walking_tick_mj-t_start_<<endl;
            cout<<"current_support_foot_is_left: " << current_support_foot_is_left << endl;
            cout<<"current_step_num_: " << current_step_num_ << endl;
            cout<<"foot_step_next_planning_idx: " << foot_step_next_planning_idx << endl;
            cout<<"foot_step_: \n" << foot_step_ <<endl;
            cout<<"--------------------------------"<<endl;
        }
    }

}
Eigen::Isometry3d AvatarController::oneStepPlanner(double del_x, double del_y, double del_yaw, bool support_foot_is_left)
{
    Eigen::Isometry3d next_foot_step;
    next_foot_step.setIdentity();
    double nominal_step_width = 0.1225*2;
    double maximum_x_dist_btw_feet = 0.30;
    double minimum_y_dist_btw_feet = 0.20;
    double maximum_y_dist_btw_feet = 0.35;
    double half_foot_size_x = 0.15;

    if(support_foot_is_left)
    {
        // right swing foot planning
        next_foot_step.translation()(0) = del_x;
        next_foot_step.translation()(0) = DyrosMath::minmax_cut(next_foot_step.translation()(0), -maximum_x_dist_btw_feet, maximum_x_dist_btw_feet);
        next_foot_step.translation()(1) = -nominal_step_width + del_y;
        next_foot_step.translation()(1) = DyrosMath::minmax_cut(next_foot_step.translation()(1), -maximum_y_dist_btw_feet, -minimum_y_dist_btw_feet - half_foot_size_x*sin(abs(del_yaw)));
        next_foot_step.translation()(2) = 0;

        next_foot_step.linear() = DyrosMath::rotateWithZ(del_yaw);
    }
    else
    {
        // left swing foot planning
        next_foot_step.translation()(0) = del_x;
        next_foot_step.translation()(0) = DyrosMath::minmax_cut(next_foot_step.translation()(0), -maximum_x_dist_btw_feet, maximum_x_dist_btw_feet);
        next_foot_step.translation()(1) = nominal_step_width + del_y;
        next_foot_step.translation()(1) = DyrosMath::minmax_cut(next_foot_step.translation()(1), minimum_y_dist_btw_feet + half_foot_size_x*sin(abs(del_yaw)), maximum_y_dist_btw_feet);
        next_foot_step.translation()(2) = 0;

        next_foot_step.linear() = DyrosMath::rotateWithZ(del_yaw);
    }

    return next_foot_step;
}
void AvatarController::calculateFootStepTotalOmniEnd(bool support_foot_is_left)
{
    double total_foot_step_planning_num;
    double nominal_step_width = 0.1225*2;
    double one_step_stop_margin_ratio = 0.25;
    bool debugging_mode = false;
    Eigen::Isometry3d next_step_observed_in_first_step;
    next_step_observed_in_first_step.setIdentity();
    next_step_observed_in_first_step.translation()(1) = -(2*support_foot_is_left - 1)*nominal_step_width/2; 

    if(walking_enable_ == true)
    {
        if(current_step_num_ == 0)
        {
            if(walking_tick_mj < t_start_ + t_total_*one_step_stop_margin_ratio)
            { 
                total_foot_step_planning_num = 1;
                // current_step_num_ = 1;
                total_step_num_ = total_foot_step_planning_num;

                foot_step_.resize(total_foot_step_planning_num , 7);
                foot_step_.setZero();

                next_step_observed_in_first_step.translation()(1) = -(2*support_foot_is_left - 1)*nominal_step_width/2; 

                // stop at the first step
                foot_step_(0, 0) = next_step_observed_in_first_step.translation()(0);
                foot_step_(0, 1) = next_step_observed_in_first_step.translation()(1);
                foot_step_(0, 6) = support_foot_is_left;
            }
            else
            {
                total_foot_step_planning_num = 2;
                // current_step_num_ = 1;
                total_step_num_ = total_foot_step_planning_num;

                foot_step_.resize(total_foot_step_planning_num , 7);
                foot_step_.setZero();

                // perform the first command step
                foot_step_.row(0) = foot_step_joy_temp_.row(0);

                for(int i = 0; i < 3; i++)
                {
                    next_step_observed_in_first_step.translation()(i) = foot_step_(0, i);
                }
                next_step_observed_in_first_step.linear() = DyrosMath::rotateWithZ(foot_step_(0, 5));
                
                // stop at the second step 
                next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(0, 0, 0, !support_foot_is_left);
                foot_step_(1, 0) = next_step_observed_in_first_step.translation()(0);
                foot_step_(1, 1) = next_step_observed_in_first_step.translation()(1);
                foot_step_(1, 5) = DyrosMath::rot2Euler(next_step_observed_in_first_step.linear())(2);
                foot_step_(1, 6) = !support_foot_is_left;
            }
        }
        else
        {
            if(walking_tick_mj < t_start_ + t_total_*one_step_stop_margin_ratio)
            {
                total_foot_step_planning_num = current_step_num_ + 1;
                // current_step_num_ = 1;
                total_step_num_ = total_foot_step_planning_num;

                foot_step_.resize(total_foot_step_planning_num , 7);
                foot_step_.setZero();
                // 
                // foot_step_(0, 0) = next_step_observed_in_first_step.translation()(0);
                // foot_step_(0, 1) = next_step_observed_in_first_step.translation()(1);
                // foot_step_(0, 6) = !support_foot_is_left;

                // // 
                // next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(0, 0, 0, support_foot_is_left);
                // foot_step_(1, 0) = next_step_observed_in_first_step.translation()(0);
                // foot_step_(1, 1) = next_step_observed_in_first_step.translation()(1);
                // foot_step_(1, 6) = support_foot_is_left;

                // save the steps until current
                for(int i = 0; i < current_step_num_; i++)
                {
                    foot_step_.row(i) = foot_step_joy_temp_.row(i);
                }

                for(int i = 0; i < 3; i++)
                {
                    next_step_observed_in_first_step.translation()(i) = foot_step_(current_step_num_-1, i);
                }
                next_step_observed_in_first_step.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1, 5));
                // stop at the first step 
                next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(0, 0, 0, support_foot_is_left);
                foot_step_(current_step_num_, 0) = next_step_observed_in_first_step.translation()(0);
                foot_step_(current_step_num_, 1) = next_step_observed_in_first_step.translation()(1);
                foot_step_(current_step_num_, 5) = DyrosMath::rot2Euler(next_step_observed_in_first_step.linear())(2);
                foot_step_(current_step_num_, 6) = support_foot_is_left;
            }
            else
            { 
                total_foot_step_planning_num = current_step_num_ + 2;
                // current_step_num_ = 1;
                total_step_num_ = total_foot_step_planning_num;

                foot_step_.resize(total_foot_step_planning_num , 7);
                foot_step_.setZero();
                // 
                // foot_step_(0, 0) = next_step_observed_in_first_step.translation()(0);
                // foot_step_(0, 1) = next_step_observed_in_first_step.translation()(1);
                // foot_step_(0, 6) = !support_foot_is_left;

                // // 
                // next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(0, 0, 0, support_foot_is_left);
                // foot_step_(1, 0) = next_step_observed_in_first_step.translation()(0);
                // foot_step_(1, 1) = next_step_observed_in_first_step.translation()(1);
                // foot_step_(1, 6) = support_foot_is_left;

                for(int i = 0; i <= current_step_num_; i++)
                {
                    foot_step_.row(i) = foot_step_joy_temp_.row(i);
                }

                for(int i = 0; i < 3; i++)
                {
                    next_step_observed_in_first_step.translation()(i) = foot_step_(current_step_num_, i);
                }
                next_step_observed_in_first_step.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_, 5));
                // 
                next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(0, 0, 0, !support_foot_is_left);
                foot_step_(current_step_num_+1, 0) = next_step_observed_in_first_step.translation()(0);
                foot_step_(current_step_num_+1, 1) = next_step_observed_in_first_step.translation()(1);
                foot_step_(current_step_num_+1, 5) = DyrosMath::rot2Euler(next_step_observed_in_first_step.linear())(2);
                foot_step_(current_step_num_+1, 6) = !support_foot_is_left;
            }
        }
    }
    else
    {
        cout<<"walking disabled foot step generation"<<endl;

        total_foot_step_planning_num = 3;
        total_step_num_ = total_foot_step_planning_num;

        foot_step_.resize(total_foot_step_planning_num , 7);
        foot_step_.setZero();
        
        // 
        foot_step_(0, 0) = next_step_observed_in_first_step.translation()(0);
        foot_step_(0, 1) = next_step_observed_in_first_step.translation()(1);
        foot_step_(0, 6) = support_foot_is_left;

        // 
        next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(0, 0, 0, !support_foot_is_left);
        foot_step_(1, 0) = next_step_observed_in_first_step.translation()(0);
        foot_step_(1, 1) = next_step_observed_in_first_step.translation()(1);
        foot_step_(1, 6) = !support_foot_is_left;

        // 
        next_step_observed_in_first_step = next_step_observed_in_first_step*oneStepPlanner(0, 0, 0, support_foot_is_left);
        foot_step_(2, 0) = next_step_observed_in_first_step.translation()(0);
        foot_step_(2, 1) = next_step_observed_in_first_step.translation()(1);
        foot_step_(2, 6) = support_foot_is_left;
    }
    
    foot_step_joy_temp_.setZero(total_foot_step_planning_num, 7);
    foot_step_joy_temp_ = foot_step_;

    foot_step_support_frame_.setZero(total_foot_step_planning_num , 7);
    // foot_step_support_frame_thread_.setZero(total_foot_step_planning_num, 7);
    // foot_step_support_frame_mpc_.setZero(total_foot_step_planning_num, 7);
    // mpc footstep adjustment  
    modified_del_zmp_.setZero(total_foot_step_planning_num, 2);
    m_del_zmp_x.setZero(total_foot_step_planning_num, 2);
    m_del_zmp_y.setZero(total_foot_step_planning_num, 2);

    if(debugging_mode)
    {
        cout<<"-------- END STEP ----------"<<endl;
        cout<<"walking_tick: "<<walking_tick_mj-t_start_<<endl;
        cout<<"current_support_foot_is_left: " << current_support_foot_is_left_ << endl;
        cout<<"current_step_num_: " << current_step_num_ << endl;
        cout<<"foot_step_: \n" << foot_step_ <<endl;
        cout<<"---------------------------"<<endl;

    }
}

void AvatarController::computePlanner()
{
}

void AvatarController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}