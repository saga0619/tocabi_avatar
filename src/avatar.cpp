#include "avatar.h"
#include <fstream>
using namespace TOCABI;

ofstream MJ_graph("/home/dyros/data/myeongju/MJ_graph.txt");
ofstream MJ_graph1("/home/dyros/data/myeongju/MJ_graph1.txt");
ofstream MJ_graph2("/home/dyros/data/myeongju/MJ_graph2.txt");
ofstream MJ_graph_foottra_x("/home/dyros/data/myeongju/MJ_graph_foottra_x.txt");
ofstream MJ_graph_foottra_y("/home/dyros/data/myeongju/MJ_graph_foottra_y.txt");
ofstream MJ_opto("/home/dyros/data/myeongju/MJ_opto.txt");

// ofstream MJ_graph("/home/myeongju/MJ_graph.txt");
// ofstream MJ_graph1("/home/myeongju/MJ_graph1.txt");
// ofstream MJ_graph2("/home/myeongju/MJ_graph2.txt");
// ofstream MJ_graph_stepping("/home/myeongju/MJ_graph_stepping.txt");
// ofstream MJ_graph_foottra_x("/home/myeongju/MJ_graph_foottra_x.txt");
// ofstream MJ_graph_foottra_y("/home/myeongju/MJ_graph_foottra_y.txt");
// ofstream MJ_q_("/home/myeongju/MJ_q_.txt");
// ofstream MJ_q_dot_("/home/myeongju/MJ_q_dot_.txt");
// ofstream MJ_CAM_("/home/myeongju/MJ_CAM_.txt"); 
// ofstream MJ_CP_ZMP("/home/myeongju/MJ_CP_ZMP.txt");

AvatarController::AvatarController(RobotData &rd) : rd_(rd)
{
    nh_avatar_.setCallbackQueue(&queue_avatar_);
    // sub_1 = nh_avatar_.subscribe("/tocabi/avatar_test", 1, &AvatarController::avatar_callback, this);

    walking_slider_command = nh_avatar_.subscribe("/tocabi/dg/walkingslidercommand", 100, &AvatarController::WalkingSliderCommandCallback, this);

    upperbodymode_sub = nh_avatar_.subscribe("/tocabi/avatar/upperbodymodecommand", 100, &AvatarController::UpperbodyModeCallback, this);
    nextswingleg_sub = nh_avatar_.subscribe("/tocabi/dg/nextswinglegcommand", 100, &AvatarController::NextSwinglegCallback, this);

    com_walking_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/compospdgain", 100, &AvatarController::ComPosGainCallback, this);
    pelv_ori_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/pelvoripdgain", 100, &AvatarController::PelvOriGainCallback, this);
    support_foot_damping_gain_sub = nh_avatar_.subscribe("/tocabi/dg/supportfootdampinggain", 100, &AvatarController::SupportFootDampingGainCallback, this);
    dg_leg_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/legpdgain", 100, &AvatarController::LegJointGainCallback, this);
    alpha_x_sub = nh_avatar_.subscribe("/tocabi/dg/alpha_x", 100, &AvatarController::AlphaXCallback, this);
    alpha_y_sub = nh_avatar_.subscribe("/tocabi/dg/alpha_y", 100, &AvatarController::AlphaYCallback, this);
    step_width_sub = nh_avatar_.subscribe("/tocabi/dg/stepwidthcommand", 100, &AvatarController::StepWidthCommandCallback, this);

    test1_sub = nh_avatar_.subscribe("/tocabi/dg/test1command", 100, &AvatarController::Test1CommandCallback, this);
    test2_sub = nh_avatar_.subscribe("/tocabi/dg/test2command", 100, &AvatarController::Test2CommandCallback, this);

    arm_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/armpdgain", 100, &AvatarController::ArmJointGainCallback, this);
    waist_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/waistpdgain", 100, &AvatarController::WaistJointGainCallback, this);

    hmd_posture_sub = nh_avatar_.subscribe("/HMD", 100, &AvatarController::HmdCallback, this);
    lhand_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER3", 100, &AvatarController::LeftHandTrackerCallback, this);
    rhand_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER5", 100, &AvatarController::RightHandTrackerCallback, this);
    lelbow_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER2", 100, &AvatarController::LeftElbowTrackerCallback, this);
    relbow_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER4", 100, &AvatarController::RightElbowTrackerCallback, this);
    chest_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER1", 100, &AvatarController::ChestTrackerCallback, this);
    pelvis_tracker_posture_sub = nh_avatar_.subscribe("/TRACKER0", 100, &AvatarController::PelvisTrackerCallback, this);
    tracker_status_sub = nh_avatar_.subscribe("/TRACKERSTATUS", 100, &AvatarController::TrackerStatusCallback, this);

    vive_tracker_pose_calibration_sub = nh_avatar_.subscribe("/tocabi/avatar/pose_calibration_flag", 100, &AvatarController::PoseCalibrationCallback, this);

    calibration_state_pub = nh_avatar_.advertise<std_msgs::String>("/tocabi_status", 5);
    calibration_state_gui_log_pub = nh_avatar_.advertise<std_msgs::String>("/tocabi/guilog", 100);

    mujoco_ext_force_apply_pub = nh_avatar_.advertise<std_msgs::Float32MultiArray>("/tocabi_avatar/applied_ext_force", 10);
    mujoco_applied_ext_force_.data.resize(7);

    pedal_command = nh_avatar_.subscribe("/tocabi/pedalcommand", 100, &AvatarController::PedalCommandCallback, this); //MJ

    opto_ftsensor_sub = nh_avatar_.subscribe("/optoforce/ftsensor", 100, &AvatarController::OptoforceFTCallback, this); // real robot experiment

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
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_c_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_C_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_MJ_, true, false);

    for (int i = 0; i < FILE_CNT; i++)
    {
        file[i].open(FILE_NAMES[i]);
    }

    setGains();
    first_loop_larm_ = true;
    first_loop_rarm_ = true;
    first_loop_upperbody_ = true;
    first_loop_hqpik_ = true;
    first_loop_hqpik2_ = true;
    first_loop_qp_retargeting_ = true;
    first_loop_camhqp_ = true;
}

void AvatarController::setGains()
{
    //////////Control Gain///////////////////////////
    ////sim
    // kp_compos_.setZero();
    // kd_compos_.setZero();

    // kp_compos_(0, 0) = 100;
    // kp_compos_(1, 1) = 100;
    // kp_compos_(2, 2) = 100;

    // kd_compos_(0, 0) = 20;
    // kd_compos_(1, 1) = 20;
    // kd_compos_(2, 2) = 20;

    // kp_pelv_ori_.setZero();
    // kd_pelv_ori_.setZero();
    // kp_pelv_ori_ = 1000*Eigen::Matrix3d::Identity(); 	//angle error gain (sim: 4900)(tune)
    // // kp_pelv_ori_(0, 0) = 1000;
    // // kp_pelv_ori_(1, 1) = 1000;
    // // kp_pelv_ori_(2, 2) = 0;

    // kd_pelv_ori_ = 100*Eigen::Matrix3d::Identity();		//angular velocity gain (sim: 140)(tune)

    // support_foot_damping_gain_.setZero();
    // support_foot_damping_gain_(0, 0) = 0.5;
    // support_foot_damping_gain_(1, 1) = 0.5;
    // support_foot_damping_gain_(2, 2) = 0;

    ////real
    kp_compos_.setZero();
    kd_compos_.setZero();

    kp_compos_(0, 0) = 0.2;
    kp_compos_(1, 1) = 0.2;
    kp_compos_(2, 2) = 0.2;

    kd_compos_(0, 0) = 0.00;
    kd_compos_(1, 1) = 0.00;
    kd_compos_(2, 2) = 0.00;

    //////////COM LIMIT/////
    //min
    com_pos_limit_(0) = -0.5;
    com_pos_limit_(1) = -0.5;
    com_pos_limit_(2) = 0.5;
    //max
    com_pos_limit_(3) = 0.5;
    com_pos_limit_(4) = 0.5;
    com_pos_limit_(5) = 0.9;

    //min
    com_vel_limit_(0) = -0.5;
    com_vel_limit_(1) = -0.5;
    com_vel_limit_(2) = -0.2;
    //max
    com_vel_limit_(3) = +0.5;
    com_vel_limit_(4) = +0.5;
    com_vel_limit_(5) = +0.2;

    //min
    com_acc_limit_(0) = -5;
    com_acc_limit_(1) = -5;
    com_acc_limit_(2) = -2;
    //max
    com_acc_limit_(3) = 5;
    com_acc_limit_(4) = 5;
    com_acc_limit_(5) = 2;

    // kp_pelv_ori_ = 1000*Eigen::Matrix3d::Identity(); 	//angle error gain (sim: 4900)(tune)
    // kd_pelv_ori_ = 50*Eigen::Matrix3d::Identity();		//angular velocity gain (sim: 140)(tune)

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
    ///For Simulation
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

    ///For Real Robot
    kp_stiff_joint_(0) = 2000; //right leg
    kp_stiff_joint_(1) = 5000;
    kp_stiff_joint_(2) = 4000;
    kp_stiff_joint_(3) = 3700;
    kp_stiff_joint_(4) = 5000;
    kp_stiff_joint_(5) = 5000;
    kp_stiff_joint_(6) = 2000; //left leg
    kp_stiff_joint_(7) = 5000;
    kp_stiff_joint_(8) = 4000;
    kp_stiff_joint_(9) = 3700;
    kp_stiff_joint_(10) = 5000;
    kp_stiff_joint_(11) = 5000;
    kp_stiff_joint_(12) = 6000; //waist
    kp_stiff_joint_(13) = 10000;
    kp_stiff_joint_(14) = 10000;
    kp_stiff_joint_(15) = 2000;//400; //left arm
    kp_stiff_joint_(16) = 3000;//800;
    kp_stiff_joint_(17) = 2000;//400;
    kp_stiff_joint_(18) = 2000;//400;
    kp_stiff_joint_(19) = 125;
    kp_stiff_joint_(20) = 125;
    kp_stiff_joint_(21) = 25;
    kp_stiff_joint_(22) = 25;
    kp_stiff_joint_(23) = 50; //head
    kp_stiff_joint_(24) = 50;
    kp_stiff_joint_(25) = 2000;//400; //right arm
    kp_stiff_joint_(26) = 3000;//800;
    kp_stiff_joint_(27) = 2000;//400;
    kp_stiff_joint_(28) = 2000;//400;
    kp_stiff_joint_(29) = 125;
    kp_stiff_joint_(30) = 125;
    kp_stiff_joint_(31) = 25;
    kp_stiff_joint_(32) = 25;

    kv_stiff_joint_(0) = 15; //right leg
    kv_stiff_joint_(1) = 50;
    kv_stiff_joint_(2) = 20;
    kv_stiff_joint_(3) = 25;
    kv_stiff_joint_(4) = 30;
    kv_stiff_joint_(5) = 30;
    kv_stiff_joint_(6) = 15; //left leg
    kv_stiff_joint_(7) = 50;
    kv_stiff_joint_(8) = 20;
    kv_stiff_joint_(9) = 25;
    kv_stiff_joint_(10) = 30;
    kv_stiff_joint_(11) = 30;
    kv_stiff_joint_(12) = 200; //waist
    kv_stiff_joint_(13) = 100;
    kv_stiff_joint_(14) = 100;
    kv_stiff_joint_(15) = 20;//7; //left arm
    kv_stiff_joint_(16) = 20;//5;
    kv_stiff_joint_(17) = 20;//2.5;
    kv_stiff_joint_(18) = 20;//2.5;
    kv_stiff_joint_(19) = 2.5;
    kv_stiff_joint_(20) = 2;
    kv_stiff_joint_(21) = 2;
    kv_stiff_joint_(22) = 2;
    kv_stiff_joint_(23) = 2; //head
    kv_stiff_joint_(24) = 2;
    kv_stiff_joint_(25) = 20;//7; //right arm
    kv_stiff_joint_(26) = 20;//5;
    kv_stiff_joint_(27) = 20;//2.5;
    kv_stiff_joint_(28) = 20;//2.5;
    kv_stiff_joint_(29) = 2.5;
    kv_stiff_joint_(30) = 2;
    kv_stiff_joint_(31) = 2;
    kv_stiff_joint_(32) = 2;

    kp_soft_joint_(0) = 2000; //right leg
    kp_soft_joint_(1) = 5000;
    kp_soft_joint_(2) = 4000;
    kp_soft_joint_(3) = 3700;
    kp_soft_joint_(4) = 5000;
    kp_soft_joint_(5) = 5000;
    kp_soft_joint_(6) = 2000; //left leg
    kp_soft_joint_(7) = 5000;
    kp_soft_joint_(8) = 4000;
    kp_soft_joint_(9) = 3700;
    kp_soft_joint_(10) = 5000;
    kp_soft_joint_(11) = 5000;
    kp_soft_joint_(12) = 6000; //waist
    kp_soft_joint_(13) = 10000;
    kp_soft_joint_(14) = 10000;
    kp_soft_joint_(15) = 200; //left arm
    kp_soft_joint_(16) = 80;
    kp_soft_joint_(17) = 60;
    kp_soft_joint_(18) = 60;
    kp_soft_joint_(19) = 60;
    kp_soft_joint_(20) = 60;
    kp_soft_joint_(21) = 20;
    kp_soft_joint_(22) = 20;
    kp_soft_joint_(23) = 50; //head
    kp_soft_joint_(24) = 50;
    kp_soft_joint_(25) = 200; //right arm
    kp_soft_joint_(26) = 80;
    kp_soft_joint_(27) = 60;
    kp_soft_joint_(28) = 60;
    kp_soft_joint_(29) = 60;
    kp_soft_joint_(30) = 60;
    kp_soft_joint_(31) = 20;
    kp_soft_joint_(32) = 20;

    kv_soft_joint_(0) = 15; //right leg
    kv_soft_joint_(1) = 50;
    kv_soft_joint_(2) = 20;
    kv_soft_joint_(3) = 25;
    kv_soft_joint_(4) = 30;
    kv_soft_joint_(5) = 30;
    kv_soft_joint_(6) = 15; //left leg
    kv_soft_joint_(7) = 50;
    kv_soft_joint_(8) = 20;
    kv_soft_joint_(9) = 25;
    kv_soft_joint_(10) = 30;
    kv_soft_joint_(11) = 30;
    kv_soft_joint_(12) = 200; //waist
    kv_soft_joint_(13) = 100;
    kv_soft_joint_(14) = 100;
    kv_soft_joint_(15) = 14; //left arm
    kv_soft_joint_(16) = 10;
    kv_soft_joint_(17) = 5;
    kv_soft_joint_(18) = 5;
    kv_soft_joint_(19) = 2.5;
    kv_soft_joint_(20) = 2;
    kv_soft_joint_(21) = 2;
    kv_soft_joint_(22) = 2;
    kv_soft_joint_(23) = 2; //head
    kv_soft_joint_(24) = 2;
    kv_soft_joint_(25) = 14; //right arm
    kv_soft_joint_(26) = 10;
    kv_soft_joint_(27) = 5;
    kv_soft_joint_(28) = 5;
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

    //arm controller
    joint_limit_l_.resize(33);
    joint_limit_h_.resize(33);
    joint_vel_limit_l_.resize(33);
    joint_vel_limit_h_.resize(33);

    //LEG
    for (int i = 0; i < 12; i++)
    {
        joint_limit_l_(i) = -180 * DEG2RAD;
        joint_limit_h_(i) = 180 * DEG2RAD;
    }

    //WAIST
    joint_limit_l_(12) = -30 * DEG2RAD;
    joint_limit_h_(12) = 30 * DEG2RAD;
    joint_limit_l_(13) = -15 * DEG2RAD;
    joint_limit_h_(13) = 30 * DEG2RAD;
    joint_limit_l_(14) = -15 * DEG2RAD;
    joint_limit_h_(14) = 15 * DEG2RAD;
    //LEFT ARM
    joint_limit_l_(15) = -30 * DEG2RAD;
    joint_limit_h_(15) = 30 * DEG2RAD;
    joint_limit_l_(16) = -160 * DEG2RAD;
    joint_limit_h_(16) = 70 * DEG2RAD;
    joint_limit_l_(17) = -95 * DEG2RAD;
    joint_limit_h_(17) = 95 * DEG2RAD;
    joint_limit_l_(18) = -180 * DEG2RAD;
    joint_limit_h_(18) = 180 * DEG2RAD;
    joint_limit_l_(19) = -150 * DEG2RAD;
    joint_limit_h_(19) = -10 * DEG2RAD;
    joint_limit_l_(20) = -180 * DEG2RAD;
    joint_limit_h_(20) = 180 * DEG2RAD;
    joint_limit_l_(21) = -70 * DEG2RAD;
    joint_limit_h_(21) = 70 * DEG2RAD;
    joint_limit_l_(22) = -60 * DEG2RAD;
    joint_limit_h_(22) = 60 * DEG2RAD;
    //HEAD
    joint_limit_l_(23) = -80 * DEG2RAD;
    joint_limit_h_(23) = 80 * DEG2RAD;
    joint_limit_l_(24) = -40 * DEG2RAD;
    joint_limit_h_(24) = 30 * DEG2RAD;
    //RIGHT ARM
    joint_limit_l_(25) = -30 * DEG2RAD;
    joint_limit_h_(25) = 30 * DEG2RAD;
    joint_limit_l_(26) = -70 * DEG2RAD;
    joint_limit_h_(26) = 160 * DEG2RAD;
    joint_limit_l_(27) = -95 * DEG2RAD;
    joint_limit_h_(27) = 95 * DEG2RAD;
    joint_limit_l_(28) = -180 * DEG2RAD;
    joint_limit_h_(28) = 180 * DEG2RAD;
    joint_limit_l_(29) = 10 * DEG2RAD;
    joint_limit_h_(29) = 150 * DEG2RAD;
    joint_limit_l_(30) = -180 * DEG2RAD;
    joint_limit_h_(30) = 180 * DEG2RAD;
    joint_limit_l_(31) = -70 * DEG2RAD;
    joint_limit_h_(31) = 70 * DEG2RAD;
    joint_limit_l_(32) = -60 * DEG2RAD;
    joint_limit_h_(32) = 60 * DEG2RAD;

    //LEG
    for (int i = 0; i < 12; i++)
    {
        joint_vel_limit_l_(i) = -2 * M_PI;
        joint_vel_limit_h_(i) = 2 * M_PI;
    }

    //UPPERBODY
    for (int i = 12; i < 33; i++)
    {
        joint_vel_limit_l_(i) = -M_PI * 1.5; // *2
        joint_vel_limit_h_(i) = M_PI * 1.5; // *2
    }

    // joint_vel_limit_l_(13) = -M_PI * 3;
    // joint_vel_limit_h_(13) = M_PI * 3;
    // joint_vel_limit_l_(14) = -M_PI * 3;
    // joint_vel_limit_h_(14) = M_PI * 3;

    //1st arm joint vel limit
    joint_vel_limit_l_(15) = -M_PI / 4;
    joint_vel_limit_h_(15) = M_PI / 4;

    joint_vel_limit_l_(25) = -M_PI / 4;
    joint_vel_limit_h_(25) = M_PI / 4;

    // Head joint vel limit
    joint_vel_limit_l_(23) = -2 * M_PI;
    joint_vel_limit_h_(23) = 2 * M_PI;
    joint_vel_limit_l_(24) = -2 * M_PI;
    joint_vel_limit_h_(24) = 2 * M_PI;

    // forearm joint vel limit
    joint_vel_limit_l_(20) = -2.0 * M_PI; // 2 * // IROS22 -> 2 * M_PI
    joint_vel_limit_h_(20) = 2.0 * M_PI; // 2 *
    joint_vel_limit_l_(30) = -2.0 * M_PI; // 2 *
    joint_vel_limit_h_(30) = 2.0 * M_PI; // 2 *
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
            
            CAM_upper_init_q_(15) = + 15.0 * DEG2RAD; // Left Shoulder Yaw joint // 17 deg
            CAM_upper_init_q_(16) = + 10.0 * DEG2RAD; // Left Shoulder Pitch joint // 17 deg
            CAM_upper_init_q_(17) = + 65.0 * DEG2RAD; // Left Shoulder Roll joint // 86 deg
            CAM_upper_init_q_(18) = - 70.0 * DEG2RAD; // Left Elbow Yaw joint // -72 deg
            // CAM_upper_init_q_(19) = - 65.0 * DEG2RAD; // Left Elbow Pitch joint // -57 deg

            CAM_upper_init_q_(25) = - 15.0 * DEG2RAD; // Right Shoulder Yaw joint // -17 deg
            CAM_upper_init_q_(26) = - 10.0 * DEG2RAD; // Right Shoulder Pitch joint           
            CAM_upper_init_q_(27) = - 65.0 * DEG2RAD; // Right Shoulder Roll joint 
            CAM_upper_init_q_(28) = + 70.0 * DEG2RAD; // Right Elbow Yaw joint
            // CAM_upper_init_q_(29) = + 65.0 * DEG2RAD; // Right Elbow Pich joint                       
            
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            parameterSetting();
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
        if(initial_tick_mj <= 2.0 * hz_)
        {
            ref_q_(15) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(15), CAM_upper_init_q_(15), 0.0, 0.0); // Left Shoulder Yaw joint
            ref_q_(16) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(16), CAM_upper_init_q_(16), 0.0, 0.0); // Left Shoulder Pitch joint
            ref_q_(17) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(17), CAM_upper_init_q_(17), 0.0, 0.0); // Left Shoulder Roll joint
            ref_q_(18) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(18), CAM_upper_init_q_(18), 0.0, 0.0); // Left Elbow Yaw joint
            // ref_q_(19) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(19), CAM_upper_init_q_(19), 0.0, 0.0);  // Left Elbow Pitch joint
            
            ref_q_(25) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(25), CAM_upper_init_q_(25), 0.0, 0.0); // Right Shoulder Yaw joint
            ref_q_(26) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(26), CAM_upper_init_q_(26), 0.0, 0.0); // Right Shoulder Pitch joint  
            ref_q_(27) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(27), CAM_upper_init_q_(27), 0.0, 0.0); // Right Shoulder Roll joint 
            ref_q_(28) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(28), CAM_upper_init_q_(28), 0.0, 0.0); // Right Elbow Yaw joint
            // ref_q_(29) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(29), CAM_upper_init_q_(29), 0.0, 0.0); // Right Elbow Pich joint  
                        
            initial_tick_mj ++;         
        }
        
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
                // CPMPC_bolt_Controller_MJ_ICRA();
                // BoltController_MJ(); // Stepping Controller for DCM eos                
                
                CentroidalMomentCalculator_new(); // working with computefast() (CAM controller)

                // getFootTrajectory(); 
                getFootTrajectory_stepping(); // working with CPMPC_bolt_Controller_MJ()  
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
                Compliant_control(q_des_);
                for (int i = 0; i < 12; i++)
                {
                    // ref_q_(i) = q_des_(i);
                    ref_q_(i) = DOB_IK_output_(i);
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
                del_zmp(0) = 1.4 * (cp_measured_(0) - cp_desired_(0));
                del_zmp(1) = 1.3 * (cp_measured_(1) - cp_desired_(1));
                CP_compen_MJ();
                CP_compen_MJ_FT();
                torque_lower_.setZero();
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + Tau_CP(i) + Gravity_MJ_fast_(i);
                    // 4 (Ankle_pitch_L), 5 (Ankle_roll_L), 10 (Ankle_pitch_R),11 (Ankle_roll_R)
                }


                desired_q_not_compensated_ = ref_q_;
                updateNextStepTime();
                q_prev_MJ_ = rd_.q_;
                                
                if(current_step_num_ == 4 && (walking_tick_mj >= t_start_ + 0.15*hz_ + 0.6*0.3*hz_)  && (walking_tick_mj < t_start_ + 0.15*hz_ + 0.6*0.3*hz_ + 0.2*hz_))
                {    
                    mujoco_applied_ext_force_.data[0] = force_temp_*sin(theta_temp_*DEG2RAD); //x-axis linear force
                    mujoco_applied_ext_force_.data[1] = -force_temp_*cos(theta_temp_*DEG2RAD); //y-axis linear force  
                    mujoco_applied_ext_force_.data[2] = 0.0; //z-axis linear force
                    mujoco_applied_ext_force_.data[3] = 0.0; //x-axis angular moment
                    mujoco_applied_ext_force_.data[4] = 0.0; //y-axis angular moment
                    mujoco_applied_ext_force_.data[5] = 0.0; //z-axis angular moment

                    mujoco_applied_ext_force_.data[6] = 1; //link idx; 1:pelvis

                    mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);                    
                } 
                else
                {
                    mujoco_applied_ext_force_.data[0] = 0; //x-axis linear force
                    mujoco_applied_ext_force_.data[1] = 0; //y-axis linear force
                    mujoco_applied_ext_force_.data[2] = 0; //z-axis linear force
                    mujoco_applied_ext_force_.data[3] = 0; //x-axis angular moment
                    mujoco_applied_ext_force_.data[4] = 0; //y-axis angular moment
                    mujoco_applied_ext_force_.data[5] = 0; //z-axis angular moment

                    mujoco_applied_ext_force_.data[6] = 1; //link idx; 1:pelvis

                    mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);
                }
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
                torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + Gravity_MJ_fast_(i);
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
            torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - rd_.q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - rd_.q_dot_(i)) + Gravity_MJ_fast_(i));
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
            ref_q_ = rd_.q_;

            for (int i = 0; i < 12; i++)
            {
                Initial_ref_q_(i) = ref_q_(i);
            }
            init_leg_time_ = rd_.control_time_;
            desired_q_fast_ = rd_.q_;
            desired_q_dot_fast_.setZero();

            initial_flag = 1;
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            joy_input_enable_ = true;

            chair_mode_ = false; ///avatar semifinals //1025

            parameterSetting();
            cout << "mode = 12 : Pedal Init" << endl;
            cout << "chair_mode_: " << chair_mode_ << endl;
            WBC::SetContact(rd_, 1, 1);
            Gravity_MJ_ = WBC::GravityCompensationTorque(rd_);
            atb_grav_update_ = false;
        }

        if (atb_grav_update_ == false)
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
    }
    else if (rd_.tc_.mode == 13)
    {
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

                pelv_trajectory_support_init_ = pelv_trajectory_support_;
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

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            updateInitialStateJoy();
            getRobotState();
            floatToSupportFootstep();
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            if (current_step_num_ < total_step_num_)
            {
                std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
                getZmpTrajectory();
                std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
                getComTrajectory();
                std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
                getFootTrajectory();
                std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
                getPelvTrajectory();
                std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
                supportToFloatPattern();
                std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
                computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);
                std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();

                Compliant_control(q_des_);
                for (int i = 0; i < 12; i++)
                {
                    //ref_q_(i) = q_des_(i);
                    ref_q_(i) = DOB_IK_output_(i);
                }
                hip_compensator();

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

                desired_q_not_compensated_ = ref_q_;

                updateNextStepTimeJoy();

                q_prev_MJ_ = rd_.q_;

                // if (int(walking_tick_mj) % 50 == 0)
                // {
                //     cout<<"get state time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
                //     cout<<"getZmpTrajectory time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() <<endl;
                //     cout<<"getComTrajectory time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() <<endl;
                //     cout<<"getFootTrajectory time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() <<endl;
                //     cout<<"getPelvTrajectory time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t7 - t6).count() <<endl;
                //     cout<<"supportToFloatPattern time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t8 - t7).count() <<endl;
                //     cout<<"computeIkControl_MJ time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t9 - t8).count() <<endl;
                //     cout<<"total time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t9 - t1).count() <<endl;
                // }
            }
        }
        else
        {
            // double init_time_;
            if (walking_end_flag == 0)
            {
                cout << "com_desired_1: " << com_desired_ << endl;
                parameterSetting(); //Don't delete this!!
                updateInitialStateJoy();
                //updateInitialState();
                getRobotState();
                floatToSupportFootstep();
                getZmpTrajectory();
                getComTrajectory();
                getFootTrajectory();
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
                init_leg_time_ = rd_.control_time_;
                walking_end_flag = 1;
                cout << "com_desired_3: " << com_desired_ << endl;
            }

            getRobotState();
            getPelvTrajectory();
            supportToFloatPattern();
            computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);

            Compliant_control(q_des_);
            for (int i = 0; i < 12; i++)
            {
                //ref_q_(i) = q_des_(i);
                ref_q_(i) = DOB_IK_output_(i);
            }

            hip_compensator();

            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_MJ_fast_ = Gravity_MJ_;
                atb_grav_update_ = false;
            }

            if (rd_.control_time_ <= init_leg_time_ + 2.0)
            {
                for (int i = 0; i < 12; i++)
                {
                    ref_q_(i) = DyrosMath::cubic(rd_.control_time_, init_leg_time_, init_leg_time_ + 2.0, Initial_ref_q_(i), q_des_(i), 0.0, 0.0);
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
            torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - current_q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            torque_upper_(i) = torque_upper_(i) * pd_control_mask_(i); // masking for joint pd control
        }

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

            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
                //cout<<"comutefast tc.mode =10 is initialized"<<endl;
            }
            //initial_flag = 2;
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
        walkingStateManager(); //avatar
        getProcessedRobotData();

        //motion planing and control//
        motionGenerator();
        //STEP3: Compute q_dot for CAM control
         
        computeCAMcontrol_HQP();
         
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
            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }
        }
    }
    else if (rd_.tc_.mode == 13)
    {
        std::chrono::steady_clock::time_point tt1 = std::chrono::steady_clock::now();

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
        std::chrono::steady_clock::time_point tt2 = std::chrono::steady_clock::now();
        /////////////////////////////////////////////////////////////////////////////////////////

        if (rd_.tc_init == true)
        {
            initWalkingParameter();
            rd_.tc_init = false;
        }

        //data process//
        getRobotData(); // 47~64us
        std::chrono::steady_clock::time_point tt3 = std::chrono::steady_clock::now();
        walkingStateManager(); //avatar // <<1us
        std::chrono::steady_clock::time_point tt4 = std::chrono::steady_clock::now();
        getProcessedRobotData(); // <<1us
        std::chrono::steady_clock::time_point tt5 = std::chrono::steady_clock::now();
        //motion planing and control//

        if (current_q_(24) > 5 * DEG2RAD)
        {
            if (abs(current_q_(23)) > 18 * DEG2RAD)
            {
                joint_limit_h_(24) = 10 * DEG2RAD;
                joint_limit_h_(23) = 80 * DEG2RAD;
                joint_limit_l_(23) = -80 * DEG2RAD;
            }
            else
            {
                joint_limit_h_(24) = 30 * DEG2RAD;
                joint_limit_h_(23) = 13 * DEG2RAD;
                joint_limit_l_(23) = -13 * DEG2RAD;
            }
        }
        else
        {
            joint_limit_h_(24) = 10 * DEG2RAD;
            joint_limit_h_(23) = 80 * DEG2RAD;
            joint_limit_l_(23) = -80 * DEG2RAD;
        }

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
        std::chrono::steady_clock::time_point tt6 = std::chrono::steady_clock::now();
        for (int i = 12; i < MODEL_DOF; i++)
        {
            desired_q_(i) = motion_q_(i);
            desired_q_dot_(i) = motion_q_dot_(i);
            // desired_q_dot_(i) = 0;
        }
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_slow_ = desired_q_;
            desired_q_dot_slow_ = desired_q_dot_;
            atb_desired_q_update_ = false;
        }
        
        savePreData();
        if (int(rd_.control_time_ * 10000) % 10000 == 0)
        {
            // cout<<"gravity compensation torque time: "<< std::chrono::duration_cast<std::chrono::microseconds>(tt2 - tt1).count() <<endl;
            // cout<<"getRobotData time: "<< std::chrono::duration_cast<std::chrono::microseconds>(tt3 - tt2).count() <<endl;
            // cout<<"walkingStateManager time: "<< std::chrono::duration_cast<std::chrono::microseconds>(tt4 - tt3).count() <<endl;
            // cout<<"getProcessedRobotData time: "<< std::chrono::duration_cast<std::chrono::microseconds>(tt5 - tt4).count() <<endl;
            // cout<<"motionGenerator time: "<< std::chrono::duration_cast<std::chrono::microseconds>(tt6 - tt5).count() <<endl;
        }
        // printOutTextFile();
    }
    else if (rd_.tc_.mode == 14)
    {
        ////////////////////////////////////////////////////////////////////////////
        /////////////////// AVATAR Controller ////////////////////
        ////////////////////////////////////////////////////////////////////////////

        torque_task_.setZero();
        torque_init_.setZero();
        if (rd_.tc_init == true)
        {
            initWalkingParameter();
            rd_.tc_init = false;
        }

        //data process//
        getRobotData();
        walkingStateManager(); //avatar
        getProcessedRobotData();

        foot_swing_trigger_ = false; //stay avatar

        //motion planing and control//
        motionGenerator();

        // getComTrajectory_Preview(); 
        // getSwingFootXYTrajectory(walking_phase_, com_pos_current_, com_vel_current_, com_vel_desired_);

        if ((current_time_) >= program_start_time_ + program_ready_duration_)
        {
            torque_task_ += comVelocityControlCompute(); //support chain control for COM velocity and pelvis orientation
            // if( int(current_time_*10000)%1000 == 0 )
            // cout<<"torque_task_(12) 1st: " << torque_task_(13) << endl;
            torque_task_ += swingFootControlCompute(); //swing foot control
            // if( int(current_time_*10000)%1000 == 0 )
            // cout<<"torque_task_(12) 2nd: " << torque_task_(13) << endl;
            torque_task_ += jointTrajectoryPDControlCompute(); //upper body motion + joint damping control
                                                               // if( int(current_time_*10000)%1000 == 0 )
                                                               // cout<<"torque_task_(12) 3rd: " << torque_task_(13) << endl;
                                                               // torque_task_ += dampingControlCompute(wbc_);
                                                               // if( int(current_time_*10000)%1000 == 0 )
                                                               // 	cout<<"torque_task_(12) 4th: " << torque_task_(13) << endl;
                                                               // torque_task_ += jointLimit();
                                                               // if( int(current_time_*10000)%1000 == 0 )
                                                               // 	cout<<"torque_task_(12) 5th: " << torque_task_(13) << endl;
        }
        torque_task_.segment(0, 12).setZero();
        torque_task_ += ikBalanceControlCompute();

        // //CoM pos & pelv orientation control
        // wbc_.SetContact(rd_, 1, 1);

        // int task_number = 6;	//CoM is not controlled in z direction
        // rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
        // rd_.f_star.setZero(task_number);

        // rd_.J_task = rd_.link_[COM_id].jac;
        // //rd_.J_task.block(2, 0, 1, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].jac.block(2, 0, 1, MODEL_DOF_VIRTUAL);

        // // rd_.J_task.block(2, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);

        // rd_.link_[COM_id].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
        // rd_.link_[COM_id].x_desired(2) = tc.height;
        // rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + 3);

        // rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
        // // rd_.link_[COM_id].rot_desired = pelv_yaw_rot_current_from_global_mj_;
        // rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + 3, false);

        // rd_.f_star = wbc_.getfstar6d(rd_, COM_id);
        // // rd_.f_star.segment(0, 2) = wbc_.getfstar6d(rd_, COM_id).segment(0, 2);
        // // rd_.f_star.segment(2, 3) = wbc_.getfstar6d(rd_, COM_id).segment(3, 3);
        // //tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));
        // //torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
        // Eigen::VectorQd torque_com_control;
        // torque_com_control = wbc_.task_control_torque_with_gravity(rd_, rd_.J_task, rd_.f_star);
        // rd_.contact_redistribution_mode = 0;

        // torque_task_.segment(0, 12) = torque_com_control.segment(0, 12);
        // torque_task_(3) = (kp_joint_(3) * (desired_q_(3) - current_q_(3)) + kv_joint_(3) * (desired_q_dot_(3) - current_q_dot_(3)));
        // torque_task_(9) = (kp_joint_(9) * (desired_q_(9) - current_q_(9)) + kv_joint_(9) * (desired_q_dot_(9) - current_q_dot_(9)));
        ////////////////////////////////////////

        savePreData();

        ////////////////////////////////TORQUE LIMIT//////// //////////////////////
        for (int i = 0; i < MODEL_DOF; i++)
        {
            torque_task_(i) = DyrosMath::minmax_cut(torque_task_(i), torque_task_min_(i), torque_task_max_(i));
        }
        ///////////////////////////////////////////////////////////////////////////

        //////////////////////////////FALLING//////////////////////////////
        // fallDetection();
        ///////////////////////////////////////////////////////////////////

        rd_.torque_desired = torque_task_;
        if (int(current_time_ * 10000) % 1000 == 0)
        {
            // 	cout<<"torque_task_(12): "<<torque_task_(12)<<endl;
            // 	cout<<"torque_task_(13): "<<torque_task_(13)<<endl;
            // 	cout<<"torque_task_(14): "<<torque_task_(14)<<endl;
            // 	cout<<"desired_q_(12): "<<desired_q_(12)<<endl;
            // 	cout<<"desired_q_(13): "<<desired_q_(13)<<endl;
            // 	cout<<"desired_q_(14): "<<desired_q_(14)<<endl;

            // rd_.torque_desired .setZero();
            // rd_.torque_desired (23) = torque_task_(23);
            // rd_.torque_desired (24) = torque_task_(24);
            // Vector3d temp_sh = pelv_yaw_rot_current_from_global_mj_.transpose() * (rd_.link_[Left_Hand-5].xpos - pelv_pos_current_);
            // Vector3d temp_elbow = pelv_yaw_rot_current_from_global_mj_.transpose() * (rd_.link_[Left_Hand-3].xpos - pelv_pos_current_);
            // Vector3d temp_hand = pelv_yaw_rot_current_from_global_mj_.transpose() * (rd_.link_[Left_Hand-1].xpos - pelv_pos_current_);
        }
        // printOutTextFile();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AvatarController::initWalkingParameter()
{
    walking_mode_on_ = true;
    program_ready_duration_ = 0;
    walking_control_transition_duration_ = 0.1;
    upper_body_mode_ = 3;
    stop_vel_threshold_ = 0.20;
    walking_duration_cmd_ = 1.3;
    dsp_duration_ = 0.6;
    dsp_ratio_ = dsp_duration_ / walking_duration_cmd_;
    turning_duration_ = (walking_duration_cmd_ - dsp_duration_) * 0.8;
    walking_phase_ = 0;
    turning_phase_ = 0;
    walking_speed_ = 0.00;
    // walking_speed_ = 0.05/1.3; //5cm walking speed
    // walking_speed_ = 0.10/1.3; //5cm walking speed
    walking_speed_side_ = 0.0;
    // knee_target_angle_ = 18*DEG2RAD;
    knee_target_angle_ = 0.6; //4.5degree
    com_target_height_ = 0.71;

    swingfoot_highest_time_ = (1 - dsp_ratio_) / 2 + dsp_ratio_;
    ankle2footcenter_offset_ = 0.02;
    yaw_angular_vel_ = 0; //   rad/s
    swing_foot_height_ = 0.05;
    switching_phase_duration_ = 0.05;
    foot_contact_ = -1;
    foot_contact_pre_ = foot_contact_;
    step_width_ = 0.22; //for preview control
    alpha_x_ = 0.01;
    alpha_y_ = 0.18;
    alpha_x_command_ = alpha_x_;
    alpha_y_command_ = alpha_y_;

    start_walking_trigger_ = false;
    first_step_trigger_ = false;
    foot_swing_trigger_ = false;
    stop_walking_trigger_ = true;
    falling_detection_flag_ = false;

    upperbody_mode_recieved_ = true;

    preview_horizon_ = 1.6; //seconds
    preview_hz_ = 2000;
    zmp_size_ = preview_horizon_ * preview_hz_;
    ref_zmp_.setZero(zmp_size_, 2);
    zmp_y_offset_ = -0.04; //outward from com

    walking_duration_start_delay_ = preview_horizon_;
    max_stop_walking_num_ = int(preview_horizon_ / walking_duration_cmd_) + 1;
    stop_walking_counter_ = 0;

    jac_rhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_rfoot_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lfoot_.setZero(6, MODEL_DOF_VIRTUAL);

    com_pos_error_.setZero();
    com_vel_error_.setZero();
    //set init pre data
    com_pos_desired_pre_ = rd_.link_[COM_id].xpos;
    com_vel_desired_pre_.setZero();
    com_acc_desired_pre_.setZero();

    com_vel_cutoff_freq_ = 1;

    pre_time_ = rd_.control_time_ - 0.001;
    pre_desired_q_ = rd_.q_;
    last_desired_q_ = rd_.q_;
    pre_desired_q_dot_.setZero();

    init_q_ = rd_.q_;
    zero_q_ = init_q_;
    desired_q_ = init_q_;
    desired_q_dot_.setZero();
    desired_q_ddot_.setZero();
    torque_task_.setZero();
    torque_task_pre_.setZero();

    A_mat_pre_ = rd_.A_;

    mob_integral_.setZero();
    mob_residual_.setZero();

    motion_q_pre_ = init_q_;
    motion_q_dot_pre_.setZero();

    contact_force_lfoot_.setZero();
    contact_force_rfoot_.setZero();
    contact_force_lfoot_local_.setZero();
    contact_force_rfoot_local_.setZero();

    zmp_local_lfoot_.setZero();
    zmp_local_rfoot_.setZero();
    zmp_measured_.setZero();
    zmp_dot_measured_.setZero();

    f_star_l_.setZero();
    f_star_r_.setZero();
    f_star_l_pre_.setZero();
    f_star_r_pre_.setZero();

    swingfoot_f_star_l_.setZero();
    swingfoot_f_star_r_.setZero();
    swingfoot_f_star_l_pre_.setZero();
    swingfoot_f_star_r_pre_.setZero();

    f_lfoot_damping_.setZero();
    f_rfoot_damping_.setZero();
    f_lfoot_damping_pre_.setZero();
    f_rfoot_damping_pre_.setZero();

    foot_lift_count_ = 0;
    foot_landing_count_ = 0;

    lhand_control_point_offset_.setZero();
    rhand_control_point_offset_.setZero();
    lhand_control_point_offset_(2) = -0.13;
    rhand_control_point_offset_(2) = -0.13;

    robot_shoulder_width_ = 0.6;

    robot_upperarm_max_l_ = 0.3376 * 1.0;
    robot_lowerarm_max_l_ = 0.31967530867;
    // robot_arm_max_l_ = 0.98*sqrt(robot_upperarm_max_l_*robot_upperarm_max_l_ + robot_lowerarm_max_l_*robot_lowerarm_max_l_ + 2*robot_upperarm_max_l_*robot_lowerarm_max_l_*cos( -joint_limit_h_(19)) );
    robot_arm_max_l_ = (robot_upperarm_max_l_ + robot_lowerarm_max_l_) * 0.999 + lhand_control_point_offset_.norm();

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

    hmd_pelv_pose_raw_last_.setIdentity();
    hmd_lshoulder_pose_raw_last_.setIdentity();
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
    // hmd_tracker_status_raw_ = true;
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
    current_time_ = rd_.control_time_;

    if (current_time_ != pre_time_)
    {
        dt_ = current_time_ - pre_time_;
    }

    current_q_ = rd_.q_;
    current_q_dot_ = rd_.q_dot_;
    current_q_ddot_ = rd_.q_ddot_virtual_.segment(6, MODEL_DOF);
    pelv_pos_current_ = rd_.link_[Pelvis].xpos;
    pelv_vel_current_.segment(0, 3) = rd_.link_[Pelvis].v;
    pelv_vel_current_.segment(3, 3) = rd_.link_[Pelvis].w;

    pelv_rot_current_ = rd_.link_[Pelvis].rotm;
    pelv_rpy_current_ = DyrosMath::rot2Euler(pelv_rot_current_); //ZYX multiply
    // pelv_rpy_current_ = (pelv_rot_current_).eulerAngles(2, 1, 0);
    // pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
    pelv_yaw_rot_current_from_global_ = pelv_rot_current_;
    pelv_rot_current_yaw_aline_ = pelv_yaw_rot_current_from_global_.transpose() * pelv_rot_current_;
    // pelv_pos_current_ = pelv_yaw_rot_current_from_global_.transpose() * pelv_pos_current_;

    pelv_transform_current_from_global_.translation().setZero();
    pelv_transform_current_from_global_.linear() = pelv_rot_current_yaw_aline_;

    pelv_angvel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Pelvis].w;

    com_pos_current_ = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[COM_id].xpos - pelv_pos_current_);
    // com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].v;
    com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].v;
    com_mass_ = rd_.link_[COM_id].mass;
    // com_acc_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].a;
    // com_acc_current_ = ;

    /////////////////////////Feet Transformation and Velocity/////////////////////
    lfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Foot].xpos - pelv_pos_current_);
    lfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].rotm;
    rfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Foot].xpos - pelv_pos_current_);
    rfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].rotm;

    lfoot_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].v;
    lfoot_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].w;
    rfoot_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].v;
    rfoot_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].w;
    ///////////////////////////////////////////////////////////////////////////////

    ////////////////////////Knee Trnasformation and Velocity///////////////////////
    lknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Foot - 2].xpos - pelv_pos_current_);
    lknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot - 2].rotm;
    rknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Foot - 2].xpos - pelv_pos_current_);
    rknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot - 2].rotm;
    ///////////////////////////////////////////////////////////////////////////////

    ////////////////////////Hand Trnasformation and Velocity///////////////////////
    lhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand].xpos - pelv_pos_current_);
    lhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].rotm;
    rhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand].xpos - pelv_pos_current_);
    rhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].rotm;

    lhand_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].v;
    lhand_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].w;
    rhand_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].v;
    rhand_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].w;
    ///////////////////////////////////////////////////////////////////////////////

    ////////////////////////Elbow Trnasformation and Velocity///////////////////////
    lelbow_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand - 3].xpos - pelv_pos_current_);
    lelbow_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 3].rotm;
    relbow_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand - 3].xpos - pelv_pos_current_);
    relbow_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 3].rotm;

    lelbow_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 3].v;
    lelbow_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 3].w;
    relbow_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 3].v;
    relbow_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 3].w;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Upper Arm Trnasformation and Velocity////////////////////
    lupperarm_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand - 4].xpos - pelv_pos_current_);
    lupperarm_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 4].rotm;
    rupperarm_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand - 4].xpos - pelv_pos_current_);
    rupperarm_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 4].rotm;

    lupperarm_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 4].v;
    lupperarm_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 4].w;
    rupperarm_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 4].v;
    rupperarm_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 4].w;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Shoulder Trnasformation and Velocity////////////////////
    lshoulder_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand - 5].xpos - pelv_pos_current_);
    lshoulder_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 5].rotm;
    rshoulder_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand - 5].xpos - pelv_pos_current_);
    rshoulder_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 5].rotm;

    lshoulder_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 5].v;
    lshoulder_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 5].w;
    rshoulder_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 5].v;
    rshoulder_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 5].w;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Acromion Trnasformation and Velocity////////////////////
    lacromion_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand - 6].xpos - pelv_pos_current_);
    lacromion_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 6].rotm;
    racromion_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand - 6].xpos - pelv_pos_current_);
    racromion_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 6].rotm;

    lacromion_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 6].v;
    lacromion_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 6].w;
    racromion_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 6].v;
    racromion_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 6].w;
    ////////////////////////////////////////////////////////////////////////////////

    ///////////////////////Armbase Trasformation and ///////////////////////////////
    larmbase_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand - 7].xpos - pelv_pos_current_);
    larmbase_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand - 7].rotm;
    rarmbase_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand - 7].xpos - pelv_pos_current_);
    rarmbase_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand - 7].rotm;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////Head & Upperbody Trnasformation ////////////////////////
    head_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Head].xpos - pelv_pos_current_);
    head_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Head].rotm;

    upperbody_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Upper_Body].xpos - pelv_pos_current_);
    upperbody_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Upper_Body].rotm;
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

    lfoot_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Foot].id, Eigen::Vector3d::Zero(), false);
    lfoot_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Foot].id, false)).transpose();

    lhand_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, lhand_control_point_offset_, false);
    lhand_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, false)).transpose();

    lelbow_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 3].id, Eigen::Vector3d::Zero(), false);
    lelbow_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 3].id, false).transpose();

    lupperarm_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 4].id, Eigen::Vector3d::Zero(), false);
    lupperarm_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 4].id, false).transpose();

    lshoulder_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 5].id, Eigen::Vector3d::Zero(), false);
    lshoulder_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 5].id, false).transpose();

    lacromion_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, Eigen::Vector3d::Zero(), false);
    lacromion_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, false).transpose();

    larmbase_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 7].id, Eigen::Vector3d::Zero(), false);
    larmbase_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 7].id, false).transpose();

    rhand_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, rhand_control_point_offset_, false);
    rhand_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, false)).transpose();

    relbow_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 3].id, Eigen::Vector3d::Zero(), false);
    relbow_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 3].id, false).transpose();

    rupperarm_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 4].id, Eigen::Vector3d::Zero(), false);
    rupperarm_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 4].id, false).transpose();

    rshoulder_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 5].id, Eigen::Vector3d::Zero(), false);
    rshoulder_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 5].id, false).transpose();

    racromion_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, Eigen::Vector3d::Zero(), false);
    racromion_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, false).transpose();

    rarmbase_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 7].id, Eigen::Vector3d::Zero(), false);
    rarmbase_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 7].id, false).transpose();

    head_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Head].id, Eigen::Vector3d::Zero(), false);
    head_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Head].id, false).transpose();

    upperbody_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Upper_Body].id, Eigen::Vector3d::Zero(), false);
    upperbody_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Upper_Body].id, false).transpose();

    RigidBodyDynamics::Math::Vector3d com_pos_temp, com_vel_temp, com_accel_temp, com_ang_momentum_temp, com_ang_moment_temp;

    RigidBodyDynamics::Utils::CalcCenterOfMass(model_d_, q_virtual, q_dot_virtual, &q_ddot_virtual, com_mass_, com_pos_temp, &com_vel_temp, &com_accel_temp, &com_ang_momentum_temp, &com_ang_moment_temp, false);
    ///////////////////////////////////////////////////////////////////////////////////////////

    Matrix6d R_R;
    R_R.setZero();
    R_R.block(0, 0, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
    R_R.block(3, 3, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
    // R_R.setIdentity();

    jac_com_ = R_R * rd_.link_[COM_id].jac.cast<double>();
    jac_com_pos_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].jac_com.cast<double>().topRows(3);
    jac_rhand_ = R_R * rd_.link_[Right_Hand].jac.cast<double>();
    jac_lhand_ = R_R * rd_.link_[Left_Hand].jac.cast<double>();
    jac_rfoot_ = R_R * rd_.link_[Right_Foot].jac.cast<double>();
    jac_lfoot_ = R_R * rd_.link_[Left_Foot].jac.cast<double>();

    lfoot_to_com_jac_from_global_.setZero(6, MODEL_DOF_VIRTUAL);
    rfoot_to_com_jac_from_global_.setZero(6, MODEL_DOF_VIRTUAL);
    Matrix6d adjoint_pelv_to_ankle;
    adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
    adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - lfoot_transform_current_from_global_.translation());
    adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

    lfoot_to_com_jac_from_global_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_lfoot_).block(0, 0, 3, MODEL_DOF_VIRTUAL) + jac_com_pos_;
    lfoot_to_com_jac_from_global_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_lfoot_).block(3, 0, 3, MODEL_DOF_VIRTUAL);

    adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
    adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - rfoot_transform_current_from_global_.translation());
    adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

    rfoot_to_com_jac_from_global_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_rfoot_).block(0, 0, 3, MODEL_DOF_VIRTUAL) + jac_com_pos_;
    rfoot_to_com_jac_from_global_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_rfoot_).block(3, 0, 3, MODEL_DOF_VIRTUAL);

    // A_mat_ = rd_.A_;
    Eigen::MatrixXd A_temp;
    A_temp.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_C_, rd_.q_virtual_, A_temp, true);
    A_mat_ = A_temp;

    Eigen::MatrixXd C_mat_temp;
    // cout<<"test1"<<endl;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // C_mat_temp = getCMatrix(rd_.q_virtual_, rd_.q_dot_virtual_);    //>1ms... too slow
    // C_mat_temp.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // cout<<"test2"<<endl;
    // cout<<"getCMatrix time: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() <<endl;
    // C_mat_ = C_mat_temp.block(0, 0, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    // cout<<"test3"<<endl;
    // C_T_mat_ = C_mat_.transpose();
    // cout<<"test4"<<endl;

    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    A_dot_mat_ = (A_mat_ - A_mat_pre_) / dt_;
    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    // cout<<"getAdotmat time: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count() <<endl;

    Eigen::VectorXd nonlinear_torque_temp, gravity_torque_temp;
    nonlinear_torque_temp.setZero(MODEL_DOF_VIRTUAL, 1);
    gravity_torque_temp.setZero(MODEL_DOF_VIRTUAL, 1);
    // inv_dya_temp.setZero(MODEL_DOF_VIRTUAL, 1);
    VectorXd q_ddot_virtual_c, q_dot_virtual_c, q_virtual_c;
    q_ddot_virtual_c = rd_.q_ddot_virtual_;
    q_dot_virtual_c = rd_.q_dot_virtual_;
    q_virtual_c = rd_.q_virtual_;
    // cout<<"test5"<<endl;
    // RigidBodyDynamics::UpdateKinematicsCustom(model_c_, &q_virtual_c, &q_dot_virtual_c, &q_ddot_virtual_c);

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
    // RigidBodyDynamics::NonlinearEffects(model_c_, q_virtual_c, q_dot_virtual_c, nonlinear_torque_temp);
    std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
    // cout<<"NonlinearEffects time: "<< std::chrono::duration_cast<std::chrono::nanoseconds>(t6 - t5).count() <<endl;

    // RigidBodyDynamics::NonlinearEffects(model_c_, q_virtual_c, Eigen::VectorXd::Zero(MODEL_DOF_VIRTUAL, 1), gravity_torque_temp);
    // cout<<"test7"<<endl;
    // nonlinear_torque_ = nonlinear_torque_temp;
    // nonlinear_torque_.segment(0, 6) = nonlinear_torque_temp.segment(0, 6) - gravity_torque_temp.segment(0, 6);
    // cout<<"test8"<<endl;
    if (int((current_time_)*10000) % 1000 == 0)
    {
        // cout<<"nonlinear_torque:" << nonlinear_torque_temp.transpose() <<endl;
        // cout<<"gravity_torque_temp:" << gravity_torque_temp.transpose() <<endl;

        // cout<<"corioli from rbdl:" << nonlinear_torque_temp.transpose() - gravity_torque_temp.transpose() <<endl;
        // cout<<"corioli from dg:" << (C_mat_*rd_.q_dot_virtual_).transpose() << endl;
        // cout<<"q_virtual_c:" << q_virtual_c.transpose() <<endl;
        // cout<<"q_dot_virtual_c:" << q_dot_virtual_c.transpose() <<endl;
        // cout<<"q_ddot_virtual_c:" << q_ddot_virtual_c.transpose() <<endl;
        // cout<<"getCmatrix error: "<< (C_mat_*rd_.q_dot_virtual_ - (nonlinear_torque_temp - gravity_torque_temp)).transpose()<<endl;
        // cout<<"Adot error: "<< (A_dot_mat_*rd_.q_dot_virtual_ - (nonlinear_torque_temp - gravity_torque_temp + C_T_mat_*rd_.q_dot_virtual_)).transpose() <<endl;
        // cout<<"A_mat_: \n" << A_mat_ <<endl;
    }
    // A_inv_mat_ = rd_.A_matrix_inverse;
    // motor_inertia_mat_ = rd_.Motor_inertia;
    // motor_inertia_inv_mat_ = rd_.Motor_inertia_inverse;

    // Eigen::Vector3d zmp_local_both_foot;
    // rd_.ZMP_ft = wc_.GetZMPpos(rd_);

    // zmp_local_both_foot = wc.GetZMPpos_fromFT(rd_).segment(0, 2);  //get zmp using f/t sensors on both foot
    Eigen::VectorXd current_momentum = A_mat_ * q_dot_virtual_c;
    Eigen::VectorXd current_torque, nonlinear_torque_g, mob_residual_pre;
    current_torque.setZero(MODEL_DOF_VIRTUAL, 1);
    current_torque.segment(6, MODEL_DOF) = rd_.torque_desired;
    // cout<<"current_torque:" << current_torque.transpose() <<endl;

    nonlinear_torque_g = A_dot_mat_ * rd_.q_dot_virtual_ - (nonlinear_torque_);

    mob_residual_pre = mob_residual_;
    // mob_residual_ = momentumObserver(current_momentum, current_torque, nonlinear_torque_g, mob_residual_pre, dt_, 50);

    // J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    // RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Foot].id, lhand_control_point_offset_, J_temp_, false);
    // J_hqpik_[1].block(0, 0, 3, variable_size_hqpik_) = J_temp_.block(3, 18, 3, variable_size_hqpik_); //position
    // J_hqpik_[1].block(3, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); //orientation

    // cout<<"body velocity: " << q_dot_virtual_c.segment(0, 6).transpose() <<endl;
    // cout<<"mob_residual_: " << mob_residual_.transpose() <<endl;
    // cout<<"left_force_resi: " << (jac_lfoot_*mob_residual_).transpose() <<endl;
    // cout<<"right_force_resi: " << (jac_rfoot_*mob_residual_).transpose() <<endl;
    // cout<<"left_force_resi: " << (jac_lhand_*mob_residual_).transpose() <<endl;
    // cout<<"right_force_resi: " << (jac_rhand_*mob_residual_).transpose() <<endl;

    contact_force_lfoot_ = rd_.LF_CF_FT;
    contact_force_rfoot_ = rd_.RF_CF_FT;

    Matrix6d adt;
    adt.setZero();
    adt.block(0, 0, 3, 3) = lfoot_transform_current_from_global_.linear();
    adt.block(3, 3, 3, 3) = lfoot_transform_current_from_global_.linear();

    contact_force_lfoot_local_ = adt.inverse() * contact_force_lfoot_;

    adt.block(0, 0, 3, 3) = rfoot_transform_current_from_global_.linear();
    adt.block(3, 3, 3, 3) = rfoot_transform_current_from_global_.linear();

    contact_force_rfoot_local_ = adt.inverse() * contact_force_rfoot_;

    zmp_local_lfoot_(0) = -contact_force_lfoot_local_(4) / contact_force_lfoot_local_(2);
    zmp_local_lfoot_(1) = contact_force_lfoot_local_(3) / contact_force_lfoot_local_(2);
    zmp_local_rfoot_(0) = -contact_force_rfoot_local_(4) / contact_force_rfoot_local_(2);
    zmp_local_rfoot_(1) = contact_force_rfoot_local_(3) / contact_force_rfoot_local_(2);

    zmp_dot_local_lfoot_ = (zmp_local_lfoot_ - zmp_local_lfoot_pre_) / dt_;
    zmp_dot_local_rfoot_ = (zmp_local_rfoot_ - zmp_local_rfoot_pre_) / dt_;

    zmp_measured_lfoot_ = lfoot_transform_current_from_global_.linear() * zmp_local_lfoot_ + lfoot_transform_current_from_global_.translation(); //from global

    zmp_measured_rfoot_ = rfoot_transform_current_from_global_.linear() * zmp_local_lfoot_ + rfoot_transform_current_from_global_.translation();

    zmp_measured_ = (zmp_measured_lfoot_ * rd_.LF_CF_FT(2) + zmp_measured_rfoot_ * rd_.RF_CF_FT(2)) / (rd_.LF_CF_FT(2) + rd_.RF_CF_FT(2)); //from global
    zmp_dot_measured_ = (zmp_measured_ - zmp_measured_pre_) / dt_;

    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    first_torque_supplier_ = DyrosMath::cubic(current_time_, program_start_time_ + program_ready_duration_, program_start_time_ + program_ready_duration_ + walking_control_transition_duration_, 0, 1, 0, 0);
}

void AvatarController::walkingStateManager()
{
    if (walking_phase_ < 1)
    {
        if (walking_speed_ == 0)
        {
            //first step start
            if (foot_swing_trigger_ == false)
            {

                start_walking_trigger_ = false;
                stop_walking_trigger_ = true;

                if (stop_walking_trigger_ == true)
                {
                    // if (balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0))) //gonna be fall
                    // // if(false)
                    // {
                    // 	foot_swing_trigger_ = true;
                    // 	first_step_trigger_ = true;
                    // 	start_walking_trigger_ = false;
                    // 	stop_walking_trigger_ = false;
                    // 	start_time_ = current_time_;

                    // 	if(com_vel_current_(1) >=0)
                    // 	{
                    // 		foot_contact_ = -1;
                    // 	}
                    // 	else
                    // 	{
                    // 		foot_contact_ = 1;
                    // 	}

                    // 	// foot_contact_ = -foot_contact_;  //support foot change
                    // 	// std::cout << " ################################ Balancing Control ON! ################################" << std::endl;
                    // }
                    // else
                    // {
                    foot_swing_trigger_ = false;
                    first_step_trigger_ = false;
                    start_time_ = current_time_;
                    // std::cout << " ################################ STOP WALKING ON! ################################" << std::endl;
                    // }
                }
            }
        }
        else
        {
            stop_walking_trigger_ = false;

            if (foot_swing_trigger_ == false)
            {

                start_walking_trigger_ = true;

                if (current_time_ >= start_time_ + walking_duration_start_delay_) // swing foot starts to move
                {
                    foot_swing_trigger_ = true;
                    first_step_trigger_ = true;
                    start_walking_trigger_ = false;
                    start_time_ = current_time_;

                    std::cout << " ################################ First Step Triggered! ################################" << std::endl;
                }
            }
            else
            {

                if (foot_contact_ == 1)
                {

                    // if (-r_ft_(2) > rd_.link_[COM_id].mass * GRAVITY / 2)
                    // {
                    // 	foot_landing_count_ += 1;
                    // }
                    // else
                    // {
                    // 	foot_landing_count_ = 0;
                    // }

                    // if ((walking_phase_ > 0.5) && (foot_landing_count_ > 100) )
                    // {

                    // 	// std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -r_ft_(2) <<") #########################" << std::endl;
                    // 	walking_phase_ = 1;
                    // 	// start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
                    // }
                }
                else if (foot_contact_ == -1)
                {

                    // if (-l_ft_(2) > rd_.link_[COM_id].mass * GRAVITY / 2)
                    // {
                    // 	foot_landing_count_ += 1;
                    // }
                    // else
                    // {
                    // 	foot_landing_count_ = 0;
                    // }

                    // if ((walking_phase_ > 0.5) && (foot_landing_count_ > 100))
                    // {
                    // 	// std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -l_ft_(2) <<") #########################" << std::endl;
                    // 	// start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
                    // 	walking_phase_ = 1;
                    // }
                }
            }
        }
    }

    if (walking_phase_ == 1)
    {
        if (walking_speed_ == 0)
        {
            // if (balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0))) //gonna be fall

            stop_walking_counter_++;
            if (stop_walking_counter_ < max_stop_walking_num_)
            {
                foot_swing_trigger_ = true;
                foot_contact_ = -foot_contact_; //support foot change

                if (first_step_trigger_ == true)
                {
                    first_step_trigger_ = false;
                }
                std::cout << " ################################ Robot Is Stopping! ################################" << std::endl;
                std::cout << " ################################" << max_stop_walking_num_ - stop_walking_counter_ << "steps are left################################" << std::endl;
            }
            else
            {
                foot_swing_trigger_ = false;
                stop_walking_trigger_ = true; //robot stop
                first_step_trigger_ = false;
                start_walking_trigger_ = false;

                // foot_contact_ = -foot_contact_;
                stance_start_time_ = current_time_;

                stop_walking_counter_ = 0;
                std::cout << " ################################ Robot Stops Walking! ################################" << std::endl;
            }
        }
        else
        {
            foot_swing_trigger_ = true;
            stop_walking_trigger_ = false;
            first_step_trigger_ = false;
            start_walking_trigger_ = false;

            foot_contact_ = -foot_contact_;
            std::cout << " ################################ Support Foot Changed! ################################" << std::endl;
        }
        start_time_ = current_time_;
    }

    if (start_walking_trigger_ == true)
    {
        walking_duration_ = walking_duration_cmd_ + walking_duration_start_delay_;
    }
    else
    {
        walking_duration_ = walking_duration_cmd_;
        walking_duration_ = DyrosMath::minmax_cut(walking_duration_, 0.2, 1.5);
    }

    // turning_duration_ = walking_duration_*0.8;
    turning_duration_ = DyrosMath::minmax_cut(turning_duration_, 0.2, 1.5);

    walking_phase_ = (current_time_ - start_time_) / walking_duration_;
    walking_phase_ = DyrosMath::minmax_cut(walking_phase_, 0.0, 1.0);
    turning_phase_ = (current_time_ - start_time_ - (dsp_duration_)) / turning_duration_;
    turning_phase_ = DyrosMath::minmax_cut(turning_phase_, 0.0, 1.0);
    // walking_duration_ = walking_duration_cmd_  - 1.0*(abs(com_pos_error_(1)) + abs(com_vel_error_(1))*0.3) - 1.0*(abs(com_pos_error_(0)) + abs(com_vel_error_(0))*0.3);

    // if( true)
    // {
    // 	cout<<"walking_phase: "<<walking_phase_<<endl;
    // 	cout<<"turning phase: "<<turning_phase_<<endl;
    // }
}

bool AvatarController::balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d)
{
    bool trigger = false;
    Vector2d capture_point_2d;
    Vector2d middle_point_of_foot_2d;
    double omega;
    omega = sqrt(GRAVITY / (com_pos_current_(2) - support_foot_transform_current_.translation()(2)));
    capture_point_2d = com_pos_2d + com_vel_2d / omega;
    middle_point_of_foot_2d = middle_of_both_foot_.segment(0, 2);

    // if(capture_point_2d.norm() > stop_vel_threshold_)
    // {
    //     trigger = true;
    //     cout<<"balance swing foot control activated"<<endl;
    // }
    if ((capture_point_2d(0) > middle_point_of_foot_2d(0) + 0.10) || (capture_point_2d(0) < middle_point_of_foot_2d(0) - 0.05))
    {
        trigger = true;
        // std::cout << "Catpure point in X axis is over the safety boundary! balance swing foot control activated" << std::endl;
    }

    if ((capture_point_2d(1) > lfoot_transform_current_from_global_.translation()(1) - 0.02) || (capture_point_2d(1) < rfoot_transform_current_from_global_.translation()(1) + 0.02))
    {
        trigger = true;
        // std::cout << "Catpure point in Y axis is over the safety boundary! balance swing foot control activated" << std::endl;
    }

    // if( com_vel_2d.norm() > stop_vel_threshold_)
    // {
    //     trigger = true;
    //     cout<<"com vel is over the limit ("<< com_vel_2d.norm()<<")"<<endl;
    // }

    if (abs(com_vel_2d(0)) > 0.2 || abs(com_vel_2d(1)) > 0.15)
    {
        trigger = true;
        // std::cout << "com vel is over the limit (" << com_vel_2d(0) << "," << com_vel_2d(1) << ")" << std::endl;
    }

    if (abs(lfoot_transform_current_from_global_.translation()(0) - rfoot_transform_current_from_global_.translation()(0)) > 0.03 || abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) > 0.25 || abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) < 0.18)
    {
        trigger = true;
        // std::cout << "Foot is not aligned" << std::endl;
    }

    // if (current_q_(3) > 0.2 || current_q_(9) > 0.2)
    // {
    // 	trigger = true;
    // 	cout << "Knee is bent" << endl;
    // }

    return trigger;
}

int AvatarController::checkZMPinWhichFoot(Eigen::Vector2d zmp_measured)
{
    int flag;
    Eigen::Vector2d diff_zmp_lfoot;
    Eigen::Vector2d diff_zmp_rfoot;
    Eigen::Vector2d foot_size;
    double safe_region_ratio = 0.9;

    diff_zmp_lfoot(0) = abs(zmp_measured(0) - lfoot_transform_current_from_global_.translation()(0));
    diff_zmp_lfoot(1) = abs(zmp_measured(1) - lfoot_transform_current_from_global_.translation()(1));

    diff_zmp_rfoot(0) = abs(zmp_measured(0) - rfoot_transform_current_from_global_.translation()(0));
    diff_zmp_rfoot(1) = abs(zmp_measured(1) - rfoot_transform_current_from_global_.translation()(1));

    foot_size(0) = 0.15;
    foot_size(1) = 0.085;
    if ((diff_zmp_lfoot(0) < safe_region_ratio * foot_size(0)) && (diff_zmp_lfoot(1) < safe_region_ratio * foot_size(1)))
    {
        flag = 1; //zmp is in the left foot
    }
    else if ((diff_zmp_rfoot(0) < safe_region_ratio * foot_size(0)) && (diff_zmp_rfoot(1) < safe_region_ratio * foot_size(1)))
    {
        flag = -1; //zmp is in the right foot
    }
    else
    {
        flag = 0;
    }

    return flag;
}

void AvatarController::getProcessedRobotData()
{
    if (foot_contact_ == 1) // left support foot
    {
        swing_foot_transform_current_ = rfoot_transform_current_from_global_;
        support_foot_transform_current_ = lfoot_transform_current_from_global_;
        // swing_foot_transform_current_ = rfoot_transform_pre_desired_from_;
        // support_foot_transform_current_ = lfoot_transform_pre_desired_from_;
        swing_foot_vel_current_ = rfoot_vel_current_from_global_;
        // support_foot_vel_current_.setZero();
        // support_foot_vel_current_.segment(3, 3) = pelv_angvel_current_ - lfoot_to_com_jac_from_global_.block(3, 6, 3, MODEL_DOF)*current_q_dot_;

        // com_vel_est1_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ ;
        // com_vel_est2_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ + DyrosMath::skm(lfoot_transform_current_from_global_.translation())*(support_foot_vel_current_.segment(3, 3));
    }
    else if (foot_contact_ == -1) //right support foot
    {
        swing_foot_transform_current_ = lfoot_transform_current_from_global_;
        support_foot_transform_current_ = rfoot_transform_current_from_global_;
        // swing_foot_transform_current_ = lfoot_transform_pre_desired_from_;
        // support_foot_transform_current_ = rfoot_transform_pre_desired_from_;
        swing_foot_vel_current_ = lfoot_vel_current_from_global_;
        // support_foot_vel_current_.setZero();
        // support_foot_vel_current_.segment(3, 3) = pelv_angvel_current_ - rfoot_to_com_jac_from_global_.block(3, 6, 3, MODEL_DOF)*current_q_dot_;

        // com_vel_est1_ = rfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ ;
        // com_vel_est2_ = rfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ + DyrosMath::skm(rfoot_transform_current_from_global_.translation())*(support_foot_vel_current_.segment(3, 3));
    }
    else if (foot_swing_trigger_ == false)
    {
    }

    //////////////////////////////Variables in Support Foot Frame////////////////////////
    ///////Support Foot Frame's origin is attatched to the Support Foot Frame origin////////////////////////////////
    ///////z axis is aligned with gravity force and upward//////////////////////////////////////////
    ////// x axis is poining out from center of foot to the toe direction//////////////
    Vector3d swing_foot_rpy = DyrosMath::rot2Euler(support_foot_transform_current_.linear());
    Isometry3d support_foot_transform_yaw_align = support_foot_transform_current_;
    // support_foot_transform_yaw_align.linear() = DyrosMath::rotateWithZ(swing_foot_rpy(2));	//global orientation in roll and pitch

    support_foot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * support_foot_transform_yaw_align;
    swing_foot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * swing_foot_transform_current_;
    lfoot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * lfoot_transform_current_from_global_;
    rfoot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * rfoot_transform_current_from_global_;
    pelv_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * pelv_transform_current_from_global_;

    middle_of_both_foot_ = (lfoot_transform_current_from_support_.translation() + rfoot_transform_current_from_support_.translation()) / 2;

    com_pos_current_from_support_ = DyrosMath::multiplyIsometry3dVector3d(support_foot_transform_yaw_align.inverse(), com_pos_current_);
    com_vel_current_from_support_ = support_foot_transform_yaw_align.linear().transpose() * com_vel_current_;
    com_acc_current_from_support_ = support_foot_transform_yaw_align.linear().transpose() * com_acc_current_;

    if (foot_contact_ != foot_contact_pre_)
    {
        com_pos_pre_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_pre_from_support_);
        com_pos_ppre_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_ppre_from_support_);
        com_vel_pre_lpf_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_pre_lpf_from_support_;
        com_vel_ppre_lpf_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_ppre_lpf_from_support_;
        com_vel_pre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_pre_from_support_;
        com_vel_ppre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_ppre_from_support_;
        com_acc_pre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_pre_from_support_;
        com_acc_ppre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_ppre_from_support_;

        com_pos_desired_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_desired_from_support_);
        com_vel_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_desired_from_support_;
        com_acc_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_desired_from_support_;
        com_jerk_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_jerk_desired_from_support_;

        com_pos_pre_desired_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_pre_desired_from_support_);
        com_vel_pre_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_pre_desired_from_support_;
        com_acc_pre_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_pre_desired_from_support_;
        com_jerk_pre_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_jerk_pre_desired_from_support_;

        lfoot_transform_desired_last_ = swing_foot_transform_current_from_support_ * lfoot_transform_desired_;
        rfoot_transform_desired_last_ = swing_foot_transform_current_from_support_ * rfoot_transform_desired_;
        pelv_transform_desired_last_ = swing_foot_transform_current_from_support_ * pelv_transform_desired_;

        cout << "_______________Support Foot is Changed!!!_______________" << endl;
    }
    /////////////////////////////////////////////////////////////////////////////////////

    com_vel_current_lpf_from_support_ = DyrosMath::secondOrderLowPassFilter<3>(
        com_vel_current_from_support_, com_vel_pre_from_support_, com_vel_ppre_from_support_, com_vel_pre_lpf_from_support_, com_vel_ppre_lpf_from_support_,
        com_vel_cutoff_freq_, 1 / sqrt(2), 1 / dt_);
    // com_vel_current_lpf_from_support_ = DyrosMath::lpf<3>(com_vel_current_from_support_, com_vel_pre_lpf_from_support_, 1 / dt_, com_vel_cutoff_freq_);

    zc_ = com_pos_current_from_support_(2);
    wn_ = sqrt(GRAVITY / zc_);

    cp_current_from_suppport_ = com_pos_current_from_support_ + com_vel_current_lpf_from_support_ / wn_;

    // zmp_measured_local_ = WBC::GetZMPpos_fromFT(rd_, true);

    swing_foot_pos_error_from_support_ = swing_foot_pos_trajectory_from_support_ - swing_foot_transform_current_from_support_.translation();
    // middle_of_both_foot_ = (lfoot_transform_current_from_global_.translation() + rfoot_transform_current_from_global_.translation()) / 2;
    // middle_of_both_foot_ = (lfoot_transform_pre_desired_from_.translation() + rfoot_transform_pre_desired_from_.translation())/2;

    if (walking_mode_on_) //command on
    {
        stance_start_time_ = current_time_;
        start_time_ = current_time_;
        program_start_time_ = current_time_;

        init_q_ = current_q_;
        last_desired_q_ = current_q_;

        com_pos_pre_from_support_ = com_pos_current_from_support_;
        com_pos_ppre_from_support_ = com_pos_current_from_support_;
        com_vel_pre_from_support_.setZero();
        com_vel_ppre_from_support_.setZero();
        com_acc_pre_from_support_.setZero();
        com_acc_ppre_from_support_.setZero();

        com_vel_pre_lpf_from_support_.setZero();
        com_vel_ppre_lpf_from_support_.setZero();

        com_pos_desired_preview_ = com_pos_current_;
        com_vel_desired_preview_.setZero();
        com_acc_desired_preview_.setZero();

        com_vel_desired_preview_pre_ = com_pos_current_;
        com_vel_desired_preview_pre_.setZero();
        com_vel_desired_preview_pre_.setZero();

        com_pos_init_from_support_ = com_pos_current_from_support_;

        com_pos_desired_ = com_pos_current_;
        com_vel_desired_.setZero();
        com_acc_desired_.setZero();

        com_pos_desired_last_ = com_pos_current_;
        com_vel_desired_last_.setZero();
        com_acc_desired_last_.setZero();

        com_pos_desired_from_support_ = com_pos_current_from_support_;
        com_vel_desired_from_support_.setZero();
        com_acc_desired_from_support_.setZero();
        com_jerk_desired_from_support_.setZero();

        com_pos_pre_desired_from_support_ = com_pos_current_from_support_;
        com_vel_pre_desired_from_support_.setZero();
        com_acc_pre_desired_from_support_.setZero();
        com_jerk_pre_desired_from_support_.setZero();

        xs_.setZero();
        ys_.setZero();

        xi_ = com_pos_current_from_support_(0);
        yi_ = com_pos_current_from_support_(1);

        xd_.setZero();
        yd_.setZero();
        xd_(0) = com_pos_current_from_support_(0);
        yd_(0) = com_pos_current_from_support_(1);

        xd_b.setZero();
        yd_b.setZero();
        xd_b(0) = com_pos_current_from_support_(0);
        yd_b(0) = com_pos_current_from_support_(1);

        walking_mode_on_ = false;

        swing_foot_pos_error_from_support_.setZero();

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
    }

    bool robot_goes_into_stance_phase = (current_time_ == stance_start_time_);
    bool robot_start_walking = ((start_walking_trigger_ == true) && (current_time_ == start_time_));
    bool robot_start_swing = ((foot_swing_trigger_ == true) && (current_time_ == start_time_));

    if (robot_goes_into_stance_phase || robot_start_walking || robot_start_swing)
    {
        com_pos_init_ = com_pos_current_;
        com_vel_init_ = com_vel_current_;
        com_acc_init_ = com_acc_current_;

        com_pos_init_from_support_ = com_pos_current_from_support_;

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
        if (foot_contact_ == 1) // left support foot
        {
            swing_foot_transform_init_ = rfoot_transform_current_from_global_;
            support_foot_transform_init_ = lfoot_transform_current_from_global_;
            // swing_foot_transform_init_ = rfoot_transform_pre_desired_from_;
            // support_foot_transform_init_ = lfoot_transform_pre_desired_from_;
            swing_foot_vel_init_ = rfoot_vel_current_from_global_;
        }
        else if (foot_contact_ == -1) //right support foot
        {
            swing_foot_transform_init_ = lfoot_transform_current_from_global_;
            support_foot_transform_init_ = rfoot_transform_current_from_global_;
            // swing_foot_transform_init_ = lfoot_transform_pre_desired_from_;
            // support_foot_transform_init_ = rfoot_transform_pre_desired_from_;
            swing_foot_vel_init_ = lfoot_vel_current_from_global_;
        }
        swing_foot_rpy_init_ = DyrosMath::rot2Euler(swing_foot_transform_init_.linear());
        support_foot_rpy_init_ = DyrosMath::rot2Euler(support_foot_transform_init_.linear());

        // init_q_ = current_q_;
        last_desired_q_ = desired_q_;
        foot_lift_count_ = 0;

        com_pos_desired_last_ = com_pos_desired_;
        com_vel_desired_last_ = com_vel_desired_;
        com_acc_desired_last_ = com_acc_desired_;

        middle_of_both_foot_init_ = middle_of_both_foot_;

        swingfoot_f_star_l_pre_.setZero();
        swingfoot_f_star_r_pre_.setZero();

        swing_foot_transform_init_from_support_ = swing_foot_transform_current_from_support_;
        swing_foot_rpy_init_from_support_ = DyrosMath::rot2Euler(swing_foot_transform_init_from_support_.linear());
        support_foot_transform_init_from_support_ = support_foot_transform_current_from_support_;
        support_foot_rpy_init_from_support_ = DyrosMath::rot2Euler(support_foot_transform_init_from_support_.linear());

        lfoot_transform_init_from_support_ = lfoot_transform_current_from_support_;
        rfoot_transform_init_from_support_ = rfoot_transform_current_from_support_;
        pelv_transform_init_from_support_ = pelv_transform_current_from_support_;
        pelv_rpy_init_from_support_ = DyrosMath::rot2Euler(pelv_transform_init_from_support_.linear());
        swing_foot_pos_error_from_support_.setZero();
    }

    if (current_time_ == program_start_time_)
    {
        support_foot_transform_pre_ = support_foot_transform_current_;
        swing_foot_transform_pre_ = swing_foot_transform_current_;

        com_pos_desired_preview_pre_ = com_pos_current_from_support_;
        com_vel_desired_preview_pre_.setZero();
        com_acc_desired_preview_pre_.setZero();

        //preview gain update
        //previewParam_MJ(1 / preview_hz_, zmp_size_, zc_, K_act_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);

        // cout<<"===================First Preview is On==========================="<<endl;
        // cout<<"zc_:"<<endl;
        // cout<<zc_<<endl;
        // cout<<"K_act_:"<<endl;
        // cout<<K_act_<<endl;
        // cout<<"Gi_:"<<endl;
        // cout<<Gi_<<endl;
        // cout<<"Gd_:"<<endl;
        // cout<<Gd_<<endl;
        // cout<<"Gx_:"<<endl;
        // cout<<Gx_<<endl;
        // cout<<"A_:"<<endl;
        // cout<<A_<<endl;
        // cout<<"B_:"<<endl;
        // cout<<B_<<endl;
        // cout<<"C_:"<<endl;
        // cout<<C_<<endl;
        // cout<<"D_:"<<endl;
        // cout<<D_<<endl;
        // cout<<"===================First Preview is On==========================="<<endl;

        last_preview_param_update_time_ = current_time_;
        preview_update_time_ = current_time_;

        for (int i = 0; i < zmp_size_; i++)
        {
            ref_zmp_(i, 0) = com_pos_init_from_support_(0);
            ref_zmp_(i, 1) = com_pos_init_from_support_(1);
        }
    }

    swingfoot_force_control_converter_ = DyrosMath::cubic(walking_phase_, 0.8, 0.9, 0, 1, 0, 0);
    // swingfoot_force_control_converter_ = 0;
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
    // motion_q_(3)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
    motion_q_(3) = knee_target_angle_;
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
    // motion_q_(9)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
    motion_q_(9) = knee_target_angle_;
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
        motion_q_(13) = 0; //pitch
        motion_q_(14) = 0; //roll
        pd_control_mask_(12) = 1;
        pd_control_mask_(13) = 1;
        pd_control_mask_(14) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////HEAD/////////////////////////
        motion_q_(23) = 0; //yaw
        motion_q_(24) = 0; //pitch
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

        for (int i = 12; i < 32; i++)
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
        motion_q_(12) = 0; //pitch
        motion_q_(13) = 0; //pitch
        motion_q_(14) = 0; //roll
        pd_control_mask_(12) = 1;
        pd_control_mask_(13) = 1;
        pd_control_mask_(14) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////HEAD/////////////////////////
        motion_q_(23) = 0; //yaw
        motion_q_(24) = 0; //pitch
        pd_control_mask_(23) = 1;
        pd_control_mask_(24) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////ARM/////////////////////////
        //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
        motion_q_(15) = 0.3;
        motion_q_(16) = 0.12;
        motion_q_(17) = 1.43;
        motion_q_(18) = -0.85;
        motion_q_(19) = -0.45; //elbow
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
        motion_q_(29) = 0.45; //elbow
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

        for (int i = 12; i < 32; i++)
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
        motion_q_(12) = 0; //pitch
        motion_q_(13) = 0; //pitch
        motion_q_(14) = 0; //roll
        pd_control_mask_(12) = 1;
        pd_control_mask_(13) = 1;
        pd_control_mask_(14) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////HEAD/////////////////////////
        motion_q_(23) = 0; //yaw
        motion_q_(24) = 0; //pitch
        pd_control_mask_(23) = 1;
        pd_control_mask_(24) = 1;
        /////////////////////////////////////////////////////

        ///////////////////////ARM/////////////////////////
        //////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
        motion_q_(15) = 0.3;
        motion_q_(16) = -0.6;
        motion_q_(17) = 1.2;
        motion_q_(18) = -0.80;
        motion_q_(19) = -2.3; //elbow
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
        motion_q_(29) = 2.3; //elbow
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

        for (int i = 12; i < 32; i++)
        {
            motion_q_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 4, upperbody_mode_q_init_(i), 0, 0, motion_q_(i), 0, 0)(0);
        }
    }
    else if (upper_body_mode_ == 5) //HEAD ONLY
    {
        if (still_pose_cali_flag_ == false)
        {
            cout << " WARNING: Calibration[STILL POSE] is not completed! Upperbody returns to the init pose" << endl;
            upper_body_mode_ = 3;
            upperbody_mode_recieved_ = true;
            upperbody_command_time_ = current_time_;
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
            J_head.block(0, 0, 3, 2) = J_temp.block(0, 29, 3, 2); //orientation
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
    else if ((upper_body_mode_ == 6)) //HQPIK ver1
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << endl;
            upper_body_mode_ = 3;
            upperbody_mode_recieved_ = true;
            upperbody_command_time_ = current_time_;
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
    else if (upper_body_mode_ == 7) //HQPIK ver2
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << endl;
            upper_body_mode_ = 3;
            upperbody_mode_recieved_ = true;
            upperbody_command_time_ = current_time_;
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
            motionRetargeting_HQPIK2();

            // if (int(current_time_ * 10000) % 10000 == 0)
            // {
            //     cout<<"hqpik_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
            // }
        }
    }
    else if (upper_body_mode_ == 8) //Absolute mapping
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << endl;
            upper_body_mode_ = 3; //freezing
            upperbody_mode_recieved_ = true;
            upperbody_command_time_ = current_time_;
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #7 (ABSOLUTE HAND POS MAPPING)" << endl;

                first_loop_hqpik_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Motion Tracking Contorol in On (ABSOLUTE HAND POS MAPPING)";
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
    else if (upper_body_mode_ == 9) //Propositional mapping
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << endl;
            upper_body_mode_ = 3; //freezing
            upperbody_mode_recieved_ = true;
            upperbody_command_time_ = current_time_;
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #8 (PROPOSITIONAL HAND POS MAPPING)" << endl;

                first_loop_hqpik_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Motion Tracking Contorol in On (PROPOSITIONAL HAND POS MAPPING)";
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
    else if (upper_body_mode_ == 10) //Cali Pose Direction Only
    {
        if (hmd_check_pose_calibration_[3] == false)
        {
            cout << " WARNING: Calibration is not completed! Upperbody returns to the init pose" << endl;
            upper_body_mode_ = 3;
            upperbody_mode_recieved_ = true;
            upperbody_command_time_ = current_time_;
            motion_q_ = motion_q_pre_;
        }
        else
        {
            if (upperbody_mode_recieved_ == true)
            {
                cout << "Upperbody Mode is Changed to #10 (HQPIK1 - Direction Only)" << endl;

                first_loop_hqpik_ = true;
                first_loop_qp_retargeting_ = true;

                std_msgs::String msg;
                std::stringstream upperbody_mode_ss;
                upperbody_mode_ss << "Motion Tracking Contorol in On (HQPIK1 - Direction Only)";
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

    if (int(current_time_ * 1e4) % int(1e3) == 0)
    {
    }
    /////////////////FOOT HEIGHT/////////////////////////

    // if (foot_swing_trigger_ == true)
    // {
    // 	if (walking_phase_ < swingfoot_highest_time_)
    // 	{
    // 		// swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(0);
    // 		// swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(1);
    // 		swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, swingfoot_highest_time_, support_foot_transform_current_.translation()(2)+ swing_foot_transform_init_.translation()(2) - support_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(0);
    // 		swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, swingfoot_highest_time_, support_foot_transform_current_.translation()(2)+ swing_foot_transform_init_.translation()(2) - support_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(1);
    // 		swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, swingfoot_highest_time_, support_foot_transform_current_.translation()(2)+ swing_foot_transform_init_.translation()(2) - support_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(2);
    // 	}
    // 	else
    // 	{
    // 		swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.005, 0, 0)(0);
    // 		swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.005, 0, 0)(1);
    // 		swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.005, 0, 0)(2);
    // 	}

    // }
}

void AvatarController::motionRetargeting()
{

    ///////////////////////WAIST/////////////////////////
    if (yaw_angular_vel_ == 0)
    {
        motion_q_(12) = 0;
    }
    else
    {
        motion_q_(12) = (1 - turning_phase_) * last_desired_q_(12) + turning_phase_ * turning_duration_ * (-yaw_angular_vel_) * 1.0; //yaw
    }
    motion_q_(13) = 0; //pitch
    motion_q_(14) = 0; //roll
    pd_control_mask_(12) = 1;
    pd_control_mask_(13) = 1;
    pd_control_mask_(14) = 1;
    /////////////////////////////////////////////////////

    ///////////////////////HEAD/////////////////////////
    if (yaw_angular_vel_ == 0)
    {
        motion_q_(23) = 0;
    }
    else
    {
        motion_q_(23) = (1 - turning_phase_) * last_desired_q_(23) + turning_phase_ * turning_duration_ * (-yaw_angular_vel_) * 1.2; //yaw
    }
    // motion_q_(23) = 0; //yaw
    motion_q_(24) = 0; //pitch
    pd_control_mask_(23) = 1;
    pd_control_mask_(24) = 1;
    /////////////////////////////////////////////////////

    /////////////////////ARM/////////////////////////////////
    Vector7d qdot_d_larm;
    Vector7d qdot_d_rarm;

    Vector6d u_dot_lhand;
    Vector6d u_dot_rhand;

    Matrix3d kp_pos_lhand;
    Matrix3d kp_pos_rhand;

    Matrix3d kp_ori_lhand;
    Matrix3d kp_ori_rhand;

    Isometry3d lhand_transform_pre_desired_from;
    Isometry3d rhand_transform_pre_desired_from;

    // VectorQVQd q_desired_pre;
    // q_desired_pre.setZero();
    // q_desired_pre(39) = 1;
    // q_desired_pre.segment(6, MODEL_DOF) = motion_q_pre_;

    lhand_transform_pre_desired_from.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, Eigen::Vector3d::Zero(), true);
    rhand_transform_pre_desired_from.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, Eigen::Vector3d::Zero(), true);

    lhand_transform_pre_desired_from.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, false)).transpose();
    rhand_transform_pre_desired_from.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, false)).transpose();

    u_dot_lhand.setZero();
    u_dot_rhand.setZero();
    kp_pos_lhand.setZero();
    kp_pos_rhand.setZero();
    kp_ori_lhand.setZero();
    kp_ori_rhand.setZero();

    kp_pos_lhand(0, 0) = 500;
    kp_pos_lhand(1, 1) = 500;
    kp_pos_lhand(2, 2) = 500;

    kp_pos_rhand(0, 0) = 500;
    kp_pos_rhand(1, 1) = 500;
    kp_pos_rhand(2, 2) = 500;

    kp_ori_lhand(0, 0) = 200;
    kp_ori_lhand(1, 1) = 200;
    kp_ori_lhand(2, 2) = 200;

    kp_ori_rhand(0, 0) = 200;
    kp_ori_rhand(1, 1) = 200;
    kp_ori_rhand(2, 2) = 200;

    // master_rhand_pose_.translation() 	= 	rd_.link_[Right_Hand].x_traj;
    // master_rhand_pose_.linear() 		= 	rd_.link_[Right_Hand].r_traj;
    // master_rhand_vel_.segment(0, 3) 	= 	rd_.link_[Right_Hand].v_traj;
    // master_rhand_vel_.segment(3, 3) 	= 	rd_.link_[Right_Hand].w_traj;

    // master_lhand_pose_.translation() 	= 	rd_.link_[Left_Hand].x_traj;
    // master_lhand_pose_.linear() 		= 	rd_.link_[Left_Hand].r_traj;
    // master_lhand_vel_.segment(0, 3) 	= 	rd_.link_[Left_Hand].v_traj;
    // master_lhand_vel_.segment(3, 3) 	= 	rd_.link_[Left_Hand].w_traj;

    // master_lhand_pose_.translation() 		= lhand_transform_init_from_global_.translation();
    // master_lhand_pose_.translation()(0) 	+= 	0.1*sin(current_time_*2*M_PI/1);
    // master_lhand_pose_.linear().Identity();
    // master_lhand_pose_.linear() = DyrosMath::rotateWithX(90*DEG2RAD)*DyrosMath::rotateWithZ(-90*DEG2RAD)*master_lhand_pose_.linear();
    // master_lhand_vel_.setZero();
    // master_lhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/1)*2*M_PI/1;

    // master_rhand_pose_.translation() 		= rhand_transform_init_from_global_.translation();
    // master_rhand_pose_.translation()(0) 	+= 	0.1*sin(current_time_*2*M_PI/1);
    // master_rhand_pose_.linear().Identity();
    // master_rhand_pose_.linear() = DyrosMath::rotateWithX(-90*DEG2RAD)*DyrosMath::rotateWithZ(90*DEG2RAD)*master_rhand_pose_.linear();
    // master_rhand_vel_.setZero();
    // master_rhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/1)*2*M_PI/1;

    u_dot_lhand.segment(0, 3) = master_lhand_vel_.segment(0, 3) + kp_pos_lhand * (master_lhand_pose_.translation() - lhand_transform_pre_desired_from.translation());
    u_dot_rhand.segment(0, 3) = master_rhand_vel_.segment(0, 3) + kp_pos_rhand * (master_rhand_pose_.translation() - rhand_transform_pre_desired_from.translation());

    for (int i = 0; i < 3; i++)
    {
        u_dot_lhand(i) = DyrosMath::minmax_cut(u_dot_lhand(i), -0.5, 0.5);
        u_dot_rhand(i) = DyrosMath::minmax_cut(u_dot_rhand(i), -0.5, 0.5);
    }

    Vector3d lhand_phi = -DyrosMath::getPhi(lhand_transform_pre_desired_from.linear(), master_lhand_pose_.linear());
    Vector3d rhand_phi = -DyrosMath::getPhi(rhand_transform_pre_desired_from.linear(), master_rhand_pose_.linear());
    u_dot_lhand.segment(3, 3) = master_lhand_vel_.segment(3, 3) + kp_ori_lhand * lhand_phi;
    u_dot_rhand.segment(3, 3) = master_rhand_vel_.segment(3, 3) + kp_ori_rhand * rhand_phi;

    qdot_d_larm = DyrosMath::pinv_SVD(jac_lhand_.block(0, 22, 6, 7), 0.0001) * u_dot_lhand;
    qdot_d_rarm = DyrosMath::pinv_SVD(jac_rhand_.block(0, 32, 6, 7), 0.0001) * u_dot_rhand;

    motion_q_dot_.segment(16, 7) = qdot_d_larm;
    motion_q_dot_.segment(26, 7) = qdot_d_rarm;

    for (int i = 0; i < 7; i++)
    {
        motion_q_(16 + i) += motion_q_pre_(16 + i) + qdot_d_larm(i) * dt_;
        motion_q_(26 + i) += motion_q_pre_(26 + i) + qdot_d_rarm(i) * dt_;
        pd_control_mask_(16 + i) = 1;
        pd_control_mask_(26 + i) = 1;
    }

    motion_q_(15) = 0.3;
    motion_q_(25) = -0.3;
    pd_control_mask_(15) = 1;
    pd_control_mask_(25) = 1;

    motion_q_(19) = DyrosMath::minmax_cut(motion_q_(19), joint_limit_l_(4), joint_limit_h_(4)); //elbow
    motion_q_(29) = DyrosMath::minmax_cut(motion_q_(29), joint_limit_l_(12), joint_limit_h_(12));
    ///////////////////////////////////////////////////////////////////////////////////
}

void AvatarController::motionRetargeting2()
{
    ///////////////////////WAIST/////////////////////////
    motion_q_(12) = (1 - turning_phase_) * init_q_(12) + turning_phase_ * turning_duration_ * (-yaw_angular_vel_) * 1; //yaw
    motion_q_(13) = 0;                                                                                                 //pitch
    motion_q_(14) = 0;                                                                                                 //roll
    pd_control_mask_(12) = 1;
    pd_control_mask_(13) = 1;
    pd_control_mask_(14) = 1;
    /////////////////////////////////////////////////////

    ///////////////////////HEAD/////////////////////////
    motion_q_(23) = (1 - turning_phase_) * init_q_(23) + turning_phase_ * turning_duration_ * (-yaw_angular_vel_) * 1.2; //yaw
    // motion_q_(23) = 0; //yaw
    motion_q_(24) = 0; //pitch
    pd_control_mask_(23) = 1;
    pd_control_mask_(24) = 1;
    /////////////////////////////////////////////////////

    /////////////////////ARM/////////////////////////////////
    for (int i = 0; i < 8; i++)
    {
        pd_control_mask_(15 + i) = 0;
        pd_control_mask_(25 + i) = 0;
    }

    motion_q_(15) = 0.3;
    motion_q_(25) = -0.3;
    pd_control_mask_(15) = 1;
    pd_control_mask_(25) = 1;

    motion_q_(18) = -1.2;
    motion_q_(28) = 1.2;
    pd_control_mask_(18) = 1;
    pd_control_mask_(28) = 1;

    motion_q_(19) = -0.3;
    motion_q_(29) = 0.3;
    pd_control_mask_(19) = 1;
    pd_control_mask_(29) = 1;

    motion_q_(20) = 0.0;
    motion_q_(21) = 0.0;
    motion_q_(22) = 0.0;
    pd_control_mask_(20) = 1;
    pd_control_mask_(21) = 1;
    pd_control_mask_(22) = 1;

    motion_q_(30) = 0.0;
    motion_q_(31) = 0.0;
    motion_q_(32) = 0.0;
    pd_control_mask_(30) = 1;
    pd_control_mask_(31) = 1;
    pd_control_mask_(32) = 1;
    VectorQd torque_d_larm;
    VectorQd torque_d_rarm;

    Vector6d f_d_lhand;
    Vector6d f_d_rhand;

    Matrix3d kp_pos_lhand;
    Matrix3d kp_pos_rhand;
    Matrix3d kd_pos_lhand;
    Matrix3d kd_pos_rhand;

    Matrix3d kp_ori_lhand;
    Matrix3d kp_ori_rhand;
    Matrix3d kd_ori_lhand;
    Matrix3d kd_ori_rhand;

    Isometry3d lhand_transform_pre_desired_from;
    Isometry3d rhand_transform_pre_desired_from;

    f_d_lhand.setZero();
    f_d_rhand.setZero();
    kp_ori_lhand.setZero();
    kp_ori_rhand.setZero();

    kp_pos_lhand(0, 0) = 2500;
    kp_pos_lhand(1, 1) = 2500;
    kp_pos_lhand(2, 2) = 0;

    kp_pos_rhand(0, 0) = 2500;
    kp_pos_rhand(1, 1) = 2500;
    kp_pos_rhand(2, 2) = 0;

    kd_pos_lhand(0, 0) = 100;
    kd_pos_lhand(1, 1) = 100;
    kd_pos_lhand(2, 2) = 0;

    kd_pos_rhand(0, 0) = 100;
    kd_pos_rhand(1, 1) = 100;
    kd_pos_rhand(2, 2) = 0;

    ///////////////swing arm//////////////////
    master_lhand_pose_ = lhand_transform_init_from_global_;
    master_lhand_vel_.setZero();

    master_lhand_pose_.translation()(0) = 1.5 * rknee_transform_current_from_global_.translation()(0) - 0.15;
    master_lhand_pose_.translation()(1) = lhand_transform_init_from_global_.translation()(1);
    master_lhand_pose_.translation()(2) = lhand_transform_init_from_global_.translation()(2);

    master_lhand_pose_.translation()(0) = DyrosMath::minmax_cut(master_lhand_pose_.translation()(0), -0.1, 0.6);

    master_lhand_pose_.linear() = Eigen::Matrix3d::Identity();

    master_rhand_pose_ = rhand_transform_init_from_global_;
    master_rhand_vel_.setZero();

    master_rhand_pose_.translation()(0) = 1.5 * lknee_transform_current_from_global_.translation()(0) - 0.15;
    master_rhand_pose_.translation()(1) = rhand_transform_init_from_global_.translation()(1);
    master_rhand_pose_.translation()(2) = rhand_transform_init_from_global_.translation()(2);

    master_rhand_pose_.translation()(0) = DyrosMath::minmax_cut(master_rhand_pose_.translation()(0), -0.1, 0.6);

    master_rhand_pose_.linear() = Eigen::Matrix3d::Identity();

    /////////////////test////////////////////////
    // master_lhand_pose_.translation() 		= 	lhand_transform_init_from_global_.translation();
    // master_lhand_pose_.translation()(0) 	= 	0.1*sin(current_time_*2*M_PI/3);
    // master_lhand_pose_.linear().Identity();
    // master_lhand_vel_.setZero();
    // master_lhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/3)*2*M_PI/3;

    // master_rhand_pose_.translation() 		= 	rhand_transform_init_from_global_.translation();
    // master_rhand_pose_.translation()(0) 	= 	0.1*sin(current_time_*2*M_PI/3);
    // master_rhand_pose_.linear().Identity();
    // master_rhand_vel_.setZero();
    // master_rhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/3)*2*M_PI/3;

    // hand position
    f_d_lhand.segment(0, 3) = kp_pos_lhand * (master_lhand_pose_.translation() - lhand_transform_current_from_global_.translation()) + kd_pos_lhand * (master_lhand_vel_.segment(0, 3) - lhand_vel_current_from_global_.segment(0, 3));
    f_d_rhand.segment(0, 3) = kp_pos_rhand * (master_rhand_pose_.translation() - rhand_transform_current_from_global_.translation()) + kd_pos_rhand * (master_rhand_vel_.segment(0, 3) - rhand_vel_current_from_global_.segment(0, 3));

    f_d_lhand(2) -= 0;
    f_d_rhand(2) -= 0;
    // hand orientation
    Vector3d phi_lhand;
    Vector3d phi_rhand;
    phi_lhand = -DyrosMath::getPhi(lhand_transform_current_from_global_.linear(), master_lhand_pose_.linear());
    phi_rhand = -DyrosMath::getPhi(rhand_transform_current_from_global_.linear(), master_rhand_pose_.linear());

    double kpa_hand = 000; //angle error gain
    double kva_hand = 00;  //angular velocity gain

    f_d_lhand.segment(3, 3) = kpa_hand * phi_lhand + kva_hand * (master_lhand_vel_.segment(3, 3) - lhand_vel_current_from_global_.segment(3, 3));
    f_d_rhand.segment(3, 3) = kpa_hand * phi_rhand + kva_hand * (master_rhand_vel_.segment(3, 3) - rhand_vel_current_from_global_.segment(3, 3));

    torque_d_larm = (jac_lhand_.transpose() * f_d_lhand).segment(6, MODEL_DOF);
    torque_d_rarm = (jac_rhand_.transpose() * f_d_rhand).segment(6, MODEL_DOF);

    torque_d_larm.segment(12, 3).setZero(); // penalize pelvi torques
    torque_d_rarm.segment(12, 3).setZero();

    torque_task_ += (torque_d_larm + torque_d_rarm);
    // for(int i =0; i<8; i++)
    // {
    // 	torque_task_(15 + i) += torque_d_larm(i);
    // 	torque_task_(25 + i) += torque_d_rarm(i);
    // }
    ///////////////////////////////////////////////////////////////////////////////////
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

    double w1 = 2500;  //hand tracking
    double w2 = 0.002; //joint acc
    double w3 = 1;     //task space vel
    double w4 = 30;    //joint vel
    double w5 = 50;    //upperarm oriention tracking
    double w6 = 1;     //shoulder oriention tracking

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
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, lhand_control_point_offset_, J_temp1, false);
    J_l_arm.block(0, 0, 3, 8) = J_temp1.block(3, 21, 3, 8); //position
    J_l_arm.block(3, 0, 3, 8) = J_temp1.block(0, 21, 3, 8); //orientation
    // J_l_arm.block(0, 0, 6, 1).setZero(); //penalize 1st joint for hand control

    J_temp2.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 4].id, zero3, J_temp2, false);
    J_l_upperarm.block(0, 0, 3, 8) = J_temp2.block(0, 21, 3, 8); //orientation
    // J_l_upperarm.block(0, 0, 3, 1).setZero(); //penalize 1st joint for hand control

    J_temp3.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, zero3, J_temp3, false);
    J_l_shoulder.block(0, 0, 3, 8) = J_temp3.block(0, 21, 3, 8); //orientation

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

    for (int i = 0; i < 3; i++) //position velocity limit
    {
        lbA(i) = -1;
        ubA(i) = 1;
    }

    for (int i = 3; i < 6; i++) //angular velocity limit
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

    double w1 = 2500;  //hand tracking
    double w2 = 0.002; //joint acc
    double w3 = 1;     //task space vel
    double w4 = 30;    //joint vel
    double w5 = 50;    //elbow position tracking
    double w6 = 1;     //shoulder oriention tracking

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
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, rhand_control_point_offset_, J_temp1, false);
    J_r_arm.block(0, 0, 3, 8) = J_temp1.block(3, 31, 3, 8); //position
    J_r_arm.block(3, 0, 3, 8) = J_temp1.block(0, 31, 3, 8); //orientation
    // J_r_arm.block(0, 0, 6, 1).setZero(); //penalize 1st joint for hand control

    J_temp2.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 4].id, zero3, J_temp2, false);
    J_r_upperarm.block(0, 0, 3, 8) = J_temp2.block(0, 31, 3, 8); //orientation
    // J_r_upperarm.block(0, 0, 3, 1).setZero(); //penalize 1st joint for hand control

    J_temp3.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, zero3, J_temp3, false);
    J_r_shoulder.block(0, 0, 3, 8) = J_temp3.block(0, 31, 3, 8); //orientation

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

    for (int i = 0; i < 3; i++) //position velocity limit
    {
        lbA(i) = -1;
        ubA(i) = 1;
    }

    for (int i = 3; i < 6; i++) //angular velocity limit
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

        w1_upperbody_ = 2500;  //upperbody tracking(5000)
        w2_upperbody_ = 2500;  //hand & head(2500)
        w3_upperbody_ = 2500;  //upperarm(50)
        w4_upperbody_ = 2500;  //shoulder(1)
        w5_upperbody_ = 0;     //kinematic energy(50)
        w6_upperbody_ = 0.000; //acceleration(0.002)

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

    //Upperbody
    Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());

    //Hand error
    Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
    Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
    Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
    Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());

    //Head error
    Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
    error_w_head = head_transform_pre_desired_from_.linear().transpose() * error_w_head;
    error_w_head(0) = 0;
    error_w_head = head_transform_pre_desired_from_.linear() * error_w_head;

    //Upperarm error
    Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    error_w_lupperarm(0) = 0;
    Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    error_w_rupperarm(0) = 0;

    //Shoulder error
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

    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Upper_Body].id, zero3, J_temp_, false);
    J_upperbody_[0].block(0, 0, 3, variable_size_upperbody_) = J_temp_.block(0, 18, 3, variable_size_upperbody_); //orientation

    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, lhand_control_point_offset_, J_temp_, false);
    J_upperbody_[1].block(0, 0, 3, variable_size_upperbody_) = J_temp_.block(3, 18, 3, variable_size_upperbody_); //position
    J_upperbody_[1].block(3, 0, 3, variable_size_upperbody_) = J_temp_.block(0, 18, 3, variable_size_upperbody_); //orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, rhand_control_point_offset_, J_temp_, false);
    J_upperbody_[1].block(6, 0, 3, variable_size_upperbody_) = J_temp_.block(3, 18, 3, variable_size_upperbody_); //position
    J_upperbody_[1].block(9, 0, 3, variable_size_upperbody_) = J_temp_.block(0, 18, 3, variable_size_upperbody_); //orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Head].id, zero3, J_temp_, false);
    J_upperbody_[1].block(12, 0, 2, variable_size_upperbody_) = (head_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); //orientation

    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 4].id, zero3, J_temp_, false);
    J_upperbody_[2].block(0, 0, 2, variable_size_upperbody_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); //orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 4].id, zero3, J_temp_, false);
    J_upperbody_[2].block(2, 0, 2, variable_size_upperbody_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); //orientation

    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, zero3, J_temp_, false);
    J_upperbody_[3].block(0, 0, 2, variable_size_upperbody_) = (lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); //orientation
    J_temp_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, zero3, J_temp_, false);
    J_upperbody_[3].block(2, 0, 2, variable_size_upperbody_) = (racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_upperbody_)).block(1, 0, 2, variable_size_upperbody_); //orientation

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

    for (int i = 0; i < 3; i++) //linear velocity limit
    {
        lbA_upperbody_(i) = -2;
        ubA_upperbody_(i) = 2;
        lbA_upperbody_(i + 6) = -2;
        ubA_upperbody_(i + 6) = 2;
    }

    for (int i = 0; i < 3; i++) //angular velocity limit
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
    // const int hierarchy_num_hqpik_ = 4;
    // const int variable_size_hqpik_ = 21;
    // const int constraint_size1_hqpik_ = 21;	//[lb <=	x	<= 	ub] form constraints
    // const int constraint_size2_hqpik_[4] = {12, 15, 17, 21};	//[lb <=	Ax 	<=	ub] or [Ax = b]
    // const int control_size_hqpik_[4] = {3, 14, 4, 4};		//1: upperbody, 2: head + hand, 3: upperarm, 4: shoulder
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

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

            w1_hqpik_[i] = 2500;  //upperbody tracking (2500)
            w2_hqpik_[i] = 50;    //kinetic energy (50)
            w3_hqpik_[i] = 0.000; //acceleration (0.000)
        }

        // upper arm orientation control gain
        w1_hqpik_[2] = 250;   //upperbody tracking (250)
        w2_hqpik_[2] = 50;    //kinetic energy (50)
        w3_hqpik_[2] = 0.002; //acceleration (0.002)

        // shoulder orientation control gain
        w1_hqpik_[3] = 250;   //upperbody tracking (250)
        w2_hqpik_[3] = 50;    //kinetic energy (50)
        w3_hqpik_[3] = 0.002; //acceleration (0.002)

        last_solved_hierarchy_num_ = -1;

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
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Upper_Body].id, zero3, J_temp_, true);
    J_hqpik_[0].block(0, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); //orientation

    Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());
    u_dot_hqpik_[0] = 100 * error_w_upperbody;

    ////2nd Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, lhand_control_point_offset_, J_temp_, false);
    J_hqpik_[1].block(0, 0, 3, variable_size_hqpik_) = J_temp_.block(3, 18, 3, variable_size_hqpik_); //position
    J_hqpik_[1].block(3, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); //orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, rhand_control_point_offset_, J_temp_, false);
    J_hqpik_[1].block(6, 0, 3, variable_size_hqpik_) = J_temp_.block(3, 18, 3, variable_size_hqpik_); //position
    J_hqpik_[1].block(9, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); //orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Head].id, zero3, J_temp_, false);
    J_hqpik_[1].block(12, 0, 2, variable_size_hqpik_) = (head_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation

    //Hand error
    Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
    Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
    Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
    Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());

    //Head error
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
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik_[2].block(0, 0, 2, variable_size_hqpik_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);

    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik_[2].block(2, 0, 2, variable_size_hqpik_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation

    //Upperarm error
    Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    error_w_lupperarm(0) = 0;

    Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    error_w_rupperarm(0) = 0;

    u_dot_hqpik_[2].segment(0, 2) = 100 * error_w_lupperarm.segment(1, 2);
    u_dot_hqpik_[2].segment(2, 2) = 100 * error_w_rupperarm.segment(1, 2);

    ////4th Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik_[3].block(0, 0, 2, variable_size_hqpik_) = (lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik_[3].block(2, 0, 2, variable_size_hqpik_) = (racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation

    //Shoulder error
    Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
    error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
    error_w_lshoulder(0) = 0;

    Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
    error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
    error_w_rshoulder(0) = 0;

    u_dot_hqpik_[3].segment(0, 2) = 100 * error_w_lshoulder.segment(1, 2);
    u_dot_hqpik_[3].segment(2, 2) = 100 * error_w_rshoulder.segment(1, 2);

    for (int i = 0; i < hierarchy_num_hqpik_; i++)
    {
        if (i > last_solved_hierarchy_num_)
        {
            QP_qdot_hqpik_[i].InitializeProblemSize(variable_size_hqpik_, constraint_size2_hqpik_[i]);
        }
    }

    last_solved_hierarchy_num_ = -1;

    std::chrono::steady_clock::time_point t_start_hqpik[hierarchy_num_hqpik_];
    std::chrono::steady_clock::time_point t_end_hqpik[hierarchy_num_hqpik_];

    for (int i = 0; i < hierarchy_num_hqpik_; i++)
    {
        t_start_hqpik[i] = std::chrono::steady_clock::now();
        MatrixXd H1, H2, H3;
        VectorXd g1, g2, g3;

        H1 = J_hqpik_[i].transpose() * J_hqpik_[i];
        // H2 = Eigen::MatrixXd::Identity(variable_size_hqpik_, variable_size_hqpik_);
        H2 = A_mat_.block(18, 18, variable_size_hqpik_, variable_size_hqpik_) + Eigen::MatrixXd::Identity(variable_size_hqpik_, variable_size_hqpik_) * (2e-2);
        H2(3, 3) += 10;   //left arm 1st joint
        H2(13, 13) += 10; //right arm 1st joint
        H3 = Eigen::MatrixXd::Identity(variable_size_hqpik_, variable_size_hqpik_) * (1 / dt_) * (1 / dt_);

        g1 = -J_hqpik_[i].transpose() * u_dot_hqpik_[i];
        g2.setZero(variable_size_hqpik_);
        g3 = -motion_q_dot_pre_.segment(12, variable_size_hqpik_) * (1 / dt_) * (1 / dt_);

        if (i >= 2)
        {
        }

        H_hqpik_[i] = w1_hqpik_[i] * H1 + w2_hqpik_[i] * H2 + w3_hqpik_[i] * H3;
        g_hqpik_[i] = w1_hqpik_[i] * g1 + w2_hqpik_[i] * g2 + w3_hqpik_[i] * g3;

        double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

        for (int j = 0; j < constraint_size1_hqpik_; j++)
        {
            lb_hqpik_[i](j) = min(max(speed_reduce_rate * (joint_limit_l_(j + 12) - current_q_(j + 12)), joint_vel_limit_l_(j + 12)), joint_vel_limit_h_(j + 12));
            ub_hqpik_[i](j) = max(min(speed_reduce_rate * (joint_limit_h_(j + 12) - current_q_(j + 12)), joint_vel_limit_h_(j + 12)), joint_vel_limit_l_(j + 12));
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
                //linear velocity limit
                lbA_hqpik_[i](higher_task_equality_num + j) = -2;
                ubA_hqpik_[i](higher_task_equality_num + j) = 2;
                lbA_hqpik_[i](higher_task_equality_num + j + 6) = -2;
                ubA_hqpik_[i](higher_task_equality_num + j + 6) = 2;

                //angular velocity limit
                lbA_hqpik_[i](higher_task_equality_num + j + 3) = -6;
                ubA_hqpik_[i](higher_task_equality_num + j + 3) = 6;
                lbA_hqpik_[i](higher_task_equality_num + j + 9) = -6;
                ubA_hqpik_[i](higher_task_equality_num + j + 9) = 6;
            }
        }

        // QP_qdot_hqpik_[i].SetPrintLevel(PL_NONE);
        QP_qdot_hqpik_[i].EnableEqualityCondition(equality_condition_eps_);
        QP_qdot_hqpik_[i].UpdateMinProblem(H_hqpik_[i], g_hqpik_[i]);
        // QP_qdot_hqpik_[i].DeleteSubjectToAx();
        // QP_qdot_hqpik_[i].DeleteSubjectToX();
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
            if (i < 3)
            {
                if (int(current_time_ * 2000) % 1000 == 0)
                {
                    std::cout << "Error hierarchy: " << i << std::endl;
                    std::cout << "last solved q_dot: " << q_dot_hqpik_[last_solved_hierarchy_num_].transpose() << std::endl;
                }
            }
            // cout<<"Error qpres_: \n"<< qpres_ << endl;
            break;
        }
        // cout<<"ubA_[0]: " << ubA_[0]<<endl;
        t_end_hqpik[i] = std::chrono::steady_clock::now();
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    if (int(current_time_ * 2000) % 1000 == 0)
    {
        // cout<<"u_dot_[0]- J_hqpik_[0]*q_dot_hqpik_[0]: \n" << u_dot_hqpik_[0] - J_hqpik_[0]*q_dot_hqpik_[0] << endl;
        // cout<<"u_dot_[0]- J_hqpik_[0]*q_dot_hqpik_[1]: \n" << u_dot_hqpik_[0] - J_hqpik_[0]*q_dot_hqpik_[1] << endl;
        // cout<<"u_dot_[0]- J_hqpik_[0]*q_dot_hqpik_[2]: \n" << u_dot_hqpik_[0] - J_hqpik_[0]*q_dot_hqpik_[2] << endl;
        // cout<<"u_dot_[0]- J_hqpik_[0]*q_dot_hqpik_[3]: \n" << u_dot_hqpik_[0] - J_hqpik_[0]*q_dot_hqpik_[3] << endl;

        // cout<<"u_dot_[1]- J_hqpik_[1]*q_dot_hqpik_[1]: \n" << u_dot_hqpik_[1] - J_hqpik_[1]*q_dot_hqpik_[1] << endl;
        // cout<<"u_dot_[2]- J_hqpik_[2]*q_dot_hqpik_[2]: \n" << u_dot_hqpik_[2] - J_hqpik_[2]*q_dot_hqpik_[2] << endl;

        // cout<<"J_hqpik_[1]: \n" << J_hqpik_[1]<<endl;
        // cout<<"u_dot_[0]: \n" << u_dot_[0]<<endl;
        // cout<<"u_dot_[1]: \n" << u_dot_[1]<<endl;
        // cout<<"u_dot_[2]: \n" << u_dot_[2]<<endl;

        cout << "HQPIK time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << endl;

        for (int i = 0; i < hierarchy_num_hqpik_; i++)
        {
            cout << "iteration " << i << "-th time: " << std::chrono::duration_cast<std::chrono::microseconds>(t_end_hqpik[i] - t_start_hqpik[i]).count() << endl;
        }
    }

    // cout<<"J_hqpik_[0]: \n"<< J_hqpik_[0]<<endl;
    for (int i = 0; i < variable_size_hqpik_; i++)
    {
        motion_q_dot_(12 + i) = q_dot_hqpik_[last_solved_hierarchy_num_](i);
        motion_q_(12 + i) = motion_q_pre_(12 + i) + motion_q_dot_(12 + i) * dt_;
        pd_control_mask_(12 + i) = 1;
    }

    lhand_vel_error_ = J_hqpik_[1].block(0, 0, 6, variable_size_hqpik_) * q_dot_hqpik_[last_solved_hierarchy_num_] - u_dot_hqpik_[1].segment(0, 6);
    lelbow_vel_error_.segment(1, 2) = J_hqpik_[2].block(0, 0, 2, variable_size_hqpik_) * q_dot_hqpik_[last_solved_hierarchy_num_] - u_dot_hqpik_[2].segment(0, 2);
    lacromion_vel_error_.segment(1, 2) = J_hqpik_[3].block(0, 0, 2, variable_size_hqpik_) * q_dot_hqpik_[last_solved_hierarchy_num_] - u_dot_hqpik_[3].segment(0, 2);

    rhand_vel_error_ = J_hqpik_[1].block(6, 0, 6, variable_size_hqpik_) * q_dot_hqpik_[last_solved_hierarchy_num_] - u_dot_hqpik_[1].segment(6, 6);
    relbow_vel_error_.segment(1, 2) = J_hqpik_[2].block(2, 0, 2, variable_size_hqpik_) * q_dot_hqpik_[last_solved_hierarchy_num_] - u_dot_hqpik_[2].segment(2, 2);
    racromion_vel_error_.segment(1, 2) = J_hqpik_[3].block(2, 0, 2, variable_size_hqpik_) * q_dot_hqpik_[last_solved_hierarchy_num_] - u_dot_hqpik_[3].segment(2, 2);

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

            w1_hqpik2_[i] = 2500;  //upperbody tracking (2500)
            w2_hqpik2_[i] = 50;    //kinetic energy (50)
            w3_hqpik2_[i] = 0.000; //acceleration ()
        }

        // upper arm orientation control gain
        w1_hqpik2_[3] = 250;   //upperbody tracking (2500)
        w2_hqpik2_[3] = 50;    //kinetic energy (50)
        w3_hqpik2_[3] = 0.002; //acceleration ()

        // shoulder orientation control gain
        w1_hqpik2_[4] = 250;   //upperbody tracking (2500)
        w2_hqpik2_[4] = 50;    //kinetic energy (50)
        w3_hqpik2_[4] = 0.002; //acceleration ()

        last_solved_hierarchy_num_ = -1;

        first_loop_hqpik2_ = false;
    }
    // VectorQVQd q_desired_pre;
    // q_desired_pre.setZero();
    // q_desired_pre(39) = 1;
    // q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;
    Vector3d zero3;
    zero3.setZero();

    ////1st Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Head].id, zero3, J_temp_, false);
    J_hqpik2_[0].block(0, 0, 2, variable_size_hqpik2_) = J_temp_.block(3, 18, 2, variable_size_hqpik2_);                                                                                                 //x, y position
    J_hqpik2_[0].block(2, 0, 2, variable_size_hqpik2_) = (head_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); //y, z orientation
    //Head error
    Vector3d error_v_head = master_head_pose_.translation() - head_transform_pre_desired_from_.translation();
    Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
    error_w_head = head_transform_pre_desired_from_.linear().transpose() * error_w_head;
    error_w_head(0) = 0;
    u_dot_hqpik2_[0].segment(0, 2) = 100 * error_v_head.segment(0, 2);
    u_dot_hqpik2_[0].segment(2, 2) = 200 * error_w_head.segment(1, 2);

    ///2nd Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, lhand_control_point_offset_, J_temp_, false);
    J_hqpik2_[2].block(0, 0, 3, variable_size_hqpik2_) = J_temp_.block(3, 18, 3, variable_size_hqpik2_); //position
    J_hqpik2_[2].block(3, 0, 3, variable_size_hqpik2_) = J_temp_.block(0, 18, 3, variable_size_hqpik2_); //orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, rhand_control_point_offset_, J_temp_, false);
    J_hqpik2_[2].block(6, 0, 3, variable_size_hqpik2_) = J_temp_.block(3, 18, 3, variable_size_hqpik2_); //position
    J_hqpik2_[2].block(9, 0, 3, variable_size_hqpik2_) = J_temp_.block(0, 18, 3, variable_size_hqpik2_); //orientation
    //Hand error
    Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
    Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
    Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
    Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());
    u_dot_hqpik2_[2].segment(0, 3) = 200 * error_v_lhand;
    u_dot_hqpik2_[2].segment(3, 3) = 100 * error_w_lhand;
    u_dot_hqpik2_[2].segment(6, 3) = 200 * error_v_rhand;
    u_dot_hqpik2_[2].segment(9, 3) = 100 * error_w_rhand;

    ////3rd Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Upper_Body].id, zero3, J_temp_, true);
    J_hqpik2_[1].block(0, 0, 3, variable_size_hqpik2_) = J_temp_.block(0, 18, 3, variable_size_hqpik2_); //orientation
    //upper body error
    Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());
    u_dot_hqpik2_[1] = 100 * error_w_upperbody;

    ////4th Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik2_[3].block(0, 0, 2, variable_size_hqpik2_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); //orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 4].id, zero3, J_temp_, false);
    J_hqpik2_[3].block(2, 0, 2, variable_size_hqpik2_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); //orientation
    //Upper arm error
    Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
    error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
    error_w_lupperarm(0) = 0;
    Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
    error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
    error_w_rupperarm(0) = 0;
    u_dot_hqpik2_[3].segment(0, 2) = 100 * error_w_lupperarm.segment(1, 2);
    u_dot_hqpik2_[3].segment(2, 2) = 100 * error_w_rupperarm.segment(1, 2);

    ////5th Task
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik2_[4].block(0, 0, 2, variable_size_hqpik2_) = (lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); //orientation
    J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, zero3, J_temp_, false);
    J_hqpik2_[4].block(2, 0, 2, variable_size_hqpik2_) = (racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik2_)).block(1, 0, 2, variable_size_hqpik2_); //orientation
    //Shoulder error
    Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
    error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
    error_w_lshoulder(0) = 0;
    Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
    error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
    error_w_rshoulder(0) = 0;
    u_dot_hqpik2_[4].segment(0, 2) = 100 * error_w_lshoulder.segment(1, 2);
    u_dot_hqpik2_[4].segment(2, 2) = 100 * error_w_rshoulder.segment(1, 2);

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
        // H2 = Eigen::MatrixXd::Identity(variable_size_hqpik2_, variable_size_hqpik2_);
        H2 = A_mat_.block(18, 18, variable_size_hqpik2_, variable_size_hqpik2_) + Eigen::MatrixXd::Identity(variable_size_hqpik2_, variable_size_hqpik2_) * (2e-2);
        H2(3, 3) += 10;   //left arm 1st joint
        H2(13, 13) += 10; //right arm 1st joint
        H3 = Eigen::MatrixXd::Identity(variable_size_hqpik2_, variable_size_hqpik2_) * (1 / dt_) * (1 / dt_);

        g1 = -J_hqpik2_[i].transpose() * u_dot_hqpik2_[i];
        g2.setZero(variable_size_hqpik2_);
        g3 = -motion_q_dot_pre_.segment(12, variable_size_hqpik2_) * (1 / dt_) * (1 / dt_);

        if (i >= 2)
        {
        }
        H_hqpik2_[i] = w1_hqpik2_[i] * H1 + w2_hqpik2_[i] * H2 + w3_hqpik2_[i] * H3;
        g_hqpik2_[i] = w1_hqpik2_[i] * g1 + w2_hqpik2_[i] * g2 + w3_hqpik2_[i] * g3;

        double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

        for (int j = 0; j < constraint_size1_hqpik2_; j++)
        {
            lb_hqpik2_[i](j) = max(speed_reduce_rate * (joint_limit_l_(j + 12) - current_q_(j + 12)), joint_vel_limit_l_(j + 12));
            ub_hqpik2_[i](j) = min(speed_reduce_rate * (joint_limit_h_(j + 12) - current_q_(j + 12)), joint_vel_limit_h_(j + 12));
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
        if (i < 3)
        {
            A_hqpik2_[i].block(higher_task_equality_num, 0, 12, variable_size_hqpik2_) = J_hqpik2_[2].block(0, 0, 12, variable_size_hqpik2_);

            for (int j = 0; j < 3; j++)
            {
                //linear velocity limit
                lbA_hqpik2_[i](higher_task_equality_num + j) = -1;
                ubA_hqpik2_[i](higher_task_equality_num + j) = 1;
                lbA_hqpik2_[i](higher_task_equality_num + j + 6) = -1;
                ubA_hqpik2_[i](higher_task_equality_num + j + 6) = 1;

                //angular velocity limit
                lbA_hqpik2_[i](higher_task_equality_num + j + 3) = -3;
                ubA_hqpik2_[i](higher_task_equality_num + j + 3) = 3;
                lbA_hqpik2_[i](higher_task_equality_num + j + 9) = -3;
                ubA_hqpik2_[i](higher_task_equality_num + j + 9) = 3;
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

// void AvatarController::motionRetargeting_HQPIK_lexls()
// {

//     // std::vector<std::vector<LexLS::ConstraintActivationType>> active_set_guess;
//     // Eigen::VectorXd solution_guess;
//     // Eigen::VectorXd solution;

//     if (first_loop_hqpik_)
//     {
//         //set variables and objectives
//         type_of_hierarchy = LexLS::tools::HIERARCHY_TYPE_EQUALITY;
//         number_of_variables = 21;
//         number_of_objectives = 5;
//         number_of_constraints.resize(number_of_objectives);
//         types_of_objectives.resize(number_of_objectives);
//         objectives.resize(number_of_objectives);
//         number_of_constraints[0] = 33;
//         number_of_constraints[1] = 3;
//         number_of_constraints[2] = 14;
//         number_of_constraints[3] = 4;
//         number_of_constraints[4] = 4;

//         //// without inequality constraints /////
//         // number_of_constraints[0] = 3;
//         // number_of_constraints[1] = 14;
//         // number_of_constraints[2] = 4;
//         // number_of_constraints[3] = 4;

//         objectives[0].resize(number_of_constraints[0], number_of_variables + 2);
//         objectives[1].resize(number_of_constraints[1], number_of_variables + 2);
//         objectives[2].resize(number_of_constraints[2], number_of_variables + 2);
//         objectives[3].resize(number_of_constraints[3], number_of_variables + 2);
//         objectives[4].resize(number_of_constraints[4], number_of_variables + 2);

//         for(unsigned int i = 0; i < number_of_objectives; i++)
//         {
//             types_of_objectives[i] = LexLS::ObjectiveType::GENERAL_OBJECTIVE;
//         }

//         // parameters.setDefaults();
//         // parameters.max_number_of_factorizations = 100;
//         // parameters.variable_regularization_factor = 0.0;
//         // lsi_.setParameters(parameters);

//         for (int i = 0; i < hierarchy_num_hqpik_; i++)
//         {
//             J_hqpik_[i].setZero(control_size_hqpik_[i], variable_size_hqpik_);
//             u_dot_hqpik_[i].setZero(control_size_hqpik_[i]);

//             ubA_hqpik_[i].setZero(constraint_size2_hqpik_[i]);
//             lbA_hqpik_[i].setZero(constraint_size2_hqpik_[i]);

//             H_hqpik_[i].setZero(variable_size_hqpik_, variable_size_hqpik_);
//             g_hqpik_[i].setZero(variable_size_hqpik_);

//             ub_hqpik_[i].setZero(constraint_size1_hqpik_);
//             lb_hqpik_[i].setZero(constraint_size1_hqpik_);

//             q_dot_hqpik_[i].setZero(variable_size_hqpik_);
//         }

//         first_loop_hqpik_ = false;
//     }

//     LexLS::internal::LexLSI lsi_(number_of_variables, number_of_objectives, &number_of_constraints[0], &types_of_objectives[0]);

//     parameters.setDefaults();
//     parameters.max_number_of_factorizations = 100;
//     parameters.regularization_type = LexLS::RegularizationType::REGULARIZATION_R;
//     parameters.variable_regularization_factor = 0.02;

//     lsi_.setParameters(parameters);
//     // lsi_.resize(number_of_variables, number_of_objectives, &number_of_constraints[0], &types_of_objectives[0]);
//     lsi_.set_x0(motion_q_dot_.segment(12, 21));
//     // cout<<"test1"<<endl;

//     Vector3d zero3;
//     zero3.setZero();
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);

//     ////1st Task
//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Upper_Body].id, zero3, J_temp_, true);
//     J_hqpik_[0].block(0, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); //orientation

//     Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());
//     u_dot_hqpik_[0] = 100 * error_w_upperbody;

//     ////2nd Task
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand].id, lhand_control_point_offset_, J_temp_, false);
//     J_hqpik_[1].block(0, 0, 3, variable_size_hqpik_) = J_temp_.block(3, 18, 3, variable_size_hqpik_); //position
//     J_hqpik_[1].block(3, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); //orientation
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand].id, rhand_control_point_offset_, J_temp_, false);
//     J_hqpik_[1].block(6, 0, 3, variable_size_hqpik_) = J_temp_.block(3, 18, 3, variable_size_hqpik_); //position
//     J_hqpik_[1].block(9, 0, 3, variable_size_hqpik_) = J_temp_.block(0, 18, 3, variable_size_hqpik_); //orientation
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Head].id, zero3, J_temp_, false);
//     J_hqpik_[1].block(12, 0, 2, variable_size_hqpik_) = (head_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation

//     //Hand error
//     Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
//     Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
//     Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
//     Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());

//     //Head error
//     Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
//     error_w_head = head_transform_pre_desired_from_.linear().transpose() * error_w_head;
//     error_w_head(0) = 0;

//     u_dot_hqpik_[1].segment(0, 3) = 200 * error_v_lhand;
//     u_dot_hqpik_[1].segment(3, 3) = 100 * error_w_lhand;
//     u_dot_hqpik_[1].segment(6, 3) = 200 * error_v_rhand;
//     u_dot_hqpik_[1].segment(9, 3) = 100 * error_w_rhand;
//     u_dot_hqpik_[1].segment(12, 2) = 200 * error_w_head.segment(1, 2);

//     ////3rd Task
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 4].id, zero3, J_temp_, false);
//     J_hqpik_[2].block(0, 0, 2, variable_size_hqpik_) = (lupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);

//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 4].id, zero3, J_temp_, false);
//     J_hqpik_[2].block(2, 0, 2, variable_size_hqpik_) = (rupperarm_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation

//     //Upperarm error
//     Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
//     error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose() * error_w_lupperarm;
//     error_w_lupperarm(0) = 0;

//     Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
//     error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose() * error_w_rupperarm;
//     error_w_rupperarm(0) = 0;

//     u_dot_hqpik_[2].segment(0, 2) = 100 * error_w_lupperarm.segment(1, 2);
//     u_dot_hqpik_[2].segment(2, 2) = 100 * error_w_rupperarm.segment(1, 2);

//     ////4th Task
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Left_Hand - 6].id, zero3, J_temp_, false);
//     J_hqpik_[3].block(0, 0, 2, variable_size_hqpik_) = (lacromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation
//     J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
//     RigidBodyDynamics::CalcPointJacobian6D(model_d_, pre_desired_q_qvqd_, rd_.link_[Right_Hand - 6].id, zero3, J_temp_, false);
//     J_hqpik_[3].block(2, 0, 2, variable_size_hqpik_) = (racromion_transform_pre_desired_from_.linear().transpose() * J_temp_.block(0, 18, 3, variable_size_hqpik_)).block(1, 0, 2, variable_size_hqpik_); //orientation

//     //Shoulder error
//     Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
//     error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose() * error_w_lshoulder;
//     error_w_lshoulder(0) = 0;

//     Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
//     error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose() * error_w_rshoulder;
//     error_w_rshoulder(0) = 0;

//     u_dot_hqpik_[3].segment(0, 2) = 100 * error_w_lshoulder.segment(1, 2);
//     u_dot_hqpik_[3].segment(2, 2) = 100 * error_w_rshoulder.segment(1, 2);

//     double speed_reduce_rate = 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

//     for (int j = 0; j < constraint_size1_hqpik_; j++)
//     {
//         lb_hqpik_[0](j) = min( max(speed_reduce_rate * (joint_limit_l_(j + 12) - current_q_(j + 12)), joint_vel_limit_l_(j + 12)), joint_vel_limit_h_(j + 12));
//         ub_hqpik_[0](j) = max( min(speed_reduce_rate * (joint_limit_h_(j + 12) - current_q_(j + 12)), joint_vel_limit_h_(j + 12)), joint_vel_limit_l_(j + 12));
//     }

//     A_hqpik_[0].setZero(constraint_size2_hqpik_[0], variable_size_hqpik_);
//     A_hqpik_[0].block(0, 0, 12, variable_size_hqpik_) = J_hqpik_[1].block(0, 0, 12, variable_size_hqpik_);

//     for (int j = 0; j < 3; j++)
//     {
//         //linear velocity limit
//         lbA_hqpik_[0](0 + j) = -2;
//         ubA_hqpik_[0](0 + j) = 2;
//         lbA_hqpik_[0](0 + j + 6) = -2;
//         ubA_hqpik_[0](0 + j + 6) = 2;

//         //angular velocity limit
//         lbA_hqpik_[0](0 + j + 3) = -6;
//         ubA_hqpik_[0](0 + j + 3) = 6;
//         lbA_hqpik_[0](0 + j + 9) = -6;
//         ubA_hqpik_[0](0 + j + 9) = 6;
//     }
//     // cout<<"test2"<<endl;
//     std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//     for(unsigned int i = 0; i < number_of_objectives; ++i)
//     {
//         if( i == 0)
//         {
//             objectives[i].block(0, 0, 12, number_of_variables) = A_hqpik_[0];
//             objectives[i].block(0, number_of_variables, 12, 1) = lbA_hqpik_[0];
//             objectives[i].block(0, number_of_variables+1, 12, 1) = ubA_hqpik_[0];

//             objectives[i].block(12, 0, number_of_variables, number_of_variables) = Eigen::MatrixXd::Identity(number_of_variables, number_of_variables);
//             objectives[i].block(12, number_of_variables, number_of_variables, 1) = lb_hqpik_[0];
//             objectives[i].block(12, number_of_variables+1, number_of_variables, 1) = ub_hqpik_[0];
//         }
//         else
//         {
//             objectives[i].block(0, 0, number_of_constraints[i], number_of_variables) = J_hqpik_[i-1];
//             objectives[i].block(0, number_of_variables, number_of_constraints[i], 1) = u_dot_hqpik_[i-1];
//             objectives[i].block(0, number_of_variables+1, number_of_constraints[i], 1) = u_dot_hqpik_[i-1];
//         }

//         // objectives[i].block(0, 0, number_of_constraints[i], number_of_variables) = J_hqpik_[i];
//         // objectives[i].block(0, number_of_variables, number_of_constraints[i], 1) = u_dot_hqpik_[i];
//         // objectives[i].block(0, number_of_variables+1, number_of_constraints[i], 1) = u_dot_hqpik_[i];

//         // for(int j=0; j<objectives[i].rows(); j++)
//         // {
//         //     if( objectives[i](j, 21) > objectives[i](j, 22))
//         //     {
//         //         cout<<"lower bound is greater than the upperbodyn at ("<<i<<", "<<j<<"): "<< objectives[i](j, 21)<<", "<<objectives[i](j, 22)<<endl;
//         //     }
//         // }
//     }

//     std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//     // cout<<"test3"<<endl;
//     for (unsigned int i = 0; i < number_of_objectives; ++i)
//     {
//         lsi_.setData(i, objectives[i]);
//     }
//     std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
//     // cout<<"test4"<<endl;
//     LexLS::TerminationStatus solve_flag;
//     solve_flag = lsi_.solve();
//     if(solve_flag == LexLS::TerminationStatus::PROBLEM_SOLVED_CYCLING_HANDLING)
//     {
//         if( int(rd_.control_time_ * 2000) % int(1000) == 0 )
//             cout<<"lexls output is PROBLEM_SOLVED_CYCLING_HANDLING "<<lsi_.getFactorizationsCount()<<endl;
//     }
//     else if(solve_flag == LexLS::TerminationStatus::MAX_NUMBER_OF_FACTORIZATIONS_EXCEEDED)
//     {
//         if( int(rd_.control_time_ * 2000) % int(1000) == 0 )
//             cout<<"lexls output is MAX_NUMBER_OF_FACTORIZATIONS_EXCEEDED "<<lsi_.getFactorizationsCount()<<endl;
//     }
//     else if(solve_flag == LexLS::TerminationStatus::PROBLEM_SOLVED)
//     {
//         if( int(rd_.control_time_ * 2000) % int(1000) == 0 )
//             cout<<"lexls output is PROBLEM_SOLVED "<<lsi_.getFactorizationsCount()<<endl;
//     }

//     Eigen::VectorXd X(number_of_variables);
//     // X = lsi_.get_xStar();
//     X = lsi_.get_x();

//     std::vector<LexLS::ConstraintIdentifier> ctr;
//     lsi_.getActiveCtr_order(ctr);
//     std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
//     if(  int(rd_.control_time_ * 2000) % int(1000) == 0  )
//     {
//         cout<<"HQPIK_lexls time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t1).count() <<endl;
//         // cout<<"lexls setting time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() <<endl;
//         // cout<<"lsi_ solve time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() <<endl;
//         ctr[0].print();
//         cout<<"Lexls output: "<<X.transpose()<<endl;
//     }

//     // cout<<"HQPIK output: "<<motion_q_dot_.segment(12, 21).transpose()<<endl;
//     // cout<<"X - motion_q_ : "<< (X - motion_q_dot_.segment(12, 21)).transpose()<<endl;

//     for (int i = 0; i < variable_size_hqpik_; i++)
//     {
//         motion_q_dot_(12 + i) = X(i);
//         motion_q_(12 + i) = motion_q_pre_(12 + i) + motion_q_dot_(12 + i) * dt_;
//         pd_control_mask_(12 + i) = 1;
//     }
// }

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

        if (current_time_ - tracker_status_changed_time_ <= 5)
        {
            // double w = DyrosMath::cubic(current_time_, tracker_status_changed_time_, tracker_status_changed_time_+5, 0, 1, 0, 0);
            double w = (current_time_ - tracker_status_changed_time_) / 5;
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
                cout << "Motion Tracking Resume!" << int((current_time_ - tracker_status_changed_time_) / 5 * 100) << "%" << endl;
        }
        else
        {
        }
    }
    else //false
    {
        if (hmd_tracker_status_pre_ == true)
        {
            tracker_status_changed_time_ = current_time_;
            cout << "tracker is detatched" << endl;

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

    if (int(current_time_ * 1e4) % int(1e3) == 0)
    {
        // cout<<"hmd_lhand_pose_: "<<hmd_lhand_pose_.translation()<<endl;
        // cout<<"hmd_lhand_pose_raw_: "<<hmd_lhand_pose_raw_.translation()<<endl;
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

        // hmd_pelv_pose_.translation() = kinematicFilter(hmd_pelv_pose_.translation(), hmd_pelv_pose_pre_.translation(), hmd_pelv_pose_init_.translation(), pelv_pos_boundary, far_pelv_move);

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
    }
    else
    {
        hmd_pelv_pose_yaw_only.translation() = hmd_pelv_pose_init_.translation();
    }

    // hmd_pelv_pose_yaw_only.translation() = hmd_pelv_pose_.translation();
    hmd_pelv_rpy = DyrosMath::rot2Euler(hmd_pelv_pose_.linear());
    hmd_pelv_yaw_rot = DyrosMath::rotateWithZ(hmd_pelv_rpy(2));
    hmd_pelv_pose_yaw_only.linear() = hmd_pelv_yaw_rot;

    //coordinate conversion
    hmd_head_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_head_pose_;
    hmd_lupperarm_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_lupperarm_pose_;
    hmd_lhand_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_lhand_pose_;
    hmd_rupperarm_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_rupperarm_pose_;
    hmd_rhand_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_rhand_pose_;
    hmd_chest_pose_ = hmd_pelv_pose_yaw_only.inverse() * hmd_chest_pose_;
    // hmd_pelv_pose_.linear().setIdentity();

    Eigen::Vector3d tracker_offset;
    // tracker_offset << -0.08, 0, 0;  //bebop
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
        //20210209
        // 		hmd_tpose_cali_lhand_pos_: -0.284813
        //  0.803326
        //  0.356484
        // hmd_tpose_cali_rhand_pos_: -0.146704
        // -0.806712
        //  0.357469
        // hmd_tpose_cali_lhand_pos_ <<  -0.284813, 0.803326, 0.356484;
        // hmd_tpose_cali_rhand_pos_ <<  -0.146704, -0.806712, 0.357469;

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

        //20210209

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

        // hmd_still_cali_lhand_pos_
        // hmd_still_cali_rhand_pos_

        // hmd_head_pose_init_
        // hmd_lupperarm_pose_init_
        // hmd_lhand_pose_init_
        // hmd_rshoulder_pose_init_
        // hmd_rupperarm_pose_init_
        // hmd_rhand_pose_init_
        // hmd_pelv_pose_init_
        // hmd_chest_pose_init_

        // hmd_tpose_cali_lhand_pos_
        // hmd_tpose_cali_rhand_pos_

        // hmd_forward_cali_lhand_pos_
        // hmd_forward_cali_rhand_pos_

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
                        << "l_angle_btw_bases: " << l_angle_btw_bases.transpose() << "\n"
                        << "r_angle_btw_bases: " << r_angle_btw_bases.transpose() << endl;

        msg.data = arm_length_data.str();
        calibration_state_pub.publish(msg);
        calibration_state_gui_log_pub.publish(msg);

        cout << "hmd_lshoulder_center_pos_: " << hmd_lshoulder_center_pos_.transpose() << endl;
        cout << "hmd_rshoulder_center_pos_: " << hmd_rshoulder_center_pos_.transpose() << endl;
        cout << "hmd_larm_max_l_: " << hmd_larm_max_l_ << endl;
        cout << "hmd_rarm_max_l_: " << hmd_rarm_max_l_ << endl;
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

        cout << "l_angles_btw_bases(degree); should be near 90degrees: " << l_angle_btw_bases.transpose() << endl;
        cout << "r_angles_btw_bases(degree); should be near 90degrees: " << r_angle_btw_bases.transpose() << endl;

        hmd_lshoulder_pose_init_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_lshoulder_center_pos_ + hmd_chest_pose_init_.translation();
        hmd_lshoulder_pose_init_.linear() = hmd_chest_pose_init_.linear();
        hmd_rshoulder_pose_init_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_rshoulder_center_pos_ + hmd_chest_pose_init_.translation();
        hmd_rshoulder_pose_init_.linear() = hmd_chest_pose_init_.linear();
    }

    //Shoulder Data
    hmd_lshoulder_pose_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_lshoulder_center_pos_ + hmd_chest_pose_.translation();
    hmd_lshoulder_pose_.linear() = hmd_chest_pose_.linear();
    hmd_rshoulder_pose_.translation() = hmd_chest_pose_.linear() * hmd_chest_pose_init_.linear().transpose() * hmd_chest_2_rshoulder_center_pos_ + hmd_chest_pose_.translation();
    hmd_rshoulder_pose_.linear() = hmd_chest_pose_.linear();

    //HMD Velocity
    hmd_head_vel_.segment(0, 3) = (hmd_head_pose_.translation() - hmd_head_pose_pre_.translation()) / dt_;
    hmd_lshoulder_vel_.segment(0, 3) = (hmd_lshoulder_pose_.translation() - hmd_lshoulder_pose_pre_.translation()) / dt_;
    hmd_lupperarm_vel_.segment(0, 3) = (hmd_lupperarm_pose_.translation() - hmd_lupperarm_pose_pre_.translation()) / dt_;
    hmd_lhand_vel_.segment(0, 3) = (hmd_lhand_pose_.translation() - hmd_lhand_pose_pre_.translation()) / dt_;
    hmd_rshoulder_vel_.segment(0, 3) = (hmd_rshoulder_pose_.translation() - hmd_rshoulder_pose_pre_.translation()) / dt_;
    hmd_rupperarm_vel_.segment(0, 3) = (hmd_rupperarm_pose_.translation() - hmd_rupperarm_pose_pre_.translation()) / dt_;
    hmd_rhand_vel_.segment(0, 3) = (hmd_rhand_pose_.translation() - hmd_rhand_pose_pre_.translation()) / dt_;
    hmd_chest_vel_.segment(0, 3) = (hmd_chest_pose_.translation() - hmd_chest_pose_pre_.translation()) / dt_;

    Eigen::AngleAxisd ang_temp_1(hmd_head_pose_.linear() * hmd_head_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_2(hmd_lshoulder_pose_.linear() * hmd_lshoulder_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_3(hmd_lupperarm_pose_.linear() * hmd_lupperarm_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_4(hmd_lhand_pose_.linear() * hmd_lhand_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_5(hmd_rshoulder_pose_.linear() * hmd_rshoulder_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_6(hmd_rupperarm_pose_.linear() * hmd_rupperarm_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_7(hmd_rhand_pose_.linear() * hmd_rhand_pose_pre_.linear().transpose());
    Eigen::AngleAxisd ang_temp_8(hmd_chest_pose_.linear() * hmd_chest_pose_pre_.linear().transpose());

    hmd_head_vel_.segment(3, 3) = ang_temp_1.axis() * ang_temp_1.angle() / dt_;
    hmd_lshoulder_vel_.segment(3, 3) = ang_temp_2.axis() * ang_temp_2.angle() / dt_;
    hmd_lupperarm_vel_.segment(3, 3) = ang_temp_3.axis() * ang_temp_3.angle() / dt_;
    hmd_lhand_vel_.segment(3, 3) = ang_temp_4.axis() * ang_temp_4.angle() / dt_;
    hmd_rshoulder_vel_.segment(3, 3) = ang_temp_5.axis() * ang_temp_5.angle() / dt_;
    hmd_rupperarm_vel_.segment(3, 3) = ang_temp_6.axis() * ang_temp_6.angle() / dt_;
    hmd_rhand_vel_.segment(3, 3) = ang_temp_7.axis() * ang_temp_7.angle() / dt_;
    hmd_chest_vel_.segment(3, 3) = ang_temp_8.axis() * ang_temp_8.angle() / dt_;
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
        cout << "WARNING: Re-Calibration is REQUIRED!" << endl;
    }
    CenterOfShoulder_cali = center_of_cali_plane1 - normal_to_cali_plane.normalized() * k_star;
}

Eigen::Vector3d AvatarController::kinematicFilter(Eigen::Vector3d position_data, Eigen::Vector3d pre_position_data, Eigen::Vector3d reference_position, double boundary, bool &check_boundary)
{
    Eigen::Vector3d result;
    check_boundary = (position_data - reference_position).norm() >= boundary;

    if (check_boundary)
    {
        result = reference_position + boundary * (position_data - reference_position).normalized();
        // result = pre_position_data;
    }
    else
    {
        result = position_data;
    }

    return result;
}

Eigen::Isometry3d AvatarController::velocityFilter(Eigen::Isometry3d data, Eigen::Isometry3d pre_data, Eigen::Vector6d &vel_data, double max_vel, int &cur_iter, int max_iter, bool &check_velocity)
{
    Eigen::Isometry3d result;
    result.setIdentity();
    // check_velocity = ( (vel_data).norm() > max_vel );
    check_velocity = (data.translation() - pre_data.translation()).norm() > max_vel / 130;

    Eigen::AngleAxisd angle_diff(data.linear() * pre_data.linear().transpose());
    bool check_orienation = (angle_diff.angle() > 2 * M_PI / 130);
    double cutoff_f = 1;

    result = data;

    if ((check_velocity))
    {
        // result.translation() = DyrosMath::lpf<3>(data.translation(), pre_data.translation(), 1/dt_, cutoff_f);
        // Eigen::AngleAxisd ang_diff(data.linear()*pre_data.linear().transpose());
        // Eigen::Matrix3d diff_m;
        // diff_m = Eigen::AngleAxisd( DyrosMath::lpf(ang_diff.angle(), 0, 1/dt_, cutoff_f), ang_diff.axis() );
        // result.linear() = diff_m*pre_data.linear();
        // result = pre_data;
        result.translation() = (data.translation() - pre_data.translation()).normalized() * max_vel / 130 + pre_data.translation();

        cur_iter++;
        vel_data.setZero();
        check_velocity = true;
    }

    if ((check_orienation))
    {
        Matrix3d rot;
        rot = AngleAxisd(1.5 * M_PI / 130, angle_diff.axis());
        result.linear() = rot * pre_data.linear();
        check_velocity = true;
    }

    return result;
}

void AvatarController::abruptMotionFilter()
{
    int maximum_data_cut_num = 200;
    bool verbose = 0;
    bool fast_head_move_flag = false;
    bool fast_lupperarm_move_flag = false;
    bool fast_lhand_move_flag = false;
    bool fast_rupperarm_move_flag = false;
    bool fast_rhand_move_flag = false;
    bool fast_chest_move_flag = false;

    double head_max_vel = 25;
    double lupperarm_max_vel = 25;
    double lhand_max_vel = 25;
    double rupperarm_max_vel = 25;
    double rhand_max_vel = 25;
    double chest_max_vel = 25;

    Vector6d hmd_lupperarm_angvel_only;
    Vector6d hmd_rupperarm_angvel_only;
    hmd_lupperarm_angvel_only.setZero();
    hmd_lupperarm_angvel_only.segment(3, 3) = hmd_lupperarm_vel_.segment(3, 3);
    hmd_rupperarm_angvel_only.setZero();
    hmd_rupperarm_angvel_only.segment(3, 3) = hmd_rupperarm_vel_.segment(3, 3);

    hmd_head_pose_ = velocityFilter(hmd_head_pose_, hmd_head_pose_pre_, hmd_head_vel_, head_max_vel, hmd_head_abrupt_motion_count_, maximum_data_cut_num, fast_head_move_flag);
    // hmd_lupperarm_pose_  = velocityFilter( hmd_lupperarm_pose_, hmd_lupperarm_pose_pre_, hmd_lupperarm_angvel_only, lupperarm_max_vel, hmd_lupperarm_abrupt_motion_count_, maximum_data_cut_num, fast_lupperarm_move_flag);
    hmd_lupperarm_pose_ = velocityFilter(hmd_lupperarm_pose_, hmd_lupperarm_pose_pre_, hmd_lupperarm_vel_, lupperarm_max_vel, hmd_lupperarm_abrupt_motion_count_, maximum_data_cut_num, fast_lupperarm_move_flag);
    hmd_lhand_pose_ = velocityFilter(hmd_lhand_pose_, hmd_lhand_pose_pre_, hmd_lhand_vel_, lhand_max_vel, hmd_lhand_abrupt_motion_count_, maximum_data_cut_num, fast_lhand_move_flag);
    // hmd_rupperarm_pose_  = velocityFilter( hmd_rupperarm_pose_, hmd_rupperarm_pose_pre_, hmd_rupperarm_angvel_only, rupperarm_max_vel, hmd_rupperarm_abrupt_motion_count_, maximum_data_cut_num, fast_rupperarm_move_flag);
    hmd_rupperarm_pose_ = velocityFilter(hmd_rupperarm_pose_, hmd_rupperarm_pose_pre_, hmd_rupperarm_vel_, rupperarm_max_vel, hmd_rupperarm_abrupt_motion_count_, maximum_data_cut_num, fast_rupperarm_move_flag);
    hmd_rhand_pose_ = velocityFilter(hmd_rhand_pose_, hmd_rhand_pose_pre_, hmd_rhand_vel_, rhand_max_vel, hmd_rhand_abrupt_motion_count_, maximum_data_cut_num, fast_rhand_move_flag);
    hmd_chest_pose_ = velocityFilter(hmd_chest_pose_, hmd_chest_pose_pre_, hmd_chest_vel_, chest_max_vel, hmd_chest_abrupt_motion_count_, maximum_data_cut_num, fast_chest_move_flag);

    if (verbose)
    {
        if (fast_head_move_flag)
        {
            cout << "abrupt movement is detected at the head tracker! (" << hmd_head_abrupt_motion_count_ << ")" << endl;
        }

        if (fast_lupperarm_move_flag)
        {
            cout << "abrupt movement is detected at the lupperarm tracker! (" << hmd_lupperarm_abrupt_motion_count_ << ")" << endl;
        }

        if (fast_lhand_move_flag)
        {
            cout << "abrupt movement is detected at the lhand tracker! (" << hmd_lhand_abrupt_motion_count_ << ")" << endl;
        }

        if (fast_rupperarm_move_flag)
        {
            cout << "abrupt movement is detected at the rupperarm tracker! (" << hmd_rupperarm_abrupt_motion_count_ << ")" << endl;
        }

        if (fast_rhand_move_flag)
        {
            cout << "abrupt movement is detected at the rhand tracker! (" << hmd_rhand_abrupt_motion_count_ << ")" << endl;
        }

        if (fast_chest_move_flag)
        {
            cout << "abrupt movement is detected at the chest tracker! (" << hmd_chest_abrupt_motion_count_ << ")" << endl;
        }
    }
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

        upperbody_mode_recieved_ = false;

        hmd_init_pose_calibration_ = true;

        // hmd_shoulder_width_ = (hmd_lupperarm_pose_.translation() - hmd_rupperarm_pose_.translation()).norm();
    }

    // abruptMotionFilter();
    hmdRawDataProcessing();

    if (upper_body_mode_ == 8)
    {
        /////Absolute hand position mapping //////
        Vector3d hand_offset;
        hand_offset << 0.0, 0.0, 0.15;
        // hand_offset << 0.15, 0, 0.15;
        master_lhand_pose_raw_.translation() = hmd_lhand_pose_.translation() + hand_offset;
        master_rhand_pose_raw_.translation() = hmd_rhand_pose_.translation() + hand_offset;
        ///////////////////////////////////////////
    }
    else if (upper_body_mode_ == 9)
    {
        ///////Propotional hand position mapping////////////
        Vector3d hand_offset;
        hand_offset << 0.0, 0.0, 0.15;
        // hand_offset << 0.15, 0, 0.15;
        master_lhand_pose_raw_.translation() = 0.9 * robot_arm_max_l_ / ((hmd_larm_max_l_ + hmd_rarm_max_l_) / 2) * hmd_lhand_pose_.translation() + hand_offset;
        master_rhand_pose_raw_.translation() = 0.9 * robot_arm_max_l_ / ((hmd_larm_max_l_ + hmd_rarm_max_l_) / 2) * hmd_rhand_pose_.translation() + hand_offset;
    }

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

    double fc_filter = 3.0; //hz

    if (current_time_ <= upperbody_command_time_ + 5)
    {
        for (int i = 0; i < 3; i++)
        {
            master_lhand_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lhand_transform_pre_desired_from_.translation()(i), 0, 0, master_lhand_pose_raw_.translation()(i), 0, 0)(0);
            master_rhand_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rhand_transform_pre_desired_from_.translation()(i), 0, 0, master_rhand_pose_raw_.translation()(i), 0, 0)(0);

            master_lelbow_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lupperarm_transform_pre_desired_from_.translation()(i), 0, 0, master_lelbow_pose_raw_.translation()(i), 0, 0)(0);
            master_relbow_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rupperarm_transform_pre_desired_from_.translation()(i), 0, 0, master_relbow_pose_raw_.translation()(i), 0, 0)(0);

            master_lshoulder_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lacromion_transform_pre_desired_from_.translation()(i), 0, 0, master_lshoulder_pose_raw_.translation()(i), 0, 0)(0);
            master_rshoulder_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, racromion_transform_pre_desired_from_.translation()(i), 0, 0, master_rshoulder_pose_raw_.translation()(i), 0, 0)(0);

            master_relative_lhand_pos_raw_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lhand_transform_pre_desired_from_.translation()(i) - rhand_transform_pre_desired_from_.translation()(i), 0, 0, master_relative_lhand_pos_raw_(i), 0, 0)(0);
            master_relative_rhand_pos_raw_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rhand_transform_pre_desired_from_.translation()(i) - lhand_transform_pre_desired_from_.translation()(i), 0, 0, master_relative_rhand_pos_raw_(i), 0, 0)(0);
        }

        master_lhand_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lhand_transform_pre_desired_from_.linear(), master_lhand_pose_raw_.linear());
        master_rhand_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rhand_transform_pre_desired_from_.linear(), master_rhand_pose_raw_.linear());
        master_lelbow_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_raw_.linear());
        master_relbow_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_raw_.linear());
        master_lshoulder_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_raw_.linear());
        master_rshoulder_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_raw_.linear());
        master_head_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, head_transform_pre_desired_from_.linear(), master_head_pose_raw_.linear());
        master_upperbody_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_raw_.linear());
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
    // To Do List: omit the relative shoulder basis

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
        qpRetargeting_1(); //calc lhand_mapping_vector_, rhand_mapping_vector_ //1025
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
    else //transition
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

    if (upper_body_mode_ == 10)
    {
        hmd2robot_lhand_pos_mapping_ *= hmd_larm_max_l_ / robot_arm_max_l_;
        hmd2robot_rhand_pos_mapping_ *= hmd_rarm_max_l_ / robot_arm_max_l_;
    }

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

    if ((hmd_init_pose_calibration_ == true) && (hmd_check_pose_calibration_[3] == true)) //still cali로 옮기기
    {
        cout << "Motion Retargeting Parameter Initialized" << endl;
        hmd_init_pose_calibration_ = false;
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
    master_lhand_pose_raw_.linear() = hmd_lhand_pose_.linear() * DyrosMath::rotateWithZ(M_PI / 2); //absolute orientation

    master_rhand_pose_raw_.translation() = rarmbase_transform_pre_desired_from_.translation() + upperbody_transform_pre_desired_from_.linear() * (robot_init_rshoulder_pos + hmd2robot_rhand_pos_mapping_);
    // master_rhand_pose_raw_.linear() = hmd_rhand_pose_.linear()*hmd_rhand_pose_init_.linear().transpose()*robot_rhand_ori_init;	//relative orientation
    master_rhand_pose_raw_.linear() = hmd_rhand_pose_.linear() * DyrosMath::rotateWithZ(-M_PI / 2); //absolute orientation

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

void AvatarController::getLegIK()
{
    //real
    // double kp_com = 0.7;
    // double kp_zmp = 0.0;
    //sim
    double kp_com = 0.7;
    double kp_zmp = 0.0;
    Vector12d q_leg_desired;
    q_leg_desired.setZero();

    // lfoot_transform_desired = lfoot_transform_start_from_global_;
    // rfoot_transform_desired = rfoot_transform_start_from_global_;

    // if(foot_swing_trigger_ == true)
    // {
    double target_pelv_yaw = yaw_angular_vel_ * turning_duration_ / 2;
    double pelv_yaw_current = DyrosMath::cubic(turning_phase_, 0, 1, pelv_rpy_init_from_support_(2), target_pelv_yaw, 0, 0);

    pelv_transform_desired_.linear().setIdentity();
    // pelv_transform_desired_.linear() = DyrosMath::rotationCubic(walking_phase_, 0, dsp_ratio_, pelv_transform_desired_last_.linear(),  DyrosMath::rotateWithZ(pelv_yaw_current));
    pelv_transform_desired_.translation() = pelv_transform_current_from_support_.translation() + kp_com * (com_pos_desired_from_support_ - com_pos_current_from_support_) - kp_zmp * (middle_of_both_foot_ - zmp_measured_);
    // pelv_transform_desired.translation().setZero();
    pelv_transform_desired_.translation()(2) = DyrosMath::QuinticSpline(walking_phase_, 0, dsp_ratio_, pelv_transform_desired_last_.translation()(2), 0, 0, pelv_transform_start_from_support_.translation()(2), 0, 0)(0);
    // pelv_transform_desired.translation()(2) = com_target_height_;

    Vector3d zeros;
    zeros.setZero();

    if (foot_contact_ == -1) //right support foot
    {
        lfoot_transform_desired_.translation() = DyrosMath::cubicVector<3>(walking_phase_, dsp_ratio_ / 3, dsp_ratio_ * 2 / 3, lfoot_transform_desired_last_.translation(), swing_foot_pos_trajectory_from_support_, zeros, zeros);
        lfoot_transform_desired_.linear() = DyrosMath::rotationCubic(walking_phase_, dsp_ratio_ / 3, dsp_ratio_ * 2 / 3, lfoot_transform_desired_last_.linear(), swing_foot_rot_trajectory_from_support_);
        // lfoot_transform_desired_.linear() = swing_foot_rot_trajectory_from_support_;

        rfoot_transform_desired_.translation() = DyrosMath::cubicVector<3>(walking_phase_, dsp_ratio_ / 3, dsp_ratio_ * 2 / 3, rfoot_transform_desired_last_.translation(), support_foot_transform_current_from_support_.translation(), zeros, zeros);
        // rfoot_transform_desired_.linear() = DyrosMath::rotationCubic(walking_phase_, dsp_ratio_/3, dsp_ratio_*2/3, rfoot_transform_desired_last_.linear(),  Eigen::Matrix3d::Identity());
        // rfoot_transform_desired_.linear() = support_foot_transform_current_from_support_.linear();
        rfoot_transform_desired_.linear().setIdentity();

        // rfoot_transform_desired_ = support_foot_transform_current_from_support_; //support foot
    }
    else
    {
        rfoot_transform_desired_.translation() = DyrosMath::cubicVector<3>(walking_phase_, dsp_ratio_ / 3, dsp_ratio_ * 2 / 3, rfoot_transform_desired_last_.translation(), swing_foot_pos_trajectory_from_support_, zeros, zeros);
        rfoot_transform_desired_.linear() = DyrosMath::rotationCubic(walking_phase_, dsp_ratio_ / 3, dsp_ratio_ * 2 / 3, rfoot_transform_desired_last_.linear(), swing_foot_rot_trajectory_from_support_);
        // rfoot_transform_desired_.linear() = swing_foot_rot_trajectory_from_support_;

        lfoot_transform_desired_.translation() = DyrosMath::cubicVector<3>(walking_phase_, dsp_ratio_ / 3, dsp_ratio_ * 2 / 3, lfoot_transform_desired_last_.translation(), support_foot_transform_current_from_support_.translation(), zeros, zeros);
        // lfoot_transform_desired_.linear() = DyrosMath::rotationCubic(walking_phase_, dsp_ratio_/3, dsp_ratio_*2/3, lfoot_transform_desired_last_.linear(),  Eigen::Matrix3d::Identity());
        lfoot_transform_desired_.linear().setIdentity();
        // lfoot_transform_desired_.linear() = support_foot_transform_current_from_support_.linear();
    }

    computeIk(pelv_transform_desired_, lfoot_transform_desired_, rfoot_transform_desired_, q_leg_desired);

    for (int i = 0; i < 12; i++)
    {
        desired_q_(i) = q_leg_desired(i);
        pd_control_mask_(i) = 1;
    }

    if (int(current_time_ * 10000) % 1000 == 0)
    {
        // cout<<"desired_q_: \n"<<desired_q_.segment(0, 12)<<endl;
        // cout<<"com_pos_current_from_support_: "<<com_pos_current_from_support_.transpose()<<endl;
        // cout<<"pelv_transform_desired.translation(): \n"<<pelv_transform_desired.translation()<<endl;
        // cout<<"lfoot_transform_desired.translation(): \n"<< lfoot_transform_desired.translation()<<endl;
        // cout<<"rfoot_transform_desired.translation(): \n"<< rfoot_transform_desired.translation()<<endl;
        // cout<<"pelv_transform_current_from_support_.translation(): \n"<<pelv_transform_current_from_support_.translation()<<endl;
        // cout<<"pelv_transform_init_from_support_.translation(): \n"<<pelv_transform_init_from_support_.translation()<<endl;
        // cout<<"lfoot_transform_current_from_support_.translation(): \n"<<lfoot_transform_current_from_support_.translation()<<endl;
        // cout<<"rfoot_transform_current_from_support_.translation(): \n"<<rfoot_transform_current_from_support_.translation()<<endl;

        // cout<<"ref_zmp_(0, 0): "<<ref_zmp_(0, 0)<<endl;
        // cout<<"ref_zmp_(0, 1): "<<ref_zmp_(0, 1)<<endl;
    }
}

void AvatarController::getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired)
{
    Eigen::Vector2d d;
    Eigen::Vector2d d_prime;
    double d_temp_x;
    double d_temp_y;
    double ipm_calc_end_phsse = 0.9;

    if (walking_speed_ == 0)
    {
        // alpha_y_ = 0.15;
        alpha_x_ = 0.0;
    }
    else
    {
        alpha_x_ = alpha_x_command_;
        alpha_y_ = alpha_y_command_;
    }

    if (foot_swing_trigger_ == true)
    {
        // x axis
        d_temp_x = (com_pos_current(2) - support_foot_transform_current_.translation()(2)) / GRAVITY + (com_vel_current(0) / (2 * GRAVITY)) * (com_vel_current(0) / (2 * GRAVITY));

        if (d_temp_x < 0)
            d_temp_x = 0;

        d_temp_x = sqrt(d_temp_x);

        d(0) = com_vel_current(0) * d_temp_x;

        // y axis
        d_temp_y = (com_pos_current(2) - support_foot_transform_current_.translation()(2)) / GRAVITY + (com_vel_current(1) / (2 * GRAVITY)) * (com_vel_current(1) / (2 * GRAVITY));

        if (d_temp_y < 0)
            d_temp_y = 0;

        d_temp_y = sqrt(d_temp_y);

        d(1) = com_vel_current(1) * d_temp_y;

        if (com_vel_current(0) * walking_speed_ < 0)
        {
            alpha_x_ = 0.05;
        }

        d_prime(0) = d(0) - alpha_x_ * com_vel_desired(0);
        d_prime(1) = d(1) - alpha_y_ * step_width_ / walking_duration_ * foot_contact_;
        // d_prime(1) = d(1) - alpha_y * com_vel_desired(1) * foot_contact_;

        if (walking_phase_ < ipm_calc_end_phsse)
        {
            target_foot_landing_from_pelv_ = com_pos_current.segment<2>(0) + d_prime;
        }

        Vector2d landing_point;
        Vector2d desired_landing_point;
        double swingfoot_target_weight;

        // swingfoot_target_weight = (walking_phase_ - 0.1)/0.5;
        swingfoot_target_weight = 1;
        swingfoot_target_weight = DyrosMath::minmax_cut(swingfoot_target_weight, 0.0, 1.0);

        desired_landing_point(0) = walking_speed_ * walking_duration_ + support_foot_transform_current_.translation()(0);
        desired_landing_point(1) = step_width_ * (-foot_contact_) + support_foot_transform_current_.translation()(1);

        landing_point = swingfoot_target_weight * target_foot_landing_from_pelv_ + (1 - swingfoot_target_weight) * desired_landing_point;

        double dsp_coeff = 2;
        if (walking_phase_ < dsp_coeff * switching_phase_duration_)
        {
            swing_foot_pos_trajectory_from_global_.segment(0, 2) = support_foot_transform_current_.translation().segment<2>(0) + swing_foot_transform_init_.translation().segment<2>(0) - support_foot_transform_init_.translation().segment<2>(0);
            swing_foot_vel_trajectory_from_global_.segment(0, 2).setZero();
            swing_foot_vel_trajectory_from_global_.segment(3, 3).setZero();
            // swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
            swing_foot_rot_trajectory_from_global_.setIdentity();
        }
        else
        {
            double ssp = (walking_phase_ - dsp_coeff * switching_phase_duration_) / (ipm_calc_end_phsse - dsp_coeff * switching_phase_duration_);
            ssp = DyrosMath::minmax_cut(ssp, 0.0, 1.0);

            swing_foot_pos_trajectory_from_global_.segment(0, 2) = support_foot_transform_current_.translation().segment<2>(0) +
                                                                   (1 - ssp) * (swing_foot_transform_init_.translation().segment<2>(0) - support_foot_transform_init_.translation().segment<2>(0)) +
                                                                   (ssp) * (target_foot_landing_from_pelv_ - support_foot_transform_current_.translation().segment<2>(0));

            swing_foot_vel_trajectory_from_global_.segment(0, 2) = (target_foot_landing_from_pelv_ - swing_foot_transform_init_.translation().segment<2>(0)) / (ipm_calc_end_phsse - dsp_coeff * switching_phase_duration_);
            swing_foot_rot_trajectory_from_global_.setIdentity();
        }
    }
    else
    {
        swing_foot_pos_trajectory_from_global_ = support_foot_transform_current_.translation() + swing_foot_transform_init_.translation() - support_foot_transform_init_.translation();
        swing_foot_vel_trajectory_from_global_.setZero();
        // swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
        swing_foot_rot_trajectory_from_global_.setIdentity();
    }

    if (foot_contact_ == 1) //left support
    {
        swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), -0.6, 1.2);
        swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), -0.7, lfoot_transform_current_from_global_.translation()(1) - 0.21);
    }
    else if (foot_contact_ == -1) // right support
    {
        swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), -0.6, 1.2);
        swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), rfoot_transform_current_from_global_.translation()(1) + 0.21, 0.7);
    }
}

void AvatarController::getSwingFootXYZTrajectory()
{

    double target_swing_yaw = yaw_angular_vel_ * turning_duration_;
    double swing_foot_height_zero;

    if (foot_swing_trigger_ == true)
    {
        if (walking_phase_ < dsp_ratio_)
        {
            swing_foot_pos_trajectory_from_support_ = swing_foot_transform_init_from_support_.translation();

            swing_foot_pos_trajectory_from_support_(2) =
                DyrosMath::QuinticSpline(walking_phase_, 0, dsp_ratio_ / 2,
                                         swing_foot_transform_init_from_support_.translation()(2), 0, 0,
                                         0, 0, 0)(0);

            swing_foot_vel_trajectory_from_support_.setZero();
            // swing_foot_rot_trajectory_from_support_ = swing_foot_transform_init_from_support_.linear();
            // swing_foot_rot_trajectory_from_support_.setIdentity();
            swing_foot_rot_trajectory_from_support_ = DyrosMath::rotateWithZ(swing_foot_rpy_init_from_support_(2));
        }
        else
        {
            Eigen::Vector3d target_landing_pos;

            target_landing_pos.setZero();
            target_landing_pos(0) = walking_speed_ * walking_duration_;
            target_landing_pos(1) = -foot_contact_ * step_width_;

            swing_foot_pos_trajectory_from_support_(0) =
                DyrosMath::QuinticSpline(walking_phase_, dsp_ratio_, 1,
                                         swing_foot_transform_init_from_support_.translation()(0), 0, 0,
                                         target_landing_pos(0), 0, 0)(0);

            swing_foot_vel_trajectory_from_support_(0) =
                DyrosMath::QuinticSpline(walking_phase_, dsp_ratio_, 1,
                                         swing_foot_transform_init_from_support_.translation()(0), 0, 0,
                                         target_landing_pos(0), 0, 0)(1);

            swing_foot_pos_trajectory_from_support_(1) =
                DyrosMath::QuinticSpline(walking_phase_, dsp_ratio_, 1,
                                         swing_foot_transform_init_from_support_.translation()(1), 0, 0,
                                         target_landing_pos(1), 0, 0)(0);

            swing_foot_vel_trajectory_from_support_(1) =
                DyrosMath::QuinticSpline(walking_phase_, dsp_ratio_, 1,
                                         swing_foot_transform_init_from_support_.translation()(1), 0, 0,
                                         target_landing_pos(1), 0, 0)(1);

            if (walking_phase_ < swingfoot_highest_time_)
            {
                swing_foot_pos_trajectory_from_support_(2) =
                    DyrosMath::QuinticSpline(walking_phase_, dsp_ratio_, swingfoot_highest_time_,
                                             0, 0, 0,
                                             swing_foot_height_, 0, 0)(0);

                swing_foot_vel_trajectory_from_support_(2) =
                    DyrosMath::QuinticSpline(walking_phase_, dsp_ratio_, swingfoot_highest_time_,
                                             0, 0, 0,
                                             swing_foot_height_, 0, 0)(1);
            }
            else
            {
                swing_foot_pos_trajectory_from_support_(2) =
                    DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1,
                                             swing_foot_height_, 0, 0,
                                             0, 0, 0)(0);

                swing_foot_vel_trajectory_from_support_(2) =
                    DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1,
                                             swing_foot_height_, 0, 0,
                                             0, 0, 0)(1);
            }

            swing_foot_vel_trajectory_from_support_(5) = DyrosMath::cubicDot(turning_phase_, 0, 1, swing_foot_rpy_init_from_support_(2), target_swing_yaw, 0, 0, 1 / dt_);
            double swing_yaw_current = DyrosMath::cubic(turning_phase_, 0, 1, swing_foot_rpy_init_from_support_(2), target_swing_yaw, 0, 0);

            swing_foot_rot_trajectory_from_support_ = DyrosMath::rotateWithZ(swing_yaw_current);
            // swing_foot_rot_trajectory_from_support_.setIdentity();
        }
    }
    else
    {
        swing_foot_pos_trajectory_from_support_ = swing_foot_transform_init_from_support_.translation();
        swing_foot_vel_trajectory_from_support_.setZero();
        swing_foot_rot_trajectory_from_support_ = DyrosMath::rotateWithZ(swing_foot_rpy_init_from_support_(2));
    }

    if (foot_contact_ == 1) //left support
    {
        swing_foot_pos_trajectory_from_support_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_support_(0), -0.3, 0.5);
        swing_foot_pos_trajectory_from_support_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_support_(1), -0.7, -0.18);
    }
    else if (foot_contact_ == -1) // right support
    {
        swing_foot_pos_trajectory_from_support_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_support_(0), -0.3, 0.5);
        swing_foot_pos_trajectory_from_support_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_support_(1), 0.18, 0.7);
    }
}

Eigen::VectorQd AvatarController::comVelocityControlCompute()
{
    Eigen::VectorQd torque;

    Eigen::VectorXd f_star;
    Eigen::MatrixXd J_task;
    VectorQd torque_r_vel_tun;
    VectorQd torque_l_vel_tun;
    const int task_dof = 6;

    Vector3d alpha_unit;
    Vector3d
        _unit;

    double f_star_mag_alpha;
    double f_star_mag_beta;
    double f_star_mag_z;

    double f_star_mag_alpha_lfoot;
    double f_star_mag_beta_lfoot;
    double f_star_mag_z_lfoot;
    double f_star_mag_alpha_rfoot;
    double f_star_mag_beta_rfoot;
    double f_star_mag_z_rfoot;

    double d_l;     //distance from CoM_alpha to lfoot contact point
    double d_r;     //distance from CoM_alpha to rfoot contact point
    double w_l;     //weighting factor to how the CoM is closer to the lfoot
    double w_r;     //weighting factor to how the CoM is closer to the rfoot
    double h_l;     //the height of CoM with respect to the left foot
    double h_r;     //the height of CoM with respect to the right foot
    double gamma_l; //tangent value of the angle from CoM to the lfoot;
    double gamma_r; //tangent value of the angle from CoM to the rfoot;

    double gamma_l_min; //minimum tangent value of the angle from CoM to the lfoot;
    double gamma_l_max; //maximum tangent value of the angle from CoM to the lfoot;
    double gamma_r_min; //minimum tangent value of the angle from CoM to the rfoot;
    double gamma_r_max; //maximum tangent value of the angle from CoM to the rfoot;

    torque.setZero();
    phi_pelv_.setZero();
    torque_pelv_.setZero();
    J_task.setZero(task_dof, MODEL_DOF_VIRTUAL);
    f_star.setZero(task_dof);

    // WBC::SetContact(rd_, 0, 0); // for graviti torque calc
    // if(foot_swing_trigger_ == true)
    // {
    //     if( (foot_contact_ == 1) )
    //     {
    //         wc.SetContact(rd_, 1, 0);

    //     }
    //     else if ( (foot_contact_ == -1) )
    //     {
    //         wc.SetContact(rd_, 0, 1);
    //     }
    // }
    // wbc_.SetContact(rd_, 1, 1);
    torque_g_.setZero();
    torque_g_ = WBC::GravityCompensationTorque(rd_);

    ////////////// Set f_start  ////////////////

    // com_pos_desired_(1) = com_pos_init_(0) + sin(2*M_PI/8*current_time_-tc.command_time);
    // com_vel_desired_(1) = M_PI/2*cos(2*M_PI/8*current_time_-tc.command_time);
    com_pos_error_ = com_pos_desired_ - com_pos_current_;
    com_vel_error_ = com_vel_desired_ - com_vel_current_;

    f_star(0) = kd_compos_(0, 0) * (com_vel_error_(0)) + kp_compos_(0, 0) * (com_pos_error_(0)) + com_acc_desired_(0); //X axis PD control
    f_star(1) = kd_compos_(1, 1) * (com_vel_error_(1)) + kp_compos_(1, 1) * (com_pos_error_(1)) + com_acc_desired_(1); //Y axis PD control
    f_star(2) = kd_compos_(2, 2) * (com_vel_error_(2)) + kp_compos_(2, 2) * (com_pos_error_(2)) + com_acc_desired_(2);

    f_star(0) *= rd_.link_[COM_id].mass; //cancle out mass effect
    f_star(1) *= rd_.link_[COM_id].mass;
    f_star(2) *= rd_.link_[COM_id].mass;

    // f_star.segment(0, 3).setZero();

    phi_pelv_ = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
    torque_pelv_ = kp_pelv_ori_ * phi_pelv_ - kd_pelv_ori_ * pelv_angvel_current_;
    torque_pelv_(2) = 0;
    f_star.segment(3, 3) = torque_pelv_ * 3;
    /////////////////////////////////////////////

    /////////////////////JACOBIAN///////////////////////////////////////
    lfoot_to_com_jac_from_global_.block(0, 6, 6, 1).setZero();  // 	left yaw
    lfoot_to_com_jac_from_global_.block(0, 12, 6, 6).setZero(); //	right leg
    lfoot_to_com_jac_from_global_.block(0, 21, 6, 8).setZero(); //	left arm
    lfoot_to_com_jac_from_global_.block(0, 29, 6, 2).setZero(); //	head
    lfoot_to_com_jac_from_global_.block(0, 31, 6, 8).setZero(); // 	right arm

    lfoot_to_com_jac_from_global_.block(3, 9, 3, 3).setZero(); // 	left leg roational component: knee pitch, ankle pitch, ankle roll

    rfoot_to_com_jac_from_global_.block(0, 6, 6, 6).setZero();  //	left leg
    rfoot_to_com_jac_from_global_.block(0, 12, 6, 1).setZero(); // 	right yaw
    rfoot_to_com_jac_from_global_.block(0, 21, 6, 8).setZero(); //	left arm
    rfoot_to_com_jac_from_global_.block(0, 29, 6, 2).setZero(); //	head
    rfoot_to_com_jac_from_global_.block(0, 31, 6, 8).setZero(); //	right arm

    rfoot_to_com_jac_from_global_.block(3, 15, 3, 3).setZero(); // 	right leg roational component: knee pitch, ankle pitch, ankle roll

    // std::cout<<"rfoot_to_com_jac_from_global_.block(3, 18, 3, 3): \n"<<rfoot_to_com_jac_from_global_.block(3, 18, 3, 3)<<std::endl;

    if (foot_swing_trigger_ == true)
    {
        if (foot_contact_ == 1) //left support
        {
            h_l = (com_pos_current_ - lfoot_transform_current_from_global_.translation())(2) + 0.12;
            h_l = DyrosMath::minmax_cut(h_l, 0.1, 1.2);

            // x axis
            gamma_l_min = (com_pos_current_(0) - (lfoot_transform_current_from_global_.translation()(0) + 0.20)) / h_l;
            gamma_l_max = (com_pos_current_(0) - (lfoot_transform_current_from_global_.translation()(0) - 0.15)) / h_l;

            f_star(0) = DyrosMath::minmax_cut(f_star(0), f_star(2) * gamma_l_min, f_star(2) * gamma_l_max);

            // y axis
            gamma_l_min = (com_pos_current_(1) - (lfoot_transform_current_from_global_.translation()(1) + 0.07)) / h_l;
            gamma_l_max = (com_pos_current_(1) - (lfoot_transform_current_from_global_.translation()(1) - 0.07)) / h_l;

            f_star(1) = DyrosMath::minmax_cut(f_star(1), f_star(2) * gamma_l_min, f_star(2) * gamma_l_max);

            f_star_l_ = 0.5 * f_star + 0.5 * f_star_l_pre_; //lpf
            f_star_r_ = 0.5 * f_star_r_pre_;
        }
        else if (foot_contact_ == -1) //right support
        {
            h_r = (com_pos_current_ - rfoot_transform_current_from_global_.translation())(2) + 0.12;
            h_r = DyrosMath::minmax_cut(h_r, 0.1, 1.2);

            // x axis
            gamma_r_min = (com_pos_current_(0) - (rfoot_transform_current_from_global_.translation()(0) + 0.20)) / h_r;
            gamma_r_max = (com_pos_current_(0) - (rfoot_transform_current_from_global_.translation()(0) - 0.15)) / h_r;

            f_star(0) = DyrosMath::minmax_cut(f_star(0), f_star(2) * gamma_r_min, f_star(2) * gamma_r_max);

            // y axis
            gamma_r_min = (com_pos_current_(1) - rfoot_transform_current_from_global_.translation()(1) - 0.07) / h_r;
            gamma_r_max = (com_pos_current_(1) - rfoot_transform_current_from_global_.translation()(1) + 0.07) / h_r;

            f_star(1) = DyrosMath::minmax_cut(f_star(1), f_star(2) * gamma_r_min, f_star(2) * gamma_r_max);

            f_star_r_ = 0.5 * f_star + 0.5 * f_star_r_pre_; //lpf
            f_star_l_ = 0.5 * f_star_l_pre_;
        }
    }
    else
    {

        alpha_unit = (lfoot_transform_current_from_global_.translation() - rfoot_transform_current_from_global_.translation());
        alpha_unit(2) = 0;
        alpha_unit.normalize();

        Eigen::Vector3d beta_unit = DyrosMath::rotateWithZ(-M_PI / 2) * alpha_unit;

        d_l = (com_pos_current_ - lfoot_transform_current_from_global_.translation()).transpose() * alpha_unit;
        d_l = DyrosMath::minmax_cut(d_l, -1.0, 0.0);
        d_l = abs(d_l);
        d_r = (com_pos_current_ - rfoot_transform_current_from_global_.translation()).transpose() * alpha_unit;
        d_r = DyrosMath::minmax_cut(d_r, 0.0, 1.0);
        d_r = abs(d_r);

        w_l = d_r / (d_l + d_r);
        w_r = d_l / (d_l + d_r);

        h_l = (com_pos_current_ - lfoot_transform_current_from_global_.translation())(2) + 0.12;
        h_r = (com_pos_current_ - rfoot_transform_current_from_global_.translation())(2) + 0.12;
        h_l = DyrosMath::minmax_cut(h_l, 0.1, 1.2);
        h_r = DyrosMath::minmax_cut(h_r, 0.1, 1.2);

        gamma_l = d_l / h_l;
        gamma_r = d_r / h_r;

        f_star_mag_alpha = (f_star.segment(0, 3).transpose() * alpha_unit);
        f_star_mag_beta = (f_star.segment(0, 3).transpose() * beta_unit);
        f_star_mag_z = f_star(2);

        f_star_mag_alpha_lfoot = (gamma_l * f_star_mag_alpha - gamma_l * gamma_r * f_star_mag_z) / (gamma_l + gamma_r);
        f_star_mag_z_lfoot = (-f_star_mag_alpha + gamma_r * f_star_mag_z) / (gamma_l + gamma_r);

        f_star_mag_alpha_rfoot = (gamma_r * f_star_mag_alpha + gamma_l * gamma_r * f_star_mag_z) / (gamma_l + gamma_r);
        f_star_mag_z_rfoot = (f_star_mag_alpha + gamma_l * f_star_mag_z) / (gamma_l + gamma_r);

        f_star_mag_beta_lfoot = w_l * f_star_mag_beta;
        f_star_mag_beta_rfoot = w_r * f_star_mag_beta;

        f_star_l_.segment(0, 3) = f_star_mag_alpha_lfoot * alpha_unit + f_star_mag_beta_lfoot * beta_unit;
        f_star_l_(2) += f_star_mag_z_lfoot;
        f_star_l_.segment(3, 3) = w_l * f_star.segment(3, 3);

        f_star_r_.segment(0, 3) = f_star_mag_alpha_rfoot * alpha_unit + f_star_mag_beta_rfoot * beta_unit;
        f_star_r_(2) += f_star_mag_z_rfoot;
        f_star_r_.segment(3, 3) = w_r * f_star.segment(3, 3);

        f_star_l_ = 0.5 * f_star_l_ + 0.5 * f_star_l_pre_;
        f_star_r_ = 0.5 * f_star_r_ + 0.5 * f_star_r_pre_;
    }

    torque_l_vel_tun = (lfoot_to_com_jac_from_global_.transpose() * (f_star_l_)).segment(6, MODEL_DOF);
    torque_r_vel_tun = (rfoot_to_com_jac_from_global_.transpose() * (f_star_r_)).segment(6, MODEL_DOF);

    ////////////TORQUE CLACULATION/////////////////////////////////
    double lfoot_torque_g_switch;
    double rfoot_torque_g_switch;
    double lfoot_task_torque_switch;
    double rfoot_task_torque_switch;

    if (foot_swing_trigger_ == true)
    {
        double vel_tune_switching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

        if (foot_contact_ == 1) //left support
        {
            if (first_step_trigger_ == true)
            {
                lfoot_task_torque_switch = 1;
                rfoot_task_torque_switch = 1 - vel_tune_switching;

                lfoot_torque_g_switch = 0;
                rfoot_torque_g_switch = vel_tune_switching;
            }
            else
            {
                lfoot_task_torque_switch = vel_tune_switching;
                rfoot_task_torque_switch = 1 - vel_tune_switching;

                lfoot_torque_g_switch = 1 - vel_tune_switching;
                rfoot_torque_g_switch = vel_tune_switching;
            }
        }
        else if (foot_contact_ == -1)
        {
            if (first_step_trigger_ == true)
            {
                lfoot_task_torque_switch = 1 - vel_tune_switching;
                rfoot_task_torque_switch = 1;

                lfoot_torque_g_switch = vel_tune_switching;
                rfoot_torque_g_switch = 0;
            }
            else
            {
                lfoot_task_torque_switch = 1 - vel_tune_switching;
                rfoot_task_torque_switch = vel_tune_switching;

                lfoot_torque_g_switch = vel_tune_switching;
                rfoot_torque_g_switch = 1 - vel_tune_switching;
            }
        }
    }
    else
    {
        if (stance_start_time_ == program_start_time_)
        {

            lfoot_task_torque_switch = first_torque_supplier_;
            rfoot_task_torque_switch = first_torque_supplier_;

            lfoot_torque_g_switch = 0;
            rfoot_torque_g_switch = 0;
        }
        else if (stop_walking_trigger_ == true)
        {
            if (foot_contact_ == -1) // right support, left support previously
            {

                lfoot_task_torque_switch = 1;
                rfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 0, 1, 0, 0);

                lfoot_torque_g_switch = 0;
                rfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 1, 0, 0, 0);
            }
            else if (foot_contact_ == 1)
            {

                lfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 0, 1, 0, 0);
                rfoot_task_torque_switch = 1;

                lfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 1, 0, 0, 0);
                rfoot_torque_g_switch = 0;
            }
        }
        else if (start_walking_trigger_ == true)
        {
            if (foot_contact_ == -1) // right support, left support previously
            {
                // torque_l_vel_tun.segment(12, 3).setZero(); // waist torque
                // torque_l_vel_tun.segment(23, 2).setZero(); // head torque

                lfoot_task_torque_switch = 1;
                rfoot_task_torque_switch = 1;

                lfoot_torque_g_switch = 0;
                rfoot_torque_g_switch = 0;
            }
            else if (foot_contact_ == 1)
            {
                // torque_r_vel_tun.segment(12, 3).setZero(); // waist torque
                // torque_r_vel_tun.segment(23, 2).setZero(); // head torque

                lfoot_task_torque_switch = 1;
                rfoot_task_torque_switch = 1;

                lfoot_torque_g_switch = 0;
                rfoot_torque_g_switch = 0;
            }
        }
    }

    torque_g_.segment(0, 6) = torque_g_.segment(0, 6) * lfoot_torque_g_switch;
    torque_g_.segment(6, 6) = torque_g_.segment(6, 6) * rfoot_torque_g_switch;

    // torque_g_.segment(12, 3).setZero();
    // if( int(walking_duration_*100)%10 == 0 )
    // cout<<"walking_phase_: \n"<<walking_phase_<<endl;
    // torque += torque_l_vel_tun * lfoot_task_torque_switch + torque_r_vel_tun * rfoot_task_torque_switch;
    // torque += torque_l_vel_tun + torque_r_vel_tun;
    // torque(0) =0;
    // torque(6) =0;
    /////////////////////////////////////////////////////////////////

    // torque = wc.task_control_torque_QP2(rd_, jac_com_xy, f_com);  //jacobian control + gravity torque

    // if(int(control_time_) %2 ==0)
    // {
    //     cout<<"Com Vel torque: \n"<< torque <<endl;
    //     cout<<"f_com: \n"<< f_com <<endl;
    // }

    torque += torque_g_;

    // if( walking_phase_< 0.1)
    // {
    // 	cout<<"pelv_rot_current_: \n"<<pelv_rot_current_<<endl;
    // 	cout<<"pelv_rpy_current_: \n"<<pelv_rpy_current_<<endl;
    // 	cout<<"f_star: \n"<<f_star<<endl;
    // 	cout<<"torque: \n"<<torque<<endl;
    // 	cout<<"torque_g: \n"<<torque_g_<<endl;
    // 	cout<<"lfoot_task_torque_switch: \n"<<lfoot_task_torque_switch<<endl;
    // 	cout<<"rfoot_task_torque_switch: \n"<<rfoot_task_torque_switch<<endl;
    // 	cout<<"torque_l_vel_tun: \n"<<torque_l_vel_tun<<endl;
    // 	cout<<"torque_r_vel_tun: \n"<<torque_r_vel_tun<<endl;
    // 	cout<<"lfoot_to_com_jac_from_global_: \n"<<lfoot_to_com_jac_from_global_<<endl;
    // }

    // if( (walking_phase_ > 0.2) && (walking_phase_ < 0.7))
    // torque = tuneTorqueForZMPSafety(torque); //turn off velocity tuning if the zmp is outside of the foot

    return torque;
}

Eigen::VectorQd AvatarController::swingFootControlCompute()
{
    VectorQd torque;
    torque.setZero();

    if (foot_swing_trigger_ == true)
    {
        Vector3d lhip_joint_position;
        lhip_joint_position << 0.11, 0.1025, -0.1025;
        lhip_joint_position = pelv_rot_current_yaw_aline_ * lhip_joint_position;
        Vector3d rhip_joint_position;
        rhip_joint_position << 0.11, -0.1025, -0.1025;
        rhip_joint_position = pelv_rot_current_yaw_aline_ * rhip_joint_position;

        Vector3d landing_3d_point;
        Vector2d desired_landing_xy_point;
        double swingfoot_target_weight;

        // swingfoot_target_weight = (walking_phase_ - 0.3)/0.2;
        swingfoot_target_weight = 1;
        swingfoot_target_weight = DyrosMath::minmax_cut(swingfoot_target_weight, 0.0, 1.0);

        desired_landing_xy_point(0) = walking_speed_ * walking_duration_ + support_foot_transform_current_.translation()(0);
        desired_landing_xy_point(1) = step_width_ * (-foot_contact_) + support_foot_transform_current_.translation()(1);

        landing_3d_point.segment(0, 2) = swingfoot_target_weight * swing_foot_pos_trajectory_from_global_.segment(0, 2) + (1 - swingfoot_target_weight) * desired_landing_xy_point;
        landing_3d_point(2) = swing_foot_pos_trajectory_from_global_(2);

        if (foot_contact_ == -1)
        {
            swingfoot_f_star_l_ = landing_3d_point - lhip_joint_position;
            swingfoot_f_star_l_.normalize();
            swingfoot_f_star_l_ *= 1 * rd_.link_[COM_id].mass * GRAVITY;
            swingfoot_f_star_l_ = 0.7 * swingfoot_f_star_l_ + 0.3 * swingfoot_f_star_l_pre_;

            swingfoot_f_star_r_ = 0.3 * swingfoot_f_star_r_pre_;
        }
        else if (foot_contact_ == 1)
        {
            swingfoot_f_star_r_ = landing_3d_point - rhip_joint_position;
            swingfoot_f_star_r_.normalize();
            swingfoot_f_star_r_ *= 1 * rd_.link_[COM_id].mass * GRAVITY;
            swingfoot_f_star_r_ = 0.7 * swingfoot_f_star_r_ + 0.3 * swingfoot_f_star_r_pre_;

            swingfoot_f_star_l_ = 0.3 * swingfoot_f_star_l_pre_;
        }
    }
    else
    {
        swingfoot_f_star_l_ = 0.3 * swingfoot_f_star_l_pre_;
        swingfoot_f_star_r_ = 0.3 * swingfoot_f_star_r_pre_;
    }

    torque += ((jac_lfoot_.block(0, 6, 3, MODEL_DOF)).transpose() * swingfoot_f_star_l_ + (jac_rfoot_.block(0, 6, 3, MODEL_DOF)).transpose() * swingfoot_f_star_r_) * swingfoot_force_control_converter_;

    return torque;
}

Eigen::VectorQd AvatarController::jointTrajectoryPDControlCompute()
{
    Eigen::VectorQd torque;
    Eigen::Vector12d desired_q_leg;
    Eigen::Isometry3d pelv_transform_from_global;
    Eigen::Isometry3d lleg_transform_from_global;
    Eigen::Isometry3d rleg_transform_from_global;
    Eigen::Isometry3d lleg_transform_target;
    Eigen::Isometry3d rleg_transform_target;
    Eigen::Matrix3d pevl_target_rot;
    Eigen::Vector3d torque_pelv_for_ankle;

    // double default_stance_foot_z_from_pelv = -0.349 * (cos(0.02) + cos(0.12)) - 0.1025;
    // lleg_transform_target.translation()(0) = -0.015;
    // lleg_transform_target.translation()(1) = 0.1025;
    // lleg_transform_target.translation()(2) = default_stance_foot_z_from_pelv;
    lleg_transform_target.linear().setIdentity();
    // rleg_transform_target.translation()(0) = -0.015;
    // rleg_transform_target.translation()(1) = -0.1025;
    // rleg_transform_target.translation()(2) = default_stance_foot_z_from_pelv;
    rleg_transform_target.linear().setIdentity();
    lleg_transform_target.translation() = lfoot_transform_current_from_global_.translation();
    rleg_transform_target.translation() = rfoot_transform_current_from_global_.translation();

    // cout<<"lfoot_transform_init_from_global_.translation(): \n"<<lfoot_transform_init_from_global_.translation()<<endl;
    // cout<<"lfoot_transform_init_from_global_.linear(): \n"<<lfoot_transform_init_from_global_.linear()<<endl;
    // cout<<"rfoot_transform_init_from_global_.translation(): \n"<<rfoot_transform_init_from_global_.translation()<<endl;
    // cout<<"rfoot_transform_init_from_global_.linear(): \n"<<rfoot_transform_init_from_global_.linear()<<endl;
    /////////////////////////////////PELVIS/////////////////////////////////////

    pelv_transform_from_global.translation().setZero();
    pelv_transform_from_global.linear() = pelv_rot_current_yaw_aline_; //
    // pelv_transform_from_global.linear() = pelv_yaw_rot_current_from_global_;
    // pelv_transform_from_global.linear().setIdentity();

    torque_pelv_.setZero();
    torque_pelv_for_ankle.setZero();
    torque_swing_assist_.setZero();

    //////////////////////////////////////////////////////////////////////////////////////////

    double swing_pd_switch;

    torque.setZero();

    //////////////////////////SWING FOOT & STANCE FOOT ankle and knee joints///////////////////////
    if (foot_swing_trigger_ == true)
    {
        if (foot_contact_ == -1) //right support
        {
            lleg_transform_from_global.translation() = swing_foot_pos_trajectory_from_global_;
            lleg_transform_from_global.linear() = swing_foot_rot_trajectory_from_global_;

            rleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(0), 0, 0, rleg_transform_target.translation()(0), 0, 0)(0);
            rleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(1), 0, 0, rleg_transform_target.translation()(1), 0, 0)(0);
            rleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(2), 0, 0, rleg_transform_target.translation()(2), 0, 0)(0);

            rleg_transform_from_global.linear() = rleg_transform_target.linear();
            // rleg_transform_from_global = rleg_transform_target;
            computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

            for (int i = 1; i < 4; i++) //hip and knee
            {
                desired_q_(i) = desired_q_leg(i); //left swing foot
                                                  // kp_joint(i) = 900; //swing foot gain
                                                  // kv_joint(i) = 60;

                // if(walking_phase_ < 0.1)
                // {
                //     cout<<"foot_contact_: \n"<<foot_contact_<<endl;
                //     cout<<"desired_q_leg: \n"<<desired_q_leg<<endl;
                //     cout<<"lleg_transform_from_global.translation(): \n" <<lleg_transform_from_global.translation()<<endl;
                //     cout<<"lleg_transform_from_global.rotation(): \n" <<lleg_transform_from_global.rotation()<<endl;
                //     cout<<"rleg_transform_from_global.translation(): \n" <<rleg_transform_from_global.traznslation()<<endl;
                //     cout<<"rleg_transform_from_global.rotation(): \n" <<rleg_transform_from_global.rotation()<<endl;
                //     cout<<"pelv_transform_from_global.translation(): \n" <<pelv_transform_from_global.translation()<<endl;
                //     cout<<"pelv_transform_from_global.rotation(): \n" <<pelv_transform_from_global.rotation()<<endl;
                // }
            }

            desired_q_(0) = 0.5 * 0 + 0.5 * pre_desired_q_(0);
            desired_q_(3) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, last_desired_q_(3), 0, 0, desired_q_leg(3), 0, 0)(0);

            // desired_q_(0) = motion_q_(0);
            Vector3d phi_swing_ankle;
            // phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].rotm, pelv_yaw_rot_current_from_global_);
            phi_swing_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), Eigen::Matrix3d::Identity());
            // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

            desired_q_dot_(4) = 200 * bandBlock(phi_swing_ankle(1), 15 * DEG2RAD, -15 * DEG2RAD); //swing ankle pitch	//(tune)
            desired_q_dot_(5) = 200 * bandBlock(phi_swing_ankle(0), 0 * DEG2RAD, 0 * DEG2RAD);    //swing ankle roll	//(tune)
            desired_q_(4) = current_q_(4) + desired_q_dot_(4) * dt_;
            desired_q_(5) = current_q_(5) + desired_q_dot_(5) * dt_;
            // desired_q_dot_(4) = 0;
            // desired_q_dot_(5) = 0;
            // // desired_q_(4) = current_q_(4);
            // // desired_q_(5) = current_q_(5);
            // desired_q_(4) = 0;
            // desired_q_(5) = 0;

            Vector3d phi_support_ankle;
            // phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].rotm, pelv_yaw_rot_current_from_global_);
            phi_support_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), DyrosMath::rotateWithZ(current_q_(6)));

            desired_q_dot_(10) = 200 * bandBlock(phi_support_ankle(1), 15 * DEG2RAD, -15 * DEG2RAD); //(tune)
            desired_q_dot_(11) = 200 * bandBlock(phi_support_ankle(0), 10 * DEG2RAD, -10 * DEG2RAD); //(tune)
            desired_q_(10) = current_q_(10) + desired_q_dot_(10) * dt_;
            desired_q_(11) = current_q_(11) + desired_q_dot_(11) * dt_;
            // desired_q_dot_(10) = 0;
            // desired_q_dot_(11) = 0;
            // desired_q_(10) = current_q_(10);
            // desired_q_(11) = current_q_(11);

            // low pass filter for suppport foot target position
            // desired_q_(6) = 0.5 * motion_q_(6) + 0.5 * pre_desired_q_(6); //right support foot hip yaw
            desired_q_(6) = DyrosMath::QuinticSpline(turning_phase_, 0, 1, last_desired_q_(6), 0, 0, motion_q_(6), 0, 0)(0);
            desired_q_(9) = DyrosMath::QuinticSpline(walking_phase_, 0.0, 0.3, last_desired_q_(9), 0, 0, motion_q_(9), 0, 0)(0);
            // desired_q_(9) = 0.5*motion_q_(9) + 0.5*pre_desired_q_(9); //right support foot
            // desired_q_(9) = desired_q_leg(9); //right support foot knee
            // desired_q_(10) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
            // desired_q_(11) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll
            // desired_q_(10) = 0.3*motion_q_(10) + 0.7*pre_desired_q_(10); //right support foot ankle pitc
            // desired_q_(11) = 0.3*motion_q_(11) + 0.7*pre_desired_q_(11); //right support foot anlke roll
            // desired_q_(10) = desired_q_leg(10) ;
            // desired_q_(11) = desired_q_leg(11) ;

            for (int i = 1; i < 6; i++)
            {
                if (kp_joint_(i + 6) == kp_soft_joint_(i + 6))
                {
                    kp_joint_(i + 6) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint_(i + 6), kp_stiff_joint_(i + 6), 0, 0); //support foot
                }

                if (kp_joint_(i) == kp_stiff_joint_(i))
                {
                    kp_joint_(i) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint_(i), kp_soft_joint_(i), 0, 0); //swing foot
                }
            }
            // kp_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 2000, 600, 0, 0); //swing foot knee
            // kv_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 60, 50, 0, 0);

            // kp_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0); //swing foot ankle gain
            // kv_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);
            // kp_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0);
            // kv_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);

            swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

            if (first_step_trigger_ == true)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = swing_pd_switch;
                pd_control_mask_(2) = swing_pd_switch;
                // pd_control_mask_(3) = swing_pd_switch;
                // pd_control_mask_(4) = swing_pd_switch; //test
                // pd_control_mask_(5) = swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 0;
                pd_control_mask_(8) = 0;
                // pd_control_mask_(9) = 0;
                // pd_control_mask_(10) = 0; //test
                // pd_control_mask_(11) = 0;
            }
            else
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = swing_pd_switch;
                pd_control_mask_(2) = swing_pd_switch;
                // pd_control_mask_(3) = swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 1 - swing_pd_switch;
                pd_control_mask_(8) = 1 - swing_pd_switch;
                // pd_control_mask_(9) = 0;
                // pd_control_mask_(10) = 1 - swing_pd_switch; //test
                // pd_control_mask_(11) = 1 - swing_pd_switch;
            }
        }
        else if (foot_contact_ == 1) //left support
        {

            rleg_transform_from_global.translation() = swing_foot_pos_trajectory_from_global_;
            rleg_transform_from_global.linear() = swing_foot_rot_trajectory_from_global_;

            lleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(0), 0, 0, lleg_transform_target.translation()(0), 0, 0)(0);
            lleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(1), 0, 0, lleg_transform_target.translation()(1), 0, 0)(0);
            lleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(2), 0, 0, lleg_transform_target.translation()(2), 0, 0)(0);

            lleg_transform_from_global.linear() = lleg_transform_target.linear();
            // lleg_transform_from_global = lleg_transform_target;

            computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

            for (int i = 7; i < 10; i++)
            {
                desired_q_(i) = desired_q_leg(i); //right swing foot
                                                  // kp_joint(i) = 900; //swing foot gain
                                                  // kv_joint(i) = 60;

                // desired_q_(i-6) = 0.5*motion_q_(i-6) + 0.5*pre_desired_q_(i-6); //left support foot
                // if(walking_phase_ < 0.1)
                // {
                //     cout<<"desired_q_leg: \n"<<desired_q_leg<<endl;
                //     cout<<"foot_contact_: \n"<<foot_contact_<<endl;
                //     cout<<"lleg_transform_from_global.translation(): \n" <<lleg_transform_from_global.translation()<<endl;
                //     cout<<"lleg_transform_from_global.rotation(): \n" <<lleg_transform_from_global.rotation()<<endl;
                //     cout<<"rleg_transform_from_global.translation(): \n" <<rleg_transform_from_global.translation()<<endl;
                //     cout<<"rleg_transform_from_global.rotation(): \n" <<rleg_transform_from_global.rotation()<<endl;
                //     cout<<"pelv_transform_from_global.translation(): \n" <<pelv_transform_from_global.translation()<<endl;
                //     cout<<"pelv_transform_from_global.rotation(): \n" <<pelv_transform_from_global.rotation()<<endl;
                // }
            }

            desired_q_(6) = 0.5 * 0 + 0.5 * pre_desired_q_(6);
            desired_q_(9) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, last_desired_q_(9), 0, 0, desired_q_leg(9), 0, 0)(0);

            Vector3d phi_swing_ankle;
            // phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].rotm, pelv_yaw_rot_current_from_global_);
            phi_swing_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), Eigen::Matrix3d::Identity());
            // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

            desired_q_dot_(10) = 200 * bandBlock(phi_swing_ankle(1), 15 * DEG2RAD, -15 * DEG2RAD); //swing ankle pitch
            desired_q_dot_(11) = 200 * bandBlock(phi_swing_ankle(0), 0 * DEG2RAD, 0 * DEG2RAD);    //swing ankle roll
            desired_q_(10) = current_q_(10) + desired_q_dot_(10) * dt_;
            desired_q_(11) = current_q_(11) + desired_q_dot_(11) * dt_;
            // desired_q_dot_(10) = 0;
            // desired_q_dot_(11) = 0;
            // // desired_q_(10) = current_q_(10);
            // // desired_q_(11) = current_q_(11);
            // desired_q_(10) = 0;
            // desired_q_(11) = 0;

            Vector3d phi_support_ankle;
            // phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].rotm, pelv_yaw_rot_current_from_global_);
            phi_support_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), DyrosMath::rotateWithZ(current_q_(0)));

            desired_q_dot_(4) = 200 * bandBlock(phi_support_ankle(1), 15 * DEG2RAD, -15 * DEG2RAD);
            ;
            desired_q_dot_(5) = 200 * bandBlock(phi_support_ankle(0), 10 * DEG2RAD, -10 * DEG2RAD);
            ;
            desired_q_(4) = current_q_(4) + desired_q_dot_(4) * dt_;
            desired_q_(5) = current_q_(5) + desired_q_dot_(5) * dt_;
            // desired_q_dot_(4) = 0;
            // desired_q_dot_(5) = 0;
            // desired_q_(4) = current_q_(4);
            // desired_q_(5) = current_q_(5);

            // desired_q_(0) = 0.5 * motion_q_(0) + 0.5 * pre_desired_q_(0); //left support foot hip yaw
            desired_q_(0) = DyrosMath::QuinticSpline(turning_phase_, 0, 1, last_desired_q_(0), 0, 0, motion_q_(0), 0, 0)(0); //left support foot hip yaw
            desired_q_(3) = DyrosMath::QuinticSpline(walking_phase_, 0.0, 0.3, last_desired_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
            // desired_q_(3) = 0.5*motion_q_(3) + 0.5*pre_desired_q_(3); //left support foot knee
            // desired_q_(3) = desired_q_leg(3); //left support foot knee
            // desired_q_(4) = 0.5*desired_q_leg(4) + 0.5*pre_desired_q_(4); //left support foot ankle pitch
            // desired_q_(5) = 0.5*desired_q_leg(5) + 0.5*pre_desired_q_(5); //left support foot anlke roll
            // desired_q_(4) = 0.3*motion_q_(4) + 0.7*pre_desired_q_(4); //left support foot ankle pitch
            // desired_q_(5) = 0.3*motion_q_(5) + 0.7*pre_desired_q_(5); //left support foot anlke roll
            // desired_q_(4) = desired_q_leg(4) ;
            // desired_q_(5) = desired_q_leg(5) ;

            for (int i = 1; i < 6; i++)
            {
                if (kp_joint_(i) == kp_soft_joint_(i))
                {
                    kp_joint_(i) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint_(i), kp_stiff_joint_(i), 0, 0); //support foot
                }

                if (kp_joint_(i + 6) == kp_stiff_joint_(i + 6))
                {
                    kp_joint_(i + 6) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint_(i + 6), kp_soft_joint_(i + 6), 0, 0); //swing foot
                }
            }
            // kp_joint(9) = DyrosMath::cubic(walking_phase_, 0.8, 1, 2000, 600, 0, 0); //swing foot knee
            // kv_joint(9) = DyrosMath::cubic(walking_phase_, 0.8, 1, 60, 50, 0, 0);

            // kp_joint(10) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0); //swing foot ankle gain
            // kv_joint(10) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);
            // kp_joint(11) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0);
            // kv_joint(11) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);

            /////////////////Swing Assist Torque/////////////////////////////////////////
            // right foot swing height assist feed forward torque on hip roll, hip pitch, knee pitch
            Eigen::VectorXd f_star;
            f_star.setZero(3);
            f_star(2) = swing_foot_acc_trajectory_from_global_(2);

            // torque_swing_assist_.segment(7, 3) = WBC::task_control_torque(rd_, jac_rfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(7, 3);
            // WBC::SetContact(rd_, 1, 0);
            // torque_swing_assist_.segment(7, 3) = WBC::task_control_torque_motor(rd_, jac_rfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(7, 3);

            // torque_swing_assist_.segment(7, 3) = (jac_rfoot_.transpose()).block(13, 0, 3, 6)*rd_.lambda.block(0, 2, 6, 1)*swing_foot_acc_trajectory_from_global_(2);
            /////////////////////////////////////////////////////////////////////////////

            swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

            if (first_step_trigger_ == true)
            {

                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 0;
                pd_control_mask_(2) = 0;
                // pd_control_mask_(3) = 0;
                // pd_control_mask_(4) = 0; //test
                // pd_control_mask_(5) = 0;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = swing_pd_switch;
                pd_control_mask_(8) = swing_pd_switch;
                // pd_control_mask_(9) = swing_pd_switch;
            }
            else
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 1 - swing_pd_switch;
                pd_control_mask_(2) = 1 - swing_pd_switch;
                // pd_control_mask_(3) = 0;
                // pd_control_mask_(4) = 1 - swing_pd_switch; //test
                // pd_control_mask_(5) = 1 - swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = swing_pd_switch;
                pd_control_mask_(8) = swing_pd_switch;
                // pd_control_mask_(9) = swing_pd_switch;
            }
        }
    }
    else
    {

        lleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(0), 0, 0, lleg_transform_target.translation()(0), 0, 0)(0);
        lleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(1), 0, 0, lleg_transform_target.translation()(1), 0, 0)(0);
        lleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(2), 0, 0, lleg_transform_target.translation()(2), 0, 0)(0);
        lleg_transform_from_global.linear() = lleg_transform_target.linear();

        rleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(0), 0, 0, rleg_transform_target.translation()(0), 0, 0)(0);
        rleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(1), 0, 0, rleg_transform_target.translation()(1), 0, 0)(0);
        rleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(2), 0, 0, rleg_transform_target.translation()(2), 0, 0)(0);
        rleg_transform_from_global.linear() = rleg_transform_target.linear();
        computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

        // for(int i = 4; i<6; i++)
        // {
        //     desired_q_(i) = 0.3*desired_q_leg(i) + 0.7*pre_desired_q_(i); //left ankle
        //     desired_q_(i+6) = 0.3*desired_q_leg(i+6) + 0.7*pre_desired_q_(i+6); //right ankle
        // }
        desired_q_(0) = 0.5 * 0 + 0.5 * pre_desired_q_(0); // hip yaw
        desired_q_(6) = 0.5 * 0 + 0.5 * pre_desired_q_(6);

        desired_q_(3) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 3, last_desired_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
        desired_q_(9) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 3, last_desired_q_(9), 0, 0, motion_q_(9), 0, 0)(0);
        // desired_q_(3) = 0.4 * motion_q_(3) + 0.6 * pre_desired_q_(3); //left knee
        // desired_q_(9) = 0.4 * motion_q_(9) + 0.6 * pre_desired_q_(9); //right knee
        // desired_q_(3) =	current_q_(3);
        // desired_q_(9) =	current_q_(9);

        // desired_q_(4) = 0.3*motion_q_(4) + 0.7*pre_desired_q_(4); //lefft ankle pitch
        // desired_q_(5) = 0.3*motion_q_(5) + 0.7*pre_desired_q_(5); //lefft ankle roll
        // desired_q_(10) = 0.3*motion_q_(10) + 0.7*pre_desired_q_(10); //right ankle pitch
        // desired_q_(11) = 0.3*motion_q_(11) + 0.7*pre_desired_q_(11); //right ankle roll

        // desired_q_(4) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
        // desired_q_(5) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll
        // desired_q_(10) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
        // desired_q_(11) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll

        // desired_q_(4) = desired_q_leg(4);
        // desired_q_(5) = desired_q_leg(5);
        // desired_q_(10) = desired_q_leg(10);
        // desired_q_(11) = desired_q_leg(11);

        desired_q_(4) = current_q_(4); // test
        desired_q_(5) = current_q_(5);
        desired_q_(10) = current_q_(10);
        desired_q_(11) = current_q_(11);

        desired_q_dot_(4) = 0; // test
        desired_q_dot_(5) = 0;
        desired_q_dot_(10) = 0;
        desired_q_dot_(11) = 0;
        for (int i = 1; i < 6; i++)
        {
            kp_joint_(i + 6) = kp_stiff_joint_(i + 6); //swing foot
            kp_joint_(i) = kp_stiff_joint_(i);         //support foot
        }

        swing_pd_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, 0, 1, 0, 0);

        if (program_start_time_ == stance_start_time_)
        {
            // desired_q_(5) = current_q_(5); //aknle roll free for start motion
            // desired_q_(11) = current_q_(11); //aknle roll

            // kp_joint(5) = 600;
            // kv_joint(5) = 40;
            // kp_joint(11)= 600;
            // kv_joint(11)= 40;

            // kp_joint(4) = 400;
            // kv_joint(4) = 40;
            // kp_joint(10)= 400;
            // kv_joint(10)= 40;

            pd_control_mask_(6) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
            pd_control_mask_(7) = 0;
            pd_control_mask_(8) = 0;
            // pd_control_mask_(9) = 0;
            // pd_control_mask_(10) = 0;	//test
            // pd_control_mask_(11) = 0;
            // pd_control_mask_(9) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);

            pd_control_mask_(0) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
            pd_control_mask_(1) = 0;
            pd_control_mask_(2) = 0;
            // pd_control_mask_(3) = 0;
            // pd_control_mask_(4) = 0;	//test
            // pd_control_mask_(5) = 0;
            // pd_control_mask_(3) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);

            // cout<<"hip_control_switch: "<< hip_control_switch <<endl;
        }
        else if (stop_walking_trigger_ == true)
        {
            if (foot_contact_ == 1)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 1 - swing_pd_switch;
                pd_control_mask_(2) = 1 - swing_pd_switch;
                // pd_control_mask_(3) = 1 - swing_pd_switch; //test
                // pd_control_mask_(4) = 1 - swing_pd_switch;; //test
                // pd_control_mask_(5) = 1 - swing_pd_switch;; //test

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 0;
                pd_control_mask_(8) = 0;
                // pd_control_mask_(9) = 0;
                // pd_control_mask_(10) = 0; //test
                // pd_control_mask_(11) = 0; //test
            }
            else if (foot_contact_ == -1)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 0;
                pd_control_mask_(2) = 0;
                // pd_control_mask_(3) = 0; //test
                // pd_control_mask_(4) = 0; //test
                // pd_control_mask_(5) = 0; //test

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 1 - swing_pd_switch;
                pd_control_mask_(8) = 1 - swing_pd_switch;
                // pd_control_mask_(9) = 1 - swing_pd_switch; //test
                // pd_control_mask_(10) = 1 - swing_pd_switch; //test
                // pd_control_mask_(11) = 1 - swing_pd_switch; //test
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////HIP YAW/////////////////////////////
    // desired_q_(0)=motion_q_(0);
    // desired_q_(6)=motion_q_(6);
    ////////////////////////////////////////////////////////////////////

    //////////////////////////////////LEG Q DOT/////////////////////////////////
    desired_q_dot_.segment(0, 4) = (desired_q_.segment(0, 4) - pre_desired_q_.segment(0, 4)) / dt_; //left hip and knee
    desired_q_dot_.segment(6, 4) = (desired_q_.segment(6, 4) - pre_desired_q_.segment(6, 4)) / dt_; //left hip and knee
    if (walking_phase_ == 0)
    {
        desired_q_dot_.segment(0, 4).setZero();
        desired_q_dot_.segment(6, 4).setZero();
    }
    ///////////////////////////////////////////////////////////////////////////

    /////////////////////////////////WAIST DESIRED JOINT ANGLES//////////////////////////////
    // Vector3d phi_trunk;
    // Matrix3d upper_body_rotaion_matrix = DyrosMath::rotateWithZ(motion_q_(12));
    // phi_trunk = -DyrosMath::getPhi(pelv_yaw_rot_current_from_global_.transpose()* rd_.link_[Upper_Body].rotm, upper_body_rotaion_matrix);
    // // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

    // desired_q_dot_(12) = 50 * phi_trunk(2);	 //waist yaw //(tune)
    // desired_q_dot_(13) = 30 * phi_trunk(1);	 //waist pitch
    // desired_q_dot_(14) = -30 * phi_trunk(0); //waist roll

    // desired_q_.segment(12, 3) = current_q_.segment(12, 3) + desired_q_dot_.segment(12, 3) * dt_;

    // desired_q_(12) = motion_q_(12);
    for (int i = 12; i < 15; i++)
    {
        desired_q_(i) = motion_q_(i);
        desired_q_dot_(i) = motion_q_dot_(i);
    }
    //////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////ARM & HEAD///////////////////////////////////////////////
    for (int i = 15; i < MODEL_DOF; i++)
    {
        desired_q_(i) = motion_q_(i);
        desired_q_dot_(i) = motion_q_dot_(i);
    }
    ////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////MOTION CONTROL/////////////////////////////////////
    // torque += stablePDControl(1000, 1000*dt_*16, current_q_, current_q_dot_, current_q_ddot_, desired_q_, desired_q_dot_);
    // kp_joint_(4) = 0;	//only do damping control for ankle
    // kp_joint_(5) = 0;
    // kp_joint_(10) = 0;
    // kp_joint_(11) = 0;

    pd_control_mask_(1) *= 1 - swingfoot_force_control_converter_;
    pd_control_mask_(2) *= 1 - swingfoot_force_control_converter_;
    // pd_control_mask_(3) *= 1 - swingfoot_force_control_converter_; //knee
    pd_control_mask_(7) *= 1 - swingfoot_force_control_converter_;
    pd_control_mask_(8) *= 1 - swingfoot_force_control_converter_;
    // pd_control_mask_(9) *= 1 - swingfoot_force_control_converter_; //knee

    for (int i = 0; i < MODEL_DOF; i++)
    {
        torque(i) = (kp_joint_(i) * (desired_q_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_(i) - current_q_dot_(i)));
        torque(i) = torque(i) * pd_control_mask_(i); // masking for joint pd control
    }
    return torque;
}

Eigen::VectorQd AvatarController::jointControl(Eigen::VectorQd current_q, Eigen::VectorQd &desired_q, Eigen::VectorQd current_q_dot, Eigen::VectorQd &desired_q_dot, Eigen::VectorQd pd_mask)
{
    VectorQd torque_retrun;
    torque_grav_.setZero();
    torque_retrun.setZero();
    torque_grav_ = WBC::GravityCompensationTorque(rd_);

    // start spline
    if (current_time_ < program_start_time_ + 2.0)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            desired_q(i) = DyrosMath::QuinticSpline(current_time_, program_start_time_, program_start_time_ + 2.0, init_q_(i), 0, 0, desired_q(i), 0, 0)(0);
        }
    }

    // support foot change spline
    if ((walking_phase_ < switching_phase_duration_) && (foot_swing_trigger_ == true))
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            desired_q(i) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, last_desired_q_(i), 0, 0, desired_q(i), 0, 0)(0);
        }
    }

    // stop spline
    if ((current_time_ < stance_start_time_ + 0.03) && (stop_walking_trigger_ == true))
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            desired_q(i) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.03, last_desired_q_(i), 0, 0, desired_q(i), 0, 0)(0);
        }
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        torque_retrun(i) = (kp_joint_(i) * (desired_q(i) - current_q(i)) + kv_joint_(i) * (desired_q_dot(i) - current_q_dot(i)));
        torque_retrun(i) = torque_retrun(i) * pd_control_mask_(i) + torque_grav_(i); // masking for joint pd control
    }

    if (int(current_time_ * 10000) % 1000 == 0)
    {
        // cout<<"torque_grav_: \n"<<torque_grav_.segment(0, 12)<<endl;
        // cout<<"torque_retrun: \n"<<torque_retrun.segment(0, 12)<<endl;
        // cout<<"foot_contact_: "<<foot_contact_<<endl;
    }
    return torque_retrun;
}

Eigen::VectorQd AvatarController::gravityCompensator(Eigen::VectorQd current_q)
{
    double contact_ratio = 0.0;
    Eigen::Vector12d A;
    double B = 0.0;
    Eigen::VectorQd torque_g_ssp;
    Eigen::VectorQd torque_g_dsp;
    // Eigen::VectorQd contact_redist_torque;
    Eigen::VectorQd total_gravity_compensation_torque;
    double dsp_smoothing_l = 0.05;
    double support_foot;
    total_gravity_compensation_torque.setZero();

    if (foot_swing_trigger_ == true)
    {
        if (walking_phase_ <= dsp_ratio_)
        {
            WBC::SetContact(rd_, 1, 1);
            torque_g_dsp = WBC::GravityCompensationTorque(rd_);
            torque_g_ssp.setZero();

            if (walking_phase_ < dsp_ratio_ * dsp_smoothing_l)
            {
                if (first_step_trigger_ == true)
                {
                    contact_ratio = 1;
                }
                else
                {
                    contact_ratio = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * dsp_smoothing_l, 0.0, 1.0, 0.0, 0.0);
                }

                if (foot_contact_ == 1)
                {
                    support_foot = 0;
                }
                else
                {
                    support_foot = 1;
                }
            }
            else if (walking_phase_ > dsp_ratio_ * (1 - dsp_smoothing_l))
            {
                contact_ratio = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - dsp_smoothing_l), dsp_ratio_, 1.0, 0.0, 0.0, 0.0);
                if (foot_contact_ == 1)
                {
                    support_foot = 1;
                }
                else
                {
                    support_foot = 0;
                }
            }
            else
            {
                contact_ratio = 1;
            }

            torque_g_dsp = WBC::ContactForceRedistributionTorqueWalking(rd_, torque_g_dsp, 0.9, contact_ratio, support_foot);
        }
        else
        {
            torque_g_dsp.setZero();
            // contact_redist_torque.setZero();

            if (foot_contact_ == 1)
            {
                WBC::SetContact(rd_, 1, 0);
                torque_g_ssp = WBC::GravityCompensationTorque(rd_);

                //real robot
                // torque_g_ssp(1) = 1.4*torque_g_ssp(1);
                // torque_g_ssp(5) = 1.15*torque_g_ssp(5);
            }
            else
            {
                WBC::SetContact(rd_, 0, 1);
                torque_g_ssp = WBC::GravityCompensationTorque(rd_);

                //real robot
                // torque_g_ssp(7) = 1.4*torque_g_ssp(7);
                // torque_g_ssp(11) = 1.15*torque_g_ssp(11);
            }
        }
    }
    else
    {
        WBC::SetContact(rd_, 1, 1);
        torque_g_dsp = WBC::GravityCompensationTorque(rd_);
        torque_g_ssp.setZero();

        contact_ratio = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + dsp_duration_ * 0.1, 0.0, 1.0, 0.0, 0.0);
        // contact_ratio = 1;

        if (foot_contact_ == 1)
        {
            support_foot = 1;
        }
        else
        {
            support_foot = 0;
        }

        torque_g_dsp = WBC::ContactForceRedistributionTorqueWalking(rd_, torque_g_dsp, 0.9, contact_ratio, support_foot);
    }

    total_gravity_compensation_torque = torque_g_ssp + torque_g_dsp;
    return total_gravity_compensation_torque;
}

Eigen::VectorQd AvatarController::hipAngleCompensator(Eigen::VectorQd desired_q)
{
    double left_hip_roll = -0.4 * DEG2RAD, right_hip_roll = -0.4 * DEG2RAD, left_hip_roll_first = -0.50 * DEG2RAD, right_hip_roll_first = -0.50 * DEG2RAD, //실험, 제자리 0.6, 0.4
        left_hip_pitch = 0.4 * DEG2RAD, right_hip_pitch = 0.4 * DEG2RAD, left_hip_pitch_first = 0.40 * DEG2RAD, right_hip_pitch_first = 0.40 * DEG2RAD,    // 실험 , 제자리 0.75deg
        left_ank_pitch = 0.0 * DEG2RAD, right_ank_pitch = 0.0 * DEG2RAD, left_ank_pitch_first = 0.0 * DEG2RAD, right_ank_pitch_first = 0.0 * DEG2RAD,
           left_hip_roll_temp = 0.0, right_hip_roll_temp = 0.0, left_hip_pitch_temp = 0.0, right_hip_pitch_temp = 0.0, left_ank_pitch_temp = 0.0, right_ank_pitch_temp = 0.0, comp_ratio = 0.05;

    Eigen::VectorQd compensated_desired_q;

    if (foot_swing_trigger_ == true)
    {
        if (first_step_trigger_ == true)
        {
            if (foot_contact_ == 1) //left support foot
            {
                if (walking_phase_ <= dsp_ratio_ * comp_ratio)
                {
                    left_hip_roll_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, left_hip_roll_first, 0.0, 0.0);
                    left_hip_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, left_hip_pitch_first, 0.0, 0.0);
                    left_ank_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, left_ank_pitch_first, 0.0, 0.0);
                }
                else if (walking_phase_ <= dsp_ratio_)
                {
                    left_hip_roll_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, left_hip_roll_first, 0.0, 0.0, 0.0);
                    left_hip_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, left_hip_pitch_first, 0.0, 0.0, 0.0);
                    left_ank_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, left_ank_pitch_first, 0.0, 0.0, 0.0);
                }
                else
                {
                    left_hip_roll_temp = 0.0;
                    left_hip_pitch_temp = 0.0;
                    left_ank_pitch_temp = 0.0;
                }
            }
            else // right support foot
            {
                if (walking_phase_ <= dsp_ratio_ * comp_ratio)
                {
                    right_hip_roll_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, right_hip_roll_first, 0.0, 0.0);
                    right_hip_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, right_hip_pitch_first, 0.0, 0.0);
                    right_ank_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, right_ank_pitch_first, 0.0, 0.0);
                }
                else if (walking_phase_ <= dsp_ratio_)
                {
                    right_hip_roll_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, right_hip_roll_first, 0.0, 0.0, 0.0);
                    right_hip_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, right_hip_pitch_first, 0.0, 0.0, 0.0);
                    right_ank_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, right_ank_pitch_first, 0.0, 0.0, 0.0);
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
            if (foot_contact_ == 1) //left support foot
            {
                if (walking_phase_ <= dsp_ratio_ * comp_ratio)
                {
                    left_hip_roll_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, left_hip_roll, 0.0, 0.0);
                    left_hip_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, left_hip_pitch, 0.0, 0.0);
                    left_ank_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, left_ank_pitch, 0.0, 0.0);
                }
                else if (walking_phase_ <= dsp_ratio_)
                {
                    left_hip_roll_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, left_hip_roll, 0.0, 0.0, 0.0);
                    left_hip_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, left_hip_pitch, 0.0, 0.0, 0.0);
                    left_ank_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, left_ank_pitch, 0.0, 0.0, 0.0);
                }
                else
                {
                    left_hip_roll_temp = 0.0;
                    left_hip_pitch_temp = 0.0;
                    left_ank_pitch_temp = 0.0;
                }
            }
            else // right support foot
            {
                if (walking_phase_ <= dsp_ratio_ * comp_ratio)
                {
                    right_hip_roll_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, right_hip_roll, 0.0, 0.0);
                    right_hip_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, right_hip_pitch, 0.0, 0.0);
                    right_ank_pitch_temp = DyrosMath::cubic(walking_phase_, 0, dsp_ratio_ * comp_ratio, 0.0, right_ank_pitch, 0.0, 0.0);
                }
                else if (walking_phase_ <= dsp_ratio_)
                {
                    right_hip_roll_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, right_hip_roll, 0.0, 0.0, 0.0);
                    right_hip_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, right_hip_pitch, 0.0, 0.0, 0.0);
                    right_ank_pitch_temp = DyrosMath::cubic(walking_phase_, dsp_ratio_ * (1 - comp_ratio), dsp_ratio_, right_ank_pitch, 0.0, 0.0, 0.0);
                }
                else
                {
                    right_hip_roll_temp = 0.0;
                    right_hip_pitch_temp = 0.0;
                    right_ank_pitch_temp = 0.0;
                }
            }
        }
    }

    compensated_desired_q = desired_q;
    compensated_desired_q(1) = desired_q(1) - left_hip_roll_temp;
    compensated_desired_q(7) = desired_q(7) + right_hip_roll_temp;
    compensated_desired_q(2) = desired_q(2) - left_hip_pitch_temp;
    compensated_desired_q(8) = desired_q(8) - right_hip_pitch_temp;
    compensated_desired_q(4) = desired_q(4) - left_ank_pitch_temp;
    compensated_desired_q(10) = desired_q(10) - right_ank_pitch_temp;

    return compensated_desired_q;
}

Eigen::VectorQd AvatarController::dampingControlCompute()
{
    VectorQd torque;
    torque.setZero();
    ////////////////////support angle operational space damping////////////////////////////////////////
    f_lfoot_damping_ = -support_foot_damping_gain_ * lfoot_vel_current_from_global_.segment(3, 3);
    f_rfoot_damping_ = -support_foot_damping_gain_ * rfoot_vel_current_from_global_.segment(3, 3);

    if (foot_swing_trigger_ == true)
    {
        if (foot_contact_ == 1)
        {
            f_lfoot_damping_ = 0.7 * f_lfoot_damping_ + 0.3 * f_lfoot_damping_pre_;
            f_rfoot_damping_ = 0.3 * f_rfoot_damping_pre_;
        }
        else if (foot_contact_ == -1)
        {
            f_rfoot_damping_ = 0.7 * f_rfoot_damping_ + 0.3 * f_rfoot_damping_pre_;
            f_lfoot_damping_ = 0.3 * f_lfoot_damping_pre_;
        }
    }
    else
    {
        f_lfoot_damping_ = 0.7 * f_lfoot_damping_ + 0.3 * f_lfoot_damping_pre_;
        f_rfoot_damping_ = 0.7 * f_rfoot_damping_ + 0.3 * f_rfoot_damping_pre_;
    }

    torque(4) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_rfoot_damping_)(4);
    torque(5) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_rfoot_damping_)(5);
    torque(10) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_rfoot_damping_)(10);
    torque(11) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose() * f_rfoot_damping_)(11);
    /////////////////////////////////////////////////////////////////////////////////

    ///////////////////joint angle damping///////////////////////////////////////////
    torque(1) += -0.1 * current_q_dot_(1) * swingfoot_force_control_converter_; // left hip roll
    torque(2) += -0.1 * current_q_dot_(2) * swingfoot_force_control_converter_; // left hip pitch
    torque(7) += -0.1 * current_q_dot_(7) * swingfoot_force_control_converter_; // right hip roll
    torque(8) += -0.1 * current_q_dot_(8) * swingfoot_force_control_converter_; // right hip pitch

    // torque(3) += -20*current_q_dot_(3);
    // torque(9) += -20*current_q_dot_(9);

    // torque(4) += -20*current_q_dot_(4);
    // torque(5) += -50*current_q_dot_(5);
    // torque(10) += -20*current_q_dot_(10);
    // torque(11) += -50*current_q_dot_(11);
    /////////////////////////////////////////////////////////////////////////////////

    ////////////////joint limit avoidance control/////////////////////////////////////////

    return torque;
}

Eigen::VectorQd AvatarController::ikBalanceControlCompute()
{
    //real
    // double kp_com = 0.2;
    // double kp_zmp = 0.04;
    //sim
    double kp_com = 0.7;
    double kp_zmp = 0.1;
    Eigen::Isometry3d lfoot_transform_desired;
    Eigen::Isometry3d rfoot_transform_desired;
    Eigen::Isometry3d pelv_transform_desired;
    Vector12d q_leg_desired;
    VectorQd torque_output, torque_g;
    double pd_control_schedule, falling_detection;
    pd_control_schedule = DyrosMath::cubic(current_time_, program_start_time_, program_start_time_ + 3, 0, 1, 0, 0);
    q_leg_desired.setZero();
    torque_output.setZero();

    lfoot_transform_desired = lfoot_transform_start_from_global_;
    rfoot_transform_desired = rfoot_transform_start_from_global_;

    pelv_transform_desired.translation().setZero();
    // pelv_transform_desired.translation() = (lfoot_transform_init_from_global_.translation()+ rfoot_transform_init_from_global_.translation())/2;
    // pelv_transform_desired.translation()(2) = 0;
    pelv_transform_desired.linear() = pelv_rot_current_yaw_aline_;

    // lfoot_transform_desired.translation()(0) = DyrosMath::cubic(current_time_, program_start_time_, program_start_time_ + 3, lfoot_transform_init_from_global_.translation()(0), 0, 0, 0);
    // rfoot_transform_desired.translation()(0) = DyrosMath::cubic(current_time_, program_start_time_, program_start_time_ + 3, rfoot_transform_init_from_global_.translation()(0), 0, 0, 0);

    // lfoot_transform_desired.linear().setIdentity();
    // rfoot_transform_desired.linear().setIdentity();
    // pelv_transform_desired.linear().setIdentity();
    lfoot_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, lfoot_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());
    rfoot_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, rfoot_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());
    pelv_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, pelv_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());

    // pelv_transform_desired.translation() += kp_com*(com_pos_desired_ - com_pos_current_) - kp_zmp*(middle_of_both_foot_ - zmp_measured_);
    com_pos_desired_(0) = com_pos_init_(0);
    com_pos_desired_(1) = com_pos_init_(1);

    pelv_transform_desired.translation() += kp_compos_ * (com_pos_desired_ - com_pos_current_) - kd_compos_ * (middle_of_both_foot_ - zmp_measured_);
    computeIk(pelv_transform_desired, lfoot_transform_desired, rfoot_transform_desired, q_leg_desired);

    WBC::SetContact(rd_, 1, 1);
    torque_g = WBC::GravityCompensationTorque(rd_);
    VectorQd cfrd_torque_g;
    Vector12d redistribution_force;
    double eta = 0;

    cfrd_torque_g.setZero();
    cfrd_torque_g = WBC::ContactForceRedistributionTorqueWalking(rd_, torque_g);

    for (int i = 0; i < 12; i++)
    {
        torque_output(i) = pd_control_schedule * (kp_joint_(i) * (q_leg_desired(i) - current_q_(i)) - kv_joint_(i) * current_q_dot_(i)) + torque_g(i) + cfrd_torque_g(i);
    }

    return torque_output;
}

void AvatarController::computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d &q_des)
{
    //float = World/ trunk = pelvis
    // 명주 정리 (KAJITA 책 <-> Code 구성)
    // float_trunk_transform.rotation() : float에서 바라본 pelvis rotation -> R1
    // float_trunk_transform.translation() : float에서 바라본 pelvis 좌표 -> P1
    // float_rleg_transform.rotation() : float에서 바라본 발끝 rotation -> R7
    // float_rleg_transform.translation() : float에서 바라본 발끝 좌표 -> P7
    // float_trunk_transform.translation() + float_trunk_transform.rotation()*D  : float에서 바라본 pelvis 좌표 + float 좌표계로 변환 * pelvis 좌표계에서 바라본 pelvis ~ hip까지 거리-> P2

    // R7.transpose * (P2 - P7) , P2 = P1 + R1*D

    Eigen::Vector3d R_r, R_D, L_r, L_D;

    // L_D << 0, 0.1025, -0.1225;
    // R_D << 0, -0.1025, -0.1225;
    L_D << 0.11, 0.1025, -0.1025;
    R_D << 0.11, -0.1025, -0.1025;

    L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * L_D - float_lleg_transform.translation());
    R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * R_D - float_rleg_transform.translation());

    double R_C = 0, L_C = 0, L_upper = 0.35, L_lower = 0.35, R_alpha = 0, L_alpha = 0;
    double L_max = L_upper + L_lower;
    L_r(2) = DyrosMath::minmax_cut(L_r(2), 0.2, L_max);
    R_r(2) = DyrosMath::minmax_cut(R_r(2), 0.2, L_max);

    R_C = R_r.norm();
    L_C = L_r.norm();

    //   L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
    if (foot_swing_trigger_ == true)
    {
        if (foot_contact_ == 1)
        {
            if (R_C > L_max)
            {
                double mapping_xy = sqrt((pow(L_max, 2) - pow(R_r(2), 2)) / (pow(R_r(0), 2) + pow(R_r(1), 2))); //xy mapping
                // double mapping_xy =  (L_max - 1e-2)/R_r.norm();
                R_r(0) *= mapping_xy;
                R_r(1) *= mapping_xy;
                // R_r(2) *= mapping_xy;

                // R_r.normalize();	//test
                // R_r *= L_max;

                R_C = L_max;
                if (int(current_time_ * 1000) % 500 == 1)
                {
                    cout << "Swing leg tajectory is out of the workspace" << endl;
                }
            }
        }
        else if (foot_contact_ == -1)
        {
            if (L_C > L_max)
            {
                double mapping_xy = sqrt((pow(L_max, 2) - pow(L_r(2), 2)) / (pow(L_r(0), 2) + pow(L_r(1), 2)));
                // double mapping_xy =  (L_max - 1e-2)/L_r.norm();
                L_r(0) *= mapping_xy;
                L_r(1) *= mapping_xy;
                // L_r(2) *= mapping_xy;

                // L_r.normalize();	//test
                // L_r *= L_max;

                L_C = L_max;
                if (int(current_time_ * 1000) % 500 == 1)
                {
                    cout << "Swing leg tajectory is out of the workspace" << endl;
                }
            }
        }
    }

    double temp_q_des;
    temp_q_des = (pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2)) / (2 * L_upper * L_lower);
    temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1.0, 1.0);
    q_des(3) = abs(-acos(temp_q_des) + M_PI);
    temp_q_des = (pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2)) / (2 * L_upper * L_lower);
    temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1.0, 1.0);
    q_des(9) = abs(-acos(temp_q_des) + M_PI);
    L_alpha = asin(DyrosMath::minmax_cut(L_upper / L_C * sin(M_PI - q_des(3)), -1.0, 1.0));
    //   L_alpha = q_des(3)/2;
    R_alpha = asin(DyrosMath::minmax_cut(L_upper / R_C * sin(M_PI - q_des(9)), -1.0, 1.0));
    //   R_alpha = q_des(9)/2;

    double temp_q_des_4;
    temp_q_des_4 = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2)));
    if (temp_q_des_4 > M_PI / 2)
    {
        temp_q_des_4 -= M_PI;
    }
    else if (temp_q_des_4 < -M_PI / 2)
    {
        temp_q_des_4 += M_PI;
    }
    q_des(4) = temp_q_des_4 - L_alpha;

    double temp_q_des_10;
    temp_q_des_10 = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2)));
    if (temp_q_des_10 > M_PI / 2)
    {
        temp_q_des_10 -= M_PI;
    }
    else if (temp_q_des_10 < -M_PI / 2)
    {
        temp_q_des_10 += M_PI;
    }
    q_des(10) = temp_q_des_10 - R_alpha;
    //   q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha ;

    //   if( walking_phase_ > 0.1 )
    {
        // cout<<"L_C: "<<L_C<<endl;
        // cout<<"R_C: "<<R_C<<endl;

        // cout<<"L_r: "<<L_r<<endl;
        // cout<<"R_r: "<<R_r<<endl;
        // cout<<"q_des(4): "<<q_des(4)<<endl;
        // cout<<"L_alpha: "<<L_alpha<<endl;
        // cout<<"q_des(10): "<<q_des(10)<<endl;
    }

    // trunk_lleg_rotation -> R1.transpose * R7
    // Ryaw * Rroll * Rpitch = R1.transpose * R7 * ~
    q_des(11) = atan2(R_r(1), R_r(2));
    if (q_des(11) > M_PI / 2)
    {
        q_des(11) -= M_PI;
    }
    else if (q_des(11) < -M_PI / 2)
    {
        q_des(11) += M_PI;
    }

    q_des(5) = atan2(L_r(1), L_r(2)); // Ankle roll
    if (q_des(5) > M_PI / 2)
    {
        q_des(5) -= M_PI;
    }
    else if (q_des(5) < -M_PI / 2)
    {
        q_des(5) += M_PI;
    }

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
    Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

    L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3) - q_des(4));
    L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
    R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9) - q_des(10));
    R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11));

    L_Hip_rot_mat.setZero();
    R_Hip_rot_mat.setZero();

    L_Hip_rot_mat = float_trunk_transform.linear().transpose() * float_lleg_transform.linear() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
    R_Hip_rot_mat = float_trunk_transform.linear().transpose() * float_rleg_transform.linear() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

    q_des(0) = -atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1)); // Hip yaw
                                                                  //   q_des(1) =  atan2(L_Hip_rot_mat(2,1), -L_Hip_rot_mat(0,1) * sin(q_des(0)) + L_Hip_rot_mat(1,1)*cos(q_des(0))); // Hip roll
    q_des(1) = asin(DyrosMath::minmax_cut(L_Hip_rot_mat(2, 1), -1.0, 1.0));
    q_des(2) = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2)); // Hip pitch
    q_des(3) = q_des(3);                                         // Knee pitch
    q_des(4) = q_des(4);                                         // Ankle pitch

    //   cout<<"L_Hip_rot_mat: \n"<<L_Hip_rot_mat<<endl;
    //   cout<<"R_Hip_rot_mat: \n"<<R_Hip_rot_mat<<endl;

    if (q_des(0) > M_PI / 2)
    {
        q_des(0) -= M_PI;
    }
    else if (q_des(0) < -M_PI / 2)
    {
        q_des(0) += M_PI;
    }

    if (q_des(1) > M_PI / 2)
    {
        q_des(1) -= M_PI;
    }
    else if (q_des(1) < -M_PI / 2)
    {
        q_des(1) += M_PI;
    }

    if (q_des(2) > M_PI / 2)
    {
        q_des(2) -= M_PI;
    }
    else if (q_des(2) < -M_PI / 2)
    {
        q_des(2) += M_PI;
    }

    q_des(6) = -atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    //   q_des(7) =  atan2(R_Hip_rot_mat(2,1), -R_Hip_rot_mat(0,1) * sin(q_des(6)) + R_Hip_rot_mat(1,1)*cos(q_des(6)));
    q_des(7) = asin(DyrosMath::minmax_cut(R_Hip_rot_mat(2, 1), -1.0, 1.0));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);

    if (q_des(6) > M_PI / 2)
    {
        q_des(6) -= M_PI;
    }
    else if (q_des(6) < -M_PI / 2)
    {
        q_des(6) += M_PI;
    }

    if (q_des(7) > M_PI / 2)
    {
        q_des(7) -= M_PI;
    }
    else if (q_des(7) < -M_PI / 2)
    {
        q_des(7) += M_PI;
    }

    if (q_des(8) > M_PI / 2)
    {
        q_des(8) -= M_PI;
    }
    else if (q_des(8) < -M_PI / 2)
    {
        q_des(8) += M_PI;
    }
}

Eigen::VectorQd AvatarController::jointLimit()
{
    Eigen::VectorQd joint_limit_torque;
    joint_limit_torque.setZero();

    return joint_limit_torque;
}

Eigen::VectorXd AvatarController::momentumObserver(VectorXd current_momentum, VectorXd current_torque, VectorXd nonlinear_term, VectorXd mob_residual_pre, double dt, double k)
{
    //input: current_momentum, current_torque, nonlinear term, mob_residual_pre, dt, k
    //output: mob_residual
    const int dof = current_momentum.size();
    Eigen::VectorXd mob_residual(dof);
    mob_integral_ = mob_integral_ + dt * (current_torque + nonlinear_term + mob_residual_pre);
    mob_residual = k * (current_momentum - mob_integral_);

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
        if (i >= 3 & i < 6) //quaternion exeption
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
            w2_camhqp_[0] = 50.0; // |q_dot| 
            
            w1_camhqp_[1] = 2500; // |q_dot - q_dot_zero|
            w2_camhqp_[1] = 0.0; // |q_dot| 
        }
         
        control_joint_idx_camhqp_[0] = 13; // waist pitch
        control_joint_idx_camhqp_[1] = 14; // waist roll

        control_joint_idx_camhqp_[2] = 15; // left shoulder yaw 
        control_joint_idx_camhqp_[3] = 16; // left shoulder pitch
        control_joint_idx_camhqp_[4] = 17; // left shoulder roll
        control_joint_idx_camhqp_[5] = 18; // left elbow yaw

        control_joint_idx_camhqp_[6] = 25; // right shoulder yaw
        control_joint_idx_camhqp_[7] = 26; // right shoulder pitch
        control_joint_idx_camhqp_[8] = 27; // right shoulder roll      
        control_joint_idx_camhqp_[9] = 28; // right elbow yaw       

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
    q_test.segment(18, 21) = motion_q_pre_.segment(12,21);
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

    Eigen::Vector2d del_ang_momentum_slow_2;
    del_ang_momentum_slow_2 = del_ang_momentum_slow_.segment(0, 2); 
    J_camhqp_[0] = cmm_selected;
    
    u_dot_camhqp_[0] = del_ang_momentum_slow_2;
    J_camhqp_[1].setIdentity(variable_size_camhqp_, variable_size_camhqp_);
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

    last_solved_hierarchy_num_camhqp_ = -1;
    for (int i = 0; i < hierarchy_num_camhqp_; i++)
    {
        MatrixXd H1, H2;
        VectorXd g1, g2;

        H1 = J_camhqp_[i].transpose() * J_camhqp_[i];
        H2 = Eigen::MatrixXd::Identity(variable_size_camhqp_, variable_size_camhqp_);
        
        g1 = -J_camhqp_[i].transpose() * u_dot_camhqp_[i]; // (variable_size_camhqp_ x 1 (i.e. 6x1))
        g2.setZero(variable_size_camhqp_);  
          
        H_camhqp_[i] = w1_camhqp_[i] * H1 + w2_camhqp_[i] * H2 ; 
        g_camhqp_[i] = w1_camhqp_[i] * g1 + w2_camhqp_[i] * g2 ; 

        double speed_reduce_rate = 20; // simul : 40 -> exp: 20 // IROS22 : 15
            
        // MJ's joint limit 
        lb_camhqp_[i](0) = min(max(speed_reduce_rate * (-15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[0])), joint_vel_limit_l_(control_joint_idx_camhqp_[0])), joint_vel_limit_h_(control_joint_idx_camhqp_[0])); // simul:-20.0 -> exp: -15.0
        ub_camhqp_[i](0) = max(min(speed_reduce_rate * ( 15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[0])), joint_vel_limit_h_(control_joint_idx_camhqp_[0])), joint_vel_limit_l_(control_joint_idx_camhqp_[0])); // simul: 30.0 -> exp: 15.0
        lb_camhqp_[i](1) = min(max(speed_reduce_rate * (-15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[1])), joint_vel_limit_l_(control_joint_idx_camhqp_[1])), joint_vel_limit_h_(control_joint_idx_camhqp_[1])); // simul: -20.0 -> exp: -15.0
        ub_camhqp_[i](1) = max(min(speed_reduce_rate * ( 15.0*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[1])), joint_vel_limit_h_(control_joint_idx_camhqp_[1])), joint_vel_limit_l_(control_joint_idx_camhqp_[1])); // simul: 20.0 -> exp: 15.0
        // Left Shoulder yaw  
        lb_camhqp_[i](2) = min(max(speed_reduce_rate * (-30*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[2])), joint_vel_limit_l_(control_joint_idx_camhqp_[2])), joint_vel_limit_h_(control_joint_idx_camhqp_[2])); // unchanged
        ub_camhqp_[i](2) = max(min(speed_reduce_rate * (+20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[2])), joint_vel_limit_h_(control_joint_idx_camhqp_[2])), joint_vel_limit_l_(control_joint_idx_camhqp_[2])); // unchanged
        // Left Shoulder pitch // Init 17 deg, CAM 10 deg // Roll 방향 test -20~20?
        lb_camhqp_[i](3) = min(max(speed_reduce_rate * (-20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[3])), joint_vel_limit_l_(control_joint_idx_camhqp_[3])), joint_vel_limit_h_(control_joint_idx_camhqp_[3])); // simul:-50.0 -> exp: -40.0
        ub_camhqp_[i](3) = max(min(speed_reduce_rate * (20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[3])), joint_vel_limit_h_(control_joint_idx_camhqp_[3])), joint_vel_limit_l_(control_joint_idx_camhqp_[3])); // simul: 50.0 -> exp: 40.0
        // Left Shoulder roll // Init 86 deg, CAM 75 deg // 80 deg 보다 크면 몸통 부딪힘.  
        lb_camhqp_[i](4) = min(max(speed_reduce_rate * (45*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[4])), joint_vel_limit_l_(control_joint_idx_camhqp_[4])), joint_vel_limit_h_(control_joint_idx_camhqp_[4])); // unchanged
        ub_camhqp_[i](4) = max(min(speed_reduce_rate * (65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[4])), joint_vel_limit_h_(control_joint_idx_camhqp_[4])), joint_vel_limit_l_(control_joint_idx_camhqp_[4])); // unchanged
        // Left elbow yaw // Init -72 deg, CAM -70 deg +40 deg 보다 크면 왼쪽 옆구리에 부딪힘. 
        lb_camhqp_[i](5) = min(max(speed_reduce_rate * (-90*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[5])), joint_vel_limit_l_(control_joint_idx_camhqp_[5])), joint_vel_limit_h_(control_joint_idx_camhqp_[5])); // unchanged
        ub_camhqp_[i](5) = max(min(speed_reduce_rate * (-65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[5])), joint_vel_limit_h_(control_joint_idx_camhqp_[5])), joint_vel_limit_l_(control_joint_idx_camhqp_[5])); // unchanged

        // Right Shoulder yaw      // Y dir disturbance -> yaw joint, pitch joint singular 느낌.  Y dir일때 pitch joint 거의없애야할듯..
        lb_camhqp_[i](6) = min(max(speed_reduce_rate * (-20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[6])), joint_vel_limit_l_(control_joint_idx_camhqp_[6])), joint_vel_limit_h_(control_joint_idx_camhqp_[6])); // unchanged
        ub_camhqp_[i](6) = max(min(speed_reduce_rate * (+30*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[6])), joint_vel_limit_h_(control_joint_idx_camhqp_[6])), joint_vel_limit_l_(control_joint_idx_camhqp_[6])); // unchanged
        // Right Shoulder pitch   
        lb_camhqp_[i](7) = min(max(speed_reduce_rate * (-20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[7])), joint_vel_limit_l_(control_joint_idx_camhqp_[7])), joint_vel_limit_h_(control_joint_idx_camhqp_[7])); // simul:-50.0 -> exp: -40.0
        ub_camhqp_[i](7) = max(min(speed_reduce_rate * (+20*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[7])), joint_vel_limit_h_(control_joint_idx_camhqp_[7])), joint_vel_limit_l_(control_joint_idx_camhqp_[7])); // simul: 50.0 -> exp: 40.0
        // Right Shoulder roll
        lb_camhqp_[i](8) = min(max(speed_reduce_rate * (-65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[8])), joint_vel_limit_l_(control_joint_idx_camhqp_[8])), joint_vel_limit_h_(control_joint_idx_camhqp_[8])); // unchanged
        ub_camhqp_[i](8) = max(min(speed_reduce_rate * (-45*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[8])), joint_vel_limit_h_(control_joint_idx_camhqp_[8])), joint_vel_limit_l_(control_joint_idx_camhqp_[8])); // unchanged
        // Right elbow yaw 
        lb_camhqp_[i](9) = min(max(speed_reduce_rate * (+65*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[9])), joint_vel_limit_l_(control_joint_idx_camhqp_[9])), joint_vel_limit_h_(control_joint_idx_camhqp_[9])); // unchanged
        ub_camhqp_[i](9) = max(min(speed_reduce_rate * (+90*DEG2RAD - motion_q_pre_(control_joint_idx_camhqp_[9])), joint_vel_limit_h_(control_joint_idx_camhqp_[9])), joint_vel_limit_l_(control_joint_idx_camhqp_[9])); // unchanged
                      
        int higher_task_equality_num = 0;        
     
        for(int k = 0; k < constraint_size2_camhqp_[1]; k++)
        {
            eps(k) = DyrosMath::cubic(del_ang_momentum_slow_2.norm(), 1, 3, 0.7, 0.0, 0.0, 0.0); // 빠르게 돌리고 싶을때 늘리기..
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
        }
 
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
            
            std::cout << "Error hierarchy: " << i << std::endl;
            std::cout << "last solved q_dot: " << q_dot_camhqp_[last_solved_hierarchy_num_camhqp_].transpose() << std::endl;
             
            break;
        }
    }
        
    for (int i = 0; i < variable_size_camhqp_; i++)
    {
        //motion_q_dot_(control_joint_idx_camhqp_[i]) = q_dot_camhqp_[0](i); // first hierarchy solution
        motion_q_dot_(control_joint_idx_camhqp_[i]) = q_dot_camhqp_[last_solved_hierarchy_num_camhqp_](i);
        motion_q_(control_joint_idx_camhqp_[i]) = motion_q_pre_(control_joint_idx_camhqp_[i]) + motion_q_dot_(control_joint_idx_camhqp_[i]) * 0.0005;
        pd_control_mask_(control_joint_idx_camhqp_[i]) = 1;
    }
    // MJ_graph2 << motion_q_(25) << "," << motion_q_(26) << "," << motion_q_(27) << "," << motion_q_(28) << "," << motion_q_(29) << endl;

    // // CAM calculation based on actual joint angular velocity    
    // Eigen::VectorXd cam_a;
    // Eigen::VectorXd real_q_dot_hqp_; 
    // cam_a.setZero(3);
    // real_q_dot_hqp_.setZero(8);
    
    // for(int i = 0; i < variable_size_camhqp_; i++)
    // {
    //   real_q_dot_hqp_(i) = rd_.q_dot_(control_joint_idx_camhqp_[i]);  
    // }
    // cam_a = -J_camhqp_[0]*real_q_dot_hqp_;

    // // CAM calculation based on commanded joint angular velocity    
    Eigen::VectorXd cam_c;
    Eigen::VectorXd cmd_q_dot_hqp_;
    cam_c.setZero(3);
    cmd_q_dot_hqp_.setZero(10);

    for(int i = 0; i < variable_size_camhqp_; i++)
    {
      cmd_q_dot_hqp_(i) = motion_q_dot_(control_joint_idx_camhqp_[i]);  
    }
    cam_c = -J_camhqp_[0]*cmd_q_dot_hqp_;
      
}   

// void AvatarController::CPMPC_bolt_Controller_MJ_ICRA()
// {   
//     std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//     //Setting up nominal values w.r.t current footstep
 
//     // L : del_F_x , W : del_F_y 
    
//     double L_nom = 0.0;
//     double L_min = 0.0; 
//     double L_max = 0.0; 

//     double W_nom = 0.0;
//     double W_min = 0.0;
//     double W_max = 0.0;

//     double T_nom = 0.0;
//     double T_min = 0.0; 
//     double T_max = 0.0;
//     double tau_nom = 0.0;
        
//     // 500, 5000, 1, 500, 5000
//     double w1_step = w_ux_temp_, w2_step = w_uy_temp_, w3_step = w_time_temp_, w4_step = w_bx_temp_, w5_step = w_by_temp_;

//     //double w1_step = 1.0, w2_step = 0.02, w3_step = 3.0; // real robot experiment
//     double u0_x = 0, u0_y = 0;   
//     double b_nom_x_cpmpc = 0, b_nom_y_cpmpc = 0;
//      double b_nom_x = 0, b_nom_y = 0;
//     if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
//     {
//         del_F_x_ = 0;
//         del_F_y_ = 0;
//         u0_x = 0;
//         u0_y = 0;        
//     }
//     else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
//     {
//         del_F_x_ = 0;
//         del_F_y_ = 0;
//         u0_x = 0;
//         u0_y = 0;
//     }

//     L_nom = foot_step_support_frame_(current_step_num_, 0) + del_F_x_;  
//     W_nom = foot_step_support_frame_(current_step_num_, 1) + del_F_y_;  
//     L_min = L_nom - 0.05; // 0.05
//     L_max = L_nom + 0.05;
//     W_min = W_nom - 0.05;
//     W_max = W_nom + 0.05; 
    
//     T_nom = (t_total_const_ - (t_rest_init_ + t_rest_last_ + t_double1_ + t_double2_))/hz_; // 0.6하면 370 못버팀.
//     T_min = T_nom - 0.2;  
//     T_max = T_nom + 0.2;
//     tau_nom = exp(wn*T_nom); 

//     // Bolt
//     // b_nom_x = L_nom/(exp(wn*T_nom)-1); 
//     // b_nom_y = l_p/(1 + exp(wn*T_nom)) - W_nom/(1 - exp(wn*T_nom));
//     // Wieber
//     b_nom_x = cp_eos_x_mpc_ - L_nom; 
//     b_nom_y = cp_eos_y_mpc_ - W_nom;

//     double cp_eos_x_cpmpc_temp = 0, cp_eos_y_cpmpc_temp = 0;

//     cp_eos_x_cpmpc_temp = DyrosMath::minmax_cut(cp_eos_x_cpmpc_, -0.25, 0.25);
//     cp_eos_y_cpmpc_temp = DyrosMath::minmax_cut(cp_eos_y_cpmpc_, -0.25, 0.25); 

//     b_nom_x_cpmpc = cp_eos_x_cpmpc_temp - L_nom;
//     b_nom_y_cpmpc = cp_eos_y_cpmpc_temp - W_nom;
    
//     Eigen::MatrixXd H_step;
//     Eigen::VectorXd g_step; 
    
//     H_step.setZero(5,5);
//     H_step(0,0) = w1_step; // U_T,x (step position in x-direction)
//     H_step(1,1) = w2_step; // U_T,y (step position in y-direction) // 200
//     H_step(2,2) = w3_step; // tau (step timing)
//     H_step(3,3) = w4_step; // DCM offset in x
//     H_step(4,4) = w5_step; // DCM offset in y // 0.01
    
//     g_step.setZero(5);
//     g_step(0) = -w1_step * (u0_x + L_nom);
//     g_step(1) = -w2_step * (u0_y + W_nom); 
//     g_step(2) = -w3_step * tau_nom;
//     g_step(3) = -w4_step * b_nom_x;  
//     g_step(4) = -w5_step * b_nom_y;  

//     Eigen::VectorXd lb_step;
//     Eigen::VectorXd ub_step;
//     Eigen::MatrixXd A_step;         

//     double stepping_start_time = 0;
    
//     if (current_step_num_ == 0)
//     {
//         stepping_start_time = t_start_ + t_double1_ + t_rest_init_;
//     }
//     else
//     {
//         stepping_start_time = t_start_ + t_double1_ + t_rest_init_;
//     }    
    
//     A_step.setZero(7,5);

//     A_step(0,0) = 1; // U_T,x
//     A_step(0,1) = 0; // U_T,y
//     A_step(0,2) = -(cp_measured_(0)-u0_x)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
//     A_step(0,3) = 1; // b_x
//     A_step(0,4) = 0; // b_y

//     A_step(1,0) = 0; // U_T,x
//     A_step(1,1) = 1; // U_T,y 
//     A_step(1,2) = -(cp_measured_(1)-u0_y)*exp(-wn*(walking_tick_mj - stepping_start_time)/hz_); // tau
//     A_step(1,3) = 0; // b_x
//     A_step(1,4) = 1; // b_y

//     A_step(2,0) = 1; // U_T,x

//     A_step(3,1) = 1; // U_T,y

//     A_step(4,2) = 1; // tau

//     A_step(5,3) = 1; // b_x
//     A_step(6,4) = 1; // b_x

//     lb_step.setZero(7);
//     ub_step.setZero(7);

//     lb_step(0) = u0_x;
//     lb_step(1) = u0_y;
//     lb_step(2) = u0_x + L_min;
//     lb_step(3) = u0_y + W_min;
//     lb_step(4) = exp(wn*T_min);
//     lb_step(5) = b_nom_x - 0.15; 
//     lb_step(6) = b_nom_y - 0.15;
    
//     ub_step(0) = u0_x;
//     ub_step(1) = u0_y;
//     ub_step(2) = u0_x + L_max;
//     ub_step(3) = u0_y + W_max;
//     ub_step(4) = exp(wn*T_max);
//     ub_step(5) = b_nom_x + 0.15; 
//     ub_step(6) = b_nom_y + 0.15;    
    
//     if(walking_tick_mj == 0)
//     {
//         stepping_input_.setZero(5);
//         stepping_input_(0) = foot_step_support_frame_(current_step_num_, 0);
//         stepping_input_(1) = foot_step_support_frame_(current_step_num_, 1);
//         stepping_input_(2) = T_nom;
//     }     
        
//     if(current_step_num_ > 0 && (current_step_num_ != total_step_num_-1))
//     {   // Solving the QP during only SSP
//         if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
//         {
//             QP_stepping_.InitializeProblemSize(5, 7);
//             QP_stepping_.EnableEqualityCondition(equality_condition_eps_);
//             QP_stepping_.UpdateMinProblem(H_step, g_step);
//             QP_stepping_.DeleteSubjectToAx();      
//             QP_stepping_.UpdateSubjectToAx(A_step, lb_step, ub_step);
        
//             if(QP_stepping_.SolveQPoases(200, stepping_input))
//             {   
//                 stepping_input_ = stepping_input.segment(0, 5);
//             }
//             else
//             {
//                 cout << "bolt is not solved." << endl;
//             }
//         }

//         if(stepping_input_(2) != 0)
//         {
//             if(walking_tick_mj - stepping_start_time < round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ - zmp_modif_time_margin_ - 1 )
//             {           
//                 t_total_ = round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ + t_rest_init_ + t_double1_ + t_rest_last_ + t_double2_;
//                 t_total_ = DyrosMath::minmax_cut(t_total_, t_total_const_ - 0.2*hz_, t_total_const_ + 0.2*hz_);
//                 // t_total_ = 0.8*hz_;
//                 t_last_ = t_start_ + t_total_ - 1;
//             }
//         }
//     }  
    
//     if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
//     {
//         stepping_input_(0) = L_nom;
//         stepping_input_(1) = W_nom;
//         stepping_input_(2) = T_nom;
//     }
//     else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
//     {
//         stepping_input_(0) = del_F_(0);
//         stepping_input_(1) = del_F_(1);
//     }

//     if(walking_tick_mj > t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - (t_rest_last_ + t_double2_) )
//     {
//         MJ_graph << del_F_(0) << "," << u0_x + L_nom << "," << stepping_input_(3) << "," << b_nom_x_cpmpc << "," << t_total_/hz_ << endl;
//         MJ_graph1 << del_F_(1) << "," << u0_y + W_nom << "," << stepping_input_(4) << "," << b_nom_y_cpmpc << endl;
//     }
//     MJ_graph2 << cp_desired_(0) << "," << cp_measured_(0) << "," << cp_measured_(1) << "," << cp_measured_(1) << endl;

//     // MJ_graph << b_nom_x_cpmpc << "," << b_nom_y_cpmpc << "," << stepping_input_(0) << "," << stepping_input_(1) << "," << t_total_/hz_ << endl;
//     // MJ_graph2 << t_total_ << "," << t_rest_init_ << "," << t_rest_last_ << "," << dsp_scaler_(0) << "," << dsp_scaler_(1) << "," << dsp_time_reducer_ << "," << dsp_time_reducer_fixed_ << endl;
//     del_F_(0) = stepping_input_(0);
//     del_F_(1) = stepping_input_(1);
//     //MJ_graph << del_F_(0) << "," << del_F_(1) << "," << del_F_x_ << "," << del_F_y_ << "," << stepping_input_(3) << "," << stepping_input_(4) << "," << t_total_/hz_ << endl;
//     // MJ_graph << cp_eos_x_cpmpc_temp << "," << cp_eos_y_cpmpc_temp << endl;
//     std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//     // MJ_graph1 << del_zmp(0) << "," << del_zmp(1) << "," << des_zmp_interpol_(0) << "," << des_zmp_interpol_(1) << "," << ZMP_X_REF_ << "," << ZMP_Y_REF_ << endl;
// }

void AvatarController::CPMPC_bolt_Controller_MJ()
{   
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //Setting up nominal values w.r.t current footstep
 
    // L : del_F_x , W : del_F_y 
    
    double L_nom = 0.0;
    double L_min = 0.0; 
    double L_max = 0.0; 

    double W_nom = 0.0;
    double W_min = 0.0;
    double W_max = 0.0;

    double T_nom = 0.0;
    double T_min = 0.0; 
    double T_max = 0.0;
    double tau_nom = 0.0;
        
    double l_p = 0.0;
    l_p = foot_step_support_frame_(current_step_num_, 1);
    // double w1_step = w_ux_temp_, w2_step = w_uy_temp_, w3_step = w_time_temp_, w4_step = w_bx_temp_, w5_step = w_by_temp_;
    double w1_step = 1000.0, w2_step = 1000.0, w3_step = 1.0, w4_step = 3000.0, w5_step = 3000.0;
    //double w1_step = 1.0, w2_step = 0.02, w3_step = 3.0; w4_step = 200.0, w5_step = 0.03; // ICRA real robot experiment
    double u0_x = 0, u0_y = 0;   
    double b_nom_x_cpmpc = 0, b_nom_y_cpmpc = 0;
    
    // support foot // 어짜피 MPC 제어입력을 쓰는거기 때문에 아래의 minmax_cut이 의미가 없긴함. 혹시나 입력이 튈 경우
    // u0_x = DyrosMath::minmax_cut(des_cmp_ssp_mpc_x_, -0.09 - 0.016, 0.12 + 0.016); 
    // u0_y = DyrosMath::minmax_cut(des_cmp_ssp_mpc_y_, -0.06 - 0.016, 0.06 + 0.016);         

    u0_x = DyrosMath::minmax_cut(des_cmp_ssp_mpc_x_, -0.05 - 0.01, 0.12 + 0.01); 
    u0_y = DyrosMath::minmax_cut(des_cmp_ssp_mpc_y_, -0.07 - 0.01, 0.07 + 0.01); 

    u0_x_data_ = u0_x;
    u0_y_data_ = u0_y;
 
    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        del_F_x_ = 0;
        del_F_y_ = 0;
        u0_x = 0;
        u0_y = 0;        
    }
    else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
    {
        del_F_x_ = 0;
        del_F_y_ = 0;
        u0_x = 0;
        u0_y = 0;
    }

    L_nom = foot_step_support_frame_(current_step_num_, 0) + del_F_x_;     
    W_nom = foot_step_support_frame_(current_step_num_, 1) + del_F_y_;   
    // W_nom = 0*foot_step_support_frame_(current_step_num_, 1) + del_F_y_;  
    L_min = L_nom - 0.15; // 0.05
    L_max = L_nom + 0.15;
    W_min = W_nom - 0.05; // 0.05 in simulation
    W_max = W_nom + 0.05; 
    
    T_nom = (t_total_const_ - (t_rest_init_ + t_rest_last_ + t_double1_ + t_double2_))/hz_; // 0.6하면 370 못버팀.
    T_min = T_nom - 0.2;  
    T_max = T_nom + 0.2;
    tau_nom = exp(wn*T_nom); 

    // Bolt
    // b_nom_x = L_nom/(exp(wn*T_nom)-1); 
    // b_nom_y = l_p/(1 + exp(wn*T_nom)) - W_nom/(1 - exp(wn*T_nom));
    // Wieber
    // b_nom_x = cp_eos_x_mpc_ - L_nom; 
    // b_nom_y = cp_eos_y_mpc_ - W_nom;

    double cp_eos_x_cpmpc_temp = 0, cp_eos_y_cpmpc_temp = 0;

    cp_eos_x_cpmpc_temp = DyrosMath::minmax_cut(cp_eos_x_cpmpc_, -0.25, 0.25);
    cp_eos_y_cpmpc_temp = DyrosMath::minmax_cut(cp_eos_y_cpmpc_, -0.25, 0.25); 

    // b_nom_x_cpmpc = L_nom/(exp(wn*T_nom)-1);
    // b_nom_y_cpmpc = l_p/(1 + exp(wn*T_nom)) - W_nom/(1 - exp(wn*T_nom)); 
    b_nom_x_cpmpc = cp_eos_x_cpmpc_temp - L_nom;
    b_nom_y_cpmpc = cp_eos_y_cpmpc_temp - W_nom;
   
    Eigen::MatrixXd H_step;
    Eigen::VectorXd g_step; 
    
    H_step.setZero(5,5);
    H_step(0,0) = w1_step; // U_T,x (step position in x-direction)
    H_step(1,1) = w2_step; // U_T,y (step position in y-direction) // 200
    H_step(2,2) = w3_step; // tau (step timing)
    H_step(3,3) = w4_step; // DCM offset in x
    H_step(4,4) = w5_step; // DCM offset in y // 0.01
    
    g_step.setZero(5);
    g_step(0) = -w1_step * L_nom;
    g_step(1) = -w2_step * W_nom; 
    g_step(2) = -w3_step * tau_nom;
    g_step(3) = -w4_step * b_nom_x_cpmpc;  
    g_step(4) = -w5_step * b_nom_y_cpmpc;  
    // g_step.setZero(5);
    // g_step(0) = -w1_step * (u0_x + L_nom);
    // g_step(1) = -w2_step * (u0_y + W_nom); 
    // g_step(2) = -w3_step * tau_nom;
    // g_step(3) = -w4_step * b_nom_x_cpmpc;  
    // g_step(4) = -w5_step * b_nom_y_cpmpc;  
    Eigen::VectorXd lb_step;
    Eigen::VectorXd ub_step;
    Eigen::MatrixXd A_step;         

    double stepping_start_time = 0;
    
    if (current_step_num_ == 0)
    {
        stepping_start_time = t_start_ + t_double1_ + t_rest_init_;
    }
    else
    {
        stepping_start_time = t_start_ + t_double1_ + t_rest_init_;
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
    lb_step(2) = L_min;
    lb_step(3) = W_min;
    lb_step(4) = exp(wn*T_min);
    lb_step(5) = b_nom_x_cpmpc - 0.15; 
    lb_step(6) = b_nom_y_cpmpc - 0.15;
    
    ub_step(0) = u0_x;
    ub_step(1) = u0_y;
    ub_step(2) = L_max;
    ub_step(3) = W_max;
    ub_step(4) = exp(wn*T_max);
    ub_step(5) = b_nom_x_cpmpc + 0.15; 
    ub_step(6) = b_nom_y_cpmpc + 0.15;    
    
    // lb_step(0) = u0_x;
    // lb_step(1) = u0_y;
    // lb_step(2) = u0_x + L_min;
    // lb_step(3) = u0_y + W_min;
    // lb_step(4) = exp(wn*T_min);
    // lb_step(5) = b_nom_x_cpmpc - 0.15; 
    // lb_step(6) = b_nom_y_cpmpc - 0.15;
    
    // ub_step(0) = u0_x;
    // ub_step(1) = u0_y;
    // ub_step(2) = u0_x + L_max;
    // ub_step(3) = u0_y + W_max;
    // ub_step(4) = exp(wn*T_max);
    // ub_step(5) = b_nom_x_cpmpc + 0.15; 
    // ub_step(6) = b_nom_y_cpmpc + 0.15;  

    if(walking_tick_mj == 0)
    {
        stepping_input_.setZero(5);
        stepping_input_(0) = foot_step_support_frame_(current_step_num_, 0);
        stepping_input_(1) = foot_step_support_frame_(current_step_num_, 1);
        stepping_input_(2) = T_nom;
    }     
        
    // DSP scaler
    Eigen::Vector2d stepping_err;
    stepping_err.setZero();

    if(walking_tick_mj > t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {        
        stepping_err(1) = target_swing_foot(1) + del_F_(1) - fixed_swing_foot(1);

        if(target_swing_foot(0) + del_F_(0) < 0)
        {
            stepping_err(0) = target_swing_foot(0) + del_F_(0) - fixed_swing_foot(0);
            
            if(stepping_err(0) > 0)
            {
                stepping_err(0) = 0;
            }
        }
        else if(target_swing_foot(0) + del_F_(0) >= 0)
        {
            stepping_err(0) = target_swing_foot(0) + del_F_(0) - fixed_swing_foot(0);

            if(stepping_err(0) < 0)
            {
                stepping_err(0) = 0;
            }
        }     
    }
    else
    {
        stepping_err.setZero();
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
            else
            {
                // cout << "bolt is not solved." << endl;
            }
        }

        if(stepping_input_(2) != 0)
        {
            if(walking_tick_mj - stepping_start_time < round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ - zmp_modif_time_margin_ - 1 )
            {           
                t_total_ = round(log(stepping_input_(2))/wn*1000)/1000.0*hz_ + t_rest_init_ + t_double1_ + t_rest_last_ + t_double2_;
                t_total_ = DyrosMath::minmax_cut(t_total_, t_total_const_ - 0.2*hz_, t_total_const_ + 0.2*hz_);
                // t_total_ = 0.8*hz_;
                t_last_ = t_start_ + t_total_ - 1;
            }
        }
    }   
 
    if(walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        // stepping_input_(0) = 0*L_nom;
        // stepping_input_(1) = 0*W_nom;
        stepping_input_(0) = L_nom;
        stepping_input_(1) = W_nom;
        stepping_input_(2) = T_nom; 
    }
    else if(walking_tick_mj >= t_start_ + t_total_ - (t_rest_last_ + t_double2_) && walking_tick_mj < t_start_ + t_total_ )
    {
        // stepping_input_(0) = 0*del_F_(0);
        // stepping_input_(1) = 0*del_F_(1);
        stepping_input_(0) = del_F_(0);
        stepping_input_(1) = del_F_(1);
    }
      
    del_F_(0) = stepping_input_(0);
    del_F_(1) = stepping_input_(1);
 
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now(); 
}

/*
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
    // MJ_graph << stepping_input_(0) << "," << stepping_input_(1) << "," << t_total_ / hz_ << del_zmp(1) << "," << ZMP_Y_REF_alpha_ << endl;
     
}
*/

void AvatarController::computeThread3()
{
    comGenerator_MPC_wieber(50.0, 1.0/50.0, 2.5, 2000/50.0); // Hz, T, Preview window
    // cpcontroller_MPC_MJDG(50.0, 1.5); // Hz, Preview window
    // new_cpcontroller_MPC_ankle(50.0, 1.5);
    // new_cpcontroller_MPC_ankle_hip(50.0, 1.5);
    new_cpcontroller_MPC_MJDG(50.0, 1.5);
     
}

void AvatarController::comGenerator_MPC_wieber(double MPC_freq, double T, double preview_window, int MPC_synchro_hz_)
{   //https://doi.org/10.1163/016918610X493552
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    if(atb_mpc_update_ == false)  
    {
        atb_mpc_update_ = true;
        walking_tick_mj_mpc_ = walking_tick_mj_thread_;
        t_total_mpc_ = t_total_thread_;
        t_rest_init_mpc_ = t_rest_init_thread_;
        t_rest_last_mpc_ = t_rest_last_thread_;
        current_step_num_mpc_ = current_step_num_thread_;
        total_step_num_mpc_ = total_step_num_thread_; // only used in CP-MPC
        zmp_start_time_mj_mpc_ = zmp_start_time_mj_thread_;
        ref_zmp_mpc_ = ref_zmp_thread_;        
        ref_zmp_wo_offset_mpc_ = ref_zmp_wo_offset_thread_; // only used in CP-MPC   

        lfoot_support_current_mpc_ = lfoot_support_current_thread_;
        rfoot_support_current_mpc_ = rfoot_support_current_thread_;
         
        alpha_step_mpc_ = alpha_step_mpc_thread_; // only used in CP-MPC

        x_hat_ = x_hat_thread2_;
        y_hat_ = y_hat_thread2_;
        x_hat_p_ = x_hat_p_thread2_;
        y_hat_p_ = y_hat_p_thread2_;

        cam_mpc_init_ = cam_thread_;

        atb_mpc_update_ = false;
    }
    
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
    Eigen::VectorXd Z_x_ref(N);
    Eigen::VectorXd Z_y_ref(N);
    
    Z_x_ref_cpmpc_only_.resize(N);
    Z_y_ref_cpmpc_only_.resize(N);

    for(int i = 0; i < N; i ++)
    {
        Z_x_ref(i) = ref_zmp_mpc_(mpc_tick + MPC_synchro_hz_*i, 0); // 20 = Control freq (2000) / MPC_freq (100)
        Z_y_ref(i) = ref_zmp_mpc_(mpc_tick + MPC_synchro_hz_*i, 1);
        Z_x_ref_cpmpc_only_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + MPC_synchro_hz_*i, 0); // 왜 여기서 받는거랑 다르지?
        Z_y_ref_cpmpc_only_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + MPC_synchro_hz_*i, 1); // without zmp offset
    }    
    // static int aa = 0;
    // if(walking_tick_mj_mpc_ >= 6.3*2000 && aa == 0)
    // {
    //     aa = 1;

    //     for(int i = 0; i < N; i ++)
    //     {
    //         MJ_graph2 << Z_x_ref(i) << endl;
    //     }
            
    // }
    //define cost functions
    p_x = W2_mpc_*P_zu_mpc_.transpose()*(P_zs_mpc_*x_hat_ - Z_x_ref);
    p_y = W2_mpc_*P_zu_mpc_.transpose()*(P_zs_mpc_*y_hat_ - Z_y_ref);
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
        zmp_bound(i) = 1.0; // 0.1
    }
    
    lb_b_x = Z_x_ref - zmp_bound * 1.3 - P_zs_mpc_*x_hat_;
    ub_b_x = Z_x_ref + zmp_bound * 1.7 - P_zs_mpc_*x_hat_;
    lb_b_y = Z_y_ref - zmp_bound - P_zs_mpc_*y_hat_;
    ub_b_y = Z_y_ref + zmp_bound - P_zs_mpc_*y_hat_;
    
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
        if(atb_mpc_x_update_ == false)
        {
            atb_mpc_x_update_ = true;
            x_hat_p_thread_ = x_hat_p_;
            x_hat_thread_ = x_hat_;
            current_step_num_thread2_ = current_step_num_mpc_;
            atb_mpc_x_update_ = false;
        }
        mpc_x_update_ = true;
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

        if(atb_mpc_y_update_ == false)
        {
            atb_mpc_y_update_ = true;
            // cout<<"walking_tick_mj_mpc_:"<<walking_tick_mj_mpc_<<endl;
            y_hat_p_thread_ = y_hat_p_;
            y_hat_thread_ = y_hat_;
            current_step_num_thread2_ = current_step_num_mpc_;
            atb_mpc_y_update_ = false;
        }
        mpc_y_update_ = true;  
    }    

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    
    if((int)walking_tick_mj_mpc_ % 200 == 0)
    {   
       // cout<<"wieber mpc calculation time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << endl;
    }    
}

// void AvatarController::cpcontroller_MPC_MJDG(double MPC_freq, double preview_window)
// {
//     ///////////////////////////////////////// CP control + stepping MPC ///////////////////////////////////////
//     //// https://doi.org/10.3182/20120905-3-HR-2030.00165

//     std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();    

//     int mpc_tick = walking_tick_mj_mpc_ - zmp_start_time_mj_mpc_;
//     static int CP_MPC_first_loop = 0;
//     int N_cp = preview_window * MPC_freq; 
//     int footprint_num = 2;
//     double T = 1/MPC_freq;
//     int MPC_synchro_hz = 2000.0 / MPC_freq;

//     Eigen::VectorXd cp_x_ref(N_cp);
//     Eigen::VectorXd cp_y_ref(N_cp);

//     Eigen::VectorXd Z_x_ref_wo_offset(N_cp);
//     Eigen::VectorXd Z_y_ref_wo_offset(N_cp); 

//     Eigen::MatrixXd zeros_Ncp_x_f(N_cp, footprint_num);
//     Eigen::MatrixXd zeros_Ncp_x_Ncp(N_cp, N_cp);
//     Eigen::MatrixXd eye2(2,2);

//     Eigen::MatrixXd P_sel;  

//     zeros_Ncp_x_f.setZero();
//     zeros_Ncp_x_Ncp.setZero();
//     eye2.setIdentity();

//     // reference CP trajectory generation
//     cp_x_ref = x_com_pos_recur_.segment(0, N_cp) + x_com_vel_recur_.segment(0, N_cp)/wn; // 1.5 s
//     cp_y_ref = y_com_pos_recur_.segment(0, N_cp) + y_com_vel_recur_.segment(0, N_cp)/wn;
    
//     if(atb_cpmpc_rcv_update_ == false) // Receive datas from the compute slow thread 
//     {
//         atb_cpmpc_rcv_update_ = true;
                
//         if(current_step_num_thread_ == current_step_num_mpc_)
//         {
//             cp_measured_mpc_ = cp_measured_thread_;          
//         }
//         else
//         {
//             cout << "computeslow thread step num = " << current_step_num_thread_ << endl;
//             cout << "MPC thread step num = " << current_step_num_mpc_ << endl;            
//             cout << "stepchange was occured in only computeslow thread." << endl;
//         }

//         if(current_step_num_mpc_ != current_step_num_mpc_prev_) // receive step change control input (stepchange state in MPC thread)
//         {      
//             cpmpc_deszmp_x_(0) = cpmpc_des_zmp_x_thread2_; // To apply step change desired ZMP (computeslow) for gradient vector (thread 3/MPC)       
//             cpmpc_deszmp_y_(0) = cpmpc_des_zmp_y_thread2_; 
//         } // In this case, step change of the des.ZMP was occured in only computeslow and the step num is also increased in MPC, but MPC is not fully ended.

//         atb_cpmpc_rcv_update_ = false;        
//     }  

//     if(CP_MPC_first_loop == 0)
//     {
//         // Define Input matrix
//         Eigen::VectorXd B_cp_mpc(1);
//         B_cp_mpc(0) = 1 - exp(wn*T); 
        
//         // Define recursive state, input matrix
//         F_cp_.setZero(N_cp, 1);
//         F_zmp_.setZero(N_cp, N_cp);
        
//         for(int i = 0; i < N_cp; i++)
//         {   
//             F_cp_(i,0) = exp(wn*T*i);

//             for(int j = 0; j < N_cp; j++)
//             {
//                 if(j >= i)
//                 {
//                     F_zmp_(j,i) = exp(wn*T*(j-i))*B_cp_mpc(0);
//                 }
//             }
//         }        
        
//         // Define diffence matrix
//         diff_matrix_.setIdentity(N_cp, N_cp);

//         for(int i = 0; i < N_cp-1; i ++)
//         {
//             diff_matrix_(i+1, i) = -1.0;
//         }
        
//         e1_cpmpc_.setZero(N_cp);
//         e1_cpmpc_(0) = 1.0;
        
//         weighting_cp_.setZero(N_cp, N_cp);
//         weighting_zmp_diff_.setZero(N_cp, N_cp);
//         double weighting_foot = 0.01;// 100.0;  //0.01;

//         // Weighting parameter
//         for(int i = 0; i < N_cp; i++) // N_cp = 75
//         {
//             if(i < 1)
//             {
//                 weighting_cp_(i,i) = 10.0;
//                 weighting_zmp_diff_(i,i) = 0.2;
//                 // weighting_cp_(i,i) = 2.0;
//                 // weighting_zmp_diff_(i,i) = 0.2;
//             }
//             else if (i < 50)
//             {
//                 weighting_cp_(i,i) = 5.0;
//                 weighting_zmp_diff_(i,i) = 1.0;                
//             }
//             else
//             {
//                 weighting_cp_(i,i) = 100.0;
//                 weighting_zmp_diff_(i,i) = 0.10; 
//             }            
//         }

//         // Hessian matrix
//         H_cpmpc_.setZero(N_cp, N_cp);
//         H_cpmpc_ = diff_matrix_.transpose()*weighting_zmp_diff_*diff_matrix_ + F_zmp_.transpose()*weighting_cp_*F_zmp_;        

//         H_cpStepping_mpc_.setZero(N_cp + footprint_num, N_cp + footprint_num);
//         H_cpStepping_mpc_.block(0, 0, N_cp, N_cp) = H_cpmpc_; // CP-MPC
//         H_cpStepping_mpc_.block(N_cp, 0, footprint_num, N_cp) = zeros_Ncp_x_f.transpose();
//         H_cpStepping_mpc_.block(0, N_cp, N_cp, footprint_num) = zeros_Ncp_x_f;
//         H_cpStepping_mpc_.block(N_cp, N_cp, footprint_num, footprint_num) = weighting_foot*eye2; // Foot mpc

//         // Control input (desired zmp) initinalization 
//         cpmpc_deszmp_x_.setZero(N_cp + footprint_num);
//         cpmpc_deszmp_x_(0) = x_hat_(0); // Position of the CoM
        
//         cpmpc_deszmp_y_.setZero(N_cp + footprint_num);
//         cpmpc_deszmp_y_(0) = y_hat_(0); // Position of the CoM
        
//         QP_cpmpc_x_.InitializeProblemSize(N_cp + footprint_num, N_cp + footprint_num); // MPC variable : desired ZMP, foot position 
//         QP_cpmpc_y_.InitializeProblemSize(N_cp + footprint_num, N_cp + footprint_num);

//         CP_MPC_first_loop = 1;
//         cout << "Initialization of CP_MPC parameters is complete." << endl;
//     }  
      
//     // For foot adjustment
//     int swing_time_cur = 0, swing_time_next = 0, swing_time_n_next = 0;
    
//     if(current_step_num_mpc_ > 0 && current_step_num_mpc_ != total_step_num_mpc_-1) // Define selection vector for swingfoot adjustment
//     {
//         swing_time_cur = (t_total_mpc_ - mpc_tick)/MPC_synchro_hz; // remaining sampling time in current foot. 

//         if(N_cp - swing_time_cur >= t_total_mpc_/MPC_synchro_hz) // 3 footholds are included in N_cp step.(current, next, n_next foothold)
//         {
//             swing_time_next = t_total_mpc_/MPC_synchro_hz;
//             swing_time_n_next = N_cp - (t_total_mpc_/MPC_synchro_hz + swing_time_cur); 
//         }
//         else // 2 footholds are included in N_cp step.(current, next foothold)
//         {
//             swing_time_next = N_cp - swing_time_cur;
//             swing_time_n_next = 0;
//         } 

//         Eigen::VectorXd sel_swingfoot(swing_time_cur); 
//         Eigen::VectorXd sel_swingfoot_next(swing_time_next);  
//         Eigen::VectorXd sel_swingfoot_n_next(swing_time_n_next);  
        
//         sel_swingfoot.setZero();        
//         sel_swingfoot_next.setOnes();
        
//         P_sel.setZero(N_cp, footprint_num);  
//         P_sel.block(0, 0, swing_time_cur, 1) = sel_swingfoot;        
//         P_sel.block(swing_time_cur, 0, swing_time_next, 1) = sel_swingfoot_next;  

//         if(swing_time_n_next != 0)
//         {            
//             sel_swingfoot_n_next.setOnes(); 
//             P_sel.block(swing_time_cur + swing_time_next, 1, swing_time_n_next, 1) = sel_swingfoot_n_next;
//         } 
//     } 
//     else
//     {       
//         P_sel.setZero(N_cp,footprint_num); 
//     }
    
//     Eigen::VectorXd g_cpmpc_x(N_cp);
//     Eigen::VectorXd g_cpmpc_y(N_cp);
//     Eigen::VectorXd g_cpStepping_mpc_x(N_cp + footprint_num);
//     Eigen::VectorXd g_cpStepping_mpc_y(N_cp + footprint_num);
    
//     // gradient vector
//     g_cpmpc_x = F_zmp_.transpose()*weighting_cp_*(F_cp_*cp_measured_mpc_(0) - cp_x_ref) - diff_matrix_.transpose()*weighting_zmp_diff_*e1_cpmpc_*cpmpc_deszmp_x_(0);
//     g_cpStepping_mpc_x.setZero(N_cp + footprint_num);
//     g_cpStepping_mpc_x.segment(0, N_cp) = g_cpmpc_x;

//     g_cpmpc_y = F_zmp_.transpose()*weighting_cp_*(F_cp_*cp_measured_mpc_(1) - cp_y_ref) - diff_matrix_.transpose()*weighting_zmp_diff_*e1_cpmpc_*cpmpc_deszmp_y_(0);
//     g_cpStepping_mpc_y.setZero(N_cp + footprint_num);
//     g_cpStepping_mpc_y.segment(0, N_cp) = g_cpmpc_y;
    
//     // constraint formulation
//     Eigen::MatrixXd A_cpStepping_mpc(N_cp+footprint_num, N_cp+footprint_num);
//     Eigen::MatrixXd A_cp_mpc(N_cp, N_cp);
//     A_cp_mpc.setIdentity();
//     A_cpStepping_mpc.block(0, 0, N_cp, N_cp) = A_cp_mpc;  
//     A_cpStepping_mpc.block(0, N_cp, N_cp, footprint_num) = -P_sel;
//     A_cpStepping_mpc.block(N_cp, 0, footprint_num, N_cp) = zeros_Ncp_x_f.transpose();
//     A_cpStepping_mpc.block(N_cp, N_cp, footprint_num, footprint_num) = eye2;

//     Eigen::VectorXd ub_x_cp_mpc(N_cp);
//     Eigen::VectorXd lb_x_cp_mpc(N_cp);
//     Eigen::VectorXd ub_y_cp_mpc(N_cp);
//     Eigen::VectorXd lb_y_cp_mpc(N_cp);    

//     Eigen::VectorXd ub_x_foot_cp_mpc(footprint_num);
//     Eigen::VectorXd lb_x_foot_cp_mpc(footprint_num);
//     Eigen::VectorXd ub_y_foot_cp_mpc(footprint_num);
//     Eigen::VectorXd lb_y_foot_cp_mpc(footprint_num);   

//     Eigen::VectorXd ub_x_cpStepping_mpc(N_cp + footprint_num);
//     Eigen::VectorXd lb_x_cpStepping_mpc(N_cp + footprint_num);
//     Eigen::VectorXd ub_y_cpStepping_mpc(N_cp + footprint_num);
//     Eigen::VectorXd lb_y_cpStepping_mpc(N_cp + footprint_num);    
 
//     // reference zmp trajectory for ZMP constraint
//     Z_x_ref_wo_offset = Z_x_ref_cpmpc_only_.segment(0, N_cp); // without zmp offset
//     Z_y_ref_wo_offset = Z_y_ref_cpmpc_only_.segment(0, N_cp);

//     Eigen::VectorXd zmp_bound_x(N_cp);
//     Eigen::VectorXd zmp_bound_y(N_cp);

//     for(int i = 0; i < N_cp; i++)  
//     {
//         zmp_bound_x(i) = 0.1;
//         zmp_bound_y(i) = 0.07;  
//     }
    
//     lb_x_cp_mpc = Z_x_ref_wo_offset - zmp_bound_x * 0.9;
//     ub_x_cp_mpc = Z_x_ref_wo_offset + zmp_bound_x * 1.2;

//     lb_y_cp_mpc = Z_y_ref_wo_offset - zmp_bound_y; // Z_y_ref is the trajectory considering the ZMP offset for COM planning.
//     ub_y_cp_mpc = Z_y_ref_wo_offset + zmp_bound_y; // However, Ref. ZMP without ZMP offset is required for CP control.
     
//     // double del_F_y_rightswing_min = -0.08, del_F_y_rightswing_max = 0.03;
//     // double del_F_y_leftswing_min = -0.03, del_F_y_leftswing_max = 0.08;
//     double del_F_y_rightswing_min = -0.10, del_F_y_rightswing_max = 0.03;
//     double del_F_y_leftswing_min = -0.03, del_F_y_leftswing_max = 0.10;

//     ub_x_foot_cp_mpc.setZero();   
//     lb_x_foot_cp_mpc.setZero();
    
//     if(alpha_step_mpc_ == 1) // left foot support
//     {
//         ub_x_foot_cp_mpc(0) = 0.2 - rfoot_support_current_mpc_.translation()(0);
//         lb_x_foot_cp_mpc(0) = -0.2 - rfoot_support_current_mpc_.translation()(0);
//         ub_x_foot_cp_mpc(1) = 0.2; 
//         lb_x_foot_cp_mpc(1) = -0.2; 

//         // standard value of rfoot_support_current = -0.25
//         ub_y_foot_cp_mpc(0) = -0.22 - rfoot_support_current_mpc_.translation()(1); // 0.03
//         lb_y_foot_cp_mpc(0) = -0.35 - rfoot_support_current_mpc_.translation()(1); //-0.1
//         ub_y_foot_cp_mpc(1) = del_F_y_leftswing_max;
//         lb_y_foot_cp_mpc(1) = del_F_y_leftswing_min; 
//     }
//     else if(alpha_step_mpc_ == -1) // right foot support
//     {
//         ub_x_foot_cp_mpc(0) = 0.2 - lfoot_support_current_mpc_.translation()(0);
//         lb_x_foot_cp_mpc(0) = -0.2 - lfoot_support_current_mpc_.translation()(0);
//         ub_x_foot_cp_mpc(1) = 0.2; 
//         lb_x_foot_cp_mpc(1) = -0.2; 

//         // standard value of lfoot_support_current = +0.25
//         ub_y_foot_cp_mpc(0) =  0.35 - lfoot_support_current_mpc_.translation()(1); // 0.1
//         lb_y_foot_cp_mpc(0) =  0.22 - lfoot_support_current_mpc_.translation()(1); // -0.03
//         ub_y_foot_cp_mpc(1) = del_F_y_rightswing_max;
//         lb_y_foot_cp_mpc(1) = del_F_y_rightswing_min; 
//     } 
     
//     lb_x_cpStepping_mpc.segment(0, N_cp) = lb_x_cp_mpc;
//     lb_x_cpStepping_mpc.segment(N_cp, footprint_num) = lb_x_foot_cp_mpc;
//     ub_x_cpStepping_mpc.segment(0, N_cp) = ub_x_cp_mpc;
//     ub_x_cpStepping_mpc.segment(N_cp, footprint_num) = ub_x_foot_cp_mpc; 

//     lb_y_cpStepping_mpc.segment(0, N_cp) = lb_y_cp_mpc;
//     lb_y_cpStepping_mpc.segment(N_cp, footprint_num) = lb_y_foot_cp_mpc;
//     ub_y_cpStepping_mpc.segment(0, N_cp) = ub_y_cp_mpc;
//     ub_y_cpStepping_mpc.segment(N_cp, footprint_num) = ub_y_foot_cp_mpc; 
    
    
//     if(mpc_tick <= t_total_const_ - t_rest_last_ - t_double2_)
//     {
//         int landing_mpc_time = int((t_total_const_ - mpc_tick - t_rest_last_ - t_double2_)/MPC_synchro_hz );
//         cp_eos_x_mpc_ = cp_x_ref(landing_mpc_time); 
//         cp_eos_y_mpc_ = cp_y_ref(landing_mpc_time);    
//     }

//     // Define QP problem for CP-MPC  
//     QP_cpmpc_x_.EnableEqualityCondition(equality_condition_eps_);
//     QP_cpmpc_x_.UpdateMinProblem(H_cpStepping_mpc_,g_cpStepping_mpc_x);
//     QP_cpmpc_x_.DeleteSubjectToAx();      
//     QP_cpmpc_x_.UpdateSubjectToAx(A_cpStepping_mpc, lb_x_cpStepping_mpc, ub_x_cpStepping_mpc);             
      
//     if (QP_cpmpc_x_.SolveQPoases(200, cpmpc_input_x_))
//     {                     
//         cpmpc_deszmp_x_ = cpmpc_input_x_.segment(0, N_cp + footprint_num);
//         // del_F_(0) = cpmpc_deszmp_x_(N_cp);
//         if(atb_cpmpc_x_update_ == false)
//         {
//             atb_cpmpc_x_update_ = true;
//             del_F_x_thread_ = cpmpc_deszmp_x_(N_cp);
//             cpmpc_des_zmp_x_thread_ = cpmpc_deszmp_x_(0);
//             atb_cpmpc_x_update_ = false;
//         }       
//         cpmpc_x_update_ = true;
//     }
    
//     QP_cpmpc_y_.EnableEqualityCondition(equality_condition_eps_);
//     QP_cpmpc_y_.UpdateMinProblem(H_cpStepping_mpc_,g_cpStepping_mpc_y);
//     QP_cpmpc_y_.DeleteSubjectToAx();      
//     QP_cpmpc_y_.UpdateSubjectToAx(A_cpStepping_mpc, lb_y_cpStepping_mpc, ub_y_cpStepping_mpc);    
    
//     if (QP_cpmpc_y_.SolveQPoases(200, cpmpc_input_y_))
//     {             
//         cpmpc_deszmp_y_ = cpmpc_input_y_.segment(0, N_cp + footprint_num);
//         // del_F_(1) = cpmpc_deszmp_y_(N_cp);
//         // y_cp_recur_ = F_cp_*cp_measured_mpc_(1) + F_zmp_* cpmpc_deszmp_y_;
//         if(atb_cpmpc_y_update_ == false)
//         {
//             atb_cpmpc_y_update_ = true;
//             del_F_y_thread_ = cpmpc_deszmp_y_(N_cp);
//             cpmpc_des_zmp_y_thread_ = cpmpc_deszmp_y_(0);
//             atb_cpmpc_y_update_ = false;
//         }       
//         cpmpc_y_update_ = true;
//     }
    
//     MJ_graph << cp_x_ref(0) << "," << cp_measured_mpc_(0) << "," << Z_x_ref_wo_offset(0) << "," << cpmpc_deszmp_x_(0) << "," << cpmpc_deszmp_x_(N_cp) << endl; //"," << t_total_ << "," << cp_err_norm_x << "," << weighting_dsp << "," << cp_predicted_x(0) - cp_x_ref(0) << endl;
//     MJ_graph1 << cp_y_ref(0) << "," << cp_measured_mpc_(1) << "," << Z_y_ref_wo_offset(0) << "," << cpmpc_deszmp_y_(0) << "," << cpmpc_deszmp_y_(N_cp) << endl; //"," << t_total_ << "," << cp_err_integ_y_ << "," << weighting_dsp <<  endl;
            
//     current_step_num_mpc_prev_ = current_step_num_mpc_;
//     std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
   
// }

// void AvatarController::new_cpcontroller_MPC_ankle(double MPC_freq, double preview_window)
// {

//     std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();    

//     int mpc_tick = walking_tick_mj_mpc_ - zmp_start_time_mj_mpc_;
//     static int CP_MPC_first_loop = 0;
//     int N_cp = preview_window * MPC_freq; 
//     int footprint_num = 2;
//     double T = 1/MPC_freq;
//     int MPC_synchro_hz = 2000.0 / MPC_freq;

//     Eigen::VectorXd cp_x_ref(N_cp);
//     Eigen::VectorXd cp_y_ref(N_cp);

//     Eigen::VectorXd Z_x_ref_wo_offset(N_cp);
//     Eigen::VectorXd Z_y_ref_wo_offset(N_cp); 

//     Eigen::MatrixXd zeros_Ncp_x_f(N_cp, footprint_num);
//     Eigen::MatrixXd zeros_Ncp_x_Ncp(N_cp, N_cp);
//     Eigen::MatrixXd eye2(2,2);

//     Eigen::MatrixXd P_sel;  

//     zeros_Ncp_x_f.setZero();
//     zeros_Ncp_x_Ncp.setZero();
//     eye2.setIdentity();

//     // reference CP trajectory generation
//     cp_x_ref = x_com_pos_recur_.segment(0, N_cp) + x_com_vel_recur_.segment(0, N_cp)/wn; // 1.5 s
//     cp_y_ref = y_com_pos_recur_.segment(0, N_cp) + y_com_vel_recur_.segment(0, N_cp)/wn;
    
//     // reference zmp trajectory for ZMP constraint
//     Z_x_ref_wo_offset = Z_x_ref_cpmpc_only_.segment(0, N_cp); // without zmp offset
//     Z_y_ref_wo_offset = Z_y_ref_cpmpc_only_.segment(0, N_cp);
    

//     Eigen::VectorXd zmp_bound_x(N_cp);
//     Eigen::VectorXd zmp_bound_y(N_cp);

//     for(int i = 0; i < N_cp; i++)  
//     {
//         zmp_bound_x(i) = 0.1;
//         zmp_bound_y(i) = 0.07;  
//     }

//     if(atb_cpmpc_rcv_update_ == false) // Receive datas from the compute slow thread 
//     {
//         atb_cpmpc_rcv_update_ = true;
                
//         if(current_step_num_thread_ == current_step_num_mpc_)
//         {
//             cp_measured_mpc_ = cp_measured_thread_;          
//         }
//         else
//         {
//             cout << "computeslow thread step num = " << current_step_num_thread_ << endl;
//             cout << "MPC thread step num = " << current_step_num_mpc_ << endl;            
//             cout << "stepchange was occured in only computeslow thread." << endl;
//         }

//         if(current_step_num_mpc_ != current_step_num_mpc_prev_) // receive step change control input (stepchange state in MPC thread)
//         {      
//             cpmpc_deszmp_x_(0) = cpmpc_des_zmp_x_thread2_; // To apply step change desired ZMP (computeslow) for gradient vector (thread 3/MPC)       
//             cpmpc_deszmp_y_(0) = cpmpc_des_zmp_y_thread2_; 
//         } // In this case, step change of the des.ZMP was occured in only computeslow and the step num is also increased in MPC, but MPC is not fully ended.

//         atb_cpmpc_rcv_update_ = false;        
//     }  

//     if(CP_MPC_first_loop == 0)
//     {
//         // Define Input matrix
//         Eigen::VectorXd B_cp_mpc(1);
//         B_cp_mpc(0) = 1 - exp(wn*T); 
        
//         // Define recursive state, input matrix
//         F_cp_.setZero(N_cp, 1);
//         F_zmp_.setZero(N_cp, N_cp);
        
//         for(int i = 0; i < N_cp; i++)
//         {   
//             F_cp_(i,0) = exp(wn*T*i);

//             for(int j = 0; j < N_cp; j++)
//             {
//                 if(j >= i)
//                 {
//                     F_zmp_(j,i) = exp(wn*T*(j-i))*B_cp_mpc(0);
//                 }
//             }
//         }        
        
//         // Define diffence matrix
//         diff_matrix_.setIdentity(N_cp, N_cp);

//         for(int i = 0; i < N_cp-1; i ++)
//         {
//             diff_matrix_(i+1, i) = -1.0;
//         }
        
//         e1_cpmpc_.setZero(N_cp);
//         e1_cpmpc_(0) = 1.0;
        
//         weighting_cp_.setZero(N_cp, N_cp);
//         weighting_zmp_diff_.setZero(N_cp, N_cp); 

//         // Weighting parameter
//         for(int i = 0; i < N_cp; i++) // N_cp = 75
//         {
//             if(i < 1)
//             {
//                 weighting_cp_(i,i) = 10.0;
//                 weighting_zmp_diff_(i,i) = 0.2; 
//             }
//             else if (i < 50)
//             {
//                 weighting_cp_(i,i) = 5.0;
//                 weighting_zmp_diff_(i,i) = 1.0;                
//             }
//             else
//             {
//                 weighting_cp_(i,i) = 100.0;
//                 weighting_zmp_diff_(i,i) = 0.10; 
//             }            
//         }

//         // Hessian matrix
//         H_cpmpc_.setZero(N_cp, N_cp);
//         H_cpmpc_ = diff_matrix_.transpose()*weighting_zmp_diff_*diff_matrix_ + F_zmp_.transpose()*weighting_cp_*F_zmp_;        
 

//         // Control input (desired zmp) initinalization 
//         cpmpc_deszmp_x_.setZero(N_cp);
//         cpmpc_deszmp_x_(0) = x_hat_(0); // Position of the CoM
        
//         cpmpc_deszmp_y_.setZero(N_cp);
//         cpmpc_deszmp_y_(0) = y_hat_(0); // Position of the CoM
        
//         QP_cpmpc_x_.InitializeProblemSize(N_cp, N_cp); // MPC variable : desired ZMP, foot position 
//         QP_cpmpc_y_.InitializeProblemSize(N_cp, N_cp);

//         CP_MPC_first_loop = 1;
//         cout << "Initialization of CP_MPC parameters is complete." << endl;
//     }  
        
    
//     Eigen::VectorXd g_cpmpc_x(N_cp);
//     Eigen::VectorXd g_cpmpc_y(N_cp);
    
//     // gradient vector
//     g_cpmpc_x = F_zmp_.transpose()*weighting_cp_*(F_cp_*cp_measured_mpc_(0) - cp_x_ref) - diff_matrix_.transpose()*weighting_zmp_diff_*e1_cpmpc_*cpmpc_deszmp_x_(0);
//     g_cpmpc_y = F_zmp_.transpose()*weighting_cp_*(F_cp_*cp_measured_mpc_(1) - cp_y_ref) - diff_matrix_.transpose()*weighting_zmp_diff_*e1_cpmpc_*cpmpc_deszmp_y_(0);
     
//     // constraint formulation 
//     Eigen::MatrixXd A_cp_mpc(N_cp, N_cp);
//     A_cp_mpc.setIdentity();

//     Eigen::VectorXd ub_x_cp_mpc(N_cp);
//     Eigen::VectorXd lb_x_cp_mpc(N_cp);
//     Eigen::VectorXd ub_y_cp_mpc(N_cp);
//     Eigen::VectorXd lb_y_cp_mpc(N_cp);    
 
//     // for(int i = 0; i < N_cp; i ++)
//     // {
//     //     Z_x_ref_wo_offset(i) = ref_zmp_mpc_(mpc_tick + MPC_synchro_hz*(i+1), 0); // 20 = Control freq (2000) / MPC_freq (100)
//     //     Z_y_ref_wo_offset(i) = ref_zmp_wo_offset_mpc_(mpc_tick + MPC_synchro_hz*(i+1), 1);
//     // }      
    
//     lb_x_cp_mpc = Z_x_ref_wo_offset - zmp_bound_x * 0.9;
//     ub_x_cp_mpc = Z_x_ref_wo_offset + zmp_bound_x * 1.2;

//     lb_y_cp_mpc = Z_y_ref_wo_offset - zmp_bound_y; // Z_y_ref is the trajectory considering the ZMP offset for COM planning.
//     ub_y_cp_mpc = Z_y_ref_wo_offset + zmp_bound_y; // However, Ref. ZMP without ZMP offset is required for CP control.
         
//     // Define QP problem for CP-MPC  
//     QP_cpmpc_x_.EnableEqualityCondition(equality_condition_eps_);
//     QP_cpmpc_x_.UpdateMinProblem(H_cpmpc_,g_cpmpc_x);
//     QP_cpmpc_x_.DeleteSubjectToAx();      
//     QP_cpmpc_x_.UpdateSubjectToAx(A_cp_mpc, lb_x_cp_mpc, ub_x_cp_mpc);             
      
//     if (QP_cpmpc_x_.SolveQPoases(200, cpmpc_input_x_))
//     {                     
//         cpmpc_deszmp_x_ = cpmpc_input_x_.segment(0, N_cp);
        
//         if(atb_cpmpc_x_update_ == false)
//         {
//             atb_cpmpc_x_update_ = true; 
//             cpmpc_des_zmp_x_thread_ = cpmpc_deszmp_x_(0);
//             atb_cpmpc_x_update_ = false;
//         }       
//         cpmpc_x_update_ = true;
//     }
    
//     QP_cpmpc_y_.EnableEqualityCondition(equality_condition_eps_);
//     QP_cpmpc_y_.UpdateMinProblem(H_cpmpc_,g_cpmpc_y);
//     QP_cpmpc_y_.DeleteSubjectToAx();      
//     QP_cpmpc_y_.UpdateSubjectToAx(A_cp_mpc, lb_y_cp_mpc, ub_y_cp_mpc);    
    
//     if (QP_cpmpc_y_.SolveQPoases(200, cpmpc_input_y_))
//     {             
//         cpmpc_deszmp_y_ = cpmpc_input_y_.segment(0, N_cp); 

//         if(atb_cpmpc_y_update_ == false)
//         {
//             atb_cpmpc_y_update_ = true; 
//             cpmpc_des_zmp_y_thread_ = cpmpc_deszmp_y_(0);
//             atb_cpmpc_y_update_ = false;
//         }       
//         cpmpc_y_update_ = true;
//     }

//     MJ_graph << Z_x_ref_wo_offset(0) << "," << cpmpc_deszmp_x_(0) << "," << Z_y_ref_wo_offset(0) << "," << cpmpc_deszmp_y_(0) << endl; 

//     current_step_num_mpc_prev_ = current_step_num_mpc_;  
// }

// void AvatarController::new_cpcontroller_MPC_ankle_hip(double MPC_freq, double preview_window)
// {
//     ////////////////////////// New CP control (ZMP + CAM) + stepping MPC ///////////////////////////////////////
 
//     // std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();    

//     int mpc_tick = walking_tick_mj_mpc_ - zmp_start_time_mj_mpc_; // local varible
//     static int New_CP_MPC_first_loop = 0;
//     int N_cp = preview_window * MPC_freq; // future step num 
//     int footprint_num = 2;
//     double T = 1/MPC_freq;
//     int MPC_synchro_hz = 2000.0 / MPC_freq;

//     Eigen::VectorXd cp_x_ref_new(N_cp);
//     Eigen::VectorXd cp_y_ref_new(N_cp);

//     Eigen::VectorXd Z_x_ref_wo_offset_new(2*N_cp);
//     Eigen::VectorXd Z_y_ref_wo_offset_new(2*N_cp);  

//     Eigen::VectorXd Tau_x_limit(2*N_cp);
//     Eigen::VectorXd Tau_y_limit(2*N_cp);   

//     Eigen::VectorXd zmp_bound_x_new(2*N_cp);
//     Eigen::VectorXd zmp_bound_y_new(2*N_cp);

//     Eigen::MatrixXd zeros_Ncp_x_f(N_cp, footprint_num);
//     Eigen::MatrixXd zeros_2Ncp_x_f(2*N_cp, footprint_num);
//     Eigen::MatrixXd zeros_Ncp_x_Ncp(N_cp, N_cp);
//     Eigen::MatrixXd eye2(2,2);
//     Eigen::MatrixXd eyeN(N_cp, N_cp);

//     Eigen::MatrixXd P_sel;
//     Eigen::MatrixXd P_sel_new;  

//     zeros_Ncp_x_f.setZero();
//     zeros_2Ncp_x_f.setZero();
//     zeros_Ncp_x_Ncp.setZero();
//     eye2.setIdentity();
//     eyeN.setIdentity();
    
//     // reference CP trajectory generation
//     cp_x_ref_new = x_com_pos_recur_.segment(0, N_cp) + x_com_vel_recur_.segment(0, N_cp)/wn; // 1.5 s
//     cp_y_ref_new = y_com_pos_recur_.segment(0, N_cp) + y_com_vel_recur_.segment(0, N_cp)/wn;
    
//     Z_x_ref_wo_offset_new.setZero();
//     Z_y_ref_wo_offset_new.setZero();
//     zmp_bound_x_new.setZero();
//     zmp_bound_y_new.setZero();    
//     Tau_x_limit.setZero();
//     Tau_y_limit.setZero();    
           
    
//     for(int i = 0; i < N_cp; i++)  
//     {
//         Z_x_ref_wo_offset_new(2*i) = Z_x_ref_cpmpc_only_(i); 
//         Z_y_ref_wo_offset_new(2*i) = Z_y_ref_cpmpc_only_(i);
//         zmp_bound_x_new(2*i) = 0.1; 
//         zmp_bound_y_new(2*i) = 0.07;  
//         Tau_x_limit(2*i + 1) = 15.0;//20.0;
//         Tau_y_limit(2*i + 1) = 15.0;//20.0;
//     }

//     if(atb_new_cpmpc_rcv_update_ == false) // Receive datas from the compute slow thread 
//     {
//         atb_new_cpmpc_rcv_update_ = true;
                
//         if(current_step_num_thread_ == current_step_num_mpc_)
//         {
//             cp_measured_mpc_ = cp_measured_thread_;          
//         }
//         else
//         {
//             cout << "computeslow thread step num = " << current_step_num_thread_ << endl;
//             cout << "MPC thread step num = " << current_step_num_mpc_ << endl;            
//             cout << "stepchange was occured in only computeslow thread." << endl;
//         }
        
//         if(current_step_num_mpc_ != current_step_num_mpc_new_prev_) // receive step change control input (stepchange state in MPC thread)
//         {      
//             cpmpc_output_x_new_(0) = cpmpc_des_zmp_x_thread2_; // To apply step change desired ZMP (computeslow) for gradient vector (thread 3/MPC)       
//             cpmpc_output_y_new_(0) = cpmpc_des_zmp_y_thread2_;                        
//         } // In this case, step change of the des.ZMP was occured in only computeslow and the step num is also increased in MPC, but MPC is not fully ended.

//         atb_new_cpmpc_rcv_update_ = false;        
//     }  

//     if(New_CP_MPC_first_loop == 0)
//     {
//         // Define Input matrix
//         Eigen::MatrixXd B_cp_mpc_new(1,2);
//         B_cp_mpc_new(0,0) = 1 - exp(wn*T);
//         B_cp_mpc_new(0,1) = (1 - exp(wn*T))/(rd_.link_[COM_id].mass * GRAVITY);

//         // Define prediction state over N step, input matrix
//         F_cp_new_.setZero(N_cp, 1);
//         F_cmp_new_.setZero(N_cp, 2*N_cp);
        
//         for(int i = 0; i < N_cp; i++) // prediction state matrix, N X 1
//         {   
//             F_cp_new_(i,0) = exp(wn*T*i); 
//         }        
        
//         for(int i = 0; i < N_cp; i++) // prediction input matrix, N X 2N 
//         {   
//             for(int j = 0; j < N_cp; j++)
//             {
//                 if(j >= i)
//                 {
//                     F_cmp_new_(j,2*i) = exp(wn*T*(j-i))*B_cp_mpc_new(0);
//                     F_cmp_new_(j,2*i+1) = exp(wn*T*(j-i))*B_cp_mpc_new(1);
//                 }
//             }
//         }  
         
//         // Define diffence matrix
//         diff_matrix_new_.setIdentity(2*N_cp, 2*N_cp);

//         for(int i = 0; i < 2*(N_cp-1); i ++)
//         {
//             diff_matrix_new_(i+2, i) = -1.0; // 2N X 2N
//         }
        
//         e1_cpmpc_new_.setZero(2*N_cp, 2);
//         e1_cpmpc_new_.block<2, 2>(0, 0) = eye2;

//         // Tau_sel_.setIdentity(2*N_cp, 2*N_cp);

//         // for(int i = 0; i < N_cp; i++) // selection matrix for variable Tau
//         // {
//         //     Tau_sel_(2*i,2*i) = 0.0; 
//         // }

//         desZmp_sel_.setIdentity(2*N_cp, 2*N_cp);

//         for(int i = 0; i < N_cp; i++) // selection matrix for variable Tau
//         {
//             desZmp_sel_(2*i+1, 2*i+1) = 0.0; 
//         }

//         weighting_cp_new_x_.setZero(N_cp, N_cp);
//         weighting_cp_new_y_.setZero(N_cp, N_cp);
//         weighting_cmp_diff_new_.setZero(2*N_cp, 2*N_cp);
        
             
//         // damping function 
//         double cam_damping_gain = 70.0; // Tau = - K_damping * CAM // 높으면 -Tau 한번에 크게 발생
        
//         cam_damping_mat_.setIdentity(N_cp, N_cp);
//         cam_damping_mat_ = cam_damping_gain * cam_damping_mat_;
         
//         // Define integral matrix
//         Eigen::MatrixXd integral_matrix_temp;
//         integral_matrix_temp.setOnes(N_cp, N_cp);
//         for(int i = 0; i < N_cp; i++)
//         {
//             integral_matrix_temp(i,i) = 0.0;
//         }
        
//         integral_matrix_.setZero(N_cp, N_cp);
//         integral_matrix_ = integral_matrix_temp.triangularView<Lower>();
        
//         damping_integral_mat_.setZero(N_cp, N_cp);
//         damping_integral_mat_ = T * cam_damping_mat_ * integral_matrix_;         
         
//         H_damping_Nsize_x_.setZero(N_cp, N_cp);
//         H_damping_Nsize_y_.setZero(N_cp, N_cp);
//         H_damping_x_.setZero(2*N_cp, 2*N_cp); 
//         H_damping_y_.setZero(2*N_cp, 2*N_cp);  
 
        
//         // Weighting parameter // Freq: 50 Hz/ Preview window: 1.5 s => N step = 75, 2N = 150
//         for(int i = 0; i < N_cp; i++) // For cp control
//         {
//             if(i < 1)
//             {                
//                 weighting_cp_new_x_(i,i) = 10.0; // X dir   
//                 weighting_cp_new_y_(i,i) = 10.0; // Y dir
//             }
//             else if (i < 50)
//             {                
//                 weighting_cp_new_x_(i,i) = 20.0; // X dir     
//                 weighting_cp_new_y_(i,i) = 5.0; // Y dir          
//             }
//             else
//             {                
//                 weighting_cp_new_x_(i,i) = 100.0; 
//                 weighting_cp_new_y_(i,i) = 100.0; // Y dir
//             }            
//         }
//         for(int i = 0; i < 2*N_cp; i++) // for smoothing control input  
//         {
//             if(i < 2) // des.zmp, des.tau
//             {
//                 weighting_cmp_diff_new_(i,i) = 0.2; // original gain 0.2 
//             }
//             else if (i < 100)
//             {
//                 weighting_cmp_diff_new_(i,i) = 1.0; // original gain 10.0               
//             }
//             else
//             {
//                 weighting_cmp_diff_new_(i,i) = 0.10; 
//             }            
//         }
        
//         // Hessian matrix (cp control + change regulation)
//         H_cp_control_x_.setZero(2*N_cp, 2*N_cp);
//         H_cp_control_y_.setZero(2*N_cp, 2*N_cp);
//         H_change_regul_.setZero(2*N_cp, 2*N_cp);
//         H_cp_control_x_ = F_cmp_new_.transpose()*weighting_cp_new_x_*F_cmp_new_;
//         H_cp_control_y_ = F_cmp_new_.transpose()*weighting_cp_new_y_*F_cmp_new_;
//         H_change_regul_ = diff_matrix_new_.transpose()*weighting_cmp_diff_new_*diff_matrix_new_;
        
//         H_cpmpc_new_x_.setZero(2*N_cp, 2*N_cp);  
//         H_cpmpc_new_y_.setZero(2*N_cp, 2*N_cp);             
         
//         // Control input (desired zmp, desired tau) initinalization 
//         cpmpc_output_x_new_.setZero(2*N_cp);
//         cpmpc_output_x_new_(0) = x_hat_(0); // Initial position of the CoM and ZMP is equal.
        
//         cpmpc_output_y_new_.setZero(2*N_cp);
//         cpmpc_output_y_new_(0) = y_hat_(0); // Initial position of the CoM and ZMP is equal.
        
//         QP_cpmpc_x_new_.InitializeProblemSize(2*N_cp, 2*N_cp);
//         QP_cpmpc_y_new_.InitializeProblemSize(2*N_cp, 2*N_cp);
//         New_CP_MPC_first_loop = 1;
        
//         cout << "Initialization of New CP_MPC parameters is complete." << endl;
//     }  
    
//     weighting_tau_damping_x_.setIdentity(N_cp, N_cp);
//     weighting_tau_damping_y_.setIdentity(N_cp, N_cp); 
        
//     // for(int i = 0; i < N_cp; i++)
//     // {   
//     //     weighting_tau_damping_x_(i, i) = DyrosMath::cubic(abs(cpmpc_output_x_new_(2*i) - Z_x_ref_wo_offset_new(2*i)), 0.00, 0.07, 0.0000001, 0.0, 0.0, 0.0);
//     //     weighting_tau_damping_y_(i, i) = DyrosMath::cubic(abs(cpmpc_output_y_new_(2*i) - Z_y_ref_wo_offset_new(2*i)), 0.00, 0.03, 0.0000001, 0.0, 0.0, 0.0); 
//     // }      
    
//     weighting_tau_damping_x_ = 0.0000001 * weighting_tau_damping_x_;
//     weighting_tau_damping_y_ = 0.0000001 * weighting_tau_damping_y_;
//     // for(int i = 0; i < N_cp; i++) 
//     // {   
//     //     weighting_zmp_regul_(i, i) = DyrosMath::cubic(abs(Z_x_ref_wo_offset_new(2*i)) + zmp_bound_x_new(2*i)*0.9 + abs(cpmpc_output_x_new_(2*N_cp)) - abs(cpmpc_output_x_new_(2*i)), 0.0, 0.04, 100.0, 0.0, 0.0, 0.0);
//     //     // weighting_tau_damping_y_(i, i) = DyrosMath::cubic(abs(cpmpc_output_y_new_(2*i) - Z_y_ref_wo_offset_new(2*i)), 0.00, 0.03, 0.0000001, 0.0, 0.0, 0.0); 
//     // }      
    
//     H_damping_Nsize_x_ = (eyeN + damping_integral_mat_).transpose() * weighting_tau_damping_x_ * (eyeN + damping_integral_mat_);
//     H_damping_Nsize_y_ = (eyeN + damping_integral_mat_).transpose() * weighting_tau_damping_y_ * (eyeN + damping_integral_mat_);

//     for(int i = 0; i < N_cp; i++)
//     {
//         for(int j = 0; j < N_cp; j++)
//         {
//             H_damping_x_(2*i+1, 2*j+1) = H_damping_Nsize_x_(i, j);
//             H_damping_y_(2*i+1, 2*j+1) = H_damping_Nsize_y_(i, j);
//         }            
//     }
    
//     H_cpmpc_new_x_ = H_change_regul_ + H_cp_control_x_ + H_damping_x_; // + desZmp_sel_.transpose()*weighting_zmp_regul_*desZmp_sel_;
//     H_cpmpc_new_y_ = H_change_regul_ + H_cp_control_y_ + H_damping_y_; 

//     Eigen::VectorXd onevector_N;
//     onevector_N.setOnes(N_cp);
//     Eigen::VectorXd g_damping_Nsize_x;
//     Eigen::VectorXd g_damping_Nsize_y;
//     Eigen::VectorXd g_damping_x;
//     Eigen::VectorXd g_damping_y;
//     g_damping_Nsize_x.setZero(N_cp);
//     g_damping_Nsize_y.setZero(N_cp);
//     g_damping_x.setZero(2*N_cp);
//     g_damping_Nsize_x = (eyeN + damping_integral_mat_).transpose() * (cam_mpc_init_(1)*weighting_tau_damping_x_.transpose()*cam_damping_mat_*onevector_N); // 0.01에 들어가는 value를 h0로
//     g_damping_y.setZero(2*N_cp);
//     g_damping_Nsize_y = (eyeN + damping_integral_mat_).transpose() * (-cam_mpc_init_(0)*weighting_tau_damping_y_.transpose()*cam_damping_mat_*onevector_N); // 0.01에 들어가는 value를 h0로
     
//     for(int i=0; i<N_cp; i++)
//     {
//         g_damping_x(2*i+1) = g_damping_Nsize_x(i);
//         g_damping_y(2*i+1) = g_damping_Nsize_y(i);
//     }       

//     Eigen::VectorXd g_cpmpc_x_new(2*N_cp);  
//     Eigen::VectorXd g_cpmpc_y_new(2*N_cp);
    
//     // gradient vector
//     g_cpmpc_x_new = g_damping_x + F_cmp_new_.transpose()*weighting_cp_new_x_*(F_cp_new_*cp_measured_mpc_(0) - cp_x_ref_new) - diff_matrix_new_.transpose()*weighting_cmp_diff_new_*e1_cpmpc_new_*cpmpc_output_x_new_.segment<2>(0);
//     g_cpmpc_y_new = g_damping_y + F_cmp_new_.transpose()*weighting_cp_new_y_*(F_cp_new_*cp_measured_mpc_(1) - cp_y_ref_new) - diff_matrix_new_.transpose()*weighting_cmp_diff_new_*e1_cpmpc_new_*cpmpc_output_y_new_.segment<2>(0);
    
//     // constraint formulation
//     Eigen::MatrixXd A_cpStepping_mpc_new(2*N_cp + footprint_num, 2*N_cp + footprint_num);
//     Eigen::MatrixXd A_cp_mpc_new(2*N_cp, 2*N_cp);  
//     A_cp_mpc_new.setIdentity();

//     Eigen::VectorXd ub_x_cp_mpc_new(2*N_cp);
//     Eigen::VectorXd lb_x_cp_mpc_new(2*N_cp);
//     Eigen::VectorXd ub_y_cp_mpc_new(2*N_cp);
//     Eigen::VectorXd lb_y_cp_mpc_new(2*N_cp);    
         
//     lb_x_cp_mpc_new = (Z_x_ref_wo_offset_new - zmp_bound_x_new * 0.9) - (Tau_y_limit);
//     ub_x_cp_mpc_new = (Z_x_ref_wo_offset_new + zmp_bound_x_new * 1.2) + (Tau_y_limit);

//     lb_y_cp_mpc_new = (Z_y_ref_wo_offset_new - zmp_bound_y_new) - (Tau_x_limit); // Z_y_ref is the trajectory considering the ZMP offset for COM planning.
//     ub_y_cp_mpc_new = (Z_y_ref_wo_offset_new + zmp_bound_y_new) + (Tau_x_limit); // However, Ref. ZMP without ZMP offset is required for CP control.
        
//     // Define QP problem for new CP-MPC  
//     QP_cpmpc_x_new_.EnableEqualityCondition(equality_condition_eps_);
//     QP_cpmpc_x_new_.UpdateMinProblem(H_cpmpc_new_x_, g_cpmpc_x_new);
//     QP_cpmpc_x_new_.DeleteSubjectToAx();      
//     QP_cpmpc_x_new_.UpdateSubjectToAx(A_cp_mpc_new, lb_x_cp_mpc_new, ub_x_cp_mpc_new);     
          
//     if (QP_cpmpc_x_new_.SolveQPoases(200, cpmpc_input_x_new_))
//     {                     
//         cpmpc_output_x_new_ = cpmpc_input_x_new_.segment(0, 2*N_cp); 
                
//         if(atb_cpmpc_x_update_ == false)
//         {
//             atb_cpmpc_x_update_ = true;
//             // del_F_x_thread_ = cpmpc_output_x_new_(2*N_cp);
//             cpmpc_des_zmp_x_thread_ = cpmpc_output_x_new_(0);
//             des_tau_y_thread_ = cpmpc_output_x_new_(1);
//             atb_cpmpc_x_update_ = false;
//         }       
//         cpmpc_x_update_ = true;
//     } 
    
//     QP_cpmpc_y_new_.EnableEqualityCondition(equality_condition_eps_);
//     QP_cpmpc_y_new_.UpdateMinProblem(H_cpmpc_new_y_, g_cpmpc_y_new);
//     QP_cpmpc_y_new_.DeleteSubjectToAx();      
//     QP_cpmpc_y_new_.UpdateSubjectToAx(A_cp_mpc_new, lb_y_cp_mpc_new, ub_y_cp_mpc_new);       

//     if (QP_cpmpc_y_new_.SolveQPoases(200, cpmpc_input_y_new_))
//     {                     
//         cpmpc_output_y_new_ = cpmpc_input_y_new_.segment(0, 2*N_cp); 
                
//         if(atb_cpmpc_y_update_ == false)
//         {
//             atb_cpmpc_y_update_ = true;
//             // del_F_y_thread_ = cpmpc_output_y_new_(2*N_cp);
//             cpmpc_des_zmp_y_thread_ = cpmpc_output_y_new_(0);
//             des_tau_x_thread_ = cpmpc_output_y_new_(1);
//             atb_cpmpc_y_update_ = false;
//         }       
//         cpmpc_y_update_ = true;
//     } 
    
//     MJ_graph << Z_x_ref_wo_offset_new(0) << "," << cpmpc_output_x_new_(0) << "," <<  del_tau_(1) << "," << del_ang_momentum_(1) << endl; 
//     MJ_graph1 << Z_y_ref_wo_offset_new(0) << "," << cpmpc_output_y_new_(0) << "," << del_tau_(0) << "," << del_ang_momentum_(0) << endl; 
//     // MJ_graph2 << Z_x_ref_wo_offset_new(0) << "," << Z_y_ref_wo_offset_new(0) << "," << cpmpc_output_x_new_(0) << "," << cpmpc_output_y_new_(0) << "," << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << endl;    
//     current_step_num_mpc_new_prev_ = current_step_num_mpc_;
//     // std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();    
// }

void AvatarController::new_cpcontroller_MPC_MJDG(double MPC_freq, double preview_window)
{
    ////////////////////////// New CP control (ZMP + CAM) + stepping MPC ///////////////////////////////////////
 
    // std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();    

    int mpc_tick = walking_tick_mj_mpc_ - zmp_start_time_mj_mpc_; // local varible
    static int New_CP_MPC_first_loop = 0;
    int N_cp = preview_window * MPC_freq; // future step num 
    int footprint_num = 2;
    double T = 1/MPC_freq;
    int MPC_synchro_hz = 2000.0 / MPC_freq;

    Eigen::VectorXd cp_x_ref_new(N_cp);
    Eigen::VectorXd cp_y_ref_new(N_cp);

    Eigen::VectorXd Z_x_ref_wo_offset_new(2*N_cp);
    Eigen::VectorXd Z_y_ref_wo_offset_new(2*N_cp);  

    Eigen::VectorXd Tau_x_limit(2*N_cp);
    Eigen::VectorXd Tau_y_limit(2*N_cp);   

    Eigen::VectorXd zmp_bound_x_new(2*N_cp);
    Eigen::VectorXd zmp_bound_y_new(2*N_cp);

    Eigen::MatrixXd zeros_Ncp_x_f(N_cp, footprint_num);
    Eigen::MatrixXd zeros_2Ncp_x_f(2*N_cp, footprint_num);
    Eigen::MatrixXd zeros_Ncp_x_Ncp(N_cp, N_cp);
    Eigen::MatrixXd eye2(2,2);
    Eigen::MatrixXd eyeN(N_cp, N_cp);

    Eigen::MatrixXd P_sel;
    Eigen::MatrixXd P_sel_new;  

    zeros_Ncp_x_f.setZero();
    zeros_2Ncp_x_f.setZero();
    zeros_Ncp_x_Ncp.setZero();
    eye2.setIdentity();
    eyeN.setIdentity();
    
    // reference CP trajectory generation
    cp_x_ref_new = x_com_pos_recur_.segment(0, N_cp) + x_com_vel_recur_.segment(0, N_cp)/wn; // 1.5 s
    cp_y_ref_new = y_com_pos_recur_.segment(0, N_cp) + y_com_vel_recur_.segment(0, N_cp)/wn;
    
    Z_x_ref_wo_offset_new.setZero();
    Z_y_ref_wo_offset_new.setZero();
    zmp_bound_x_new.setZero();
    zmp_bound_y_new.setZero();    
    Tau_x_limit.setZero();
    Tau_y_limit.setZero();    
           
    
    for(int i = 0; i < N_cp; i++)  
    {
        Z_x_ref_wo_offset_new(2*i) = Z_x_ref_cpmpc_only_(i); 
        Z_y_ref_wo_offset_new(2*i) = Z_y_ref_cpmpc_only_(i);
        zmp_bound_x_new(2*i) = 0.1; 
        zmp_bound_y_new(2*i) = 0.07;  
        Tau_x_limit(2*i + 1) = 7.0;//20.0;
        Tau_y_limit(2*i + 1) = 10.0;//20.0;
    }

    if(atb_new_cpmpc_rcv_update_ == false) // Receive datas from the compute slow thread 
    {
        atb_new_cpmpc_rcv_update_ = true;
                
        if(current_step_num_thread_ == current_step_num_mpc_)
        {
            cp_measured_mpc_ = cp_measured_thread_;          
        }
        else
        {
            cout << "computeslow thread step num = " << current_step_num_thread_ << endl;
            cout << "MPC thread step num = " << current_step_num_mpc_ << endl;            
            cout << "stepchange was occured in only computeslow thread." << endl;
        }
        
        if(current_step_num_mpc_ != current_step_num_mpc_new_prev_) // receive step change control input (stepchange state in MPC thread)
        {      
            cpmpc_output_x_new_(0) = cpmpc_des_zmp_x_thread2_; // To apply step change desired ZMP (computeslow) for gradient vector (thread 3/MPC)       
            cpmpc_output_y_new_(0) = cpmpc_des_zmp_y_thread2_;                        
        } // In this case, step change of the des.ZMP was occured in only computeslow and the step num is also increased in MPC, but MPC is not fully ended.

        atb_new_cpmpc_rcv_update_ = false;        
    }  

    if(New_CP_MPC_first_loop == 0)
    {
        // Define Input matrix
        Eigen::MatrixXd B_cp_mpc_new(1,2);
        B_cp_mpc_new(0,0) = 1 - exp(wn*T);
        B_cp_mpc_new(0,1) = (1 - exp(wn*T))/(rd_.link_[COM_id].mass * GRAVITY);

        // Define prediction state over N step, input matrix
        F_cp_new_.setZero(N_cp, 1);
        F_cmp_new_.setZero(N_cp, 2*N_cp);
        
        for(int i = 0; i < N_cp; i++) // prediction state matrix, N X 1
        {   
            F_cp_new_(i,0) = exp(wn*T*i); 
        }        
        
        for(int i = 0; i < N_cp; i++) // prediction input matrix, N X 2N 
        {   
            for(int j = 0; j < N_cp; j++)
            {
                if(j >= i)
                {
                    F_cmp_new_(j,2*i) = exp(wn*T*(j-i))*B_cp_mpc_new(0);
                    F_cmp_new_(j,2*i+1) = exp(wn*T*(j-i))*B_cp_mpc_new(1);
                }
            }
        }  
         
        // Define diffence matrix
        diff_matrix_new_.setIdentity(2*N_cp, 2*N_cp);

        for(int i = 0; i < 2*(N_cp-1); i ++)
        {
            diff_matrix_new_(i+2, i) = -1.0; // 2N X 2N
        }
        
        e1_cpmpc_new_.setZero(2*N_cp, 2);
        e1_cpmpc_new_.block<2, 2>(0, 0) = eye2;

        // Tau_sel_.setIdentity(2*N_cp, 2*N_cp);

        // for(int i = 0; i < N_cp; i++) // selection matrix for variable Tau
        // {
        //     Tau_sel_(2*i,2*i) = 0.0; 
        // }

        desZmp_sel_.setIdentity(2*N_cp, 2*N_cp);

        for(int i = 0; i < N_cp; i++) // selection matrix for variable Tau
        {
            desZmp_sel_(2*i+1, 2*i+1) = 0.0; 
        }

        weighting_cp_new_x_.setZero(N_cp, N_cp);
        weighting_cp_new_y_.setZero(N_cp, N_cp);
        weighting_cmp_diff_new_.setZero(2*N_cp, 2*N_cp);
        
             
        // damping function 
        double cam_damping_gain = 50.0; // Tau = - K_damping * CAM // 높으면 -Tau 한번에 크게 발생
        
        cam_damping_mat_.setIdentity(N_cp, N_cp);
        cam_damping_mat_ = cam_damping_gain * cam_damping_mat_;
         
        // Define integral matrix
        Eigen::MatrixXd integral_matrix_temp;
        integral_matrix_temp.setOnes(N_cp, N_cp);
        for(int i = 0; i < N_cp; i++)
        {
            integral_matrix_temp(i,i) = 0.0;
        }
        
        integral_matrix_.setZero(N_cp, N_cp);
        integral_matrix_ = integral_matrix_temp.triangularView<Lower>();
        
        damping_integral_mat_.setZero(N_cp, N_cp);
        damping_integral_mat_ = T * cam_damping_mat_ * integral_matrix_;         
         
        H_damping_Nsize_x_.setZero(N_cp, N_cp);
        H_damping_Nsize_y_.setZero(N_cp, N_cp);
        H_damping_x_.setZero(2*N_cp, 2*N_cp); 
        H_damping_y_.setZero(2*N_cp, 2*N_cp);  

        // weighting_zmp_regul_.setIdentity(2*N_cp, 2*N_cp);
               
        double weighting_foot_new = 0.001; // ICRA: 0.01 
        double weighting_foot_new_y = 0.001;
        
        // Weighting parameter // Freq: 50 Hz/ Preview window: 1.5 s => N step = 75, 2N = 150
        for(int i = 0; i < N_cp; i++) // For cp control
        {
            if(i < 1)
            {                
                weighting_cp_new_x_(i,i) = 10.0; // X dir   
                weighting_cp_new_y_(i,i) = 10.0; // Y dir
            }
            else if (i < 50)
            {                
                weighting_cp_new_x_(i,i) = 5.0; // X dir     
                weighting_cp_new_y_(i,i) = 5.0; // Y dir          
            }
            else
            {                
                weighting_cp_new_x_(i,i) = 100.0; 
                weighting_cp_new_y_(i,i) = 100.0; // Y dir
            }            
        }
        for(int i = 0; i < 2*N_cp; i++) // for smoothing control input  
        {
            if(i < 2) // des.zmp, des.tau
            {
                weighting_cmp_diff_new_(i,i) = 0.2; // original gain 0.2 
            }
            else if (i < 100)
            {
                weighting_cmp_diff_new_(i,i) = 10.0; // original gain 10.0               
            }
            else
            {
                weighting_cmp_diff_new_(i,i) = 0.10; 
            }            
        }
        
        // Hessian matrix (cp control + change regulation)
        H_cp_control_x_.setZero(2*N_cp, 2*N_cp);
        H_cp_control_y_.setZero(2*N_cp, 2*N_cp);
        H_change_regul_.setZero(2*N_cp, 2*N_cp);
        H_cp_control_x_ = F_cmp_new_.transpose()*weighting_cp_new_x_*F_cmp_new_;
        H_cp_control_y_ = F_cmp_new_.transpose()*weighting_cp_new_y_*F_cmp_new_;
        H_change_regul_ = diff_matrix_new_.transpose()*weighting_cmp_diff_new_*diff_matrix_new_;
        
        H_cpmpc_new_x_.setZero(2*N_cp, 2*N_cp);  
        H_cpmpc_new_y_.setZero(2*N_cp, 2*N_cp);     
        
        H_cpStepping_mpc_new_x_.setZero(2*N_cp + footprint_num, 2*N_cp + footprint_num);
        // H_cpStepping_mpc_new_.block(0, 0, 2*N_cp, 2*N_cp) = H_cpmpc_new_; // CP-MPC         
        H_cpStepping_mpc_new_x_.block(2*N_cp, 0, footprint_num, 2*N_cp) = zeros_2Ncp_x_f.transpose();
        H_cpStepping_mpc_new_x_.block(0, 2*N_cp, 2*N_cp, footprint_num) = zeros_2Ncp_x_f;
        H_cpStepping_mpc_new_x_.block(2*N_cp, 2*N_cp, footprint_num, footprint_num) = weighting_foot_new*eye2; // Foot mpc

        H_cpStepping_mpc_new_y_.setZero(2*N_cp + footprint_num, 2*N_cp + footprint_num);
        // H_cpStepping_mpc_new_.block(0, 0, 2*N_cp, 2*N_cp) = H_cpmpc_new_; // CP-MPC         
        H_cpStepping_mpc_new_y_.block(2*N_cp, 0, footprint_num, 2*N_cp) = zeros_2Ncp_x_f.transpose();
        H_cpStepping_mpc_new_y_.block(0, 2*N_cp, 2*N_cp, footprint_num) = zeros_2Ncp_x_f;
        H_cpStepping_mpc_new_y_.block(2*N_cp, 2*N_cp, footprint_num, footprint_num) = weighting_foot_new_y*eye2; // Foot mpc
        
        // Control input (desired zmp, desired tau) initinalization 
        cpmpc_output_x_new_.setZero(2*N_cp + footprint_num);
        cpmpc_output_x_new_(0) = x_hat_(0); // Initial position of the CoM and ZMP is equal.
        
        cpmpc_output_y_new_.setZero(2*N_cp + footprint_num);
        cpmpc_output_y_new_(0) = y_hat_(0); // Initial position of the CoM and ZMP is equal.
        
        QP_cpmpc_x_new_.InitializeProblemSize(2*N_cp + footprint_num, 2*N_cp + footprint_num);
        QP_cpmpc_y_new_.InitializeProblemSize(2*N_cp + footprint_num, 2*N_cp + footprint_num);
        New_CP_MPC_first_loop = 1;
        
        cout << "Initialization of New CP_MPC parameters is complete." << endl;
    }  
    
    weighting_tau_damping_x_.setIdentity(N_cp, N_cp);
    weighting_tau_damping_y_.setIdentity(N_cp, N_cp); 
    
    for(int i = 0; i < N_cp; i++)
    {   
        weighting_tau_damping_x_(i, i) = DyrosMath::cubic(abs(cpmpc_output_x_new_(2*i) - Z_x_ref_wo_offset_new(2*i)), 0.05, 0.10, 0.00000005, 0.0, 0.0, 0.0);
        weighting_tau_damping_y_(i, i) = DyrosMath::cubic(abs(cpmpc_output_y_new_(2*i) - Z_y_ref_wo_offset_new(2*i)), 0.05, 0.07, 0.00000005, 0.0, 0.0, 0.0); // Y dir disturbance 0.00000005 // Uneven 0.0000003
    }      

    // weighting_tau_damping_x_ = 0.0000002 * weighting_tau_damping_x_;
    // weighting_tau_damping_y_ = 0.0000002 * weighting_tau_damping_y_;
    
    // for(int i = 0; i < N_cp; i++) 
    // {   
    //     weighting_zmp_regul_(i, i) = DyrosMath::cubic(abs(Z_x_ref_wo_offset_new(2*i)) + zmp_bound_x_new(2*i)*0.9 + abs(cpmpc_output_x_new_(2*N_cp)) - abs(cpmpc_output_x_new_(2*i)), 0.0, 0.04, 100.0, 0.0, 0.0, 0.0);
    //     // weighting_tau_damping_y_(i, i) = DyrosMath::cubic(abs(cpmpc_output_y_new_(2*i) - Z_y_ref_wo_offset_new(2*i)), 0.00, 0.03, 0.0000001, 0.0, 0.0, 0.0); 
    // }      
    
    H_damping_Nsize_x_ = (eyeN + damping_integral_mat_).transpose() * weighting_tau_damping_x_ * (eyeN + damping_integral_mat_);
    H_damping_Nsize_y_ = (eyeN + damping_integral_mat_).transpose() * weighting_tau_damping_y_ * (eyeN + damping_integral_mat_);

    for(int i=0; i<N_cp; i++)
    {
        for(int j=0; j<N_cp; j++)
        {
            H_damping_x_(2*i+1, 2*j+1) = H_damping_Nsize_x_(i, j);
            H_damping_y_(2*i+1, 2*j+1) = H_damping_Nsize_y_(i, j);
        }            
    }
    
    H_cpmpc_new_x_ = H_change_regul_ + H_cp_control_x_ + H_damping_x_; // + desZmp_sel_.transpose()*weighting_zmp_regul_*desZmp_sel_;
    H_cpmpc_new_y_ = H_change_regul_ + H_cp_control_y_ + H_damping_y_;
    H_cpStepping_mpc_new_x_.block(0, 0, 2*N_cp, 2*N_cp) = H_cpmpc_new_x_; // CP-MPC
    H_cpStepping_mpc_new_y_.block(0, 0, 2*N_cp, 2*N_cp) = H_cpmpc_new_y_; // CP-MPC         
   
    // For foot adjustment

    int swing_time_cur = 0, swing_time_next = 0, swing_time_n_next = 0;
    
    if(current_step_num_mpc_ > 0 && current_step_num_mpc_ != total_step_num_mpc_-1) // Define selection vector for swingfoot adjustment
    {
        swing_time_cur = (t_total_mpc_ - mpc_tick)/MPC_synchro_hz; // remaining sampling time in current foot. 

        if(N_cp - swing_time_cur >= t_total_mpc_/MPC_synchro_hz) // 3 footholds are included in N_cp step.(current, next, n_next foothold)
        {
            swing_time_next = t_total_mpc_/MPC_synchro_hz;
            swing_time_n_next = N_cp - (t_total_mpc_/MPC_synchro_hz + swing_time_cur); 
        }
        else // 2 footholds are included in N_cp step.(current, next foothold)
        {
            swing_time_next = N_cp - swing_time_cur;
            swing_time_n_next = 0;
        } 

        Eigen::VectorXd sel_swingfoot(swing_time_cur); 
        Eigen::VectorXd sel_swingfoot_next(swing_time_next);  
        Eigen::VectorXd sel_swingfoot_n_next(swing_time_n_next);  
        
        sel_swingfoot.setZero();        
        sel_swingfoot_next.setOnes();
        
        P_sel.setZero(N_cp, footprint_num);  
        P_sel.block(0, 0, swing_time_cur, 1) = sel_swingfoot;        
        P_sel.block(swing_time_cur, 0, swing_time_next, 1) = sel_swingfoot_next;  

        if(swing_time_n_next != 0)
        {            
            sel_swingfoot_n_next.setOnes(); 
            P_sel.block(swing_time_cur + swing_time_next, 1, swing_time_n_next, 1) = sel_swingfoot_n_next;
        } 
    }
    else
    {       
        P_sel.setZero(N_cp,footprint_num); 
    }
    P_sel_new.setZero(2*N_cp, footprint_num);
    
    for(int j = 0; j < footprint_num; j ++)
    {
        for(int i = 0; i < N_cp; i++)
        {
            P_sel_new(2*i,j) = P_sel(i,j);
        }        
    }

    Eigen::VectorXd onevector_N;
    onevector_N.setOnes(N_cp);
    Eigen::VectorXd g_damping_Nsize_x;
    Eigen::VectorXd g_damping_Nsize_y;
    Eigen::VectorXd g_damping_x;
    Eigen::VectorXd g_damping_y;
    g_damping_Nsize_x.setZero(N_cp);
    g_damping_Nsize_y.setZero(N_cp);
    g_damping_x.setZero(2*N_cp);
    g_damping_Nsize_x = (eyeN + damping_integral_mat_).transpose() * (cam_mpc_init_(1)*weighting_tau_damping_x_.transpose()*cam_damping_mat_*onevector_N); // 0.01에 들어가는 value를 h0로
    g_damping_y.setZero(2*N_cp);
    g_damping_Nsize_y = (eyeN + damping_integral_mat_).transpose() * (-cam_mpc_init_(0)*weighting_tau_damping_y_.transpose()*cam_damping_mat_*onevector_N); // 0.01에 들어가는 value를 h0로
     
    for(int i=0; i<N_cp; i++)
    {
        g_damping_x(2*i+1) = g_damping_Nsize_x(i);
        g_damping_y(2*i+1) = g_damping_Nsize_y(i);
    }       

    Eigen::VectorXd g_cpmpc_x_new(2*N_cp);  
    Eigen::VectorXd g_cpmpc_y_new(2*N_cp);
    Eigen::VectorXd g_cpStepping_mpc_x_new(2*N_cp + footprint_num);
    Eigen::VectorXd g_cpStepping_mpc_y_new(2*N_cp + footprint_num);
    
    // gradient vector
    g_cpmpc_x_new = g_damping_x + F_cmp_new_.transpose()*weighting_cp_new_x_*(F_cp_new_*cp_measured_mpc_(0) - cp_x_ref_new) - diff_matrix_new_.transpose()*weighting_cmp_diff_new_*e1_cpmpc_new_*cpmpc_output_x_new_.segment<2>(0);
    g_cpStepping_mpc_x_new.setZero(2*N_cp + footprint_num);
    g_cpStepping_mpc_x_new.segment(0, 2*N_cp) = g_cpmpc_x_new;

    g_cpmpc_y_new = g_damping_y + F_cmp_new_.transpose()*weighting_cp_new_y_*(F_cp_new_*cp_measured_mpc_(1) - cp_y_ref_new) - diff_matrix_new_.transpose()*weighting_cmp_diff_new_*e1_cpmpc_new_*cpmpc_output_y_new_.segment<2>(0);
    g_cpStepping_mpc_y_new.setZero(2*N_cp + footprint_num);
    g_cpStepping_mpc_y_new.segment(0, 2*N_cp) = g_cpmpc_y_new;
    
    // constraint formulation
    Eigen::MatrixXd A_cpStepping_mpc_new(2*N_cp + footprint_num, 2*N_cp + footprint_num);
    Eigen::MatrixXd A_cp_mpc_new(2*N_cp, 2*N_cp);  
    A_cp_mpc_new.setIdentity();

    // Eigen::MatrixXd eye2_fake(2,2);
    // eye2_fake.setZero();
    // eye2_fake(0,0) = 1.0;
    A_cpStepping_mpc_new.block(0, 0, 2*N_cp, 2*N_cp) = A_cp_mpc_new;  
    A_cpStepping_mpc_new.block(0, 2*N_cp, 2*N_cp, footprint_num) = -P_sel_new; //
    A_cpStepping_mpc_new.block(2*N_cp, 0, footprint_num, 2*N_cp) = zeros_2Ncp_x_f.transpose();
    A_cpStepping_mpc_new.block(2*N_cp, 2*N_cp, footprint_num, footprint_num) = eye2;

    Eigen::VectorXd ub_x_cp_mpc_new(2*N_cp);
    Eigen::VectorXd lb_x_cp_mpc_new(2*N_cp);
    Eigen::VectorXd ub_y_cp_mpc_new(2*N_cp);
    Eigen::VectorXd lb_y_cp_mpc_new(2*N_cp);    

    Eigen::VectorXd ub_x_foot_cp_mpc_new(footprint_num);
    Eigen::VectorXd lb_x_foot_cp_mpc_new(footprint_num);
    Eigen::VectorXd ub_y_foot_cp_mpc_new(footprint_num);
    Eigen::VectorXd lb_y_foot_cp_mpc_new(footprint_num);   

    Eigen::VectorXd ub_x_cpStepping_mpc_new(2*N_cp + footprint_num);
    Eigen::VectorXd lb_x_cpStepping_mpc_new(2*N_cp + footprint_num);
    Eigen::VectorXd ub_y_cpStepping_mpc_new(2*N_cp + footprint_num);
    Eigen::VectorXd lb_y_cpStepping_mpc_new(2*N_cp + footprint_num);    
       
        
    lb_x_cp_mpc_new = (Z_x_ref_wo_offset_new - zmp_bound_x_new * 0.9) - (Tau_y_limit);
    ub_x_cp_mpc_new = (Z_x_ref_wo_offset_new + zmp_bound_x_new * 1.2) + (Tau_y_limit);

    lb_y_cp_mpc_new = (Z_y_ref_wo_offset_new - zmp_bound_y_new) - (Tau_x_limit); // Z_y_ref is the trajectory considering the ZMP offset for COM planning.
    ub_y_cp_mpc_new = (Z_y_ref_wo_offset_new + zmp_bound_y_new) + (Tau_x_limit); // However, Ref. ZMP without ZMP offset is required for CP control.
       
    ub_x_foot_cp_mpc_new.setZero();   
    lb_x_foot_cp_mpc_new.setZero();
    ub_y_foot_cp_mpc_new.setZero();   
    lb_y_foot_cp_mpc_new.setZero();

    double del_F_y_rightswing_min = -0.08, del_F_y_rightswing_max = 0.03;
    double del_F_y_leftswing_min = -0.03, del_F_y_leftswing_max = 0.08;
      
    // real robot experiment 0.2? 
    // 1을 줄이면 0이 더 많이 생길수도? 별 차이없음
    if(alpha_step_mpc_ == 1) // left foot support
    {
        ub_x_foot_cp_mpc_new(0) = 0.2 - rfoot_support_current_mpc_.translation()(0);
        lb_x_foot_cp_mpc_new(0) = -0.2 - rfoot_support_current_mpc_.translation()(0);
        ub_x_foot_cp_mpc_new(1) = 0.2; 
        lb_x_foot_cp_mpc_new(1) = -0.2; 

        // standard value of rfoot_support_current = -0.25
        ub_y_foot_cp_mpc_new(0) = -0.22 - rfoot_support_current_mpc_.translation()(1); // 0.03 // -0.22
        lb_y_foot_cp_mpc_new(0) = -0.33 - rfoot_support_current_mpc_.translation()(1); //-0.1 // -0.35
        ub_y_foot_cp_mpc_new(1) = del_F_y_leftswing_max;
        lb_y_foot_cp_mpc_new(1) = del_F_y_leftswing_min; 
    }
    else if(alpha_step_mpc_ == -1) // right foot support
    {
        ub_x_foot_cp_mpc_new(0) = 0.2 - lfoot_support_current_mpc_.translation()(0);
        lb_x_foot_cp_mpc_new(0) = -0.2 - lfoot_support_current_mpc_.translation()(0);
        ub_x_foot_cp_mpc_new(1) = 0.2; 
        lb_x_foot_cp_mpc_new(1) = -0.2; 

        // standard value of lfoot_support_current = +0.25
        ub_y_foot_cp_mpc_new(0) =  0.33 - lfoot_support_current_mpc_.translation()(1); // 0.1 // 0.35
        lb_y_foot_cp_mpc_new(0) =  0.22 - lfoot_support_current_mpc_.translation()(1); // -0.03 // 0.22
        ub_y_foot_cp_mpc_new(1) = del_F_y_rightswing_max;
        lb_y_foot_cp_mpc_new(1) = del_F_y_rightswing_min; 
    }   

    lb_x_cpStepping_mpc_new.segment(0, 2*N_cp) = lb_x_cp_mpc_new;
    lb_x_cpStepping_mpc_new.segment(2*N_cp, footprint_num) = lb_x_foot_cp_mpc_new;
    ub_x_cpStepping_mpc_new.segment(0, 2*N_cp) = ub_x_cp_mpc_new;
    ub_x_cpStepping_mpc_new.segment(2*N_cp, footprint_num) = ub_x_foot_cp_mpc_new; 

    lb_y_cpStepping_mpc_new.segment(0, 2*N_cp) = lb_y_cp_mpc_new;
    lb_y_cpStepping_mpc_new.segment(2*N_cp, footprint_num) = lb_y_foot_cp_mpc_new;
    ub_y_cpStepping_mpc_new.segment(0, 2*N_cp) = ub_y_cp_mpc_new;
    ub_y_cpStepping_mpc_new.segment(2*N_cp, footprint_num) = ub_y_foot_cp_mpc_new; 
    
    // Define QP problem for new CP-MPC  
    QP_cpmpc_x_new_.EnableEqualityCondition(equality_condition_eps_);
    QP_cpmpc_x_new_.UpdateMinProblem(H_cpStepping_mpc_new_x_,g_cpStepping_mpc_x_new);
    QP_cpmpc_x_new_.DeleteSubjectToAx();      
    QP_cpmpc_x_new_.UpdateSubjectToAx(A_cpStepping_mpc_new, lb_x_cpStepping_mpc_new, ub_x_cpStepping_mpc_new);     
          
    if (QP_cpmpc_x_new_.SolveQPoases(200, cpmpc_input_x_new_))
    {                     
        cpmpc_output_x_new_ = cpmpc_input_x_new_.segment(0, 2*N_cp + footprint_num); 
                
        if(atb_cpmpc_x_update_ == false)
        {
            atb_cpmpc_x_update_ = true;
            del_F_x_thread_ = cpmpc_output_x_new_(2*N_cp);
            cpmpc_des_zmp_x_thread_ = cpmpc_output_x_new_(0);
            des_tau_y_thread_ = cpmpc_output_x_new_(1);
            atb_cpmpc_x_update_ = false;
        }       
        cpmpc_x_update_ = true;
    } 
    
    QP_cpmpc_y_new_.EnableEqualityCondition(equality_condition_eps_);
    QP_cpmpc_y_new_.UpdateMinProblem(H_cpStepping_mpc_new_y_,g_cpStepping_mpc_y_new);
    QP_cpmpc_y_new_.DeleteSubjectToAx();      
    QP_cpmpc_y_new_.UpdateSubjectToAx(A_cpStepping_mpc_new, lb_y_cpStepping_mpc_new, ub_y_cpStepping_mpc_new);       

    if (QP_cpmpc_y_new_.SolveQPoases(200, cpmpc_input_y_new_))
    {                     
        cpmpc_output_y_new_ = cpmpc_input_y_new_.segment(0, 2*N_cp + footprint_num); 
                
        if(atb_cpmpc_y_update_ == false)
        {
            atb_cpmpc_y_update_ = true;
            del_F_y_thread_ = cpmpc_output_y_new_(2*N_cp);
            cpmpc_des_zmp_y_thread_ = cpmpc_output_y_new_(0);
            des_tau_x_thread_ = cpmpc_output_y_new_(1);
            atb_cpmpc_y_update_ = false;
        }       
        cpmpc_y_update_ = true;
    } 

    Eigen::VectorXd cp_predicted_x(N_cp);        
    Eigen::VectorXd cp_predicted_y(N_cp);

    cp_predicted_x = F_cp_new_*cp_measured_mpc_(0) + F_cmp_new_*cpmpc_output_x_new_.segment(0, 2*N_cp);
    cp_predicted_y = F_cp_new_*cp_measured_mpc_(1) + F_cmp_new_*cpmpc_output_y_new_.segment(0, 2*N_cp);   
    
    if(mpc_tick <= t_total_mpc_ - t_rest_last_ - t_double2_)
    {
        int landing_mpc_time = int((t_total_mpc_ - mpc_tick - t_rest_last_ - t_double2_)/MPC_synchro_hz );
        // cp_eos_x_mpc_ = cp_x_ref_new(landing_mpc_time); 
        // cp_eos_y_mpc_ = cp_y_ref_new(landing_mpc_time);

        cp_eos_x_cpmpc_ = cp_predicted_x(landing_mpc_time); 
        cp_eos_y_cpmpc_ = cp_predicted_y(landing_mpc_time);      
    }
    // t_start_, t_rest_init_, t_double1 should be replaced with mpc variable
    double des_cmp_ssp_tmp_x = 0;
    double des_cmp_ssp_tmp_y = 0;
    
    // double del_zmp_ssp_tmp_y = 0;
    // double del_zmp_ssp_mpc_y = 0;
    int landing_mpc_time_sum = 0;
    int landing_mpc_time_decrease = 0;

    if(walking_tick_mj_mpc_ >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj_mpc_ < t_start_ + t_total_mpc_ - t_rest_last_ - t_double2_)
    {    
        int landing_mpc_time = int((t_total_mpc_ - mpc_tick - t_rest_last_ - t_double2_)/MPC_synchro_hz);
        landing_mpc_time_decrease = landing_mpc_time;
        ///////////////////////////////////////////// #1 CMP current value 
        // des_cmp_ssp_mpc_x_ = cpmpc_output_x_new_(0) + cpmpc_output_x_new_(1)/940.0;
        // des_cmp_ssp_mpc_y_ = cpmpc_output_y_new_(0) - cpmpc_output_y_new_(1)/940.0;
        
        ///////////////////////////////////////////// #2 CMP average 
        for(int i = 0; i < landing_mpc_time; i ++)
        {   // desired zmp is equal to delta zmp in SSP.
             
            des_cmp_ssp_tmp_x += (cpmpc_output_x_new_(2*i) + cpmpc_output_x_new_(2*i + 1)/940.0); 
            des_cmp_ssp_tmp_y += (cpmpc_output_y_new_(2*i) - cpmpc_output_y_new_(2*i + 1)/940.0); 
        }
        if(landing_mpc_time != 0)
        {   
            des_cmp_ssp_mpc_x_ = des_cmp_ssp_tmp_x / landing_mpc_time;
            des_cmp_ssp_mpc_y_ = des_cmp_ssp_tmp_y / landing_mpc_time;
            // del_zmp_ssp_mpc_y = del_zmp_ssp_tmp_y / landing_mpc_time;
        }
        else
        {
            des_cmp_ssp_mpc_x_ = des_cmp_ssp_tmp_x;
            des_cmp_ssp_mpc_y_ = des_cmp_ssp_tmp_y;
            // del_zmp_ssp_mpc_y = del_zmp_ssp_tmp_y;
        }      

        /////////////////////////////////////////// #3 CMP weighted average
        // for(int i = 0; i < landing_mpc_time; i ++)
        // {   // desired zmp is equal to delta zmp in SSP.
            
        //     des_cmp_ssp_tmp_x += (cpmpc_output_x_new_(2*i) + cpmpc_output_x_new_(2*i + 1)/940.0) * landing_mpc_time_decrease; 
        //     des_cmp_ssp_tmp_y += (cpmpc_output_y_new_(2*i) - cpmpc_output_y_new_(2*i + 1)/940.0) * landing_mpc_time_decrease; 
        //     landing_mpc_time_sum = landing_mpc_time_sum + landing_mpc_time_decrease;
        //     landing_mpc_time_decrease = landing_mpc_time_decrease * 0.98;        
        //     // landing_mpc_time_decrease = landing_mpc_time_decrease - 1;      
        //     // del_zmp_ssp_tmp_y += cpmpc_output_y_new_(2*i) - Z_y_ref_wo_offset_new(2*i);
        //     // MJ_graph << landing_mpc_time_decrease << endl;
        // }
        // if(landing_mpc_time != 0)
        // {
        //     des_cmp_ssp_mpc_x_ = des_cmp_ssp_tmp_x / landing_mpc_time_sum;
        //     des_cmp_ssp_mpc_y_ = des_cmp_ssp_tmp_y / landing_mpc_time_sum;
        //     // del_zmp_ssp_mpc_y = del_zmp_ssp_tmp_y / landing_mpc_time;
        // }
        // else
        // {
        //     des_cmp_ssp_mpc_x_ = des_cmp_ssp_tmp_x;
        //     des_cmp_ssp_mpc_y_ = des_cmp_ssp_tmp_y;
        //     // del_zmp_ssp_mpc_y = del_zmp_ssp_tmp_y;
        // }                       
    }
    else
    {
        des_cmp_ssp_mpc_x_ = 0;
        des_cmp_ssp_mpc_y_ = 0;
    }
    
    // static int aa = 0;
    // if(walking_tick_mj_mpc_ >= 6.0*2000 && aa == 0)
    // {
    //     aa = 1;

    //     for(int i = 0; i < N_cp; i ++)
    //     {
    //         MJ_graph << Z_y_ref_wo_offset_new(2*i) << "," << cpmpc_output_y_new_(2*i) << endl;
    //     }
    // }
    
    // for(int i = 0; i < N_cp; i ++)
    // {        
    //     MJ_graph2 << weighting_tau_damping_x_(i, i) << "," << i << "," << walking_tick_mj_mpc_ << endl; 
    // } 

    // MJ_graph1 << des_cmp_ssp_mpc_x_ << "," << cpmpc_output_x_new_(0) << "," << des_cmp_ssp_mpc_y_ << "," << cpmpc_output_y_new_(0) << endl;
    // MJ_graph2 << Z_x_ref_wo_offset_new(0) << "," << Z_y_ref_wo_offset_new(0) << "," << cpmpc_output_x_new_(0) << "," << cpmpc_output_y_new_(0) << "," << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << endl;    
    current_step_num_mpc_new_prev_ = current_step_num_mpc_;
    // std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now(); cp_x_ref_new(0) cp_measured_mpc_(0)   

    // // CP-MPC Journal - CPMPC data (Ref.Des ZMP, Centroidal moment, CAM, Del F)
    MJ_graph  << Z_x_ref_wo_offset_new(0) << "," << cpmpc_output_x_new_(0) << "," << cpmpc_output_x_new_(1) << "," << cam_mpc_init_(1) << "," << cpmpc_output_x_new_(2*N_cp) << "," << cp_x_ref_new(0) << "," << cp_measured_mpc_(0) << endl; 
    MJ_graph1 << Z_y_ref_wo_offset_new(0) << "," << cpmpc_output_y_new_(0) << "," << cpmpc_output_y_new_(1) << "," << cam_mpc_init_(0) << "," << cpmpc_output_y_new_(2*N_cp) << "," << cp_y_ref_new(0) << "," << cp_measured_mpc_(1) << endl; 
  
    // // // CPMPC Journal foot trajecotry data
    MJ_graph_foottra_x << del_F_(0) << "," << lfoot_trajectory_support_.translation()(0) << "," << rfoot_trajectory_support_.translation()(0) << "," << u0_x_data_ << "," << t_total_/hz_ << endl;
    MJ_graph_foottra_y << del_F_(1) << "," << lfoot_trajectory_support_.translation()(1) << "," << rfoot_trajectory_support_.translation()(1) << "," << u0_y_data_ << endl;
}

void AvatarController::savePreData()
{
    pre_time_ = current_time_;
    pre_q_ = rd_.q_;
    pre_desired_q_ = desired_q_;
    pre_desired_q_dot_ = desired_q_dot_;
    motion_q_pre_ = motion_q_;
    motion_q_dot_pre_ = motion_q_dot_;

    zmp_measured_ppre_ = zmp_measured_pre_;
    zmp_measured_pre_ = zmp_measured_;
    com_pos_desired_pre_ = com_pos_desired_;
    com_vel_desired_pre_ = com_vel_desired_;
    com_acc_desired_pre_ = com_acc_desired_;

    com_pos_ppre_from_support_ = com_pos_pre_from_support_;
    com_vel_ppre_from_support_ = com_vel_pre_from_support_;
    com_acc_ppre_from_support_ = com_acc_pre_from_support_;

    com_pos_pre_from_support_ = com_pos_current_from_support_;
    com_vel_pre_from_support_ = com_vel_current_from_support_;
    com_acc_pre_from_support_ = com_acc_current_from_support_;

    com_vel_ppre_lpf_from_support_ = com_vel_pre_lpf_from_support_;
    com_vel_pre_lpf_from_support_ = com_vel_current_lpf_from_support_;

    if (current_time_ == preview_update_time_)
    {
        com_pos_pre_desired_from_support_ = com_pos_desired_from_support_;
        com_vel_pre_desired_from_support_ = com_vel_desired_from_support_;
        com_acc_pre_desired_from_support_ = com_acc_desired_from_support_;
        com_jerk_pre_desired_from_support_ = com_jerk_desired_from_support_;
    }

    f_star_xy_pre_ = f_star_xy_;
    f_star_6d_pre_ = f_star_6d_;
    f_star_l_pre_ = f_star_l_;
    f_star_r_pre_ = f_star_r_;

    torque_task_pre_ = torque_task_;
    torque_grav_pre_ = torque_grav_;

    foot_contact_pre_ = foot_contact_;

    A_mat_pre_ = A_mat_;

    zmp_desired_pre_ = zmp_desired_from_global_;
    zmp_local_lfoot_pre_ = zmp_local_lfoot_;
    zmp_local_rfoot_pre_ = zmp_local_rfoot_;

    swing_foot_transform_pre_ = swing_foot_transform_current_;
    support_foot_transform_pre_ = support_foot_transform_current_;
    swing_foot_transform_pre_from_support_ = swing_foot_transform_current_from_support_;
    support_foot_transform_pre_from_support_ = support_foot_transform_current_from_support_;

    com_pos_desired_preview_pre_ = com_pos_desired_preview_;
    com_vel_desired_preview_pre_ = com_vel_desired_preview_;
    com_acc_desired_preview_pre_ = com_acc_desired_preview_;

    swingfoot_f_star_l_pre_ = swingfoot_f_star_l_;
    swingfoot_f_star_r_pre_ = swingfoot_f_star_r_;

    f_lfoot_damping_pre_ = f_lfoot_damping_;
    f_rfoot_damping_pre_ = f_rfoot_damping_;

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
}

void AvatarController::WalkingSliderCommandCallback(const std_msgs::Float32MultiArray &msg)
{
    walking_speed_ = msg.data[0];
    walking_speed_ = DyrosMath::minmax_cut(walking_speed_, -0.1, 0.1);

    // walking_duration_cmd_ = msg.data[1];
    // walking_duration_cmd_ = DyrosMath::minmax_cut(walking_duration_cmd_, 0.4, 1.0);

    // yaw_angular_vel_ = msg.data[2];
    // yaw_angular_vel_ = DyrosMath::minmax_cut(yaw_angular_vel_, -0.3, 0.3);

    // knee_target_angle_ = msg.data[3];
    // knee_target_angle_ = DyrosMath::minmax_cut(knee_target_angle_, 0.0, M_PI/2);

    // swing_foot_height_ = msg.data[4];
    // swing_foot_height_ = DyrosMath::minmax_cut(swing_foot_height_, 0.005, 0.10);
}

void AvatarController::UpperbodyModeCallback(const std_msgs::Int8 &msg)
{
    upper_body_mode_ = msg.data;
    upperbody_mode_recieved_ = true;
}

void AvatarController::NextSwinglegCallback(const std_msgs::Float32 &msg)
{
    foot_contact_ = msg.data;
}

void AvatarController::ComPosGainCallback(const std_msgs::Float32MultiArray &msg)
{
    kp_compos_(0, 0) = msg.data[0];
    kp_compos_(1, 1) = msg.data[1];
    kp_compos_(2, 2) = msg.data[2];
    kd_compos_(0, 0) = msg.data[3];
    kd_compos_(1, 1) = msg.data[4];
    kd_compos_(2, 2) = msg.data[5];
}

void AvatarController::PelvOriGainCallback(const std_msgs::Float32MultiArray &msg)
{
    kp_pelv_ori_(0, 0) = msg.data[0];
    kp_pelv_ori_(1, 1) = msg.data[1];
    kp_pelv_ori_(2, 2) = msg.data[2];
    kd_pelv_ori_(0, 0) = msg.data[3];
    kd_pelv_ori_(1, 1) = msg.data[4];
    kd_pelv_ori_(2, 2) = msg.data[5];
}

void AvatarController::SupportFootDampingGainCallback(const std_msgs::Float32MultiArray &msg)
{
    support_foot_damping_gain_(0, 0) = msg.data[0];
    support_foot_damping_gain_(1, 1) = msg.data[1];
    support_foot_damping_gain_(2, 2) = msg.data[2];
}

void AvatarController::LegJointGainCallback(const std_msgs::Float32MultiArray &msg)
{
    kp_stiff_joint_(0) = msg.data[0];
    kp_stiff_joint_(1) = msg.data[1];
    kp_stiff_joint_(2) = msg.data[2];
    kp_stiff_joint_(3) = msg.data[3];
    kp_stiff_joint_(4) = msg.data[4];
    kp_stiff_joint_(5) = msg.data[5];

    kp_stiff_joint_(6) = msg.data[6];
    kp_stiff_joint_(7) = msg.data[7];
    kp_stiff_joint_(8) = msg.data[8];
    kp_stiff_joint_(9) = msg.data[9];
    kp_stiff_joint_(10) = msg.data[10];
    kp_stiff_joint_(11) = msg.data[11];

    kv_stiff_joint_(0) = msg.data[12 + 0];
    kv_stiff_joint_(1) = msg.data[12 + 1];
    kv_stiff_joint_(2) = msg.data[12 + 2];
    kv_stiff_joint_(3) = msg.data[12 + 3];
    kv_stiff_joint_(4) = msg.data[12 + 4];
    kv_stiff_joint_(5) = msg.data[12 + 5];

    kv_stiff_joint_(6) = msg.data[12 + 6];
    kv_stiff_joint_(7) = msg.data[12 + 7];
    kv_stiff_joint_(8) = msg.data[12 + 8];
    kv_stiff_joint_(9) = msg.data[12 + 9];
    kv_stiff_joint_(10) = msg.data[12 + 10];
    kv_stiff_joint_(11) = msg.data[12 + 11];
}

void AvatarController::AlphaXCallback(const std_msgs::Float32 &msg)
{
    alpha_x_command_ = msg.data;
}

void AvatarController::AlphaYCallback(const std_msgs::Float32 &msg)
{
    alpha_y_command_ = msg.data;
}

void AvatarController::StepWidthCommandCallback(const std_msgs::Float32 &msg)
{
    step_width_ = msg.data;
}

void AvatarController::Test1CommandCallback(const std_msgs::Float32 &msg)
{
    zmp_y_offset_ = msg.data;
}

void AvatarController::Test2CommandCallback(const std_msgs::Float32 &msg)
{
}

void AvatarController::ArmJointGainCallback(const std_msgs::Float32MultiArray &msg)
{
    //left arm kp
    kp_joint_(15) = msg.data[0];
    kp_joint_(16) = msg.data[1];
    kp_joint_(17) = msg.data[2];
    kp_joint_(18) = msg.data[3];
    kp_joint_(19) = msg.data[4];
    kp_joint_(20) = msg.data[5];
    kp_joint_(21) = msg.data[6];
    kp_joint_(22) = msg.data[7];
    //right arm kp
    kp_joint_(25) = msg.data[0];
    kp_joint_(26) = msg.data[1];
    kp_joint_(27) = msg.data[2];
    kp_joint_(28) = msg.data[3];
    kp_joint_(29) = msg.data[4];
    kp_joint_(30) = msg.data[5];
    kp_joint_(31) = msg.data[6];
    kp_joint_(32) = msg.data[7];

    //left arm kd
    kv_joint_(15) = msg.data[8];
    kv_joint_(16) = msg.data[9];
    kv_joint_(17) = msg.data[10];
    kv_joint_(18) = msg.data[11];
    kv_joint_(19) = msg.data[12];
    kv_joint_(20) = msg.data[13];
    kv_joint_(21) = msg.data[14];
    kv_joint_(22) = msg.data[15];
    //right arm kd
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
    else if (msg.data == 2) //T pose
    {
        hmd_check_pose_calibration_[1] = true;
        cout << "T Pose Calibration is On." << endl;
    }
    else if (msg.data == 3) //forward stretch
    {
        hmd_check_pose_calibration_[2] = true;
        cout << "Forward Stretch Pose Calibration is On." << endl;
    }
    else if (msg.data == 4) //reset callibration
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

        hmd_init_pose_calibration_ = true;
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

    hmd_pelv_pose_raw_.linear() = hmd_pelv_pose_raw_.linear() * DyrosMath::rotateWithZ(M_PI); //tracker is behind the chair
}

void AvatarController::TrackerStatusCallback(const std_msgs::Bool &msg)
{
    hmd_tracker_status_raw_ = msg.data;
}
 
// real robot experiment
void AvatarController::OptoforceFTCallback(const tocabi_msgs::FTsensor &msg)
{
    opto_ft_raw_(0) = msg.Fx;
    opto_ft_raw_(1) = msg.Fy;
    opto_ft_raw_(2) = msg.Fz;
    opto_ft_raw_(3) = msg.Tx;
    opto_ft_raw_(4) = msg.Ty;
    opto_ft_raw_(5) = msg.Tz;
}

Eigen::MatrixXd AvatarController::discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q)
{
    int n = a.rows(); //number of rows
    int m = b.cols(); //number of columns

    Eigen::MatrixXd z11(n, n), z12(n, n), z21(n, n), z22(n, n);

    z11 = a.inverse();
    z12 = a.inverse() * b * r.inverse() * b.transpose();
    z21 = q * a.inverse();
    z22 = a.transpose() + q * a.inverse() * b * r.inverse() * b.transpose();

    Eigen::MatrixXd z;
    z.setZero(2 * n, 2 * n);
    z.topLeftCorner(n, n) = z11;
    z.topRightCorner(n, n) = z12;
    z.bottomLeftCorner(n, n) = z21;
    z.bottomRightCorner(n, n) = z22;

    std::vector<Eigen::VectorXd> eigVec_real(2 * n);
    std::vector<Eigen::VectorXd> eigVec_img(2 * n);

    for (int i = 0; i < 8; i++)
    {
        eigVec_real[i].setZero(2 * n);
        eigVec_img[i].setZero(2 * n);
    }

    Eigen::VectorXd deigVal_real(2 * n);
    Eigen::VectorXd deigVal_img(2 * n);
    deigVal_real.setZero();
    deigVal_img.setZero();
    Eigen::MatrixXd deigVec_real(2 * n, 2 * n);
    Eigen::MatrixXd deigVec_img(2 * n, 2 * n);
    deigVec_real.setZero();
    deigVec_img.setZero();

    deigVal_real = z.eigenvalues().real();
    deigVal_img = z.eigenvalues().imag();
    Eigen::EigenSolver<Eigen::MatrixXd> ev(z);

    for (int i = 0; i < 2 * n; i++)
    {
        for (int j = 0; j < 2 * n; j++)
        {
            deigVec_real(j, i) = ev.eigenvectors().col(i)(j).real();
            deigVec_img(j, i) = ev.eigenvectors().col(i)(j).imag();
        }
    }

    //Order the eigenvectors
    //move e-vectors correspnding to e-value outside the unite circle to the left

    Eigen::MatrixXd tempZ_real(2 * n, n), tempZ_img(2 * n, n);
    tempZ_real.setZero();
    tempZ_img.setZero();
    int c = 0;

    for (int i = 0; i < 2 * n; i++)
    {
        if ((deigVal_real(i) * deigVal_real(i) + deigVal_img(i) * deigVal_img(i)) > 1.0) //outside the unit cycle
        {
            for (int j = 0; j < 2 * n; j++)
            {
                tempZ_real(j, c) = deigVec_real(j, i);
                tempZ_img(j, c) = deigVec_img(j, i);
            }
            c++;
        }
    }

    Eigen::MatrixXcd tempZ_comp(2 * n, n);
    for (int i = 0; i < 2 * n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            tempZ_comp.real()(i, j) = tempZ_real(i, j);
            tempZ_comp.imag()(i, j) = tempZ_img(i, j);
        }
    }
    Eigen::MatrixXcd U11(n, n), U21(n, n), X(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            U11(i, j) = tempZ_comp(i, j);
            U21(i, j) = tempZ_comp(i + n, j);
        }
    }
    X = U21 * (U11.inverse());

    Eigen::MatrixXd X_sol(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            X_sol(i, j) = X.real()(i, j);
        }
    }

    return X_sol;
}


void AvatarController::fallDetection()
{
    Vector3d pelv_angle_to_g;
    pelv_angle_to_g = DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
    pelv_angle_to_g(0) = abs(pelv_angle_to_g(0));
    pelv_angle_to_g(1) = abs(pelv_angle_to_g(1));
    pelv_angle_to_g(2) = abs(pelv_angle_to_g(2));

    if (falling_detection_flag_ == false)
    {
        if ((pelv_angle_to_g(0) > 20 * DEG2RAD) || (pelv_angle_to_g(1) > 20 * DEG2RAD))
        {
            falling_detection_flag_ = true;
            fall_init_q_ = current_q_;
            fall_start_time_ = current_time_;
            cout << "###########FALLING IS DETECTED###################" << endl;
        }

        if (l_ft_.segment(0, 3).norm() + r_ft_.segment(0, 3).norm() < rd_.link_[COM_id].mass * GRAVITY * 0.1)
        {
            foot_lift_count_ += 1;
        }
        else
        {
            foot_lift_count_ = 0;
        }

        if (foot_lift_count_ > 600)
        {
            falling_detection_flag_ = true;
            fall_init_q_ = current_q_;
            fall_start_time_ = current_time_;
            // cout<<"###########FEET IS DETACHED###################"<<endl;
        }
    }

    if (falling_detection_flag_ == true)
    {
        torque_g_ = WBC::GravityCompensationTorque(rd_);

        for (int i = 0; i < MODEL_DOF; i++)
        {
            desired_q_(i) = DyrosMath::QuinticSpline(current_time_, fall_start_time_, fall_start_time_ + 3.0, fall_init_q_(i), 0, 0, zero_q_(i), 0, 0)(0);
            desired_q_dot_(i) = DyrosMath::QuinticSpline(current_time_, fall_start_time_, fall_start_time_ + 3.0, fall_init_q_(i), 0, 0, zero_q_(i), 0, 0)(1);
            torque_task_(i) = (kp_joint_(i) * (desired_q_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_(i) - current_q_dot_(i))) + torque_g_(i);
        }
    }
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
    // if (int( (current_time_ - program_start_time_) * 2000) % 1 == 0) // 2000 hz
    // if ((current_time_ - program_start_time_) >= 0.0)
    // {
    // file[0] << current_time_ - program_start_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"
    //         << foot_swing_trigger_ << "\t" << first_step_trigger_ << "\t" << start_walking_trigger_ << "\t"
    //         << stop_walking_trigger_ << "\t" << stance_start_time_ << "\t" << walking_duration_ << "\t"
    //         << turning_duration_ << "\t" << turning_phase_ << "\t" << knee_target_angle_ << endl;

    // file[1] << current_time_ - program_start_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"                                   //1
    //         << com_pos_current_from_support_(0) << "\t" << com_pos_current_from_support_(1) << "\t" << com_pos_current_from_support_(2) << "\t" //4
    //         << com_vel_current_from_support_(0) << "\t" << com_vel_current_from_support_(1) << "\t" << com_vel_current_from_support_(2) << "\t" //7
    //         << com_acc_current_from_support_(0) << "\t" << com_acc_current_from_support_(1) << "\t" << com_acc_current_from_support_(2) << "\t" //10
    //         << com_pos_desired_from_support_(0) << "\t" << com_pos_desired_from_support_(1) << "\t" << com_pos_desired_from_support_(2) << "\t" //13
    //         << com_vel_desired_from_support_(0) << "\t" << com_vel_desired_from_support_(1) << "\t" << com_vel_desired_from_support_(2) << "\t" //16
    //         << com_acc_desired_from_support_(0) << "\t" << com_acc_desired_from_support_(1) << "\t" << com_acc_desired_from_support_(2) << endl;

    // file[2]<<ref_zmp_(0,0)<<"\t"<<ref_zmp_(0,1)<<"\t"<< zmp_current_by_com_from_support_(0)<<"\t"<< zmp_current_by_com_from_support_(1)<<endl;

    // for (int i = 0; i < zmp_size_; i++)
    // {
    //     if (i % 10 == 0)
    //         file[2] << ref_zmp_(i, 0) << "\t";
    // }
    // for (int i = 0; i < zmp_size_; i++)
    // {
    //     if (i % 10 == 0)
    //         file[2] << ref_zmp_(i, 1) << "\t";
    // }
    // file[2] << zmp_current_by_com_from_support_(0) << "\t" << zmp_current_by_com_from_support_(1) << endl;

    // file[3] << current_time_ - program_start_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"                                                                                                     //1
    //         << lfoot_transform_current_from_support_.translation()(0) << "\t" << lfoot_transform_current_from_support_.translation()(1) << "\t" << lfoot_transform_current_from_support_.translation()(2) << "\t" //4
    //         << rfoot_transform_current_from_support_.translation()(0) << "\t" << rfoot_transform_current_from_support_.translation()(1) << "\t" << rfoot_transform_current_from_support_.translation()(2) << "\t" //7
    //         << swing_foot_pos_trajectory_from_support_(0) << "\t" << swing_foot_pos_trajectory_from_support_(1) << "\t" << swing_foot_pos_trajectory_from_support_(2) << endl;                                    //28

    // file[4]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
    // <<rd_.torque_desired (0)<<"\t"<<rd_.torque_desired (1)<<"\t"<<rd_.torque_desired (2)<<"\t"<<rd_.torque_desired (3)<<"\t"<<rd_.torque_desired (4)<<"\t"<<rd_.torque_desired (5)<<"\t"<<rd_.torque_desired (6)<<"\t"<<rd_.torque_desired (7)<<"\t"<<rd_.torque_desired (8)<<"\t"<<rd_.torque_desired (9)<<"\t"<<rd_.torque_desired (10)<<"\t"<<rd_.torque_desired (11)<<"\t"<<rd_.torque_desired (12)<<"\t"<<rd_.torque_desired (13)<<"\t"<<rd_.torque_desired (14)<<"\t"<<rd_.torque_desired (15)<<"\t"<<rd_.torque_desired (16)<<"\t"<<rd_.torque_desired (17)<<"\t"<<rd_.torque_desired (18)<<"\t"<<rd_.torque_desired (19)<<"\t"<<rd_.torque_desired (20)<<"\t"<<rd_.torque_desired (21)<<"\t"<<rd_.torque_desired (22)<<"\t"<<rd_.torque_desired (23)<<"\t"<<rd_.torque_desired (24)<<"\t"<<rd_.torque_desired (25)<<"\t"<<rd_.torque_desired (26)<<"\t"<<rd_.torque_desired (27)<<"\t"<<rd_.torque_desired (28)<<"\t"<<rd_.torque_desired (29)<<"\t"<<rd_.torque_desired (30)<<"\t"<<rd_.torque_desired (31)<<"\t"<<rd_.torque_desired (32)<<endl;
    // file[4] << torque_grav_(0) << "\t" << torque_grav_(1) << "\t" << torque_grav_(2) << "\t" << torque_grav_(3) << "\t" << torque_grav_(4) << "\t" << torque_grav_(5) << "\t" << torque_grav_(6) << "\t" << torque_grav_(7) << "\t" << torque_grav_(8) << "\t" << torque_grav_(9) << "\t" << torque_grav_(10) << "\t" << torque_grav_(11) << endl;

    // file[5] << current_time_ - upperbody_command_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"
    //         << desired_q_(0) << "\t" << desired_q_(1) << "\t" << desired_q_(2) << "\t" << desired_q_(3) << "\t" << desired_q_(4) << "\t" << desired_q_(5) << "\t" << desired_q_(6) << "\t" << desired_q_(7) << "\t" << desired_q_(8) << "\t" << desired_q_(9) << "\t" << desired_q_(10) << "\t" << desired_q_(11) << "\t" << desired_q_(12) << "\t" << desired_q_(13) << "\t" << desired_q_(14) << "\t" << desired_q_(15) << "\t" << desired_q_(16) << "\t" << desired_q_(17) << "\t" << desired_q_(18) << "\t" << desired_q_(19) << "\t" << desired_q_(20) << "\t" << desired_q_(21) << "\t" << desired_q_(22) << "\t" << desired_q_(23) << "\t" << desired_q_(24) << "\t" << desired_q_(25) << "\t" << desired_q_(26) << "\t" << desired_q_(27) << "\t" << desired_q_(28) << "\t" << desired_q_(29) << "\t" << desired_q_(30) << "\t" << desired_q_(31) << "\t" << desired_q_(32) << "\t"
    //         << current_q_(0) << "\t" << current_q_(1) << "\t" << current_q_(2) << "\t" << current_q_(3) << "\t" << current_q_(4) << "\t" << current_q_(5) << "\t" << current_q_(6) << "\t" << current_q_(7) << "\t" << current_q_(8) << "\t" << current_q_(9) << "\t" << current_q_(10) << "\t" << current_q_(11) << "\t" << current_q_(12) << "\t" << current_q_(13) << "\t" << current_q_(14) << "\t" << current_q_(15) << "\t" << current_q_(16) << "\t" << current_q_(17) << "\t" << current_q_(18) << "\t" << current_q_(19) << "\t" << current_q_(20) << "\t" << current_q_(21) << "\t" << current_q_(22) << "\t" << current_q_(23) << "\t" << current_q_(24) << "\t" << current_q_(25) << "\t" << current_q_(26) << "\t" << current_q_(27) << "\t" << current_q_(28) << "\t" << current_q_(29) << "\t" << current_q_(30) << "\t" << current_q_(31) << "\t" << current_q_(32) << "\t"
    //         << desired_q_dot_(0) << "\t" << desired_q_dot_(1) << "\t" << desired_q_dot_(2) << "\t" << desired_q_dot_(3) << "\t" << desired_q_dot_(4) << "\t" << desired_q_dot_(5) << "\t" << desired_q_dot_(6) << "\t" << desired_q_dot_(7) << "\t" << desired_q_dot_(8) << "\t" << desired_q_dot_(9) << "\t" << desired_q_dot_(10) << "\t" << desired_q_dot_(11) << "\t" << desired_q_dot_(12) << "\t" << desired_q_dot_(13) << "\t" << desired_q_dot_(14) << "\t" << desired_q_dot_(15) << "\t" << desired_q_dot_(16) << "\t" << desired_q_dot_(17) << "\t" << desired_q_dot_(18) << "\t" << desired_q_dot_(19) << "\t" << desired_q_dot_(20) << "\t" << desired_q_dot_(21) << "\t" << desired_q_dot_(22) << "\t" << desired_q_dot_(23) << "\t" << desired_q_dot_(24) << "\t" << desired_q_dot_(25) << "\t" << desired_q_dot_(26) << "\t" << desired_q_dot_(27) << "\t" << desired_q_dot_(28) << "\t" << desired_q_dot_(29) << "\t" << desired_q_dot_(30) << "\t" << desired_q_dot_(31) << "\t" << desired_q_dot_(32) << "\t"
    //         << current_q_dot_(0) << "\t" << current_q_dot_(1) << "\t" << current_q_dot_(2) << "\t" << current_q_dot_(3) << "\t" << current_q_dot_(4) << "\t" << current_q_dot_(5) << "\t" << current_q_dot_(6) << "\t" << current_q_dot_(7) << "\t" << current_q_dot_(8) << "\t" << current_q_dot_(9) << "\t" << current_q_dot_(10) << "\t" << current_q_dot_(11) << "\t" << current_q_dot_(12) << "\t" << current_q_dot_(13) << "\t" << current_q_dot_(14) << "\t" << current_q_dot_(15) << "\t" << current_q_dot_(16) << "\t" << current_q_dot_(17) << "\t" << current_q_dot_(18) << "\t" << current_q_dot_(19) << "\t" << current_q_dot_(20) << "\t" << current_q_dot_(21) << "\t" << current_q_dot_(22) << "\t" << current_q_dot_(23) << "\t" << current_q_dot_(24) << "\t" << current_q_dot_(25) << "\t" << current_q_dot_(26) << "\t" << current_q_dot_(27) << "\t" << current_q_dot_(28) << "\t" << current_q_dot_(29) << "\t" << current_q_dot_(30) << "\t" << current_q_dot_(31) << "\t" << current_q_dot_(32) << endl;

    // file[5] << rd_.control_time_ << "t";
    // for(int i = 0; i<40; i++)
    // {
    //     file[5] << rd_.q_virtual_(i) << "t";
    // }
    // for(int i = 0; i<39; i++)
    // {
    //     file[5] << rd_.q_dot_virtual_(i) << "t" ;
    // }
    // for(int i = 0; i<33; i++)
    // {
    //     file[5] << rd_.torque_desired(i) << "t" ;
    // }
    // file[5] << endl;
    // for(int i = 0; i<5; i++)
    // {
    //     file[5] << rd_.q_ddot_virtual_(i) << "t" ;
    // }
    // file[5] << rd_.q_ddot_virtual_(5) << endl;

    // file[6]
    // <<current_time_ - upperbody_command_time_ << lhand_transform_current_from_global_.translation()(0)<<"\t"<<lhand_transform_current_from_global_.translation()(1)<<"\t"<<lhand_transform_current_from_global_.translation()(2)<<"\t"<<rhand_transform_current_from_global_.translation()(0)<<"\t"<<rhand_transform_current_from_global_.translation()(1)<<"\t"<<rhand_transform_current_from_global_.translation()(2)<<"\t"
    // <<lhand_rpy_current_from_global_(0)<<"\t"<<lhand_rpy_current_from_global_(1)<<"\t"<<lhand_rpy_current_from_global_(2)<<"\t"<<rhand_rpy_current_from_global_(0)<<"\t"<<rhand_rpy_current_from_global_(1)<<"\t"<<rhand_rpy_current_from_global_(2)<<"\t"
    // <<lhand_transform_pre_desired_from_.translation()(0)<<"\t"<<lhand_transform_pre_desired_from_.translation()(1)<<"\t"<<lhand_transform_pre_desired_from_.translation()(2)<<"\t"<<rhand_transform_pre_desired_from_.translation()(0)<<"\t"<<rhand_transform_pre_desired_from_.translation()(1)<<"\t"<<rhand_transform_pre_desired_from_.translation()(2)<<endl;

    // file[7]
    // <<lelbow_transform_current_from_global_.translation()(0)<<"\t"<<lelbow_transform_current_from_global_.translation()(1)<<"\t"<<lelbow_transform_current_from_global_.translation()(2)<<"\t"<<relbow_transform_current_from_global_.translation()(0)<<"\t"<<relbow_transform_current_from_global_.translation()(1)<<"\t"<<relbow_transform_current_from_global_.translation()(2)<<"\t"
    // <<lelbow_rpy_current_from_global_(0)<<"\t"<<lelbow_rpy_current_from_global_(1)<<"\t"<<lelbow_rpy_current_from_global_(2)<<"\t"<<relbow_rpy_current_from_global_(0)<<"\t"<<relbow_rpy_current_from_global_(1)<<"\t"<<relbow_rpy_current_from_global_(2)<<"\t"
    // <<lelbow_vel_current_from_global_(0)<<"\t"<<lelbow_vel_current_from_global_(1)<<"\t"<<lelbow_vel_current_from_global_(2)<<"\t"<<relbow_vel_current_from_global_(0)<<"\t"<<relbow_vel_current_from_global_(1)<<"\t"<<relbow_vel_current_from_global_(2)<<endl;

    // file[8]
    // <<lshoulder_transform_current_from_global_.translation()(0)<<"\t"<<lshoulder_transform_current_from_global_.translation()(1)<<"\t"<<lshoulder_transform_current_from_global_.translation()(2)<<"\t"<<rshoulder_transform_current_from_global_.translation()(0)<<"\t"<<rshoulder_transform_current_from_global_.translation()(1)<<"\t"<<rshoulder_transform_current_from_global_.translation()(2)<<"\t"
    // <<lshoulder_rpy_current_from_global_(0)<<"\t"<<lshoulder_rpy_current_from_global_(1)<<"\t"<<lshoulder_rpy_current_from_global_(2)<<"\t"<<rshoulder_rpy_current_from_global_(0)<<"\t"<<rshoulder_rpy_current_from_global_(1)<<"\t"<<rshoulder_rpy_current_from_global_(2)<<"\t"
    // <<lshoulder_vel_current_from_global_(0)<<"\t"<<lshoulder_vel_current_from_global_(1)<<"\t"<<lshoulder_vel_current_from_global_(2)<<"\t"<<rshoulder_vel_current_from_global_(0)<<"\t"<<rshoulder_vel_current_from_global_(1)<<"\t"<<rshoulder_vel_current_from_global_(2)<<endl;

    // file[9]
    // <<lacromion_transform_current_from_global_.translation()(0)<<"\t"<<lacromion_transform_current_from_global_.translation()(1)<<"\t"<<lacromion_transform_current_from_global_.translation()(2)<<"\t"<<racromion_transform_current_from_global_.translation()(0)<<"\t"<<racromion_transform_current_from_global_.translation()(1)<<"\t"<<racromion_transform_current_from_global_.translation()(2)<<"\t"
    // <<lacromion_rpy_current_from_global_(0)<<"\t"<<lacromion_rpy_current_from_global_(1)<<"\t"<<lacromion_rpy_current_from_global_(2)<<"\t"<<racromion_rpy_current_from_global_(0)<<"\t"<<racromion_rpy_current_from_global_(1)<<"\t"<<racromion_rpy_current_from_global_(2)<<"\t"
    // <<lacromion_vel_current_from_global_(0)<<"\t"<<lacromion_vel_current_from_global_(1)<<"\t"<<lacromion_vel_current_from_global_(2)<<"\t"<<racromion_vel_current_from_global_(0)<<"\t"<<racromion_vel_current_from_global_(1)<<"\t"<<racromion_vel_current_from_global_(2)<<endl;

    // file[11]
    //     << hmd_lhand_pose_.translation()(0) << "\t" << hmd_lhand_pose_.translation()(1) << "\t" << hmd_lhand_pose_.translation()(2) << "\t" << hmd_rhand_pose_.translation()(0) << "\t" << hmd_rhand_pose_.translation()(1) << "\t" << hmd_rhand_pose_.translation()(2) << "\t"
    //     << master_lhand_pose_raw_.translation()(0) << "\t" << master_lhand_pose_raw_.translation()(1) << "\t" << master_lhand_pose_raw_.translation()(2) << "\t" << master_rhand_pose_raw_.translation()(0) << "\t" << master_rhand_pose_raw_.translation()(1) << "\t" << master_rhand_pose_raw_.translation()(2) << endl;
    // << master_lhand_pose_.translation()(0) << "\t" << master_lhand_pose_.translation()(1) << "\t" << master_lhand_pose_.translation()(2) << "\t" << master_rhand_pose_.translation()(0) << "\t" << master_rhand_pose_.translation()(1) << "\t" << master_rhand_pose_.translation()(2) << endl;
    // << master_lhand_rqy_(0) << "\t" << master_lhand_rqy_(1) << "\t" << master_lhand_rqy_(2) << "\t" << master_rhand_rqy_(0) << "\t" << master_rhand_rqy_(1) << "\t" << master_rhand_rqy_(2) << "\t"
    // << hmd_lupperarm_pose_.translation()(0) << "\t" << hmd_lupperarm_pose_.translation()(1) << "\t" << hmd_lupperarm_pose_.translation()(2) << "\t" << hmd_rupperarm_pose_.translation()(0) << "\t" << hmd_rupperarm_pose_.translation()(1) << "\t" << hmd_rupperarm_pose_.translation()(2) << "\t"
    // << master_lelbow_pose_raw_.translation()(0) << "\t" << master_lelbow_pose_raw_.translation()(1) << "\t" << master_lelbow_pose_raw_.translation()(2) << "\t" << master_relbow_pose_raw_.translation()(0) << "\t" << master_relbow_pose_raw_.translation()(1) << "\t" << master_relbow_pose_raw_.translation()(2) << "\t"
    // << master_lelbow_pose_.translation()(0) << "\t" << master_lelbow_pose_.translation()(1) << "\t" << master_lelbow_pose_.translation()(2) << "\t" << master_relbow_pose_.translation()(0) << "\t" << master_relbow_pose_.translation()(1) << "\t" << master_relbow_pose_.translation()(2) << "\t"
    // << master_lelbow_rqy_(0) << "\t" << master_lelbow_rqy_(1) << "\t" << master_lelbow_rqy_(2) << "\t" << master_relbow_rqy_(0) << "\t" << master_relbow_rqy_(1) << "\t" << master_relbow_rqy_(2) << "\t"
    // << hmd_lshoulder_pose_.translation()(0) << "\t" << hmd_lshoulder_pose_.translation()(1) << "\t" << hmd_lshoulder_pose_.translation()(2) << "\t" << hmd_rshoulder_pose_.translation()(0) << "\t" << hmd_rshoulder_pose_.translation()(1) << "\t" << hmd_rshoulder_pose_.translation()(2) << "\t"
    // << master_lshoulder_pose_raw_.translation()(0) << "\t" << master_lshoulder_pose_raw_.translation()(1) << "\t" << master_lshoulder_pose_raw_.translation()(2) << "\t" << master_rshoulder_pose_raw_.translation()(0) << "\t" << master_rshoulder_pose_raw_.translation()(1) << "\t" << master_rshoulder_pose_raw_.translation()(2) << "\t"
    // << master_lshoulder_pose_.translation()(0) << "\t" << master_lshoulder_pose_.translation()(1) << "\t" << master_lshoulder_pose_.translation()(2) << "\t" << master_rshoulder_pose_.translation()(0) << "\t" << master_rshoulder_pose_.translation()(1) << "\t" << master_rshoulder_pose_.translation()(2) << "\t"
    // << master_lshoulder_rqy_(0) << "\t" << master_lshoulder_rqy_(1) << "\t" << master_lshoulder_rqy_(2) << "\t" << master_rshoulder_rqy_(0) << "\t" << master_rshoulder_rqy_(1) << "\t" << master_rshoulder_rqy_(2) << "\t"
    // << hmd_head_pose_.translation()(0) << "\t" << hmd_head_pose_.translation()(1) << "\t" << hmd_head_pose_.translation()(2) << "\t"
    // << master_head_pose_raw_.translation()(0) << "\t" << master_head_pose_raw_.translation()(1) << "\t" << master_head_pose_raw_.translation()(2) << "\t"
    // << master_head_pose_.translation()(0) << "\t" << master_head_pose_.translation()(1) << "\t" << master_head_pose_.translation()(2) << "\t"
    // << master_head_rqy_(0) << "\t" << master_head_rqy_(1) << "\t" << master_head_rqy_(2) << "\t"
    // << hmd_pelv_pose_.translation()(0) << "\t" << hmd_pelv_pose_.translation()(1) << "\t" << hmd_pelv_pose_.translation()(2) << endl;

    // file[12]
    //     << lhand_pos_error_(0) << "\t" << lhand_pos_error_(1) << "\t" << lhand_pos_error_(2) << "\t" << rhand_pos_error_(0) << "\t" << rhand_pos_error_(1) << "\t" << rhand_pos_error_(2) << "\t"
    //     << lhand_ori_error_(0) << "\t" << lhand_ori_error_(1) << "\t" << lhand_ori_error_(2) << "\t" << rhand_ori_error_(0) << "\t" << rhand_ori_error_(1) << "\t" << rhand_ori_error_(2) << "\t"
    //     << lelbow_ori_error_(0) << "\t" << lelbow_ori_error_(1) << "\t" << lelbow_ori_error_(2) << "\t" << relbow_ori_error_(0) << "\t" << relbow_ori_error_(1) << "\t" << relbow_ori_error_(2) << "\t"
    //     << lshoulder_ori_error_(0) << "\t" << lshoulder_ori_error_(1) << "\t" << lshoulder_ori_error_(2) << "\t" << rshoulder_ori_error_(0) << "\t" << rshoulder_ori_error_(1) << "\t" << rshoulder_ori_error_(2) << "\t"
    //     << lhand_vel_error_(0) << "\t" << lhand_vel_error_(1) << "\t" << lhand_vel_error_(2) << "\t" << lhand_vel_error_(3) << "\t" << lhand_vel_error_(4) << "\t" << lhand_vel_error_(5) << "\t"
    //     << rhand_vel_error_(0) << "\t" << rhand_vel_error_(1) << "\t" << rhand_vel_error_(2) << "\t" << rhand_vel_error_(3) << "\t" << rhand_vel_error_(4) << "\t" << rhand_vel_error_(5) << "\t"
    //     << lelbow_vel_error_(0) << "\t" << lelbow_vel_error_(1) << "\t" << lelbow_vel_error_(2) << "\t" << relbow_vel_error_(0) << "\t" << relbow_vel_error_(1) << "\t" << relbow_vel_error_(2) << "\t"
    //     << lacromion_vel_error_(0) << "\t" << lacromion_vel_error_(1) << "\t" << lacromion_vel_error_(2) << "\t" << racromion_vel_error_(0) << "\t" << racromion_vel_error_(1) << "\t" << racromion_vel_error_(2) << endl;

    // file[13]
    // <<hmd_head_vel_(0)<<"\t"<<hmd_head_vel_(1)<<"\t"<<hmd_head_vel_(2)<<"\t"<<hmd_head_vel_(3)<<"\t"<<hmd_head_vel_(4)<<"\t"<<hmd_head_vel_(5)<<"\t"
    // <<hmd_lshoulder_vel_(0)<<"\t"<<hmd_lshoulder_vel_(1)<<"\t"<<hmd_lshoulder_vel_(2)<<"\t"<<hmd_lshoulder_vel_(3)<<"\t"<<hmd_lshoulder_vel_(4)<<"\t"<<hmd_lshoulder_vel_(5)<<"\t"
    // <<hmd_lupperarm_vel_(0)<<"\t"<<hmd_lupperarm_vel_(1)<<"\t"<<hmd_lupperarm_vel_(2)<<"\t"<<hmd_lupperarm_vel_(3)<<"\t"<<hmd_lupperarm_vel_(4)<<"\t"<<hmd_lupperarm_vel_(5)<<"\t"
    // <<hmd_lhand_vel_(0)<<"\t"<<hmd_lhand_vel_(1)<<"\t"<<hmd_lhand_vel_(2)<<"\t"<<hmd_lhand_vel_(3)<<"\t"<<hmd_lhand_vel_(4)<<"\t"<<hmd_lhand_vel_(5)<<"\t"
    // <<hmd_rshoulder_vel_(0)<<"\t"<<hmd_rshoulder_vel_(1)<<"\t"<<hmd_rshoulder_vel_(2)<<"\t"<<hmd_rshoulder_vel_(3)<<"\t"<<hmd_rshoulder_vel_(4)<<"\t"<<hmd_rshoulder_vel_(5)<<"\t"
    // <<hmd_rupperarm_vel_(0)<<"\t"<<hmd_rupperarm_vel_(1)<<"\t"<<hmd_rupperarm_vel_(2)<<"\t"<<hmd_rupperarm_vel_(3)<<"\t"<<hmd_rupperarm_vel_(4)<<"\t"<<hmd_rupperarm_vel_(5)<<"\t"
    // <<hmd_rhand_vel_(0)<<"\t"<<hmd_rhand_vel_(1)<<"\t"<<hmd_rhand_vel_(2)<<"\t"<<hmd_rhand_vel_(3)<<"\t"<<hmd_rhand_vel_(4)<<"\t"<<hmd_rhand_vel_(5)<<"\t"
    // <<hmd_chest_vel_(0)<<"\t"<<hmd_chest_vel_(1)<<"\t"<<hmd_chest_vel_(2)<<"\t"<<hmd_chest_vel_(3)<<"\t"<<hmd_chest_vel_(4)<<"\t"<<hmd_chest_vel_(5)<<"\t"
    // <<hmd_pelv_vel_(0)<<"\t"<<hmd_pelv_vel_(1)<<"\t"<<hmd_pelv_vel_(2)<<"\t"<<hmd_pelv_vel_(3)<<"\t"<<hmd_pelv_vel_(4)<<"\t"<<hmd_pelv_vel_(5)<<endl;

    // }
}

//////////////////////////////MJ's Functions////////////////////
void AvatarController::PedalCommandCallback(const tocabi_msgs::WalkingCommandConstPtr &msg)
{
    if (joy_input_enable_ == true)
    {
        joystick_input(0) = DyrosMath::minmax_cut(2 * (msg->step_length_x), 0.0, 2.0) - 1.0; //FW
        joystick_input(2) = DyrosMath::minmax_cut(2 * (msg->theta) - DyrosMath::sign(msg->theta), -0.5 + 0.5 * DyrosMath::sign(msg->theta), 0.5 + 0.5 * DyrosMath::sign(msg->theta));
        // joystick_input(2) = msg->theta;
        joystick_input(3) = DyrosMath::minmax_cut(2 * (msg->z), 0.0, 2.0) - 1.0; //BW
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
        //calculateFootStepTotal();
        calculateFootStepTotal_MJ();

        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        //pelv_float_init_.translation()(0) += 0.11;

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

        if (foot_step_(0, 6) == 0) //right foot support
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

        if (foot_step_(0, 6) == 1) //left suppport foot
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
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

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

        if (foot_step_(current_step_num_, 6) == 0) //right foot support
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
    pelv_rpy_current_mj_.setZero();
    pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

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
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot;
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

    //cout << "L : " << lfoot_float_current_.linear() << "\n" <<  "R : " << rfoot_float_current_.linear() << endl;
    com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);

    SC_err_compen(com_support_current_(0), com_support_current_(1));
    
    cp_measured_(0) = com_support_current_(0) + com_float_current_dot_LPF(0) / wn;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;
 
    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    if (walking_tick_mj == 0)
    {
        l_ft_LPF = l_ft_;
        r_ft_LPF = r_ft_;
    }

    l_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_;
    r_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_;
 
    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);

    zmp_measured_mj_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP X
    zmp_measured_mj_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y
    // MJ_graph << cp_measured_(0) << "," << cp_measured_(1) << endl;
    wn = sqrt(GRAVITY / zc_mj_);

    // real robot experiment
    opto_ft_ = opto_ft_raw_; 
    // cout << opto_ft_(0) << "," << opto_ft_(1) << "," << opto_ft_(2) << endl; 
    MJ_opto <<  opto_ft_(0) << "," << opto_ft_(1) << "," << opto_ft_(2) << "," << opto_ft_(3) << "," << opto_ft_(4) << "," << opto_ft_(5) << endl; 
    
    if (walking_tick_mj == 0)
    {
        zmp_measured_LPF_.setZero();
    } 
    zmp_measured_LPF_ = (2 * M_PI * 2.0 * del_t) / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_LPF_;
    // MJ_graph2 << ZMP_X_REF_ << "," << ZMP_Y_REF_ << "," << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << "," << zmp_measured_LPF_(0) << "," << zmp_measured_LPF_(1) << endl;
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
        middle_total_step_number = 20; //total foot step number
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
    modified_del_zmp_.setZero(number_of_foot_step, 2);
    m_del_zmp_x.setZero(number_of_foot_step, 2); 
    m_del_zmp_y.setZero(number_of_foot_step, 2);
    
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
    
    //
    bool Forward = false, Side = false;

    if(Forward)
    {
        number_of_foot_step = 11;
        foot_step_.resize(number_of_foot_step, 7);
        foot_step_.setZero();
        foot_step_support_frame_.resize(number_of_foot_step, 7);
        foot_step_support_frame_.setZero();
        modified_del_zmp_.setZero(number_of_foot_step, 2);
        m_del_zmp_x.setZero(number_of_foot_step, 2); 
        m_del_zmp_y.setZero(number_of_foot_step, 2);

        // Forward
        foot_step_(0, 0) = 0.0; foot_step_(0, 1) = -0.1125; foot_step_(0, 6) = 1.0; 
        foot_step_(1, 0) = 0.0; foot_step_(1, 1) =  0.1225; foot_step_(1, 6) = 0.0;
        foot_step_(2, 0) = 0.05; foot_step_(2, 1) = -0.1225; foot_step_(2, 6) = 1.0; 
        foot_step_(3, 0) = 0.15; foot_step_(3, 1) =  0.1225; foot_step_(3, 6) = 0.0;
        foot_step_(4, 0) = 0.30; foot_step_(4, 1) = -0.1225; foot_step_(4, 6) = 1.0; 
        foot_step_(5, 0) = 0.15; foot_step_(5, 1) =  0.1225; foot_step_(5, 6) = 0.0;
        foot_step_(6, 0) = 0.30; foot_step_(6, 1) = -0.1225; foot_step_(6, 6) = 1.0; 
        foot_step_(7, 0) = 0.15; foot_step_(7, 1) =  0.1225; foot_step_(7, 6) = 0.0;
        foot_step_(8, 0) = 0.05; foot_step_(8, 1) = -0.1225; foot_step_(8, 6) = 1.0;
        foot_step_(9, 0) = 0.0; foot_step_(9, 1) =  0.1225; foot_step_(9, 6) = 0.0; 
        foot_step_(10, 0) = 0.0; foot_step_(10, 1) = -0.1225; foot_step_(10, 6) = 1.0; 
    }
    if(Side)
    {
        number_of_foot_step = 10;
        foot_step_.resize(number_of_foot_step, 7);
        foot_step_.setZero();
        foot_step_support_frame_.resize(number_of_foot_step, 7);
        foot_step_support_frame_.setZero();
        modified_del_zmp_.setZero(number_of_foot_step, 2);
        m_del_zmp_x.setZero(number_of_foot_step, 2); 
        m_del_zmp_y.setZero(number_of_foot_step, 2);

        // Side
        foot_step_(0, 0) = 0.0; foot_step_(0, 1) = -0.1125; foot_step_(0, 6) = 1.0; 
        foot_step_(1, 0) = 0.0; foot_step_(1, 1) =  0.1225; foot_step_(1, 6) = 0.0;
        foot_step_(2, 0) = 0.0; foot_step_(2, 1) = -0.1225; foot_step_(2, 6) = 1.0;
        foot_step_(3, 0) = 0.0; foot_step_(3, 1) =  0.0; foot_step_(3, 6) = 0.0;
        foot_step_(4, 0) = 0.0; foot_step_(4, 1) = -0.1225*2; foot_step_(4, 6) = 1.0;
        foot_step_(5, 0) = 0.0; foot_step_(5, 1) = -0.1225; foot_step_(5, 6) = 0.0;
        foot_step_(6, 0) = 0.0; foot_step_(6, 1) = -0.1225*3; foot_step_(6, 6) = 1.0; 
        foot_step_(7, 0) = 0.0; foot_step_(7, 1) = -0.1225*2; foot_step_(7, 6) = 0.0; 
        foot_step_(8, 0) = 0.0; foot_step_(8, 1) = -0.1225*4; foot_step_(8, 6) = 1.0;
        foot_step_(9, 0) = 0.0; foot_step_(9, 1) = -0.1225*2; foot_step_(9, 6) = 0.0;
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
    // //simulation gains
    // Kp(0) = 1800.0;
    // Kd(0) = 70.0; // Left Hip yaw
    // Kp(1) = 2100.0;
    // Kd(1) = 90.0; // Left Hip roll
    // Kp(2) = 2100.0;
    // Kd(2) = 90.0; // Left Hip pitch
    // Kp(3) = 2100.0;
    // Kd(3) = 90.0; // Left Knee pitch
    // Kp(4) = 2100.0;
    // Kd(4) = 90.0; // Left Ankle pitch
    // Kp(5) = 2100.0;
    // Kd(5) = 90.0; // Left Ankle roll

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

    //real robot experiment
    Kp(0) = 2000.0;
    Kd(0) = 20.0; // Left Hip yaw
    Kp(1) = 5000.0;
    Kd(1) = 55.0; // Left Hip roll //55
    Kp(2) = 4000.0;
    Kd(2) = 45.0; // Left Hip pitch
    Kp(3) = 3700.0;
    Kd(3) = 40.0; // Left Knee pitch
    Kp(4) = 4000.0; // 5000
    Kd(4) = 65.0; // Left Ankle pitch /5000 / 30  //55
    Kp(5) = 4000.0; // 5000
    Kd(5) = 65.0; // Left Ankle roll /5000 / 30 //55

    Kp(6) = 2000.0;
    Kd(6) = 20.0; // Right Hip yaw
    Kp(7) = 5000.0;
    Kd(7) = 55.0; // Right Hip roll  //55
    Kp(8) = 4000.0;
    Kd(8) = 45.0; // Right Hip pitch
    Kp(9) = 3700.0;
    Kd(9) = 40.0; // Right Knee pitch
    Kp(10) = 4000.0; // 5000
    Kd(10) = 65.0; // Right Ankle pitch //55
    Kp(11) = 4000.0; // 5000
    Kd(11) = 65.0; // Right Ankle roll //55

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

    // lfoot_zmp_offset_ = -0.02; // 1.1 초
    // rfoot_zmp_offset_ = 0.02;

    lfoot_zmp_offset_ = -0.015; // MJ CPMPC final offset
    rfoot_zmp_offset_ = 0.015;

    foot_step_support_frame_offset_ = foot_step_support_frame_;

    supportfoot_support_init_offset_ = supportfoot_support_init_;

    if (foot_step_(0, 6) == 0) //right support foot
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
        //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
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
    unsigned int planning_step_number = 5;
    unsigned int norm_size = 0;

    if (current_step_num_ >= total_step_num_ - planning_step_number)
    {
        norm_size = t_total_const_ * (total_step_num_ - current_step_num_) + 4.0 * hz_;        
    }
    else
    {
        norm_size = t_total_const_ * planning_step_number + 1.0 * hz_;
        // norm_size = (t_last_ - t_start_ + 1) * (planning_step_number) + 1.0 * hz_;
    }         
    // t_total_ * planning step_num인데 t_total_이 현재스텝에서 0.7이면 norm size가 3.5s가 되어서 0.7 + 0.9 *(planning_step_num -1) size보다 작아서 터졌었음.
//    cout << total_step_num_ << "," << planning_step_number << "," << current_step_num_ << endl;
    if (current_step_num_ == 0)
    {
        norm_size = norm_size + t_temp_ + 1;
    }        
    
    addZmpOffset(); 
    zmpGenerator(norm_size, planning_step_number);

    // static int aa = 0;
    // if(walking_tick_mj >= 4.3*2000 && aa == 0)
    // {
    //     aa = 1;

    //     for(int i = 0; i < 5000; i ++)
    //     {        
    //         MJ_graph << ref_zmp_mj_wo_offset_(i, 1) << endl;            
    //     }            
    // }

    ref_zmp_wo_offset_mpc_.resize(norm_size, 2);
    ref_zmp_mpc_.resize(norm_size, 2);
}

void AvatarController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
    ref_zmp_mj_.resize(norm_size, 2);
    ref_zmp_mj_.setZero();
    ref_zmp_mj_wo_offset_.resize(norm_size, 2);    
    ref_zmp_mj_wo_offset_.setZero();

    Eigen::VectorXd temp_px;
    Eigen::VectorXd temp_py;
    Eigen::VectorXd temp_px_wo_offset;
    Eigen::VectorXd temp_py_wo_offset;

    unsigned int index = 0;
    double t_total_zmp = t_total_; 
    if (current_step_num_ == 0)  
    {
        for (int i = 0; i <= t_temp_; i++) 
        {
            if (i < 1.0 * hz_)
            {
                ref_zmp_mj_(i, 0) = com_support_init_(0);
                ref_zmp_mj_(i, 1) = com_support_init_(1);

                ref_zmp_mj_wo_offset_(i, 0) = com_support_init_(0);
                ref_zmp_mj_wo_offset_(i, 1) = com_support_init_(1);
            }
            else if (i < 2.0 * hz_)
            {
                double del_x = i - 1.0 * hz_;
                ref_zmp_mj_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (1.0 * hz_);
                ref_zmp_mj_(i, 1) = com_support_init_(1);

                ref_zmp_mj_wo_offset_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (1.0 * hz_);
                ref_zmp_mj_wo_offset_(i, 1) = com_support_init_(1);
            }
            else
            {
                ref_zmp_mj_(i, 0) = 0.0;
                ref_zmp_mj_(i, 1) = com_support_init_(1);

                ref_zmp_mj_wo_offset_(i, 0) = 0.0;
                ref_zmp_mj_wo_offset_(i, 1) = com_support_init_(1);
            }
            index++;
        }
    }
    /////////////////////////////////////////////////////////////////////.
    
    if(current_step_num_ >= total_step_num_ - planning_step_num)
    {   
        for(unsigned int i = current_step_num_; i < total_step_num_; i++)
        {   
            if(current_step_num_ > 1)
            {
                if(i == current_step_num_) // current_step_number => (current step_num_) ~ (current_step_num_ + planning_step_num)
                {            
                    first_current_step_flag_ = 0; 
                    first_current_step_number_ = i; // save the value of the first current_step_number
                    
                    if(walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_)
                    {
                        t_total_zmp = t_total_const_;
                    }
                    else if(walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_)
                    {
                        t_total_zmp = t_total_;
                    }
                }
                else
                {
                    first_current_step_flag_ = 1;
                    t_total_zmp = t_total_const_;
                }
            }
            else
            {
                first_current_step_flag_ = 0;
                t_total_zmp = t_total_;
            }
            
            // onestepZmp(i, temp_px, temp_py);
            onestepZmp_wo_offset(i, t_total_zmp, temp_px, temp_py, temp_px_wo_offset, temp_py_wo_offset);
            for(unsigned int j = 0; j < t_total_zmp; j++)
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);

                ref_zmp_mj_wo_offset_(index + j, 0) = temp_px_wo_offset(j);
                ref_zmp_mj_wo_offset_(index + j, 1) = temp_py_wo_offset(j);
            }
            index = index + t_total_zmp;
        }

        for(unsigned int j = 0; j < 3.0 * hz_; j++)
        {
            ref_zmp_mj_(index + j, 0) = ref_zmp_mj_(index - 1, 0);
            ref_zmp_mj_(index + j, 1) = ref_zmp_mj_(index - 1, 1);

            ref_zmp_mj_wo_offset_(index + j, 0) = ref_zmp_mj_wo_offset_(index - 1, 0);
            ref_zmp_mj_wo_offset_(index + j, 1) = ref_zmp_mj_wo_offset_(index - 1, 1);
        }
        index = index + 3.0 * hz_; // Norm size must be larger than this addtional zmp size.
    }
    else // reference ZMP during walking
    {       
        for (unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)
        {   
            if(current_step_num_ > 1)
            {
                if(i == current_step_num_) // current_step_number => (current step_num_) ~ (current_step_num_ + planning_step_num)
                {            
                    first_current_step_flag_ = 0; 
                    first_current_step_number_ = current_step_num_; // save the value of the first current_step_number
                    // t_total_zmp = t_total_;
                    if(walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_)
                    {
                        t_total_zmp = t_total_const_;
                    }
                    else if(walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_)
                    {
                        t_total_zmp = t_total_;
                    }
                }
                else
                {
                    first_current_step_flag_ = 1;
                    t_total_zmp = t_total_const_;
                }
            }
            else
            {
                first_current_step_flag_ = 0;
                t_total_zmp = t_total_;
            }
                     
            // onestepZmp(i, temp_px, temp_py);             
            onestepZmp_wo_offset(i, t_total_zmp, temp_px, temp_py, temp_px_wo_offset, temp_py_wo_offset); // temp px, py에 1 step의 ZMP를 planning step num 번 담는다.
             
            for (unsigned int j = 0; j < t_total_zmp; j++)  
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);

                ref_zmp_mj_wo_offset_(index + j, 0) = temp_px_wo_offset(j);
                ref_zmp_mj_wo_offset_(index + j, 1) = temp_py_wo_offset(j);
            }
            index = index + t_total_zmp;                                                          
        }
    }

}

// void AvatarController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py)
// {
//     temp_px.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
//     temp_py.resize(t_total_);
//     temp_px.setZero();
//     temp_py.setZero();
    
//     double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0, Kx2 = 0, Ky2 = 0;
//     if (current_step_number == 0)
//     {
//         Kx = 0;
//         Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
//         Kx2 = foot_step_support_frame_offset_(current_step_number, 0) / 2 - supportfoot_support_init_offset_(0);
//         Ky2 = foot_step_support_frame_offset_(current_step_number, 1) / 2 - supportfoot_support_init_offset_(1);

//         for (int i = 0; i < t_total_; i++)
//         {
//             if (i >= 0 && i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
//             {
//                 temp_px(i) = Kx / (t_double1_ + t_rest_init_) * (i + 1);
//                 temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i + 1);
//             }
//             else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
//             {
//                 temp_px(i) = 0;
//                 temp_py(i) = supportfoot_support_init_offset_(1);
//             }
//             else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick
//             {
//                 temp_px(i) = 0 + Kx2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
//                 temp_py(i) = supportfoot_support_init_offset_(1) + Ky2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
//             }
//         }
//     }
//     else if (current_step_number == 1)
//     { 
//         Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2;
//         Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2;
//         Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
//         Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

//         for (int i = 0; i < t_total_; i++)
//         {
//             if (i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
//             {
//                 temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
//                 temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
//             }
//             else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
//             {
//                 temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
//                 temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
//             }
//             else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick
//             {
//                 temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
//                 temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
//             }
//         }
//     }
//     else
//     { 
//         Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - ((foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2);
//         Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - ((foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2);
//         Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
//         Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

//         for (int i = 0; i < t_total_; i++)
//         {   
//             int current_stepping_flag = 0;
//             int prev_stepping_flag = 0;
//             if(foot_step_(current_step_num_, 6) == 1) // left foot support
//             {
//                 prev_stepping_flag = 0; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
//                 current_stepping_flag = 1; // Current first DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
//             }
//             else // right foot support
//             {
//                 prev_stepping_flag = 1; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
//                 current_stepping_flag = 0; // First DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
//             }

//             if (i < t_rest_init_ + t_double1_)  
//             {
//                 temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 2, 0) - m_del_zmp_x(current_step_number -1,current_stepping_flag) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 + (Kx + m_del_zmp_x(current_step_number -1, current_stepping_flag)/2) / (t_rest_init_ + t_double1_) * (i + 1);
//                 temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 2, 1) - m_del_zmp_y(current_step_number -1,current_stepping_flag) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 + (Ky + m_del_zmp_y(current_step_number -1, current_stepping_flag)/2) / (t_rest_init_ + t_double1_) * (i + 1);
//             }
//             else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_)  
//             {   
//                 temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
//                 temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
//             }
//             else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_)  
//             {
//                 temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + (Kx2 + m_del_zmp_x(current_step_number, prev_stepping_flag)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
//                 temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + (Ky2 + m_del_zmp_y(current_step_number, prev_stepping_flag)/2) / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
//             }
//         }
//     }
// }

void AvatarController::onestepZmp_wo_offset(unsigned int current_step_number, double t_total_zmp, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py, Eigen::VectorXd &temp_px_wo_offset, Eigen::VectorXd &temp_py_wo_offset)
{
    temp_px.resize(t_total_zmp);  
    temp_py.resize(t_total_zmp);
    temp_px_wo_offset.resize(t_total_zmp); 
    temp_py_wo_offset.resize(t_total_zmp);
    temp_px.setZero();
    temp_py.setZero();    
    temp_px_wo_offset.setZero();
    temp_py_wo_offset.setZero();
    
    
    double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0, Kx2 = 0, Ky2 = 0;
    double Kx_wo_offset = 0, Ky_wo_offset = 0, Kx2_wo_offset = 0, Ky2_wo_offset = 0;
    
    if (current_step_number == 0)
    {
        Kx = 0;
        Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
        Kx2 = foot_step_support_frame_offset_(current_step_number, 0) / 2 - supportfoot_support_init_offset_(0);
        Ky2 = foot_step_support_frame_offset_(current_step_number, 1) / 2 - supportfoot_support_init_offset_(1);
        Kx_wo_offset = 0;
        Ky_wo_offset = supportfoot_support_init_(1) - com_support_init_(1);
        Kx2_wo_offset = foot_step_support_frame_(current_step_number, 0) / 2 - supportfoot_support_init_(0);
        Ky2_wo_offset = foot_step_support_frame_(current_step_number, 1) / 2 - supportfoot_support_init_(1);

        for (int i = 0; i < t_total_zmp; i++)
        {
            if (i >= 0 && i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = Kx / (t_double1_ + t_rest_init_) * (i + 1);
                temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i + 1);
                
                temp_px_wo_offset(i) = Kx_wo_offset / (t_double1_ + t_rest_init_) * (i + 1);
                temp_py_wo_offset(i) = com_support_init_(1) + Ky_wo_offset / (t_double1_ + t_rest_init_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_zmp - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = 0;
                temp_py(i) = supportfoot_support_init_offset_(1);
                
                temp_px_wo_offset(i) = 0;
                temp_py_wo_offset(i) = supportfoot_support_init_(1);
            }
            else if (i >= t_total_zmp - t_rest_last_ - t_double2_ && i < t_total_zmp) //1.05 ~ 1.15초 , 210 ~ 230 tick
            {
                temp_px(i) = 0 + Kx2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
                temp_py(i) = supportfoot_support_init_offset_(1) + Ky2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
                
                temp_px_wo_offset(i) = 0 + Kx2_wo_offset / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
                temp_py_wo_offset(i) = supportfoot_support_init_(1) + Ky2_wo_offset / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
            }
        }
    }
    else if (current_step_number == 1)
    { 
        Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2;
        Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2;
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

        Kx_wo_offset = foot_step_support_frame_(current_step_number - 1, 0) - (foot_step_support_frame_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2;
        Ky_wo_offset = foot_step_support_frame_(current_step_number - 1, 1) - (foot_step_support_frame_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2;
        Kx2_wo_offset = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_(current_step_number - 1, 0);
        Ky2_wo_offset = (foot_step_support_frame_(current_step_number, 1) + foot_step_support_frame_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_(current_step_number - 1, 1);

        for (int i = 0; i < t_total_zmp; i++)
        {
            if (i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
                
                temp_px_wo_offset(i) = (foot_step_support_frame_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2 + Kx_wo_offset / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py_wo_offset(i) = (foot_step_support_frame_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2 + Ky_wo_offset / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_zmp - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);

                temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0);
                temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1);
            }
            else if (i >= t_total_zmp - t_rest_last_ - t_double2_ && i < t_total_zmp) //1.05 ~ 1.2초 , 210 ~ 240 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));

                temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0) + Kx2_wo_offset / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
                temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1) + Ky2_wo_offset / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
            }
        }
    }
    else
    {   
        int current_stepping_flag = 0;
        int prev_stepping_flag = 0;

        if(foot_step_(current_step_num_, 6) == 1) // left foot support
        {
            prev_stepping_flag = 0; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
            current_stepping_flag = 1; // Current first DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
        }
        else // right foot support
        {
            prev_stepping_flag = 1; // Previous second DSP (착지 이후 + Step change 이전)에서 다음 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그
            current_stepping_flag = 0; // First DSP (착지 이후 + Step change 이후)에서 이전 지지발과 현재 지지발의 ZMP를 이어주기 위한 플래그 
        }
        
        if(first_current_step_flag_ == 0)
        {   
            Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - ((foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2);
            Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - ((foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2);
            Kx2 = (m_del_zmp_x(current_step_number, prev_stepping_flag) + foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
            Ky2 = (m_del_zmp_y(current_step_number, prev_stepping_flag) + foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

            Kx_wo_offset = foot_step_support_frame_(current_step_number - 1, 0) - ((foot_step_support_frame_(current_step_number - 2, 0) + foot_step_support_frame_(current_step_number - 1, 0)) / 2);
            Ky_wo_offset = foot_step_support_frame_(current_step_number - 1, 1) - ((foot_step_support_frame_(current_step_number - 2, 1) + foot_step_support_frame_(current_step_number - 1, 1)) / 2);
            Kx2_wo_offset = (m_del_zmp_x(current_step_number, prev_stepping_flag) + foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_(current_step_number - 1, 0);
            Ky2_wo_offset = (m_del_zmp_y(current_step_number, prev_stepping_flag) + foot_step_support_frame_(current_step_number, 1) + foot_step_support_frame_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_(current_step_number - 1, 1);
            
            for (int i = 0; i < t_total_zmp; i++)
            {                    
                if (i < t_rest_init_ + t_double1_) // first flag 
                {   // when the supporting foot is change, previously adjusted footstep must be reflected in the current DSP.              
                    temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 2, 0) - m_del_zmp_x(current_step_number - 1, current_stepping_flag) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 + (Kx + m_del_zmp_x(current_step_number -1, current_stepping_flag)/2) / (t_rest_init_ + t_double1_) * (i + 1);
                    temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 2, 1) - m_del_zmp_y(current_step_number - 1, current_stepping_flag) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 + (Ky + m_del_zmp_y(current_step_number -1, current_stepping_flag)/2) / (t_rest_init_ + t_double1_) * (i + 1);

                    temp_px_wo_offset(i) = (foot_step_support_frame_(current_step_number - 2, 0) - m_del_zmp_x(current_step_number - 1, current_stepping_flag) + foot_step_support_frame_(current_step_number - 1, 0)) / 2 + (Kx_wo_offset + m_del_zmp_x(current_step_number -1, current_stepping_flag)/2) / (t_rest_init_ + t_double1_) * (i + 1);
                    temp_py_wo_offset(i) = (foot_step_support_frame_(current_step_number - 2, 1) - m_del_zmp_y(current_step_number - 1, current_stepping_flag) + foot_step_support_frame_(current_step_number - 1, 1)) / 2 + (Ky_wo_offset + m_del_zmp_y(current_step_number -1, current_stepping_flag)/2) / (t_rest_init_ + t_double1_) * (i + 1);
                }
                else if (i >= t_rest_init_ + t_double1_ && i < t_total_zmp - t_rest_last_ - t_double2_)  
                {   
                    temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                    temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
                    
                    temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0); 
                    temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1);
                }
                else if (i >= t_total_zmp - t_rest_last_ - t_double2_ && i < t_total_zmp)  
                {
                    temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2/(t_double2_ + t_rest_last_) * (i+1 -(t_total_zmp - t_rest_last_ - t_double2_));
                    temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2/(t_double2_ + t_rest_last_) * (i+1 -(t_total_zmp - t_rest_last_ - t_double2_));

                    temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0) + Kx2_wo_offset/(t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
                    temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1) + Ky2_wo_offset/(t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_));
                }
                             
            }
        }
        else if(first_current_step_flag_ == 1)
        {
            Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - ((foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2);
            Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - ((foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2);
            Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
            Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

            Kx_wo_offset = foot_step_support_frame_(current_step_number - 1, 0) - ((foot_step_support_frame_(current_step_number - 2, 0) + foot_step_support_frame_(current_step_number - 1, 0)) / 2);
            Ky_wo_offset = foot_step_support_frame_(current_step_number - 1, 1) - ((foot_step_support_frame_(current_step_number - 2, 1) + foot_step_support_frame_(current_step_number - 1, 1)) / 2);
            Kx2_wo_offset = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_(current_step_number - 1, 0);
            Ky2_wo_offset = (foot_step_support_frame_(current_step_number, 1) + foot_step_support_frame_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_(current_step_number - 1, 1);
            
            for (int i = 0; i < t_total_zmp; i++)
            {   
                if (i < t_rest_init_ + t_double1_)  
                {
                    if(first_current_step_number_ + 1 == current_step_number)
                    {
                        temp_px(i) = (m_del_zmp_x(first_current_step_number_, prev_stepping_flag) + foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0))/2 + (Kx + m_del_zmp_x(first_current_step_number_, prev_stepping_flag)/2) / (t_rest_init_ + t_double1_)*(i+1);
                        temp_py(i) = (m_del_zmp_y(first_current_step_number_, prev_stepping_flag) + foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1))/2 + (Ky + m_del_zmp_y(first_current_step_number_, prev_stepping_flag)/2) / (t_rest_init_ + t_double1_)*(i+1);

                        temp_px_wo_offset(i) = (m_del_zmp_x(first_current_step_number_, prev_stepping_flag) + foot_step_support_frame_(current_step_number - 2, 0) + foot_step_support_frame_(current_step_number - 1, 0))/2 + (Kx_wo_offset + m_del_zmp_x(first_current_step_number_, prev_stepping_flag)/2) / (t_rest_init_ + t_double1_)*(i+1);
                        temp_py_wo_offset(i) = (m_del_zmp_y(first_current_step_number_, prev_stepping_flag) + foot_step_support_frame_(current_step_number - 2, 1) + foot_step_support_frame_(current_step_number - 1, 1))/2 + (Ky_wo_offset + m_del_zmp_y(first_current_step_number_, prev_stepping_flag)/2) / (t_rest_init_ + t_double1_)*(i+1);
                    }
                    else
                    {
                        temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0))/2 + Kx / (t_rest_init_ + t_double1_)*(i+1) + m_del_zmp_x(first_current_step_number_, prev_stepping_flag);
                        temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1))/2 + Ky / (t_rest_init_ + t_double1_)*(i+1) + m_del_zmp_y(first_current_step_number_, prev_stepping_flag);

                        temp_px_wo_offset(i) = (foot_step_support_frame_(current_step_number - 2, 0) + foot_step_support_frame_(current_step_number - 1, 0))/2 + Kx_wo_offset / (t_rest_init_ + t_double1_)*(i+1) + m_del_zmp_x(first_current_step_number_, prev_stepping_flag);
                        temp_py_wo_offset(i) = (foot_step_support_frame_(current_step_number - 2, 1) + foot_step_support_frame_(current_step_number - 1, 1))/2 + Ky_wo_offset / (t_rest_init_ + t_double1_)*(i+1) + m_del_zmp_y(first_current_step_number_, prev_stepping_flag);
                    }                        
                }
                else if (i >= t_rest_init_ + t_double1_ && i < t_total_zmp - t_rest_last_ - t_double2_)  
                {   
                    temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + m_del_zmp_x(first_current_step_number_, prev_stepping_flag);
                    temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + m_del_zmp_y(first_current_step_number_, prev_stepping_flag);
                    
                    temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0) + m_del_zmp_x(first_current_step_number_, prev_stepping_flag); 
                    temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1) + m_del_zmp_y(first_current_step_number_, prev_stepping_flag);
                }
                else if (i >= t_total_zmp - t_rest_last_ - t_double2_ && i < t_total_zmp)  
                {
                    temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2/(t_double2_ + t_rest_last_) * (i + 1 -(t_total_zmp - t_rest_last_ - t_double2_)) + m_del_zmp_x(first_current_step_number_, prev_stepping_flag);
                    temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2/(t_double2_ + t_rest_last_) * (i + 1 -(t_total_zmp - t_rest_last_ - t_double2_)) + m_del_zmp_y(first_current_step_number_, prev_stepping_flag);

                    temp_px_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 0) + Kx2_wo_offset / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_)) + m_del_zmp_x(first_current_step_number_, prev_stepping_flag);
                    temp_py_wo_offset(i) = foot_step_support_frame_(current_step_number - 1, 1) + Ky2_wo_offset / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_zmp - t_rest_last_ - t_double2_)) + m_del_zmp_y(first_current_step_number_, prev_stepping_flag);
                }                
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
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
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
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            }

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
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
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
        del_F_.setZero();
        opt_F_.setZero();
    }
          
    for (int i = 0; i < 6; i++)
    {
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i);        
    }
    zmp_modif_time_margin_ = 0.1*hz_;

    if(walking_tick_mj == t_start_ + t_total_ - t_double2_ - t_rest_last_ - zmp_modif_time_margin_) // 조현민 처럼 Step으로 zmp를 변경하는게 아니라 부드럽게 바꿔줘도 좋을듯 / SSP 끝나기 0.1초 전 스윙 발 X,Y 고정
    {
        fixed_swing_foot(0) = desired_swing_foot(0); 
        fixed_swing_foot(1) = desired_swing_foot(1);
        
        modified_del_zmp_(current_step_num_,0) = del_F_(0) - target_swing_foot(0);
        modified_del_zmp_(current_step_num_,1) = del_F_(1) - target_swing_foot(1);                         
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
        desired_swing_foot(0) = del_F_(0); // del_F_ is optimized by target_swing_foot_(0) + del_F_x
        desired_swing_foot(1) = del_F_(1);  
    }
    else
    {
        desired_swing_foot(0) = fixed_swing_foot(0);
        desired_swing_foot(1) = fixed_swing_foot(1);
    }    
 
    // real robot experiment 
    desired_swing_foot_LPF_(0) = 1 / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot_LPF_(0) + (2 * M_PI * 1.0 * del_t) / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot(0);
    desired_swing_foot_LPF_(1) = 1 / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot_LPF_(1) + (2 * M_PI * 1.0 * del_t) / (1 + 2 * M_PI * 1.0 * del_t) * desired_swing_foot(1); 
   
    
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
        ssp_flag = 0.1;        

        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            // for (int i = 0; i < 2; i++)
            // {
            //     rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            // }

            // 220422
            rfoot_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(0), desired_swing_foot(0), 0.0, 0.0);
            rfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    
            
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
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, foot_height_, 0.0, 0.0);
            }
            else
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            // for (int i = 0; i < 2; i++)
            // {
            //     lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            // }
            // 220422
            lfoot_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(0), desired_swing_foot(0), 0.0, 0.0);
            lfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    
            
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
            rfoot_trajectory_support_.translation()(0) =  desired_swing_foot(0);
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
            lfoot_trajectory_support_.translation()(0) =  desired_swing_foot(0); 
            lfoot_trajectory_support_.translation()(1) =  desired_swing_foot(1);    
            
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }

    // CPMPC Journal foot trajecotry data
    // MJ_graph_foottra_x << lfoot_trajectory_support_.translation()(0) << "," << rfoot_trajectory_support_.translation()(0) << "," << del_F_(0) << "," << desired_swing_foot(0) << "," << ssp_flag << endl;
    // MJ_graph_foottra_y << lfoot_trajectory_support_.translation()(1) << "," << rfoot_trajectory_support_.translation()(1) << "," << del_F_(1) << "," << desired_swing_foot(1) << endl;
     
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
    Gi(0, 0) = 872.3477; //Temp_mat_inv * B_bar_tran * K * I_bar ;
    //Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;
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
    ZMP_X_REF_ = ref_zmp_mj_(tick,0);
    ZMP_Y_REF_ = ref_zmp_mj_(tick,1);  

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
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (ref_zmp_mj_(tick + 1 + i,0) - ref_zmp_mj_(tick + i,0));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (ref_zmp_mj_(tick + 1 + i,1) - ref_zmp_mj_(tick + i,1));
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

    del_ux(0, 0) = -(px(0) - ref_zmp_mj_(tick,0)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - ref_zmp_mj_(tick,1)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x_mj + B * UX;
    YD = A * preview_y_mj + B * UY;    

    cp_desired_(0) = XD(0) + XD(1) / wn;
    cp_desired_(1) = YD(0) + YD(1) / wn; 

    // MJ_graph << XD(0) << "," << YD(0) << "," << ZMP_X_REF_ << "," << ZMP_Y_REF_ << "," << cp_desired_(0) << "," << cp_desired_(1) << endl;
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

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - 0*0.15 * damping_x - com_support_current_(0)); 
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7 * (com_desired_(1) - 0*0.6 * damping_y - com_support_current_(1));  
    // pelv_trajectory_support_.translation()(2) = com_desired_(2) + pelv_height_offset_; //DG
    pelv_trajectory_support_.translation()(2) = com_desired_(2) - 0*pelv_height_offset_;

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

    P_angle_input_dot = 1.5 * (0.0 - P_angle) ;
    R_angle_input_dot = 2.0 * (0.0 - R_angle) ;

    P_angle_input = P_angle_input + P_angle_input_dot * del_t;
    R_angle_input = R_angle_input + R_angle_input_dot * del_t;

    if (R_angle_input > 3 * DEG2RAD) //1.5 degree
    {
        R_angle_input = 3 * DEG2RAD;
    }
    else if (R_angle_input < -3 * DEG2RAD)
    {
        R_angle_input = -3 * DEG2RAD;
    }

    if (P_angle_input > 5 * DEG2RAD) //5 degree
    {
        P_angle_input = 5 * DEG2RAD;
        // cout << "a" << endl;
    }
    else if (P_angle_input < -5 * DEG2RAD)
    {
        P_angle_input = -5 * DEG2RAD;
        // cout << "b" << endl;
    }
    //Trunk_trajectory_euler(0) = R_angle_input;
    Trunk_trajectory_euler(1) = P_angle_input;    
    
    pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2)) * DyrosMath::rotateWithY(Trunk_trajectory_euler(1)) * DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void AvatarController::supportToFloatPattern()
{
    //lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
    //rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
    pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
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

    ZMP_X_REF_ = ref_zmp_mj_(walking_tick_mj - zmp_start_time_mj_,0);
    ZMP_Y_REF_ = ref_zmp_mj_(walking_tick_mj - zmp_start_time_mj_,1); 
    
    // State variables x_hat_ and Control input U_mpc are updated with every MPC frequency.
        
    int alpha_step = 0;

    if (foot_step_(current_step_num_, 6) == 1)
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
    if(atb_mpc_update_ == false)  
    {
        atb_mpc_update_ = true;
        walking_tick_mj_thread_ = walking_tick_mj;
        t_total_thread_ = t_total_;
        t_rest_init_thread_ = t_rest_init_;
        t_rest_last_thread_ = t_rest_last_;
        current_step_num_thread_ = current_step_num_;
        total_step_num_thread_ = total_step_num_;
        zmp_start_time_mj_thread_ = zmp_start_time_mj_;
        ref_zmp_thread_ = ref_zmp_mj_; 
        
        ref_zmp_wo_offset_thread_ = ref_zmp_mj_wo_offset_;  
        alpha_step_mpc_thread_ = alpha_step;        
        cp_measured_thread_ = cp_measured_;

        cam_thread_(1) = del_ang_momentum_(1);
        cam_thread_(0) = del_ang_momentum_(0);

        lfoot_support_current_thread_= lfoot_support_current_;
        rfoot_support_current_thread_= rfoot_support_current_;

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
  
        atb_mpc_update_ = false;
    } 

     // get the mpc result from thread3
    if(mpc_x_update_ == true) // 0.011 ~ 0.012 주기로 업데이트
    {   
        if(atb_mpc_x_update_ == false)
        {
            atb_mpc_x_update_ = true;
            if(current_step_num_thread2_ == current_step_num_)
            {    
                x_hat_r_ = x_hat_thread_;
                x_hat_r_p_ = x_hat_p_thread_;                  
                wieber_interpol_cnt_x_ = 1;
            }
            else
            {
                cout<<"MPC output X is ignored"<<endl;
            }
            atb_mpc_x_update_ = false;
        }
        x_diff_ = x_hat_r_ - x_hat_r_p_;
        mpc_x_update_ = false;
    } 

    if(mpc_y_update_ == true) // 0.011 ~ 0.012 주기로 업데이트
    {   
        if(atb_mpc_y_update_ == false)
        {
            atb_mpc_y_update_ = true; 
            if(current_step_num_thread2_ == current_step_num_)
            {        
                // cout<<"walking_tick_mj: "<<walking_tick_mj<<endl;
                y_hat_r_ = y_hat_thread_;
                y_hat_r_p_ = y_hat_p_thread_;
                wieber_interpol_cnt_y_ = 1;
            }
            else
            {
                cout<<"MPC output Y is ignored"<<endl;
            }
            atb_mpc_y_update_ = false;
        }
        
        y_diff_ = y_hat_r_ - y_hat_r_p_;
        mpc_y_update_ = false;
    }

    if(cpmpc_x_update_ == true)
    {
        if(atb_cpmpc_x_update_ == false) // 여기서 cp_des_zmp_y를 step change 시키고 cpmpc_des_zmp_y_thread_에 담아서 전달.
        {
            atb_cpmpc_x_update_ = true;

            if(current_step_num_thread2_ == current_step_num_)        
            {
                cp_des_zmp_x_prev_ = cp_des_zmp_x_; 
                cp_des_zmp_x_ = cpmpc_des_zmp_x_thread_;
                
                del_F_x_ = del_F_x_thread_;
                del_F_x_next_ = del_F_x_thread_;
                
                des_tau_y_ = des_tau_y_thread_;
                
                cpmpc_interpol_cnt_x_ = 1;
            }
            else
            {
                cout<<"CP output X is ignored"<<endl;
            }
            atb_cpmpc_x_update_ = false;
        }
        cpmpc_diff_(0) = cp_des_zmp_x_ - cp_des_zmp_x_prev_;
        cpmpc_x_update_ = false;
    }

    if(cpmpc_y_update_ == true)
    {
        if(atb_cpmpc_y_update_ == false) // 여기서 cp_des_zmp_y를 step change 시키고 cpmpc_des_zmp_y_thread_에 담아서 전달.
        {
            atb_cpmpc_y_update_ = true;

            if(current_step_num_thread2_ == current_step_num_)        
            {
                cp_des_zmp_y_prev_ = cp_des_zmp_y_; //cpmpc_des_zmp_y_prev_thread_; 
                cp_des_zmp_y_ = cpmpc_des_zmp_y_thread_;
                
                del_F_y_ = del_F_y_thread_;
                del_F_y_next_ = del_F_y_thread_;

                des_tau_x_ = des_tau_x_thread_;
                
                cpmpc_interpol_cnt_y_ = 1;
            }
            else
            {
                cout<<"CP output Y is ignored"<<endl;
            }
            atb_cpmpc_y_update_ = false;
        }
        cpmpc_diff_(1) = cp_des_zmp_y_ - cp_des_zmp_y_prev_;
        cpmpc_y_update_ = false;
    }
    
    double thread_freq = 50.0;

    double x_com_lin_spline = (thread_freq/hz_)*wieber_interpol_cnt_x_;
    double y_com_lin_spline = (thread_freq/hz_)*wieber_interpol_cnt_y_;

    x_com_lin_spline = DyrosMath::minmax_cut(x_com_lin_spline, 0.0, 1.0);
    y_com_lin_spline = DyrosMath::minmax_cut(y_com_lin_spline, 0.0, 1.0);

    x_mpc_i_ = x_com_lin_spline*x_diff_ + x_hat_r_p_; // 50.0 = MPC freq.
    y_mpc_i_ = y_com_lin_spline*y_diff_ + y_hat_r_p_;

    wieber_interpol_cnt_x_ ++;
    wieber_interpol_cnt_y_ ++;
    
    double x_cpmpc_lin_spline = (thread_freq/hz_)*cpmpc_interpol_cnt_x_;
    double y_cpmpc_lin_spline = (thread_freq/hz_)*cpmpc_interpol_cnt_y_;

    x_cpmpc_lin_spline = DyrosMath::minmax_cut(x_cpmpc_lin_spline, 0.0, 1.0);
    y_cpmpc_lin_spline = DyrosMath::minmax_cut(y_cpmpc_lin_spline, 0.0, 1.0);
    
    des_zmp_interpol_(0) = x_cpmpc_lin_spline*cpmpc_diff_(0) + cp_des_zmp_x_prev_;
    des_zmp_interpol_(1) = y_cpmpc_lin_spline*cpmpc_diff_(1) + cp_des_zmp_y_prev_;

    // del_F_(0) = del_F_x_;
    // del_F_(1) = del_F_y_;
    
    // MJ_graph << cp_des_zmp_y_prev_ << "," << cp_des_zmp_y_ << "," << del_F_x_prev_ << "," << del_F_x_ << "," << del_F_(0) << "," << des_zmp_interpol_(1) << endl;
    
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
         
        com_pos_prev(0) = x_hat_r_sc_(0);
        com_pos_prev(1) = y_hat_r_sc_(0);

        com_pos = temp_rot * (com_pos_prev - temp_pos); 

        com_vel_prev(0) = x_hat_r_sc_(1);
        com_vel_prev(1) = y_hat_r_sc_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_hat_r_sc_(2);
        com_acc_prev(1) = y_hat_r_sc_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_hat_r_sc_(0) = com_pos(0);
        y_hat_r_sc_(0) = com_pos(1);
        x_hat_r_sc_(1) = com_vel(0);
        y_hat_r_sc_(1) = com_vel(1);
        x_hat_r_sc_(2) = com_acc(0);
        y_hat_r_sc_(2) = com_acc(1);        

        com_pos_prev(0) = x_hat_r_p_sc_(0);
        com_pos_prev(1) = y_hat_r_p_sc_(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = x_hat_r_p_sc_(1);
        com_vel_prev(1) = y_hat_r_p_sc_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = x_hat_r_p_sc_(2);
        com_acc_prev(1) = y_hat_r_p_sc_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        x_hat_r_p_sc_(0) = com_pos(0);
        y_hat_r_p_sc_(0) = com_pos(1);
        x_hat_r_p_sc_(1) = com_vel(0);
        y_hat_r_p_sc_(1) = com_vel(1); 
        x_hat_r_p_sc_(2) = com_acc(0);
        y_hat_r_p_sc_(2) = com_acc(1);

        //com_pos_prev(0) = x_hat_r_p_sc_(0);
        com_pos_prev(0) = des_zmp_x_stepchange_;
        com_pos_prev(1) = des_zmp_y_stepchange_;
        com_pos = temp_rot * (com_pos_prev - temp_pos);        

        des_zmp_x_stepchange_ = com_pos(0);
        des_zmp_y_stepchange_ = com_pos(1); // step change 1 tick 전 desired ZMP (MPC output) step change    

        com_pos_prev(0) = des_zmp_x_prev_stepchange_;
        com_pos_prev(1) = des_zmp_y_prev_stepchange_;
        com_pos = temp_rot * (com_pos_prev - temp_pos);        

        des_zmp_x_prev_stepchange_ = com_pos(0);
        des_zmp_y_prev_stepchange_ = com_pos(1); // step change 1 tick 전 desired ZMP (MPC output) step change  
                
    }    
    //MJ_graph1 << ZMP_X_REF_ << "," << ZMP_Y_REF_ << "," << com_desired_(0) << "," << com_desired_(1) << "," << cp_desired_(0) << ","  << cp_desired_(1) << endl;  
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
    //com_desired_(2) = pelv_support_start_.translation()(2);
    com_desired_(2) = 0.77172;
    //cout << pelv_support_start_.translation()(2) << endl;
    
    
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
        
        temp_pos(0) = temp_pos(0) + modified_del_zmp_(current_step_num_,0); // 왼발 오른발 나눌까?
        temp_pos(1) = temp_pos(1) + modified_del_zmp_(current_step_num_,1);  
        
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
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            WBC::SetContact(rd_, 0, 1);
            Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
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
    //return grav_;
}

void AvatarController::parameterSetting()
{       
    target_x_ = 0.0;
    target_y_ = 0.0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0.0;
    step_length_x_ = 0.10;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;

    // t_rest_init_ = 0.27*hz_;
    // t_rest_last_ = 0.27*hz_;
    // t_double1_ = 0.03*hz_;
    // t_double2_ = 0.03*hz_;
    // t_total_= 1.3*hz_;

    t_rest_init_ = 0.12 * hz_; // Slack, 0.9 step time
    t_rest_last_ = 0.12 * hz_;
    t_double1_ = 0.03 * hz_;
    t_double2_ = 0.03 * hz_;
    t_total_ = 0.9 * hz_;
    t_total_const_ = 0.9 * hz_; 

    // t_rest_init_ = 0.02 * hz_; // Slack, 0.9 step time
    // t_rest_last_ = 0.02 * hz_;
    // t_double1_ = 0.03 * hz_;
    // t_double2_ = 0.03 * hz_;
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

    t_temp_ = 2.0 * hz_;
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
            // t_total_ = t_total_ + 0.05*hz_;
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

void AvatarController::hip_compensator()
{
    double left_hip_roll = -0.2 * DEG2RAD, right_hip_roll = -0.2 * DEG2RAD, left_hip_roll_first = -0.20 * DEG2RAD, right_hip_roll_first = -0.20 * DEG2RAD, //실험, 제자리 0.6, 0.4
        left_hip_pitch = 0.4 * DEG2RAD, right_hip_pitch = 0.4 * DEG2RAD, left_hip_pitch_first = 0.40 * DEG2RAD, right_hip_pitch_first = 0.40 * DEG2RAD,    // 실험 , 제자리 0.75deg
        left_ank_pitch = 0.0 * DEG2RAD, right_ank_pitch = 0.0 * DEG2RAD, left_ank_pitch_first = 0.0 * DEG2RAD, right_ank_pitch_first = 0.0 * DEG2RAD,
           left_hip_roll_temp = 0.0, right_hip_roll_temp = 0.0, left_hip_pitch_temp = 0.0, right_hip_pitch_temp = 0.0, left_ank_pitch_temp = 0.0, right_ank_pitch_temp = 0.0, temp_time = 0.05 * hz_;

    if (current_step_num_ == 0)
    {
        if (foot_step_(current_step_num_, 6) == 1) //left support foot
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
        if (foot_step_(current_step_num_, 6) == 1) //left support foot
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

    Tau_CP(4) = 0 * F_L * del_zmp(0);  // L pitch
    Tau_CP(10) = 0 * F_R * del_zmp(0); // R pitch

    Tau_CP(5) = -0 * F_L * del_zmp(1);  // L roll
    Tau_CP(11) = -0 * F_R * del_zmp(1); // R roll
}

//real robot experiment
void AvatarController::CP_compen_MJ_FT() 
{ // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF_ 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
    double alpha = 0;
    double F_R = 0, F_L = 0;
    double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
    double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;
    double zmp_offset = 0;
    double alpha_new = 0;

    zmp_offset = 0.015; // 0.9초

    if (walking_tick_mj > t_temp_)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
            }
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset;
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset;
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset - zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset + zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
            }
        }
        else
        {
            ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
        }
    }
    else
    {
        ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
    }

    del_zmp(0) = des_zmp_interpol_(0) - ZMP_X_REF_;
    del_zmp(1) = des_zmp_interpol_(1) - ZMP_Y_REF_alpha_;

    ////////////////////////
    //   double A = 0, B = 0, d = 0, X1 = 0, Y1 = 0, e_2 = 0, L = 0, l = 0;
    //   A =  (lfoot_support_current_.translation()(0) - rfoot_support_current_.translation()(0));
    //   B = -(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    //   X1 = ZMP_Y_REF_alpha + 0*del_zmp(1) - rfoot_support_current_.translation()(1);
    //   Y1 = ZMP_X_REF_ + 0*del_zmp(0) - rfoot_support_current_.translation()(0);
    //   L = sqrt(A*A + B*B);
    //   d = abs(A*X1 + B*Y1) / L;
    //   e_2 = X1*X1 + Y1*Y1;
    //   l = sqrt(e_2 - d*d);
    //   alpha_new = l/L;
    alpha = (ZMP_Y_REF_alpha_ + 1.0 * del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    if (alpha > 1)
    {
        alpha = 1.0;
    } // 왼발 지지때 alpha = 1
    else if (alpha < 0)
    {
        alpha = 0.0;
    }

    if(walking_tick_mj == 0)
    {
        alpha_lpf_ = alpha;
    }

    alpha_lpf_ = 1 / (1 + 2 * M_PI * 6.0 * del_t) * alpha_lpf_ + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * alpha;
    //   if(alpha_new > 1)
    //   { alpha_new = 1; } // 왼발 지지때 alpha = 1
    //   else if(alpha_new < 0)
    //   { alpha_new = 0; }

    double real_robot_mass_offset_ = 52; // 42 75

    F_R = -(1 - alpha_lpf_) * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ + 0*15);
    F_L = -alpha_lpf_ * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ - 0*15); // alpha가 0~1이 아니면 desired force가 로봇 무게보다 계속 작게나와서 지면 반발력을 줄이기위해 다리길이를 줄임.

    if (walking_tick_mj == 0)
    {
        F_F_input = 0.0;
        F_T_L_x_input = 0.0;
        F_T_R_x_input = 0.0;
        F_T_L_y_input = 0.0;
        F_T_R_y_input = 0.0;
    }

    F_F_error_pre_ = F_F_error_;
    
    F_F_error_ = (l_ft_LPF(2) - r_ft_LPF(2)) - (F_L - F_R);    
    
    F_F_error_dot_ = (F_F_error_ - F_F_error_pre_)*hz_;

    //////////// Force
    F_F_input_dot = 0.0001 * F_F_error_  + 0.00000001*F_F_error_dot_ - 3.0 * F_F_input;  //DG's code Kp : 0.00005

    F_F_input = F_F_input + F_F_input_dot * del_t;
    F_F_input = DyrosMath::minmax_cut(F_F_input, -0.02, 0.02);
  

    //////////// Torque 
    Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha_ + del_zmp(1))) * F_L);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_L);

    Tau_all_x = DyrosMath::minmax_cut(Tau_all_x, -100.0, 100.0);
    Tau_all_y = DyrosMath::minmax_cut(Tau_all_y, -100.0, 100.0);

    Tau_R_x = (1 - alpha_lpf_) * Tau_all_x;
    Tau_L_x = (alpha_lpf_)*Tau_all_x;

    Tau_L_y = -alpha_lpf_ * Tau_all_y;
    Tau_R_y = -(1 - alpha_lpf_) * Tau_all_y;
    
    Tau_L_x_error_pre_ = Tau_L_x_error_;
    Tau_L_x_error_ = -(Tau_L_x - l_ft_LPF(3));
    Tau_L_x_error_dot_ = (Tau_L_x_error_ - Tau_L_x_error_pre_)*hz_;

    Tau_R_x_error_pre_ = Tau_R_x_error_;
    Tau_R_x_error_ = -(Tau_R_x - r_ft_LPF(3));
    Tau_R_x_error_dot_ = (Tau_R_x_error_ - Tau_R_x_error_pre_)*hz_;

    Tau_L_y_error_pre_ = Tau_L_y_error_;
    Tau_L_y_error_ = Tau_L_y - l_ft_LPF(4);
    Tau_L_y_error_dot_ = (Tau_L_y_error_ - Tau_L_y_error_pre_)*hz_;


    Tau_R_y_error_pre_ = Tau_R_y_error_;
    Tau_R_y_error_ = Tau_R_y - r_ft_LPF(4);
    Tau_R_y_error_dot_ = (Tau_R_y_error_ - Tau_R_y_error_pre_)*hz_;
    
 
    // Roll: 0.030, 0.0005, 10 Pitch: 0.030, 0.0005, 5 -> 3 Degree slope
    // Roll 방향 (-0.02/-30 0.9초) large foot(blue pad): 0.05/50 / small foot(orange pad): 0.07/50
    //   F_T_L_x_input_dot = -0.015*(Tau_L_x - l_ft_LPF(3)) - Kl_roll*F_T_L_x_input;
    // 0.025/0.0005/-10 : DG data collection
    F_T_L_x_input_dot = 0.02 * (Tau_L_x_error_) +0.0005*Tau_L_x_error_dot_ - 10.0 * F_T_L_x_input;
    F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
    //   F_T_L_x_input = 0;
    //   F_T_R_x_input_dot = -0.015*(Tau_R_x - r_ft_LPF(3)) - Kr_roll*F_T_R_x_input;
    F_T_R_x_input_dot = 0.02 * (Tau_R_x_error_) +0.0005*Tau_R_x_error_dot_ - 10.0 * F_T_R_x_input;
    F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
    //   F_T_R_x_input = 0;

    // Pitch 방향  (0.005/-30 0.9초) large foot(blue pad): 0.04/50 small foot(orange pad): 0.06/50
    //   F_T_L_y_input_dot = 0.005*(Tau_L_y - l_ft_LPF(4)) - Kl_pitch*F_T_L_y_input;
    // 0.035/0.0005/-5: 3degree slope possilbe
    // 0.02/0.0005/-5 : DG data collection
    F_T_L_y_input_dot = 0.02 * (Tau_L_y_error_) + 0.0005*Tau_L_y_error_dot_ - 10.0 * F_T_L_y_input;
    F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
    //   F_T_L_y_input = 0;
    //   F_T_R_y_input_dot = 0.005*(Tau_R_y - r_ft_LPF(4)) - Kr_pitch*F_T_R_y_input;
    F_T_R_y_input_dot = 0.02 * (Tau_R_y_error_) + 0.0005*Tau_R_y_error_dot_ - 10.0 * F_T_R_y_input;
    F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;
  
    F_T_L_x_input = DyrosMath::minmax_cut(F_T_L_x_input, -20*DEG2RAD, 20*DEG2RAD);
    F_T_R_x_input = DyrosMath::minmax_cut(F_T_R_x_input, -20*DEG2RAD, 20*DEG2RAD);
    F_T_L_y_input = DyrosMath::minmax_cut(F_T_L_y_input, -20*DEG2RAD, 20*DEG2RAD);
    F_T_R_y_input = DyrosMath::minmax_cut(F_T_R_y_input, -20*DEG2RAD, 20*DEG2RAD);
    
}

//simulation
// void AvatarController::CP_compen_MJ_FT()
// { 
//     // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF_ 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
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
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
//             }
//             else
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
//             }
//         }
//         else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
//         {
//             if (foot_step_(current_step_num_, 6) == 1)
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset;
//             }
//             else
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset;
//             }
//         }
//         else if (walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
//         {
//             if (foot_step_(current_step_num_, 6) == 1)
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset - zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
//             }
//             else
//             {
//                 ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset + zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
//             }
//         }
//         else
//         {
//             ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
//         }
//     }
//     else
//     {
//         ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
//     }
     
//     del_zmp(0) = des_zmp_interpol_(0) - ZMP_X_REF_;
//     del_zmp(1) = des_zmp_interpol_(1) - ZMP_Y_REF_alpha_;
     
//     // del_zmp(0) = DyrosMath::minmax_cut(del_zmp(0), -0.1, 0.1);
//     // del_zmp(1) = DyrosMath::minmax_cut(del_zmp(1), -0.07, 0.07); 
//     ////////////////////////
//     // double A = 0, B = 0, d = 0, X1 = 0, Y1 = 0, e_2 = 0, L = 0, l = 0;
//     // A = (lfoot_support_current_.translation()(0) - rfoot_support_current_.translation()(0));
//     // B = -(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
//     // X1 = ZMP_Y_REF_alpha_ + 0 * del_zmp(1) - rfoot_support_current_.translation()(1);
//     // Y1 = ZMP_X_REF_ + 0 * del_zmp(0) - rfoot_support_current_.translation()(0);
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
//     Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_L);
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
  
//     // MJ_graph << stepping_input_(0) << "," << stepping_input_(1) << "," << t_total_ / hz_ << "," << ZMP_Y_REF_alpha_ + del_zmp(1) << "," << ZMP_Y_REF_alpha_ << endl;
// }

void AvatarController::CentroidalMomentCalculator_new()
{
    
    if (walking_tick_mj == 0)
    {
        del_tau_.setZero();
        del_ang_momentum_.setZero();
        del_ang_momentum_prev_.setZero();
    }

    del_ang_momentum_prev_ = del_ang_momentum_;   
    
    // double recovery_damping = 2.0; //damping 20 is equivalent to 0,99 exp gain // 2정도 하면 반대방향으로 치는게 15Nm, 20하면 150Nm
    // 나중에 반대방향 토크 limit 걸어야됨.
    // X direction CP control  
    del_tau_(1) =  des_tau_y_ ;//- recovery_damping*del_ang_momentum_(1);
 
    // Y direction CP control        
    del_tau_(0) = -des_tau_x_ ;//- recovery_damping*del_ang_momentum_(0); 
    
    //// Integrate Centroidal Moment
    del_ang_momentum_(1) = del_ang_momentum_prev_(1) + del_t * del_tau_(1);
    del_ang_momentum_(0) = del_ang_momentum_prev_(0) + del_t * del_tau_(0);

    // del CAM output limitation (220118/ DLR's CAM output is an approximately 4 Nms and TORO has a weight of 79.2 kg)    
    double A_limit = 10.0;
       
    if(del_ang_momentum_(0) > A_limit)
    { 
        del_ang_momentum_(0) = A_limit; 
    }
    else if(del_ang_momentum_(0) < -A_limit)
    { 
        del_ang_momentum_(0) = -A_limit; 
    }
    if(del_ang_momentum_(1) > A_limit)
    { 
        del_ang_momentum_(1) = A_limit; 
    }
    else if(del_ang_momentum_(1) < -A_limit)
    { 
        del_ang_momentum_(1) = -A_limit; 
    }
     
}

void AvatarController::getCentroidalMomentumMatrix(MatrixXd mass_matrix, MatrixXd &CMM)
{ // 1. CMM 계산 함수.
    //reference: Cavenago, et.al "Contact force observer for space robots." 2019 IEEE 58th Conference on Decision and Control (CDC). IEEE, 2019.
    // mass_matrix: inertia matrix expressed in the base frame
    // Using this CMM, calculation error of the angular velocity occurs in second decimal place with respect to the rbdl CalcCenterOfMass() function when tocabi is walking in place.
    // Calculation Time : 3~4us
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
    if (walking_tick_mj == 0)
    {
        calculateFootStepTotal_MJoy(); // joystick&pedal Footstep
        joy_enable_ = true;
        std::cout << "step_length : " << joystick_input_(0) << " trigger(z) : " << joystick_input_(1) << " theta : " << joystick_input_(2) << std::endl;

        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        //pelv_float_init_.translation()(0) += 0.11;

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

        if (foot_step_(0, 6) == 0) //right foot support
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

        if (foot_step_(0, 6) == 1) //left suppport foot
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
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        //pelv_float_init_.translation()(0) += 0.11;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        Eigen::Isometry3d ref_frame;

        if (foot_step_(current_step_num_, 6) == 0) //right foot support
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
            joystick_input_(0) = (joystick_input(0) + 1) / 2; //FW
            joystick_input_(3) = (joystick_input(3) + 1) / 2; //BW
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

        foot_step_(0, 5) = temp2 * joystick_input_(2) * theta;                                                            //0.0;
        foot_step_(0, 0) = (width - width_buffer) * sin(foot_step_(0, 5)) + temp2 * length_total * cos(foot_step_(0, 5)); //0.0;
        foot_step_(0, 1) = -(width - width_buffer) * cos(foot_step_(0, 5)) + temp2 * length_total * sin(foot_step_(0, 5));
        foot_step_(0, 6) = 1.0;
        temp2++;

        foot_step_(1, 5) = temp2 * joystick_input_(2) * theta; //0.0;
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
