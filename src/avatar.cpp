#include "avatar.h"
#include <fstream>
using namespace TOCABI;

ofstream e_mpc_time_graph("/home/econom2-20/data/e_mpc_time_graph.txt");
ofstream e_mpc_time_graph3("/home/econom2-20/data/e_mpc_time_graph3.txt");
ofstream e_mpc_time_graph4("/home/econom2-20/data/e_mpc_time_graph4.txt");
ofstream e_main_time_graph("/home/econom2-20/data/e_main_time_graph.txt");
ofstream e_mpc_planner_data("/home/econom2-20/data/e_mpc_planner_data.txt");
ofstream e_mpc_stabilizer_data("/home/econom2-20/data/e_mpc_stabilizer_data.txt");

ofstream e_tmp_graph1("/home/econom2-20/data/e_tmp_graph1.txt");
ofstream e_tmp_graph2("/home/econom2-20/data/e_tmp_graph2.txt");
ofstream e_tmp_graph3("/home/econom2-20/data/e_tmp_graph3.txt");
ofstream e_tmp_graph4("/home/econom2-20/data/e_tmp_graph4.txt");
ofstream e_tmp_graph5("/home/econom2-20/data/e_tmp_graph5.txt");
ofstream e_tmp_graph6("/home/econom2-20/data/e_tmp_graph6.txt");
ofstream e_tmp_graph7("/home/econom2-20/data/e_tmp_graph7.txt");
ofstream e_tmp_graph8("/home/econom2-20/data/e_tmp_graph8.txt");
ofstream e_tmp_graph9("/home/econom2-20/data/e_tmp_graph9.txt");
ofstream e_tmp_graph10("/home/econom2-20/data/e_tmp_graph10.txt");
ofstream e_tmp_graph11("/home/econom2-20/data/e_tmp_graph11.txt");
ofstream e_tmp_graph12("/home/econom2-20/data/e_tmp_graph12.txt");
ofstream e_tmp_graph13("/home/econom2-20/data/e_tmp_graph13.txt");
ofstream e_tmp_graph14("/home/econom2-20/data/e_tmp_graph14.txt");
ofstream e_tmp_graph15("/home/econom2-20/data/e_tmp_graph15.txt");
ofstream e_tmp_graph16("/home/econom2-20/data/e_tmp_graph16.txt");
ofstream e_tmp_graph17("/home/econom2-20/data/e_tmp_graph17.txt");
ofstream e_tmp_graph18("/home/econom2-20/data/e_tmp_graph18.txt");
ofstream e_tmp_graph19("/home/econom2-20/data/e_tmp_graph19.txt");
ofstream e_tmp_graph20("/home/econom2-20/data/e_tmp_graph20.txt");
ofstream e_tmp_graph21("/home/econom2-20/data/e_tmp_graph21.txt");
ofstream e_tmp_graph22("/home/econom2-20/data/e_tmp_graph22.txt");
ofstream e_tmp_graph23("/home/econom2-20/data/e_tmp_graph23.txt");
ofstream e_tmp_graph24("/home/econom2-20/data/e_tmp_graph24.txt");
ofstream e_tmp_graph25("/home/econom2-20/data/e_tmp_graph25.txt");
ofstream e_tmp_graph26("/home/econom2-20/data/e_tmp_graph26.txt");
ofstream e_tmp_graph27("/home/econom2-20/data/e_tmp_graph27.txt");
ofstream e_tmp_graph28("/home/econom2-20/data/e_tmp_graph28.txt");

AvatarController::AvatarController(RobotData &rd) : rd_(rd)
{
    nh_avatar_.setCallbackQueue(&queue_avatar_);

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

    //opto_ftsensor_sub = nh_avatar_.subscribe("/atiforce/ftsensor", 100, &AvatarController::OptoforceFTCallback, this); // real robot experiment

    bool urdfmode = false;
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);
    ros::param::get("/tocabi_controller/sim_mode", param_sim_mode_);
    ros::param::get("/econom2_ext_time",param_ext_force_time_);
    ros::param::get("/econom2_ext_step",param_ext_force_step_);
    ros::param::get("/econom2_extforce",param_ext_force_);
    ros::param::get("/econom2_exttheta",param_ext_theta_);

    ros::param::get("/econom2_scenario",param_scenario_);

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_d_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_c_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_C_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_MJ_, true, false);

    setGains();

    first_loop_hqpik_ = true;
    first_loop_hqpik2_ = true;
    first_loop_qp_retargeting_ = true;
}

void AvatarController::setGains()
{
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

    //1st arm joint vel limit
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
    joint_vel_limit_l_(20) = -1.3 * M_PI; // 2 *
    joint_vel_limit_h_(20) = 1.3 * M_PI; // 2 *
    joint_vel_limit_l_(30) = -1.3 * M_PI; // 2 *
    joint_vel_limit_h_(30) = 1.3 * M_PI; // 2 *
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
            
            CAM_upper_init_q_.setZero();
            Initial_ref_upper_q_.setZero();

            for (int i = 12; i < MODEL_DOF; i++)
            {
                Initial_ref_upper_q_(i) = ref_q_(i);
                CAM_upper_init_q_(i) = ref_q_(i);
            }

            CAM_upper_init_q_(13) = 0.15;

            CAM_upper_init_q_(15) = + 15.0 * DEG2RAD; // Left Shoulder Yaw joint // 17 deg
            CAM_upper_init_q_(16) = + 10.0 * DEG2RAD; // Left Shoulder Pitch joint // 17 deg
            CAM_upper_init_q_(17) = + 65.0 * DEG2RAD; // Left Shoulder Roll joint // 86 deg
            CAM_upper_init_q_(18) = - 70.0 * DEG2RAD; // Left Elbow Yaw joint // -72 deg
            CAM_upper_init_q_(19) = - 65.0 * DEG2RAD; // Left Elbow Pitch joint // -57 deg

            CAM_upper_init_q_(25) = - 15.0 * DEG2RAD; // Right Shoulder Yaw joint // -17 deg
            CAM_upper_init_q_(26) = - 10.0 * DEG2RAD; // Right Shoulder Pitch joint           
            CAM_upper_init_q_(27) = - 65.0 * DEG2RAD; // Right Shoulder Roll joint 
            CAM_upper_init_q_(28) = + 70.0 * DEG2RAD; // Right Elbow Yaw joint
            CAM_upper_init_q_(29) = + 65.0 * DEG2RAD; // Right Elbow Pich joint                       
            
            q_prev_MJ_ = rd_.q_;
            walking_tick_ = 0;
            scenario_tick_ = 0;
            walking_end_flag = 0;
            parameterSetting();
            cout << "computeslow mode = 10 is initialized" << endl;
            cout << "time: "<<rd_.control_time_ << endl; //dg add

            WBC::SetContact(rd_, 1, 1);
            Gravity_MJ_ = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);
            atb_grav_update_ = false;
            initial_flag = 1;
        }

        if (atb_grav_update_ == false)
        {
            atb_grav_update_ = true;
            Gravity_MJ_fast_ = Gravity_MJ_;
            atb_grav_update_ = false;
        }

        if(initial_tick_ <= 3.0 * hz_)
        {
            //waist pitch
            ref_q_(13) = DyrosMath::cubic(initial_tick_, 0, 2.0 * hz_, Initial_ref_upper_q_(13), CAM_upper_init_q_(13), 0.0, 0.0);

            for (int i = 15; i < 22; i++)
            {
                //left arm
                ref_q_(i)      = DyrosMath::cubic(initial_tick_, 0, 2.0 * hz_, Initial_ref_upper_q_(i),      CAM_upper_init_q_(i),      0.0, 0.0);
                
                //right arm
                ref_q_(i + 10) = DyrosMath::cubic(initial_tick_, 0, 2.0 * hz_, Initial_ref_upper_q_(i + 10), CAM_upper_init_q_(i + 10), 0.0, 0.0);
            }

            if(initial_tick_ == 3.0 * hz_)
            {
                cout << "computeslow mode = 10 initialization finished" << endl;
            }
        }

        initial_tick_ ++;

        if(initial_tick_ > 3.0*hz_)
        {
            //cout << "initial_tick_: " << initial_tick_ << endl;
        }

        for (int i = 0; i < MODEL_DOF; i++)
        {
            rd_.torque_desired(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
        }    
    }
    else if (rd_.tc_.mode == 11)
    {
        ////////////////////////////////////////////////////////////////////////////
        ////////////// Biped Walking Controller modified by Econom2 ////////////////
        ////////////////////////////////////////////////////////////////////////////

        ////////////////////////////// Econom2 working /////////////////////////////
        if (walking_enable_ == true)
        {
            if (walking_tick_ == 0)
            {
                parameterSetting();
                initial_flag = 0;

                atb_grav_update_ = false;
                atb_desired_q_update_ = false;
                atb_walking_traj_update_ = false;
                atb_desired_torque_update_ = false;

                torque_upper_fast_.setZero();
                torque_upper_fast_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);
                torque_upper_.setZero();
                torque_upper_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);

                torque_desired_prev_.setZero();
                torque_desired_prev_container_to_fast_.setZero();
                torque_desired_prev_fast_.setZero();

                cout << "parameter setting OK" << endl;
                cout << "mode = 11" << endl;
            }

            updateInitialState();     
            getRobotState(); 
            floatToSupportFootstep();      

            if (current_step_num_ < total_step_num_)
            {   
                getZmpTrajectory();
                getComTrajectory_mpc();
                //getComTrajectory();
                getFootTrajectory_stepping();
                getPelvTrajectory(); 
                supportToFloatPattern();
                computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);

                if(walking_tick_ == 1999 && param_scenario_ == 1)
                {
                    double calc_13_1 = DyrosMath::cubic(scenario_tick_ - walking_tick_, 1.0*hz_, 3.0*hz_, 0.1, 0.4, 0.0, 0.0);
                    double calc_13_2 = DyrosMath::cubic(scenario_tick_ - walking_tick_, 6.0*hz_, 8.0*hz_, 0.4, 0.1, 0.0, 0.0);

                    CAM_upper_init_q_(13) = min(calc_13_1, calc_13_2);
                    
                    CAM_upper_init_q_(16) = DyrosMath::cubic(scenario_tick_ - walking_tick_, 1.0*hz_, 3.0*hz_,  0.174533,  0.174533 - 0.5, 0.0, 0.0);
                    CAM_upper_init_q_(26) = DyrosMath::cubic(scenario_tick_ - walking_tick_, 1.0*hz_, 3.0*hz_, -0.174533, -0.174533 + 0.5, 0.0, 0.0);

                    CAM_upper_init_q_(17) = DyrosMath::cubic(scenario_tick_ - walking_tick_, 1.0*hz_, 3.0*hz_,  1.134460,  1.134460 + 0.4, 0.0, 0.0);
                    CAM_upper_init_q_(27) = DyrosMath::cubic(scenario_tick_ - walking_tick_, 1.0*hz_, 3.0*hz_, -1.134460, -1.134460 - 0.4, 0.0, 0.0);
                }

                ref_q_.segment(0, 12) = q_des_;

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    Gravity_MJ_fast_ = Gravity_MJ_;
                    atb_grav_update_ = false;
                }

                if (walking_tick_ < 1.0 * hz_)
                {
                    for (int i = 0; i < 12; i++) //for leg
                    {
                        ref_q_(i) = DyrosMath::cubic(walking_tick_, 0, 1.0 * hz_, Initial_ref_q_(i), q_des_(i), 0.0, 0.0);
                    }
                }

                CP_compen_MJ_FT();
                
                torque_lower_.setZero();
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
                }

                desired_q_not_compensated_ = ref_q_;
                updateNextStepTime();
                q_prev_MJ_ = rd_.q_;

                if(current_step_num_ == param_ext_force_step_ && (walking_tick_ >= t_start_ + param_ext_force_time_*hz_)  && (walking_tick_ < t_start_ + (param_ext_force_time_ + 0.2)*hz_))
                { 
                    mujoco_applied_ext_force_.data[0] = param_ext_force_*cos(param_ext_theta_*DEG2RAD);
                    mujoco_applied_ext_force_.data[1] = param_ext_force_*sin(param_ext_theta_*DEG2RAD);
                    mujoco_applied_ext_force_.data[2] =  0.0; //z-axis linear force
                    mujoco_applied_ext_force_.data[3] =  0.0; //x-axis angular moment
                    mujoco_applied_ext_force_.data[4] =  0.0; //y-axis angular moment
                    mujoco_applied_ext_force_.data[5] =  0.0; //z-axis angular moment

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
        q_desired_virtual_pre_ = q_desired_virtual_;

        q_desired_virtual_.segment(0,6).setZero();
        q_desired_virtual_.segment(0,3)   = pelv_trajectory_support_.translation();
        q_desired_virtual_.segment(3,3)   = DyrosMath::rot2Euler(pelv_trajectory_support_.linear());
        q_desired_virtual_.segment(6,12)  = ref_q_.segment(0,12);
        q_desired_virtual_.segment(18,21) = CAM_upper_init_q_.segment(12,21);

        if(walking_tick_ == 0)
        {
            q_desired_virtual_pre_ = q_desired_virtual_;
        }

        qdot_desired_virtual_ = (q_desired_virtual_ - q_desired_virtual_pre_) * hz_;

        q_error_virtual_ = q_desired_virtual_ - q_virtual_;

        q_error_virtual_.segment(3,3) = - DyrosMath::getPhi(DyrosMath::Euler2rot(q_virtual_(0),         q_virtual_(1),         q_virtual_(2)),
                                                            DyrosMath::Euler2rot(q_desired_virtual_(0), q_desired_virtual_(1), q_desired_virtual_(2)));

        qddot_desired_virtual_ = Kp_virtual_.asDiagonal()*q_error_virtual_ - Kd_virtual_.asDiagonal()*rd_.q_dot_virtual_;

        if(atb_walking_traj_update_ == false)
        {
            atb_walking_traj_update_ = true;
            del_ang_momentum_fast_ = del_ang_momentum_;

            qddot_desired_virtual_container_to_fast_ = qddot_desired_virtual_;
            contact_wrench_container_to_fast_ = contact_wrench_;

            torque_desired_prev_container_to_fast_ = torque_desired_prev_;

            is_lfoot_support_container_to_fast_ = is_lfoot_support_;
            is_rfoot_support_container_to_fast_ = is_rfoot_support_;

            is_ssp_container_to_fast_ = is_ssp_;
            is_dsp_container_to_fast_ = is_dsp_;

            atb_walking_traj_update_ = false;
        }

        if(atb_grav_update_ == false)
        {
            atb_grav_update_ = true;
            Gravity_MJ_fast_ = Gravity_MJ_;
            atb_grav_update_ = false;
        }
        
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
        }

        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = torque_lower_ + torque_upper_;
        ////////////////////////////////////////////////////////////////////////////////
        

        if(atb_desired_torque_update_ == false)
        {
            atb_desired_torque_update_ = true;
            torque_wbd_ = torque_wbd_container_to_fast_;
            atb_desired_torque_update_ = false;
        }

        //WBD
        Eigen::VectorQd torque_sum = torque_wbd_ + (Kp_virtual_.asDiagonal()*q_error_virtual_ - Kd_virtual_.asDiagonal()*rd_.q_dot_virtual_).segment(6, MODEL_DOF);

        for(int i = 0; i < MODEL_DOF; i ++)
        {
            torque_sum(i) = DyrosMath::minmax_cut(torque_sum(i), -rd_.torque_limit(i), rd_.torque_limit(i));
        }

        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        torque_desired_prev_ = torque_sum;

        //rd_.torque_desired   = torque_sum;
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
            walking_tick_ = 0;
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
            if (walking_tick_ == 0)
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

                for (int i = 0; i < 12; i++)
                {
                    ref_q_(i) = q_des_(i);
                }

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

                if (walking_tick_ < 1.0 * hz_)
                {
                    for (int i = 0; i < 12; i++)
                    {
                        ref_q_(i) = DyrosMath::cubic(walking_tick_, 0, 1.0 * hz_, Initial_ref_q_(i), q_des_(i), 0.0, 0.0);
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

            for (int i = 0; i < 12; i++)
            {
                ref_q_(i) = q_des_(i);
            }

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
            }
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

        if(atb_walking_traj_update_ == false)
        {
            atb_walking_traj_update_ = true;
            del_ang_momentum_slow_ = del_ang_momentum_fast_;

            qddot_desired_virtual_fast_ = qddot_desired_virtual_container_to_fast_;
            contact_wrench_fast_ = contact_wrench_container_to_fast_;

            torque_desired_prev_fast_ = torque_desired_prev_container_to_fast_;

            is_lfoot_support_fast_ = is_lfoot_support_container_to_fast_;
            is_rfoot_support_fast_ = is_rfoot_support_container_to_fast_;

            is_ssp_fast_ = is_ssp_container_to_fast_;
            is_dsp_fast_ = is_dsp_container_to_fast_;

            atb_walking_traj_update_ = false;
        }

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

        torque_wbd_fast_ = MitWholebodyInverseDynamicsController(torque_wbd_fast_, qddot_desired_virtual_fast_, contact_wrench_fast_);
        //STEP3: Compute q_dot for CAM control
         
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

        if (atb_desired_torque_update_ == false)
        {
            atb_desired_torque_update_ = true;

            torque_wbd_container_to_fast_ = torque_wbd_fast_;

            atb_desired_torque_update_ = false;
        }

        savePreData();
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

        if ((current_time_) >= program_start_time_ + program_ready_duration_)
        {
            torque_task_ += comVelocityControlCompute(); //support chain control for COM velocity and pelvis orientation
            torque_task_ += swingFootControlCompute(); //swing foot control
            torque_task_ += jointTrajectoryPDControlCompute(); //upper body motion + joint damping control
        }
        torque_task_.segment(0, 12).setZero();
        torque_task_ += ikBalanceControlCompute();

        savePreData();

        ////////////////////////////////TORQUE LIMIT//////// //////////////////////
        for (int i = 0; i < MODEL_DOF; i++)
        {
            torque_task_(i) = DyrosMath::minmax_cut(torque_task_(i), torque_task_min_(i), torque_task_max_(i));
        }
        ///////////////////////////////////////////////////////////////////////////

        rd_.torque_desired = torque_task_;
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
    //ref_zmp_.setZero(zmp_size_, 2);
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
    pelv_yaw_rot_current_from_global_ = pelv_rot_current_;
    pelv_rot_current_yaw_aline_ = pelv_yaw_rot_current_from_global_.transpose() * pelv_rot_current_;

    pelv_transform_current_from_global_.translation().setZero();
    pelv_transform_current_from_global_.linear() = pelv_rot_current_yaw_aline_;

    pelv_angvel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Pelvis].w;

    com_pos_current_ = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[COM_id].xpos - pelv_pos_current_);
    com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].v;
    com_mass_ = rd_.link_[COM_id].mass;

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
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

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
    
    Eigen::VectorXd current_momentum = A_mat_ * q_dot_virtual_c;
    Eigen::VectorXd current_torque, nonlinear_torque_g, mob_residual_pre;
    current_torque.setZero(MODEL_DOF_VIRTUAL, 1);
    current_torque.segment(6, MODEL_DOF) = rd_.torque_desired;
    // cout<<"current_torque:" << current_torque.transpose() <<endl;

    nonlinear_torque_g = A_dot_mat_ * rd_.q_dot_virtual_ - (nonlinear_torque_);

    mob_residual_pre = mob_residual_;

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
                    foot_swing_trigger_ = false;
                    first_step_trigger_ = false;
                    start_time_ = current_time_;
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
        }
    }

    if (walking_phase_ == 1)
    {
        if (walking_speed_ == 0)
        {
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
}

void AvatarController::getProcessedRobotData()
{
    if (foot_contact_ == 1) // left support foot
    {
        swing_foot_transform_current_ = rfoot_transform_current_from_global_;
        support_foot_transform_current_ = lfoot_transform_current_from_global_;
        swing_foot_vel_current_ = rfoot_vel_current_from_global_;
    }
    else if (foot_contact_ == -1) //right support foot
    {
        swing_foot_transform_current_ = lfoot_transform_current_from_global_;
        support_foot_transform_current_ = rfoot_transform_current_from_global_;
        swing_foot_vel_current_ = lfoot_vel_current_from_global_;
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

    zc_ = com_pos_current_from_support_(2);
    wn_ = sqrt(GRAVITY / zc_);

    cp_current_from_suppport_ = com_pos_current_from_support_ + com_vel_current_lpf_from_support_ / wn_;

    swing_foot_pos_error_from_support_ = swing_foot_pos_trajectory_from_support_ - swing_foot_transform_current_from_support_.translation();

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

        last_preview_param_update_time_ = current_time_;
        preview_update_time_ = current_time_;

        for (int i = 0; i < zmp_size_; i++)
        {
            //ref_zmp_(i, 0) = com_pos_init_from_support_(0);
            //ref_zmp_(i, 1) = com_pos_init_from_support_(1);
        }
    }

    swingfoot_force_control_converter_ = DyrosMath::cubic(walking_phase_, 0.8, 0.9, 0, 1, 0, 0);
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
        }
    }

    if (int(current_time_ * 1e4) % int(1e3) == 0)
    {
    }
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

void AvatarController::motionRetargeting_HQPIK()
{
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
        cout << "HQPIK time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << endl;

        for (int i = 0; i < hierarchy_num_hqpik_; i++)
        {
            cout << "iteration " << i << "-th time: " << std::chrono::duration_cast<std::chrono::microseconds>(t_end_hqpik[i] - t_start_hqpik[i]).count() << endl;
        }
    }

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
        }
        else
        {
            q_dot_hqpik2_[i].setZero();

            if (int(current_time_ * 10000) % 1000 == 0)
                std::cout << "Error hierarchy: " << i << std::endl;

            break;
        }
    }

    for (int i = 0; i < variable_size_hqpik2_; i++)
    {
        motion_q_dot_(12 + i) = q_dot_hqpik2_[last_solved_hierarchy_num_](i);
        motion_q_(12 + i) = motion_q_pre_(12 + i) + motion_q_dot_(12 + i) * dt_;
        pd_control_mask_(12 + i) = 1;
    }
}

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

    hmd_pelv_vel_.segment(0, 3) = (hmd_pelv_pose_.translation() - hmd_pelv_pose_pre_.translation()) / dt_;
    Eigen::AngleAxisd ang_temp(hmd_pelv_pose_.linear() * hmd_pelv_pose_pre_.linear().transpose());
    hmd_pelv_vel_.segment(3, 3) = ang_temp.axis() * ang_temp.angle() / dt_;

    bool fast_pelv_move = false;
    bool far_pelv_move = false;
    int maximum_data_cut_num = 200;
    double pelv_max_vel = 5;
    double pelv_pos_boundary = 0.3;

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
        hmd_still_cali_lhand_pos_ = hmd_lhand_pose_.translation();
        hmd_still_cali_rhand_pos_ = hmd_rhand_pose_.translation();

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
        hmd_tpose_cali_lhand_pos_ = hmd_lhand_pose_.translation();
        hmd_tpose_cali_rhand_pos_ = hmd_rhand_pose_.translation();

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
        hmd_forward_cali_lhand_pos_ = hmd_lhand_pose_.translation();
        hmd_forward_cali_rhand_pos_ = hmd_rhand_pose_.translation();

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
        //// Geometric Shoulder Calculation ///////////////////////////
        getCenterOfShoulderCali(hmd_still_cali_lhand_pos_, hmd_tpose_cali_lhand_pos_, hmd_forward_cali_lhand_pos_, hmd_lshoulder_center_pos_);
        getCenterOfShoulderCali(hmd_still_cali_rhand_pos_, hmd_tpose_cali_rhand_pos_, hmd_forward_cali_rhand_pos_, hmd_rshoulder_center_pos_);

        hmd_larm_max_l_ = 0;
        hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_still_cali_lhand_pos_).norm();
        hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_tpose_cali_lhand_pos_).norm();
        hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_forward_cali_lhand_pos_).norm();
        hmd_larm_max_l_ /= 3;

        hmd_rarm_max_l_ = 0;
        hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_still_cali_rhand_pos_).norm();
        hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_tpose_cali_rhand_pos_).norm();
        hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_forward_cali_rhand_pos_).norm();
        hmd_rarm_max_l_ /= 3;

        hmd_shoulder_width_ = (hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_).norm();

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

        lhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_lhand_;
        lhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_lhand_;
        lhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_lhand_;

        rhand_master_ref_stack_.block(0, 0, 3, 1) = hmd_still_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        rhand_master_ref_stack_.block(0, 1, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
        rhand_master_ref_stack_.block(0, 2, 3, 1) = hmd_forward_cali_rhand_pos_ - hmd_rshoulder_center_pos_;

        rhand_robot_ref_stack_.block(0, 0, 3, 1) = robot_still_pose_rhand_;
        rhand_robot_ref_stack_.block(0, 1, 3, 1) = robot_t_pose_rhand_;
        rhand_robot_ref_stack_.block(0, 2, 3, 1) = robot_forward_pose_rhand_;

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

        ub_retargeting_(3) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(0)), w_dot_max_);
        ub_retargeting_(4) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(1)), w_dot_max_);
        ub_retargeting_(5) = min(speed_reduce_rate * (1.0 - rhand_mapping_vector_pre_(2)), w_dot_max_);

        lb_retargeting_(0) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(0)), w_dot_min_);
        lb_retargeting_(1) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(1)), w_dot_min_);
        lb_retargeting_(2) = max(speed_reduce_rate * (-1.1 - lhand_mapping_vector_pre_(2)), w_dot_min_);

        lb_retargeting_(3) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(0)), w_dot_min_);
        lb_retargeting_(4) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(1)), w_dot_min_);
        lb_retargeting_(5) = max(speed_reduce_rate * (-1.1 - rhand_mapping_vector_pre_(2)), w_dot_min_);

        h_pre_lhand_ = lhand_master_ref_stack_ * lhand_mapping_vector_pre_;
        h_pre_rhand_ = rhand_master_ref_stack_ * rhand_mapping_vector_pre_;
        r_pre_lhand_ = lhand_robot_ref_stack_ * lhand_mapping_vector_pre_;
        r_pre_rhand_ = rhand_robot_ref_stack_ * rhand_mapping_vector_pre_;
    }

    double hand_d = (hmd_lhand_pose_.translation() - hmd_rhand_pose_.translation()).norm();
    double beta = 0;

    if (beta == 0)
    {
        qpRetargeting_1(); //calc lhand_mapping_vector_, rhand_mapping_vector_ //1025
    }
    else if (beta == 1)
    {
        qpRetargeting_21();
    }
    else //transition
    {
        qpRetargeting_1();
        qpRetargeting_21Transition(beta); // qpRetargeting_1() must be preceded
    }

    hmd2robot_lhand_pos_mapping_ = lhand_robot_ref_stack_ * lhand_mapping_vector_;
    hmd2robot_rhand_pos_mapping_ = rhand_robot_ref_stack_ * rhand_mapping_vector_;

    if (upper_body_mode_ == 10)
    {
        hmd2robot_lhand_pos_mapping_ *= hmd_larm_max_l_ / robot_arm_max_l_;
        hmd2robot_rhand_pos_mapping_ *= hmd_rarm_max_l_ / robot_arm_max_l_;
    }

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

    torque_g_.setZero();
    torque_g_ = WBC::GravityCompensationTorque(rd_);

    ////////////// Set f_start  ////////////////
    com_pos_error_ = com_pos_desired_ - com_pos_current_;
    com_vel_error_ = com_vel_desired_ - com_vel_current_;

    f_star(0) = kd_compos_(0, 0) * (com_vel_error_(0)) + kp_compos_(0, 0) * (com_pos_error_(0)) + com_acc_desired_(0); //X axis PD control
    f_star(1) = kd_compos_(1, 1) * (com_vel_error_(1)) + kp_compos_(1, 1) * (com_pos_error_(1)) + com_acc_desired_(1); //Y axis PD control
    f_star(2) = kd_compos_(2, 2) * (com_vel_error_(2)) + kp_compos_(2, 2) * (com_pos_error_(2)) + com_acc_desired_(2);

    f_star(0) *= rd_.link_[COM_id].mass; //cancle out mass effect
    f_star(1) *= rd_.link_[COM_id].mass;
    f_star(2) *= rd_.link_[COM_id].mass;

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

    torque += torque_g_;

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

    lleg_transform_target.linear().setIdentity();

    rleg_transform_target.linear().setIdentity();
    lleg_transform_target.translation() = lfoot_transform_current_from_global_.translation();
    rleg_transform_target.translation() = rfoot_transform_current_from_global_.translation();

    /////////////////////////////////PELVIS/////////////////////////////////////

    pelv_transform_from_global.translation().setZero();
    pelv_transform_from_global.linear() = pelv_rot_current_yaw_aline_; //

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

            Vector3d phi_support_ankle;
            // phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].rotm, pelv_yaw_rot_current_from_global_);
            phi_support_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), DyrosMath::rotateWithZ(current_q_(6)));

            desired_q_dot_(10) = 200 * bandBlock(phi_support_ankle(1), 15 * DEG2RAD, -15 * DEG2RAD); //(tune)
            desired_q_dot_(11) = 200 * bandBlock(phi_support_ankle(0), 10 * DEG2RAD, -10 * DEG2RAD); //(tune)
            desired_q_(10) = current_q_(10) + desired_q_dot_(10) * dt_;
            desired_q_(11) = current_q_(11) + desired_q_dot_(11) * dt_;

            desired_q_(6) = DyrosMath::QuinticSpline(turning_phase_, 0, 1, last_desired_q_(6), 0, 0, motion_q_(6), 0, 0)(0);
            desired_q_(9) = DyrosMath::QuinticSpline(walking_phase_, 0.0, 0.3, last_desired_q_(9), 0, 0, motion_q_(9), 0, 0)(0);

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

            swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

            if (first_step_trigger_ == true)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = swing_pd_switch;
                pd_control_mask_(2) = swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 0;
                pd_control_mask_(8) = 0;
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

            Vector3d phi_support_ankle;
            // phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].rotm, pelv_yaw_rot_current_from_global_);
            phi_support_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), DyrosMath::rotateWithZ(current_q_(0)));

            desired_q_dot_(4) = 200 * bandBlock(phi_support_ankle(1), 15 * DEG2RAD, -15 * DEG2RAD);
            ;
            desired_q_dot_(5) = 200 * bandBlock(phi_support_ankle(0), 10 * DEG2RAD, -10 * DEG2RAD);
            ;
            desired_q_(4) = current_q_(4) + desired_q_dot_(4) * dt_;
            desired_q_(5) = current_q_(5) + desired_q_dot_(5) * dt_;

            desired_q_(0) = DyrosMath::QuinticSpline(turning_phase_, 0, 1, last_desired_q_(0), 0, 0, motion_q_(0), 0, 0)(0); //left support foot hip yaw
            desired_q_(3) = DyrosMath::QuinticSpline(walking_phase_, 0.0, 0.3, last_desired_q_(3), 0, 0, motion_q_(3), 0, 0)(0);

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

            /////////////////Swing Assist Torque/////////////////////////////////////////
            // right foot swing height assist feed forward torque on hip roll, hip pitch, knee pitch
            Eigen::VectorXd f_star;
            f_star.setZero(3);
            f_star(2) = swing_foot_acc_trajectory_from_global_(2);
            /////////////////////////////////////////////////////////////////////////////

            swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

            if (first_step_trigger_ == true)
            {

                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 0;
                pd_control_mask_(2) = 0;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = swing_pd_switch;
                pd_control_mask_(8) = swing_pd_switch;
            }
            else
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 1 - swing_pd_switch;
                pd_control_mask_(2) = 1 - swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = swing_pd_switch;
                pd_control_mask_(8) = swing_pd_switch;
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

        desired_q_(0) = 0.5 * 0 + 0.5 * pre_desired_q_(0); // hip yaw
        desired_q_(6) = 0.5 * 0 + 0.5 * pre_desired_q_(6);

        desired_q_(3) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 3, last_desired_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
        desired_q_(9) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 3, last_desired_q_(9), 0, 0, motion_q_(9), 0, 0)(0);

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
            pd_control_mask_(6) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
            pd_control_mask_(7) = 0;
            pd_control_mask_(8) = 0;

            pd_control_mask_(0) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
            pd_control_mask_(1) = 0;
            pd_control_mask_(2) = 0;
        }
        else if (stop_walking_trigger_ == true)
        {
            if (foot_contact_ == 1)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 1 - swing_pd_switch;
                pd_control_mask_(2) = 1 - swing_pd_switch;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 0;
                pd_control_mask_(8) = 0;
            }
            else if (foot_contact_ == -1)
            {
                pd_control_mask_(0) = 1;
                pd_control_mask_(1) = 0;
                pd_control_mask_(2) = 0;

                pd_control_mask_(6) = 1;
                pd_control_mask_(7) = 1 - swing_pd_switch;
                pd_control_mask_(8) = 1 - swing_pd_switch;
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////LEG Q DOT///////////////////////////////////////////////
    desired_q_dot_.segment(0, 4) = (desired_q_.segment(0, 4) - pre_desired_q_.segment(0, 4)) / dt_; //left hip and knee
    desired_q_dot_.segment(6, 4) = (desired_q_.segment(6, 4) - pre_desired_q_.segment(6, 4)) / dt_; //left hip and knee
    if (walking_phase_ == 0)
    {
        desired_q_dot_.segment(0, 4).setZero();
        desired_q_dot_.segment(6, 4).setZero();
    }
    //////////////////////////////////////////////////////////////////////////////////////////
    for (int i = 12; i < 15; i++)
    {
        desired_q_(i) = motion_q_(i);
        desired_q_dot_(i) = motion_q_dot_(i);
    }
    //////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////ARM & HEAD/////////////////////////////////////////////////
    for (int i = 15; i < MODEL_DOF; i++)
    {
        desired_q_(i) = motion_q_(i);
        desired_q_dot_(i) = motion_q_dot_(i);
    }
    //////////////////////////////////////////////////////////////////////////////////////////

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

    pelv_transform_desired.linear() = pelv_rot_current_yaw_aline_;

    lfoot_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, lfoot_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());
    rfoot_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, rfoot_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());
    pelv_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, pelv_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());

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

void AvatarController::computeThread3()
{   
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    if(atb_main_to_mpc_update_ == false)
    {
        atb_main_to_mpc_update_ = true;

        walking_tick_mpc_                   = walking_tick_container_to_mpc_;
        current_step_num_mpc_               = current_step_num_container_to_mpc_;
        com_start_tick_mpc_                 = com_start_tick_container_to_mpc_;

        t_start_mpc_                        = t_start_container_to_mpc_;

        ref_zmp_wo_offset_mpc_              = ref_zmp_wo_offset_container_to_mpc_;
        ref_zmp_mpc_                        = ref_zmp_container_to_mpc_;

        ref_vrp_mpc_                        = ref_vrp_container_to_mpc_;

        dcm_measured_mpc_                   = dcm_measured_container_to_mpc_;
        com_measured_mpc_                   = com_measured_container_to_mpc_;
        com_dot_measured_mpc_               = com_dot_measured_container_to_mpc_;
            
        foot_step_support_frame_offset_mpc_ = foot_step_support_frame_offset_container_to_mpc_;
        foot_step_support_frame_mpc_        = foot_step_support_frame_container_to_mpc_;

        atb_main_to_mpc_update_ = false;
    }

    //double preview_time = 1.7;
    double preview_time = 0.9;
    //IS_FIPM_CoM_Planner_MPC(thread3_hz_, 1.0/thread3_hz_, preview_time, 2000/thread3_hz_);
    IS_FIPM_CoM_Seq_Planner_MPC(thread3_hz_, 1.0/thread3_hz_, preview_time, 2000/thread3_hz_);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    e_mpc_time_graph << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()*1e-6 << ",";
    //IS_FIPM_3D_DCM_Stabililzer_MPC(thread3_hz_, preview_time);
    IS_FIPM_3D_DCM_Seq_Stabilizer_MPC(thread3_hz_, preview_time);

    //send mpc data
    if(atb_mpc_to_main_update_ == false)
    {
        atb_mpc_to_main_update_ = true;

        step_enable_bool_container_from_mpc_               = step_enable_bool_mpc_;

        step_enable_bool_one_tick_container_from_mpc_      = step_enable_bool_one_tick_mpc_;

        MPC_Planner_state_container_from_mpc_              = MPC_Planner_state_mpc_;

        MPC_Planner_u_container_from_mpc_                  = MPC_Planner_u_mpc_sep_;

        current_step_num_container_from_mpc_               = current_step_num_mpc_;

        MPC_Stabilizer_state_container_from_mpc_           = MPC_Stabilizer_state_mpc_;

        MPC_Stabilizer_u_container_from_mpc_               = MPC_Stabilizer_u_mpc_sep_;

        MPC_Stabilizer_delf_container_from_mpc_            = MPC_Stabilizer_delf_mpc_;

        MPC_Stabilizer_time_adj_tick_x_container_from_mpc_ = MPC_Stabilizer_time_adj_tick_x_mpc_;
        MPC_Stabilizer_time_adj_tick_y_container_from_mpc_ = MPC_Stabilizer_time_adj_tick_y_mpc_;

        atb_mpc_to_main_update_ = false;
    }
    mpc_update_ = true;

    econom2_thread_stepchange();
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    e_mpc_time_graph << std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count()*1e-6 << endl;
}

void AvatarController::econom2_thread_stepchange()
{
    Eigen::Vector2d del_F_mpc; del_F_mpc.setZero();
    del_F_mpc(0) = MPC_Stabilizer_delf_mpc_x_.sum() - foot_step_support_frame_mpc_(current_step_num_mpc_, 0);
    del_F_mpc(1) = MPC_Stabilizer_delf_mpc_y_.sum() - foot_step_support_frame_mpc_(current_step_num_mpc_, 1);

    foot_step_support_frame_mpc_(current_step_num_mpc_, 0) += del_F_mpc(0);
    foot_step_support_frame_mpc_(current_step_num_mpc_, 1) += del_F_mpc(1);
    if((walking_tick_mpc_ - (t_start_mpc_ + t_total_mpc_) >= -hz_/thread3_hz_) && (current_step_num_mpc_ < total_step_num_ - 1))
    {
        Eigen::Vector3d var_after_step_change, var_before_step_change, frame_pos_diff;
        Eigen::Matrix3d frame_rot_diff;
        
        Eigen::VectorXd MPC_qcqp_sqp_gurobi_calc; MPC_qcqp_sqp_gurobi_calc.setZero(9);
        MPC_qcqp_sqp_gurobi_calc = MPC_Planner_state_mpc_;

        frame_rot_diff = DyrosMath::rotateWithZ(-foot_step_support_frame_mpc_(current_step_num_mpc_, 5));
        frame_pos_diff(0) = foot_step_support_frame_mpc_(current_step_num_mpc_,0);
        frame_pos_diff(1) = foot_step_support_frame_mpc_(current_step_num_mpc_,1);
        frame_pos_diff(2) = foot_step_support_frame_mpc_(current_step_num_mpc_,2);

        //com pos step change
        var_before_step_change(0) = MPC_qcqp_sqp_gurobi_calc(0);
        var_before_step_change(1) = MPC_qcqp_sqp_gurobi_calc(3);
        var_before_step_change(2) = MPC_qcqp_sqp_gurobi_calc(6);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);

        MPC_Planner_state_mpc_(0) = var_after_step_change(0);
        MPC_Planner_state_mpc_(3) = var_after_step_change(1);
        MPC_Planner_state_mpc_(6) = var_after_step_change(2);

        //com vel step change
        var_before_step_change(0) = MPC_qcqp_sqp_gurobi_calc(1);
        var_before_step_change(1) = MPC_qcqp_sqp_gurobi_calc(4);
        var_before_step_change(2) = MPC_qcqp_sqp_gurobi_calc(7);
        var_after_step_change = frame_rot_diff*var_before_step_change;

        MPC_Planner_state_mpc_(1) = var_after_step_change(0);
        MPC_Planner_state_mpc_(4) = var_after_step_change(1);
        MPC_Planner_state_mpc_(7) = var_after_step_change(2);

        //vrp pos step change
        var_before_step_change(0) = MPC_qcqp_sqp_gurobi_calc(2);
        var_before_step_change(1) = MPC_qcqp_sqp_gurobi_calc(5);
        var_before_step_change(2) = MPC_qcqp_sqp_gurobi_calc(8);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);

        MPC_Planner_state_mpc_(2) = var_after_step_change(0);
        MPC_Planner_state_mpc_(5) = var_after_step_change(1);
        MPC_Planner_state_mpc_(8) = var_after_step_change(2);


        //com pos step change
        var_before_step_change(0) = MPC_Stabilizer_state_mpc_(0);
        var_before_step_change(1) = MPC_Stabilizer_state_mpc_(3);
        var_before_step_change(2) = MPC_Stabilizer_state_mpc_(6);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);

        MPC_Stabilizer_state_mpc_(0) = var_after_step_change(0);
        MPC_Stabilizer_state_mpc_(3) = var_after_step_change(1);
        MPC_Stabilizer_state_mpc_(6) = var_after_step_change(2);

        //com vel step change
        var_before_step_change(0) = MPC_Stabilizer_state_mpc_(1);
        var_before_step_change(1) = MPC_Stabilizer_state_mpc_(4);
        var_before_step_change(2) = MPC_Stabilizer_state_mpc_(7);
        var_after_step_change = frame_rot_diff*var_before_step_change;

        MPC_Stabilizer_state_mpc_(1) = var_after_step_change(0);
        MPC_Stabilizer_state_mpc_(4) = var_after_step_change(1);
        MPC_Stabilizer_state_mpc_(7) = var_after_step_change(2);

        //vrp pos step change
        var_before_step_change(0) = MPC_Stabilizer_state_mpc_(2);
        var_before_step_change(1) = MPC_Stabilizer_state_mpc_(5);
        var_before_step_change(2) = MPC_Stabilizer_state_mpc_(8);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);

        MPC_Stabilizer_state_mpc_(2) = var_after_step_change(0);
        MPC_Stabilizer_state_mpc_(5) = var_after_step_change(1);
        MPC_Stabilizer_state_mpc_(8) = var_after_step_change(2);
    }
    foot_step_support_frame_mpc_(current_step_num_mpc_, 0) -= del_F_mpc(0);
    foot_step_support_frame_mpc_(current_step_num_mpc_, 1) -= del_F_mpc(1);
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
// void AvatarController::OptoforceFTCallback(const tocabi_msgs::FTsensor &msg)
// {
//     opto_ft_raw_(0) = msg.Fx;
//     opto_ft_raw_(1) = msg.Fy;
//     opto_ft_raw_(2) = msg.Fz;
//     opto_ft_raw_(3) = msg.Tx;
//     opto_ft_raw_(4) = msg.Ty;
//     opto_ft_raw_(5) = msg.Tz;
// }

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
    if (walking_tick_ == 0)
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
    else if (current_step_num_ != 0 && walking_tick_ == t_start_) // step change
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
    //lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

    rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
    //rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

    com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치
    com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].v);
    com_float_current_ddot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.imu_lin_acc);

    if (walking_tick_ == 0)
    {
        com_float_current_dot_LPF = com_float_current_dot;
        com_float_current_dot_prev = com_float_current_dot;

        com_float_current_ddot_LPF = com_float_current_ddot;
        com_float_current_ddot_prev = com_float_current_ddot;
    }

    com_float_current_dot_prev = com_float_current_dot;
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot;

    com_float_current_ddot_prev = com_float_current_ddot;
    com_float_current_ddot_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_ddot_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_ddot;
    
    // modified cut off freq. of CP error for joe's MPC 
    if (walking_tick_ == 0)
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
    //supportfoot_float_current_yaw_only.linear() = supportfoot_float_current_.linear();

    pelv_support_current_  = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * pelv_float_current_;
    lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * lfoot_float_current_;
    lfoot_support_current_calc_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * lfoot_float_current_;
    rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * rfoot_float_current_;
    rfoot_support_current_calc_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_) * rfoot_float_current_;
    ////////////////////

    com_support_current_        = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_    = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);

    com_support_current_ddot_    = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_ddot);
    com_support_current_ddot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_ddot_LPF);

    lfoot_support_current_dot_.segment(0, 3) = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), rd_.link_[Left_Foot].v);
    lfoot_support_current_dot_.segment(3, 3) = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), rd_.link_[Left_Foot].w);
    rfoot_support_current_dot_.segment(0, 3) = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), rd_.link_[Right_Foot].v);
    rfoot_support_current_dot_.segment(3, 3) = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), rd_.link_[Right_Foot].w);

    getVirtualJointState(pelv_yaw_rot_current_from_global_mj_, supportfoot_float_current_yaw_only);

    if(walking_tick_ == 0)
    {
        q_desired_virtual_ = q_virtual_;
        qdot_desired_virtual_ = qdot_virtual_;
        qddot_desired_virtual_ = qddot_virtual_;
    }

    SC_err_compen(com_support_current_(0), com_support_current_(1));

    b_ = sqrt(1.00*zc_mj_/GRAVITY);
    w_ = 1.0/b_;

    cp_measured_(0) = com_support_cp_(0)      + com_float_current_dot_LPF(0) / w_;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / w_;

    dcm_measured_(0) = com_support_current_(0) + b_*com_float_current_dot_LPF(0);
    dcm_measured_(1) = com_support_current_(1) + b_*com_float_current_dot_LPF(1);
    dcm_measured_(2) = com_support_current_(2) + b_*com_float_current_dot_LPF(2);

    com_measured_(0) = com_support_current_(0);
    com_measured_(1) = com_support_current_(1);
    com_measured_(2) = com_support_current_(2);

    com_dot_measured_(0) = com_float_current_dot_LPF(0);
    com_dot_measured_(1) = com_float_current_dot_LPF(1);
    com_dot_measured_(2) = com_float_current_dot_LPF(2);
    
    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    if (walking_tick_ == 0)
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
    
    if (walking_tick_ == 0) { zmp_measured_LPF_.setZero(); }
    zmp_measured_LPF_ = (2 * M_PI * 2.0 * del_t) / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_LPF_;
    if (walking_tick_ == t_start_) { zmp_measured_LPF_ = zmp_measured_mj_; }

    Eigen::VectorVQd init_ref_q; init_ref_q.setZero();
    init_ref_q.segment(6, MODEL_DOF) = rd_.q_;

    lhand_trajectory_float_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_MJ_, init_ref_q, rd_.link_[Left_Hand].id, lhand_control_point_offset_, true);
    lhand_trajectory_float_.linear()      = RigidBodyDynamics::CalcBodyWorldOrientation( model_MJ_, init_ref_q, rd_.link_[Left_Hand].id, true).transpose();

    rhand_trajectory_float_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_MJ_, init_ref_q, rd_.link_[Right_Hand].id, rhand_control_point_offset_, true);
    rhand_trajectory_float_.linear()      = RigidBodyDynamics::CalcBodyWorldOrientation( model_MJ_, init_ref_q, rd_.link_[Right_Hand].id, true).transpose();
        
    chest_trajectory_float_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_MJ_, init_ref_q, rd_.link_[Upper_Body].id, rhand_control_point_offset_, true);
    chest_trajectory_float_.linear()      = RigidBodyDynamics::CalcBodyWorldOrientation( model_MJ_, init_ref_q, rd_.link_[Upper_Body].id, true).transpose();
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
        //middle_total_step_number = 10; //total foot step number
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
    cout << index << endl;
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
    //simulation gains
    if(param_sim_mode_)
    {
        Kp(0) = 1800.0;
        Kd(0) = 70.0; // Left Hip yaw
        Kp(1) = 2100.0;
        Kd(1) = 90.0; // Left Hip roll
        Kp(2) = 2100.0;
        Kd(2) = 90.0; // Left Hip pitch
        Kp(3) = 2100.0;
        Kd(3) = 90.0; // Left Knee pitch
        Kp(4) = 2100.0;
        Kd(4) = 90.0; // Left Ankle pitch
        //Kp(5) = 2100.0;
        //Kd(5) = 90.0; // Left Ankle roll
        Kp(5) = 4000.0;
        Kd(5) = 65.0; // Left Ankle roll

        Kp(6) = 1800.0;
        Kd(6) = 70.0; // Right Hip yaw
        Kp(7) = 2100.0;
        Kd(7) = 90.0; // Right Hip roll
        Kp(8) = 2100.0;
        Kd(8) = 90.0; // Right Hip pitch
        Kp(9) = 2100.0;
        Kd(9) = 90.0; // Right Knee pitch
        Kp(10) = 2100.0;
        Kd(10) = 90.0; // Right Ankle pitch
        //Kp(11) = 2100.0;
        //Kd(11) = 90.0; // Right Ankle roll
        Kp(11) = 4000.0;
        Kd(11) = 65.0; // Right Ankle roll

        Kp(12) = 2200.0;
        Kd(12) = 90.0; // Waist yaw
        Kp(13) = 2200.0;
        Kd(13) = 90.0; // Waist pitch
        Kp(14) = 2200.0;
        Kd(14) = 90.0; // Waist roll

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

        cout << "simulation joint gain set" << endl;
    }
    else
    {
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

        cout << "experiment joint gain set" << endl;
    }

    Kp_virtual_(0) = 100;
    Kd_virtual_(0) =  20;
    Kp_virtual_(1) = 100;
    Kd_virtual_(1) =  20;
    Kp_virtual_(2) = 100;
    Kd_virtual_(2) =  20;
    Kp_virtual_(3) = 100;
    Kd_virtual_(3) =  20;
    Kp_virtual_(4) = 100;
    Kd_virtual_(4) =  20;
    Kp_virtual_(5) = 100;
    Kd_virtual_(5) =  20;

    Kp_virtual_.segment(6, MODEL_DOF) = Kp;
    Kd_virtual_.segment(6, MODEL_DOF) = Kd;
}

void AvatarController::addZmpOffset()
{
    double lfoot_zmp_offset_, rfoot_zmp_offset_;

    //lfoot_zmp_offset_ = -0.02;
    //rfoot_zmp_offset_ =  0.02;
    
    lfoot_zmp_offset_ = -(0.03 + 0.005*(1 - (bool)current_step_num_)); //0.02 for preview
    rfoot_zmp_offset_ =  (0.03 + 0.005*(1 - (bool)current_step_num_)); //0.02 for preview
    
    foot_step_support_frame_offset_ = foot_step_support_frame_;

    supportfoot_support_init_offset_ = supportfoot_support_init_;

    if (foot_step_(0, 6) == 0) //right support foot
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    }
    else
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_; 
    }

    for (int i = 0; i < total_step_num_; i++)
    //for (int i = current_step_num_; i < min(total_step_num_, current_step_num_ + 3); i++)
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
    unsigned int planning_step_number = 3;
    unsigned int planning_zmp_size = 0;

    if (current_step_num_ >= total_step_num_ - planning_step_number)
    { planning_zmp_size = t_total_ + t_total_const_ * (total_step_num_ - current_step_num_ - 1) + 4.0 * hz_; }
    else
    { planning_zmp_size = t_total_ + t_total_const_ * (planning_step_number - 1) + 1.0 * hz_; }

    if (current_step_num_ == 0)
    { planning_zmp_size = planning_zmp_size + t_temp_ + 1; }

    addZmpOffset(); 

    if(walking_tick_ - t_start_ < t_dsp1_const_)
    {
        foot_step_support_frame_.       block(max(current_step_num_ - 2, 0),               0, 1, 2) -= del_F_prev_.transpose();
        foot_step_support_frame_offset_.block(max(current_step_num_ - 2, 0),               0, 1, 2) -= del_F_prev_.transpose();
    }
    foot_step_support_frame_.block(current_step_num_,                                      0, 1, 2) += del_F_.transpose();
    foot_step_support_frame_.block(min(current_step_num_ + 1, total_step_num_ - 1),        0, 1, 2) += del_F_.transpose();
    foot_step_support_frame_.block(min(current_step_num_ + 2, total_step_num_ - 1),        0, 1, 2) += del_F_.transpose();
    foot_step_support_frame_offset_.block(current_step_num_,                               0, 1, 2) += del_F_.transpose();
    foot_step_support_frame_offset_.block(min(current_step_num_ + 1, total_step_num_ - 1), 0, 1, 2) += del_F_.transpose();
    foot_step_support_frame_offset_.block(min(current_step_num_ + 2, total_step_num_ - 1), 0, 1, 2) += del_F_.transpose();

    zmpGenerator(planning_zmp_size, planning_step_number);

    if(walking_tick_ - t_start_ < t_dsp1_const_)
    {
        foot_step_support_frame_.       block(max(current_step_num_ - 2, 0),               0, 1, 2) += del_F_prev_.transpose();
        foot_step_support_frame_offset_.block(max(current_step_num_ - 2, 0),               0, 1, 2) += del_F_prev_.transpose();
    }
    foot_step_support_frame_.block(current_step_num_,                                      0, 1, 2) -= del_F_.transpose(); 
    foot_step_support_frame_.block(min(current_step_num_ + 1, total_step_num_ - 1),        0, 1, 2) -= del_F_.transpose();
    foot_step_support_frame_.block(min(current_step_num_ + 2, total_step_num_ - 1),        0, 1, 2) -= del_F_.transpose();
    foot_step_support_frame_offset_.block(current_step_num_,                               0, 1, 2) -= del_F_.transpose();
    foot_step_support_frame_offset_.block(min(current_step_num_ + 1, total_step_num_ - 1), 0, 1, 2) -= del_F_.transpose();
    foot_step_support_frame_offset_.block(min(current_step_num_ + 2, total_step_num_ - 1), 0, 1, 2) -= del_F_.transpose();

    zmp_desired_(0) = ref_zmp_(walking_tick_ - (bool)current_step_num_*t_start_,0);
    zmp_desired_(1) = ref_zmp_(walking_tick_ - (bool)current_step_num_*t_start_,1);

    ref_zmp_wo_offset_mpc_.resize(planning_zmp_size, 2);
    ref_zmp_mpc_.resize(planning_zmp_size, 2);
    ref_zmp_container_to_mpc_.resize(planning_zmp_size,2);

    ref_vrp_mpc_.resize(planning_zmp_size, 3);
    ref_vrp_container_to_mpc_.resize(planning_zmp_size, 3);
}

void AvatarController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
    ref_zmp_.resize(norm_size, 2); ref_zmp_.setZero();
    ref_zmp_wo_offset_.setZero(norm_size, 2);
    ref_vrp_.setZero(norm_size, 3);

    Eigen::VectorXd temp_px,           temp_py,           temp_pz;
    Eigen::VectorXd temp_px_wo_offset, temp_py_wo_offset;

    unsigned int index = 0;
    double t_total_zmp = t_total_const_;

    if (current_step_num_ == 0)  
    {
        ref_zmp_.block(0, 0, 1.0*hz_, 1).setConstant(com_support_init_(0));
        ref_zmp_.block(0, 1, 1.0*hz_, 1).setConstant(com_support_init_(1));
        
        ref_vrp_.block(0, 0, 1.0*hz_, 2) = ref_zmp_.block(0, 0, 1.0*hz_, 2);
        ref_vrp_.block(0, 2, 1.0*hz_, 1).setConstant(zc_mj_);

        ref_zmp_wo_offset_.block(0, 0, 1.0*hz_, 1).setConstant(com_support_init_(0));
        ref_zmp_wo_offset_.block(0, 1, 1.0*hz_, 1).setConstant(com_support_init_(1));

        index = index + 1.0*hz_;

        for (int i = 1.0*hz_; i < 2.0*hz_; i++)
        {
            double del_x = i - 1.0 * hz_;

            ref_zmp_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (1.0 * hz_);
            ref_zmp_(i, 1) = com_support_init_(1);
            
            ref_vrp_(i, 0) = ref_zmp_(i, 0);
            ref_vrp_(i, 1) = ref_zmp_(i, 1);
            ref_vrp_(i, 2) = zc_mj_ - 0.10*param_scenario_;

            ref_zmp_wo_offset_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (1.0 * hz_);
            ref_zmp_wo_offset_(i, 1) = com_support_init_(1);

            index++;
        }

        ref_zmp_.block(2.0*hz_, 0, t_temp_ - 2.0*hz_, 1).setConstant(0.0);
        ref_zmp_.block(2.0*hz_, 1, t_temp_ - 2.0*hz_, 1).setConstant(com_support_init_(1));
        
        ref_vrp_.block(2.0*hz_, 0, t_temp_ - 2.0*hz_, 2) = ref_zmp_.block(2.0*hz_, 0, t_temp_ - 2.0*hz_, 2);
        ref_vrp_.block(2.0*hz_, 2, t_temp_ - 2.0*hz_, 1).setConstant(zc_mj_ - 0.10*param_scenario_);

        ref_zmp_wo_offset_.block(2.0*hz_, 0, t_temp_ - 2.0*hz_, 1).setConstant(0.0);
        ref_zmp_wo_offset_.block(2.0*hz_, 1, t_temp_ - 2.0*hz_, 1).setConstant(com_support_init_(1));

        index = index + t_temp_ - 2.0*hz_;
    }
    /////////////////////////////////////////////////////////////////////.

    if(current_step_num_ >= total_step_num_ - planning_step_num)
    {   
        for(unsigned int i = current_step_num_; i < total_step_num_; i++)
        {   
            if(i == current_step_num_)
            {
                t_total_zmp = t_total_;
            }
            else
            {
                t_total_zmp = t_total_const_;
            }

            onestepZmp_wo_offset(i, t_total_zmp, temp_px, temp_py, temp_px_wo_offset, temp_py_wo_offset);
            onestepVrpZ(i, t_total_zmp, temp_pz);
            
            ref_zmp_.block(index, 0, t_total_zmp, 1) = temp_px.block(0, 0, t_total_zmp, 1);
            ref_zmp_.block(index, 1, t_total_zmp, 1) = temp_py.block(0, 0, t_total_zmp, 1);

            ref_vrp_.block(index, 0, t_total_zmp, 2) = ref_zmp_.block(index, 0, t_total_zmp, 2);
            ref_vrp_.block(index, 2, t_total_zmp, 1) = temp_pz.block (    0, 0, t_total_zmp, 1);

            ref_zmp_wo_offset_.block(index, 0, t_total_zmp, 1) = temp_px_wo_offset.block(0, 0, t_total_zmp, 1);
            ref_zmp_wo_offset_.block(index, 1, t_total_zmp, 1) = temp_py_wo_offset.block(0, 0, t_total_zmp, 1);

            index = index + t_total_zmp;
        }

        ref_zmp_.block(index, 0, 3.0 * hz_, 1).setConstant(ref_zmp_(index - 1,0));
        ref_zmp_.block(index, 1, 3.0 * hz_, 1).setConstant(ref_zmp_(index - 1,1));

        ref_vrp_.block(index, 0, 3.0*hz_, 2) = ref_zmp_.block(index, 0, 3.0*hz_, 2);
        ref_vrp_.block(index, 2, 3.0*hz_, 1).setConstant(ref_vrp_(index - 1, 2));

        ref_zmp_wo_offset_.block(index, 0, 3.0 * hz_, 1).setConstant(ref_zmp_wo_offset_(index - 1,0));
        ref_zmp_wo_offset_.block(index, 1, 3.0 * hz_, 1).setConstant(ref_zmp_wo_offset_(index - 1,1));

        index = index + 3.0 * hz_; // Norm size must be larger than this addtional zmp size.
    }
    else // reference ZMP during walking
    {       
        for (unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)
        {   
            if(i == current_step_num_)
            {
                t_total_zmp = t_total_;
            }
            else
            {
                t_total_zmp = t_total_const_;
            }

            onestepZmp_wo_offset(i, t_total_zmp, temp_px, temp_py, temp_px_wo_offset, temp_py_wo_offset); // temp px, py에 1 step의 ZMP를 planning step num 번 담는다.
            onestepVrpZ(i, t_total_zmp, temp_pz);

            ref_zmp_.block(index, 0, t_total_zmp, 1) = temp_px.block(0, 0, t_total_zmp, 1);
            ref_zmp_.block(index, 1, t_total_zmp, 1) = temp_py.block(0, 0, t_total_zmp, 1);
            
            ref_vrp_.block(index, 0, t_total_zmp, 2) = ref_zmp_.block(index, 0, t_total_zmp, 2);
            ref_vrp_.block(index, 2, t_total_zmp, 1) = temp_pz.block(     0, 0, t_total_zmp, 1);

            ref_zmp_wo_offset_.block(index, 0, t_total_zmp, 1) = temp_px_wo_offset.block(0, 0, t_total_zmp, 1);
            ref_zmp_wo_offset_.block(index, 1, t_total_zmp, 1) = temp_py_wo_offset.block(0, 0, t_total_zmp, 1);

            index = index + t_total_zmp;                                       
        }
    }
}

void AvatarController::onestepZmp_wo_offset(unsigned int current_step_number, double t_total_zmp, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py, Eigen::VectorXd &temp_px_wo_offset, Eigen::VectorXd &temp_py_wo_offset)
{
    //resize the varibales to save the calculated zmp trajectory.
    temp_px.setZero(t_total_zmp);           temp_py.setZero(t_total_zmp);
    temp_px_wo_offset.setZero(t_total_zmp); temp_py_wo_offset.setZero(t_total_zmp);
    
    //variables for the start, and end point for each phase. 
    //detailed explanations are given below.
    double v0_x_dsp1_wo_offset = 0.0; double v0_y_dsp1_wo_offset = 0.0; double v0_x_dsp1_offset = 0.0; double v0_y_dsp1_offset = 0.0;
    double vT_x_dsp1_wo_offset = 0.0; double vT_y_dsp1_wo_offset = 0.0; double vT_x_dsp1_offset = 0.0; double vT_y_dsp1_offset = 0.0;
    double v0_x_ssp_wo_offset  = 0.0; double v0_y_ssp_wo_offset  = 0.0; double v0_x_ssp_offset  = 0.0; double v0_y_ssp_offset  = 0.0;
    double vT_x_ssp_wo_offset  = 0.0; double vT_y_ssp_wo_offset  = 0.0; double vT_x_ssp_offset  = 0.0; double vT_y_ssp_offset  = 0.0;
    double v0_x_dsp2_wo_offset = 0.0; double v0_y_dsp2_wo_offset = 0.0; double v0_x_dsp2_offset = 0.0; double v0_y_dsp2_offset = 0.0;
    double vT_x_dsp2_wo_offset = 0.0; double vT_y_dsp2_wo_offset = 0.0; double vT_x_dsp2_offset = 0.0; double vT_y_dsp2_offset = 0.0;

    double t_ssp_calc  = t_total_const_ - t_dsp1_ - t_dsp2_;

    double dsp1_mid_point_calc = t_dsp1_/(t_dsp1_ + t_dsp2_);
    double dsp2_mid_point_calc = t_dsp2_/(t_dsp1_ + t_dsp2_);
    
    if(current_step_number == current_step_num_)
    {
        t_ssp_calc  = t_total_ - t_dsp1_ - t_dsp2_;
    }

    if (current_step_number == 0)
    {
        v0_x_dsp1_wo_offset = 0.0;
        vT_x_dsp1_wo_offset = 0.0;
        v0_y_dsp1_wo_offset = com_support_init_(1);
        vT_y_dsp1_wo_offset = supportfoot_support_init_(1);

        v0_x_dsp1_offset    = 0.0;
        vT_x_dsp1_offset    = 0.0;
        v0_y_dsp1_offset    = com_support_init_(1);
        vT_y_dsp1_offset    = supportfoot_support_init_offset_(1);

        v0_x_ssp_wo_offset  = 0.0;
        vT_x_ssp_wo_offset  = 0.0;
        v0_y_ssp_wo_offset  = supportfoot_support_init_(1);
        vT_y_ssp_wo_offset  = supportfoot_support_init_(1);

        v0_x_ssp_offset     = 0.0;
        vT_x_ssp_offset     = 0.0;
        v0_y_ssp_offset     = supportfoot_support_init_offset_(1);
        vT_y_ssp_offset     = supportfoot_support_init_offset_(1);

        v0_x_dsp2_wo_offset = 0.0;
        vT_x_dsp2_wo_offset = supportfoot_support_init_(0)
                            + (foot_step_support_frame_(current_step_number - 0, 0) - supportfoot_support_init_(0)) * dsp2_mid_point_calc;
        v0_y_dsp2_wo_offset = supportfoot_support_init_(1);
        vT_y_dsp2_wo_offset = supportfoot_support_init_(1)
                            + (foot_step_support_frame_(current_step_number - 0, 1) - supportfoot_support_init_(1)) * dsp2_mid_point_calc;

        v0_x_dsp2_offset    = 0.0;
        vT_x_dsp2_offset    = supportfoot_support_init_offset_(0)
                            + (foot_step_support_frame_offset_(current_step_number - 0, 0) - supportfoot_support_init_offset_(0)) * dsp2_mid_point_calc;
        v0_y_dsp2_offset    = supportfoot_support_init_offset_(1);
        vT_y_dsp2_offset    = supportfoot_support_init_offset_(1)
                            + (foot_step_support_frame_offset_(current_step_number - 0, 1) - supportfoot_support_init_offset_(1)) * dsp2_mid_point_calc;
    }
    else if (current_step_number == 1)
    { 
        v0_x_dsp1_wo_offset = supportfoot_support_init_(0)
                            + (foot_step_support_frame_(current_step_number - 1, 0) - supportfoot_support_init_(0)) * dsp2_mid_point_calc;
        vT_x_dsp1_wo_offset =  foot_step_support_frame_(current_step_number - 1, 0);
        v0_y_dsp1_wo_offset = supportfoot_support_init_(1)
                            + (foot_step_support_frame_(current_step_number - 1, 1) - supportfoot_support_init_(1)) * dsp2_mid_point_calc;
        vT_y_dsp1_wo_offset = foot_step_support_frame_(current_step_number - 1, 1);

        v0_x_dsp1_offset    = supportfoot_support_init_offset_(0)
                            + (foot_step_support_frame_offset_(current_step_number - 1, 0) - supportfoot_support_init_offset_(0)) * dsp2_mid_point_calc;
        vT_x_dsp1_offset    =  foot_step_support_frame_offset_(current_step_number - 1, 0);
        v0_y_dsp1_offset    = supportfoot_support_init_offset_(1)
                            + (foot_step_support_frame_offset_(current_step_number - 1, 1) - supportfoot_support_init_offset_(1)) * dsp2_mid_point_calc;
        vT_y_dsp1_offset    = foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 0);
        vT_x_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 0);
        v0_y_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 1);
        vT_y_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 1);

        v0_x_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 0);
        v0_y_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_dsp2_wo_offset =  foot_step_support_frame_(current_step_number - 1, 0);
        vT_x_dsp2_wo_offset =  foot_step_support_frame_(current_step_number - 1, 0)
                            + (foot_step_support_frame_(current_step_number, 0) - foot_step_support_frame_(current_step_number - 1, 0)) * dsp2_mid_point_calc;
        v0_y_dsp2_wo_offset = foot_step_support_frame_(current_step_number - 1, 1);
        vT_y_dsp2_wo_offset = foot_step_support_frame_(current_step_number - 1, 1)
                            + (foot_step_support_frame_(current_step_number, 1) - foot_step_support_frame_(current_step_number - 1, 1)) * dsp2_mid_point_calc;

        v0_x_dsp2_offset    =  foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_dsp2_offset    = foot_step_support_frame_offset_(current_step_number - 1, 0)
                            + (foot_step_support_frame_offset_(current_step_number, 0) - foot_step_support_frame_offset_(current_step_number - 1, 0)) * dsp2_mid_point_calc;
        v0_y_dsp2_offset    = foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_dsp2_offset    = foot_step_support_frame_offset_(current_step_number - 1, 1)
                            + (foot_step_support_frame_offset_(current_step_number, 1) - foot_step_support_frame_offset_(current_step_number - 1, 1)) * dsp2_mid_point_calc;
    }
    else
    {   
        //v0_x_dsp1_wo_offset = foot_step_support_frame_(current_step_number - 2, 0)
        //                    + (foot_step_support_frame_(current_step_number - 1, 0) - foot_step_support_frame_(current_step_number - 2, 0)) * dsp2_mid_point_calc;
        v0_x_dsp1_wo_offset = foot_step_support_frame_(current_step_number - 1, 0)
                            - (foot_step_support_frame_(current_step_number - 1, 0) - foot_step_support_frame_(current_step_number - 2, 0)) * dsp1_mid_point_calc;
        vT_x_dsp1_wo_offset =  foot_step_support_frame_(current_step_number - 1, 0);
        //v0_y_dsp1_wo_offset = foot_step_support_frame_(current_step_number - 2, 1)
        //                    + (foot_step_support_frame_(current_step_number - 1, 1) - foot_step_support_frame_(current_step_number - 2, 1)) * dsp2_mid_point_calc;
        v0_y_dsp1_wo_offset = foot_step_support_frame_(current_step_number - 1, 1)
                            - (foot_step_support_frame_(current_step_number - 1, 1) - foot_step_support_frame_(current_step_number - 2, 1)) * dsp1_mid_point_calc;
        vT_y_dsp1_wo_offset = foot_step_support_frame_(current_step_number - 1, 1);

        //v0_x_dsp1_offset    = foot_step_support_frame_offset_(current_step_number - 2, 0)
        //                    + (foot_step_support_frame_offset_(current_step_number - 1, 0) - foot_step_support_frame_offset_(current_step_number - 2, 0)) * dsp2_mid_point_calc;
        v0_x_dsp1_offset    = foot_step_support_frame_offset_(current_step_number - 1, 0)
                            - (foot_step_support_frame_offset_(current_step_number - 1, 0) - foot_step_support_frame_offset_(current_step_number - 2, 0)) * dsp1_mid_point_calc;
        vT_x_dsp1_offset    =  foot_step_support_frame_offset_(current_step_number - 1, 0);
        //v0_y_dsp1_offset    = foot_step_support_frame_offset_(current_step_number - 2, 1)
        //                    + (foot_step_support_frame_offset_(current_step_number - 1, 1) - foot_step_support_frame_offset_(current_step_number - 2, 1)) * dsp2_mid_point_calc;
        v0_y_dsp1_offset    = foot_step_support_frame_offset_(current_step_number - 1, 1)
                            - (foot_step_support_frame_offset_(current_step_number - 1, 1) - foot_step_support_frame_offset_(current_step_number - 2, 1)) * dsp1_mid_point_calc;
        vT_y_dsp1_offset    = foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 0);
        vT_x_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 0);
        v0_y_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 1);
        vT_y_ssp_wo_offset  = foot_step_support_frame_(current_step_number - 1, 1);

        v0_x_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 0);
        v0_y_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_ssp_offset     = foot_step_support_frame_offset_(current_step_number - 1, 1);

        v0_x_dsp2_wo_offset = foot_step_support_frame_(current_step_number - 1, 0);
        vT_x_dsp2_wo_offset = foot_step_support_frame_(current_step_number - 1, 0)
                            + (foot_step_support_frame_(current_step_number - 0, 0) - foot_step_support_frame_(current_step_number - 1, 0)) * dsp2_mid_point_calc;
        v0_y_dsp2_wo_offset = foot_step_support_frame_(current_step_number - 1, 1);
        vT_y_dsp2_wo_offset = foot_step_support_frame_(current_step_number - 1, 1)
                            + (foot_step_support_frame_(current_step_number - 0, 1) - foot_step_support_frame_(current_step_number - 1, 1)) * dsp2_mid_point_calc;

        v0_x_dsp2_offset    = foot_step_support_frame_offset_(current_step_number - 1, 0);
        vT_x_dsp2_offset    = foot_step_support_frame_offset_(current_step_number - 1, 0)
                            + (foot_step_support_frame_offset_(current_step_number - 0, 0) - foot_step_support_frame_offset_(current_step_number - 1, 0)) * dsp2_mid_point_calc;
        v0_y_dsp2_offset    = foot_step_support_frame_offset_(current_step_number - 1, 1);
        vT_y_dsp2_offset    = foot_step_support_frame_offset_(current_step_number - 1, 1)
                            + (foot_step_support_frame_offset_(current_step_number - 0, 1) - foot_step_support_frame_offset_(current_step_number - 1, 1)) * dsp2_mid_point_calc;
    }

    Eigen::VectorXd zmp_plan_lin_calc;

    zmp_plan_lin_calc.setLinSpaced(t_dsp1_, v0_x_dsp1_wo_offset, vT_x_dsp1_wo_offset);
    temp_px_wo_offset.segment(0,   t_dsp1_) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_dsp1_, v0_x_dsp1_offset,    vT_x_dsp1_offset);
    temp_px.segment(0, t_dsp1_) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_dsp1_, v0_y_dsp1_wo_offset, vT_y_dsp1_wo_offset);
    temp_py_wo_offset.segment(0,   t_dsp1_) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_dsp1_, v0_y_dsp1_offset,    vT_y_dsp1_offset);
    temp_py.segment(0, t_dsp1_) = zmp_plan_lin_calc;

    zmp_plan_lin_calc.setLinSpaced(t_ssp_calc, v0_x_ssp_wo_offset, vT_x_ssp_wo_offset);
    temp_px_wo_offset.segment(t_dsp1_, t_ssp_calc) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_ssp_calc, v0_x_ssp_offset,    vT_x_ssp_offset);
    temp_px.segment(t_dsp1_, t_ssp_calc) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_ssp_calc, v0_y_ssp_wo_offset, vT_y_ssp_wo_offset);
    temp_py_wo_offset.segment(t_dsp1_, t_ssp_calc) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_ssp_calc, v0_y_ssp_offset,    vT_y_ssp_offset);
    temp_py.segment(t_dsp1_, t_ssp_calc) = zmp_plan_lin_calc;

    zmp_plan_lin_calc.setLinSpaced(t_dsp2_, v0_x_dsp2_wo_offset, vT_x_dsp2_wo_offset);
    temp_px_wo_offset.segment(t_dsp1_ + t_ssp_calc, t_dsp2_) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_dsp2_, v0_x_dsp2_offset,    vT_x_dsp2_offset);
    temp_px.segment(t_dsp1_ + t_ssp_calc, t_dsp2_) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_dsp2_, v0_y_dsp2_wo_offset, vT_y_dsp2_wo_offset);
    temp_py_wo_offset.segment(t_dsp1_ + t_ssp_calc, t_dsp2_) = zmp_plan_lin_calc;
    zmp_plan_lin_calc.setLinSpaced(t_dsp2_, v0_y_dsp2_offset,    vT_y_dsp2_offset);
    temp_py.segment(t_dsp1_ + t_ssp_calc, t_dsp2_) = zmp_plan_lin_calc;
}

void AvatarController::onestepVrpZ(unsigned int current_step_number, double t_total_zmp, Eigen::VectorXd& temp_pz)
{
    temp_pz.setZero(t_total_zmp);

    double height_diff = 0.0;

    if(current_step_number ==  0) { height_diff = - 0.10*param_scenario_ - 0.00*(1 - param_scenario_); }
    if(current_step_number ==  1) { height_diff = - 0.10*param_scenario_ - 0.00*(1 - param_scenario_); }
    if(current_step_number ==  2) { height_diff = - 0.10*param_scenario_ - 0.00*(1 - param_scenario_); }
    if(current_step_number ==  3) { height_diff = - 0.10*param_scenario_ - 0.00*(1 - param_scenario_); }
    if(current_step_number ==  4) { height_diff = - 0.00*param_scenario_ - 0.05*(1 - param_scenario_); }

    //foot_step_support_frame_(current_step_number, 2) = height_diff;

    temp_pz.setConstant(zc_mj_ + height_diff);
}

void AvatarController::getFootTrajectory()
{
    Eigen::Vector6d target_swing_foot;
    for (int i = 0; i < 6; i++)
    { target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i); }

    //before swing
    if (walking_tick_ < t_start_ + t_dsp1_)
    {
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            lfoot_trajectory_support_.translation().setZero();
            lfoot_trajectory_euler_support_.setZero();
            
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.translation().setZero();

            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;            
        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
    }
    //mid swing
    else if (walking_tick_ >= t_start_ + t_dsp1_ && walking_tick_ < t_start_ + t_total_ - t_dsp2_)
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();
            
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            if (walking_tick_ < t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0)
            {

                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, rfoot_support_init_.translation()(2), rfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
            }
            else
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_ - t_dsp2_, rfoot_support_init_.translation()(2) + foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, rfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            }

            rfoot_trajectory_euler_support_(0) = 0;
            rfoot_trajectory_euler_support_(1) = 0;
            rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, rfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            if (walking_tick_ < t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0)
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, lfoot_support_init_.translation()(2), lfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
            }
            else
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_ - t_dsp2_, lfoot_support_init_.translation()(2) + foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, lfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            }

            lfoot_trajectory_euler_support_(0) = 0;
            lfoot_trajectory_euler_support_(1) = 0;
            lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, lfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        }
    }
    //after swing
    else
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_euler_support_.setZero();
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            for (int i = 0; i < 3; i++)
            {
                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            for (int i = 0; i < 3; i++)
            {
                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        }
    }
}

void AvatarController::getFootTrajectory_stepping()
{   
    if(walking_tick_ == 0)
    {
        desired_swing_foot.setZero();
        target_swing_foot.setZero();    
        del_F_.setZero();
    }
          
    for (int i = 0; i < 6; i++)
    {
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i);
    }
             
    desired_swing_foot(0) = target_swing_foot(0) + del_F_(0);
    desired_swing_foot(1) = target_swing_foot(1) + del_F_(1);
    
    if(walking_tick_ == t_start_)
    {
        foot_pos_compen_.setZero();
    }

    Eigen::Vector3d foot_pos_compen_calc; foot_pos_compen_calc.setZero();

    if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
    {
        if(desired_swing_foot(0) < 0)
        {
            foot_pos_compen_calc(0) = desired_swing_foot(0) - zmp_x_min_foot_width_;
        }
        else
        {
            foot_pos_compen_calc(0) = desired_swing_foot(0) + zmp_x_max_foot_width_;
        }
        foot_pos_compen_calc(1) = desired_swing_foot(1) - zmp_y_min_foot_width_;
        if((walking_tick_ > t_start_ + t_dsp1_ + 0.1*hz_) && (walking_tick_ < t_start_ + t_total_ - t_dsp2_ - 0.05*hz_))
        {
            foot_pos_compen_ = (lfoot_float_current_.linear().transpose()*foot_pos_compen_calc).transpose();
        }
    }
    else
    {
        if(desired_swing_foot(0) < 0)
        {
            foot_pos_compen_calc(0) = desired_swing_foot(0) - zmp_x_min_foot_width_;
        }
        else
        {
            foot_pos_compen_calc(0) = desired_swing_foot(0) + zmp_x_max_foot_width_;
        }
        foot_pos_compen_calc(1) = desired_swing_foot(1) + zmp_y_max_foot_width_;
        if((walking_tick_ > t_start_ + t_dsp1_ + 0.1*hz_) && (walking_tick_ < t_start_ + t_total_ - t_dsp2_ - 0.05*hz_))
        {
            foot_pos_compen_ = (rfoot_float_current_.linear().transpose()*foot_pos_compen_calc).transpose();
        }
    }

    if(abs(del_F_(1)) > 1e-2)
    {
        target_swing_foot(2) = target_swing_foot(2) + 0.5*foot_pos_compen_(2);
    }

    double admittance_cubic_l_calc = 0.0;
    double admittance_cubic_r_calc = 0.0;

    double time_adj_tick_main_foot_traj;
    time_adj_tick_main_foot_traj = max(MPC_Stabilizer_time_adj_tick_x_main_, MPC_Stabilizer_time_adj_tick_y_main_);
    time_adj_tick_main_foot_traj = min(time_adj_tick_main_foot_traj, double(step_time_adj_candidate_num_ - 1));

    double t_total_foot_traj_;
    t_total_foot_traj_ = t_total_const_ - time_adj_tick_main_foot_traj*hz_/thread3_hz_;
    //t_total_foot_traj_ = t_total_const_;

    Eigen::Vector3d lfoot_float_current_euler;
    Eigen::Vector3d rfoot_float_current_euler;

    lfoot_float_current_euler.setZero() = DyrosMath::rot2Euler(lfoot_float_current_.linear());
    rfoot_float_current_euler.setZero() = DyrosMath::rot2Euler(rfoot_float_current_.linear());

    if (walking_tick_ < t_start_ + t_dsp1_)
    {
        if (foot_step_(current_step_num_, 6) == 1) // lfoot support, rfoot swing
        {
            lfoot_trajectory_support_.translation().setZero();
            lfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_ = rfoot_support_euler_init_*DyrosMath::cubic(walking_tick_, t_start_, t_start_ + t_dsp1_, 1.0, 0.0, 0.0, 0.0);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // rfoot support, lfoot swing
        {
            rfoot_trajectory_support_.translation().setZero();
            rfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_ = lfoot_support_euler_init_*DyrosMath::cubic(walking_tick_, t_start_, t_start_ + t_dsp1_, 1.0, 0.0, 0.0, 0.0);
        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) 
                                           * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1) + F_T_L_y_input)
                                           * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0) - F_T_L_x_input);

        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) 
                                           * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1) + F_T_R_y_input)
                                           * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0) - F_T_R_x_input);
    }
    else if (walking_tick_ >= t_start_ + t_dsp1_ && walking_tick_ < t_start_ + t_total_foot_traj_ - t_dsp2_)
    {   
        if (foot_step_(current_step_num_, 6) == 1) // lfoot support, rfoot swing
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))
                                               * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1) + F_T_L_y_input)
                                               * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0) - F_T_L_x_input);

            if (walking_tick_ < t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0)
            {
                Eigen::Vector3d temp;
                if(foot_step_support_frame_(current_step_num_, 2) > 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, rfoot_support_init_.translation()(2), 0.0, 0.0, rfoot_support_init_.translation()(2) + target_swing_foot(2) + foot_height_, 0.0, 0.0);
                    rfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else if(foot_step_support_frame_(current_step_num_, 2) == 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, rfoot_support_init_.translation()(2), 0.0, 0.0, rfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
                    rfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, rfoot_support_init_.translation()(2), 0.0, 0.0, rfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
                    rfoot_trajectory_support_.translation()(2) = temp(0);
                }
            }
            else
            {
                Eigen::Vector3d temp;
                if(foot_step_support_frame_(current_step_num_, 2) > 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_foot_traj_ - t_dsp2_, rfoot_support_init_.translation()(2) + target_swing_foot(2) + foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0);
                    rfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else if(foot_step_support_frame_(current_step_num_, 2) == 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_foot_traj_ - t_dsp2_, rfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0);
                    rfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_foot_traj_ - t_dsp2_, rfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0);
                    rfoot_trajectory_support_.translation()(2) = temp(0);
                }
            }

            Eigen::Vector3d temp;
            temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_foot_traj_ - t_dsp2_, rfoot_support_init_.translation()(0), 0.0, 0.0, desired_swing_foot(0), 0.0, 0.0);
            rfoot_trajectory_support_.translation()(0) = temp(0);
            rfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_foot_traj_ - t_dsp2_, rfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    
    
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_foot_traj_ - t_dsp2_, rfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))
                                               * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1) + F_T_R_y_input)
                                               * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0) - F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // rfoot support, lfoot swing
        {
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) 
                                               * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1) + F_T_R_y_input) 
                                               * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0) - F_T_R_x_input);

            if (walking_tick_ < t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0)
            {
                Eigen::Vector3d temp;
                if(foot_step_support_frame_(current_step_num_, 2) > 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, lfoot_support_init_.translation()(2), 0.0, 0.0, lfoot_support_init_.translation()(2) + target_swing_foot(2) + foot_height_, 0.0, 0.0);
                    lfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else if(foot_step_support_frame_(current_step_num_, 2) == 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, lfoot_support_init_.translation()(2), 0.0, 0.0, lfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
                    lfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, lfoot_support_init_.translation()(2), 0.0, 0.0, lfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
                    lfoot_trajectory_support_.translation()(2) = temp(0);
                }
            }
            else
            {
                Eigen::Vector3d temp;
                if(foot_step_support_frame_(current_step_num_, 2) > 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_foot_traj_ - t_dsp2_, lfoot_support_init_.translation()(2) + target_swing_foot(2) + foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0);
                    lfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else if(foot_step_support_frame_(current_step_num_, 2) == 0)
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_foot_traj_ - t_dsp2_, lfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0);
                    lfoot_trajectory_support_.translation()(2) = temp(0);
                }
                else
                {
                    temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_ + 1.0*(t_total_foot_traj_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_foot_traj_ - t_dsp2_, lfoot_support_init_.translation()(2) + foot_height_,0.0, 0.0, target_swing_foot(2), 0.0, 0.0);
                    lfoot_trajectory_support_.translation()(2) = temp(0);
                }
            }

            Eigen::Vector3d temp;
            temp = DyrosMath::QuinticSpline(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_foot_traj_ - t_dsp2_, lfoot_support_init_.translation()(0), 0.0, 0.0, desired_swing_foot(0), 0.0, 0.0);
            lfoot_trajectory_support_.translation()(0) = temp(0);
            lfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_foot_traj_ - t_dsp2_, lfoot_support_init_.translation()(1), desired_swing_foot(1), 0.0, 0.0);    

            lfoot_trajectory_euler_support_.setZero();
            lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_foot_traj_ - t_dsp2_, lfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) 
                                               * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1) + F_T_L_y_input) 
                                               * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0) - F_T_L_x_input);
        }
    }
    else
    {
        if (foot_step_(current_step_num_, 6) == 1) // lfoot support, rfoot swing
        {
            lfoot_trajectory_euler_support_.setZero();
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))
                                               * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1) + F_T_L_y_input)
                                               * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0) - F_T_L_x_input);

            for (int i = 0; i < 3; i++)
            {
                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_support_(i)         = target_swing_foot(i + 3);
            }
            // 220422
            rfoot_trajectory_support_.translation()(0) =  desired_swing_foot(0);
            rfoot_trajectory_support_.translation()(1) =  desired_swing_foot(1);

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))
                                               * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1) + F_T_R_y_input) 
                                               * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0) - F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // rfoot support, lfoot swing
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))
                                               * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1) + F_T_R_y_input)
                                               * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0) - F_T_R_x_input);

            for (int i = 0; i < 3; i++)
            {
                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_support_(i)         = target_swing_foot(i + 3);
            }
            // 220422
            lfoot_trajectory_support_.translation()(0) =  desired_swing_foot(0); 
            lfoot_trajectory_support_.translation()(1) =  desired_swing_foot(1);    

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))
                                               * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1) + F_T_L_y_input) 
                                               * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0) - F_T_L_x_input);
        }
    }
    
    lfoot_float_current_euler = DyrosMath::rot2Euler(lfoot_float_current_.linear());
    rfoot_float_current_euler = DyrosMath::rot2Euler(rfoot_float_current_.linear());

    lfoot_trajectory_support_.translation()(2) = DyrosMath::minmax_cut(lfoot_trajectory_support_.translation()(2), -1e3, foot_height_);
    rfoot_trajectory_support_.translation()(2) = DyrosMath::minmax_cut(rfoot_trajectory_support_.translation()(2), -1e3, foot_height_);
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
    ZMP_X_REF_ = ref_zmp_(tick,0);
    ZMP_Y_REF_ = ref_zmp_(tick,1);  

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
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (ref_zmp_(tick + 1 + i,0) - ref_zmp_(tick + i,0));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (ref_zmp_(tick + 1 + i,1) - ref_zmp_(tick + i,1));
    }

    Eigen::MatrixXd del_ux(1, 1);
    Eigen::MatrixXd del_uy(1, 1);
    del_ux.setZero();
    del_uy.setZero();

    Eigen::VectorXd GX_X(1);
    GX_X = Gx * (preview_x_mj - preview_x_b_mj);
    Eigen::VectorXd GX_Y(1);
    GX_Y = Gx * (preview_y_mj - preview_y_b_mj);

    if (walking_tick_ == 0)
    {
        del_zmp.setZero();
        cout << "del_zmp : " << del_zmp(0) << "," << del_zmp(1) << endl;
    }

    del_ux(0, 0) = -(px(0) - ref_zmp_(tick,0)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - ref_zmp_(tick,1)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x_mj + B * UX;
    YD = A * preview_y_mj + B * UY;    

    cp_desired_(0) = XD(0) + XD(1) / w_;
    cp_desired_(1) = YD(0) + YD(1) / w_; 

    // MJ_graph << XD(0) << "," << YD(0) << "," << ZMP_X_REF_ << "," << ZMP_Y_REF_ << "," << cp_desired_(0) << "," << cp_desired_(1) << endl;
}

void AvatarController::SC_err_compen(double x_des, double y_des)
{
    if (walking_tick_ == 0)
    {
        SC_com.setZero();
    }
    if (walking_tick_ == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {
        sc_err_before.setZero();
        sc_err_before(0) = com_support_current_(0) - foot_step_support_frame_(current_step_num_, 0); // 1.3으로 할꺼면 마지막에 더해야됨. SC_com을 이 함수보다 나중에 더하기 때문에
                                                                                                     // sc_err_before(1) = y_des - com_support_current_(1);
    }

    if (current_step_num_ != 0 && walking_tick_ == t_start_) // step change
    {
        sc_err_after.setZero();
        sc_err_after(0) = com_support_current_(0);
        // sc_err_after(1) = y_des - com_support_current_(1);
        sc_err = sc_err_after - sc_err_before;
    }

    if (current_step_num_ != 0)
    {
        SC_com(0) = DyrosMath::cubic(walking_tick_, t_start_, t_start_ + 0.05 * hz_, sc_err(0), 0, 0.0, 0.0);
        SC_com(1) = DyrosMath::cubic(walking_tick_, t_start_, t_start_ + 0.05 * hz_, sc_err(1), 0, 0.0, 0.0);
    }

    if (current_step_num_ != total_step_num_ - 1)
    {
        if (current_step_num_ != 0 && walking_tick_ >= t_start_ && walking_tick_ < t_start_ + t_total_)
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
        if (walking_tick_ >= t_start_ && walking_tick_ < t_start_ + 2 * t_total_)
        {
            com_support_cp_(0) = com_support_current_(0) - SC_com(0);
        }
    }
}

void AvatarController::getPelvTrajectory()
{
    double z_rot = foot_step_support_frame_(current_step_num_, 5);

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - com_support_current_(0));
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.9 * (com_desired_(1) - com_support_current_(1));
    pelv_trajectory_support_.translation()(2) = pelv_support_current_.translation()(2) + 1.0 * (com_desired_(2) - com_support_current_(2));

    Eigen::Vector3d Trunk_trajectory_euler;
    Trunk_trajectory_euler.setZero();

    if (walking_tick_ < t_start_ + t_dsp1_)
    {
        Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    }
    else if (walking_tick_ >= t_start_ + t_dsp1_ && walking_tick_ < t_start_ + t_total_ - t_dsp2_)
    {
        Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, pelv_support_euler_init_(2), z_rot / 2.0, 0.0, 0.0);
    }
    else
    {
        Trunk_trajectory_euler(2) = z_rot / 2.0;
    }

    if (aa == 0 && walking_tick_ == 0 && (walking_enable_ == true))
    {
        P_angle_input = 0;
        R_angle_input = 0;
    }

    //P_angle_input_dot = 2.25 * (0.0 - P_angle) + 0.5 * (0.0 - rd_.link_[Pelvis].w(0));
    //R_angle_input_dot = 2.25 * (0.0 - R_angle) + 0.5 * (0.0 - rd_.link_[Pelvis].w(1));

    P_angle_input_dot = 1.5 * (0.0 - P_angle);
    R_angle_input_dot = 2.0 * (0.0 - R_angle);

    P_angle_input = P_angle_input + P_angle_input_dot * del_t;
    R_angle_input = R_angle_input + R_angle_input_dot * del_t;

    R_angle_input = DyrosMath::minmax_cut(R_angle_input, -3*DEG2RAD, 3*DEG2RAD);
    P_angle_input = DyrosMath::minmax_cut(P_angle_input, -5*DEG2RAD, 5*DEG2RAD);

    //Trunk_trajectory_euler(0) = R_angle_input;
    Trunk_trajectory_euler(1) = P_angle_input;

    pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2)) 
                                      * DyrosMath::rotateWithY(Trunk_trajectory_euler(1)) 
                                      * DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void AvatarController::supportToFloatPattern()
{
    pelv_trajectory_float_  = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;

    lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
    rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
}

    
void AvatarController::getComTrajectory_mpc()
{
    if(walking_tick_ == 0)
    {
        MPC_Planner_state_mpc_.setZero(9);
        MPC_Planner_state_container_from_mpc_.setZero(9);
        MPC_Planner_state_main_.setZero(9);
        
        Planner_state_main_calc_.setZero(9);
        Stabilizer_state_main_calc_.setZero(9);

        MPC_Stabilizer_state_mpc_.setZero(9);
        MPC_Stabilizer_state_container_from_mpc_.setZero(9);
        MPC_Stabilizer_state_main_.setZero(9);

        MPC_Planner_state_mpc_(0) = com_support_current_(0); 
        MPC_Planner_state_mpc_(2) = com_support_current_(0); 
        MPC_Planner_state_mpc_(3) = yi_mj_;
        MPC_Planner_state_mpc_(5) = yi_mj_;
        MPC_Planner_state_mpc_(6) = zc_mj_;
        MPC_Planner_state_mpc_(8) = zc_mj_;

        MPC_Planner_state_container_from_mpc_     = MPC_Planner_state_mpc_;
        MPC_Planner_state_main_                   = MPC_Planner_state_mpc_;
        Planner_state_main_calc_                  = MPC_Planner_state_mpc_;
        Stabilizer_state_main_calc_               = MPC_Planner_state_mpc_;
        MPC_Stabilizer_state_container_from_mpc_  = MPC_Planner_state_mpc_;
        MPC_Stabilizer_state_main_                = MPC_Planner_state_mpc_;
        MPC_Stabilizer_state_mpc_                 = MPC_Planner_state_mpc_;

        MPC_Planner_u_main_.setZero(3);

        foot_step_support_frame_offset_mpc_              = foot_step_support_frame_offset_;
        foot_step_support_frame_offset_container_to_mpc_ = foot_step_support_frame_offset_;

        thread3_hz_ = 50.0;
        //thread3_hz_ = 40.0;
        //thread3_hz_ = 30.0;

        step_enable_time_fwd_ = 0.15;
        step_enable_time_bwd_ = 0.00;
        step_enable_fix_time_pre_ = 3/thread3_hz_;
        step_time_adj_candidate_num_ = (step_enable_time_fwd_ + step_enable_time_bwd_)*thread3_hz_ + 1;

        MPC_Stabilizer_delf_main_.setZero(2*step_time_adj_candidate_num_);
        MPC_Stabilizer_delf_main_(0*step_time_adj_candidate_num_) = foot_step_support_frame_(current_step_num_,0);
        MPC_Stabilizer_delf_main_(1*step_time_adj_candidate_num_) = foot_step_support_frame_(current_step_num_,1);

        cout << "step enable time forward: " << step_enable_time_fwd_ << endl;
        cout << "step time adjustment candidate num: " << step_time_adj_candidate_num_ << endl << endl;

        A_main_.resize(3,3);
        Eigen::MatrixXd A_calc_cont; A_calc_cont.resize(3,3);
        B_main_.resize(3,1);
        Eigen::MatrixXd B_calc_cont; B_calc_cont.resize(3,1);
        double w_mpc = sqrt(GRAVITY/zc_mj_);
        double dt_main = 1/hz_;

        A_calc_cont <<           0, 1,         0,
                    w_mpc*w_mpc, 0, -w_mpc*w_mpc,
                              0, 0,            0;
        
        A_main_ = MatrixXd::Identity(3,3) 
                + A_calc_cont*dt_main 
                + A_calc_cont*A_calc_cont*dt_main*dt_main/(1*2) 
                + A_calc_cont*A_calc_cont*A_calc_cont*dt_main*dt_main*dt_main/(1*2*3)
                + A_calc_cont*A_calc_cont*A_calc_cont*A_calc_cont*dt_main*dt_main*dt_main*dt_main/(1*2*3*4);

        B_calc_cont << 0,
                       0,
                       1;
        
        B_main_ = B_calc_cont*dt_main 
                + A_calc_cont*B_calc_cont*dt_main*dt_main/(1*2) 
                + A_calc_cont*A_calc_cont*B_calc_cont*dt_main*dt_main*dt_main/(1*2*3)
                + A_calc_cont*A_calc_cont*A_calc_cont*B_calc_cont*dt_main*dt_main*dt_main*dt_main/(1*2*3*4);

        cout << "CoM MPC Main thread Parameter Initialization Complete" << endl;
    }

    com_start_tick_ = (bool)current_step_num_*t_start_;

    if(atb_main_to_mpc_update_ == false)
    {
        atb_main_to_mpc_update_ = true;
        walking_tick_container_to_mpc_ = walking_tick_;
        com_start_tick_container_to_mpc_ = com_start_tick_;
        current_step_num_container_to_mpc_ = current_step_num_;

        t_start_container_to_mpc_ = t_start_;
        t_total_container_to_mpc_ = t_total_;

        ref_zmp_container_to_mpc_ = ref_zmp_;
        ref_zmp_wo_offset_container_to_mpc_ = ref_zmp_wo_offset_;

        ref_vrp_container_to_mpc_ = ref_vrp_;
        
        dcm_measured_container_to_mpc_ = dcm_measured_;
        com_measured_container_to_mpc_ = com_measured_;
        com_dot_measured_container_to_mpc_ = com_dot_measured_;

        foot_step_support_frame_offset_container_to_mpc_ = foot_step_support_frame_offset_;
        foot_step_support_frame_container_to_mpc_ = foot_step_support_frame_;

        atb_main_to_mpc_update_ = false;
    }

    double thread_freq = thread3_hz_;
    if(mpc_update_ == true)
    { 
        if(atb_mpc_to_main_update_ == false)
        {
            atb_mpc_to_main_update_ = true;

            MPC_Planner_u_main_(0) = MPC_Planner_u_container_from_mpc_(0);
            MPC_Planner_u_main_(1) = MPC_Planner_u_container_from_mpc_(0);
            MPC_Planner_u_main_(2) = 0.0;

            MPC_Stabilizer_u_main_(0) = MPC_Stabilizer_u_container_from_mpc_(0);
            MPC_Stabilizer_u_main_(1) = MPC_Stabilizer_u_container_from_mpc_(1);
            MPC_Stabilizer_u_main_(2) = 0.0;

            if(current_step_num_container_from_mpc_ == current_step_num_)
            {
                MPC_Planner_state_main_ = MPC_Planner_state_container_from_mpc_;
                
                MPC_Stabilizer_state_main_ = MPC_Stabilizer_state_container_from_mpc_;
            }
            else
            {   
                MPC_Planner_state_main_.segment(0,3) = A_mpc_*MPC_Planner_state_main_.segment(0,3) + B_mpc_*MPC_Planner_u_main_(0);
                MPC_Planner_state_main_.segment(3,3) = A_mpc_*MPC_Planner_state_main_.segment(3,3) + B_mpc_*MPC_Planner_u_main_(1);
                MPC_Planner_state_main_.segment(6,3) = A_mpc_*MPC_Planner_state_main_.segment(6,3) + B_mpc_*MPC_Planner_u_main_(2);

                MPC_Stabilizer_state_main_.segment(0,3) = A_mpc_*MPC_Stabilizer_state_main_.segment(0,3) + B_mpc_*MPC_Stabilizer_u_main_(0);
                MPC_Stabilizer_state_main_.segment(3,3) = A_mpc_*MPC_Stabilizer_state_main_.segment(3,3) + B_mpc_*MPC_Stabilizer_u_main_(1);
                MPC_Stabilizer_state_main_.segment(6,3) = A_mpc_*MPC_Stabilizer_state_main_.segment(6,3) + B_mpc_*MPC_Stabilizer_u_main_(2);
            }

            Planner_state_main_calc_             = MPC_Planner_state_main_;
            Stabilizer_state_main_calc_          = MPC_Stabilizer_state_main_;

            MPC_Stabilizer_delf_main_            = MPC_Stabilizer_delf_container_from_mpc_;

            MPC_Stabilizer_time_adj_tick_x_main_ = MPC_Stabilizer_time_adj_tick_x_container_from_mpc_;
            MPC_Stabilizer_time_adj_tick_y_main_ = MPC_Stabilizer_time_adj_tick_y_container_from_mpc_;

            step_enable_bool_one_tick_main_      = step_enable_bool_one_tick_container_from_mpc_;

            atb_mpc_to_main_update_     = false;
        }

        mpc_update_ = false;
    }

    Planner_state_main_calc_.segment(0,3) = A_main_*Planner_state_main_calc_.segment(0,3) + B_main_*MPC_Planner_u_main_(0);
    Planner_state_main_calc_.segment(3,3) = A_main_*Planner_state_main_calc_.segment(3,3) + B_main_*MPC_Planner_u_main_(1);
    Planner_state_main_calc_.segment(6,3) = A_main_*Planner_state_main_calc_.segment(6,3) + B_main_*MPC_Planner_u_main_(2);
    
    Stabilizer_state_main_calc_.segment(0,3) = A_main_*Stabilizer_state_main_calc_.segment(0,3) + B_main_*MPC_Stabilizer_u_main_(0);
    Stabilizer_state_main_calc_.segment(3,3) = A_main_*Stabilizer_state_main_calc_.segment(3,3) + B_main_*MPC_Stabilizer_u_main_(1);
    Stabilizer_state_main_calc_.segment(6,3) = A_main_*Stabilizer_state_main_calc_.segment(6,3) + B_main_*MPC_Stabilizer_u_main_(2);

    com_desired_(0) = Planner_state_main_calc_(0);
    com_desired_(1) = Planner_state_main_calc_(3);
    com_desired_(2) = Planner_state_main_calc_(6);

    dcm_desired_(0) = Planner_state_main_calc_(0) + Planner_state_main_calc_(1)/w_;
    dcm_desired_(1) = Planner_state_main_calc_(3) + Planner_state_main_calc_(4)/w_;
    dcm_desired_(2) = Planner_state_main_calc_(6) + Planner_state_main_calc_(7)/w_;

    step_enable_bool_main_ = step_enable_bool_one_tick_main_;

    double time_adj_tick_main = 0.0;
    if(current_step_num_ != 0 && step_enable_bool_main_ == 0)
    {
        time_adj_tick_main = max(MPC_Stabilizer_time_adj_tick_x_main_, MPC_Stabilizer_time_adj_tick_y_main_);
        time_adj_tick_main = min(time_adj_tick_main, double(step_time_adj_candidate_num_ - 1));

        t_total_ = round(t_total_const_ - time_adj_tick_main*hz_/thread3_hz_);
        if(current_step_num_ != 0)
        {
            t_last_ = t_start_ + t_total_ - 1;
        }
    }

    if(walking_tick_ == t_start_ + t_total_const_ - t_dsp2_const_ - (step_enable_time_fwd_ - 0.005)*hz_ - step_enable_fix_time_pre_*hz_)
    {
        if(time_adj_tick_main > 0)
        {
            cout << "current step num: " << current_step_num_                                << endl;
            cout << "X tick adj: "       << MPC_Stabilizer_time_adj_tick_x_main_             << endl;
            cout << "X time adj: "       << MPC_Stabilizer_time_adj_tick_x_main_/thread3_hz_ << endl;
            cout << "Y tick adj: "       << MPC_Stabilizer_time_adj_tick_y_main_             << endl;
            cout << "Y time adj: "       << MPC_Stabilizer_time_adj_tick_y_main_/thread3_hz_ << endl;
            cout << "Tick Adj: "         << time_adj_tick_main                               << endl;
            cout << "Time Adj: "         << time_adj_tick_main/thread3_hz_                   << endl;
            cout << endl;
        }
    }

    if(walking_tick_ <= t_start_ + hz_/thread3_hz_)
    {
        MPC_Stabilizer_delf_main_.setZero();
        MPC_Stabilizer_delf_main_(0) = foot_step_support_frame_(current_step_num_, 0);
        MPC_Stabilizer_delf_main_(1) = foot_step_support_frame_(current_step_num_, 1);

        MPC_Stabilizer_time_adj_tick_x_main_ = 0.0;
        MPC_Stabilizer_time_adj_tick_y_main_ = 0.0;

        t_total_ = t_total_const_;
    }

    del_F_(0) = MPC_Stabilizer_delf_main_(0) - foot_step_support_frame_(current_step_num_, 0);
    del_F_(1) = MPC_Stabilizer_delf_main_(1) - foot_step_support_frame_(current_step_num_, 1);

    if(walking_tick_ == t_start_ + t_total_ - 1)
    {
        del_F_prev_ = del_F_;
    }

//e_tmp_graph1 << step_enable_bool_main_                         << "," << t_start_                                       << ","
//             << t_total_                                       << "," << current_step_num_                              << ","
//             << com_desired_(0)                                << "," << com_desired_(1)                                << "," 
//             << zmp_desired_(0)                                << "," << zmp_desired_(1)                                << "," 
//             << dcm_desired_(0)                                << "," << dcm_desired_(1)                                << "," 
//             << dcm_measured_(0)                               << "," << dcm_measured_(1)                               << "," 
//             << MPC_Stabilizer_state_main_(2)                  << "," << MPC_Stabilizer_state_main_(5)                  << "," 
//             << del_F_(0)                                      << "," << del_F_(1)                                      << "," 
//             << Planner_state_main_calc_(0)                    << "," << Planner_state_main_calc_(3)                    << ","
//             << MPC_Stabilizer_state_main_(0)                  << "," << MPC_Stabilizer_state_main_(3)                  << ","
//             << foot_step_support_frame_(current_step_num_, 0) << "," << foot_step_support_frame_(current_step_num_, 1) << ","
//             << del_F_prev_(0)                                 << "," << del_F_prev_(1)                                 << ","
//             << MPC_Stabilizer_time_adj_tick_x_main_           << "," << MPC_Stabilizer_time_adj_tick_y_main_           << ","
//             << lfoot_trajectory_support_.translation()(2)     << "," << rfoot_trajectory_support_.translation()(2)     << ","
//             << endl;

    //step change
    if(current_step_num_ != total_step_num_ - 1)
    {
        foot_step_support_frame_.block(current_step_num_, 0, 1, 2) += del_F_.transpose();
    }
    if(walking_tick_ == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1)
    {
        Eigen::Vector3d var_after_step_change, var_before_step_change, frame_pos_diff;
        Eigen::Matrix3d frame_rot_diff;

        frame_rot_diff = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        frame_pos_diff(0) = foot_step_support_frame_(current_step_num_,0);
        frame_pos_diff(1) = foot_step_support_frame_(current_step_num_,1);
        frame_pos_diff(2) = foot_step_support_frame_(current_step_num_,2);

        //com step change
        var_before_step_change(0) = Planner_state_main_calc_(0);
        var_before_step_change(1) = Planner_state_main_calc_(3);
        var_before_step_change(2) = Planner_state_main_calc_(6);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        Planner_state_main_calc_(0) = var_after_step_change(0);
        Planner_state_main_calc_(3) = var_after_step_change(1);
        Planner_state_main_calc_(6) = var_after_step_change(2);

        var_before_step_change(0) = Planner_state_main_calc_(2);
        var_before_step_change(1) = Planner_state_main_calc_(5);
        var_before_step_change(2) = Planner_state_main_calc_(8);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        Planner_state_main_calc_(2) = var_after_step_change(0);
        Planner_state_main_calc_(5) = var_after_step_change(1);
        Planner_state_main_calc_(8) = var_after_step_change(2);

        //com step change
        var_before_step_change(0) = MPC_Planner_state_main_(0);
        var_before_step_change(1) = MPC_Planner_state_main_(3);
        var_before_step_change(2) = MPC_Planner_state_main_(6);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        MPC_Planner_state_main_(0) = var_after_step_change(0);
        MPC_Planner_state_main_(3) = var_after_step_change(1);
        MPC_Planner_state_main_(6) = var_after_step_change(2);

        var_before_step_change(0) = MPC_Planner_state_main_(1);
        var_before_step_change(1) = MPC_Planner_state_main_(4);
        var_before_step_change(2) = MPC_Planner_state_main_(7);
        var_after_step_change = frame_rot_diff*var_before_step_change;
        MPC_Planner_state_main_(1) = var_after_step_change(0);
        MPC_Planner_state_main_(4) = var_after_step_change(1);
        MPC_Planner_state_main_(7) = var_after_step_change(2);

        var_before_step_change(0) = MPC_Planner_state_main_(2);
        var_before_step_change(1) = MPC_Planner_state_main_(5);
        var_before_step_change(2) = MPC_Planner_state_main_(8);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        MPC_Planner_state_main_(2) = var_after_step_change(0);
        MPC_Planner_state_main_(5) = var_after_step_change(1);
        MPC_Planner_state_main_(8) = var_after_step_change(2);

        var_before_step_change(0) = MPC_Stabilizer_state_main_(0);
        var_before_step_change(1) = MPC_Stabilizer_state_main_(3);
        var_before_step_change(2) = MPC_Stabilizer_state_main_(6);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        MPC_Stabilizer_state_main_(0) = var_after_step_change(0);
        MPC_Stabilizer_state_main_(3) = var_after_step_change(1);
        MPC_Stabilizer_state_main_(6) = var_after_step_change(2);

        var_before_step_change(0) = MPC_Stabilizer_state_main_(2);
        var_before_step_change(1) = MPC_Stabilizer_state_main_(5);
        var_before_step_change(2) = MPC_Stabilizer_state_main_(8);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        MPC_Stabilizer_state_main_(2) = var_after_step_change(0);
        MPC_Stabilizer_state_main_(5) = var_after_step_change(1);
        MPC_Stabilizer_state_main_(8) = var_after_step_change(2);

        var_before_step_change(0) = Stabilizer_state_main_calc_(0);
        var_before_step_change(1) = Stabilizer_state_main_calc_(3);
        var_before_step_change(2) = Stabilizer_state_main_calc_(6);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        Stabilizer_state_main_calc_(0) = var_after_step_change(0);
        Stabilizer_state_main_calc_(3) = var_after_step_change(1);
        Stabilizer_state_main_calc_(6) = var_after_step_change(2);

        var_before_step_change(0) = Stabilizer_state_main_calc_(2);
        var_before_step_change(1) = Stabilizer_state_main_calc_(5);
        var_before_step_change(2) = Stabilizer_state_main_calc_(8);
        var_after_step_change = frame_rot_diff*(var_before_step_change - frame_pos_diff);
        Stabilizer_state_main_calc_(2) = var_after_step_change(0);
        Stabilizer_state_main_calc_(5) = var_after_step_change(1);
        Stabilizer_state_main_calc_(8) = var_after_step_change(2);
    }
    if(current_step_num_ != total_step_num_ - 1)
    {
        foot_step_support_frame_.block(current_step_num_, 0, 1, 2) -= del_F_.transpose();
    }
}

////////////////////// Econom2 function

void AvatarController::IS_LIPM_CoM_Planner_MPC(double mpc_freq, double mpc_dt, double mpc_preview_window, int mpc_synchro_hz)
{
    double wpvx, wpvy;
    double wdvx, wdvy;

    wpvx = 1e+2; wpvy = 1e+6;
    wdvx = 1e+0; wdvy = 1e+0;

    int mpc_tick = walking_tick_mpc_ - com_start_tick_mpc_;
    const int N_plan_mpc = mpc_preview_window*mpc_freq;
    const int N_step = t_total_const_/mpc_synchro_hz;
    const int N_state = 3; //com position, com velocity, vrp position
    static int MPC_first_loop = 0;

    double lambda_is_calc = exp(-w_*mpc_dt);

    if(MPC_first_loop == 0)
    {
        cout << "Initialization of IS LIPM Planner MPC." << endl;
        A_mpc_.resize(N_state,N_state);
        Eigen::MatrixXd A_mpc_cont; A_mpc_cont.resize(N_state, N_state); A_mpc_cont.setZero();
        A_mpc_cont <<     0, 1,      0,
                      w_*w_, 0, -w_*w_,
                          0, 0,      0;
        
        A_mpc_ = MatrixXd::Identity(N_state,N_state) 
               + A_mpc_cont*mpc_dt 
               + A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt/(1*2) 
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt*mpc_dt/(1*2*3)
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt*mpc_dt*mpc_dt/(1*2*3*4);

        B_mpc_.resize(N_state,1);
        Eigen::MatrixXd B_mpc_cont; B_mpc_cont.resize(N_state,1); B_mpc_cont.setZero();
        B_mpc_cont << 0,
                      0,
                      1;
        
        B_mpc_ = B_mpc_cont*mpc_dt 
               + A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt/(1*2) 
               + A_mpc_cont*A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt*mpc_dt/(1*2*3)
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt*mpc_dt*mpc_dt/(1*2*3*4);

        Ccp_mpc_.resize(1,N_state);
        Ccv_mpc_.resize(1,N_state);
        Cvp_mpc_.resize(1,N_state);

        Ccp_mpc_ << 1, 0, 0;
        Ccv_mpc_ << 0, 1, 0;
        Cvp_mpc_ << 0, 0, 1;

        Pcps_plan_mpc_.resize(N_plan_mpc,N_state);
        Pcvs_plan_mpc_.resize(N_plan_mpc,N_state);
        Pvps_plan_mpc_.resize(N_plan_mpc,N_state);

        Eigen::MatrixXd Ps_calc;
        Ps_calc.resize(N_state,N_state);
        Ps_calc = A_mpc_;
        
        Pcpu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Pcvu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Pvpu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        
        P_IS_step_mpc_.setZero(N_step, N_step);

        b_IS_plan_mpc_.setZero(N_plan_mpc,1);
        b_IS_step_mpc_.setZero(N_step,1);

        p_IS_step_mpc_.setOnes(N_step,1);
        
        Eigen::MatrixXd Pu_calc, Pu_step_calc;
        Pu_calc.setZero(N_state,N_plan_mpc);
        Pu_step_calc.setZero(N_state,N_step);
        
        for(int i = 0; i < N_plan_mpc; i++)
        {
            Pcps_plan_mpc_.row(i) = Ccp_mpc_*Ps_calc;
            Pcvs_plan_mpc_.row(i) = Ccv_mpc_*Ps_calc;
            Pvps_plan_mpc_.row(i) = Cvp_mpc_*Ps_calc;
            Ps_calc = Ps_calc*A_mpc_;

            Pu_calc.col(i) = B_mpc_;
            Pcpu_plan_mpc_.row(i) = Ccp_mpc_*Pu_calc;
            Pcvu_plan_mpc_.row(i) = Ccv_mpc_*Pu_calc;
            Pvpu_plan_mpc_.row(i) = Cvp_mpc_*Pu_calc;
            b_IS_plan_mpc_(i,0) = pow(lambda_is_calc, i);
            
            if(i < N_step)
            {
                Pu_step_calc.col(i)   = B_mpc_;
                P_IS_step_mpc_.row(i) = Cvp_mpc_*Pu_step_calc;    
                b_IS_step_mpc_(i,0)   = pow(lambda_is_calc, i);
            }

            Pu_calc = A_mpc_*Pu_calc;
            Pu_step_calc = A_mpc_*Pu_step_calc;
        }

        SUx_plan_mpc_.setZero(N_plan_mpc, 2*N_plan_mpc); SUx_plan_mpc_ << MatrixXd::Identity(N_plan_mpc, N_plan_mpc), MatrixXd::Zero(N_plan_mpc, N_plan_mpc);
        SUy_plan_mpc_.setZero(N_plan_mpc, 2*N_plan_mpc); SUy_plan_mpc_ << MatrixXd::Zero(N_plan_mpc, N_plan_mpc), MatrixXd::Identity(N_plan_mpc, N_plan_mpc);

        ssx_plan_mpc_.setZero(N_state,2*N_state); ssx_plan_mpc_ << MatrixXd::Identity(N_state,N_state), MatrixXd::Zero(N_state,N_state);
        ssy_plan_mpc_.setZero(N_state,2*N_state); ssy_plan_mpc_ << MatrixXd::Zero(N_state,N_state), MatrixXd::Identity(N_state,N_state);

        Qmat_plan_mpc_.resize(N_plan_mpc, N_plan_mpc);
        Qmat_plan_mpc_.setIdentity();

        Qxcalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Qxcalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvx*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvx*Qmat_plan_mpc_;

        Qycalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Qycalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvy*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvy*Qmat_plan_mpc_;

        Qcalc_plan_mpc_.setZero(2*N_plan_mpc, 2*N_plan_mpc);
        Qcalc_plan_mpc_ = SUx_plan_mpc_.transpose()*Qxcalc_plan_mpc_*SUx_plan_mpc_ + SUy_plan_mpc_.transpose()*Qycalc_plan_mpc_*SUy_plan_mpc_;
        
        gxcalc_plan_mpc_.setZero(2*N_plan_mpc, N_plan_mpc);
        gxcalc_plan_mpc_ = SUx_plan_mpc_.transpose()*Pvpu_plan_mpc_.transpose()*wpvx*Qmat_plan_mpc_;

        gycalc_plan_mpc_.setZero(2*N_plan_mpc, N_plan_mpc);
        gycalc_plan_mpc_ = SUy_plan_mpc_.transpose()*Pvpu_plan_mpc_.transpose()*wpvy*Qmat_plan_mpc_;

        QP_MPC_Planner_.InitializeProblemSize(2*N_plan_mpc, 2*N_plan_mpc + 3);

        MPC_Planner_u_mpc_.setZero(2*N_plan_mpc);

        Planner_State_Prev_mpc_.setZero(9, N_plan_mpc);

        Pv_dot_ref_mpc_.setZero(N_step, 2);
        
        zmp_max_x_mpc_.setZero(N_plan_mpc);
        zmp_min_x_mpc_.setZero(N_plan_mpc);

        zmp_max_y_mpc_.setZero(N_plan_mpc);
        zmp_min_y_mpc_.setZero(N_plan_mpc);

        MPC_first_loop = 1;
        cout << "Initialization of IS LIPM Planner MPC is completed." << endl;
    }

    Eigen::VectorXd Pv_x_ref(N_plan_mpc);
    Eigen::VectorXd Pv_y_ref(N_plan_mpc);

    for(int i = 0; i < N_plan_mpc; i++)
    {
        Pv_x_ref(i) = ref_zmp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0);
        Pv_y_ref(i) = ref_zmp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1);

        if(i < N_step)
        {
            Pv_dot_ref_mpc_(i,0) = (ref_zmp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),0) - ref_zmp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),0))/mpc_dt;
            Pv_dot_ref_mpc_(i,1) = (ref_zmp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),1) - ref_zmp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),1))/mpc_dt;
        }

        zmp_max_x_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0) + zmp_x_max;     
        zmp_min_x_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0) - zmp_x_min;

        zmp_max_y_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1) + zmp_y_max;
        zmp_min_y_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1) - zmp_y_min;
    }

    gcalc_plan_mpc_ = gxcalc_plan_mpc_*(Pvps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_.segment(0,6) - Pv_x_ref)
                    + gycalc_plan_mpc_*(Pvps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_.segment(0,6) - Pv_y_ref);

    QP_MPC_Planner_.EnableEqualityCondition(equality_condition_eps_);
    QP_MPC_Planner_.UpdateMinProblem(Qcalc_plan_mpc_, gcalc_plan_mpc_);
    QP_MPC_Planner_.DeleteSubjectToAx();
    QP_MPC_Planner_.DeleteSubjectToX();
    
    const_A_mpc_.setZero(2*N_plan_mpc + 3, 2*N_plan_mpc);
    const_ub_mpc_.setZero(2*N_plan_mpc + 3, 1);
    const_lb_mpc_.setZero(2*N_plan_mpc + 3, 1);

    const_A_mpc_.block(0*N_plan_mpc, 0, 1*N_plan_mpc, 2*N_plan_mpc) = Pvpu_plan_mpc_*SUx_plan_mpc_;
    const_A_mpc_.block(1*N_plan_mpc, 0, 1*N_plan_mpc, 2*N_plan_mpc) = Pvpu_plan_mpc_*SUy_plan_mpc_;

    const_ub_mpc_.block(0*N_plan_mpc, 0, 1*N_plan_mpc, 1) = zmp_max_x_mpc_ - Pvps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_.segment(0,6);
    const_ub_mpc_.block(1*N_plan_mpc, 0, 1*N_plan_mpc, 1) = zmp_max_y_mpc_ - Pvps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_.segment(0,6);

    const_lb_mpc_.block(0*N_plan_mpc, 0, 1*N_plan_mpc, 1) = zmp_min_x_mpc_ - Pvps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_.segment(0,6);
    const_lb_mpc_.block(1*N_plan_mpc, 0, 1*N_plan_mpc, 1) = zmp_min_y_mpc_ - Pvps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_.segment(0,6);

    //IS EQ
    Eigen::MatrixXd Const_b_eq_;  Const_b_eq_.setZero(2,1);

    Const_b_eq_(0,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(0) + MPC_Planner_state_mpc_(1)/w_ - MPC_Planner_state_mpc_(2))
                       -(pow(lambda_is_calc, N_plan_mpc)/(1 - pow(lambda_is_calc,N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(0))(0,0));
    Const_b_eq_(1,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(3) + MPC_Planner_state_mpc_(4)/w_ - MPC_Planner_state_mpc_(5))
                       -(pow(lambda_is_calc, N_plan_mpc)/(1 + pow(lambda_is_calc,N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(1))(0,0));

    const_A_mpc_.row(2*N_plan_mpc + 0) = b_IS_plan_mpc_.transpose()*SUx_plan_mpc_;
    const_A_mpc_.row(2*N_plan_mpc + 1) = b_IS_plan_mpc_.transpose()*SUy_plan_mpc_;

    const_ub_mpc_.block(2*N_plan_mpc + 0, 0, 2, 1) = Const_b_eq_.block(0, 0, 2, 1);
    const_lb_mpc_.block(2*N_plan_mpc + 0, 0, 2, 1) = Const_b_eq_.block(0, 0, 2, 1);

    QP_MPC_Planner_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);

    if(QP_MPC_Planner_.SolveQPoases(100, MPC_Planner_u_mpc_))
    {
        if((walking_tick_mpc_ - mpc_synchro_hz - 20)%int(2*hz_) == 0)
        { cout << "IS LIPM Planner MPC Solved" << endl;; }

        Planner_State_Prev_mpc_.row(0).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcpu_plan_mpc_*SUx_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(1).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcvu_plan_mpc_*SUx_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(2).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pvpu_plan_mpc_*SUx_plan_mpc_*MPC_Planner_u_mpc_;

        Planner_State_Prev_mpc_.row(3).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcpu_plan_mpc_*SUy_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(4).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcvu_plan_mpc_*SUy_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(5).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pvpu_plan_mpc_*SUy_plan_mpc_*MPC_Planner_u_mpc_;
        
        Planner_State_Prev_mpc_.row(6).transpose().setConstant(zc_mj_);
        Planner_State_Prev_mpc_.row(7).transpose().setConstant(0.0);
        Planner_State_Prev_mpc_.row(8).transpose().setConstant(zc_mj_);

        MPC_Planner_state_mpc_.segment(0,3) = A_mpc_*MPC_Planner_state_mpc_.segment(0,3) + B_mpc_*(SUx_plan_mpc_*MPC_Planner_u_mpc_)(0);
        MPC_Planner_state_mpc_.segment(3,3) = A_mpc_*MPC_Planner_state_mpc_.segment(3,3) + B_mpc_*(SUy_plan_mpc_*MPC_Planner_u_mpc_)(0);
    }
    else
    { 
        cout << "IS LIPM Planner MPC Not Solved" << endl;
        cout << int(walking_tick_mpc_ - 20)/mpc_synchro_hz << endl;

        Planner_State_Prev_mpc_.row(0).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcpu_plan_mpc_*SUx_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(1).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcvu_plan_mpc_*SUx_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(2).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pvpu_plan_mpc_*SUx_plan_mpc_*MPC_Planner_u_mpc_;

        Planner_State_Prev_mpc_.row(3).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcpu_plan_mpc_*SUy_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(4).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcvu_plan_mpc_*SUy_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(5).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pvpu_plan_mpc_*SUy_plan_mpc_*MPC_Planner_u_mpc_;
                
        Planner_State_Prev_mpc_.row(6).transpose().setConstant(zc_mj_);
        Planner_State_Prev_mpc_.row(7).transpose().setConstant(0.0);
        Planner_State_Prev_mpc_.row(8).transpose().setConstant(zc_mj_);

        MPC_Planner_state_mpc_.segment(0,3) = A_mpc_*MPC_Planner_state_mpc_.segment(0,3) + B_mpc_*(SUx_plan_mpc_*MPC_Planner_u_mpc_)(0);
        MPC_Planner_state_mpc_.segment(3,3) = A_mpc_*MPC_Planner_state_mpc_.segment(3,3) + B_mpc_*(SUy_plan_mpc_*MPC_Planner_u_mpc_)(0);
    }

    e_mpc_planner_data << setprecision(10)
                       << Pv_x_ref(0)               << "," << Pv_y_ref(0)               << "," << 0 <<","
                       << MPC_Planner_state_mpc_(0) << "," << MPC_Planner_state_mpc_(3) << "," << 0 <<","
                       << MPC_Planner_state_mpc_(1) << "," << MPC_Planner_state_mpc_(4) << "," << 0 <<","
                       << MPC_Planner_state_mpc_(2) << "," << MPC_Planner_state_mpc_(5) << "," << 0 <<","
                       << zmp_max_x_mpc_(0)         << "," << zmp_max_y_mpc_(0)         << "," << 0 <<","
                       << endl;

    e_tmp_graph4 << setprecision(10) << Pv_y_ref.transpose() << endl;
    e_tmp_graph5 << setprecision(10) << Planner_State_Prev_mpc_.row(3) << endl;
    e_tmp_graph6 << setprecision(10) << Planner_State_Prev_mpc_.row(5) << endl;
    e_tmp_graph2 << setprecision(10) << zmp_max_y_mpc_.transpose() << endl;
}

void AvatarController::IS_FIPM_CoM_Planner_MPC(double mpc_freq, double mpc_dt, double mpc_preview_window, int mpc_synchro_hz)
{
    double wpvx, wpvy, wpvz;
    double wdvx, wdvy, wdvz;

    wpvx = 1e+2; wpvy = 1e+6; wpvz = 1e+2;
    wdvx = 1e+0; wdvy = 1e+0; wdvz = 1e+0;

    int mpc_tick = walking_tick_mpc_ - com_start_tick_mpc_;
    const int N_plan_mpc = mpc_preview_window*mpc_freq;
    const int N_step = t_total_const_/mpc_synchro_hz;
    const int N_state = 3; //com position, com velocity, vrp position
    static int MPC_first_loop = 0;

    double lambda_is_calc = exp(-w_*mpc_dt);

    int input_num = 3*N_plan_mpc;
    //              vrp          
    int const_num = 6*N_plan_mpc + 3;
    //              vrp            IS

    if(MPC_first_loop == 0)
    {
        cout << "Initialization of IS FIPM Planner MPC." << endl;
        A_mpc_.resize(N_state,N_state);
        Eigen::MatrixXd A_mpc_cont; A_mpc_cont.resize(N_state, N_state); A_mpc_cont.setZero();
        A_mpc_cont <<     0, 1,      0,
                      w_*w_, 0, -w_*w_,
                          0, 0,      0;
        
        A_mpc_ = MatrixXd::Identity(N_state,N_state) 
               + A_mpc_cont*mpc_dt 
               + A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt/(1*2) 
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt*mpc_dt/(1*2*3)
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt*mpc_dt*mpc_dt/(1*2*3*4);

        B_mpc_.resize(N_state,1);
        Eigen::MatrixXd B_mpc_cont; B_mpc_cont.resize(N_state,1); B_mpc_cont.setZero();
        B_mpc_cont << 0,
                      0,
                      1;
        
        B_mpc_ = B_mpc_cont*mpc_dt 
               + A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt/(1*2) 
               + A_mpc_cont*A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt*mpc_dt/(1*2*3)
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt*mpc_dt*mpc_dt/(1*2*3*4);

        Ccp_mpc_.resize(1,N_state);
        Ccv_mpc_.resize(1,N_state);
        Cvp_mpc_.resize(1,N_state);

        Ccp_mpc_ << 1, 0, 0;
        Ccv_mpc_ << 0, 1, 0;
        Cvp_mpc_ << 0, 0, 1;

        Pcps_plan_mpc_.resize(N_plan_mpc,N_state);
        Pcvs_plan_mpc_.resize(N_plan_mpc,N_state);
        Pvps_plan_mpc_.resize(N_plan_mpc,N_state);

        Eigen::MatrixXd Ps_calc;
        Ps_calc.resize(N_state,N_state);
        Ps_calc = A_mpc_;
        
        Pcpu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Pcvu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Pvpu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        
        P_IS_step_mpc_.setZero(N_step, N_step);

        b_IS_plan_mpc_.setZero(N_plan_mpc,1);
        b_IS_step_mpc_.setZero(N_step,1);

        p_IS_step_mpc_.setOnes(N_step,1);
        
        Eigen::MatrixXd Pu_calc, Pu_step_calc;
        Pu_calc.setZero(N_state,N_plan_mpc);
        Pu_step_calc.setZero(N_state,N_step);
        
        for(int i = 0; i < N_plan_mpc; i++)
        {
            Pcps_plan_mpc_.row(i) = Ccp_mpc_*Ps_calc;
            Pcvs_plan_mpc_.row(i) = Ccv_mpc_*Ps_calc;
            Pvps_plan_mpc_.row(i) = Cvp_mpc_*Ps_calc;
            Ps_calc = Ps_calc*A_mpc_;

            Pu_calc.col(i) = B_mpc_;
            Pcpu_plan_mpc_.row(i) = Ccp_mpc_*Pu_calc;
            Pcvu_plan_mpc_.row(i) = Ccv_mpc_*Pu_calc;
            Pvpu_plan_mpc_.row(i) = Cvp_mpc_*Pu_calc;
            b_IS_plan_mpc_(i,0) = pow(lambda_is_calc, i);
            
            if(i < N_step)
            {
                Pu_step_calc.col(i)   = B_mpc_;
                P_IS_step_mpc_.row(i) = Cvp_mpc_*Pu_step_calc;    
                b_IS_step_mpc_(i,0)   = pow(lambda_is_calc, i);
            }

            Pu_calc = A_mpc_*Pu_calc;
            Pu_step_calc = A_mpc_*Pu_step_calc;
        }

        SUx_plan_mpc_.setZero(N_plan_mpc, input_num); SUx_plan_mpc_ << MatrixXd::Identity(N_plan_mpc, N_plan_mpc), MatrixXd::Zero(N_plan_mpc, N_plan_mpc), MatrixXd::Zero(N_plan_mpc, N_plan_mpc);
        SUy_plan_mpc_.setZero(N_plan_mpc, input_num); SUy_plan_mpc_ << MatrixXd::Zero(N_plan_mpc, N_plan_mpc), MatrixXd::Identity(N_plan_mpc, N_plan_mpc), MatrixXd::Zero(N_plan_mpc, N_plan_mpc);
        SUz_plan_mpc_.setZero(N_plan_mpc, input_num); SUz_plan_mpc_ << MatrixXd::Zero(N_plan_mpc, N_plan_mpc), MatrixXd::Zero(N_plan_mpc, N_plan_mpc), MatrixXd::Identity(N_plan_mpc, N_plan_mpc);

        ssx_plan_mpc_.setZero(N_state, 3*N_state); ssx_plan_mpc_ << MatrixXd::Identity(N_state, N_state), MatrixXd::Zero(N_state, N_state), MatrixXd::Zero(N_state, N_state);
        ssy_plan_mpc_.setZero(N_state, 3*N_state); ssy_plan_mpc_ << MatrixXd::Zero(N_state, N_state), MatrixXd::Identity(N_state, N_state), MatrixXd::Zero(N_state, N_state);
        ssz_plan_mpc_.setZero(N_state, 3*N_state); ssz_plan_mpc_ << MatrixXd::Zero(N_state, N_state), MatrixXd::Zero(N_state, N_state), MatrixXd::Identity(N_state, N_state);

        Qmat_plan_mpc_.resize(N_plan_mpc, N_plan_mpc);
        Qmat_plan_mpc_.setIdentity();

        Qxcalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Qxcalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvx*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvx*Qmat_plan_mpc_;

        Qycalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Qycalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvy*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvy*Qmat_plan_mpc_;

        Qzcalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Qzcalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvz*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvz*Qmat_plan_mpc_;

        Qcalc_plan_mpc_.setZero(input_num, input_num);
        Qcalc_plan_mpc_ = SUx_plan_mpc_.transpose()*Qxcalc_plan_mpc_*SUx_plan_mpc_ 
                        + SUy_plan_mpc_.transpose()*Qycalc_plan_mpc_*SUy_plan_mpc_
                        + SUz_plan_mpc_.transpose()*Qzcalc_plan_mpc_*SUz_plan_mpc_;
        
        gxcalc_plan_mpc_.setZero(input_num, N_plan_mpc);
        gxcalc_plan_mpc_ = SUx_plan_mpc_.transpose()*Pvpu_plan_mpc_.transpose()*wpvx*Qmat_plan_mpc_;

        gycalc_plan_mpc_.setZero(input_num, N_plan_mpc);
        gycalc_plan_mpc_ = SUy_plan_mpc_.transpose()*Pvpu_plan_mpc_.transpose()*wpvy*Qmat_plan_mpc_;

        gzcalc_plan_mpc_.setZero(input_num, N_plan_mpc);
        gzcalc_plan_mpc_ = SUz_plan_mpc_.transpose()*Pvpu_plan_mpc_.transpose()*wpvz*Qmat_plan_mpc_;

        QP_MPC_Planner_.InitializeProblemSize(input_num, const_num);

        MPC_Planner_u_mpc_.setZero(input_num);
        MPC_Planner_SQP_du_mpc_.setZero(input_num);

        MPC_Planner_u_mpc_sep_.setZero(3);

        Planner_State_Prev_mpc_.setZero(9, N_plan_mpc);

        Pv_dot_ref_mpc_.setZero(N_step, 3);
        
        zmp_max_x_mpc_.setZero(N_plan_mpc);
        zmp_min_x_mpc_.setZero(N_plan_mpc);

        zmp_max_y_mpc_.setZero(N_plan_mpc);
        zmp_min_y_mpc_.setZero(N_plan_mpc);

        t_total_mpc_ = t_total_const_;

        zmp_time_calc_x_.setZero(N_plan_mpc);
        zmp_time_calc_y_.setZero(N_plan_mpc);

        IS_FIPM_SQP_x_phi_N_plan_mpc_.setZero(3*N_plan_mpc*N_plan_mpc, 3*N_plan_mpc);
        IS_FIPM_SQP_x_pi_N_plan_mpc_.setZero (3*N_plan_mpc*N_plan_mpc, 3*N_state);
        IS_FIPM_SQP_x_pi2_N_plan_mpc_.setZero(3*N_plan_mpc*N_plan_mpc, 1);
        IS_FIPM_SQP_x_pi3_N_plan_mpc_.setZero(3*N_plan_mpc*N_plan_mpc, 1);
        IS_FIPM_SQP_x_ri_N_plan_mpc_.setZero (3*N_state*N_plan_mpc,    3*N_state);
        IS_FIPM_SQP_x_ri2_N_plan_mpc_.setZero(1*N_plan_mpc,            3*N_state);
        IS_FIPM_SQP_x_ri3_N_plan_mpc_.setZero(1*N_plan_mpc,            3*N_state);
        IS_FIPM_SQP_y_phi_N_plan_mpc_.setZero(3*N_plan_mpc*N_plan_mpc, 3*N_plan_mpc);
        IS_FIPM_SQP_y_pi_N_plan_mpc_.setZero (3*N_plan_mpc*N_plan_mpc, 3*N_state);
        IS_FIPM_SQP_y_pi2_N_plan_mpc_.setZero(3*N_plan_mpc*N_plan_mpc, 1);
        IS_FIPM_SQP_y_pi3_N_plan_mpc_.setZero(3*N_plan_mpc*N_plan_mpc, 1);
        IS_FIPM_SQP_y_ri_N_plan_mpc_.setZero (3*N_state*N_plan_mpc,    3*N_state);
        IS_FIPM_SQP_y_ri2_N_plan_mpc_.setZero(1*N_plan_mpc,            3*N_state);
        IS_FIPM_SQP_y_ri3_N_plan_mpc_.setZero(1*N_plan_mpc,            3*N_state);
        
        int calc_index  = 0;
        int calc_index2 = 0;
        Eigen::MatrixXd Si_mpc; Si_mpc.setZero(1, N_plan_mpc);

        for(int i = 0; i < N_plan_mpc; i++)
        {
            Si_mpc.setZero(1, N_plan_mpc);
            Si_mpc(0, i) = 1;

            IS_FIPM_SQP_x_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc) = (Si_mpc*Pvpu_plan_mpc_*SUx_plan_mpc_).transpose()*(Si_mpc*Pcpu_plan_mpc_*SUz_plan_mpc_)
                                                                                           - (Si_mpc*Pvpu_plan_mpc_*SUz_plan_mpc_).transpose()*(Si_mpc*Pcpu_plan_mpc_*SUx_plan_mpc_);

            IS_FIPM_SQP_y_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc) = (Si_mpc*Pvpu_plan_mpc_*SUy_plan_mpc_).transpose()*(Si_mpc*Pcpu_plan_mpc_*SUz_plan_mpc_)
                                                                                           - (Si_mpc*Pvpu_plan_mpc_*SUz_plan_mpc_).transpose()*(Si_mpc*Pcpu_plan_mpc_*SUy_plan_mpc_);

            IS_FIPM_SQP_x_pi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_state) = (Si_mpc*Pcpu_plan_mpc_*SUz_plan_mpc_).transpose()*(Si_mpc*Pvps_plan_mpc_*ssx_plan_mpc_)
                                                                                       + (Si_mpc*Pvpu_plan_mpc_*SUx_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssz_plan_mpc_)
                                                                                       - (Si_mpc*Pcpu_plan_mpc_*SUx_plan_mpc_).transpose()*(Si_mpc*Pvps_plan_mpc_*ssz_plan_mpc_)
                                                                                       - (Si_mpc*Pvpu_plan_mpc_*SUz_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssx_plan_mpc_);
            
            IS_FIPM_SQP_x_pi2_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1) = (GRAVITY*b_*b_*(Si_mpc*Pcpu_plan_mpc_*SUx_plan_mpc_)).transpose();

            IS_FIPM_SQP_x_pi3_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1) = (Si_mpc*(Pcpu_plan_mpc_ - Pvpu_plan_mpc_)*SUz_plan_mpc_).transpose();

            IS_FIPM_SQP_y_pi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_state) = (Si_mpc*Pcpu_plan_mpc_*SUz_plan_mpc_).transpose()*(Si_mpc*Pvps_plan_mpc_*ssy_plan_mpc_)
                                                                                       + (Si_mpc*Pvpu_plan_mpc_*SUy_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssz_plan_mpc_)
                                                                                       - (Si_mpc*Pcpu_plan_mpc_*SUy_plan_mpc_).transpose()*(Si_mpc*Pvps_plan_mpc_*ssz_plan_mpc_)
                                                                                       - (Si_mpc*Pvpu_plan_mpc_*SUz_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssy_plan_mpc_);

            IS_FIPM_SQP_y_pi2_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1) = (GRAVITY*b_*b_*(Si_mpc*Pcpu_plan_mpc_*SUy_plan_mpc_)).transpose();

            IS_FIPM_SQP_y_pi3_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1) = (Si_mpc*(Pcpu_plan_mpc_ - Pvpu_plan_mpc_)*SUz_plan_mpc_).transpose();

            calc_index += 3*N_plan_mpc;

            IS_FIPM_SQP_x_ri_N_plan_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state) = (Si_mpc*Pvps_plan_mpc_*ssx_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssz_plan_mpc_)
                                                                                     - (Si_mpc*Pvps_plan_mpc_*ssz_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssx_plan_mpc_);

            IS_FIPM_SQP_x_ri2_N_plan_mpc_.block(i, 0, 1, 3*N_state)                  = GRAVITY*b_*b_*(Si_mpc*Pcps_plan_mpc_*ssx_plan_mpc_);
            
            IS_FIPM_SQP_x_ri3_N_plan_mpc_.block(i, 0, 1, 3*N_state)                  = (Si_mpc*(Pcps_plan_mpc_ - Pvps_plan_mpc_)*ssz_plan_mpc_);

            IS_FIPM_SQP_y_ri_N_plan_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state) = (Si_mpc*Pvps_plan_mpc_*ssy_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssz_plan_mpc_)
                                                                                     - (Si_mpc*Pvps_plan_mpc_*ssz_plan_mpc_).transpose()*(Si_mpc*Pcps_plan_mpc_*ssy_plan_mpc_);

            IS_FIPM_SQP_y_ri2_N_plan_mpc_.block(i, 0, 1, 3*N_state)                  = GRAVITY*b_*b_*(Si_mpc*Pcps_plan_mpc_*ssy_plan_mpc_);

            IS_FIPM_SQP_y_ri3_N_plan_mpc_.block(i, 0, 1, 3*N_state)                  = (Si_mpc*(Pcps_plan_mpc_ - Pvps_plan_mpc_)*ssz_plan_mpc_);

            calc_index2 += 3*N_state;
        }

        MPC_first_loop = 1;
        cout << "Initialization of IS FIPM Planner MPC is completed." << endl;
    }

    Eigen::VectorXd Pv_x_ref(N_plan_mpc);
    Eigen::VectorXd Pv_y_ref(N_plan_mpc);
    Eigen::VectorXd Pv_z_ref(N_plan_mpc);

    Eigen::VectorXd zmp_max_x_time_plan_mpc(N_plan_mpc);
    Eigen::VectorXd zmp_min_x_time_plan_mpc(N_plan_mpc);

    Eigen::VectorXd zmp_max_y_time_plan_mpc(N_plan_mpc);
    Eigen::VectorXd zmp_min_y_time_plan_mpc(N_plan_mpc);

    for(int i = 0; i < N_plan_mpc; i++)
    {
        Pv_x_ref(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0);
        Pv_y_ref(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1);
        Pv_z_ref(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),2);

        int step_time_adj_calc = max(MPC_Stabilizer_time_adj_tick_x_mpc_, MPC_Stabilizer_time_adj_tick_y_mpc_);
        bool nnext_step_prev_bool = (bool)(mpc_tick + mpc_synchro_hz*(i + 2 + step_time_adj_calc) > (2*t_total_const_ - t_dsp2_));

        zmp_time_calc_x_(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + 1 + (1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),0);
        zmp_time_calc_y_(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + 1 + (1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),1);

        if(i < N_step)
        {
            Pv_dot_ref_mpc_(i,0) = (ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),0) - ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),0))/mpc_dt;
            Pv_dot_ref_mpc_(i,1) = (ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),1) - ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),1))/mpc_dt;
            Pv_dot_ref_mpc_(i,2) = (ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),2) - ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),2))/mpc_dt;
        }

        zmp_max_x_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0) + zmp_x_max;     
        zmp_min_x_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0) - zmp_x_min;

        zmp_max_y_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1) + zmp_y_max;
        zmp_min_y_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1) - zmp_y_min;

        zmp_max_x_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),0) + zmp_x_max;
        zmp_min_x_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),0) - zmp_x_min;
        
        zmp_max_y_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),1) + zmp_y_max;
        zmp_min_y_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),1) - zmp_y_min;
    }

    gcalc_plan_mpc_ = gxcalc_plan_mpc_*(Pvps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_ - zmp_time_calc_x_)
                    + gycalc_plan_mpc_*(Pvps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_ - zmp_time_calc_y_)
                    + gzcalc_plan_mpc_*(Pvps_plan_mpc_*ssz_plan_mpc_*MPC_Planner_state_mpc_ - Pv_z_ref);

    SQP_deldel_Qcalc_plan_mpc_ = Qcalc_plan_mpc_;
    SQP_del_g_calc_plan_mpc_   = Qcalc_plan_mpc_*MPC_Planner_u_mpc_ + gcalc_plan_mpc_;

    QP_MPC_Planner_.EnableEqualityCondition(equality_condition_eps_);
    //QP_MPC_Planner_.UpdateMinProblem(SQP_deldel_Qcalc_plan_mpc_, SQP_del_g_calc_plan_mpc_);
    QP_MPC_Planner_.UpdateMinProblem(Qcalc_plan_mpc_, gcalc_plan_mpc_);
    QP_MPC_Planner_.DeleteSubjectToAx();
    QP_MPC_Planner_.DeleteSubjectToX();

    const_A_mpc_.setZero( const_num, input_num);
    const_ub_mpc_.setZero(const_num, 1);
    const_lb_mpc_.setZero(const_num, 1);

    int constraint_index = 0;
    int calc_index = 0;
    int calc_index2 = 0;
    ////VRP Constraint
    Eigen::MatrixXd Si_mpc; Si_mpc.setZero(1, N_plan_mpc);
    /*
    for(int i = 0; i < N_plan_mpc; i++)
    {
        const_SQP_phi_mpc_.setZero(input_num, input_num);
        const_SQP_phi_mpc_calc_.setZero(input_num, input_num);
        const_SQP_pi_mpc_.setZero(input_num, 1);
        const_SQP_ri_mpc_.setZero(1, 1);

        Si_mpc.setZero(1, N_plan_mpc);
        Si_mpc(0, i) = 1;

        //X max
        const_SQP_phi_mpc_ = 0.5*(IS_FIPM_SQP_x_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc)
                                 +IS_FIPM_SQP_x_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc).transpose());

        const_SQP_pi_mpc_  = IS_FIPM_SQP_x_pi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_state)*MPC_Planner_state_mpc_

                           + IS_FIPM_SQP_x_pi2_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1)

                           - zmp_max_x_time_plan_mpc(i)*IS_FIPM_SQP_x_pi3_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1);

        const_SQP_ri_mpc_ = MPC_Planner_state_mpc_.transpose()*IS_FIPM_SQP_x_ri_N_plan_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Planner_state_mpc_
                          
                          + IS_FIPM_SQP_x_ri2_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_

                          - zmp_max_x_time_plan_mpc(i)*IS_FIPM_SQP_x_ri3_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_
                          
                          - GRAVITY*b_*b_*zmp_max_x_time_plan_mpc(i)*MatrixXd::Identity(1,1);

        const_SQP_hi_mpc_ = MPC_Planner_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;

        const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - 1e+3*MatrixXd::Identity(1,1);
        constraint_index += 1;

        //X min
        const_SQP_phi_mpc_ = 0.5*(IS_FIPM_SQP_x_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc)
                                 +IS_FIPM_SQP_x_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc).transpose());

        const_SQP_pi_mpc_  = IS_FIPM_SQP_x_pi_N_plan_mpc_.block (calc_index, 0, 3*N_plan_mpc, 3*N_state)*MPC_Planner_state_mpc_

                           + IS_FIPM_SQP_x_pi2_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1)

                           - zmp_min_x_time_plan_mpc(i)*IS_FIPM_SQP_x_pi3_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1);

        const_SQP_ri_mpc_  = MPC_Planner_state_mpc_.transpose()*IS_FIPM_SQP_x_ri_N_plan_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Planner_state_mpc_

                           + IS_FIPM_SQP_x_ri2_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_

                           - zmp_min_x_time_plan_mpc(i)*IS_FIPM_SQP_x_ri3_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_
                          
                           - GRAVITY*b_*b_*zmp_min_x_time_plan_mpc(i)*MatrixXd::Identity(1,1);

        const_SQP_hi_mpc_  = MPC_Planner_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;

        const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = + 1e+3*MatrixXd::Identity(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        constraint_index += 1;

        //Y direction
        //Y max
        const_SQP_phi_mpc_ = 0.5*(IS_FIPM_SQP_y_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc)
                                 +IS_FIPM_SQP_y_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc).transpose());

        const_SQP_pi_mpc_  = IS_FIPM_SQP_y_pi_N_plan_mpc_.block (calc_index, 0, 3*N_plan_mpc, 3*N_state)*MPC_Planner_state_mpc_

                           + IS_FIPM_SQP_y_pi2_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1)

                           - zmp_max_y_time_plan_mpc(i)*IS_FIPM_SQP_y_pi3_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1);

        const_SQP_ri_mpc_  = MPC_Planner_state_mpc_.transpose()*IS_FIPM_SQP_y_ri_N_plan_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Planner_state_mpc_

                           + IS_FIPM_SQP_y_ri2_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_

                           - zmp_max_y_time_plan_mpc(i)*IS_FIPM_SQP_y_ri3_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_
                          
                           - GRAVITY*b_*b_*zmp_max_y_time_plan_mpc(i)*MatrixXd::Identity(1,1);

        const_SQP_hi_mpc_  = MPC_Planner_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;

        const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - 1e+3*MatrixXd::Identity(1,1);
        constraint_index += 1;

        //Y min
        const_SQP_phi_mpc_ = 0.5*(IS_FIPM_SQP_y_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc)
                                 +IS_FIPM_SQP_y_phi_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 3*N_plan_mpc).transpose());

        const_SQP_pi_mpc_  = IS_FIPM_SQP_y_pi_N_plan_mpc_.block (calc_index, 0, 3*N_plan_mpc, 3*N_state)*MPC_Planner_state_mpc_

                           + IS_FIPM_SQP_y_pi2_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1)

                           - zmp_min_y_time_plan_mpc(i)*IS_FIPM_SQP_y_pi3_N_plan_mpc_.block(calc_index, 0, 3*N_plan_mpc, 1);

        const_SQP_ri_mpc_  = MPC_Planner_state_mpc_.transpose()*IS_FIPM_SQP_y_ri_N_plan_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Planner_state_mpc_

                           + IS_FIPM_SQP_y_ri2_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_

                           - zmp_min_y_time_plan_mpc(i)*IS_FIPM_SQP_y_ri3_N_plan_mpc_.block(i, 0, 1, 3*N_state)*MPC_Planner_state_mpc_
                          
                           - GRAVITY*b_*b_*zmp_min_y_time_plan_mpc(i)*MatrixXd::Identity(1,1);

        const_SQP_hi_mpc_  = MPC_Planner_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;

        const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = + 1e+3*MatrixXd::Identity(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        constraint_index += 1;

        calc_index  += 3*N_plan_mpc;
        calc_index2 += 3*N_state;
    }
    */
    //IS EQ
    Eigen::MatrixXd Const_b_eq_;  Const_b_eq_.setZero(3,1);

    Const_b_eq_(0,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(0) + MPC_Planner_state_mpc_(1)/w_ - MPC_Planner_state_mpc_(2))
                       -(pow(lambda_is_calc, N_plan_mpc)/(1 - pow(lambda_is_calc,N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(0))(0,0));
    Const_b_eq_(1,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(3) + MPC_Planner_state_mpc_(4)/w_ - MPC_Planner_state_mpc_(5))
                       -(pow(lambda_is_calc, N_plan_mpc)/(1 + pow(lambda_is_calc,N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(1))(0,0));
    Const_b_eq_(2,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(6) + MPC_Planner_state_mpc_(7)/w_ - MPC_Planner_state_mpc_(8));

    //X direction
    const_SQP_phi_mpc_.setZero(input_num, input_num);
    const_SQP_pi_mpc_ = (b_IS_plan_mpc_.transpose()*SUx_plan_mpc_).transpose();
    const_SQP_ri_mpc_ = - Const_b_eq_.block(0, 0, 1, 1);
    //const_SQP_hi_mpc_ = MPC_Planner_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;
    const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;
    
    //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_).transpose();
    const_A_mpc_.row(constraint_index) = (const_SQP_pi_mpc_).transpose();
    //const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
    //const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
    const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_ri_mpc_;
    const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_ri_mpc_;
    constraint_index += 1;
    //Y direction
    const_SQP_phi_mpc_.setZero(input_num, input_num);
    const_SQP_pi_mpc_ = (b_IS_plan_mpc_.transpose()*SUy_plan_mpc_).transpose();
    const_SQP_ri_mpc_ = - Const_b_eq_.block(1, 0, 1, 1);
    //const_SQP_hi_mpc_ = MPC_Planner_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;
    const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;

    //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_).transpose();
    const_A_mpc_.row(constraint_index) = (const_SQP_pi_mpc_).transpose();
    //const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
    //const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
    const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_ri_mpc_;
    const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_ri_mpc_;
    constraint_index += 1;
    //Z direction
    const_SQP_phi_mpc_.setZero(input_num, input_num);
    const_SQP_pi_mpc_ = (b_IS_plan_mpc_.transpose()*SUz_plan_mpc_).transpose();
    const_SQP_ri_mpc_ = - Const_b_eq_.block(2, 0, 1, 1);
    //const_SQP_hi_mpc_ = MPC_Planner_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;
    const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Planner_u_mpc_ + const_SQP_ri_mpc_;

    //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Planner_u_mpc_ + const_SQP_pi_mpc_).transpose();
    const_A_mpc_.row(constraint_index) = (const_SQP_pi_mpc_).transpose();
    //const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
    //const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
    const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_ri_mpc_;
    const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_ri_mpc_;
    constraint_index += 1;

    QP_MPC_Planner_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);

    if(QP_MPC_Planner_.SolveQPoases(100, MPC_Planner_SQP_du_mpc_))
    {
        //if((walking_tick_mpc_ - mpc_synchro_hz - 20)%int(2*hz_) == 0)
        if((((walking_tick_mpc_ - int(mpc_synchro_hz) + 1)/int(mpc_synchro_hz))%60) == 0) //30hz
        { cout << "IS FIPM Planner MPC Solved" << endl;; }
        
        //MPC_Planner_u_mpc_ = MPC_Planner_u_mpc_ + MPC_Planner_SQP_du_mpc_;
        MPC_Planner_u_mpc_ = MPC_Planner_SQP_du_mpc_;

        Planner_State_Prev_mpc_.row(0).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_.segment(0*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(1).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_.segment(0*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(2).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_.segment(0*N_plan_mpc, N_plan_mpc);

        Planner_State_Prev_mpc_.row(3).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_.segment(1*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(4).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_.segment(1*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(5).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_.segment(1*N_plan_mpc, N_plan_mpc);

        Planner_State_Prev_mpc_.row(6).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_.segment(2*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(7).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_.segment(2*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(8).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_.segment(2*N_plan_mpc, N_plan_mpc);

        MPC_Planner_state_mpc_.segment(0,3) = A_mpc_*MPC_Planner_state_mpc_.segment(0,3) + B_mpc_*MPC_Planner_u_mpc_(0*N_plan_mpc);
        MPC_Planner_state_mpc_.segment(3,3) = A_mpc_*MPC_Planner_state_mpc_.segment(3,3) + B_mpc_*MPC_Planner_u_mpc_(1*N_plan_mpc);
        MPC_Planner_state_mpc_.segment(6,3) = A_mpc_*MPC_Planner_state_mpc_.segment(6,3) + B_mpc_*MPC_Planner_u_mpc_(2*N_plan_mpc);

        MPC_Planner_u_mpc_sep_(0) = MPC_Planner_u_mpc_(0*N_plan_mpc);
        MPC_Planner_u_mpc_sep_(1) = MPC_Planner_u_mpc_(1*N_plan_mpc);
        MPC_Planner_u_mpc_sep_(2) = MPC_Planner_u_mpc_(2*N_plan_mpc);
    }
    else
    { 
        cout << "IS FIPM Planner MPC Not Solved" << endl;
        cout << int(walking_tick_mpc_ - 20)/mpc_synchro_hz << endl;

        MPC_Planner_u_mpc_ = MPC_Planner_u_mpc_ + MPC_Planner_SQP_du_mpc_;

        Planner_State_Prev_mpc_.row(0).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_.segment(0*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(1).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_.segment(0*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(2).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_.segment(0*N_plan_mpc, N_plan_mpc);

        Planner_State_Prev_mpc_.row(3).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_.segment(1*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(4).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_.segment(1*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(5).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_.segment(1*N_plan_mpc, N_plan_mpc);

        Planner_State_Prev_mpc_.row(6).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_.segment(2*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(7).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_.segment(2*N_plan_mpc, N_plan_mpc);
        Planner_State_Prev_mpc_.row(8).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_.segment(2*N_plan_mpc, N_plan_mpc);

        MPC_Planner_state_mpc_.segment(0,3) = A_mpc_*MPC_Planner_state_mpc_.segment(0,3) + B_mpc_*MPC_Planner_u_mpc_(0*N_plan_mpc);
        MPC_Planner_state_mpc_.segment(3,3) = A_mpc_*MPC_Planner_state_mpc_.segment(3,3) + B_mpc_*MPC_Planner_u_mpc_(1*N_plan_mpc);
        MPC_Planner_state_mpc_.segment(6,3) = A_mpc_*MPC_Planner_state_mpc_.segment(6,3) + B_mpc_*MPC_Planner_u_mpc_(2*N_plan_mpc);

        MPC_Planner_u_mpc_sep_(0) = MPC_Planner_u_mpc_(0*N_plan_mpc);
        MPC_Planner_u_mpc_sep_(1) = MPC_Planner_u_mpc_(1*N_plan_mpc);
        MPC_Planner_u_mpc_sep_(2) = MPC_Planner_u_mpc_(2*N_plan_mpc);
    }

    e_mpc_planner_data << N_plan_mpc                << "," << 0                         << "," << 0                         << ","
                       << Pv_x_ref(0)               << "," << Pv_y_ref(0)               << "," << Pv_z_ref(0)               << ","
                       << MPC_Planner_state_mpc_(0) << "," << MPC_Planner_state_mpc_(3) << "," << MPC_Planner_state_mpc_(6) << ","
                       << MPC_Planner_state_mpc_(1) << "," << MPC_Planner_state_mpc_(4) << "," << MPC_Planner_state_mpc_(7) << ","
                       << MPC_Planner_state_mpc_(2) << "," << MPC_Planner_state_mpc_(5) << "," << MPC_Planner_state_mpc_(8) << ","
                       << zmp_max_x_mpc_(0)         << "," << zmp_max_y_mpc_(0)         << "," << 0                         << ","
                       << endl;

    Eigen::VectorXd data_save_calc; data_save_calc.setZero(3*N_plan_mpc);
    data_save_calc << Pv_x_ref, Pv_y_ref, Pv_z_ref;
    e_tmp_graph2 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_time_calc_x_, zmp_time_calc_y_, 0*zmp_time_calc_y_;
    e_tmp_graph3 << data_save_calc.transpose() << endl;
    data_save_calc << Planner_State_Prev_mpc_.row(0).transpose(), Planner_State_Prev_mpc_.row(3).transpose(), Planner_State_Prev_mpc_.row(6).transpose();
    e_tmp_graph4 << data_save_calc.transpose() << endl; 
    data_save_calc << Planner_State_Prev_mpc_.row(2).transpose(), Planner_State_Prev_mpc_.row(5).transpose(), Planner_State_Prev_mpc_.row(8).transpose();
    e_tmp_graph5 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_max_x_mpc_, zmp_max_y_mpc_, 0*zmp_max_y_mpc_;
    e_tmp_graph6 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_max_x_time_plan_mpc, zmp_max_y_time_plan_mpc, 0*zmp_max_y_time_plan_mpc;
    e_tmp_graph7 << data_save_calc.transpose() << endl;
}

void AvatarController::IS_FIPM_CoM_Seq_Planner_MPC(double mpc_freq, double mpc_dt, double mpc_preview_window, int mpc_synchro_hz)
{
    double wpvx, wpvy, wpvz;
    double wdvx, wdvy, wdvz;

    wpvx = 1e+2; wpvy = 1e+6; wpvz = 1e+2;
    wdvx = 1e+0; wdvy = 1e+0; wdvz = 1e+0;

    int mpc_tick = walking_tick_mpc_ - com_start_tick_mpc_;
    const int N_plan_mpc = mpc_preview_window*mpc_freq;
    const int N_step = t_total_const_/mpc_synchro_hz;
    const int N_state = 3; //com position, com velocity, vrp position
    static int MPC_first_loop = 0;

    double lambda_is_calc = exp(-w_*mpc_dt);

    int input_num = N_plan_mpc;
    //              vrp          
    int const_num = N_plan_mpc + 1;
    //              vrp          IS

    if(MPC_first_loop == 0)
    {
        cout << "Initialization of IS FIPM Seq Planner MPC." << endl;
        A_mpc_.resize(N_state,N_state);
        Eigen::MatrixXd A_mpc_cont; A_mpc_cont.resize(N_state, N_state); A_mpc_cont.setZero();
        A_mpc_cont <<     0, 1,      0,
                      w_*w_, 0, -w_*w_,
                          0, 0,      0;
        
        A_mpc_ = MatrixXd::Identity(N_state,N_state) 
               + A_mpc_cont*mpc_dt 
               + A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt/(1*2) 
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt*mpc_dt/(1*2*3)
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*A_mpc_cont*mpc_dt*mpc_dt*mpc_dt*mpc_dt/(1*2*3*4);

        B_mpc_.resize(N_state,1);
        Eigen::MatrixXd B_mpc_cont; B_mpc_cont.resize(N_state,1); B_mpc_cont.setZero();
        B_mpc_cont << 0,
                      0,
                      1;
        
        B_mpc_ = B_mpc_cont*mpc_dt 
               + A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt/(1*2) 
               + A_mpc_cont*A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt*mpc_dt/(1*2*3)
               + A_mpc_cont*A_mpc_cont*A_mpc_cont*B_mpc_cont*mpc_dt*mpc_dt*mpc_dt*mpc_dt/(1*2*3*4);

        Ccp_mpc_.resize(1,N_state);
        Ccv_mpc_.resize(1,N_state);
        Cvp_mpc_.resize(1,N_state);

        Ccp_mpc_ << 1, 0, 0;
        Ccv_mpc_ << 0, 1, 0;
        Cvp_mpc_ << 0, 0, 1;

        Pcps_plan_mpc_.resize(N_plan_mpc,N_state);
        Pcvs_plan_mpc_.resize(N_plan_mpc,N_state);
        Pvps_plan_mpc_.resize(N_plan_mpc,N_state);

        Eigen::MatrixXd Ps_calc;
        Ps_calc.resize(N_state,N_state);
        Ps_calc = A_mpc_;
        
        Pcpu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Pcvu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Pvpu_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        
        P_IS_step_mpc_.setZero(N_step, N_step);

        b_IS_plan_mpc_.setZero(N_plan_mpc,1);
        b_IS_step_mpc_.setZero(N_step,1);

        p_IS_step_mpc_.setOnes(N_step,1);
        
        Eigen::MatrixXd Pu_calc, Pu_step_calc;
        Pu_calc.setZero(N_state,N_plan_mpc);
        Pu_step_calc.setZero(N_state,N_step);
        
        for(int i = 0; i < N_plan_mpc; i++)
        {
            Pcps_plan_mpc_.row(i) = Ccp_mpc_*Ps_calc;
            Pcvs_plan_mpc_.row(i) = Ccv_mpc_*Ps_calc;
            Pvps_plan_mpc_.row(i) = Cvp_mpc_*Ps_calc;
            Ps_calc = Ps_calc*A_mpc_;

            Pu_calc.col(i) = B_mpc_;
            Pcpu_plan_mpc_.row(i) = Ccp_mpc_*Pu_calc;
            Pcvu_plan_mpc_.row(i) = Ccv_mpc_*Pu_calc;
            Pvpu_plan_mpc_.row(i) = Cvp_mpc_*Pu_calc;
            b_IS_plan_mpc_(i,0) = pow(lambda_is_calc, i);
            
            if(i < N_step)
            {
                Pu_step_calc.col(i)   = B_mpc_;
                P_IS_step_mpc_.row(i) = Cvp_mpc_*Pu_step_calc;    
                b_IS_step_mpc_(i,0)   = pow(lambda_is_calc, i);
            }

            Pu_calc = A_mpc_*Pu_calc;
            Pu_step_calc = A_mpc_*Pu_step_calc;
        }

        ssx_plan_mpc_.setZero(N_state, 3*N_state); ssx_plan_mpc_ << MatrixXd::Identity(N_state, N_state), MatrixXd::Zero(N_state, N_state), MatrixXd::Zero(N_state, N_state);
        ssy_plan_mpc_.setZero(N_state, 3*N_state); ssy_plan_mpc_ << MatrixXd::Zero(N_state, N_state), MatrixXd::Identity(N_state, N_state), MatrixXd::Zero(N_state, N_state);
        ssz_plan_mpc_.setZero(N_state, 3*N_state); ssz_plan_mpc_ << MatrixXd::Zero(N_state, N_state), MatrixXd::Zero(N_state, N_state), MatrixXd::Identity(N_state, N_state);

        Qmat_plan_mpc_.resize(N_plan_mpc, N_plan_mpc);
        Qmat_plan_mpc_.setIdentity();

        Qxcalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);        
        Qxcalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvx*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvx*Qmat_plan_mpc_;
    
        Qycalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Qycalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvy*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvy*Qmat_plan_mpc_;

        Qzcalc_plan_mpc_.setZero(N_plan_mpc, N_plan_mpc);
        Qzcalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvz*Qmat_plan_mpc_*Pvpu_plan_mpc_ + wdvz*Qmat_plan_mpc_;
        
        gxcalc_plan_mpc_.setZero(input_num, N_plan_mpc);
        gxcalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvx*Qmat_plan_mpc_;

        gycalc_plan_mpc_.setZero(input_num, N_plan_mpc);        
        gycalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvy*Qmat_plan_mpc_;

        gzcalc_plan_mpc_.setZero(input_num, N_plan_mpc);
        gzcalc_plan_mpc_ = Pvpu_plan_mpc_.transpose()*wpvz*Qmat_plan_mpc_;

        QP_MPC_Planner_.InitializeProblemSize(input_num, const_num);
        QP_SEQ_MPC_Planner_.InitializeProblemSize(input_num, const_num);        

        MPC_Planner_u_mpc_.setZero(input_num);
        MPC_Planner_SQP_du_mpc_.setZero(input_num);

        MPC_Planner_u_mpc_sep_.setZero(3);

        Planner_State_Prev_mpc_.setZero(9, N_plan_mpc);

        Pv_dot_ref_mpc_.setZero(N_step, 3);
        
        zmp_max_x_mpc_.setZero(N_plan_mpc);
        zmp_min_x_mpc_.setZero(N_plan_mpc);

        zmp_max_y_mpc_.setZero(N_plan_mpc);
        zmp_min_y_mpc_.setZero(N_plan_mpc);

        t_total_mpc_ = t_total_const_;

        zmp_time_calc_x_.setZero(N_plan_mpc);
        zmp_time_calc_y_.setZero(N_plan_mpc);

        MPC_first_loop = 1;
        cout << "Initialization of IS FIPM Seq Planner MPC is completed." << endl;
    }

    Eigen::VectorXd Pv_x_ref(N_plan_mpc);
    Eigen::VectorXd Pv_y_ref(N_plan_mpc);
    Eigen::VectorXd Pv_z_ref(N_plan_mpc);

    Eigen::VectorXd zmp_max_x_time_plan_mpc(N_plan_mpc);
    Eigen::VectorXd zmp_min_x_time_plan_mpc(N_plan_mpc);

    Eigen::VectorXd zmp_max_y_time_plan_mpc(N_plan_mpc);
    Eigen::VectorXd zmp_min_y_time_plan_mpc(N_plan_mpc);

    for(int i = 0; i < N_plan_mpc; i++)
    {
        Pv_x_ref(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0);
        Pv_y_ref(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1);
        Pv_z_ref(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i+1),2);

        int step_time_adj_calc = max(MPC_Stabilizer_time_adj_tick_x_mpc_, MPC_Stabilizer_time_adj_tick_y_mpc_);
        bool nnext_step_prev_bool = (bool)(mpc_tick + mpc_synchro_hz*(i + 2 + step_time_adj_calc) > (2*t_total_const_ - t_dsp2_));

        zmp_time_calc_x_(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + 1 + (1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),0);
        zmp_time_calc_y_(i) = ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + 1 + (1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),1);

        if(i < N_step)
        {
            Pv_dot_ref_mpc_(i,0) = (ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),0) - ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),0))/mpc_dt;
            Pv_dot_ref_mpc_(i,1) = (ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),1) - ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),1))/mpc_dt;
            Pv_dot_ref_mpc_(i,2) = (ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 1),2) - ref_vrp_mpc_(mpc_tick + mpc_synchro_hz*(i + N_plan_mpc + 0),2))/mpc_dt;
        }

        zmp_max_x_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0) + zmp_x_max;     
        zmp_min_x_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),0) - zmp_x_min;

        zmp_max_y_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1) + zmp_y_max;
        zmp_min_y_mpc_(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1),1) - zmp_y_min;

        zmp_max_x_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),0) + zmp_x_max;
        zmp_min_x_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),0) - zmp_x_min;
        
        zmp_max_y_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),1) + zmp_y_max;
        zmp_min_y_time_plan_mpc(i) = ref_zmp_wo_offset_mpc_(mpc_tick + mpc_synchro_hz*(i+1+(1 - nnext_step_prev_bool)*step_enable_bool_mpc_*step_time_adj_calc),1) - zmp_y_min;
    }

    const_A_mpc_.setZero( const_num, input_num);
    const_ub_mpc_.setZero(const_num, 1);
    const_lb_mpc_.setZero(const_num, 1);

    int constraint_index = 0;

    Eigen::MatrixXd Const_b_eq_;

    //Z direction
    gcalc_plan_mpc_ = gzcalc_plan_mpc_*(Pvps_plan_mpc_*ssz_plan_mpc_*MPC_Planner_state_mpc_ - Pv_z_ref);

    QP_MPC_Planner_.EnableEqualityCondition(equality_condition_eps_);
    QP_MPC_Planner_.UpdateMinProblem(Qzcalc_plan_mpc_, gcalc_plan_mpc_);
    QP_MPC_Planner_.DeleteSubjectToAx();
    QP_MPC_Planner_.DeleteSubjectToX();

    constraint_index = 0;
    constraint_index += N_plan_mpc;

    Const_b_eq_.setZero(1,1);
    Const_b_eq_(0,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(6) + MPC_Planner_state_mpc_(7)/w_ - MPC_Planner_state_mpc_(8));

    const_A_mpc_.row(   constraint_index)          = b_IS_plan_mpc_.transpose();
    const_ub_mpc_.block(constraint_index, 0, 1, 1) = Const_b_eq_.block(0, 0, 1, 1);
    const_lb_mpc_.block(constraint_index, 0, 1, 1) = Const_b_eq_.block(0, 0, 1, 1);
    constraint_index += 1;

    QP_MPC_Planner_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);

    if(QP_MPC_Planner_.SolveQPoases(100, MPC_Planner_u_mpc_))
    {
        if((((walking_tick_mpc_ - int(mpc_synchro_hz) + 1)/int(mpc_synchro_hz))%60) == 0) //30hz
        { cout << "IS FIPM Planner MPC Z direction Solved" << endl;; }

        Planner_State_Prev_mpc_.row(6).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(7).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(8).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_;

        MPC_Planner_state_mpc_.segment(6,3) = A_mpc_*MPC_Planner_state_mpc_.segment(6,3) + B_mpc_*MPC_Planner_u_mpc_(0);
    }
    else
    { 
        cout << "IS FIPM Planner MPC Z direction Not Solved" << endl;
        cout << int(walking_tick_mpc_ - 20)/mpc_synchro_hz << endl;

        Planner_State_Prev_mpc_.row(6).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(7).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(8).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(6,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_;

        MPC_Planner_state_mpc_.segment(6,3) = A_mpc_*MPC_Planner_state_mpc_.segment(6,3) + B_mpc_*MPC_Planner_u_mpc_(0);
    }

    MPC_Planner_u_mpc_sep_(2) = MPC_Planner_u_mpc_(0);

    Eigen::VectorXd w_square_over_lambda; w_square_over_lambda.setZero(N_plan_mpc);
    w_square_over_lambda = (w_*w_*Planner_State_Prev_mpc_.row(6)).array()/(w_*w_*(Planner_State_Prev_mpc_.row(6) - Planner_State_Prev_mpc_.row(8)) + GRAVITY*MatrixXd::Ones(1, N_plan_mpc)).array();

    //X direction
    gcalc_plan_mpc_ = gxcalc_plan_mpc_*(Pvps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_ - zmp_time_calc_x_);

    QP_MPC_Planner_.EnableEqualityCondition(equality_condition_eps_);
    QP_MPC_Planner_.UpdateMinProblem(Qxcalc_plan_mpc_, gcalc_plan_mpc_);
    QP_MPC_Planner_.DeleteSubjectToAx();
    QP_MPC_Planner_.DeleteSubjectToX();

    constraint_index = 0;
    const_A_mpc_.block( constraint_index, 0, N_plan_mpc, input_num) = (VectorXd::Ones(N_plan_mpc) - w_square_over_lambda).asDiagonal()*Pcpu_plan_mpc_ 
                                                                    + w_square_over_lambda.asDiagonal()*Pvpu_plan_mpc_;
    const_ub_mpc_.block(constraint_index, 0, N_plan_mpc, 1)         = zmp_max_x_time_plan_mpc 
                                                                    - (VectorXd::Ones(N_plan_mpc) - w_square_over_lambda).asDiagonal()*Pcps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_
                                                                    - w_square_over_lambda.asDiagonal()*Pvps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_;
    const_lb_mpc_.block(constraint_index, 0, N_plan_mpc, 1)         = zmp_min_x_time_plan_mpc 
                                                                    - (VectorXd::Ones(N_plan_mpc) - w_square_over_lambda).asDiagonal()*Pcps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_
                                                                    - w_square_over_lambda.asDiagonal()*Pvps_plan_mpc_*ssx_plan_mpc_*MPC_Planner_state_mpc_;

    constraint_index += N_plan_mpc;

    Const_b_eq_.setZero(1,1);
    Const_b_eq_(0,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(0) + MPC_Planner_state_mpc_(1)/w_ - MPC_Planner_state_mpc_(2))
                      -(pow(lambda_is_calc, N_plan_mpc)/(1 - pow(lambda_is_calc,N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(0))(0,0));

    const_A_mpc_.row(   constraint_index)          = b_IS_plan_mpc_.transpose();
    const_ub_mpc_.block(constraint_index, 0, 1, 1) = Const_b_eq_.block(0, 0, 1, 1);
    const_lb_mpc_.block(constraint_index, 0, 1, 1) = Const_b_eq_.block(0, 0, 1, 1);
    constraint_index += 1;

    QP_MPC_Planner_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);

    if(QP_MPC_Planner_.SolveQPoases(100, MPC_Planner_u_mpc_))
    {
        if((((walking_tick_mpc_ - int(mpc_synchro_hz) + 1)/int(mpc_synchro_hz))%60) == 0) //30hz
        { cout << "IS FIPM Planner MPC X direction Solved" << endl; }

        Planner_State_Prev_mpc_.row(0).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(1).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(2).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_;

        MPC_Planner_state_mpc_.segment(0,3) = A_mpc_*MPC_Planner_state_mpc_.segment(0,3) + B_mpc_*MPC_Planner_u_mpc_(0);
    }
    else
    { 
        cout << "IS FIPM Planner MPC X direction Not Solved" << endl;
        cout << int(walking_tick_mpc_ - 20)/mpc_synchro_hz << endl;

        Planner_State_Prev_mpc_.row(0).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(1).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(2).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(0,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_;

        MPC_Planner_state_mpc_.segment(0,3) = A_mpc_*MPC_Planner_state_mpc_.segment(0,3) + B_mpc_*MPC_Planner_u_mpc_(0);
    }

    MPC_Planner_u_mpc_sep_(0) = MPC_Planner_u_mpc_(0);

    //Y direction
    gcalc_plan_mpc_ = gycalc_plan_mpc_*(Pvps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_ - zmp_time_calc_y_);

    QP_MPC_Planner_.EnableEqualityCondition(equality_condition_eps_);
    QP_MPC_Planner_.UpdateMinProblem(Qycalc_plan_mpc_, gcalc_plan_mpc_);
    QP_MPC_Planner_.DeleteSubjectToAx();
    QP_MPC_Planner_.DeleteSubjectToX();

    constraint_index = 0;
    const_A_mpc_.block( constraint_index, 0, N_plan_mpc, input_num) = (VectorXd::Ones(N_plan_mpc) - w_square_over_lambda).asDiagonal()*Pcpu_plan_mpc_ 
                                                                    + w_square_over_lambda.asDiagonal()*Pvpu_plan_mpc_;
    const_ub_mpc_.block(constraint_index, 0, N_plan_mpc, 1)         = zmp_max_y_time_plan_mpc 
                                                                    - (VectorXd::Ones(N_plan_mpc) - w_square_over_lambda).asDiagonal()*Pcps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_
                                                                    - w_square_over_lambda.asDiagonal()*Pvps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_;
    const_lb_mpc_.block(constraint_index, 0, N_plan_mpc, 1)         = zmp_min_y_time_plan_mpc 
                                                                    - (VectorXd::Ones(N_plan_mpc) - w_square_over_lambda).asDiagonal()*Pcps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_
                                                                    - w_square_over_lambda.asDiagonal()*Pvps_plan_mpc_*ssy_plan_mpc_*MPC_Planner_state_mpc_;

    constraint_index += N_plan_mpc;

    Const_b_eq_.setZero(1,1);
    Const_b_eq_(0,0) = (w_/(1 - lambda_is_calc))*(MPC_Planner_state_mpc_(3) + MPC_Planner_state_mpc_(4)/w_ - MPC_Planner_state_mpc_(5))
                       -(pow(lambda_is_calc, N_plan_mpc)/(1 + pow(lambda_is_calc,N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(1))(0,0));

    const_A_mpc_.row(   constraint_index)          = b_IS_plan_mpc_.transpose();
    const_ub_mpc_.block(constraint_index, 0, 1, 1) = Const_b_eq_.block(0, 0, 1, 1);
    const_lb_mpc_.block(constraint_index, 0, 1, 1) = Const_b_eq_.block(0, 0, 1, 1);
    constraint_index += 1;

    QP_MPC_Planner_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);

    if(QP_MPC_Planner_.SolveQPoases(100, MPC_Planner_u_mpc_))
    {
        if((((walking_tick_mpc_ - int(mpc_synchro_hz) + 1)/int(mpc_synchro_hz))%60) == 0) //30hz
        { cout << "IS FIPM Planner MPC Y direction Solved" << endl << endl; }

        Planner_State_Prev_mpc_.row(3).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(4).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(5).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_;

        MPC_Planner_state_mpc_.segment(3,3) = A_mpc_*MPC_Planner_state_mpc_.segment(3,3) + B_mpc_*MPC_Planner_u_mpc_(0);
    }
    else
    { 
        cout << "IS FIPM Planner MPC Y direction Not Solved" << endl;
        cout << int(walking_tick_mpc_ - 20)/mpc_synchro_hz << endl;

        Planner_State_Prev_mpc_.row(3).transpose() = Pcps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcpu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(4).transpose() = Pcvs_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pcvu_plan_mpc_*MPC_Planner_u_mpc_;
        Planner_State_Prev_mpc_.row(5).transpose() = Pvps_plan_mpc_*MPC_Planner_state_mpc_.segment(3,3) + Pvpu_plan_mpc_*MPC_Planner_u_mpc_;

        MPC_Planner_state_mpc_.segment(3,3) = A_mpc_*MPC_Planner_state_mpc_.segment(3,3) + B_mpc_*MPC_Planner_u_mpc_(0);
    }

    MPC_Planner_u_mpc_sep_(1) = MPC_Planner_u_mpc_(0);

    e_mpc_planner_data << N_plan_mpc                << "," << 0                         << "," << 0                         << ","
                       << Pv_x_ref(0)               << "," << Pv_y_ref(0)               << "," << Pv_z_ref(0)               << ","
                       << MPC_Planner_state_mpc_(0) << "," << MPC_Planner_state_mpc_(3) << "," << MPC_Planner_state_mpc_(6) << ","
                       << MPC_Planner_state_mpc_(1) << "," << MPC_Planner_state_mpc_(4) << "," << MPC_Planner_state_mpc_(7) << ","
                       << MPC_Planner_state_mpc_(2) << "," << MPC_Planner_state_mpc_(5) << "," << MPC_Planner_state_mpc_(8) << ","
                       << zmp_max_x_mpc_(0)         << "," << zmp_max_y_mpc_(0)         << "," << 0                         << ","
                       << endl;

    Eigen::VectorXd data_save_calc; data_save_calc.setZero(3*N_plan_mpc);
    data_save_calc << Pv_x_ref, Pv_y_ref, Pv_z_ref;
    e_tmp_graph2 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_time_calc_x_, zmp_time_calc_y_, 0*zmp_time_calc_y_;
    e_tmp_graph3 << data_save_calc.transpose() << endl;
    data_save_calc << Planner_State_Prev_mpc_.row(0).transpose(), Planner_State_Prev_mpc_.row(3).transpose(), Planner_State_Prev_mpc_.row(6).transpose();
    e_tmp_graph4 << data_save_calc.transpose() << endl; 
    data_save_calc << Planner_State_Prev_mpc_.row(2).transpose(), Planner_State_Prev_mpc_.row(5).transpose(), Planner_State_Prev_mpc_.row(8).transpose();
    e_tmp_graph5 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_max_x_mpc_, zmp_max_y_mpc_, 0*zmp_max_y_mpc_;
    e_tmp_graph6 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_max_x_time_plan_mpc, zmp_max_y_time_plan_mpc, 0*zmp_max_y_time_plan_mpc;
    e_tmp_graph7 << data_save_calc.transpose() << endl;
}

void AvatarController::IS_FIPM_3D_DCM_Stabililzer_MPC(double mpc_freq, double preview_window)
{
    double Q_dcm_x, Q_dcm_y, Q_dcm_z, R_dcm_x, R_dcm_y, R_dcm_z, R_dalp, R_df_x, R_df_y;

    //Q_dcm_x = 1e-0; R_dcm_x = 1e-2; R_dalp = 3e+0; R_df_x = 1e+2;
    //Q_dcm_y = 1e-0; R_dcm_y = 1e-2;                R_df_y = 2e+1;
    //Q_dcm_z = 1e-0; R_dcm_z = 1e-2; //for 0.9 step time

    Q_dcm_x = 1e-0; R_dcm_x = 1e-2; R_dalp = 3e+0; R_df_x = 1e+2;
    Q_dcm_y = 1e-0; R_dcm_y = 1e-3;                R_df_y = 3e+2; //need tuning 1e+2 - 1e+3
    Q_dcm_z = 9e-1; R_dcm_z = 1e-1; //for 0.9 step time

    static int MPC_first_loop = 0;
    int mpc_tick = walking_tick_mpc_ - com_start_tick_mpc_;
    double MPC_synchro_hz_ = 2000/mpc_freq;
    int N_stab_mpc = preview_window*mpc_freq;
    int N_step = 1.0*mpc_freq;
    int N_state = 3;
    double dt_stab_mpc = 1/mpc_freq;   

    double lambda_is_calc = exp(-w_*dt_stab_mpc);

    int input_num = 3*N_stab_mpc       + 1*step_time_adj_candidate_num_  + 2          + 2*step_time_adj_candidate_num_;
    //              VRP                  alpha                             delf         aux                           
    int const_num = 4*N_stab_mpc  + 3  + 1*step_time_adj_candidate_num_  + 1          + 2*step_time_adj_candidate_num_  + 2;
    //              VRP min max     IS   alpha min max                     alpha sum    alpha delf aux                    delf min max

    if(MPC_first_loop == 0)
    {      
        Cdp_mpc_.resize(1,N_state);
        Cdp_mpc_ << 1, 1/w_, 0;

        Pdps_stab_mpc_.resize(N_stab_mpc, N_state);
        Pcps_stab_mpc_.resize(N_stab_mpc, N_state);
        Pcvs_stab_mpc_.resize(N_stab_mpc, N_state);
        Pvps_stab_mpc_.resize(N_stab_mpc, N_state);

        Eigen::MatrixXd Ps_calc;
        Ps_calc.resize(N_state,N_state);
        Ps_calc = A_mpc_;
        
        Pdpu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        Pcpu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        Pcvu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        Pvpu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        
        Eigen::MatrixXd Pu_calc;
        Pu_calc.setZero(N_state,N_stab_mpc);
        
        for(int i = 0; i < N_stab_mpc; i++)
        {
            Pdps_stab_mpc_.row(i) = Cdp_mpc_*Ps_calc;
            Pcps_stab_mpc_.row(i) = Ccp_mpc_*Ps_calc;
            Pcvs_stab_mpc_.row(i) = Ccv_mpc_*Ps_calc;
            Pvps_stab_mpc_.row(i) = Cvp_mpc_*Ps_calc;
            Ps_calc = Ps_calc*A_mpc_;

            Pu_calc.col(i) = B_mpc_;
            Pdpu_stab_mpc_.row(i) = Cdp_mpc_*Pu_calc;
            Pcpu_stab_mpc_.row(i) = Ccp_mpc_*Pu_calc;
            Pcvu_stab_mpc_.row(i) = Ccv_mpc_*Pu_calc;
            Pvpu_stab_mpc_.row(i) = Cvp_mpc_*Pu_calc;
            Pu_calc = A_mpc_*Pu_calc;
        }

        QP_MPC_Stabilizer_.InitializeProblemSize(input_num, const_num);

        Qmat_stab_mpc_Q_.resize(N_stab_mpc, N_stab_mpc); //DCM
        Qmat_stab_mpc_Q_.setIdentity();
        
        Qmat_stab_mpc_R_.resize(N_stab_mpc, N_stab_mpc); //VRP
        Qmat_stab_mpc_R_.setIdentity();
        
        Qmat_stab_mpc_alp_.resize(step_time_adj_candidate_num_, step_time_adj_candidate_num_);
        Qmat_stab_mpc_alp_.setIdentity();

        Qcalc_stab_mpc_.setZero(input_num, input_num);

        gcalc_stab_mpc_.setZero(input_num, 1);
        gxpcalc_stab_mpc_.setZero(input_num, N_stab_mpc);
        gypcalc_stab_mpc_.setZero(input_num, N_stab_mpc);

        MPC_Stabilizer_u_mpc_.setZero(input_num); // VRP, delf, eps

        MPC_Stabilizer_SQP_du_mpc_.setZero(input_num);

        MPC_Stabilizer_alpha_mpc_.setZero(step_time_adj_candidate_num_);
        MPC_Stabilizer_alpha_mpc_(0) = 1;

        MPC_Stabilizer_aux_mpc_.setZero(2*step_time_adj_candidate_num_);
        MPC_Stabilizer_aux_mpc_x_.setZero(step_time_adj_candidate_num_);
        MPC_Stabilizer_aux_mpc_y_.setZero(step_time_adj_candidate_num_);

        MPC_Stabilizer_delf_mpc_.setZero(2);
        MPC_Stabilizer_delf_mpc_x_.setZero(1);
        MPC_Stabilizer_delf_mpc_y_.setZero(1);

        ssx_stab_mpc_ = ssx_plan_mpc_;
        ssy_stab_mpc_ = ssy_plan_mpc_;
        ssz_stab_mpc_ = ssz_plan_mpc_;

        int input_index_calc = 0;

        SUp_stab_mpc_.setZero(3*N_stab_mpc, input_num);
        SUp_stab_mpc_.block  (0, input_index_calc, 3*N_stab_mpc, 3*N_stab_mpc) = MatrixXd::Identity(3*N_stab_mpc, 3*N_stab_mpc);

        SUpx_stab_mpc_.setZero(N_stab_mpc, 3*N_stab_mpc);
        SUpx_stab_mpc_.block  (0, 0*N_stab_mpc, N_stab_mpc, N_stab_mpc) = MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        SUpxp_stab_mpc_ = SUpx_stab_mpc_*SUp_stab_mpc_;
        SUpy_stab_mpc_.setZero(N_stab_mpc, 3*N_stab_mpc);
        SUpy_stab_mpc_.block  (0, 1*N_stab_mpc, N_stab_mpc, N_stab_mpc) = MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        SUpyp_stab_mpc_ = SUpy_stab_mpc_*SUp_stab_mpc_;
        SUpz_stab_mpc_.setZero(N_stab_mpc, 3*N_stab_mpc);
        SUpz_stab_mpc_.block  (0, 2*N_stab_mpc, N_stab_mpc, N_stab_mpc) = MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        SUpzp_stab_mpc_ = SUpz_stab_mpc_*SUp_stab_mpc_;

        input_index_calc += 3*N_stab_mpc;

        cout << "Selection Matrix VRP Complete" << endl;
        cout << "input_num: " << input_num << endl;
        cout << "input_index_calc: " << input_index_calc << endl << endl;

        SUalp_stab_mpc_.setZero(1*step_time_adj_candidate_num_, input_num);
        SUalp_stab_mpc_.block  (0, input_index_calc, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_) = MatrixXd::Identity(1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);

        input_index_calc += 1*step_time_adj_candidate_num_;

        cout << "Selection Matrix alpha Complete" << endl;
        cout << "input_num: " << input_num << endl;
        cout << "input_index_calc: " << input_index_calc << endl << endl;

        SUf_stab_mpc_.setZero(2, input_num);
        SUf_stab_mpc_.block  (0, input_index_calc, 2, 2) = MatrixXd::Identity(2, 2);

        SUfx_stab_mpc_.setZero(1, 2);
        SUfx_stab_mpc_.block  (0, 0*1, 1, 1) = MatrixXd::Identity(1, 1);
        SUfy_stab_mpc_.setZero(1, 2);
        SUfy_stab_mpc_.block  (0, 1*1, 1, 1) = MatrixXd::Identity(1, 1);

        SUfxf_stab_mpc_ = SUfx_stab_mpc_*SUf_stab_mpc_;
        SUfyf_stab_mpc_ = SUfy_stab_mpc_*SUf_stab_mpc_;

        input_index_calc += 2*1;

        cout << "Selection Matrix delf Complete" << endl;
        cout << "input_num: "                    << input_num        << endl;
        cout << "input_index_calc: "             << input_index_calc << endl << endl;

        SUaux_stab_mpc_.setZero(2*step_time_adj_candidate_num_, input_num);
        SUaux_stab_mpc_.block  (0, input_index_calc, 2*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_) = MatrixXd::Identity(2*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_);

        SUaux_x_stab_mpc_.setZero(1*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_);
        SUaux_x_stab_mpc_.block  (0, 0*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_) = MatrixXd::Identity(1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);
        SUaux_y_stab_mpc_.setZero(1*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_);
        SUaux_y_stab_mpc_.block  (0, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_) = MatrixXd::Identity(1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);

        input_index_calc += 2*step_time_adj_candidate_num_;

        cout << "Selection Matrix aux Complete" << endl;
        cout << "input_num: " << input_num << endl;
        cout << "input_index_calc: " << input_index_calc << endl << endl;

        Qcalc_stab_mpc_ = SUp_stab_mpc_.transpose()*(SUpx_stab_mpc_.transpose()*(Pdpu_stab_mpc_.transpose()*Q_dcm_x*MatrixXd::Identity(N_stab_mpc, N_stab_mpc)*Pdpu_stab_mpc_ + R_dcm_x*Qmat_stab_mpc_R_)*SUpx_stab_mpc_
                                                    +SUpy_stab_mpc_.transpose()*(Pdpu_stab_mpc_.transpose()*Q_dcm_y*MatrixXd::Identity(N_stab_mpc, N_stab_mpc)*Pdpu_stab_mpc_ + R_dcm_y*Qmat_stab_mpc_R_)*SUpy_stab_mpc_
                                                    +SUpz_stab_mpc_.transpose()*(Pdpu_stab_mpc_.transpose()*Q_dcm_z*MatrixXd::Identity(N_stab_mpc, N_stab_mpc)*Pdpu_stab_mpc_ + R_dcm_z*Qmat_stab_mpc_R_)*SUpz_stab_mpc_)*SUp_stab_mpc_

                        + SUalp_stab_mpc_.transpose()*R_dalp*Qmat_stab_mpc_alp_*SUalp_stab_mpc_

                        + SUf_stab_mpc_.transpose()*(SUfx_stab_mpc_.transpose()*R_df_x*SUfx_stab_mpc_
                                                    +SUfy_stab_mpc_.transpose()*R_df_y*SUfy_stab_mpc_)*SUf_stab_mpc_;
                         
        gxpcalc_stab_mpc_ = SUp_stab_mpc_.transpose()*SUpx_stab_mpc_.transpose()*Pdpu_stab_mpc_.transpose()*Q_dcm_x*MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        gypcalc_stab_mpc_ = SUp_stab_mpc_.transpose()*SUpy_stab_mpc_.transpose()*Pdpu_stab_mpc_.transpose()*Q_dcm_y*MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        gzpcalc_stab_mpc_ = SUp_stab_mpc_.transpose()*SUpz_stab_mpc_.transpose()*Pdpu_stab_mpc_.transpose()*Q_dcm_z*MatrixXd::Identity(N_stab_mpc, N_stab_mpc);

        gxdfcalc_stab_mpc_ = SUf_stab_mpc_.transpose()*SUfx_stab_mpc_.transpose()*R_df_x*MatrixXd::Identity(1, 1);
        gydfcalc_stab_mpc_ = SUf_stab_mpc_.transpose()*SUfy_stab_mpc_.transpose()*R_df_y*MatrixXd::Identity(1, 1);

        gdalpcalc_stab_mpc_ = SUalp_stab_mpc_.transpose()*R_dalp*Qmat_stab_mpc_alp_;

        Sf1_stab_mpc_.setZero(N_stab_mpc, step_time_adj_candidate_num_);

        MPC_Stabilizer_u_mpc_sep_.setZero(3);

        IS_FIPM_SQP_x_phi_N_stab_mpc_.setZero(input_num*N_stab_mpc, input_num);
        IS_FIPM_SQP_x_phi2_N_stab_mpc_.setZero(input_num*N_stab_mpc, N_stab_mpc);

        IS_FIPM_SQP_x_pi_N_stab_mpc_.setZero (input_num*N_stab_mpc, 3*N_state);
        IS_FIPM_SQP_x_pi2_N_stab_mpc_.setZero(input_num*N_stab_mpc, 1);
        IS_FIPM_SQP_x_pi3_N_stab_mpc_.setZero(input_num*N_stab_mpc, 1);
        IS_FIPM_SQP_x_pi4_N_stab_mpc_.setZero(3*N_state*N_stab_mpc, N_stab_mpc);

        IS_FIPM_SQP_x_ri_N_stab_mpc_.setZero (3*N_state*N_stab_mpc, 3*N_state);
        IS_FIPM_SQP_x_ri2_N_stab_mpc_.setZero(        1*N_stab_mpc, 3*N_state);
        IS_FIPM_SQP_x_ri3_N_stab_mpc_.setZero(        1*N_stab_mpc, 3*N_state);

        IS_FIPM_SQP_y_phi_N_stab_mpc_.setZero(input_num*N_stab_mpc, input_num);
        IS_FIPM_SQP_y_phi2_N_stab_mpc_.setZero(input_num*N_stab_mpc, N_stab_mpc);

        IS_FIPM_SQP_y_pi_N_stab_mpc_.setZero (input_num*N_stab_mpc, 3*N_state);
        IS_FIPM_SQP_y_pi2_N_stab_mpc_.setZero(input_num*N_stab_mpc, 1);
        IS_FIPM_SQP_y_pi3_N_stab_mpc_.setZero(input_num*N_stab_mpc, 1);
        IS_FIPM_SQP_y_pi4_N_stab_mpc_.setZero(3*N_state*N_stab_mpc, N_stab_mpc);

        IS_FIPM_SQP_y_ri_N_stab_mpc_.setZero (3*N_state*N_stab_mpc, 3*N_state);
        IS_FIPM_SQP_y_ri2_N_stab_mpc_.setZero(        1*N_stab_mpc, 3*N_state);
        IS_FIPM_SQP_y_ri3_N_stab_mpc_.setZero(        1*N_stab_mpc, 3*N_state);
        
        int calc_index  = 0;
        int calc_index2 = 0;
        Eigen::MatrixXd Si_mpc; Si_mpc.setZero(1, N_stab_mpc);

        for(int i = 0; i < N_stab_mpc; i++)
        {
            Si_mpc.setZero(1, N_stab_mpc);
            Si_mpc(0, i) = 1;

            IS_FIPM_SQP_x_phi_N_stab_mpc_.block(calc_index, 0, input_num, input_num)  = (Pvpu_stab_mpc_.row(i)*SUpx_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_)
                                                                                      - (Pvpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcpu_stab_mpc_.row(i)*SUpx_stab_mpc_*SUp_stab_mpc_);
                                                                                     
            IS_FIPM_SQP_x_phi2_N_stab_mpc_.block(calc_index, 0, input_num, N_stab_mpc)= ((Pvpu_stab_mpc_ - Pcpu_stab_mpc_).row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*Si_mpc;

            IS_FIPM_SQP_y_phi_N_stab_mpc_.block(calc_index, 0, input_num, input_num)  = (Pvpu_stab_mpc_.row(i)*SUpy_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_)
                                                                                      - (Pvpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcpu_stab_mpc_.row(i)*SUpy_stab_mpc_*SUp_stab_mpc_);

            IS_FIPM_SQP_y_phi2_N_stab_mpc_.block(calc_index, 0, input_num, N_stab_mpc)= ((Pvpu_stab_mpc_ - Pcpu_stab_mpc_).row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*Si_mpc;

            IS_FIPM_SQP_x_pi_N_stab_mpc_.block(calc_index,  0, input_num, 3*N_state)  = (Pcpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*(Pvps_stab_mpc_.row(i)*ssx_stab_mpc_)
                                                                                      + (Pvpu_stab_mpc_.row(i)*SUpx_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssz_stab_mpc_)
                                                                                      - (Pcpu_stab_mpc_.row(i)*SUpx_stab_mpc_*SUp_stab_mpc_).transpose()*(Pvps_stab_mpc_.row(i)*ssz_stab_mpc_)
                                                                                      - (Pvpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssx_stab_mpc_);

            IS_FIPM_SQP_x_pi2_N_stab_mpc_.block(calc_index, 0, input_num, 1)          = GRAVITY*b_*b_*(Si_mpc*Pcpu_stab_mpc_*SUpx_stab_mpc_*SUp_stab_mpc_).transpose();

            IS_FIPM_SQP_x_pi3_N_stab_mpc_.block(calc_index, 0, input_num, 1)          = ((Pcpu_stab_mpc_ - Pvpu_stab_mpc_).row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose();
            
            IS_FIPM_SQP_y_pi_N_stab_mpc_.block(calc_index,  0, input_num, 3*N_state)  = (Pcpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*(Pvps_stab_mpc_.row(i)*ssy_stab_mpc_)
                                                                                      + (Pvpu_stab_mpc_.row(i)*SUpy_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssz_stab_mpc_)
                                                                                      - (Pcpu_stab_mpc_.row(i)*SUpy_stab_mpc_*SUp_stab_mpc_).transpose()*(Pvps_stab_mpc_.row(i)*ssz_stab_mpc_)
                                                                                      - (Pvpu_stab_mpc_.row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssy_stab_mpc_);

            IS_FIPM_SQP_y_pi2_N_stab_mpc_.block(calc_index, 0, input_num, 1)          = GRAVITY*b_*b_*(Pcpu_stab_mpc_.row(i)*SUpy_stab_mpc_*SUp_stab_mpc_).transpose();
            
            IS_FIPM_SQP_y_pi3_N_stab_mpc_.block(calc_index, 0, input_num, 1)          = ((Pcpu_stab_mpc_ - Pvpu_stab_mpc_).row(i)*SUpz_stab_mpc_*SUp_stab_mpc_).transpose();

            calc_index += input_num;

            IS_FIPM_SQP_x_ri_N_stab_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)  = (Pvps_stab_mpc_.row(i)*ssx_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssz_stab_mpc_)
                                                                                      - (Pvps_stab_mpc_.row(i)*ssz_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssx_stab_mpc_);

            IS_FIPM_SQP_x_ri2_N_stab_mpc_.block(i, 0, 1, 3*N_state)                   = GRAVITY*b_*b_*(Pcps_stab_mpc_.row(i)*ssx_stab_mpc_);
            
            IS_FIPM_SQP_x_ri3_N_stab_mpc_.block(i, 0, 1, 3*N_state)                   = ((Pcps_stab_mpc_ - Pvps_stab_mpc_).row(i)*ssz_stab_mpc_);
            
            IS_FIPM_SQP_x_pi4_N_stab_mpc_.block(calc_index2, 0, 3*N_state, N_stab_mpc)= ((Pvps_stab_mpc_ - Pcps_stab_mpc_).row(i)*ssz_stab_mpc_).transpose()*Si_mpc;

            IS_FIPM_SQP_y_ri_N_stab_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)  = (Pvps_stab_mpc_.row(i)*ssy_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssz_stab_mpc_)
                                                                                      - (Pvps_stab_mpc_.row(i)*ssz_stab_mpc_).transpose()*(Pcps_stab_mpc_.row(i)*ssy_stab_mpc_);

            IS_FIPM_SQP_y_ri2_N_stab_mpc_.block(i, 0, 1, 3*N_state)                   = GRAVITY*b_*b_*(Pcps_stab_mpc_.row(i)*ssy_stab_mpc_);

            IS_FIPM_SQP_y_ri3_N_stab_mpc_.block(i, 0, 1, 3*N_state)                   = ((Pcps_stab_mpc_ - Pvps_stab_mpc_).row(i)*ssz_stab_mpc_);

            IS_FIPM_SQP_y_pi4_N_stab_mpc_.block(calc_index2, 0, 3*N_state, N_stab_mpc)= ((Pvps_stab_mpc_ - Pcps_stab_mpc_).row(i)*ssz_stab_mpc_).transpose()*Si_mpc;

            calc_index2 += 3*N_state;
        }

        const_SQP_phi_mpc_.setZero(input_num, input_num);
        const_SQP_pi_mpc_.setZero(input_num, 1);
        const_SQP_ri_mpc_.setZero(1, 1);

        MPC_first_loop = 1;
        cout << "Initialiazation of IS 3D DCM MPC is completed" << endl;
    }

    Sf1_stab_mpc_.setZero();
    Sf2_stab_mpc_.setZero();

    double step_x_norm = foot_step_support_frame_mpc_(current_step_num_mpc_, 0);
    double step_y_norm = foot_step_support_frame_mpc_(current_step_num_mpc_, 1);

    if(mpc_tick < hz_/mpc_freq)
    {
        MPC_Stabilizer_delf_mpc_x_(0) = step_x_norm;
        MPC_Stabilizer_delf_mpc_y_(0) = step_y_norm;
        MPC_Stabilizer_delf_mpc_ << MPC_Stabilizer_delf_mpc_x_, MPC_Stabilizer_delf_mpc_y_;

        MPC_Stabilizer_alpha_mpc_.setZero();
        MPC_Stabilizer_alpha_mpc_(0) = 1;

        MPC_Stabilizer_aux_mpc_x_ = MPC_Stabilizer_alpha_mpc_*MPC_Stabilizer_delf_mpc_x_;
        MPC_Stabilizer_aux_mpc_y_ = MPC_Stabilizer_alpha_mpc_*MPC_Stabilizer_delf_mpc_y_;
        MPC_Stabilizer_aux_mpc_ << MPC_Stabilizer_aux_mpc_x_, MPC_Stabilizer_aux_mpc_y_;
    }

    step_enable_bool_mpc_          = (bool)(current_step_num_mpc_)*(bool)(mpc_tick                   < t_total_const_ - t_dsp2_const_ - step_enable_time_fwd_*hz_ - step_enable_fix_time_pre_*hz_);
    step_enable_bool_one_tick_mpc_ = (bool)(current_step_num_mpc_)*(bool)(mpc_tick + MPC_synchro_hz_ < t_total_const_ - t_dsp2_const_ - step_enable_time_fwd_*hz_ - step_enable_fix_time_pre_*hz_);
    
    Eigen::VectorXd data_save_calc; data_save_calc.setZero(2*N_stab_mpc);
    data_save_calc << zmp_max_x_mpc_.segment(0, N_stab_mpc), zmp_max_y_mpc_.segment(0, N_stab_mpc);
    e_tmp_graph15 << data_save_calc.transpose() << endl;

    for(int i = 0; i < N_stab_mpc; i++)
    {
        for(int j = 0; j < 1; j++)
        { 
            int dsp_length_calc           = int((t_dsp1_const_ + t_dsp2_const_)/MPC_synchro_hz_ + 0.5);
            int next_step_start_prev_tick = max(ceil(((j+1)*(t_total_const_ - t_dsp2_const_) - mpc_tick)/MPC_synchro_hz_), 0.0);
            bool  next_step_prev_bool     = (bool)(mpc_tick + MPC_synchro_hz_*(i + 2) > ((j + 1)*t_total_mpc_ - t_dsp2_const_));
            bool nnext_step_prev_bool     = (bool)(mpc_tick + MPC_synchro_hz_*(i + 2) > ((j + 2)*t_total_mpc_ - t_dsp2_const_));

            if(walking_tick_mpc_ > t_temp_)
            {
                Sf1_stab_mpc_(i,0) = step_enable_bool_mpc_*next_step_prev_bool*(zmp_max_y_mpc_(i) - zmp_max_y_mpc_(int(t_dsp1_const_/MPC_synchro_hz_)))/(MPC_Stabilizer_delf_mpc_y_(0));
                zmp_max_x_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_max_x_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_max_x_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_max_x_mpc_(i);
                zmp_min_x_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_min_x_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_min_x_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_min_x_mpc_(i);
                zmp_max_y_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_max_y_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_max_y_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_max_y_mpc_(i);
                zmp_min_y_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_min_y_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_min_y_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_min_y_mpc_(i);
            }
        }
    }

    for(int i = 1; i < step_time_adj_candidate_num_; i++)
    {
        Sf1_stab_mpc_.col(i) << Sf1_stab_mpc_.col(i-1).segment(1, N_stab_mpc - 1), min((Sf1_stab_mpc_(N_stab_mpc - 1, i-1) + (Sf1_stab_mpc_(N_stab_mpc - 1, i-1) - Sf1_stab_mpc_(N_stab_mpc - 2, i-1))), 1.0);
    }

e_tmp_graph16 << Sf1_stab_mpc_.col(0).transpose() << endl;

data_save_calc << zmp_max_x_mpc_.segment(0, N_stab_mpc), zmp_max_y_mpc_.segment(0, N_stab_mpc);
e_tmp_graph24 << data_save_calc.transpose() << endl;

    Eigen::VectorXd dcm_refx, dcm_refy, dcm_refz;
    dcm_refx.setZero(N_stab_mpc), dcm_refy.setZero(N_stab_mpc), dcm_refz.setZero(N_stab_mpc);

    dcm_refx = Planner_State_Prev_mpc_.block(0, 0, 1, N_stab_mpc).transpose() + b_*Planner_State_Prev_mpc_.block(1, 0, 1, N_stab_mpc).transpose();
    dcm_refy = Planner_State_Prev_mpc_.block(3, 0, 1, N_stab_mpc).transpose() + b_*Planner_State_Prev_mpc_.block(4, 0, 1, N_stab_mpc).transpose();
    dcm_refz = Planner_State_Prev_mpc_.block(6, 0, 1, N_stab_mpc).transpose() + b_*Planner_State_Prev_mpc_.block(7, 0, 1, N_stab_mpc).transpose();

    MPC_Stabilizer_state_mpc_(0) = com_measured_mpc_(0);
    MPC_Stabilizer_state_mpc_(1) = com_dot_measured_mpc_(0);
    MPC_Stabilizer_state_mpc_(3) = com_measured_mpc_(1);
    MPC_Stabilizer_state_mpc_(4) = com_dot_measured_mpc_(1);
    MPC_Stabilizer_state_mpc_(6) = com_measured_mpc_(2);
    MPC_Stabilizer_state_mpc_(7) = com_dot_measured_mpc_(2);

    gcalc_stab_mpc_ = gxpcalc_stab_mpc_  *(Pdps_stab_mpc_*ssx_stab_mpc_*MPC_Stabilizer_state_mpc_ - dcm_refx)
                     +gypcalc_stab_mpc_  *(Pdps_stab_mpc_*ssy_stab_mpc_*MPC_Stabilizer_state_mpc_ - dcm_refy)
                     +gzpcalc_stab_mpc_  *(Pdps_stab_mpc_*ssz_stab_mpc_*MPC_Stabilizer_state_mpc_ - dcm_refz)
                    
                     +gdalpcalc_stab_mpc_*(                                                       - MPC_Stabilizer_alpha_mpc_)

                     +gxdfcalc_stab_mpc_ *(                                                       - MPC_Stabilizer_delf_mpc_x_)
                     +gydfcalc_stab_mpc_ *(                                                       - MPC_Stabilizer_delf_mpc_y_);

    SQP_deldel_Qcalc_stab_mpc_ = Qcalc_stab_mpc_;

    int sqp_iter = 1;

    const_A_mpc_.setZero(const_num, input_num);
    const_lb_mpc_.setZero(const_num, 1);
    const_ub_mpc_.setZero(const_num, 1);

    Eigen::MatrixXd prev_state;    prev_state.setZero(9,N_stab_mpc);
    Eigen::MatrixXd prev_zmp;      prev_zmp.setZero(2, N_stab_mpc);

    std::vector<Eigen::MatrixXd> IS_FIPM_SQP_x_phi_max_vec; std::vector<Eigen::MatrixXd> IS_FIPM_SQP_x_phi_min_vec;
    std::vector<Eigen::MatrixXd> IS_FIPM_SQP_y_phi_max_vec; std::vector<Eigen::MatrixXd> IS_FIPM_SQP_y_phi_min_vec;
    std::vector<Eigen::MatrixXd> IS_FIPM_SQP_x_pi_max_vec;  std::vector<Eigen::MatrixXd> IS_FIPM_SQP_x_pi_min_vec;
    std::vector<Eigen::MatrixXd> IS_FIPM_SQP_y_pi_max_vec;  std::vector<Eigen::MatrixXd> IS_FIPM_SQP_y_pi_min_vec;
    std::vector<Eigen::MatrixXd> IS_FIPM_SQP_x_ri_max_vec;  std::vector<Eigen::MatrixXd> IS_FIPM_SQP_x_ri_min_vec;
    std::vector<Eigen::MatrixXd> IS_FIPM_SQP_y_ri_max_vec;  std::vector<Eigen::MatrixXd> IS_FIPM_SQP_y_ri_min_vec;

    for(int s = 0; s < sqp_iter; s++)
    {
        std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();

        SQP_del_g_calc_stab_mpc_   = Qcalc_stab_mpc_*MPC_Stabilizer_u_mpc_ + gcalc_stab_mpc_;

        QP_MPC_Stabilizer_.EnableEqualityCondition(equality_condition_eps_);
        QP_MPC_Stabilizer_.UpdateMinProblem(SQP_deldel_Qcalc_stab_mpc_, SQP_del_g_calc_stab_mpc_);
        QP_MPC_Stabilizer_.DeleteSubjectToAx();
        QP_MPC_Stabilizer_.DeleteSubjectToX();

        int constraint_index = 0;

        int calc_index = 0;
        int calc_index2 = 0;
        ////VRP Constraint
        std::chrono::steady_clock::time_point t12 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count()*1e-6 << ",";
        }

        for(int i = 0; i < N_stab_mpc; i++)
        //for(int i = 0; i < 1; i++)
        {
            //X max
            if(s == 0)
            {
                const_SQP_phi_mpc_ = IS_FIPM_SQP_x_phi_N_stab_mpc_.block(calc_index, 0, input_num, input_num)

                                   + IS_FIPM_SQP_x_phi2_N_stab_mpc_.block(calc_index, 0, input_num, N_stab_mpc)*Sf1_stab_mpc_*SUaux_x_stab_mpc_*SUaux_stab_mpc_;

                const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());

                IS_FIPM_SQP_x_phi_max_vec.push_back(const_SQP_phi_mpc_calc_);

                const_SQP_pi_mpc_  = IS_FIPM_SQP_x_pi_N_stab_mpc_.block (calc_index, 0, input_num, 3*N_state)*MPC_Stabilizer_state_mpc_

                                   + IS_FIPM_SQP_x_pi2_N_stab_mpc_.block(calc_index, 0, input_num, 1)

                                   - zmp_max_x_mpc_(i)*IS_FIPM_SQP_x_pi3_N_stab_mpc_.block(calc_index, 0, input_num, 1)

                                   + (MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_x_pi4_N_stab_mpc_.block(calc_index2, 0, 3*N_state, N_stab_mpc)*Sf1_stab_mpc_*SUaux_x_stab_mpc_*SUaux_stab_mpc_).transpose()
                           
                                   - (((GRAVITY*b_*b_)*MatrixXd::Identity(1,1)).transpose()*Sf1_stab_mpc_.row(i)*SUaux_x_stab_mpc_*SUaux_stab_mpc_).transpose();
            
                IS_FIPM_SQP_x_pi_max_vec.push_back(const_SQP_pi_mpc_);

                const_SQP_ri_mpc_  = MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_x_ri_N_stab_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Stabilizer_state_mpc_

                                   + IS_FIPM_SQP_x_ri2_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_

                                   - zmp_max_x_mpc_(i)*IS_FIPM_SQP_x_ri3_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_
                          
                                   - GRAVITY*b_*b_*zmp_max_x_mpc_(i)*MatrixXd::Identity(1,1);
            
                IS_FIPM_SQP_x_ri_max_vec.push_back(const_SQP_ri_mpc_);
            }

            const_SQP_hi_mpc_  = MPC_Stabilizer_u_mpc_.transpose()*IS_FIPM_SQP_x_phi_max_vec[i]*MPC_Stabilizer_u_mpc_ 
                               + IS_FIPM_SQP_x_pi_max_vec[i].transpose()*MPC_Stabilizer_u_mpc_ 
                               + IS_FIPM_SQP_x_ri_max_vec[i];

            const_A_mpc_.row(constraint_index) = (2*IS_FIPM_SQP_x_phi_max_vec[i]*MPC_Stabilizer_u_mpc_ + IS_FIPM_SQP_x_pi_max_vec[i]).transpose();
            const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            const_lb_mpc_.block(constraint_index, 0, 1, 1) = - 1e+3*MatrixXd::Identity(1,1);
            constraint_index += 1;

            //X min            
            if(s == 0)
            {
                const_SQP_phi_mpc_ = IS_FIPM_SQP_x_phi_N_stab_mpc_.block(calc_index, 0, input_num, input_num)

                                   + IS_FIPM_SQP_x_phi2_N_stab_mpc_.block(calc_index, 0, input_num, N_stab_mpc)*Sf1_stab_mpc_*SUaux_x_stab_mpc_*SUaux_stab_mpc_;

                const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());

                IS_FIPM_SQP_x_phi_min_vec.push_back(const_SQP_phi_mpc_calc_);

                const_SQP_pi_mpc_  = IS_FIPM_SQP_x_pi_N_stab_mpc_.block (calc_index, 0, input_num, 3*N_state)*MPC_Stabilizer_state_mpc_

                               + IS_FIPM_SQP_x_pi2_N_stab_mpc_.block(calc_index, 0, input_num, 1)

                               - zmp_min_x_mpc_(i)*IS_FIPM_SQP_x_pi3_N_stab_mpc_.block(calc_index, 0, input_num, 1)

                               + (MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_x_pi4_N_stab_mpc_.block(calc_index2, 0, 3*N_state, N_stab_mpc)*Sf1_stab_mpc_*SUaux_x_stab_mpc_*SUaux_stab_mpc_).transpose()
                           
                               - (((GRAVITY*b_*b_)*MatrixXd::Identity(1,1)).transpose()*Sf1_stab_mpc_.row(i)*SUaux_x_stab_mpc_*SUaux_stab_mpc_).transpose();

                IS_FIPM_SQP_x_pi_min_vec.push_back(const_SQP_pi_mpc_);

                const_SQP_ri_mpc_ = MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_x_ri_N_stab_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Stabilizer_state_mpc_

                              + IS_FIPM_SQP_x_ri2_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_

                              - zmp_min_x_mpc_(i)*IS_FIPM_SQP_x_ri3_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_
                          
                              - GRAVITY*b_*b_*zmp_min_x_mpc_(i)*MatrixXd::Identity(1,1);
            
                IS_FIPM_SQP_x_ri_min_vec.push_back(const_SQP_ri_mpc_);
            }

            const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*IS_FIPM_SQP_x_phi_min_vec[i]*MPC_Stabilizer_u_mpc_ 
                              + IS_FIPM_SQP_x_pi_min_vec[i].transpose()*MPC_Stabilizer_u_mpc_ 
                              + IS_FIPM_SQP_x_ri_min_vec[i];

            const_A_mpc_.row(constraint_index) = (2*IS_FIPM_SQP_x_phi_min_vec[i]*MPC_Stabilizer_u_mpc_ + IS_FIPM_SQP_x_pi_min_vec[i]).transpose();
            const_ub_mpc_.block(constraint_index, 0, 1, 1) =   1e+3*MatrixXd::Identity(1,1);
            const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            constraint_index += 1;

            //Y max
            if(s == 0)
            {
                const_SQP_phi_mpc_ = IS_FIPM_SQP_y_phi_N_stab_mpc_.block(calc_index, 0, input_num, input_num)
                                   
                                   + IS_FIPM_SQP_y_phi2_N_stab_mpc_.block(calc_index, 0, input_num, N_stab_mpc)*Sf1_stab_mpc_*SUaux_y_stab_mpc_*SUaux_stab_mpc_;

                const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());

                IS_FIPM_SQP_y_phi_max_vec.push_back(const_SQP_phi_mpc_calc_);

                const_SQP_pi_mpc_  = IS_FIPM_SQP_y_pi_N_stab_mpc_.block(calc_index,  0, input_num, 3*N_state)*MPC_Stabilizer_state_mpc_

                                   + IS_FIPM_SQP_y_pi2_N_stab_mpc_.block(calc_index, 0, input_num, 1)

                                   - zmp_max_y_mpc_(i)*IS_FIPM_SQP_y_pi3_N_stab_mpc_.block(calc_index, 0, input_num, 1)

                                   + (MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_y_pi4_N_stab_mpc_.block(calc_index2, 0, 3*N_state, N_stab_mpc)*Sf1_stab_mpc_*SUaux_y_stab_mpc_*SUaux_stab_mpc_).transpose()
                                   
                                   - (((GRAVITY*b_*b_)*MatrixXd::Identity(1,1)).transpose()*Sf1_stab_mpc_.row(i)*SUaux_y_stab_mpc_*SUaux_stab_mpc_).transpose();

                IS_FIPM_SQP_y_pi_max_vec.push_back(const_SQP_pi_mpc_);

                const_SQP_ri_mpc_ = MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_y_ri_N_stab_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Stabilizer_state_mpc_

                              + IS_FIPM_SQP_y_ri2_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_

                              - zmp_max_y_mpc_(i)*IS_FIPM_SQP_y_ri3_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_
                          
                              - GRAVITY*b_*b_*zmp_max_y_mpc_(i)*MatrixXd::Identity(1,1);
                              
                IS_FIPM_SQP_y_ri_max_vec.push_back(const_SQP_ri_mpc_);
            }

            const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*IS_FIPM_SQP_y_phi_max_vec[i]*MPC_Stabilizer_u_mpc_ 
                              + IS_FIPM_SQP_y_pi_max_vec[i].transpose()*MPC_Stabilizer_u_mpc_ 
                              + IS_FIPM_SQP_y_ri_max_vec[i];

            const_A_mpc_.row(constraint_index) = (2*IS_FIPM_SQP_y_phi_max_vec[i]*MPC_Stabilizer_u_mpc_ + IS_FIPM_SQP_y_pi_max_vec[i]).transpose();
            const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            const_lb_mpc_.block(constraint_index, 0, 1, 1) = - 1e+3*MatrixXd::Identity(1,1);
            constraint_index += 1;
            
            //Y min
            if(s == 0)
            {
                const_SQP_phi_mpc_ = IS_FIPM_SQP_y_phi_N_stab_mpc_.block(calc_index, 0, input_num, input_num)
                           
                                   + IS_FIPM_SQP_y_phi2_N_stab_mpc_.block(calc_index, 0, input_num, N_stab_mpc)*Sf1_stab_mpc_*SUaux_y_stab_mpc_*SUaux_stab_mpc_;
                
                const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());

                IS_FIPM_SQP_y_phi_min_vec.push_back(const_SQP_phi_mpc_calc_);

                const_SQP_pi_mpc_  = IS_FIPM_SQP_y_pi_N_stab_mpc_.block (calc_index, 0, input_num, 3*N_state)*MPC_Stabilizer_state_mpc_

                                   + IS_FIPM_SQP_y_pi2_N_stab_mpc_.block(calc_index, 0, input_num, 1)

                                   - zmp_min_y_mpc_(i)*IS_FIPM_SQP_y_pi3_N_stab_mpc_.block(calc_index, 0, input_num, 1)
                           
                                   + (MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_y_pi4_N_stab_mpc_.block(calc_index2, 0, 3*N_state, N_stab_mpc)*Sf1_stab_mpc_*SUaux_y_stab_mpc_*SUaux_stab_mpc_).transpose()
                           
                                   - (((GRAVITY*b_*b_)*MatrixXd::Identity(1,1)).transpose()*Sf1_stab_mpc_.row(i)*SUaux_y_stab_mpc_*SUaux_stab_mpc_).transpose();
                               
                IS_FIPM_SQP_y_pi_min_vec.push_back(const_SQP_pi_mpc_);

                const_SQP_ri_mpc_ = MPC_Stabilizer_state_mpc_.transpose()*IS_FIPM_SQP_y_ri_N_stab_mpc_.block(calc_index2, 0, 3*N_state, 3*N_state)*MPC_Stabilizer_state_mpc_

                              + IS_FIPM_SQP_y_ri2_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_

                              - zmp_min_y_mpc_(i)*IS_FIPM_SQP_y_ri3_N_stab_mpc_.block(i, 0, 1, 3*N_state)*MPC_Stabilizer_state_mpc_
                          
                              - GRAVITY*b_*b_*zmp_min_y_mpc_(i)*MatrixXd::Identity(1,1);
                
                IS_FIPM_SQP_y_ri_min_vec.push_back(const_SQP_ri_mpc_);
            }

            const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*IS_FIPM_SQP_y_phi_min_vec[i]*MPC_Stabilizer_u_mpc_ 
                              + IS_FIPM_SQP_y_pi_min_vec[i].transpose()*MPC_Stabilizer_u_mpc_ 
                              + IS_FIPM_SQP_y_ri_min_vec[i];

            const_A_mpc_.row(constraint_index) = (2*IS_FIPM_SQP_y_phi_min_vec[i]*MPC_Stabilizer_u_mpc_ + IS_FIPM_SQP_y_pi_min_vec[i]).transpose();
            const_ub_mpc_.block(constraint_index, 0, 1, 1) =   1e+3*MatrixXd::Identity(1,1);
            const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            constraint_index += 1;
        
            calc_index  += input_num;
            calc_index2 += 3*N_state;
        }

        std::chrono::steady_clock::time_point t13 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t13 - t12).count()*1e-6 << ",";
        }

        //IS equality
        Eigen::MatrixXd b_IS_stab_mpc; b_IS_stab_mpc.resize(N_stab_mpc,1); b_IS_stab_mpc.col(0) = b_IS_plan_mpc_.col(0).segment(0,N_stab_mpc);
        //Pre planned Tail
        Eigen::MatrixXd Const_b_eq; Const_b_eq.setZero(3,1);
        Const_b_eq(0,0) = -(w_/(1 - lambda_is_calc))*(MPC_Stabilizer_state_mpc_(0) + MPC_Stabilizer_state_mpc_(1)/w_ - MPC_Stabilizer_state_mpc_(2))
                          +(pow(lambda_is_calc, N_stab_mpc)/(1 - pow(lambda_is_calc, N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(0))(0,0));
        Const_b_eq(1,0) = -(w_/(1 - lambda_is_calc))*(MPC_Stabilizer_state_mpc_(3) + MPC_Stabilizer_state_mpc_(4)/w_ - MPC_Stabilizer_state_mpc_(5))
                          +(pow(lambda_is_calc, N_stab_mpc)/(1 + pow(lambda_is_calc, N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(1))(0,0));
        Const_b_eq(2,0) = -(w_/(1 - lambda_is_calc))*(MPC_Stabilizer_state_mpc_(6) + MPC_Stabilizer_state_mpc_(7)/w_ - MPC_Stabilizer_state_mpc_(8));

        const_SQP_phi_mpc_.setZero(input_num, input_num);
        //const_SQP_pi_mpc_ = (b_IS_stab_mpc.transpose()*SUpx_stab_mpc_*SUp_stab_mpc_).transpose();
        const_SQP_pi_mpc_ = SUpxp_stab_mpc_.transpose()*b_IS_stab_mpc;
        const_SQP_ri_mpc_ = Const_b_eq.block(0, 0, 1, 1);

        //const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;
        const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

        //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_A_mpc_.row(constraint_index) = const_SQP_pi_mpc_.transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        constraint_index += 1;

        const_SQP_phi_mpc_.setZero(input_num, input_num);
        //const_SQP_pi_mpc_ = (b_IS_stab_mpc.transpose()*SUpy_stab_mpc_*SUp_stab_mpc_).transpose();
        const_SQP_pi_mpc_ = SUpyp_stab_mpc_.transpose()*b_IS_stab_mpc;
        const_SQP_ri_mpc_ = Const_b_eq.block(1, 0, 1, 1);
        //const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;
        const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

        //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_A_mpc_.row(constraint_index) = const_SQP_pi_mpc_.transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        constraint_index += 1;

        const_SQP_phi_mpc_.setZero(input_num, input_num);
        //const_SQP_pi_mpc_ = (b_IS_stab_mpc.transpose()*SUpz_stab_mpc_*SUp_stab_mpc_).transpose();
        const_SQP_pi_mpc_ = SUpzp_stab_mpc_.transpose()*b_IS_stab_mpc;
        const_SQP_ri_mpc_ = Const_b_eq.block(2, 0, 1, 1);
        //const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;
        const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

        //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_A_mpc_.row(constraint_index) = const_SQP_pi_mpc_.transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        constraint_index += 1;

        std::chrono::steady_clock::time_point t14 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t14 - t13).count()*1e-6 << ",";
        }
        
        //alpha min max
        const_A_mpc_.block (constraint_index, 0, step_time_adj_candidate_num_, input_num) = SUalp_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, step_time_adj_candidate_num_, 1)         = - SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Ones(step_time_adj_candidate_num_, 1);
        const_lb_mpc_.block(constraint_index, 0, step_time_adj_candidate_num_, 1)         = - SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Zero(step_time_adj_candidate_num_, 1);
        constraint_index += step_time_adj_candidate_num_;

        //alpha sum
        const_A_mpc_.block (constraint_index, 0, 1, input_num) = MatrixXd::Ones(1, step_time_adj_candidate_num_)*SUalp_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, 1, 1)         = - MatrixXd::Ones(1, step_time_adj_candidate_num_)*SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Ones(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1)         = - MatrixXd::Ones(1, step_time_adj_candidate_num_)*SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Ones(1,1);
        constraint_index += 1;

        std::chrono::steady_clock::time_point t15 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t15 - t14).count()*1e-6 << ",";
        }

        double delf_x_max = 0.25, delf_x_min = -0.20;
        double delf_y_max = 0.10, delf_y_min =  0.00;

        double delf_x_max_calc, delf_x_min_calc;
        double delf_y_max_calc, delf_y_min_calc;

        //delf min max
        const_A_mpc_.block (constraint_index, 0, 1, input_num) = SUfx_stab_mpc_*SUf_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, 1, 1)         = - SUfx_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_x_max*MatrixXd::Ones(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1)         = - SUfx_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_x_min*MatrixXd::Ones(1,1);
        constraint_index += 1;

        delf_y_max_calc = step_y_norm + (foot_step_(current_step_num_mpc_, 6)*delf_y_min + (1 - foot_step_(current_step_num_mpc_, 6))*delf_y_max);
        delf_y_min_calc = step_y_norm - (foot_step_(current_step_num_mpc_, 6)*delf_y_max + (1 - foot_step_(current_step_num_mpc_, 6))*delf_y_min);

        const_A_mpc_.block (constraint_index, 0, 1, input_num) = SUfy_stab_mpc_*SUf_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, 1, 1)         = - SUfy_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_y_max_calc*MatrixXd::Ones(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1)         = - SUfy_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_y_min_calc*MatrixXd::Ones(1,1);
        constraint_index += 1;

        //alpha delf aux
        for(int i = 0; i < step_time_adj_candidate_num_; i++)
        {
            const_SQP_phi_mpc_ = SUalp_stab_mpc_.row(i).transpose()*SUfx_stab_mpc_*SUf_stab_mpc_;
            const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());
            const_SQP_pi_mpc_  = (- SUaux_x_stab_mpc_.row(i)*SUaux_stab_mpc_).transpose();
            const_SQP_ri_mpc_.setZero(1,1);
            const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

            const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
            const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            constraint_index += 1;

            const_SQP_phi_mpc_ = SUalp_stab_mpc_.row(i).transpose()*SUfy_stab_mpc_*SUf_stab_mpc_;
            const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());
            const_SQP_pi_mpc_  = (- SUaux_y_stab_mpc_.row(i)*SUaux_stab_mpc_).transpose();
            const_SQP_ri_mpc_.setZero(1,1);
            const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

            const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
            const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            constraint_index += 1;
        }

        std::chrono::steady_clock::time_point t16 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t16 - t15).count()*1e-6 << ",";
        }

        QP_MPC_Stabilizer_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);

        if(QP_MPC_Stabilizer_.SolveQPoases(100, MPC_Stabilizer_SQP_du_mpc_))
        {
            //if((walking_tick_mpc_ - int(MPC_synchro_hz_) - 20)%int(2*hz_) == 0)
            if((((walking_tick_mpc_ - int(MPC_synchro_hz_) + 1)/int(MPC_synchro_hz_))%60) == 0) //30hz
            { 
                cout << "SQP Iter: " << s + 1 << endl;
                cout << "IS FIPM DCM Stabilizer Stepping Solved" << endl;
                if(s == sqp_iter - 1) { cout << endl; }
            }

            MPC_Stabilizer_u_mpc_ = MPC_Stabilizer_u_mpc_ + MPC_Stabilizer_SQP_du_mpc_;

            if(s == sqp_iter - 1)
            {
                prev_state.row(0).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(1).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(2).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);

                prev_state.row(3).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(4).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(5).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);

                prev_state.row(6).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc, N_stab_mpc);
                prev_state.row(7).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc, N_stab_mpc);
                prev_state.row(8).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc, N_stab_mpc);

                Eigen::VectorXd lambda_bb_mat, lambda_bb_mat_calc;
                lambda_bb_mat_calc = (prev_state.row(8) - GRAVITY*b_*b_*MatrixXd::Ones(1, N_stab_mpc)).array()/prev_state.row(6).array();
                lambda_bb_mat      = MatrixXd::Ones(N_stab_mpc, 1) - lambda_bb_mat_calc;

                prev_zmp.row(0)    = (prev_state.row(2).array() - (MatrixXd::Ones(1, N_stab_mpc) - lambda_bb_mat.transpose()).array()*prev_state.row(0).array()).array()/lambda_bb_mat.transpose().array();
                prev_zmp.row(1)    = (prev_state.row(5).array() - (MatrixXd::Ones(1, N_stab_mpc) - lambda_bb_mat.transpose()).array()*prev_state.row(3).array()).array()/lambda_bb_mat.transpose().array();

                MPC_Stabilizer_u_mpc_sep_(0) = MPC_Stabilizer_u_mpc_(0*N_stab_mpc);
                MPC_Stabilizer_u_mpc_sep_(1) = MPC_Stabilizer_u_mpc_(1*N_stab_mpc);
                MPC_Stabilizer_u_mpc_sep_(2) = MPC_Stabilizer_u_mpc_(2*N_stab_mpc);

                MPC_Stabilizer_alpha_mpc_  = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc, 1*step_time_adj_candidate_num_);

                MPC_Stabilizer_delf_mpc_   = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_,     2);
                MPC_Stabilizer_delf_mpc_x_ = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 0, 1);
                MPC_Stabilizer_delf_mpc_y_ = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 1, 1);

                MPC_Stabilizer_aux_mpc_    = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2,                                  2*step_time_adj_candidate_num_);
                MPC_Stabilizer_aux_mpc_x_  = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2 + 0*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);
                MPC_Stabilizer_aux_mpc_y_  = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2 + 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);

                MPC_Stabilizer_state_mpc_.segment(0,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + B_mpc_*MPC_Stabilizer_u_mpc_(0*N_stab_mpc);
                MPC_Stabilizer_state_mpc_.segment(3,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + B_mpc_*MPC_Stabilizer_u_mpc_(1*N_stab_mpc);
                MPC_Stabilizer_state_mpc_.segment(6,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + B_mpc_*MPC_Stabilizer_u_mpc_(2*N_stab_mpc);
            }
        }
        else
        {
            cout << "SQP Iter: " << s + 1 << endl;
            cout << "IS FIPM DCM Stabilizer Stepping Not Solved" << endl;
            cout << (walking_tick_mpc_ - int(MPC_synchro_hz_) + 1)/int(MPC_synchro_hz_) + 1 << endl;

            MPC_Stabilizer_u_mpc_ = MPC_Stabilizer_u_mpc_ + MPC_Stabilizer_SQP_du_mpc_;

            if(s == sqp_iter - 1)
            {
                prev_state.row(0).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(1).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(2).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);

                prev_state.row(3).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(4).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(5).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);

                prev_state.row(6).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc, N_stab_mpc);
                prev_state.row(7).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc, N_stab_mpc);
                prev_state.row(8).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc, N_stab_mpc);

                Eigen::VectorXd lambda_bb_mat, lambda_bb_mat_calc;
                lambda_bb_mat_calc = (prev_state.row(8) - GRAVITY*b_*b_*MatrixXd::Ones(1, N_stab_mpc)).array()/prev_state.row(6).array();
                lambda_bb_mat      = MatrixXd::Ones(N_stab_mpc, 1) - lambda_bb_mat_calc;

                prev_zmp.row(0)    = (prev_state.row(2).array() - (MatrixXd::Ones(1, N_stab_mpc) - lambda_bb_mat.transpose()).array()*prev_state.row(0).array()).array()/lambda_bb_mat.transpose().array();
                prev_zmp.row(1)    = (prev_state.row(5).array() - (MatrixXd::Ones(1, N_stab_mpc) - lambda_bb_mat.transpose()).array()*prev_state.row(3).array()).array()/lambda_bb_mat.transpose().array();

                MPC_Stabilizer_u_mpc_sep_(0) = MPC_Stabilizer_u_mpc_(0*N_stab_mpc);
                MPC_Stabilizer_u_mpc_sep_(1) = MPC_Stabilizer_u_mpc_(1*N_stab_mpc);
                MPC_Stabilizer_u_mpc_sep_(2) = MPC_Stabilizer_u_mpc_(2*N_stab_mpc);

                MPC_Stabilizer_alpha_mpc_  = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 0*step_time_adj_candidate_num_,     1*step_time_adj_candidate_num_);

                MPC_Stabilizer_delf_mpc_   = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_,     2);
                MPC_Stabilizer_delf_mpc_x_ = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 0, 1);
                MPC_Stabilizer_delf_mpc_y_ = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 1, 1);

                MPC_Stabilizer_aux_mpc_    = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2,                                  2*step_time_adj_candidate_num_);
                MPC_Stabilizer_aux_mpc_x_  = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2 + 0*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);
                MPC_Stabilizer_aux_mpc_y_  = MPC_Stabilizer_u_mpc_.segment(3*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2 + 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);

                MPC_Stabilizer_state_mpc_.segment(0,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + B_mpc_*MPC_Stabilizer_u_mpc_(0*N_stab_mpc);
                MPC_Stabilizer_state_mpc_.segment(3,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + B_mpc_*MPC_Stabilizer_u_mpc_(1*N_stab_mpc);
                MPC_Stabilizer_state_mpc_.segment(6,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + B_mpc_*MPC_Stabilizer_u_mpc_(2*N_stab_mpc);
            }
        }

        std::chrono::steady_clock::time_point t17 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t17 - t16).count()*1e-6 << endl;
        }
    }

    double calc_time_adj = 0.0;

    for (int i = 0; i < step_time_adj_candidate_num_; i++)
    {
        if(abs(MPC_Stabilizer_alpha_mpc_(i)) < 5e-3)
        {
            MPC_Stabilizer_alpha_mpc_(i) = 0.0;
        }
        calc_time_adj += i*MPC_Stabilizer_alpha_mpc_(i);
    }

    MPC_Stabilizer_time_adj_tick_x_mpc_ = max(round(calc_time_adj + 0.1), 0.0);
    MPC_Stabilizer_time_adj_tick_y_mpc_ = max(round(calc_time_adj + 0.1), 0.0);

    double time_adj_tick_mpc = 0.0;
    time_adj_tick_mpc = max(MPC_Stabilizer_time_adj_tick_x_mpc_, MPC_Stabilizer_time_adj_tick_y_mpc_);
    time_adj_tick_mpc = min(time_adj_tick_mpc, double(step_time_adj_candidate_num_ - 1));
    if(current_step_num_mpc_ != 0 && step_enable_bool_mpc_ == 0)
    {   
        t_total_mpc_ = t_total_const_ - time_adj_tick_mpc*hz_/thread3_hz_;
    }

    e_mpc_stabilizer_data << N_stab_mpc                          << "," << step_time_adj_candidate_num_        << "," << step_enable_bool_mpc_        << ","
                          << dcm_measured_mpc_(0)                << "," << dcm_measured_mpc_(1)                << "," << dcm_measured_mpc_(2)         << ","
                          << dcm_refx(0)                         << "," << dcm_refy(0)                         << "," << dcm_refz(0)                  << ","
                          << MPC_Stabilizer_state_mpc_(0)        << "," << MPC_Stabilizer_state_mpc_(3)        << "," << MPC_Stabilizer_state_mpc_(6) << ","
                          << MPC_Stabilizer_state_mpc_(1)        << "," << MPC_Stabilizer_state_mpc_(4)        << "," << MPC_Stabilizer_state_mpc_(7) << ","
                          << MPC_Stabilizer_state_mpc_(2)        << "," << MPC_Stabilizer_state_mpc_(5)        << "," << MPC_Stabilizer_state_mpc_(8) << ","
                          << zmp_max_x_mpc_(0)                   << "," << zmp_max_y_mpc_(0)                   << "," << 0                            << ","
                          << prev_zmp(0, 0)                      << "," << prev_zmp(1, 0)                      << "," << 0                            << ","
                          << MPC_Stabilizer_delf_mpc_x_(0)       << "," << MPC_Stabilizer_delf_mpc_y_(0)       << "," << 0                            << ","
                          << step_x_norm                         << "," << step_y_norm                         << "," << 0                            << ","
                          << MPC_Stabilizer_time_adj_tick_x_mpc_ << "," << calc_time_adj                       << "," << time_adj_tick_mpc            << ","
                          << endl;

    data_save_calc.setZero(2*N_stab_mpc);
    data_save_calc << dcm_refx, dcm_refy;
    e_tmp_graph8 << data_save_calc.transpose() << endl;
    data_save_calc << prev_state.row(2).transpose(), prev_state.row(5).transpose();
    e_tmp_graph9 << data_save_calc.transpose() << endl;
    data_save_calc << prev_state.row(0).transpose() + b_*prev_state.row(1).transpose(), prev_state.row(3).transpose() + b_*prev_state.row(4).transpose();
    e_tmp_graph10 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_max_x_mpc_.segment(0, N_stab_mpc), zmp_max_y_mpc_.segment(0, N_stab_mpc);
    e_tmp_graph11 << data_save_calc.transpose() << endl;
    data_save_calc << prev_zmp.row(0).transpose(), prev_zmp.row(1).transpose();
    e_tmp_graph12 << data_save_calc.transpose() << endl;
    data_save_calc.setZero(step_time_adj_candidate_num_);
    data_save_calc << MPC_Stabilizer_alpha_mpc_;
    e_tmp_graph13 << data_save_calc.transpose() << endl;
    data_save_calc.setZero(2*step_time_adj_candidate_num_);
    data_save_calc << MPC_Stabilizer_aux_mpc_x_, MPC_Stabilizer_aux_mpc_y_;
    e_tmp_graph14 << data_save_calc.transpose() << endl;

    step_enable_bool_mpc_ = (bool)(mpc_tick + MPC_synchro_hz_ < t_total_const_ - t_dsp2_const_ - step_enable_time_fwd_*hz_ - step_enable_fix_time_pre_*hz_);
}

void AvatarController::IS_FIPM_3D_DCM_Seq_Stabilizer_MPC(double mpc_freq, double preview_window)
{
    double Q_dcm_x, Q_dcm_y, Q_dcm_z, R_dcm_x, R_dcm_y, R_dcm_z, R_dalp, R_df_x, R_df_y;

    Q_dcm_x = 1e-0; R_dcm_x = 1e-2; R_dalp = 3e+0; R_df_x = 1e+2;
    Q_dcm_y = 1e-0; R_dcm_y = 1e-3;                R_df_y = 3e+2; //need tuning 1e+2 - 1e+3
    Q_dcm_z = 9e-1; R_dcm_z = 1e-1; //for 0.9 step time

    static int MPC_first_loop = 0;
    int mpc_tick = walking_tick_mpc_ - com_start_tick_mpc_;
    double MPC_synchro_hz_ = 2000/mpc_freq;
    int N_stab_mpc = preview_window*mpc_freq;
    int N_step = 1.0*mpc_freq;
    int N_state = 3;
    double dt_stab_mpc = 1/mpc_freq;   

    double lambda_is_calc = exp(-w_*dt_stab_mpc);

    int input_num_z  = N_stab_mpc;
    //                 VRP
    int input_num_xy = 2*N_stab_mpc  + 1*step_time_adj_candidate_num_  + 2          + 2*step_time_adj_candidate_num_;
    //                 VRP             alpha                             delf         aux                           
    int const_num_z  = N_stab_mpc + 1;
    //                 VRP          IS
    int const_num_xy = 4*N_stab_mpc  + 2  + 1*step_time_adj_candidate_num_  + 1          + 2*step_time_adj_candidate_num_  + 2;
    //                 VRP min max     IS   alpha min max                     alpha sum    alpha delf aux                    delf min max

    if(MPC_first_loop == 0)
    {      
        Cdp_mpc_.resize(1,N_state);
        Cdp_mpc_ << 1, 1/w_, 0;

        Pdps_stab_mpc_.resize(N_stab_mpc, N_state);
        Pcps_stab_mpc_.resize(N_stab_mpc, N_state);
        Pcvs_stab_mpc_.resize(N_stab_mpc, N_state);
        Pvps_stab_mpc_.resize(N_stab_mpc, N_state);

        Eigen::MatrixXd Ps_calc;
        Ps_calc.resize(N_state,N_state);
        Ps_calc = A_mpc_;
        
        Pdpu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        Pcpu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        Pcvu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        Pvpu_stab_mpc_.setZero(N_stab_mpc, N_stab_mpc);
        
        Eigen::MatrixXd Pu_calc;
        Pu_calc.setZero(N_state,N_stab_mpc);
        
        for(int i = 0; i < N_stab_mpc; i++)
        {
            Pdps_stab_mpc_.row(i) = Cdp_mpc_*Ps_calc;
            Pcps_stab_mpc_.row(i) = Ccp_mpc_*Ps_calc;
            Pcvs_stab_mpc_.row(i) = Ccv_mpc_*Ps_calc;
            Pvps_stab_mpc_.row(i) = Cvp_mpc_*Ps_calc;
            Ps_calc = Ps_calc*A_mpc_;

            Pu_calc.col(i) = B_mpc_;
            Pdpu_stab_mpc_.row(i) = Cdp_mpc_*Pu_calc;
            Pcpu_stab_mpc_.row(i) = Ccp_mpc_*Pu_calc;
            Pcvu_stab_mpc_.row(i) = Ccv_mpc_*Pu_calc;
            Pvpu_stab_mpc_.row(i) = Cvp_mpc_*Pu_calc;
            Pu_calc = A_mpc_*Pu_calc;
        }

        QP_MPC_Stabilizer_z_.InitializeProblemSize(input_num_z,  const_num_z);
        QP_MPC_Stabilizer_.InitializeProblemSize(  input_num_xy, const_num_xy);

        Qmat_stab_mpc_Q_.resize(N_stab_mpc, N_stab_mpc); //DCM
        Qmat_stab_mpc_Q_.setIdentity();
        
        Qmat_stab_mpc_R_.resize(N_stab_mpc, N_stab_mpc); //VRP
        Qmat_stab_mpc_R_.setIdentity();
        
        Qmat_stab_mpc_alp_.resize(step_time_adj_candidate_num_, step_time_adj_candidate_num_);
        Qmat_stab_mpc_alp_.setIdentity();

        Qcalc_stab_mpc_z_.setZero(input_num_z,  input_num_z);
        Qcalc_stab_mpc_.  setZero(input_num_xy, input_num_xy);

        gcalc_stab_mpc_z_.setZero(input_num_z,  1);
        gcalc_stab_mpc_.  setZero(input_num_xy, 1);
        gxpcalc_stab_mpc_.setZero(input_num_xy, N_stab_mpc);
        gypcalc_stab_mpc_.setZero(input_num_xy, N_stab_mpc);

        MPC_Stabilizer_u_mpc_z_.setZero(input_num_z);
        MPC_Stabilizer_u_mpc_.  setZero(input_num_xy); // VRP, delf, eps

        MPC_Stabilizer_SQP_du_mpc_.setZero(input_num_xy);

        MPC_Stabilizer_alpha_mpc_.setZero(step_time_adj_candidate_num_);
        MPC_Stabilizer_alpha_mpc_(0) = 1;

        MPC_Stabilizer_aux_mpc_.setZero(2*step_time_adj_candidate_num_);
        MPC_Stabilizer_aux_mpc_x_.setZero(step_time_adj_candidate_num_);
        MPC_Stabilizer_aux_mpc_y_.setZero(step_time_adj_candidate_num_);

        MPC_Stabilizer_delf_mpc_.setZero(2);
        MPC_Stabilizer_delf_mpc_x_.setZero(1);
        MPC_Stabilizer_delf_mpc_y_.setZero(1);

        ssx_stab_mpc_ = ssx_plan_mpc_;
        ssy_stab_mpc_ = ssy_plan_mpc_;
        ssz_stab_mpc_ = ssz_plan_mpc_;

        //only works in xy direction
        int input_index_calc = 0;

        SUp_stab_mpc_.setZero(2*N_stab_mpc, input_num_xy);
        SUp_stab_mpc_.block  (0, input_index_calc, 2*N_stab_mpc, 2*N_stab_mpc) = MatrixXd::Identity(2*N_stab_mpc, 2*N_stab_mpc);

        SUpx_stab_mpc_.setZero(N_stab_mpc, 2*N_stab_mpc);
        SUpx_stab_mpc_.block  (0, 0*N_stab_mpc, N_stab_mpc, N_stab_mpc) = MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        SUpxp_stab_mpc_ = SUpx_stab_mpc_*SUp_stab_mpc_;
        SUpy_stab_mpc_.setZero(N_stab_mpc, 2*N_stab_mpc);
        SUpy_stab_mpc_.block  (0, 1*N_stab_mpc, N_stab_mpc, N_stab_mpc) = MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        SUpyp_stab_mpc_ = SUpy_stab_mpc_*SUp_stab_mpc_;

        input_index_calc += 2*N_stab_mpc;

        SUalp_stab_mpc_.setZero(1*step_time_adj_candidate_num_, input_num_xy);
        SUalp_stab_mpc_.block  (0, input_index_calc, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_) = MatrixXd::Identity(1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);

        input_index_calc += 1*step_time_adj_candidate_num_;

        SUf_stab_mpc_.setZero(2, input_num_xy);
        SUf_stab_mpc_.block  (0, input_index_calc, 2, 2) = MatrixXd::Identity(2, 2);

        SUfx_stab_mpc_.setZero(1, 2);
        SUfx_stab_mpc_.block  (0, 0*1, 1, 1) = MatrixXd::Identity(1, 1);
        SUfy_stab_mpc_.setZero(1, 2);
        SUfy_stab_mpc_.block  (0, 1*1, 1, 1) = MatrixXd::Identity(1, 1);

        SUfxf_stab_mpc_ = SUfx_stab_mpc_*SUf_stab_mpc_;
        SUfyf_stab_mpc_ = SUfy_stab_mpc_*SUf_stab_mpc_;

        input_index_calc += 2*1;

        SUaux_stab_mpc_.setZero(2*step_time_adj_candidate_num_, input_num_xy);
        SUaux_stab_mpc_.block  (0, input_index_calc, 2*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_) = MatrixXd::Identity(2*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_);

        SUaux_x_stab_mpc_.setZero(1*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_);
        SUaux_x_stab_mpc_.block  (0, 0*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_) = MatrixXd::Identity(1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);
        SUaux_y_stab_mpc_.setZero(1*step_time_adj_candidate_num_, 2*step_time_adj_candidate_num_);
        SUaux_y_stab_mpc_.block  (0, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_) = MatrixXd::Identity(1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);

        input_index_calc += 2*step_time_adj_candidate_num_;

        Qcalc_stab_mpc_z_ = Pdpu_stab_mpc_.transpose()*Q_dcm_z*MatrixXd::Identity(N_stab_mpc, N_stab_mpc)*Pdpu_stab_mpc_ + R_dcm_z*Qmat_stab_mpc_R_;

        Qcalc_stab_mpc_  = SUp_stab_mpc_.transpose()*(SUpx_stab_mpc_.transpose()*(Pdpu_stab_mpc_.transpose()*Q_dcm_x*MatrixXd::Identity(N_stab_mpc, N_stab_mpc)*Pdpu_stab_mpc_ + R_dcm_x*Qmat_stab_mpc_R_)*SUpx_stab_mpc_
                                                     +SUpy_stab_mpc_.transpose()*(Pdpu_stab_mpc_.transpose()*Q_dcm_y*MatrixXd::Identity(N_stab_mpc, N_stab_mpc)*Pdpu_stab_mpc_ + R_dcm_y*Qmat_stab_mpc_R_)*SUpy_stab_mpc_)*SUp_stab_mpc_

                         + SUalp_stab_mpc_.transpose()*R_dalp*Qmat_stab_mpc_alp_*SUalp_stab_mpc_

                         + SUf_stab_mpc_.transpose()*(SUfx_stab_mpc_.transpose()*R_df_x*SUfx_stab_mpc_
                                                     +SUfy_stab_mpc_.transpose()*R_df_y*SUfy_stab_mpc_)*SUf_stab_mpc_;
        
        gxpcalc_stab_mpc_ = SUp_stab_mpc_.transpose()*SUpx_stab_mpc_.transpose()*Pdpu_stab_mpc_.transpose()*Q_dcm_x*MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        gypcalc_stab_mpc_ = SUp_stab_mpc_.transpose()*SUpy_stab_mpc_.transpose()*Pdpu_stab_mpc_.transpose()*Q_dcm_y*MatrixXd::Identity(N_stab_mpc, N_stab_mpc);
        gzpcalc_stab_mpc_ = Pdpu_stab_mpc_.transpose()*Q_dcm_z*MatrixXd::Identity(N_stab_mpc, N_stab_mpc);

        gxdfcalc_stab_mpc_ = SUf_stab_mpc_.transpose()*SUfx_stab_mpc_.transpose()*R_df_x*MatrixXd::Identity(1, 1);
        gydfcalc_stab_mpc_ = SUf_stab_mpc_.transpose()*SUfy_stab_mpc_.transpose()*R_df_y*MatrixXd::Identity(1, 1);

        gdalpcalc_stab_mpc_ = SUalp_stab_mpc_.transpose()*R_dalp*Qmat_stab_mpc_alp_;

        Sf1_stab_mpc_.setZero(N_stab_mpc, step_time_adj_candidate_num_);

        MPC_Stabilizer_u_mpc_sep_.setZero(3);

        MPC_first_loop = 1;
        cout << "Initialiazation of IS 3D DCM MPC is completed" << endl;
    }

    Sf1_stab_mpc_.setZero();
    Sf2_stab_mpc_.setZero();

    double step_x_norm = foot_step_support_frame_mpc_(current_step_num_mpc_, 0);
    double step_y_norm = foot_step_support_frame_mpc_(current_step_num_mpc_, 1);

    if(mpc_tick < hz_/mpc_freq)
    {
        MPC_Stabilizer_delf_mpc_x_(0) = step_x_norm;
        MPC_Stabilizer_delf_mpc_y_(0) = step_y_norm;
        MPC_Stabilizer_delf_mpc_ << MPC_Stabilizer_delf_mpc_x_, MPC_Stabilizer_delf_mpc_y_;

        MPC_Stabilizer_alpha_mpc_.setZero();
        MPC_Stabilizer_alpha_mpc_(0) = 1;

        MPC_Stabilizer_aux_mpc_x_ = MPC_Stabilizer_alpha_mpc_*MPC_Stabilizer_delf_mpc_x_;
        MPC_Stabilizer_aux_mpc_y_ = MPC_Stabilizer_alpha_mpc_*MPC_Stabilizer_delf_mpc_y_;
        MPC_Stabilizer_aux_mpc_ << MPC_Stabilizer_aux_mpc_x_, MPC_Stabilizer_aux_mpc_y_;
    }

    step_enable_bool_mpc_          = (bool)(current_step_num_mpc_)*(bool)(mpc_tick                   < t_total_const_ - t_dsp2_const_ - step_enable_time_fwd_*hz_ - step_enable_fix_time_pre_*hz_);
    step_enable_bool_one_tick_mpc_ = (bool)(current_step_num_mpc_)*(bool)(mpc_tick + MPC_synchro_hz_ < t_total_const_ - t_dsp2_const_ - step_enable_time_fwd_*hz_ - step_enable_fix_time_pre_*hz_);
    
    Eigen::VectorXd data_save_calc; data_save_calc.setZero(2*N_stab_mpc);
    data_save_calc << zmp_max_x_mpc_.segment(0, N_stab_mpc), zmp_max_y_mpc_.segment(0, N_stab_mpc);
    e_tmp_graph15 << data_save_calc.transpose() << endl;

    for(int i = 0; i < N_stab_mpc; i++)
    {
        for(int j = 0; j < 1; j++)
        { 
            int dsp_length_calc           = int((t_dsp1_const_ + t_dsp2_const_)/MPC_synchro_hz_ + 0.5);
            int next_step_start_prev_tick = max(ceil(((j+1)*(t_total_const_ - t_dsp2_const_) - mpc_tick)/MPC_synchro_hz_), 0.0);
            bool  next_step_prev_bool     = (bool)(mpc_tick + MPC_synchro_hz_*(i + 2) > ((j + 1)*t_total_mpc_ - t_dsp2_const_));
            bool nnext_step_prev_bool     = (bool)(mpc_tick + MPC_synchro_hz_*(i + 2) > ((j + 2)*t_total_mpc_ - t_dsp2_const_));

            if(walking_tick_mpc_ > t_temp_)
            {
                Sf1_stab_mpc_(i,0) = step_enable_bool_mpc_* next_step_prev_bool*(zmp_max_y_mpc_(i) - zmp_max_y_mpc_(int(t_dsp1_const_/MPC_synchro_hz_)))/(MPC_Stabilizer_delf_mpc_y_(0));
                zmp_max_x_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_max_x_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_max_x_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_max_x_mpc_(i);
                zmp_min_x_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_min_x_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_min_x_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_min_x_mpc_(i);
                zmp_max_y_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_max_y_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_max_y_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_max_y_mpc_(i);
                zmp_min_y_mpc_(i)  = step_enable_bool_mpc_*(next_step_prev_bool*zmp_min_y_mpc_(max(0, next_step_start_prev_tick - 2)) + (1 - next_step_prev_bool)*zmp_min_y_mpc_(i)) + (1 - step_enable_bool_mpc_)*zmp_min_y_mpc_(i);
            }
        }
    }

    for(int i = 1; i < step_time_adj_candidate_num_; i++)
    {
        Sf1_stab_mpc_.col(i) << Sf1_stab_mpc_.col(i-1).segment(1, N_stab_mpc - 1), min((Sf1_stab_mpc_(N_stab_mpc - 1, i-1) + (Sf1_stab_mpc_(N_stab_mpc - 1, i-1) - Sf1_stab_mpc_(N_stab_mpc - 2, i-1))), 1.0);
    }

e_tmp_graph16 << Sf1_stab_mpc_.col(0).transpose() << endl;

data_save_calc << zmp_max_x_mpc_.segment(0, N_stab_mpc), zmp_max_y_mpc_.segment(0, N_stab_mpc);
e_tmp_graph24 << data_save_calc.transpose() << endl;

    Eigen::VectorXd dcm_refx, dcm_refy, dcm_refz;
    dcm_refx.setZero(N_stab_mpc), dcm_refy.setZero(N_stab_mpc), dcm_refz.setZero(N_stab_mpc);

    dcm_refx = Planner_State_Prev_mpc_.block(0, 0, 1, N_stab_mpc).transpose() + b_*Planner_State_Prev_mpc_.block(1, 0, 1, N_stab_mpc).transpose();
    dcm_refy = Planner_State_Prev_mpc_.block(3, 0, 1, N_stab_mpc).transpose() + b_*Planner_State_Prev_mpc_.block(4, 0, 1, N_stab_mpc).transpose();
    dcm_refz = Planner_State_Prev_mpc_.block(6, 0, 1, N_stab_mpc).transpose() + b_*Planner_State_Prev_mpc_.block(7, 0, 1, N_stab_mpc).transpose();

    MPC_Stabilizer_state_mpc_(0) = com_measured_mpc_(0);
    MPC_Stabilizer_state_mpc_(1) = com_dot_measured_mpc_(0);
    MPC_Stabilizer_state_mpc_(3) = com_measured_mpc_(1);
    MPC_Stabilizer_state_mpc_(4) = com_dot_measured_mpc_(1);
    MPC_Stabilizer_state_mpc_(6) = com_measured_mpc_(2);
    MPC_Stabilizer_state_mpc_(7) = com_dot_measured_mpc_(2);

    int constraint_index = 0;

    gcalc_stab_mpc_z_ = gzpcalc_stab_mpc_*(Pdps_stab_mpc_*ssz_stab_mpc_*MPC_Stabilizer_state_mpc_ - dcm_refz);

    QP_MPC_Stabilizer_z_.EnableEqualityCondition(equality_condition_eps_);
    QP_MPC_Stabilizer_z_.UpdateMinProblem(Qcalc_stab_mpc_z_, gcalc_stab_mpc_z_);
    QP_MPC_Stabilizer_z_.DeleteSubjectToAx();
    QP_MPC_Stabilizer_z_.DeleteSubjectToX();

    const_A_mpc_. setZero(const_num_z, input_num_z);
    const_lb_mpc_.setZero(const_num_z, 1);
    const_ub_mpc_.setZero(const_num_z, 1);

    constraint_index += N_stab_mpc;

    //IS equality
    Eigen::MatrixXd b_IS_stab_mpc; b_IS_stab_mpc.resize(N_stab_mpc,1); b_IS_stab_mpc.col(0) = b_IS_plan_mpc_.col(0).segment(0,N_stab_mpc);
    //Pre planned Tail
    Eigen::MatrixXd Const_b_eq; Const_b_eq.setZero(3,1);
    Const_b_eq(2,0) = -(w_/(1 - lambda_is_calc))*(MPC_Stabilizer_state_mpc_(6) + MPC_Stabilizer_state_mpc_(7)/w_ - MPC_Stabilizer_state_mpc_(8));

    const_A_mpc_.row(constraint_index) = b_IS_stab_mpc.transpose();
    const_ub_mpc_.block(constraint_index, 0, 1, 1) = - Const_b_eq.block(2, 0, 1, 1);
    const_lb_mpc_.block(constraint_index, 0, 1, 1) = - Const_b_eq.block(2, 0, 1, 1);
    constraint_index += 1;
    
    Eigen::MatrixXd prev_state;                prev_state.setZero(9, N_stab_mpc);
    Eigen::MatrixXd prev_zmp;                  prev_zmp.  setZero(2, N_stab_mpc);
    Eigen::VectorXd prev_lambda_bb_vec, prev_lambda_bb_vec_calc;
    Eigen::VectorXd prev_w_square_over_lambda; prev_w_square_over_lambda.setZero(N_stab_mpc);

    QP_MPC_Stabilizer_z_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);

    if(QP_MPC_Stabilizer_z_.SolveQPoases(100, MPC_Stabilizer_u_mpc_z_))
    {
        //if((walking_tick_mpc_ - int(MPC_synchro_hz_) - 20)%int(2*hz_) == 0)
        if((((walking_tick_mpc_ - int(MPC_synchro_hz_) + 1)/int(MPC_synchro_hz_))%60) == 0) //30hz
        { 
            cout << "IS FIPM DCM Stabilizer Stepping Z direction Solved" << endl;
        }
    
        prev_state.row(6).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_z_;
        prev_state.row(7).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_z_;
        prev_state.row(8).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_z_;
    
        MPC_Stabilizer_state_mpc_.segment(6,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + B_mpc_*MPC_Stabilizer_u_mpc_(0);
    }
    else
    {
        cout << "IS FIPM DCM Stabilizer Stepping Z direction Not Solved" << endl;
        cout << (walking_tick_mpc_ - int(MPC_synchro_hz_) + 1)/int(MPC_synchro_hz_) + 1 << endl;

        prev_state.row(6).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_z_;
        prev_state.row(7).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_z_;
        prev_state.row(8).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_z_;

        MPC_Stabilizer_state_mpc_.segment(6,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(6,3) + B_mpc_*MPC_Stabilizer_u_mpc_(0);
    }

    prev_lambda_bb_vec_calc   = (prev_state.row(8) - GRAVITY*b_*b_*MatrixXd::Ones(1, N_stab_mpc)).array()/prev_state.row(6).array();
    prev_lambda_bb_vec        = MatrixXd::Ones(N_stab_mpc, 1) - prev_lambda_bb_vec_calc;
    prev_w_square_over_lambda = (w_*w_*prev_state.row(6)).array()/(w_*w_*(prev_state.row(6) - prev_state.row(8)) + GRAVITY*MatrixXd::Ones(1, N_stab_mpc)).array();

    MPC_Stabilizer_u_mpc_sep_(2) = MPC_Stabilizer_u_mpc_(0);

    gcalc_stab_mpc_ = gxpcalc_stab_mpc_  *(Pdps_stab_mpc_*ssx_stab_mpc_*MPC_Stabilizer_state_mpc_ - dcm_refx)
                     +gypcalc_stab_mpc_  *(Pdps_stab_mpc_*ssy_stab_mpc_*MPC_Stabilizer_state_mpc_ - dcm_refy)
                    
                     +gdalpcalc_stab_mpc_*(                                                       - MPC_Stabilizer_alpha_mpc_)

                     +gxdfcalc_stab_mpc_ *(                                                       - MPC_Stabilizer_delf_mpc_x_)
                     +gydfcalc_stab_mpc_ *(                                                       - MPC_Stabilizer_delf_mpc_y_);

    const_A_mpc_.setZero( const_num_xy, input_num_xy);
    const_lb_mpc_.setZero(const_num_xy, 1);
    const_ub_mpc_.setZero(const_num_xy, 1);

    int sqp_iter = 1;
    for(int s = 0; s < sqp_iter; s++)
    {
        std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();

        SQP_deldel_Qcalc_stab_mpc_ = Qcalc_stab_mpc_;
        SQP_del_g_calc_stab_mpc_   = Qcalc_stab_mpc_*MPC_Stabilizer_u_mpc_ + gcalc_stab_mpc_;

        QP_MPC_Stabilizer_.EnableEqualityCondition(equality_condition_eps_);
        QP_MPC_Stabilizer_.UpdateMinProblem(SQP_deldel_Qcalc_stab_mpc_, SQP_del_g_calc_stab_mpc_);
        QP_MPC_Stabilizer_.DeleteSubjectToAx();
        QP_MPC_Stabilizer_.DeleteSubjectToX();

        constraint_index = 0;
        ////VRP Constraint
        std::chrono::steady_clock::time_point t12 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count()*1e-6 << ",";
        }
        
        const_A_mpc_. block(constraint_index, 0, N_stab_mpc, input_num_xy) =   ((VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcpu_stab_mpc_
                                                                                + prev_w_square_over_lambda.asDiagonal()*Pvpu_stab_mpc_)*SUpxp_stab_mpc_
                                                                             - Sf1_stab_mpc_*SUaux_x_stab_mpc_*SUaux_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, N_stab_mpc, 1)            = - ((VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcpu_stab_mpc_
                                                                                + prev_w_square_over_lambda.asDiagonal()*Pvpu_stab_mpc_)*SUpxp_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + Sf1_stab_mpc_*SUaux_x_stab_mpc_*SUaux_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + (zmp_max_x_mpc_ 
                                                                                - (VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcps_stab_mpc_*ssx_stab_mpc_*MPC_Stabilizer_state_mpc_
                                                                                - prev_w_square_over_lambda.asDiagonal()*Pvps_stab_mpc_*ssx_stab_mpc_*MPC_Stabilizer_state_mpc_);
        const_lb_mpc_.block(constraint_index, 0, N_stab_mpc, 1)            = - ((VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcpu_stab_mpc_
                                                                                + prev_w_square_over_lambda.asDiagonal()*Pvpu_stab_mpc_)*SUpxp_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + Sf1_stab_mpc_*SUaux_x_stab_mpc_*SUaux_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + (zmp_min_x_mpc_ 
                                                                                - (VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcps_stab_mpc_*ssx_stab_mpc_*MPC_Stabilizer_state_mpc_
                                                                                - prev_w_square_over_lambda.asDiagonal()*Pvps_stab_mpc_*ssx_stab_mpc_*MPC_Stabilizer_state_mpc_);

        constraint_index += N_stab_mpc;

        const_A_mpc_. block(constraint_index, 0, N_stab_mpc, input_num_xy) =   ((VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcpu_stab_mpc_
                                                                                + prev_w_square_over_lambda.asDiagonal()*Pvpu_stab_mpc_)*SUpyp_stab_mpc_
                                                                             - Sf1_stab_mpc_*SUaux_y_stab_mpc_*SUaux_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, N_stab_mpc, 1)            = - ((VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcpu_stab_mpc_
                                                                                + prev_w_square_over_lambda.asDiagonal()*Pvpu_stab_mpc_)*SUpyp_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + Sf1_stab_mpc_*SUaux_y_stab_mpc_*SUaux_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + (zmp_max_y_mpc_ 
                                                                                - (VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcps_stab_mpc_*ssy_stab_mpc_*MPC_Stabilizer_state_mpc_
                                                                                - prev_w_square_over_lambda.asDiagonal()*Pvps_stab_mpc_*ssy_stab_mpc_*MPC_Stabilizer_state_mpc_);
        const_lb_mpc_.block(constraint_index, 0, N_stab_mpc, 1)            = - ((VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcpu_stab_mpc_
                                                                                + prev_w_square_over_lambda.asDiagonal()*Pvpu_stab_mpc_)*SUpyp_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + Sf1_stab_mpc_*SUaux_y_stab_mpc_*SUaux_stab_mpc_*MPC_Stabilizer_u_mpc_
                                                                             + (zmp_min_y_mpc_ 
                                                                                - (VectorXd::Ones(N_stab_mpc) - prev_w_square_over_lambda).asDiagonal()*Pcps_stab_mpc_*ssy_stab_mpc_*MPC_Stabilizer_state_mpc_
                                                                                - prev_w_square_over_lambda.asDiagonal()*Pvps_stab_mpc_*ssy_stab_mpc_*MPC_Stabilizer_state_mpc_);

        constraint_index += N_stab_mpc;

        std::chrono::steady_clock::time_point t13 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t13 - t12).count()*1e-6 << ",";
        }

        //Pre planned Tail
        Const_b_eq(0,0) = -(w_/(1 - lambda_is_calc))*(MPC_Stabilizer_state_mpc_(0) + MPC_Stabilizer_state_mpc_(1)/w_ - MPC_Stabilizer_state_mpc_(2))
                          +(pow(lambda_is_calc, N_stab_mpc)/(1 - pow(lambda_is_calc, N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(0))(0,0));
        Const_b_eq(1,0) = -(w_/(1 - lambda_is_calc))*(MPC_Stabilizer_state_mpc_(3) + MPC_Stabilizer_state_mpc_(4)/w_ - MPC_Stabilizer_state_mpc_(5))
                          +(pow(lambda_is_calc, N_stab_mpc)/(1 + pow(lambda_is_calc, N_step))*(b_IS_step_mpc_.transpose()*Pv_dot_ref_mpc_.col(1))(0,0));

        const_SQP_phi_mpc_.setZero(input_num_xy, input_num_xy);
        //const_SQP_pi_mpc_ = (b_IS_stab_mpc.transpose()*SUpx_stab_mpc_*SUp_stab_mpc_).transpose();
        const_SQP_pi_mpc_ = SUpxp_stab_mpc_.transpose()*b_IS_stab_mpc;
        const_SQP_ri_mpc_ = Const_b_eq.block(0, 0, 1, 1);

        //const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;
        const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

        //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_A_mpc_.row(constraint_index) = const_SQP_pi_mpc_.transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        constraint_index += 1;

        const_SQP_phi_mpc_.setZero(input_num_xy, input_num_xy);
        //const_SQP_pi_mpc_ = (b_IS_stab_mpc.transpose()*SUpy_stab_mpc_*SUp_stab_mpc_).transpose();
        const_SQP_pi_mpc_ = SUpyp_stab_mpc_.transpose()*b_IS_stab_mpc;
        const_SQP_ri_mpc_ = Const_b_eq.block(1, 0, 1, 1);
        //const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;
        const_SQP_hi_mpc_ = const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

        //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
        const_A_mpc_.row(constraint_index) = const_SQP_pi_mpc_.transpose();
        const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
        constraint_index += 1;

        std::chrono::steady_clock::time_point t14 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t14 - t13).count()*1e-6 << ",";
        }

        //alpha min max
        const_A_mpc_.block (constraint_index, 0, step_time_adj_candidate_num_, input_num_xy) = SUalp_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, step_time_adj_candidate_num_, 1)            = - SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Ones(step_time_adj_candidate_num_, 1);
        const_lb_mpc_.block(constraint_index, 0, step_time_adj_candidate_num_, 1)            = - SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Zero(step_time_adj_candidate_num_, 1);
        constraint_index += step_time_adj_candidate_num_;

        //alpha sum
        const_A_mpc_.block (constraint_index, 0, 1, input_num_xy) = MatrixXd::Ones(1, step_time_adj_candidate_num_)*SUalp_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, 1, 1)            = - MatrixXd::Ones(1, step_time_adj_candidate_num_)*SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Ones(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1)            = - MatrixXd::Ones(1, step_time_adj_candidate_num_)*SUalp_stab_mpc_*MPC_Stabilizer_u_mpc_ + MatrixXd::Ones(1,1);
        constraint_index += 1;

        std::chrono::steady_clock::time_point t15 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t15 - t14).count()*1e-6 << ",";
        }

        double delf_x_max = 0.25, delf_x_min = -0.20;
        double delf_y_max = 0.10, delf_y_min =  0.00;

        double delf_x_max_calc, delf_x_min_calc;
        double delf_y_max_calc, delf_y_min_calc;

        //delf min max
        const_A_mpc_.block (constraint_index, 0, 1, input_num_xy) = SUfx_stab_mpc_*SUf_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, 1, 1)            = - SUfx_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_x_max*MatrixXd::Ones(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1)            = - SUfx_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_x_min*MatrixXd::Ones(1,1);
        constraint_index += 1;

        delf_y_max_calc = step_y_norm + (foot_step_(current_step_num_mpc_, 6)*delf_y_min + (1 - foot_step_(current_step_num_mpc_, 6))*delf_y_max);
        delf_y_min_calc = step_y_norm - (foot_step_(current_step_num_mpc_, 6)*delf_y_max + (1 - foot_step_(current_step_num_mpc_, 6))*delf_y_min);

        const_A_mpc_.block (constraint_index, 0, 1, input_num_xy) = SUfy_stab_mpc_*SUf_stab_mpc_;
        const_ub_mpc_.block(constraint_index, 0, 1, 1)            = - SUfy_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_y_max_calc*MatrixXd::Ones(1,1);
        const_lb_mpc_.block(constraint_index, 0, 1, 1)            = - SUfy_stab_mpc_*SUf_stab_mpc_*MPC_Stabilizer_u_mpc_ + delf_y_min_calc*MatrixXd::Ones(1,1);
        constraint_index += 1;

        //alpha delf aux
        for(int i = 0; i < step_time_adj_candidate_num_; i++)
        {
            const_SQP_phi_mpc_ = SUalp_stab_mpc_.row(i).transpose()*SUfx_stab_mpc_*SUf_stab_mpc_;
            const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());
            const_SQP_pi_mpc_  = (- SUaux_x_stab_mpc_.row(i)*SUaux_stab_mpc_).transpose();
            const_SQP_ri_mpc_.setZero(1,1);
            const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

            //const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
            //const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            //const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            constraint_index += 1;

            const_SQP_phi_mpc_ = SUalp_stab_mpc_.row(i).transpose()*SUfy_stab_mpc_*SUf_stab_mpc_;
            const_SQP_phi_mpc_calc_ = 0.5*(const_SQP_phi_mpc_ + const_SQP_phi_mpc_.transpose());
            const_SQP_pi_mpc_  = (- SUaux_y_stab_mpc_.row(i)*SUaux_stab_mpc_).transpose();
            const_SQP_ri_mpc_.setZero(1,1);
            const_SQP_hi_mpc_ = MPC_Stabilizer_u_mpc_.transpose()*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_.transpose()*MPC_Stabilizer_u_mpc_ + const_SQP_ri_mpc_;

            const_A_mpc_.row(constraint_index) = (2*const_SQP_phi_mpc_calc_*MPC_Stabilizer_u_mpc_ + const_SQP_pi_mpc_).transpose();
            const_ub_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            const_lb_mpc_.block(constraint_index, 0, 1, 1) = - const_SQP_hi_mpc_;
            constraint_index += 1;

        }

        std::chrono::steady_clock::time_point t16 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t16 - t15).count()*1e-6 << ",";
        }

        QP_MPC_Stabilizer_.UpdateSubjectToAx(const_A_mpc_, const_lb_mpc_, const_ub_mpc_);
    
        if(QP_MPC_Stabilizer_.SolveQPoases(100, MPC_Stabilizer_SQP_du_mpc_))
        {
            //if((walking_tick_mpc_ - int(MPC_synchro_hz_) - 20)%int(2*hz_) == 0)
            if((((walking_tick_mpc_ - int(MPC_synchro_hz_) + 1)/int(MPC_synchro_hz_))%60) == 0) //30hz
            { 
                cout << "SQP Iter: " << s + 1 << endl;
                cout << "IS FIPM DCM Stabilizer Stepping XY direction Solved" << endl;
                if(s == sqp_iter - 1) { cout << endl; }
            }

            MPC_Stabilizer_u_mpc_ = MPC_Stabilizer_u_mpc_ + MPC_Stabilizer_SQP_du_mpc_;

            if(s == sqp_iter - 1)
            {
                prev_state.row(0).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(1).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(2).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);

                prev_state.row(3).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(4).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(5).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);

                prev_zmp.row(0)    = (prev_state.row(2).array() - (MatrixXd::Ones(1, N_stab_mpc) - prev_lambda_bb_vec.transpose()).array()*prev_state.row(0).array()).array()/prev_lambda_bb_vec.transpose().array();
                prev_zmp.row(1)    = (prev_state.row(5).array() - (MatrixXd::Ones(1, N_stab_mpc) - prev_lambda_bb_vec.transpose()).array()*prev_state.row(3).array()).array()/prev_lambda_bb_vec.transpose().array();

                MPC_Stabilizer_alpha_mpc_  = MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc, 1*step_time_adj_candidate_num_);

                MPC_Stabilizer_delf_mpc_   = MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc + 1*step_time_adj_candidate_num_,     2);
                MPC_Stabilizer_delf_mpc_x_ = MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc + 1*step_time_adj_candidate_num_ + 0, 1);
                MPC_Stabilizer_delf_mpc_y_ = MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc + 1*step_time_adj_candidate_num_ + 1, 1);

                MPC_Stabilizer_aux_mpc_    = MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2,                                  2*step_time_adj_candidate_num_);
                MPC_Stabilizer_aux_mpc_x_  = MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2 + 0*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);
                MPC_Stabilizer_aux_mpc_y_  = MPC_Stabilizer_u_mpc_.segment(2*N_stab_mpc + 1*step_time_adj_candidate_num_ + 2 + 1*step_time_adj_candidate_num_, 1*step_time_adj_candidate_num_);

                MPC_Stabilizer_state_mpc_.segment(0,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + B_mpc_*MPC_Stabilizer_u_mpc_(0*N_stab_mpc);
                MPC_Stabilizer_state_mpc_.segment(3,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + B_mpc_*MPC_Stabilizer_u_mpc_(1*N_stab_mpc);
            }
        }
        else
        {
            cout << "SQP Iter: " << s + 1 << endl;
            cout << "IS FIPM DCM Stabilizer Stepping XY direction Not Solved" << endl;
            cout << (walking_tick_mpc_ - int(MPC_synchro_hz_) + 1)/int(MPC_synchro_hz_) + 1 << endl;

            MPC_Stabilizer_u_mpc_ = MPC_Stabilizer_u_mpc_ + MPC_Stabilizer_SQP_du_mpc_;

            if(s == sqp_iter - 1)
            {
                prev_state.row(0).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(1).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);
                prev_state.row(2).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(0*N_stab_mpc, N_stab_mpc);

                prev_state.row(3).transpose() = Pcps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(4).transpose() = Pcvs_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pcvu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);
                prev_state.row(5).transpose() = Pvps_stab_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + Pvpu_stab_mpc_*MPC_Stabilizer_u_mpc_.segment(1*N_stab_mpc, N_stab_mpc);

                prev_zmp.row(0)    = (prev_state.row(2).array() - (MatrixXd::Ones(1, N_stab_mpc) - prev_lambda_bb_vec.transpose()).array()*prev_state.row(0).array()).array()/prev_lambda_bb_vec.transpose().array();
                prev_zmp.row(1)    = (prev_state.row(5).array() - (MatrixXd::Ones(1, N_stab_mpc) - prev_lambda_bb_vec.transpose()).array()*prev_state.row(3).array()).array()/prev_lambda_bb_vec.transpose().array();

                MPC_Stabilizer_alpha_mpc_.setZero();
                MPC_Stabilizer_alpha_mpc_(0) = 1;

                MPC_Stabilizer_delf_mpc_x_(0) = step_x_norm;
                MPC_Stabilizer_delf_mpc_y_(0) = step_y_norm;
                MPC_Stabilizer_delf_mpc_ << MPC_Stabilizer_delf_mpc_x_, MPC_Stabilizer_delf_mpc_y_;

                MPC_Stabilizer_aux_mpc_x_ = MPC_Stabilizer_alpha_mpc_*MPC_Stabilizer_delf_mpc_x_;
                MPC_Stabilizer_aux_mpc_y_ = MPC_Stabilizer_alpha_mpc_*MPC_Stabilizer_delf_mpc_y_;
                MPC_Stabilizer_aux_mpc_ << MPC_Stabilizer_aux_mpc_x_, MPC_Stabilizer_aux_mpc_y_;

                MPC_Stabilizer_state_mpc_.segment(0,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(0,3) + B_mpc_*MPC_Stabilizer_u_mpc_(0*N_stab_mpc);
                MPC_Stabilizer_state_mpc_.segment(3,3) = A_mpc_*MPC_Stabilizer_state_mpc_.segment(3,3) + B_mpc_*MPC_Stabilizer_u_mpc_(1*N_stab_mpc);
            }
        }

        MPC_Stabilizer_u_mpc_sep_(0) = MPC_Stabilizer_u_mpc_(0*N_stab_mpc);
        MPC_Stabilizer_u_mpc_sep_(1) = MPC_Stabilizer_u_mpc_(1*N_stab_mpc);

        std::chrono::steady_clock::time_point t17 = std::chrono::steady_clock::now();
        if(s == 0)
        {
            e_mpc_time_graph3 << std::chrono::duration_cast<std::chrono::microseconds>(t17 - t16).count()*1e-6 << endl;
        }
    }

    double calc_time_adj = 0.0;

    for (int i = 0; i < step_time_adj_candidate_num_; i++)
    {
        if(abs(MPC_Stabilizer_alpha_mpc_(i)) < 5e-3)
        {
            MPC_Stabilizer_alpha_mpc_(i) = 0.0;
        }
        calc_time_adj += i*MPC_Stabilizer_alpha_mpc_(i);
    }

    MPC_Stabilizer_time_adj_tick_x_mpc_ = max(round(calc_time_adj + 0.1), 0.0);
    MPC_Stabilizer_time_adj_tick_y_mpc_ = max(round(calc_time_adj + 0.1), 0.0);

    double time_adj_tick_mpc = 0.0;
    time_adj_tick_mpc = max(MPC_Stabilizer_time_adj_tick_x_mpc_, MPC_Stabilizer_time_adj_tick_y_mpc_);
    time_adj_tick_mpc = min(time_adj_tick_mpc, double(step_time_adj_candidate_num_ - 1));
    if(current_step_num_mpc_ != 0 && step_enable_bool_mpc_ == 0)
    {   
        t_total_mpc_ = t_total_const_ - time_adj_tick_mpc*hz_/thread3_hz_;
    }

    e_mpc_stabilizer_data << N_stab_mpc                          << "," << step_time_adj_candidate_num_        << "," << step_enable_bool_mpc_        << ","
                          << dcm_measured_mpc_(0)                << "," << dcm_measured_mpc_(1)                << "," << dcm_measured_mpc_(2)         << ","
                          << dcm_refx(0)                         << "," << dcm_refy(0)                         << "," << dcm_refz(0)                  << ","
                          << MPC_Stabilizer_state_mpc_(0)        << "," << MPC_Stabilizer_state_mpc_(3)        << "," << MPC_Stabilizer_state_mpc_(6) << ","
                          << MPC_Stabilizer_state_mpc_(1)        << "," << MPC_Stabilizer_state_mpc_(4)        << "," << MPC_Stabilizer_state_mpc_(7) << ","
                          << MPC_Stabilizer_state_mpc_(2)        << "," << MPC_Stabilizer_state_mpc_(5)        << "," << MPC_Stabilizer_state_mpc_(8) << ","
                          << zmp_max_x_mpc_(0)                   << "," << zmp_max_y_mpc_(0)                   << "," << 0                            << ","
                          << prev_zmp(0, 0)                      << "," << prev_zmp(1, 0)                      << "," << 0                            << ","
                          << MPC_Stabilizer_delf_mpc_x_(0)       << "," << MPC_Stabilizer_delf_mpc_y_(0)       << "," << 0                            << ","
                          << step_x_norm                         << "," << step_y_norm                         << "," << 0                            << ","
                          << MPC_Stabilizer_time_adj_tick_x_mpc_ << "," << calc_time_adj                       << "," << time_adj_tick_mpc            << ","
                          << endl;

    data_save_calc.setZero(2*N_stab_mpc);
    data_save_calc << dcm_refx, dcm_refy;
    e_tmp_graph8 << data_save_calc.transpose() << endl;
    data_save_calc << prev_state.row(2).transpose(), prev_state.row(5).transpose();
    e_tmp_graph9 << data_save_calc.transpose() << endl;
    data_save_calc << prev_state.row(0).transpose() + b_*prev_state.row(1).transpose(), prev_state.row(3).transpose() + b_*prev_state.row(4).transpose();
    e_tmp_graph10 << data_save_calc.transpose() << endl;
    data_save_calc << zmp_max_x_mpc_.segment(0, N_stab_mpc), zmp_max_y_mpc_.segment(0, N_stab_mpc);
    e_tmp_graph11 << data_save_calc.transpose() << endl;
    data_save_calc << prev_zmp.row(0).transpose(), prev_zmp.row(1).transpose();
    e_tmp_graph12 << data_save_calc.transpose() << endl;
    data_save_calc.setZero(step_time_adj_candidate_num_);
    data_save_calc << MPC_Stabilizer_alpha_mpc_;
    e_tmp_graph13 << data_save_calc.transpose() << endl;
    data_save_calc.setZero(2*step_time_adj_candidate_num_);
    data_save_calc << MPC_Stabilizer_aux_mpc_x_, MPC_Stabilizer_aux_mpc_y_;
    e_tmp_graph14 << data_save_calc.transpose() << endl;

    step_enable_bool_mpc_ = (bool)(mpc_tick + MPC_synchro_hz_ < t_total_const_ - t_dsp2_const_ - step_enable_time_fwd_*hz_ - step_enable_fix_time_pre_*hz_);
}

////////////////////// Econom2 function end
void AvatarController::getComTrajectory()
{
    if (walking_tick_ == 0)
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

        MPC_Stabilizer_state_main_.setZero(9);
    }

    zmp_start_time_ = ((bool)current_step_num_)*t_start_;

    previewcontroller(0.0005, 3200, walking_tick_ - zmp_start_time_, xi_mj_, yi_mj_, xs_mj_, ys_mj_, UX_mj_, UY_mj_, Gi_mj_, Gd_mj_, Gx_mj_, A_mj_, B_mj_, C_mj_, xd_mj_, yd_mj_);

    xs_mj_ = xd_mj_;
    ys_mj_ = yd_mj_;

    com_desired_(0) = xd_mj_(0);
    com_desired_(1) = yd_mj_(0);
    com_desired_(2) = zc_mj_;

    MPC_Stabilizer_state_main_(0) = com_desired_(0);
    MPC_Stabilizer_state_main_(3) = com_desired_(1);
    MPC_Stabilizer_state_main_(6) = com_desired_(2);

    MPC_Stabilizer_state_main_(2) = zmp_desired_(0) + 1.20*(dcm_measured_(0) - cp_desired_(0));
    MPC_Stabilizer_state_main_(5) = zmp_desired_(1) + 1.10*(dcm_measured_(1) - cp_desired_(1));
    MPC_Stabilizer_state_main_(8) = zc_mj_          + 1.01*(dcm_measured_(2) - zc_mj_);

    if (walking_tick_ == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1)
    {
        Eigen::Vector3d com_pos_prev;
        Eigen::Vector3d com_pos;
        Eigen::Vector3d com_vel_prev;
        Eigen::Vector3d com_vel;
        Eigen::Vector3d com_acc_prev;
        Eigen::Vector3d com_acc;
        Eigen::Matrix3d frame_rot_diff;
        Eigen::Vector3d frame_pos_diff;

        frame_rot_diff = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        for (int i = 0; i < 3; i++)
            frame_pos_diff(i) = foot_step_support_frame_(current_step_num_, i);
        
        frame_pos_diff(0) = frame_pos_diff(0) + modified_del_zmp_(current_step_num_,0); // 왼발 오른발 나눌까?
        frame_pos_diff(1) = frame_pos_diff(1) + modified_del_zmp_(current_step_num_,1);  
        
        com_pos_prev(0) = xs_mj_(0);
        com_pos_prev(1) = ys_mj_(0);
        com_pos = frame_rot_diff * (com_pos_prev - frame_pos_diff);

        com_vel_prev(0) = xs_mj_(1);
        com_vel_prev(1) = ys_mj_(1);
        com_vel_prev(2) = 0.0;
        com_vel = frame_rot_diff * com_vel_prev;

        com_acc_prev(0) = xs_mj_(2);
        com_acc_prev(1) = ys_mj_(2);
        com_acc_prev(2) = 0.0;
        com_acc = frame_rot_diff * com_acc_prev;

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
    q_des(3) = q_des(3);                                                                                               // Knee pitch
    q_des(4) = q_des(4);                                                                                               // Ankle pitch
    q_des(5) = atan2(L_r(1), L_r(2));                                                                                  // Ankle roll

    q_des(6) = atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    q_des(7) = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_des(6)) + R_Hip_rot_mat(1, 1) * cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = atan2(R_r(1), R_r(2));

    if (walking_tick_ == 0)
    {
        sc_joint_err.setZero();
    }

    if (walking_tick_ == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {                                                                                           //5.3, 0
        sc_joint_before.setZero();
        sc_joint_before = q_des;
    }
    if (current_step_num_ != 0 && walking_tick_ == t_start_) // step change
    {                                                          //5.3005, 1
        sc_joint_after.setZero();
        sc_joint_after = q_des;

        sc_joint_err = sc_joint_after - sc_joint_before;
    }
    if (current_step_num_ != 0)
    {
        for (int i = 0; i < 12; i++)
        {
            SC_joint(i) = DyrosMath::cubic(walking_tick_, t_start_, t_start_ + 0.005 * hz_, sc_joint_err(i), 0.0, 0.0, 0.0);
        }

        if (walking_tick_ >= t_start_ && walking_tick_ < t_start_ + 0.005 * hz_)
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

    if (walking_tick_ < t_start_ + t_dsp1_)
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
    else if (walking_tick_ >= t_start_ + t_dsp1_ && walking_tick_ < t_start_ + t_total_ - t_dsp2_) // SSP
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
    else if (walking_tick_ >= t_start_ + t_total_ - t_dsp2_ && walking_tick_ < t_start_ + t_total_)
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
    step_length_x_ = 0.2;
    step_length_y_ = 0.0;
    is_right_foot_swing_ = 1;
    
    t_dsp1_        = 0.10 * hz_;
    t_dsp2_        = 0.10 * hz_;
    t_total_       = 0.9 * hz_;

    t_dsp1_const_  = 0.10 * hz_;
    t_dsp2_const_  = 0.10 * hz_;
    t_total_const_ = 0.9 * hz_;

    t_ssp_ = t_total_ - t_dsp1_ - t_dsp2_;
    //foot_width_  = zmp_y_max;
    foot_width_  = zmp_y_max_foot_width_;
    foot_height_ = 0.055;

    t_temp_ = 3.0 * hz_;
    t_last_ = t_total_ + t_temp_;
    t_start_ = t_temp_ + 1;

    current_step_num_ = 0;
    pelv_height_offset_ = 0.0; // change pelvis height for manipulation when the robot stop walking
}

void AvatarController::updateNextStepTime()
{       
    if (walking_tick_ == t_last_)
    {   
        if (current_step_num_ != total_step_num_ - 1)
        {   
            t_start_ = t_last_ + 1;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;            
        }
        
    }
    if (current_step_num_ == total_step_num_ - 1 && walking_tick_ >= t_last_ + t_total_)
    {
        // walking_enable_ = false;
        // cout << "Last " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;
    }
    else
    {
        walking_tick_++;
        scenario_tick_++;
        if((walking_tick_ == 2000 && param_scenario_ == 1) && (scenario_tick_ < 18000))
        {
            walking_tick_--;
            cout << "walking_tick_: " << walking_tick_ << endl;
            cout << "scenario_tick_: " << scenario_tick_ << endl << endl;
        }
    }
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
{
    double alpha = 0;
    double fr = 0, fl = 0;
    double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
    double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;

    double ZMP_X_DES_CALC = 0.0;
    double ZMP_Y_DES_CALC = 0.0;
    double lambda_desired = 0.0;

    Eigen::Vector2d ZMP_calc_real; ZMP_calc_real.setZero();
    ZMP_calc_real(0) = DyrosMath::minmax_cut(MPC_Stabilizer_state_main_(2), ref_zmp_wo_offset_(walking_tick_ - ((bool)current_step_num_)*t_start_, 0) - zmp_x_min_foot_width_, ref_zmp_wo_offset_(walking_tick_ - ((bool)current_step_num_)*t_start_, 0) + zmp_x_max_foot_width_); 
    ZMP_calc_real(1) = DyrosMath::minmax_cut(MPC_Stabilizer_state_main_(5), ref_zmp_wo_offset_(walking_tick_ - ((bool)current_step_num_)*t_start_, 1) - zmp_y_min_foot_width_, ref_zmp_wo_offset_(walking_tick_ - ((bool)current_step_num_)*t_start_, 1) + zmp_y_max_foot_width_); 

    lambda_desired = (MPC_Stabilizer_state_main_(6) - MPC_Stabilizer_state_main_(8) + GRAVITY*b_*b_)/(MPC_Stabilizer_state_main_(6)*b_*b_);
    ZMP_X_DES_CALC = (ZMP_calc_real(0) - (1 - lambda_desired*b_*b_)*MPC_Stabilizer_state_main_(0))/(lambda_desired*b_*b_);
    ZMP_Y_DES_CALC = (ZMP_calc_real(1) - (1 - lambda_desired*b_*b_)*MPC_Stabilizer_state_main_(3))/(lambda_desired*b_*b_);

    vrp_desired_ << ZMP_calc_real(0), ZMP_calc_real(1), MPC_Stabilizer_state_main_(8);
    
    e_tmp_graph1 << com_desired_(0)  << "," << com_desired_(1)  << "," << com_desired_(2)  << ","
                 << dcm_desired_(0)  << "," << dcm_desired_(1)  << "," << dcm_desired_(2)  << ","
                 << dcm_measured_(0) << "," << dcm_measured_(1) << "," << dcm_measured_(2) << ","
                 << com_measured_(0) << "," << com_measured_(1) << "," << com_measured_(2) << ","
                 << ZMP_X_DES_CALC   << "," << ZMP_Y_DES_CALC   << "," << 0                << ","
                 << endl;
    double real_robot_mass_offset_ = 52/GRAVITY; // 42 75
    if(param_sim_mode_) { real_robot_mass_offset_ = 0.0; }

    Eigen::Vector3d foot_force;
    foot_force(0) = (rd_.link_[COM_id].mass + real_robot_mass_offset_)* w_*w_*(com_desired_(0) - vrp_desired_(0));
    foot_force(1) = (rd_.link_[COM_id].mass + real_robot_mass_offset_)* w_*w_*(com_desired_(1) - vrp_desired_(1));
    foot_force(2) = (rd_.link_[COM_id].mass + real_robot_mass_offset_)*(w_*w_*(com_desired_(2) - vrp_desired_(2)) + GRAVITY);

    //double calc_z_max = foot_width_;
    double calc_z_max = 0.085;

    alpha = (ZMP_Y_DES_CALC - (rfoot_support_current_.translation()(1) + calc_z_max)) / ((lfoot_support_current_.translation()(1) - calc_z_max) - (rfoot_support_current_.translation()(1) + calc_z_max));

    alpha      = DyrosMath::minmax_cut(alpha,      0.0, 1.0);

    if(walking_tick_ == 0) { alpha_lpf_ = alpha; }
    alpha_lpf_ = 1 / (1 + 2 * M_PI * 6.0 * del_t) * alpha_lpf_ + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * alpha;

    alpha_lpf_ = DyrosMath::minmax_cut(alpha_lpf_, 0.0, 1.0);

    fr = - (1 - alpha_lpf_)*foot_force(2);
    rfoot_contact_wrench_(0) = -(1 - alpha_lpf_)*foot_force(0) + (-(1 - alpha_lpf_)*foot_force(0) - rd_.RF_FT(0));
    rfoot_contact_wrench_(1) = -(1 - alpha_lpf_)*foot_force(1) + (-(1 - alpha_lpf_)*foot_force(1) - rd_.RF_FT(1));
    rfoot_contact_wrench_(2) = -(1 - alpha_lpf_)*foot_force(2) + (-(1 - alpha_lpf_)*foot_force(2) - rd_.RF_FT(2));

    fl =      - alpha_lpf_ *foot_force(2);
    lfoot_contact_wrench_(0) =     - alpha_lpf_ *foot_force(0) + (    - alpha_lpf_ *foot_force(0) - rd_.LF_FT(0));
    lfoot_contact_wrench_(1) =     - alpha_lpf_ *foot_force(1) + (    - alpha_lpf_ *foot_force(1) - rd_.LF_FT(1));
    lfoot_contact_wrench_(2) =     - alpha_lpf_ *foot_force(2) + (    - alpha_lpf_ *foot_force(2) - rd_.LF_FT(2));

    if (walking_tick_ == 0)
    {
        F_F_input = 0.0;
        F_T_L_x_input = 0.0;
        F_T_R_x_input = 0.0;
        F_T_L_y_input = 0.0;
        F_T_R_y_input = 0.0;
    }

    //////////// Force
    F_F_input_dot = 0.0001 * ((l_ft_(2) - r_ft_(2)) - (fl - fr)) - 3.0 * F_F_input;
    F_F_input = F_F_input + F_F_input_dot * del_t;
    F_F_input = DyrosMath::minmax_cut(F_F_input, -0.02, 0.02);

    //////////// Torque
    Tau_all_x = -((rfoot_support_current_.translation()(1) - ZMP_Y_DES_CALC) * fr + (lfoot_support_current_.translation()(1) - ZMP_Y_DES_CALC) * fl);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - ZMP_X_DES_CALC) * fr + (lfoot_support_current_.translation()(0) - ZMP_X_DES_CALC) * fl);
    Tau_all_x = DyrosMath::minmax_cut(Tau_all_x, - 100.0, 100.0);
    Tau_all_y = DyrosMath::minmax_cut(Tau_all_y, - 100.0, 100.0);

    Tau_R_x = (1 - alpha) * Tau_all_x;
    rfoot_contact_wrench_(3) = Tau_R_x;
    Tau_L_x = (    alpha) * Tau_all_x;
    lfoot_contact_wrench_(3) = Tau_L_x;

    Tau_R_y = -(1 - alpha) * Tau_all_y;
    rfoot_contact_wrench_(4) = Tau_R_y;
    Tau_L_y = -     alpha  * Tau_all_y;
    lfoot_contact_wrench_(4) = Tau_L_y;

    contact_wrench_.segment(0,6) = -lfoot_contact_wrench_;
    contact_wrench_.segment(6,6) = -rfoot_contact_wrench_;

    // Roll 방향 (-0.02/-30 0.9초)
    double Kp_mj_ft = 0.04;
    double Kd_mj_ft = 40.0;
    double F_T_minmaxcut_mj_ft = 0.20;
    
    if(param_sim_mode_)
    {
        Kp_mj_ft = 0.04;
        Kd_mj_ft = 40.0;
        F_T_minmaxcut_mj_ft = 0.15;
    }

    F_T_L_x_input_dot = - Kp_mj_ft * (Tau_L_x - l_ft_LPF(3)) - Kd_mj_ft * F_T_L_x_input;
    F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
    F_T_L_x_input = DyrosMath::minmax_cut(F_T_L_x_input, - F_T_minmaxcut_mj_ft, F_T_minmaxcut_mj_ft);

    F_T_R_x_input_dot = - Kp_mj_ft * (Tau_R_x - r_ft_LPF(3)) - Kd_mj_ft * F_T_R_x_input;
    F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
    F_T_R_x_input = DyrosMath::minmax_cut(F_T_R_x_input, - F_T_minmaxcut_mj_ft, F_T_minmaxcut_mj_ft);
    
    F_T_L_y_input_dot =   Kp_mj_ft * (Tau_L_y - l_ft_LPF(4)) - Kd_mj_ft * F_T_L_y_input;
    F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
    F_T_L_y_input = DyrosMath::minmax_cut(F_T_L_y_input, - F_T_minmaxcut_mj_ft, F_T_minmaxcut_mj_ft);

    F_T_R_y_input_dot =   Kp_mj_ft * (Tau_R_y - r_ft_LPF(4)) - Kd_mj_ft * F_T_R_y_input;
    F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;
    F_T_R_y_input = DyrosMath::minmax_cut(F_T_R_y_input, - F_T_minmaxcut_mj_ft, F_T_minmaxcut_mj_ft);

    e_tmp_graph18 << ZMP_Y_DES_CALC << "," << alpha         << "," << alpha_lpf_ << ","
                  << Tau_all_x      << "," << Tau_L_x       << "," << Tau_R_x    << ","
                  << F_T_L_x_input  << "," << F_T_R_x_input << ","
                  << lfoot_support_current_.translation()(1) << "," << rfoot_support_current_.translation()(1) << ","
                  << fl << "," << fr << ","
                  << endl;
}

void AvatarController::updateInitialStateJoy()
{
    if (walking_tick_ == 0)
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
    else if (current_step_num_ != 0 && walking_tick_ == t_start_) // step change
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
    if (walking_tick_ == t_start_)
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

    if (walking_tick_ != 0)
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

    if (walking_tick_ != 0)
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
    if (walking_tick_ == t_last_)
    {
        if (current_step_num_ != total_step_num_ - 1)
        {
            t_start_ = t_last_ + 1;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;
        }
    }

    walking_tick_++;

    if (current_step_num_ == total_step_num_ - 1 && walking_tick_ >= t_last_ + t_total_ + 1)
    {
        walking_enable_ = false;
        walking_tick_ = 0;
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

Eigen::VectorQd AvatarController::MitWholebodyInverseDynamicsController(const Eigen::VectorQd &torque_prev, const Eigen::VectorVQd &qddot_cmd, const Eigen::Vector12d &f_c_cmd)
{
    const int control_size_torque = MODEL_DOF;
    const int control_size_qddot  = MODEL_DOF_VIRTUAL;
    const int control_size_contact_force = 12;
    const int control_size_contact_accel = 12;
    const int variable_size = control_size_torque + control_size_qddot + control_size_contact_force + control_size_contact_accel;

    const int constraint_size_dynamics = MODEL_DOF_VIRTUAL;
    const int constraint_size_torque   = MODEL_DOF;
    const int constraint_size_contact  = 12;
    const int constraint_size_friction = 32;
    const int constraint_size = constraint_size_dynamics + constraint_size_torque + constraint_size_contact + constraint_size_friction;

    if(is_wbid_init_ == true)
    {
        QP_wbid.InitializeProblemSize(variable_size, constraint_size);
        
        J_lfoot_.setZero();     J_lfoot_     = rd_.link_[Left_Foot].Jac();
        J_lfoot_pre_.setZero(); J_lfoot_pre_ = rd_.link_[Left_Foot].Jac();
        J_lfoot_dot_.setZero();

        J_rfoot_.setZero();     J_rfoot_     = rd_.link_[Right_Foot].Jac();
        J_rfoot_pre_.setZero(); J_rfoot_pre_ = rd_.link_[Right_Foot].Jac();
        J_rfoot_dot_.setZero();

        is_wbid_init_ = false;
    }

    J_lfoot_pre_ = J_lfoot_;
    J_rfoot_pre_ = J_rfoot_;

    J_lfoot_ = rd_.link_[Left_Foot].Jac();
    J_rfoot_ = rd_.link_[Right_Foot].Jac();

    J_lfoot_dot_ = (J_lfoot_ - J_lfoot_pre_) * hz_;
    J_rfoot_dot_ = (J_rfoot_ - J_rfoot_pre_) * hz_;

    Eigen::MatrixXd J_contact;     J_contact.setZero(control_size_contact_force, control_size_qddot);
    Eigen::MatrixXd J_contact_dot; J_contact_dot.setZero(control_size_contact_force, control_size_qddot);

    J_contact.topRows(6)        = J_lfoot_;
    J_contact.bottomRows(6)     = J_rfoot_;
    J_contact_dot.topRows(6)    = J_lfoot_dot_;
    J_contact_dot.bottomRows(6) = J_rfoot_dot_;

    double W_qb_x     = 1000.0;
    double W_qb_y     = 1000.0;
    double W_qb_z     = 1000.0;
    double W_qb_roll  = 1000.0;
    double W_qb_pitch = 1000.0;
    double W_qb_yaw   = 1000.0;

    double W_qa       = 2000.0;

    double W_torque_1 = 100.0;
    double W_torque_2 = 2500.0;
    double W_c_lfoot  = 1.0;
    double W_c_rfoot  = 1.0;
    double W_f_lfoot  = 2000.0;
    double W_f_rfoot  = 200;

    if(is_dsp_fast_)    
    {
        W_c_lfoot = 1.0;
        W_c_rfoot = 1.0;
    }
    else if(is_ssp_fast_)
    {
        if(is_lfoot_support_fast_ == true)
        {
            W_c_lfoot = 1.0;
            W_c_rfoot = 0.0;
        }
        else if(is_rfoot_support_fast_ == true)
        {
            W_c_lfoot = 0.0;
            W_c_rfoot = 1.0;
        }
    }

    Eigen::MatrixXd H_wbid; H_wbid.setZero(variable_size, variable_size);
    unsigned int H_idx = 0;

    Eigen::MatrixXd W_q; W_q.setIdentity(control_size_qddot, control_size_qddot);
    W_q(0,0) = W_qb_x;
    W_q(1,1) = W_qb_y;
    W_q(2,2) = W_qb_z;
    W_q(3,3) = W_qb_roll;
    W_q(4,4) = W_qb_pitch;
    W_q(5,5) = W_qb_yaw;

    for(int i = 6; i < control_size_qddot; ++i) {W_q(i,i) = W_qa;}
    H_wbid.block(H_idx, H_idx, control_size_qddot, control_size_qddot) = W_q + 0.1*rd_.A_;
    H_idx += control_size_qddot;
    H_wbid.block(H_idx, H_idx, control_size_torque, control_size_torque) = (W_torque_1 + W_torque_2)*Eigen::MatrixXd::Identity(control_size_torque, control_size_torque);
    H_idx += control_size_torque;
    H_wbid.block(H_idx, H_idx, control_size_contact_accel / 2, control_size_contact_accel / 2) = W_c_lfoot * Eigen::MatrixXd::Identity(control_size_contact_accel / 2, control_size_contact_accel/ 2);
    H_idx += control_size_contact_accel / 2;
    H_wbid.block(H_idx, H_idx, control_size_contact_accel / 2, control_size_contact_accel / 2) = W_c_rfoot * Eigen::MatrixXd::Identity(control_size_contact_accel / 2, control_size_contact_accel/ 2);
    H_idx += control_size_contact_accel / 2;
    H_wbid.block(H_idx, H_idx, control_size_contact_force / 2, control_size_contact_force / 2) = W_f_lfoot * Eigen::MatrixXd::Identity(control_size_contact_force / 2, control_size_contact_force / 2);
    H_idx += control_size_contact_force / 2;
    H_wbid.block(H_idx, H_idx, control_size_contact_force / 2, control_size_contact_force / 2) = W_f_rfoot * Eigen::MatrixXd::Identity(control_size_contact_force / 2, control_size_contact_force / 2);
    H_idx += control_size_contact_force / 2;    

    Eigen::MatrixXd g_wbid; g_wbid.setZero(variable_size,1);
    unsigned int g_idx = 0;
    g_wbid.block(g_idx, 0, control_size_qddot, 1) = - W_q * qddot_cmd;
    g_idx += control_size_qddot;
    g_wbid.block(g_idx, 0, control_size_torque, 1) = - W_torque_2 * torque_prev;
    g_idx += control_size_torque;
    g_idx += control_size_contact_accel;
    g_wbid.block(g_idx, 0, control_size_contact_force/2, 1) = - W_f_lfoot * f_c_cmd.segment(0,6);
    g_idx += control_size_contact_force/2;
    g_wbid.block(g_idx, 0, control_size_contact_force/2, 1) = - W_f_rfoot * f_c_cmd.segment(6,6);

    //constraints
    Eigen::MatrixXd S_T; S_T.setZero(control_size_qddot, control_size_torque);
    S_T.bottomRows(control_size_torque).setIdentity();

    Eigen::MatrixXd U_fric;     U_fric.setZero(constraint_size_friction, control_size_contact_force);
    Eigen::MatrixXd U_fric_sub; U_fric_sub.setZero(constraint_size_friction/2, control_size_contact_force/2);
    double X = 0.15; double Y = 0.075; double mu = 0.7;

    U_fric_sub << -1,  0,           -mu,   0,   0,  0,
                  +1,  0,           -mu,   0,   0,  0,
                   0, -1,           -mu,   0,   0,  0,
                   0, +1,           -mu,   0,   0,  0,
                   0,  0,            -Y,  -1,   0,  0,
                   0,  0,            -Y,  +1,   0,  0,
                   0,  0,            -X,   0,  -1,  0,
                   0,  0,            -X,   0,  +1,  0,
                  -Y, -X, -(X + Y) * mu, -mu, +mu, -1, 
                  +Y, +X, -(X + Y) * mu, +mu, -mu, -1, 
                  +Y, -X, -(X + Y) * mu, +mu, +mu, -1, 
                  +Y, +X, -(X + Y) * mu, +mu, +mu, -1, 
                  +Y, -X, -(X + Y) * mu, +mu, +mu, +1, 
                  +Y, +X, -(X + Y) * mu, +mu, -mu, +1, 
                  -Y, -X, -(X + Y) * mu, -mu, -mu, +1, 
                  -Y, +X, -(X + Y) * mu, -mu, +mu, +1;
    
    U_fric.block(                         0,                            0, constraint_size_friction/2, control_size_contact_force/2) = U_fric_sub; //left  foot
    U_fric.block(constraint_size_friction/2, control_size_contact_force/2, constraint_size_friction/2, control_size_contact_force/2) = U_fric_sub; //right foot

    Eigen::MatrixXd A_wbid;   A_wbid.setZero(constraint_size, variable_size);
    Eigen::MatrixXd lbA_wbid; lbA_wbid.setZero(constraint_size, 1);
    Eigen::MatrixXd ubA_wbid; ubA_wbid.setZero(constraint_size, 1);

    unsigned int A_idx = 0;
    A_wbid.block(A_idx, 0                                                                    , constraint_size_dynamics, control_size_qddot) = rd_.A_;
    A_wbid.block(A_idx, control_size_qddot                                                   , constraint_size_dynamics, control_size_torque) = - S_T;
    A_wbid.block(A_idx, control_size_qddot + control_size_torque                             , constraint_size_dynamics, control_size_contact_accel).setZero();
    A_wbid.block(A_idx, control_size_qddot + control_size_torque + control_size_contact_accel, constraint_size_dynamics, control_size_contact_force) = - J_contact.transpose();

    A_idx += constraint_size_dynamics;
    A_wbid.block(A_idx, control_size_qddot,                                                    constraint_size_torque,   control_size_torque).setIdentity();

    A_idx += constraint_size_torque;
    A_wbid.block(A_idx, 0,                                                                     constraint_size_contact,  control_size_qddot) = J_contact;
    A_wbid.block(A_idx, control_size_qddot + control_size_torque,                              constraint_size_contact,  control_size_contact_accel) = -Eigen::MatrixXd::Identity(control_size_contact_accel, control_size_contact_accel);

    A_idx += constraint_size_contact;
    A_wbid.block(A_idx, control_size_qddot + control_size_torque + control_size_contact_accel, constraint_size_friction, control_size_contact_force) = U_fric;

    A_idx = 0;
    lbA_wbid.block(A_idx, 0, constraint_size_dynamics, 1) = -rd_.G;
    A_idx += constraint_size_dynamics;
    lbA_wbid.block(A_idx, 0, constraint_size_torque,   1) =-rd_.torque_limit;
    A_idx += constraint_size_torque;
    lbA_wbid.block(A_idx, 0, constraint_size_contact,  1) = -J_contact_dot * rd_.q_dot_virtual_;
    A_idx += constraint_size_contact;
    lbA_wbid.block(A_idx, 0, constraint_size_friction, 1).setConstant(-std::numeric_limits<double>::infinity());

    A_idx = 0;
    ubA_wbid.block(A_idx, 0, constraint_size_dynamics, 1) = -rd_.G;
    A_idx += constraint_size_dynamics;
    ubA_wbid.block(A_idx, 0, constraint_size_torque,   1) = rd_.torque_limit;
    A_idx += constraint_size_torque;
    ubA_wbid.block(A_idx, 0, constraint_size_contact,  1) = -J_contact_dot * rd_.q_dot_virtual_;
    A_idx += constraint_size_contact;
    ubA_wbid.block(A_idx, 0, constraint_size_friction, 1).setZero();

    QP_wbid.EnableEqualityCondition(equality_condition_eps_);
    QP_wbid.UpdateMinProblem(H_wbid, g_wbid);
    QP_wbid.DeleteSubjectToAx();
    QP_wbid.UpdateSubjectToAx(A_wbid, lbA_wbid, ubA_wbid);
    Eigen::VectorXd X_opt_; X_opt_.setZero(variable_size);    
    Eigen::VectorXd torque_opt_; torque_opt_.setZero(MODEL_DOF);    
    //if (QP_wbid.SolveQPoases(200, X_opt_))
    //{
    //    torque_opt_ = X_opt_.segment(MODEL_DOF_VIRTUAL, MODEL_DOF);
    //}
    //else
    //{
    //    torque_opt_.setZero();
    //    std::cout << "WBD CONTROLLER CANNOT BE SOLVED!" << std::endl;
    //}

    return (torque_opt_);
}

void AvatarController::stateMachine()
{
    if(foot_step_(current_step_num_, 6) == 0)
    {
        is_rfoot_support_ = true;
        is_lfoot_support_ = false;
    }
    else if(foot_step_(current_step_num_, 6) == 1)
    {
        is_rfoot_support_ = false;
        is_lfoot_support_ = true;
    }

    if(walking_tick_ >= t_start_ + t_dsp1_ && walking_tick_ < t_start_ + t_total_ - t_dsp2_)
    {
        is_ssp_ = true;
        is_dsp_ = false;

        num_contact_ = 1;
    }
    else
    {
        is_ssp_ = false;
        is_dsp_ = true;

        num_contact_ = 2;
    }
}

void AvatarController::getVirtualJointState(const Eigen::Isometry3d& transform_global_to_float, const Eigen::Isometry3d& transform_float_to_support)
{
    q_virtual_.setZero();

    Eigen::Vector3d q_virtual_pos;   q_virtual_pos.setZero();
    Eigen::Matrix3d q_virtual_rotm;  q_virtual_rotm.setZero();
    Eigen::Vector3d q_virtual_euler; q_virtual_euler.setZero();

    q_virtual_pos = rd_.q_virtual_.segment(0,3);

    Quaterniond q_virtual_quat(rd_.q_virtual_(39), rd_.q_virtual_(3), rd_.q_virtual_(4), rd_.q_virtual_(5));    // w, x, y, z
    q_virtual_rotm = q_virtual_quat.normalized().toRotationMatrix();
    q_virtual_euler = DyrosMath::rot2Euler(q_virtual_rotm);
    
    q_virtual_pos   = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(transform_global_to_float),  q_virtual_pos);
    q_virtual_pos   = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(transform_float_to_support), q_virtual_pos);
    q_virtual_euler = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(transform_global_to_float),  q_virtual_euler);
    // q_virtual_euler = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(transform_float_to_support), q_virtual_euler);

    q_virtual_.segment(0, 3)         = q_virtual_pos;
    q_virtual_.segment(3, 3)         = q_virtual_euler;
    q_virtual_.segment(6, MODEL_DOF) = rd_.q_;

    // JOINT VELOCITY
    qdot_virtual_.setZero();  qdot_virtual_  = rd_.q_dot_virtual_;

    // JOINT ACCELERATION
    qddot_virtual_.setZero(); qddot_virtual_ = rd_.q_ddot_virtual_;
}

void AvatarController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}
