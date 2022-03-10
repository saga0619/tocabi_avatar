#include "avatar.h"
#include <fstream>
using namespace TOCABI;

AvatarController::AvatarController(RobotData &rd) : rd_(rd)
{
    nh_avatar_.setCallbackQueue(&queue_avatar_);

    nextswingleg_sub = nh_avatar_.subscribe("/tocabi/dg/nextswinglegcommand", 100, &AvatarController::NextSwinglegCallback, this);

    dg_leg_pd_gain_sub = nh_avatar_.subscribe("/tocabi/dg/legpdgain", 100, &AvatarController::LegJointGainCallback, this);
    step_width_sub = nh_avatar_.subscribe("/tocabi/dg/stepwidthcommand", 100, &AvatarController::StepWidthCommandCallback, this);

    setGains();
    loadNetwork();

    t_u10.tv_nsec = 1000;
    t_u10.tv_sec = 0;
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


    // kd_pelv_ori_ = 100*Eigen::Matrix3d::Identity();		//angular velocity gain (sim: 140)(tune)

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
    kp_stiff_joint_(15) = 200; //left arm
    kp_stiff_joint_(16) = 400;
    kp_stiff_joint_(17) = 200;
    kp_stiff_joint_(18) = 200;
    kp_stiff_joint_(19) = 125;
    kp_stiff_joint_(20) = 125;
    kp_stiff_joint_(21) = 25;
    kp_stiff_joint_(22) = 25;
    kp_stiff_joint_(23) = 50; //head
    kp_stiff_joint_(24) = 50;
    kp_stiff_joint_(25) = 200; //right arm
    kp_stiff_joint_(26) = 400;
    kp_stiff_joint_(27) = 200;
    kp_stiff_joint_(28) = 200;
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
    kv_stiff_joint_(15) = 7; //left arm
    kv_stiff_joint_(16) = 5;
    kv_stiff_joint_(17) = 2.5;
    kv_stiff_joint_(18) = 2.5;
    kv_stiff_joint_(19) = 2.5;
    kv_stiff_joint_(20) = 2;
    kv_stiff_joint_(21) = 2;
    kv_stiff_joint_(22) = 2;
    kv_stiff_joint_(23) = 2; //head
    kv_stiff_joint_(24) = 2;
    kv_stiff_joint_(25) = 7; //right arm
    kv_stiff_joint_(26) = 5;
    kv_stiff_joint_(27) = 2.5;
    kv_stiff_joint_(28) = 2.5;
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
    kv_soft_joint_(15) = 7; //left arm
    kv_soft_joint_(16) = 5;
    kv_soft_joint_(17) = 2.5;
    kv_soft_joint_(18) = 2.5;
    kv_soft_joint_(19) = 2.5;
    kv_soft_joint_(20) = 2;
    kv_soft_joint_(21) = 2;
    kv_soft_joint_(22) = 2;
    kv_soft_joint_(23) = 2; //head
    kv_soft_joint_(24) = 2;
    kv_soft_joint_(25) = 7; //right arm
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
        joint_vel_limit_l_(i) = -M_PI;
        joint_vel_limit_h_(i) = M_PI;
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
    joint_vel_limit_l_(20) = -2 * M_PI;
    joint_vel_limit_h_(20) = 2 * M_PI;
    joint_vel_limit_l_(30) = -2 * M_PI;
    joint_vel_limit_h_(30) = 2 * M_PI;
}

void AvatarController::loadNetwork()
{
    state_.setZero();
    rl_action_.setZero();

    string cur_path = "/home/kim/tocabi_ws/src/tocabi_avatar/weight/";
    std::ifstream file[8];
    file[0].open(cur_path+"mlp_extractor_policy_net_0_weight.txt", std::ios::in);
    file[1].open(cur_path+"mlp_extractor_policy_net_0_bias.txt", std::ios::in);
    file[2].open(cur_path+"mlp_extractor_policy_net_2_weight.txt", std::ios::in);
    file[3].open(cur_path+"mlp_extractor_policy_net_2_bias.txt", std::ios::in);
    file[4].open(cur_path+"action_net_weight.txt", std::ios::in);
    file[5].open(cur_path+"action_net_bias.txt", std::ios::in);
    file[6].open(cur_path+"obs_mean.txt", std::ios::in);
    file[7].open(cur_path+"obs_variance.txt", std::ios::in);

    if(!file[0].is_open())
    {
        std::cout<<"Can not find the weight file"<<std::endl;
    }

    float temp;
    int row = 0;
    int col = 0;

    while(!file[0].eof() && row != policy_net_w0_.rows())
    {
        file[0] >> temp;
        if(temp != '\n')
        {
            policy_net_w0_(row, col) = temp;
            col ++;
            if (col == policy_net_w0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[1].eof() && row != policy_net_b0_.rows())
    {
        file[1] >> temp;
        if(temp != '\n')
        {
            policy_net_b0_(row, col) = temp;
            col ++;
            if (col == policy_net_b0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[2].eof() && row != policy_net_w2_.rows())
    {
        file[2] >> temp;
        if(temp != '\n')
        {
            policy_net_w2_(row, col) = temp;
            col ++;
            if (col == policy_net_w2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[3].eof() && row != policy_net_b2_.rows())
    {
        file[3] >> temp;
        if(temp != '\n')
        {
            policy_net_b2_(row, col) = temp;
            col ++;
            if (col == policy_net_b2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[4].eof() && row != action_net_w_.rows())
    {
        file[4] >> temp;
        if(temp != '\n')
        {
            action_net_w_(row, col) = temp;
            col ++;
            if (col == action_net_w_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[5].eof() && row != action_net_b_.rows())
    {
        file[5] >> temp;
        if(temp != '\n')
        {
            action_net_b_(row, col) = temp;
            col ++;
            if (col == action_net_b_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[6].eof() && row != state_mean_.rows())
    {
        file[6] >> temp;
        if(temp != '\n')
        {
            state_mean_(row, col) = temp;
            col ++;
            if (col == state_mean_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[7].eof() && row != state_var_.rows())
    {
        file[7] >> temp;
        if(temp != '\n')
        {
            state_var_(row, col) = temp;
            col ++;
            if (col == state_var_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
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
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            q_des.setZero();
            parameterSetting();

            WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);

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
        for (int i = 0; i < MODEL_DOF; i++)
        {
            rd_.torque_desired(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
        }

        l_ft_ = rd_.LF_FT;  //Fz: generated by robot,  Fx:   , Fy:    Mx: ,   My: , Mz:
        r_ft_ = rd_.RF_FT;    

        // MJ_graph << l_ft_(2) << "," << r_ft_(2) << endl;

    }
    else if (rd_.tc_.mode == 11)
    {
        ////////////////////////////////////////////////////////////////////////////
        /////////////////// Biped Walking Controller made by MJ ////////////////////
        ////////////////////////////////////////////////////////////////////////////

        if (walking_enable_ == true)
        {
            if (walking_tick_mj == 0)
            {
                parameterSetting();
                initial_flag = 0;

                atb_grav_update_ = false;
                atb_upper_update_ = false;
                torque_upper_fast_.setZero();
                torque_upper_fast_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);
                torque_upper_.setZero();
                torque_upper_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);

                // cout << "parameter setting OK" << endl;
                // cout << "mode = 11" << endl;
            }
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            updateInitialState();   
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            getRobotState();        
            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
            floatToSupportFootstep();
            std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

            if (current_step_num_ < total_step_num_)
            {
                getZmpTrajectory(); 
                std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
                getComTrajectory(); 
                std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
                getFootTrajectory();    
                std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
                getPelvTrajectory();    
                std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
                supportToFloatPattern();  
                reflectRLAction(false, true);
                std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();
                computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des);   //390
                std::chrono::steady_clock::time_point t10 = std::chrono::steady_clock::now();
                Compliant_control(q_des);
                for (int i = 0; i < 12; i++)
                {
                    //ref_q_(i) = q_des(i);
                    ref_q_(i) = DOB_IK_output_(i);
                }
                // hip_compensator();
                std::chrono::steady_clock::time_point t11 = std::chrono::steady_clock::now();

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    Gravity_MJ_fast_ = Gravity_MJ_;
                    atb_grav_update_ = false;
                }
                std::chrono::steady_clock::time_point t12 = std::chrono::steady_clock::now();
                // if (walking_tick_mj < 1.0 * hz_)
                // {
                //     for (int i = 0; i < 12; i++)
                //     {
                //         ref_q_(i) = DyrosMath::cubic(walking_tick_mj, 0, 1.0 * hz_, Initial_ref_q_(i), q_des(i), 0.0, 0.0);
                //     }
                // }
                
                CP_compen_MJ();
                CP_compen_MJ_FT();
                std::chrono::steady_clock::time_point t13 = std::chrono::steady_clock::now();
                torque_lower_.setZero();
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + Tau_CP(i) + 1.0 * Gravity_MJ_fast_(i);
                    // 4 (Ankle_pitch_L), 5 (Ankle_roll_L), 10 (Ankle_pitch_R),11 (Ankle_roll_R)
                }

                desired_q_not_compensated_ = ref_q_;
  
                updateNextStepTime();
                reflectRLState();

                q_prev_MJ_ = rd_.q_;
                std::chrono::steady_clock::time_point t14 = std::chrono::steady_clock::now();
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
        if (rd_.tc_init == true)
        {
            initWalkingParameter();
            rd_.tc_init = false;
        }
        if (atb_upper_update_ == false)
        {
            atb_upper_update_ = true;
            desired_q_fast_ = init_q_;
            desired_q_dot_fast_.setZero();
            atb_upper_update_ = false;
        }
        
        torque_upper_.setZero();
        for (int i = 12; i < MODEL_DOF; i++)
        {
            torque_upper_(i) = (kp_joint_(i) * (ref_q_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - current_q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            // torque_upper_(i) = 1.0 * Gravity_MJ_fast_(i);
        }

        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = torque_lower_ + torque_upper_;
        /////////////////////////////////////////////////////////////////////////////// 
    }
}

void AvatarController::computeFast()
{
    if (rd_.tc_.mode == 10)
    {
        if (initial_flag == 1)
        {
            WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);
            VectorQd Gravity_MJ_local= WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);
            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }
        }
    }
    else if (rd_.tc_.mode == 11)
    {
            std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
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
            WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);
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
                VectorQd Gravity_MJ_local= WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, support_foot);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }
        }

        // RL Donghyeon
        change_vel++;
		if (change_vel % 10000 == 0)
		{
            change_vel = 0;
            std::mt19937 gen(random_dev());
            std::uniform_real_distribution<double> dis(0.1, 0.6);
            target_data_body_vel_ = dis(gen); 
            std::cout << "Target Velocity: " << target_data_body_vel_ << std::endl;
		}

		if (round((current_time_)*1000)/1000 >= policy_eval_dt)
		{
			processObservation();
			feedforwardPolicy();
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
        savePreData();
            std::cout << "dt: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t5).count() << std::endl;
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
    walking_speed_ = 0.00;

    foot_contact_ = -1;
    foot_contact_pre_ = foot_contact_;
    step_width_ = 0.22; //for preview control

    start_walking_trigger_ = false;
    first_step_trigger_ = false;
    foot_swing_trigger_ = false;
    stop_walking_trigger_ = true;
    falling_detection_flag_ = false;


    preview_horizon_ = 1.6; //seconds
    preview_hz_ = 2000;
    zmp_size_ = preview_horizon_ * preview_hz_;
    ref_zmp_.setZero(zmp_size_, 2);

    walking_duration_start_delay_ = preview_horizon_;
    max_stop_walking_num_ = int(preview_horizon_ / walking_duration_cmd_) + 1;
    stop_walking_counter_ = 0;

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
    
    motion_q_pre_ = init_q_;
    motion_q_dot_pre_.setZero();

    zmp_local_lfoot_.setZero();
    zmp_local_rfoot_.setZero();
    zmp_measured_.setZero();
    zmp_dot_measured_.setZero();

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

    lfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Foot].xpos - pelv_pos_current_);
    lfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].rotm;
    rfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Foot].xpos - pelv_pos_current_);
    rfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].rotm;

    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

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
}

void AvatarController::getProcessedRobotData()
{
    if (foot_contact_ == 1) // left support foot
    {
        support_foot_transform_current_ = lfoot_transform_current_from_global_;
    }
    else if (foot_contact_ == -1) //right support foot
    {
        support_foot_transform_current_ = rfoot_transform_current_from_global_;
    }

    //////////////////////////////Variables in Support Foot Frame////////////////////////
    ///////Support Foot Frame's origin is attatched to the Support Foot Frame origin////////////////////////////////
    ///////z axis is aligned with gravity force and upward//////////////////////////////////////////
    ////// x axis is poining out from center of foot to the toe direction//////////////
    Isometry3d support_foot_transform_yaw_align = support_foot_transform_current_;

    com_pos_current_from_support_ = DyrosMath::multiplyIsometry3dVector3d(support_foot_transform_yaw_align.inverse(), com_pos_current_);
    com_vel_current_from_support_ = support_foot_transform_yaw_align.linear().transpose() * com_vel_current_;
    com_acc_current_from_support_ = support_foot_transform_yaw_align.linear().transpose() * com_acc_current_;

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

        // init_q_ = current_q_;
        last_desired_q_ = desired_q_;

        com_pos_desired_last_ = com_pos_desired_;
        com_vel_desired_last_ = com_vel_desired_;
        com_acc_desired_last_ = com_acc_desired_;
    }

    if (current_time_ == program_start_time_)
    {
        com_pos_desired_preview_pre_ = com_pos_current_from_support_;
        com_vel_desired_preview_pre_.setZero();
        com_acc_desired_preview_pre_.setZero();

        //preview gain update
        previewParam_MJ(1 / preview_hz_, zmp_size_, zc_, K_act_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);


        last_preview_param_update_time_ = current_time_;
        preview_update_time_ = current_time_;

        for (int i = 0; i < zmp_size_; i++)
        {
            ref_zmp_(i, 0) = com_pos_init_from_support_(0);
            ref_zmp_(i, 1) = com_pos_init_from_support_(1);
        }
    }

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

    torque_task_pre_ = torque_task_;
    torque_grav_pre_ = torque_grav_;

    foot_contact_pre_ = foot_contact_;

    A_mat_pre_ = A_mat_;

    zmp_desired_pre_ = zmp_desired_from_global_;
    zmp_local_lfoot_pre_ = zmp_local_lfoot_;
    zmp_local_rfoot_pre_ = zmp_local_rfoot_;


    com_pos_desired_preview_pre_ = com_pos_desired_preview_;
    com_vel_desired_preview_pre_ = com_vel_desired_preview_;
    com_acc_desired_preview_pre_ = com_acc_desired_preview_;


}

void AvatarController::NextSwinglegCallback(const std_msgs::Float32 &msg)
{
    foot_contact_ = msg.data;
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


void AvatarController::StepWidthCommandCallback(const std_msgs::Float32 &msg)
{
    step_width_ = msg.data;
}


void AvatarController::previewParam_MJ(double dt, int NL, double zc, Eigen::Matrix4d &K, Eigen::MatrixXd &Gi, Eigen::VectorXd &Gd, Eigen::MatrixXd &Gx,
                                       Eigen::MatrixXd &A, Eigen::VectorXd &B, Eigen::MatrixXd &C, Eigen::MatrixXd &D, Eigen::MatrixXd &A_bar, Eigen::VectorXd &B_bar)
{
    A.resize(3, 3);
    A(0, 0) = 1.0;
    A(0, 1) = dt;
    A(0, 2) = dt * dt / 2;
    A(1, 0) = 0;
    A(1, 1) = 1.0;
    A(1, 2) = dt;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    B.resize(3, 1);
    B(0, 0) = dt * dt * dt / 6;
    B(1, 0) = dt * dt / 2;
    B(2, 0) = dt;

    C.resize(1, 3);
    C(0, 0) = 1;
    C(0, 1) = 0;
    // C(0,2) = -zc/GRAVITY;
    C(0, 2) = -0.71 / 9.81; //mj gain

    B_bar.resize(4);
    B_bar.segment(0, 1) = C * B;
    B_bar.segment(1, 3) = B;

    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();

    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.setZero(4, 4);
    I_bar.setZero(4, 1);
    F_bar.setZero(4, 3);

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
    R(0, 0) = 1e-6;

    Eigen::MatrixXd Qx;
    Qx.setZero(3, 3);

    Eigen::MatrixXd Q_bar;
    Q_bar.setZero(4, 4);
    Q_bar(0, 0) = Qe(0, 0);

    // mj gain
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
    Temp_mat.setZero(1, 1);
    Temp_mat_inv.setZero(1, 1);
    Ac_bar.setZero(4, 4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();

    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;

    Eigen::MatrixXd Ac_bar_tran(4, 4);
    Ac_bar_tran = Ac_bar.transpose();

    Gi.resize(1, 1);
    Gx.resize(1, 3);
    // Gi = Temp_mat_inv * B_bar_tran * K * I_bar ;
    // Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;
    //mj gain
    Gi(0, 0) = 872.3477;
    Gx(0, 0) = 945252.1760702;
    Gx(0, 1) = 256298.6905049;
    Gx(0, 2) = 542.0544196;

    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.setZero(4, NL);
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

void AvatarController::preview_MJ(double dt, int NL, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double &UX, double &UY,
                                  Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{

    Eigen::VectorXd px_ref, py_ref;
    px_ref.resize(zmp_size_);
    py_ref.resize(zmp_size_);

    for (int i = 0; i < zmp_size_; i++)
    {
        px_ref(i) = ref_zmp_(i, 0);
        py_ref(i) = ref_zmp_(i, 1);
    }

    // Eigen::Matrix1x3d C;
    C(0, 0) = 1;
    C(0, 1) = 0;
    // C(0,2) = -zc_/GRAVITY;
    C(0, 2) = -0.71 / 9.81; //mj gain

    Eigen::VectorXd px, py;
    px.resize(1);
    py.resize(1);

    if (current_time_ == program_start_time_)
    {
        preview_x_b.setZero();
        preview_y_b.setZero();
        preview_x.setZero();
        preview_y.setZero();

        preview_x_b(0) = x_i; // 이때 before 값이 없으면 첫 tick에서 deltaC x의 에러가 갑작스럽게 생겨서 발산
        preview_y_b(0) = y_i;
        preview_x(0) = x_i;
        preview_y(0) = y_i;
    }
    else
    {
        // preview_x(0) = com_pos_pre_desired_from_support_(0);
        // preview_y(0) = com_pos_pre_desired_from_support_(1);
        // preview_x(1) = com_vel_pre_desired_from_support_(0);
        // preview_y(1) = com_vel_pre_desired_from_support_(1);
        // preview_x(2) = com_acc_pre_desired_from_support_(0);
        // preview_y(2) = com_acc_pre_desired_from_support_(1);
        preview_x = xs;
        preview_y = ys;

        preview_x_b(0) = com_pos_pre_desired_from_support_(0) - com_vel_pre_desired_from_support_(0) / preview_hz_;
        preview_y_b(0) = com_pos_pre_desired_from_support_(1) - com_vel_pre_desired_from_support_(1) / preview_hz_;
        preview_x_b(1) = com_vel_pre_desired_from_support_(0) - com_acc_pre_desired_from_support_(0) / preview_hz_;
        preview_y_b(1) = com_vel_pre_desired_from_support_(1) - com_acc_pre_desired_from_support_(1) / preview_hz_;
        preview_x_b(2) = com_acc_pre_desired_from_support_(0) - com_jerk_pre_desired_from_support_(0) / preview_hz_;
        preview_y_b(2) = com_acc_pre_desired_from_support_(1) - com_jerk_pre_desired_from_support_(1) / preview_hz_;

        // preview_x_b(0) = preview_x(0) - preview_x(1)/preview_hz_;
        // preview_y_b(0) = preview_y(0) - preview_y(1)/preview_hz_;
        // preview_x_b(1) = preview_x(1) - preview_x(2)/preview_hz_;
        // preview_y_b(1) = preview_y(1) - preview_y(2)/preview_hz_;
        // preview_x_b(2) = preview_x(2) - UX/preview_hz_;
        // preview_y_b(2) = preview_y(2) - UY/preview_hz_;

        // preview_x_b = preview_x;
        // preview_y_b = preview_y;

        // preview_x_b(0) = preview_x(0) - preview_x(1)/preview_hz_;
        // preview_y_b(0) = preview_y(0) - preview_y(1)/preview_hz_;
    }

    px = C * preview_x;
    py = C * preview_y;
    // px(0) = zmp_measured_(0);
    // py(0) = zmp_measured_(1);
    zmp_current_by_com_from_support_(0) = px(0);
    zmp_current_by_com_from_support_(1) = py(0);

    double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

    for (int i = 0; i < NL - 1; i++) // Preview Step 개수만큼 더함.
    {
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (px_ref(i + 1) - px_ref(i));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (py_ref(i + 1) - py_ref(i));
    }
    Eigen::MatrixXd del_ux(1, 1);
    Eigen::MatrixXd del_uy(1, 1);
    del_ux.setZero();
    del_uy.setZero();

    Eigen::VectorXd GX_X(1);
    GX_X = Gx * (preview_x - preview_x_b);
    Eigen::VectorXd GX_Y(1);
    GX_Y = Gx * (preview_y - preview_y_b);

    del_ux(0, 0) = -(px(0) - px_ref(0)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - py_ref(0)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x + B * UX;
    YD = A * preview_y + B * UY;
}


void AvatarController::updateInitialState()
{
    if (walking_tick_mj == 0)
    {
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
        // cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;

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

    //cout << "L : " << lfoot_float_current_.linear() << "\n" <<  "R : " << rfoot_float_current_.linear() << endl;
    com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);

    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT; 
    r_ft_ = rd_.RF_FT;

     if(walking_tick_mj == 0)
    { l_ft_LPF = l_ft_; r_ft_LPF = r_ft_; }
    
    l_ft_LPF = 1/(1+2*M_PI*6.0*del_t)*l_ft_LPF + (2*M_PI*6.0*del_t)/(1+2*M_PI*6.0*del_t)*l_ft_;
    r_ft_LPF = 1/(1+2*M_PI*6.0*del_t)*r_ft_LPF + (2*M_PI*6.0*del_t)/(1+2*M_PI*6.0*del_t)*r_ft_; 



    //MJ_graph << l_ft_LPF(2) << "," << l_ft_LPF(3) << "," << l_ft_LPF(4) << "," << r_ft_LPF(2) << "," << r_ft_LPF(3) << "," << r_ft_LPF(4) << endl;
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
        middle_total_step_number = 4; //total foot step number
        dlength = 0;
    }

    unsigned int number_of_foot_step;

    int del_size;

    del_size = 1;
    // number_of_foot_step = 2 + initial_total_step_number * del_size + middle_total_step_number * del_size + final_total_step_number * del_size;
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

    int temp0;
    temp0 = -is_right;

    double initial_dir = 0.0;

    /*
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
    */

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
    //simulation gains
    Kp(0) = 1800.0; Kd(0) = 70.0; // Left Hip yaw
    Kp(1) = 2100.0; Kd(1) = 90.0;// Left Hip roll
    Kp(2) = 2100.0; Kd(2) = 90.0;// Left Hip pitch
    Kp(3) = 2100.0; Kd(3) = 90.0;// Left Knee pitch
    Kp(4) = 900.0; Kd(4) = 40.0;// Left Ankle pitch
    Kp(5) = 900.0; Kd(5) = 40.0;// Left Ankle roll

    Kp(6) = 1800.0; Kd(6) = 70.0;// Right Hip yaw
    Kp(7) = 2100.0; Kd(7) = 90.0;// Right Hip roll
    Kp(8) = 2100.0; Kd(8) = 90.0;// Right Hip pitch
    Kp(9) = 2100.0; Kd(9) = 90.0;// Right Knee pitch
    Kp(10) = 900.0; Kd(10) = 40.0;// Right Ankle pitch
    Kp(11) = 900.0; Kd(11) = 40.0;// Right Ankle roll

    Kp(12) = 2200.0; Kd(12) = 90.0;// Waist yaw
    Kp(13) = 2200.0; Kd(13) = 90.0;// Waist pitch
    Kp(14) = 2200.0; Kd(14) = 90.0;// Waist roll

    Kp(15) = 400.0; Kd(15) = 10.0;
    Kp(16) = 800.0; Kd(16) = 10.0;
    Kp(17) = 400.0; Kd(17) = 10.0;
    Kp(18) = 400.0; Kd(18) = 10.0;
    Kp(19) = 250.0; Kd(19) = 2.5;
    Kp(20) = 250.0; Kd(20) = 2.0;
    Kp(21) = 50.0; Kd(21) = 2.0; // Left Wrist
    Kp(22) = 50.0; Kd(22) = 2.0; // Left Wrist

    Kp(23) = 50.0; Kd(23) = 2.0; // Neck
    Kp(24) = 50.0; Kd(24) = 2.0; // Neck

    Kp(25) = 400.0; Kd(25) = 10.0;
    Kp(26) = 800.0; Kd(26) = 10.0;
    Kp(27) = 400.0; Kd(27) = 10.0;
    Kp(28) = 400.0; Kd(28) = 10.0;
    Kp(29) = 250.0; Kd(29) = 2.5;
    Kp(30) = 250.0; Kd(30) = 2.0;
    Kp(31) = 50.0; Kd(31) = 2.0; // Right Wrist
    Kp(32) = 50.0; Kd(32) = 2.0; // Right Wrist

    // Kp(0) = 2000.0;
    // Kd(0) = 20.0; // Left Hip yaw
    // Kp(1) = 5000.0;
    // Kd(1) = 55.0; // Left Hip roll //55
    // Kp(2) = 4000.0;
    // Kd(2) = 45.0; // Left Hip pitch
    // Kp(3) = 3700.0;
    // Kd(3) = 40.0; // Left Knee pitch
    // Kp(4) = 4000.0; // 5000
    // Kd(4) = 65.0; // Left Ankle pitch /5000 / 30  //55
    // Kp(5) = 4000.0; // 5000
    // Kd(5) = 65.0; // Left Ankle roll /5000 / 30 //55

    // Kp(6) = 2000.0;
    // Kd(6) = 20.0; // Right Hip yaw
    // Kp(7) = 5000.0;
    // Kd(7) = 55.0; // Right Hip roll  //55
    // Kp(8) = 4000.0;
    // Kd(8) = 45.0; // Right Hip pitch
    // Kp(9) = 3700.0;
    // Kd(9) = 40.0; // Right Knee pitch
    // Kp(10) = 4000.0; // 5000
    // Kd(10) = 65.0; // Right Ankle pitch //55
    // Kp(11) = 4000.0; // 5000
    // Kd(11) = 65.0; // Right Ankle roll //55

    // Kp(12) = 6000.0;
    // Kd(12) = 200.0; // Waist yaw
    // Kp(13) = 10000.0;
    // Kd(13) = 100.0; // Waist pitch
    // Kp(14) = 10000.0;
    // Kd(14) = 100.0; // Waist roll

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

    if (foot_step_(0, 6) == 0) //right support foot
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
        //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
    }
    else
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
        //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
    }

    for (int i = 0; i < total_step_num_; i++)
    {
        if (foot_step_(i, 6) == 0) //right support, left swing
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
        for (int i = 0; i <= t_temp_; i++) //600 tick
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
            if (i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
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

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);        
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        double t_rest_temp = 0.00 * hz_;

        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();
            
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

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
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);
 
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
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        }
    }
    else
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_euler_support_.setZero();
            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);
     
            for (int i = 0; i < 3; i++)
            {
                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }
            
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);
            //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_R_y_input)*DyrosMath::rotateWithX(-F_T_R_x_input);

            //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

            for (int i = 0; i < 3; i++)
            {
                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(F_T_L_y_input)*DyrosMath::rotateWithX(-F_T_L_x_input);

            //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
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
        // cout << "del_zmp : " << del_zmp(0) << "," << del_zmp(1) << endl;
    }

    del_ux(0, 0) = -(px(0) - px_ref(tick)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - py_ref(tick)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x_mj + B * UX;
    YD = A * preview_y_mj + B * UY;
    //SC_err_compen(XD(0), YD(0));
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
    //MJ_graph << px_ref(tick) << "," << py_ref(tick) << "," << XD(0) << "," << YD(0) << endl;
    //MJ_graph << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << endl;
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

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - 0.15 * damping_x - com_support_current_(0)); //- 0.01 * zmp_err_(0) * 0;
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7 * (com_desired_(1) - 0.6 * damping_y - com_support_current_(1));  //- 0.01 * zmp_err_(1) * 0;
    // pelv_trajectory_support_.translation()(2) = com_desired_(2) + pelv_height_offset_; //DG
    pelv_trajectory_support_.translation()(2) = com_desired_(2) - 0*pelv_height_offset_;

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
    R_angle_input_dot = 2.0 * (0.0 - R_angle) - 0.01*R_angle_input;

    P_angle_input = P_angle_input + P_angle_input_dot * del_t;
    R_angle_input = R_angle_input + R_angle_input_dot*del_t;

    if(R_angle_input > 3 * DEG2RAD) //1.5 degree
    { R_angle_input = 3 * DEG2RAD; }
    else if(R_angle_input < -3 * DEG2RAD)
    { R_angle_input = -3 * DEG2RAD; }

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

    // MJ_graph << P_angle * 180 / 3.141592 << "," << Trunk_trajectory_euler(1) << endl;

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
    
    //SC_err_compen(com_desired_(0), com_desired_(1));

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
                                                                 // Ankle pitch
  
    q_des(4) = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2))) - L_alpha;
    q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2))) - R_alpha;

    q_des(5) = atan2(L_r(1), L_r(2));  
    q_des(11) = atan2(R_r(1), R_r(2)); 

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
    q_des(5) = q_des(5);                                                                                  // Ankle roll

    q_des(6) = atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    q_des(7) = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_des(6)) + R_Hip_rot_mat(1, 1) * cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = q_des(11);

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
        WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);
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

        WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);
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
            WBC::SetContact(rd_, 1, 0, 0, 0, is_inverse_fine_);
            Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_SSP_(1) = 1.0 * Gravity_SSP_(1);
            Gravity_SSP_(5) = 1.0 * Gravity_SSP_(5);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            WBC::SetContact(rd_, 0, 1, 0, 0, is_inverse_fine_);
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
            WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }
    else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
    {
        contact_gain = 1.0;

        WBC::SetContact(rd_, 1, 1, 0, 0, is_inverse_fine_);
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
    target_x_ = rd_.tc_.x;
    target_y_ = rd_.tc_.y;
    target_z_ = rd_.tc_.z;
    com_height_ = rd_.tc_.walking_height;
    target_theta_ = rd_.tc_.theta;
    step_length_x_ = rd_.tc_.step_length_x;
    step_length_y_ = rd_.tc_.step_length_y;
    is_right_foot_swing_ = 1;

    // RL
    int rand_swing = std::rand();
    if (rand_swing % 2 == 0)
    {
        is_right_foot_swing_ = 0;
    }
    else
    {
        is_right_foot_swing_ = 1;
    }
    
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

    t_rest_init_ = 0.2 * hz_; // slack 400
    t_rest_last_ = 0.2 * hz_; // 400
    t_double1_ = 0.03 * hz_; // 60
    t_double2_ = 0.03 * hz_; // 60
    t_total_ = 1.1 * hz_; // 총 시간 
    // t_rest_init_ (중앙에 정지) // t_double1_ (중앙 -> 지지발) // t_total_- 4개 time 합 동안 (0.64) single support // t_double2_ (지지발 -> 중앙) // t_rest_last_ (중앙에 정지)

    t_temp_ = 0.0 * hz_;
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
            is_new_step_updated_ = true;
        }
    }
    else
    {
        is_new_step_updated_ = false;
    }

    if (current_step_num_ == total_step_num_ - 1 && walking_tick_mj >= t_last_ + t_total_)
    {
        
    }
    else
    {
        walking_tick_mj++;
        reflectRLAction(true, false);
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

    //X_x_ssp(0) = com_float_current_(0);
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

    F_R = (1 - alpha) * rd_.link_[COM_id].mass * GRAVITY;
    F_L = alpha * rd_.link_[COM_id].mass * GRAVITY;

    Tau_CP(4) = F_L * del_zmp(0);  // L pitch
    Tau_CP(10) = F_R * del_zmp(0); // R pitch

    Tau_CP(5) = -F_L * del_zmp(1);  // L roll
    Tau_CP(11) = -F_R * del_zmp(1); // R roll
}

void AvatarController::CP_compen_MJ_FT()
{ // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
  double alpha = 0;
  double F_R = 0, F_L = 0;
  double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0 ;
  double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0 ;
  double zmp_offset = 0, ZMP_Y_REF_alpha = 0;
  double alpha_new = 0;

//   zmp_offset = 0.025; // 0.9초
  zmp_offset = 0.02; // 1.1초 
  // Preview를 이용한 COM 생성시 ZMP offset을 2cm 안쪽으로 넣었지만, alpha 계산은 2cm 넣으면 안되기 때문에 조정해주는 코드
  // 어떻게 보면 COM, CP 궤적은 ZMP offset이 반영되었고, CP 제어기는 반영안시킨게 안맞는거 같기도함
  if(walking_tick_mj > t_temp_)
  {
    if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
      if(foot_step_(current_step_num_,6) == 1) 
      {
        ZMP_Y_REF_alpha = ZMP_Y_REF + zmp_offset*(walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_)/(t_rest_init_ + t_double1_);
      }  
      else
      {
        ZMP_Y_REF_alpha = ZMP_Y_REF - zmp_offset*(walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_)/(t_rest_init_ + t_double1_);
      }
    }
    else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
      if(foot_step_(current_step_num_,6) == 1) 
      {
        ZMP_Y_REF_alpha = ZMP_Y_REF + zmp_offset ;
      }
      else
      {
        ZMP_Y_REF_alpha = ZMP_Y_REF - zmp_offset ;
      } 
    }
    else if(walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
    {
      if(foot_step_(current_step_num_,6) == 1) 
      {
        ZMP_Y_REF_alpha = ZMP_Y_REF + zmp_offset - zmp_offset*(walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_))/(t_rest_last_ + t_double2_)  ;
      }  
      else
      {
        ZMP_Y_REF_alpha = ZMP_Y_REF - zmp_offset + zmp_offset*(walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_))/(t_rest_last_ + t_double2_);
      }   
    }
    else
    {
      ZMP_Y_REF_alpha = ZMP_Y_REF ;
    }
  }    
  else
  {
    ZMP_Y_REF_alpha = ZMP_Y_REF ;
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
  alpha = (ZMP_Y_REF_alpha + 0*del_zmp(1) - rfoot_support_current_.translation()(1))/(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
  //cout << alpha << "," << ZMP_Y_REF << "," << rfoot_support_current_.translation()(1) << "," << lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1) << endl;
  // 로봇에서 구현할때 alpha가 0~1로 나오는지 확인, ZMP offset 0으로 해야됨. 
  if(alpha > 1)
  { alpha = 1; } // 왼발 지지때 alpha = 1
  else if(alpha < 0)
  { alpha = 0; }
//   if(alpha_new > 1)
//   { alpha_new = 1; } // 왼발 지지때 alpha = 1
//   else if(alpha_new < 0)
//   { alpha_new = 0; }

  double real_robot_mass_offset_ =  42; //42

  F_R = -(1 - alpha) * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ + 15);
  F_L = -alpha * (rd_.link_[COM_id].mass * GRAVITY + real_robot_mass_offset_ - 15); // alpha가 0~1이 아니면 desired force가 로봇 무게보다 계속 작게나와서 지면 반발력을 줄이기위해 다리길이를 줄임. 

  if(walking_tick_mj == 0)
  { F_F_input = 0.0; F_T_L_x_input = 0.0; F_T_R_x_input = 0.0; F_T_L_y_input = 0.0; F_T_R_y_input = 0.0; }
  
  //////////// Force
  F_F_input_dot = 0.0001*((l_ft_LPF(2) - r_ft_LPF(2)) - (F_L - F_R)) - 3.0*F_F_input; // 0.9초 0.0001/ 3.0

  F_F_input = F_F_input + F_F_input_dot*del_t;

  if(F_F_input >= 0.02) // 1.1초 0.02
  {
    F_F_input = 0.02;
  }
  else if(F_F_input <= -0.02)
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
  //MJ_graph << ZMP_Y_REF << "," << ZMP_Y_REF_alpha << "," << alpha << "," << F_L << "," << F_R << "," << F_F_input << "," << l_ft_LPF(2) << "," << r_ft_LPF(2) <<  endl;
  
  //////////// Torque
  // X,Y 축을 X,Y 방향으로 헷갈렸었고, 위치 명령을 발목 IK각도에 바로 넣었었음.
  Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_alpha + del_zmp(1))) * F_L) ;
  Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_L) ;
  
  if(Tau_all_x > 100)
  { Tau_all_x = 100; }
  else if(Tau_all_x < -100)
  { Tau_all_x = -100; }

  if(Tau_all_y > 100)
  { Tau_all_y = 100; }
  else if(Tau_all_y < -100)
  { Tau_all_y = -100; }
  
  Tau_R_x = (1 - alpha) * Tau_all_x;
  Tau_L_x = (alpha) * Tau_all_x;

  Tau_L_y = -alpha * Tau_all_y ;
  Tau_R_y = -(1-alpha) * Tau_all_y ;   

  double Kr_roll = 0.0, Kl_roll = 0.0;
  double Kr_pitch = 0.0, Kl_pitch = 0.0;

  if(walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
  {
    Kr_roll = 20.0;
    Kl_roll = 20.0;
    Kr_pitch = 20.0;
    Kl_pitch = 20.0;    
  }
  else if(walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
  {
    if(alpha == 1) // 왼발 지지
    {
      Kl_roll = 20.0;
      Kr_roll = 50.0;
      Kl_pitch = 20.0;
      Kr_pitch = 50.0;
    }    
    if(alpha == 0) // 오른발 지지
    {
      Kl_roll = 50.0;
      Kr_roll = 20.0;
      Kl_pitch = 50.0;
      Kr_pitch = 20.0;
    }
  }
  else
  {
    Kr_roll = 20.0;
    Kl_roll = 20.0;
    Kr_pitch = 20.0;
    Kl_pitch = 20.0;
  }
  
  //Roll 방향 (-0.02/-30 0.9초)
  F_T_L_x_input_dot = -0.02*(Tau_L_x - l_ft_LPF(3)) - Kl_roll*F_T_L_x_input;
//   F_T_L_x_input_dot = -0.02*(Tau_L_x - l_ft_LPF(3)) - 30.0*F_T_L_x_input; 
  F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot*del_t;
//   F_T_L_x_input = 0;   
  F_T_R_x_input_dot = -0.02*(Tau_R_x - r_ft_LPF(3)) - Kr_roll*F_T_R_x_input;
//   F_T_R_x_input_dot = -0.02*(Tau_R_x - r_ft_LPF(3)) - 30.0*F_T_R_x_input; 
  F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot*del_t;
//   F_T_R_x_input = 0;
  
  //Pitch 방향  (0.005/-30 0.9초)
  F_T_L_y_input_dot = 0.01*(Tau_L_y - l_ft_LPF(4)) - Kl_pitch*F_T_L_y_input;
//   F_T_L_y_input_dot = 0.005*(Tau_L_y - l_ft_LPF(4)) - 30.0*F_T_L_y_input; 
  F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot*del_t; 
//   F_T_L_y_input = 0;
  F_T_R_y_input_dot = 0.01*(Tau_R_y - r_ft_LPF(4)) - Kr_pitch*F_T_R_y_input;
//   F_T_R_y_input_dot = 0.005*(Tau_R_y - r_ft_LPF(4)) - 30.0*F_T_R_y_input; 
  F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot*del_t;
//   F_T_R_y_input = 0; 
  //MJ_graph << l_ft_LPF(2) - r_ft_LPF(2) << "," <<  (F_L - F_R) << "," << F_F_input << endl; 
  //MJ_graph << F_T_L_x_input << "," << F_T_R_x_input << "," <<  F_T_L_y_input << "," <<  F_T_R_y_input << "," << F_F_input << "," << cp_measured_(1) << "," << cp_desired_(1) << endl;
  if(F_T_L_x_input >= 0.1) // 5 deg limit
  { F_T_L_x_input = 0.1; }
  else if(F_T_L_x_input < -0.1)
  { F_T_L_x_input = -0.1; }
  
  if(F_T_R_x_input >= 0.1) // 5 deg limit
  { F_T_R_x_input = 0.1; }
  else if(F_T_R_x_input < -0.1)
  { F_T_R_x_input = -0.1; }
  
  if(F_T_L_y_input >= 0.1) // 5 deg limit
  { F_T_L_y_input = 0.1; }
  else if(F_T_L_y_input < -0.1)
  { F_T_L_y_input = -0.1; }
  
  if(F_T_R_y_input >= 0.1) // 5 deg limit
  { F_T_R_y_input = 0.1; }
  else if(F_T_R_y_input < -0.1)
  { F_T_R_y_input = -0.1; }

  // MJ_joint1 << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << "," << ZMP_Y_REF << "," << ZMP_Y_REF_alpha << "," << l_ft_LPF(2) << "," << r_ft_LPF(2) << endl;
  //cout << F_T_R_x_input*180/3.141592 << "," << F_T_L_x_input*180/3.141592 << "," << Tau_R_x << "," << Tau_L_x << "," << r_ft_(3) << "," << l_ft_(3) << endl;
  //MJ_graph << alpha << "," << alpha_new << endl;
  //MJ_graph << rfoot_support_current_.translation()(1) << "," << lfoot_support_current_.translation()(1) << "," << ZMP_Y_REF << "," << Tau_R_y << "," << Tau_L_y << endl;
  //MJ_graph << Tau_L_x << "," << Tau_R_x << "," << l_ft_LPF(3) << "," << r_ft_LPF(3) << "," << Tau_L_y << "," << Tau_R_y << "," << l_ft_LPF(4) << "," << r_ft_LPF(4) << "," << F_F_input << endl;
  //MJ_graph << ZMP_Y_REF << "," << alpha << "," << ZMP_Y_REF_alpha << endl;
  //MJ_graph << Tau_all_y << "," << Tau_L_y << "," << Tau_R_y << "," << l_ft_(4) << "," << r_ft_(4) << "," << cp_measured_(0) << "," << cp_desired_(0) << endl;
}
void AvatarController::computePlanner()
{
}

void AvatarController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}


void AvatarController::processObservation()
{
    int data_idx = 0;

    state_(data_idx) = rd_.q_virtual_(MODEL_DOF_VIRTUAL);
    data_idx++;	
	state_(data_idx) = rd_.q_virtual_(3);
    data_idx++;	
	state_(data_idx) = rd_.q_virtual_(4);
    data_idx++;	
	state_(data_idx) = rd_.q_virtual_(5);
    data_idx++;	

    for (int i = 0; i <MODEL_DOF; i++)
    {
        state_(data_idx) = rd_.q_(i);
        data_idx++;
    }

    state_(data_idx) = rd_.q_dot_virtual_(0);
    data_idx++;	
	state_(data_idx) = rd_.q_dot_virtual_(1);
    data_idx++;	
	state_(data_idx) = rd_.q_dot_virtual_(2);
    data_idx++;	
    state_(data_idx) = rd_.q_dot_virtual_(3);
    data_idx++;	
	state_(data_idx) = rd_.q_dot_virtual_(4);
    data_idx++;	
	state_(data_idx) = rd_.q_dot_virtual_(5);
    data_idx++;	
    
    for (int i = 0; i <MODEL_DOF; i++)
    {
        state_(data_idx) = rd_.q_dot_(i);
        data_idx++;
    }

    state_(data_idx) = rd_.q_virtual_(2);
    data_idx++;

    state_(data_idx) = target_data_body_vel_;
    data_idx++;

    Eigen::Vector3d left_Euler_lipm = DyrosMath::rot2Euler(lfoot_trajectory_float_origin_.linear());
    Eigen::Vector3d right_Euler_lipm = DyrosMath::rot2Euler(rfoot_trajectory_float_origin_.linear());
    Eigen::Vector3d left_Euler_rl = DyrosMath::rot2Euler(lfoot_trajectory_float_.linear());
    Eigen::Vector3d right_Euler_rl = DyrosMath::rot2Euler(rfoot_trajectory_float_.linear());
    
    for (int i = 0; i <3; i++)
    {
        state_(data_idx) = lfoot_trajectory_float_origin_.translation()(i);
        data_idx++;
    }
    for (int i = 0; i <3; i++)
    {
        state_(data_idx) = left_Euler_lipm(i);
        data_idx++;
    }        
    for (int i = 0; i <3; i++)
    {
        state_(data_idx) = rfoot_trajectory_float_origin_.translation()(i);
        data_idx++;
    }
    for (int i = 0; i <3; i++)
    {
        state_(data_idx) = right_Euler_lipm(i);
        data_idx++;
    }
    for (int i = 0; i <3; i++)
    {
        state_(data_idx) = lfoot_trajectory_float_.translation()(i);
        data_idx++;
    }
    for (int i = 0; i <3; i++)
    {
        state_(data_idx) = left_Euler_rl(i);
        data_idx++;
    }    
    
    for (int i = 0; i <3; i++)
    {
        state_(data_idx) = rfoot_trajectory_float_.translation()(i);
        data_idx++;
    }
    for (int i = 0; i <3;  i++)
    {
        state_(data_idx) = right_Euler_rl(i);
        data_idx++;
    }
}


void AvatarController::feedforwardPolicy()
{
    for (int i = 0; i <num_state; i++)
    {
        state_(i) = (state_(i) - state_mean_(i)) / sqrt(state_var_(i) + 1.0e-08);
    }
    
    hidden_layer1_ = policy_net_w0_ * state_ + policy_net_b0_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (hidden_layer1_(i) < 0)
            hidden_layer1_(i) = 0.0;
    }

    hidden_layer2_ = policy_net_w2_ * hidden_layer1_ + policy_net_b2_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (hidden_layer2_(i) < 0)
            hidden_layer2_(i) = 0.0;
    }

    rl_action_ = action_net_w_ * hidden_layer2_ + action_net_b_;

    rl_action_(0) = DyrosMath::minmax_cut(rl_action_(0), 0.0, 0.1);
    for (int i = 0; i <3; i++)
    {
        rl_action_(1+i) = DyrosMath::minmax_cut(rl_action_(1+i), -0.1, 0.1);
        rl_action_(7+i) = DyrosMath::minmax_cut(rl_action_(7+i), -0.1, 0.1);
    }
    for (int i = 0; i <3; i++)
    {
        rl_action_(4+i) = DyrosMath::minmax_cut(rl_action_(4+i), -3.14/3, 3.14/3);
        rl_action_(10+i) = DyrosMath::minmax_cut(rl_action_(10+i), -3.14/3, 3.14/3);
    }

    while (rd_.is_action_writing_)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }

    rd_.is_action_writing_ = true;
    int data_idx = 0;

    rd_.rl_action_phase_ = rl_action_(data_idx);
    data_idx++;

    for (int i = 0; i <6; i++)
    {
        rd_.rl_action_left_foot_[i] = rl_action_(data_idx);
        data_idx++;
    }

    for (int i = 0; i <6; i++)
    {
        rd_.rl_action_right_foot_[i] = rl_action_(data_idx);
        data_idx++;
    }
    rd_.is_action_writing_ = false;
}

void AvatarController::reflectRLAction(bool updata_phase, bool update_traj)
{
    while (rd_.is_action_writing_)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }

    rd_.is_action_writing_ = true;

    if (updata_phase)
    {
        // Phase action
        walking_tick_mj += int(rd_.rl_action_phase_ * phase_action_scale_);

        // Regularize
        walking_tick_mj = std::min(walking_tick_mj, int(t_last_));

        if (is_new_step_updated_)
        {
            walking_tick_mj = int(t_start_);
        }
        else
        {
            walking_tick_mj = std::max(walking_tick_mj, int(t_start_+1));
        }
    }
    if (update_traj)
    {
        // Foot trajectory action
        lfoot_trajectory_float_origin_ = lfoot_trajectory_float_;
        rfoot_trajectory_float_origin_ = rfoot_trajectory_float_;

        for (int i = 0; i < 3; i++)
        {
            lfoot_trajectory_float_.translation()(i) += rd_.rl_action_left_foot_[i];
            rfoot_trajectory_float_.translation()(i) += rd_.rl_action_right_foot_[i];
        }

        lfoot_trajectory_float_.linear() = DyrosMath::rotateWithZ(rd_.rl_action_left_foot_[5])*DyrosMath::rotateWithY(rd_.rl_action_left_foot_[4])*DyrosMath::rotateWithX(rd_.rl_action_left_foot_[3])*lfoot_trajectory_float_.linear();
        rfoot_trajectory_float_.linear() = DyrosMath::rotateWithZ(rd_.rl_action_right_foot_[5])*DyrosMath::rotateWithY(rd_.rl_action_right_foot_[4])*DyrosMath::rotateWithX(rd_.rl_action_right_foot_[3])*rfoot_trajectory_float_.linear();

        // Reachable limit normalize
        double L_max = 0.351 + 0.351 - 0.001; // 0.001: margin
        Eigen::Vector3d R_r, R_D, L_r, L_D;
        L_D << 0.11, +0.1025, -0.1025;
        R_D << 0.11, -0.1025, -0.1025;
        L_r = lfoot_trajectory_float_.translation() - L_D;
        R_r = rfoot_trajectory_float_.translation() - R_D;

        if (L_r.norm() > L_max)
        {
            lfoot_trajectory_float_.translation() = L_D + (lfoot_trajectory_float_.translation() - L_D)*(L_max/L_r.norm());
        }
        if (R_r.norm() > L_max)
        {
            rfoot_trajectory_float_.translation() = R_D + (rfoot_trajectory_float_.translation() - R_D)*(L_max/R_r.norm());
        }
    }
    rd_.is_action_writing_ = false;
}


void AvatarController::reflectRLState()
{
    while (rd_.is_state_writing_)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }

    Eigen::Vector3d left_Euler_lipm = DyrosMath::rot2Euler(lfoot_trajectory_float_origin_.linear());
    Eigen::Vector3d right_Euler_lipm = DyrosMath::rot2Euler(rfoot_trajectory_float_origin_.linear());
    Eigen::Vector3d left_Euler_rl = DyrosMath::rot2Euler(lfoot_trajectory_float_.linear());
    Eigen::Vector3d right_Euler_rl = DyrosMath::rot2Euler(rfoot_trajectory_float_.linear());

    rd_.is_state_writing_ = true;

    if (is_right_foot_swing_)
    {
        rd_.walking_phase_ = (walking_tick_mj % int(t_total_)) / t_total_;
    }
    else
    {
        rd_.walking_phase_ = ((walking_tick_mj+int(t_total_)/2) % int(t_total_)) / t_total_;
    }

    for (int i = 0; i <3; i++)
    {        
        rd_.left_foot_pose_lipm_[i] = lfoot_trajectory_float_origin_.translation()(i);
        rd_.right_foot_pose_lipm_[i] = rfoot_trajectory_float_origin_.translation()(i);
        rd_.left_foot_pose_lipm_[i+3] = left_Euler_lipm(i);
        rd_.right_foot_pose_lipm_[i+3] = right_Euler_lipm(i);

        rd_.left_foot_pose_rl_[i] = lfoot_trajectory_float_.translation()(i);
        rd_.right_foot_pose_rl_[i] = rfoot_trajectory_float_.translation()(i);
        rd_.left_foot_pose_rl_[i+3] = left_Euler_rl(i);
        rd_.right_foot_pose_rl_[i+3] = right_Euler_rl(i);
    }
    rd_.is_state_writing_ = false;

    while (rd_.is_sanity_writing_)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }

    rd_.is_sanity_writing_ = true;
    rd_.is_inverse_fine_ = is_inverse_fine_;
    rd_.is_sanity_writing_ = false;
}