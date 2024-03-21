#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <std_msgs/String.h>
#include "gurobi/gurobi_c++.h"

#include "math_type_define.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include "tocabi_msgs/matrix_3_4.h"
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
//#include "tocabi_msgs/FTsensor.h" // real robot experiment

//lexls
// #include <lexls/lexlsi.h>
// #include <lexls/tools.h>
// #include <lexls>

#include <iomanip>
#include <iostream>

// pedal
#include <ros/ros.h>
#include "tocabi_msgs/WalkingCommand.h"
#include <std_msgs/Float32.h>

const int FILE_CNT = 14;

const std::string FILE_NAMES[FILE_CNT] =
{
  ///change this directory when you use this code on the other computer///
    "/home/dh-sung/data/dg/0_flag_.txt",
    "/home/dh-sung/data/dg/1_com_.txt",
    "/home/dh-sung/data/dg/2_zmp_.txt",
    "/home/dh-sung/data/dg/3_foot_.txt",
    "/home/dh-sung/data/dg/4_torque_.txt",
    "/home/dh-sung/data/dg/5_joint_.txt",
    "/home/dh-sung/data/dg/6_hand_.txt",
    "/home/dh-sung/data/dg/7_elbow_.txt",
    "/home/dh-sung/data/dg/8_shoulder_.txt",
    "/home/dh-sung/data/dg/9_acromion_.txt",
    "/home/dh-sung/data/dg/10_hmd_.txt",
    "/home/dh-sung/data/dg/11_tracker_.txt",
    "/home/dh-sung/data/dg/12_qpik_.txt",
    "/home/dh-sung/data/dg/13_tracker_vel_.txt"
};

const std::string calibration_folder_dir_ = "/home/dyros/data/vive_tracker/calibration_log/dh";  //tocabi 
// const std::string calibration_folder_dir_ = "/home/dg/data/vive_tracker/calibration_log/kaleem";    //dg pc
//const std::string calibration_folder_dir_ = "/home/dh-sung/data/avatar/calibration_log/dg";  //master ubuntu 

double thread3_hz_ = 0.0;

class AvatarController
{
public:
    AvatarController(RobotData &rd);
    // ~AvatarController();
    Eigen::VectorQd getControl();
    std::ofstream file[FILE_CNT];
    std::ofstream calibration_log_file_ofstream_[4];
    std::ifstream calibration_log_file_ifstream_[4];
    //void taskCommandToCC(TaskCommand tc_);

    void computeSlow();
    void computeFast();
    void computeThread3();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);


    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_avatar_;
    ros::CallbackQueue queue_avatar_;
    void avatar_callback(const std_msgs::StringConstPtr& msg);
    ros::Subscriber sub_1;

    CQuadraticProgram QP_qdot;
    CQuadraticProgram QP_qdot_larm;
    CQuadraticProgram QP_qdot_rarm;
    CQuadraticProgram QP_qdot_upperbody_;
    CQuadraticProgram QP_qdot_wholebody_;
    std::vector<CQuadraticProgram> QP_qdot_hqpik_;        
    std::vector<CQuadraticProgram> QP_qdot_hqpik2_;
    std::vector<CQuadraticProgram> QP_cam_hqp_;
    CQuadraticProgram QP_mpc_x_;
    CQuadraticProgram QP_mpc_y_;
    CQuadraticProgram QP_motion_retargeting_lhand_;
    CQuadraticProgram QP_motion_retargeting_rhand_;
    CQuadraticProgram QP_motion_retargeting_[3];    // task1: each arm, task2: relative arm, task3: hqp second hierarchy
    CQuadraticProgram QP_stepping_;
    CQuadraticProgram QP_steptiming_;
    CQuadraticProgram QP_cpmpc_x_;
    CQuadraticProgram QP_cpmpc_y_;
    CQuadraticProgram QP_cpmpc_x_new_;
    CQuadraticProgram QP_cpmpc_y_new_;

    Eigen::VectorQd CAM_upper_init_q_; 
    //lQR-HQP (Lexls)
    // LexLS::tools::HierarchyType type_of_hierarchy;
    // LexLS::Index number_of_variables;
    // LexLS::Index number_of_objectives;
    // std::vector<LexLS::Index> number_of_constraints;
    // std::vector<LexLS::ObjectiveType> types_of_objectives;
    // std::vector<Eigen::MatrixXd> objectives;
    // LexLS::ParametersLexLSI parameters;

    // LexLS::internal::LexLSI lsi_;

    std::atomic<bool> atb_grav_update_{false};
    std::atomic<bool> atb_desired_q_update_{false};
    std::atomic<bool> atb_walking_traj_update_{false};
    std::atomic<bool> atb_mpc_x_update_{false};
    std::atomic<bool> atb_mpc_y_update_{false};
    std::atomic<bool> atb_mpc_z_update_{false};
    std::atomic<bool> atb_mpc_update_{false};
    std::atomic<bool> atb_cpmpc_rcv_update_{false};
    std::atomic<bool> atb_cpmpc_x_update_{false};
    std::atomic<bool> atb_cpmpc_y_update_{false};
    std::atomic<bool> atb_new_cpmpc_rcv_update_{false};

    RigidBodyDynamics::Model model_d_;  //updated by desired q
    RigidBodyDynamics::Model model_c_;  //updated by current q
    RigidBodyDynamics::Model model_C_;  //for calcuating Coriolis matrix
    RigidBodyDynamics::Model model_MJ_;  //for calcuating CMM

    //////////dg custom controller functions////////
    void setGains();
    void getRobotData();
    void getProcessedRobotData();
    void walkingStateManager();
    void motionGenerator();
    void getCOMTrajectory_dg();
    void getLegIK();
    void getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired);
    void getSwingFootXYZTrajectory();
    void computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des);
    Eigen::VectorQd hipAngleCompensator(Eigen::VectorQd desired_q);
    Eigen::VectorQd jointControl(Eigen::VectorQd current_q, Eigen::VectorQd &desired_q, Eigen::VectorQd current_q_dot, Eigen::VectorQd &desired_q_dot, Eigen::VectorQd pd_mask);
    Eigen::VectorQd gravityCompensator(Eigen::VectorQd current_q);
    void cpCompensator();
    
    Eigen::VectorQd comVelocityControlCompute();
    Eigen::VectorQd swingFootControlCompute();
    Eigen::VectorQd jointTrajectoryPDControlCompute();
    Eigen::VectorQd dampingControlCompute();
    Eigen::VectorQd jointLimit(); 
    Eigen::VectorQd ikBalanceControlCompute();

    void computeCAMcontrol_HQP();
    void cpcontroller_MPC_MJDG(double MPC_freq, double preview_window);
    void new_cpcontroller_MPC_MJDG(double MPC_freq, double preview_window);
    void comGenerator_MPC_wieber(double MPC_freq, double T, double preview_window, int MPC_synchro_hz_);
    void comGenerator_MPC_qcqp2(double MPC_freq_qcqp_, double T_qcqp_, double preview_window_qcqp_, int MPC_synchro_hz_qcqp_);
    void comGenerator_MPC_joe(double MPC_freq, double T, double preview_window, int MPC_synchro_hz_);

    void CPMPC_bolt_Controller_MJ();
    void steptimingController_MJ();
    void BoltController_MJ();
    void getComTrajectory_mpc();
    void getComTrajectory_mpc_ding();
    //estimator
    Eigen::VectorXd momentumObserver(VectorXd current_momentum, VectorXd current_torque, VectorXd nonlinear_term, VectorXd mob_residual_pre, double dt, double k);
    Eigen::MatrixXd getCMatrix(VectorXd q, VectorXd qdot);

    bool balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d);
    int checkZMPinWhichFoot(Eigen::Vector2d zmp_measured); // check where the zmp is
    Eigen::VectorQd tuneTorqueForZMPSafety(Eigen::VectorQd task_torque); // check where the zmp is
    // Eigen::VectorQd zmpAnkleControl();
    // Eigen::VectorQd jointComTrackingTuning();
    void fallDetection();

    //motion control
    void motionRetargeting();
    void motionRetargeting2();
    void motionRetargeting_QPIK_larm();
    void motionRetargeting_QPIK_rarm();
    void motionRetargeting_QPIK_upperbody();
    void motionRetargeting_QPIK_wholebody();
    void motionRetargeting_HQPIK();
    void motionRetargeting_HQPIK2();
    // void motionRetargeting_HQPIK_lexls();
    void rawMasterPoseProcessing();
    void exoSuitRawDataProcessing();
    void azureKinectRawDataProcessing();
    void hmdRawDataProcessing();
    void poseCalibration();
    void getCenterOfShoulderCali(Eigen::Vector3d Still_pose_cali, Eigen::Vector3d T_pose_cali, Eigen::Vector3d Forward_pose_cali, Eigen::Vector3d &CenterOfShoulder_cali);
    void abruptMotionFilter();
    Eigen::Vector3d kinematicFilter(Eigen::Vector3d position_data, Eigen::Vector3d pre_position_data, Eigen::Vector3d reference_position, double boundary, bool &check_boundary);
    Eigen::Isometry3d velocityFilter(Eigen::Isometry3d data, Eigen::Isometry3d pre_data, Eigen::Vector6d &vel_data, double max_vel, int &cur_iter, int max_iter, bool &check_velocity);
    
    void qpRetargeting_1();
    void qpRetargeting_21();
    void qpRetargeting_21Transition(double beta);

    void masterTrajectoryTest();
    
    void getTranslationDataFromText(std::ifstream &text_file, Eigen::Vector3d &trans);
    void getMatrix3dDataFromText(std::ifstream &text_file, Eigen::Matrix3d &mat);
    void getIsometry3dDataFromText(std::ifstream &text_file, Eigen::Isometry3d &isom);

    //preview related functions
    void getComTrajectory_Preview();
    void modifiedPreviewControl_MJ();
    void previewParam_MJ(double dt, int NL, double zc, Eigen::Matrix4d& K, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::MatrixXd& A_bar, Eigen::VectorXd& B_bar);
    void preview_MJ(double dt, int NL, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD);
    Eigen::MatrixXd discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q);
    void getCentroidalMomentumMatrix(MatrixXd mass_matrix, MatrixXd &CMM);
    void updateCMM_DG();
    void CentroidalMomentCalculator();
    void CentroidalMomentCalculator_new();

    void getZmpTrajectory_dg();
    void savePreData();
    void printOutTextFile();

    double bandBlock(double value, double max, double min);
    /////////////////////////////////////////////////////////

    //////////dg ROS related////////
    ros::Subscriber walking_slider_command;

    ros::Subscriber upperbodymode_sub;
    ros::Subscriber nextswingleg_sub;

    ros::Subscriber com_walking_pd_gain_sub;
    ros::Subscriber pelv_ori_pd_gain_sub;
    ros::Subscriber support_foot_damping_gain_sub;
    ros::Subscriber dg_leg_pd_gain_sub;
    ros::Subscriber alpha_x_sub;
    ros::Subscriber alpha_y_sub;
    ros::Subscriber step_width_sub;

    ros::Subscriber test1_sub;
    ros::Subscriber test2_sub;

    ros::Subscriber arm_pd_gain_sub;
    ros::Subscriber waist_pd_gain_sub;

    // master sensor
    ros::Subscriber hmd_posture_sub;
    ros::Subscriber left_controller_posture_sub;
    ros::Subscriber right_controller_posture_sub;
    ros::Subscriber lhand_tracker_posture_sub;
    ros::Subscriber rhand_tracker_posture_sub;
    ros::Subscriber lelbow_tracker_posture_sub;
    ros::Subscriber relbow_tracker_posture_sub;
    ros::Subscriber chest_tracker_posture_sub;
    ros::Subscriber pelvis_tracker_posture_sub;
    ros::Subscriber tracker_status_sub;

    ros::Subscriber vive_tracker_pose_calibration_sub;
    ros::Subscriber opto_ftsensor_sub;
    ros::Publisher calibration_state_pub;
    ros::Publisher calibration_state_gui_log_pub;

    ros::Publisher mujoco_ext_force_apply_pub;
    std_msgs::Float32MultiArray mujoco_applied_ext_force_; // 6 ext wrench + 1 link idx

    void WalkingSliderCommandCallback(const std_msgs::Float32MultiArray &msg);

    void UpperbodyModeCallback(const std_msgs::Int8 &msg);
    void NextSwinglegCallback(const std_msgs::Float32 &msg);

    void ComPosGainCallback(const std_msgs::Float32MultiArray &msg);
    void PelvOriGainCallback(const std_msgs::Float32MultiArray &msg);
    void SupportFootDampingGainCallback(const std_msgs::Float32MultiArray &msg);
    void LegJointGainCallback(const std_msgs::Float32MultiArray &msg);
    void AlphaXCallback(const std_msgs::Float32 &msg);
    void AlphaYCallback(const std_msgs::Float32 &msg);
    void StepWidthCommandCallback(const std_msgs::Float32 &msg);
    
    void Test1CommandCallback(const std_msgs::Float32 &msg);
    void Test2CommandCallback(const std_msgs::Float32 &msg);

    void ArmJointGainCallback(const std_msgs::Float32MultiArray &msg);
    void WaistJointGainCallback(const std_msgs::Float32MultiArray &msg);

    // HMD + Tracker related
    void LeftHandTrackerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void RightHandTrackerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void LeftElbowTrackerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void RightElbowTrackerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void ChestTrackerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void PelvisTrackerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void LeftControllerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void RightControllerCallback(const tocabi_msgs::matrix_3_4 &msg);
    void HmdCallback(const tocabi_msgs::matrix_3_4 &msg);
    void PoseCalibrationCallback(const std_msgs::Int8 &msg);
    void TrackerStatusCallback(const std_msgs::Bool &msg);

    void ExosuitCallback(const geometry_msgs::PoseArray &msg);

    void AzureKinectCallback(const visualization_msgs::MarkerArray &msg);

    //void OptoforceFTCallback(const tocabi_msgs::FTsensor &msg); // real robot experiment
    ///////////////////////////////

    ////////////////dg custom controller variables/////////////
    /////
    ///// global: variables represented from the gravity alined frame which is attatched at the pelvis frame
    ///// local: variables represented from its link frame
    /////
    ///////////////////////////////////////////////////////////

    unsigned int upper_body_mode_;                          // 1: init pose,  2: zero pose, 3: swing arm 4: motion retarggeting
    bool walking_mode_on_;                                  // turns on when the walking control command is received and truns off after saving start time
    double stop_vel_threshold_;                             // acceptable capture point deviation from support foot
    bool chair_mode_;                                       // For chair sitting mode

    int foot_contact_;                                      // 1:left,   -1:right,   0:double
    int foot_contact_pre_;
    bool foot_swing_trigger_;                               // trigger swing if the robot needs to balance.
    bool first_step_trigger_;                               // ture if this is first swing foot. turn on at the start of the swing.
    bool start_walking_trigger_;                            // true when the walking_speed_ is not zero and leg-swing do not start.
    bool stop_walking_trigger_;                             // turns on when the robot's speed become zero and lands last foot step
    bool falling_detection_flag_;                           // turns on when the robot is falling and is considered that it can not recover balance.

    int stop_walking_counter_;                              // Stepping number after walking speed command is zero
    int max_stop_walking_num_;                              // maximum stepping number robot will walk after walking speed is commanded zero 

    double stance_start_time_;
    double program_start_time_;
    
    double program_ready_duration_;                         // during [program_start_time, program_start_time + program_ready_duration_], init parameters are calculated and the robot is position-controlled to the inin_q
    double walking_control_transition_duration_;            

    double walking_duration_;
    double walking_duration_cmd_;
    double walking_duration_start_delay_;
    double walking_phase_;
    // double dsp_phase_;
    // double ssp_phase_;

    double turning_duration_;
    double turning_phase_;
    double switching_phase_duration_;
    double dsp_duration_;
    double dsp_ratio_;

    double current_time_;
    double pre_time_;
    double start_time_;
    double dt_;
    double init_leg_time_;  //for first smothing of leg joint angle

    double walking_speed_;
    double walking_speed_side_;
    double yaw_angular_vel_;
    double knee_target_angle_;

    double step_width_;
    double step_length_;

    double swing_foot_height_;
    double com_target_height_;

    double ankle2footcenter_offset_;

    double first_torque_supplier_;                         // this increase with cubic function from 0 to 1 during [program_start_time + program_ready_duration_, program_start_time + program_ready_duration_ + walking_control_transition_duration_]
    double swingfoot_force_control_converter_;
    double swingfoot_highest_time_;
    
    // CoM variables in global frame
    Eigen::Vector3d com_pos_desired_; 
    Eigen::Vector3d com_vel_desired_;
    Eigen::Vector3d com_acc_desired_;
    Eigen::Vector3d com_pos_current_;
    Eigen::Vector3d com_vel_current_;
    Eigen::Vector3d com_acc_current_;


    
    double com_vel_cutoff_freq_;
    double wn_;
    double com_mass_;


    Eigen::Vector3d com_pos_init_;
    Eigen::Vector3d com_vel_init_;
    Eigen::Vector3d com_acc_init_;
    Eigen::Vector3d com_pos_desired_pre_; 
    Eigen::Vector3d com_vel_desired_pre_;
    Eigen::Vector3d com_acc_desired_pre_;
    Eigen::Vector3d com_pos_desired_last_; 
    Eigen::Vector3d com_vel_desired_last_;
    Eigen::Vector3d com_acc_desired_last_;
    
    Eigen::Vector3d com_pos_pre_desired_from_;
    Eigen::Vector3d com_vel_pre_desired_from_;
    Eigen::Vector3d com_acc_pre_desired_from_;

    Eigen::Vector3d com_pos_error_;
    Eigen::Vector3d com_vel_error_;

    Eigen::Vector3d com_vel_est1_;  // Jv*q_dot
    Eigen::Vector3d com_vel_est2_;  // Jv*q_dot + r X w_f

    Eigen::Matrix3d kp_compos_; // 196(sim) (tune)
	Eigen::Matrix3d kd_compos_;	 // 28(sim) (tune)

    // Pevlis related variables
    Eigen::Vector3d pelv_pos_current_;
    Eigen::Vector6d pelv_vel_current_;
    Eigen::Vector3d pelv_angvel_current_;
    Eigen::Matrix3d pelv_rot_current_;
    Eigen::Vector3d pelv_rpy_current_;
    Eigen::Matrix3d pelv_rot_current_yaw_aline_;
    Eigen::Matrix3d pelv_yaw_rot_current_from_global_;
    Eigen::Isometry3d pelv_transform_current_from_global_;
    double pelv_height_offset_;
     
    Eigen::Vector3d pelv_pos_init_;
    Eigen::Vector6d pelv_vel_init_;
    Eigen::Matrix3d pelv_rot_init_;
    Eigen::Vector3d pelv_rpy_init_;
    Eigen::Matrix3d pelv_rot_init_yaw_aline_;
    Eigen::Isometry3d pelv_transform_init_from_global_;
    Eigen::Isometry3d pelv_trajectory_support_init_;


	Eigen::Vector3d phi_pelv_;
	Eigen::Vector3d torque_pelv_;
	Eigen::VectorQd torque_stance_hip_;
	Eigen::VectorQd torque_swing_assist_;

    Eigen::Matrix3d kp_pelv_ori_; // 196(sim) (tune)
	Eigen::Matrix3d kd_pelv_ori_;	 // 28(sim) (tune)
    // Joint related variables
    Eigen::VectorQd current_q_;
    Eigen::VectorQd current_q_dot_;
    Eigen::VectorQd current_q_ddot_;
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_q_dot_;
    Eigen::VectorQd desired_q_ddot_;
    Eigen::VectorQd pre_q_;
    Eigen::VectorQd pre_desired_q_;
    Eigen::VectorQd pre_desired_q_dot_;
    Eigen::VectorQd last_desired_q_;
    Eigen::VectorQVQd pre_desired_q_qvqd_;
    Eigen::VectorVQd pre_desired_q_dot_vqd_;
    Eigen::VectorVQd pre_desired_q_ddot_vqd_;

    Eigen::VectorQd desired_q_fast_;
    Eigen::VectorQd desired_q_dot_fast_;
    Eigen::VectorQd desired_q_slow_;
    Eigen::VectorQd desired_q_dot_slow_;

    Eigen::VectorQd motion_q_;
    Eigen::VectorQd motion_q_dot_;
    Eigen::VectorQd motion_q_pre_;
    Eigen::VectorQd motion_q_dot_pre_;
    Eigen::VectorQd init_q_;
    Eigen::VectorQd zero_q_;
    Eigen::VectorQVQd init_q_virtual_;

    Eigen::MatrixVVd A_mat_;
    Eigen::MatrixVVd A_mat_pre_;
    Eigen::MatrixVVd A_inv_mat_;
    Eigen::MatrixVVd A_dot_mat_;

    Eigen::MatrixVVd motor_inertia_mat_;
    Eigen::MatrixVVd motor_inertia_inv_mat_;

    Eigen::MatrixVVd C_mat_;
    Eigen::MatrixVVd C_T_mat_;
    Eigen::VectorVQd nonlinear_torque_;

    Eigen::VectorQd kp_joint_;
	Eigen::VectorQd kv_joint_;

	Eigen::VectorQd kp_stiff_joint_;
	Eigen::VectorQd kv_stiff_joint_;
	Eigen::VectorQd kp_soft_joint_;
	Eigen::VectorQd kv_soft_joint_;
    // walking controller variables
    double alpha_x_;
	double alpha_y_;
    double alpha_x_command_;    
    double alpha_y_command_;
    Eigen::VectorQd pd_control_mask_; //1 for joint ik pd control

    Eigen::Vector2d target_foot_landing_from_pelv_;
    Eigen::Vector2d target_foot_landing_from_sup_;
    Eigen::Vector3d swing_foot_pos_trajectory_from_global_;
    Eigen::Vector6d swing_foot_vel_trajectory_from_global_;
    Eigen::Vector6d swing_foot_acc_trajectory_from_global_;
    Eigen::Matrix3d swing_foot_rot_trajectory_from_global_;

    Eigen::Vector3d swing_foot_pos_trajectory_from_support_;
    Eigen::Vector6d swing_foot_vel_trajectory_from_support_;
    Eigen::Vector6d swing_foot_acc_trajectory_from_support_;
    Eigen::Matrix3d swing_foot_rot_trajectory_from_support_;

    Eigen::Vector3d swing_foot_pos_error_from_support_;

    Eigen::Isometry3d swing_foot_transform_init_;
    Eigen::Vector3d swing_foot_rpy_init_;
    Eigen::Isometry3d support_foot_transform_init_;
    Eigen::Vector3d support_foot_rpy_init_;

    Eigen::Isometry3d swing_foot_transform_current_;
    Eigen::Isometry3d support_foot_transform_current_;

    Eigen::Isometry3d swing_foot_transform_pre_;
    Eigen::Isometry3d support_foot_transform_pre_;

    Eigen::Vector6d swing_foot_vel_current_;
    Eigen::Vector6d swing_foot_vel_init_;

    Eigen::Vector6d support_foot_vel_current_;

    //getLegIK
    Eigen::Isometry3d lfoot_transform_desired_;
	Eigen::Isometry3d rfoot_transform_desired_;
	Eigen::Isometry3d pelv_transform_desired_;
    Eigen::Isometry3d lfoot_transform_desired_last_;
	Eigen::Isometry3d rfoot_transform_desired_last_;
	Eigen::Isometry3d pelv_transform_desired_last_;

    Eigen::MatrixXd jac_com_;
    Eigen::MatrixXd jac_com_pos_;
    Eigen::MatrixXd jac_rhand_;
    Eigen::MatrixXd jac_lhand_;
    Eigen::MatrixXd jac_rfoot_;
    Eigen::MatrixXd jac_lfoot_;

    Eigen::MatrixXd lfoot_to_com_jac_from_global_;
	Eigen::MatrixXd rfoot_to_com_jac_from_global_;
    
    Eigen::Isometry3d pelv_transform_start_from_global_;
    Eigen::Isometry3d rfoot_transform_start_from_global_;
    Eigen::Isometry3d lfoot_transform_start_from_global_;

    Eigen::Isometry3d upperbody_transform_init_from_global_;
    Eigen::Isometry3d head_transform_init_from_global_;
    Eigen::Isometry3d lfoot_transform_init_from_global_;
    Eigen::Isometry3d rfoot_transform_init_from_global_;
    Eigen::Isometry3d lhand_transform_init_from_global_;
    Eigen::Isometry3d rhand_transform_init_from_global_;
    Eigen::Isometry3d lelbow_transform_init_from_global_;
    Eigen::Isometry3d relbow_transform_init_from_global_;
    Eigen::Isometry3d lupperarm_transform_init_from_global_; //4rd axis of arm joint 
    Eigen::Isometry3d rupperarm_transform_init_from_global_; 
    Eigen::Isometry3d lshoulder_transform_init_from_global_; //3rd axis of arm joint 
    Eigen::Isometry3d rshoulder_transform_init_from_global_; 
    Eigen::Isometry3d lacromion_transform_init_from_global_; //2nd axis of arm joint (견봉)
    Eigen::Isometry3d racromion_transform_init_from_global_;
    Eigen::Isometry3d larmbase_transform_init_from_global_; //1st axis of arm joint (견봉)
    Eigen::Isometry3d rarmbase_transform_init_from_global_;

    Eigen::Isometry3d upperbody_transform_current_from_global_;
    Eigen::Isometry3d head_transform_current_from_global_;
    Eigen::Isometry3d lfoot_transform_current_from_global_;
    Eigen::Isometry3d rfoot_transform_current_from_global_;
    Eigen::Isometry3d lhand_transform_current_from_global_;
    Eigen::Isometry3d rhand_transform_current_from_global_;
    Eigen::Isometry3d lelbow_transform_current_from_global_;
    Eigen::Isometry3d relbow_transform_current_from_global_;
    Eigen::Isometry3d lupperarm_transform_current_from_global_; //4th axis of arm joint
    Eigen::Isometry3d rupperarm_transform_current_from_global_;
    Eigen::Isometry3d lshoulder_transform_current_from_global_; //3rd axis of arm joint
    Eigen::Isometry3d rshoulder_transform_current_from_global_;
    Eigen::Isometry3d lacromion_transform_current_from_global_; //2nd axis of arm joint (견봉)
    Eigen::Isometry3d racromion_transform_current_from_global_;
    Eigen::Isometry3d larmbase_transform_current_from_global_; //1st axis of arm joint 
    Eigen::Isometry3d rarmbase_transform_current_from_global_;

    Eigen::Isometry3d lknee_transform_current_from_global_;
    Eigen::Isometry3d rknee_transform_current_from_global_;

    Eigen::Vector3d lhand_rpy_current_from_global_;
    Eigen::Vector3d rhand_rpy_current_from_global_;
    Eigen::Vector3d lelbow_rpy_current_from_global_;
    Eigen::Vector3d relbow_rpy_current_from_global_;
    Eigen::Vector3d lupperarm_rpy_current_from_global_;
    Eigen::Vector3d rupperarm_rpy_current_from_global_;
    Eigen::Vector3d lshoulder_rpy_current_from_global_;
    Eigen::Vector3d rshoulder_rpy_current_from_global_;
    Eigen::Vector3d lacromion_rpy_current_from_global_;
    Eigen::Vector3d racromion_rpy_current_from_global_;

    Eigen::Isometry3d upperbody_transform_pre_desired_from_;
    Eigen::Isometry3d head_transform_pre_desired_from_;
    Eigen::Isometry3d lfoot_transform_pre_desired_from_;
    Eigen::Isometry3d rfoot_transform_pre_desired_from_;
    Eigen::Isometry3d lhand_transform_pre_desired_from_;
    Eigen::Isometry3d rhand_transform_pre_desired_from_;
    Eigen::Isometry3d lelbow_transform_pre_desired_from_;
    Eigen::Isometry3d relbow_transform_pre_desired_from_;
    Eigen::Isometry3d lupperarm_transform_pre_desired_from_;
    Eigen::Isometry3d rupperarm_transform_pre_desired_from_;
    Eigen::Isometry3d lshoulder_transform_pre_desired_from_;
    Eigen::Isometry3d rshoulder_transform_pre_desired_from_;
    Eigen::Isometry3d lacromion_transform_pre_desired_from_;
    Eigen::Isometry3d racromion_transform_pre_desired_from_;
    Eigen::Isometry3d larmbase_transform_pre_desired_from_; //1st axis of arm joint
    Eigen::Isometry3d rarmbase_transform_pre_desired_from_;

    Eigen::Vector3d lhand_control_point_offset_, rhand_control_point_offset_;   //red_hand made by sy

    Eigen::Vector6d lfoot_vel_current_from_global_;
    Eigen::Vector6d rfoot_vel_current_from_global_;
    Eigen::Vector6d lhand_vel_current_from_global_;
    Eigen::Vector6d rhand_vel_current_from_global_;
    Eigen::Vector6d lelbow_vel_current_from_global_;
    Eigen::Vector6d relbow_vel_current_from_global_;
    Eigen::Vector6d lupperarm_vel_current_from_global_;
    Eigen::Vector6d rupperarm_vel_current_from_global_;
    Eigen::Vector6d lshoulder_vel_current_from_global_;
    Eigen::Vector6d rshoulder_vel_current_from_global_;
    Eigen::Vector6d lacromion_vel_current_from_global_;
    Eigen::Vector6d racromion_vel_current_from_global_;

    Eigen::Isometry3d lfoot_transform_init_from_support_;
    Eigen::Isometry3d rfoot_transform_init_from_support_;
    Eigen::Isometry3d pelv_transform_init_from_support_;
    Eigen::Vector3d pelv_rpy_init_from_support_;

    Eigen::Isometry3d lfoot_transform_start_from_support_;
    Eigen::Isometry3d rfoot_transform_start_from_support_;
    Eigen::Isometry3d pelv_transform_start_from_support_;

    Eigen::Isometry3d lfoot_transform_current_from_support_;
    Eigen::Isometry3d rfoot_transform_current_from_support_;
    Eigen::Isometry3d pelv_transform_current_from_support_;

    Eigen::Vector6d lfoot_vel_current_from_support_;
    Eigen::Vector6d rfoot_vel_current_from_support_;

    Eigen::Isometry3d swing_foot_transform_init_from_support_;
    Eigen::Vector3d swing_foot_rpy_init_from_support_;
    Eigen::Isometry3d support_foot_transform_init_from_support_;
    Eigen::Vector3d support_foot_rpy_init_from_support_;

    Eigen::Isometry3d swing_foot_transform_current_from_support_;
    Eigen::Isometry3d support_foot_transform_current_from_support_;

    Eigen::Isometry3d swing_foot_transform_pre_from_support_;
    Eigen::Isometry3d support_foot_transform_pre_from_support_;

    Eigen::Vector3d middle_of_both_foot_;
    Eigen::Vector3d middle_of_both_foot_init_;

    Eigen::Vector3d com_pos_init_from_support_;

    Eigen::Vector3d com_pos_desired_from_support_; 
    Eigen::Vector3d com_vel_desired_from_support_;
    Eigen::Vector3d com_acc_desired_from_support_;
    Eigen::Vector3d com_jerk_desired_from_support_;

    Eigen::Vector3d com_pos_pre_desired_from_support_; 
    Eigen::Vector3d com_vel_pre_desired_from_support_;
    Eigen::Vector3d com_acc_pre_desired_from_support_;
    Eigen::Vector3d com_jerk_pre_desired_from_support_;

    Eigen::Vector3d com_pos_current_from_support_;
    Eigen::Vector3d com_vel_current_from_support_;
    Eigen::Vector3d com_acc_current_from_support_;
    Eigen::Vector3d com_pos_pre_from_support_;
    Eigen::Vector3d com_vel_pre_from_support_;
    Eigen::Vector3d com_acc_pre_from_support_;
    Eigen::Vector3d com_pos_ppre_from_support_;
    Eigen::Vector3d com_vel_ppre_from_support_;
    Eigen::Vector3d com_acc_ppre_from_support_;
    
    Eigen::Vector3d com_vel_current_lpf_from_support_;
    Eigen::Vector3d com_vel_pre_lpf_from_support_;
    Eigen::Vector3d com_vel_ppre_lpf_from_support_;

    Eigen::Vector6d com_pos_limit_; //min x y z max x y z
    Eigen::Vector6d com_vel_limit_;
    Eigen::Vector6d com_acc_limit_;

    Eigen::Vector3d cp_current_from_suppport_;

    Eigen::Vector6d contact_force_lfoot_;
    Eigen::Vector6d contact_force_rfoot_;
    
    Eigen::Vector6d contact_force_lfoot_local_;
    Eigen::Vector6d contact_force_rfoot_local_;

    Eigen::Vector3d zmp_measured_;
    Eigen::Vector3d zmp_measured_pre_;
    Eigen::Vector3d zmp_measured_ppre_;
    Eigen::Vector3d zmp_estimated_;
    Eigen::Vector3d zmp_dot_measured_;

    Eigen::Vector3d zmp_measured_local_; //calc zmp with F/T sensors according to the Robot.ee_[0].contact
    Eigen::Vector3d zmp_dot_measured_local_;

    Eigen::Vector3d zmp_local_lfoot_;
    Eigen::Vector3d zmp_local_rfoot_;
    Eigen::Vector3d zmp_local_lfoot_pre_;
    Eigen::Vector3d zmp_local_rfoot_pre_;
    Eigen::Vector3d zmp_dot_local_rfoot_;
    Eigen::Vector3d zmp_dot_local_lfoot_;

    Eigen::Vector3d zmp_measured_lfoot_; //calc only left foot zmp with a F/T sensor
    Eigen::Vector3d zmp_measured_rfoot_;

    Eigen::Vector3d zmp_current_by_com_from_support_;

    Eigen::Vector3d zmp_desired_from_global_;
    Eigen::Vector3d zmp_desired_pre_;

    double zmp_y_offset_;
    
    Eigen::Vector6d l_ft_;
    Eigen::Vector6d r_ft_;

    Eigen::Vector6d l_ft_LPF;
    Eigen::Vector6d r_ft_LPF;

    Eigen::Vector6d opto_ft_raw_;
    Eigen::Vector6d opto_ft_;

    double F_F_input_dot = 0;
    double F_F_input = 0;

    double F_T_L_x_input = 0;
    double F_T_L_x_input_dot = 0;
    double F_T_R_x_input = 0;
    double F_T_R_x_input_dot = 0;  

    double F_T_L_y_input = 0;
    double F_T_L_y_input_dot = 0;
    double F_T_R_y_input = 0;
    double F_T_R_y_input_dot = 0;

    Eigen::Vector2d f_star_xy_;
    Eigen::Vector2d f_star_xy_pre_;
    Eigen::Vector6d f_star_6d_;
    Eigen::Vector6d f_star_6d_pre_;
	Eigen::Vector6d f_star_l_;   
	Eigen::Vector6d f_star_r_;
	Eigen::Vector6d f_star_l_pre_;   
	Eigen::Vector6d f_star_r_pre_;

    Eigen::VectorQd torque_task_;
    Eigen::VectorQd torque_init_;
    Eigen::VectorQd torque_grav_;
    Eigen::VectorQd torque_task_pre_;
    Eigen::VectorQd torque_grav_pre_;
    Eigen::VectorQd torque_qp_;
    Eigen::VectorQd torque_g_;
    Eigen::VectorQd torque_upper_;
    Eigen::VectorQd torque_upper_fast_;
    Eigen::VectorQd torque_lower_;

    Eigen::VectorQd torque_task_min_;
    Eigen::VectorQd torque_task_max_;
    //getComTrajectory() variables
    double xi_;
    double yi_;
    Eigen::Vector3d xs_;
    Eigen::Vector3d ys_;
    Eigen::Vector3d xd_;
    Eigen::Vector3d yd_;
    Eigen::Vector3d xd_b;
    Eigen::Vector3d yd_b;

    //Preview Control
    double preview_horizon_;
    double preview_hz_;
    double preview_update_time_;
    double last_preview_param_update_time_;
    
    Eigen::Vector3d preview_x, preview_y, preview_x_b, preview_y_b, preview_x_b2, preview_y_b2;
    double ux_, uy_, ux_1_, uy_1_;
    double zc_;
    double gi_;
    double zmp_start_time_; //원래 코드에서는 start_time, zmp_ref 시작되는 time같음
    double mpc_start_time_;
    
    Eigen::Matrix4d k_;
    Eigen::Matrix4d K_act_;
    Eigen::VectorXd gp_l_;
    Eigen::Matrix1x3d gx_;
    Eigen::Matrix3d a_;
    Eigen::Vector3d b_;
    Eigen::Matrix1x3d c_;

    //Preview CPM
    Eigen::MatrixXd A_;
    Eigen::VectorXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd D_;
    Eigen::Matrix3d K_;
    Eigen::MatrixXd Gi_;
    Eigen::MatrixXd Gx_;
    Eigen::VectorXd Gd_;
    Eigen::MatrixXd A_bar_;
    Eigen::VectorXd B_bar_;
    Eigen::Vector2d Preview_X, Preview_Y, Preview_X_b, Preview_Y_b;
    Eigen::VectorXd X_bar_p_, Y_bar_p_;
    Eigen::Vector2d XD_;
    Eigen::Vector2d YD_;
    double UX_, UY_;

    int zmp_size_;

    Eigen::MatrixXd ref_zmp_;
    Eigen::Vector3d com_pos_desired_preview_;
    Eigen::Vector3d com_vel_desired_preview_;
    Eigen::Vector3d com_acc_desired_preview_;

    Eigen::Vector3d com_pos_desired_preview_pre_;
    Eigen::Vector3d com_vel_desired_preview_pre_;
    Eigen::Vector3d com_acc_desired_preview_pre_;

    //siwngFootControlCompute
    Vector3d swingfoot_f_star_l_;
    Vector3d swingfoot_f_star_r_;
    Vector3d swingfoot_f_star_l_pre_;
    Vector3d swingfoot_f_star_r_pre_;

    //dampingControlCompute
    Vector3d f_lfoot_damping_;
	Vector3d f_rfoot_damping_;	
    Vector3d f_lfoot_damping_pre_;
	Vector3d f_rfoot_damping_pre_;	
    Matrix3d support_foot_damping_gain_;

    //MotionRetargeting variables
    int upperbody_mode_recieved_;
    double upperbody_command_time_;
    Eigen::VectorQd upperbody_mode_q_init_;

    Eigen::Isometry3d master_lhand_pose_raw_;
    Eigen::Isometry3d master_rhand_pose_raw_;
    Eigen::Isometry3d master_head_pose_raw_;
    Eigen::Isometry3d master_lelbow_pose_raw_;
    Eigen::Isometry3d master_relbow_pose_raw_;
    Eigen::Isometry3d master_lshoulder_pose_raw_;
    Eigen::Isometry3d master_rshoulder_pose_raw_;
    Eigen::Isometry3d master_upperbody_pose_raw_;

    Eigen::Isometry3d master_lhand_pose_raw_pre_;
    Eigen::Isometry3d master_rhand_pose_raw_pre_;
    Eigen::Isometry3d master_head_pose_raw_pre_;
    Eigen::Isometry3d master_lelbow_pose_raw_pre_;
    Eigen::Isometry3d master_relbow_pose_raw_pre_;
    Eigen::Isometry3d master_lshoulder_pose_raw_pre_;
    Eigen::Isometry3d master_rshoulder_pose_raw_pre_;
    Eigen::Isometry3d master_upperbody_pose_raw_pre_;

    Eigen::Isometry3d master_lhand_pose_raw_ppre_;
    Eigen::Isometry3d master_rhand_pose_raw_ppre_;
    Eigen::Isometry3d master_head_pose_raw_ppre_;
    Eigen::Isometry3d master_lelbow_pose_raw_ppre_;
    Eigen::Isometry3d master_relbow_pose_raw_ppre_;
    Eigen::Isometry3d master_lshoulder_pose_raw_ppre_;
    Eigen::Isometry3d master_rshoulder_pose_raw_ppre_;
    Eigen::Isometry3d master_upperbody_pose_raw_ppre_;

    Eigen::Isometry3d master_lhand_pose_;
    Eigen::Isometry3d master_rhand_pose_;
    Eigen::Isometry3d master_head_pose_;
    Eigen::Isometry3d master_lelbow_pose_;
    Eigen::Isometry3d master_relbow_pose_;
    Eigen::Isometry3d master_lshoulder_pose_;
    Eigen::Isometry3d master_rshoulder_pose_;
    Eigen::Isometry3d master_upperbody_pose_;

    Eigen::Isometry3d master_lhand_pose_pre_;
    Eigen::Isometry3d master_rhand_pose_pre_;
    Eigen::Isometry3d master_head_pose_pre_;
    Eigen::Isometry3d master_lelbow_pose_pre_;
    Eigen::Isometry3d master_relbow_pose_pre_;
    Eigen::Isometry3d master_lshoulder_pose_pre_;
    Eigen::Isometry3d master_rshoulder_pose_pre_;
    Eigen::Isometry3d master_upperbody_pose_pre_;

    Eigen::Isometry3d master_lhand_pose_ppre_;
    Eigen::Isometry3d master_rhand_pose_ppre_;
    Eigen::Isometry3d master_head_pose_ppre_;
    Eigen::Isometry3d master_lelbow_pose_ppre_;
    Eigen::Isometry3d master_relbow_pose_ppre_;
    Eigen::Isometry3d master_lshoulder_pose_ppre_;
    Eigen::Isometry3d master_rshoulder_pose_ppre_;
    Eigen::Isometry3d master_upperbody_pose_ppre_;

    Eigen::Vector6d master_lhand_vel_;
    Eigen::Vector6d master_rhand_vel_;
    Eigen::Vector6d master_head_vel_;
    Eigen::Vector6d master_lelbow_vel_;
    Eigen::Vector6d master_relbow_vel_;
    Eigen::Vector6d master_lshoulder_vel_;
    Eigen::Vector6d master_rshoulder_vel_;
    Eigen::Vector6d master_upperbody_vel_;

    Eigen::Vector3d master_lhand_rqy_;
    Eigen::Vector3d master_rhand_rqy_;
    Eigen::Vector3d master_lelbow_rqy_;
    Eigen::Vector3d master_relbow_rqy_;
    Eigen::Vector3d master_lshoulder_rqy_;
    Eigen::Vector3d master_rshoulder_rqy_;
    
    Eigen::Vector3d master_head_rqy_;

    Eigen::Vector3d master_relative_lhand_pos_raw_;
    Eigen::Vector3d master_relative_rhand_pos_raw_;
    Eigen::Vector3d master_relative_lhand_pos_;
    Eigen::Vector3d master_relative_rhand_pos_;
    Eigen::Vector3d master_relative_lhand_pos_pre_;
    Eigen::Vector3d master_relative_rhand_pos_pre_;

    double robot_arm_max_l_;
    double robot_upperarm_max_l_;
    double robot_lowerarm_max_l_;
    double robot_shoulder_width_;
    ////////////HMD + VIVE TRACKER////////////
    bool hmd_init_pose_calibration_;
    // double hmd_init_pose_cali_time_;

    bool hmd_tracker_status_raw_;   //1: good, 0: bad
    bool hmd_tracker_status_;   //1: good, 0: bad
    bool hmd_tracker_status_pre_;   //1: good, 0: bad

    double tracker_status_changed_time_;
    double calibration_x_l_scale_;
    double calibration_x_r_scale_;
    double calibration_y_l_scale_;
    double calibration_y_r_scale_;
    double calibration_z_l_scale_;
    double calibration_z_r_scale_;
    

    double hmd_larm_max_l_;
    double hmd_rarm_max_l_;
    double hmd_shoulder_width_;

    bool hmd_check_pose_calibration_[5];   // 0: still cali, 1: T pose cali, 2: Forward Stretch cali, 3: Calibration is completed, 4: read calibration from log file
    bool still_pose_cali_flag_;
    bool t_pose_cali_flag_;
    bool forward_pose_cali_flag_;
    bool read_cali_log_flag_;

    Eigen::Isometry3d hmd_head_pose_raw_;
    Eigen::Isometry3d hmd_lshoulder_pose_raw_;
    Eigen::Isometry3d hmd_lupperarm_pose_raw_;
    Eigen::Isometry3d hmd_lhand_pose_raw_;
    Eigen::Isometry3d hmd_rshoulder_pose_raw_;
    Eigen::Isometry3d hmd_rupperarm_pose_raw_;
    Eigen::Isometry3d hmd_rhand_pose_raw_;
    Eigen::Isometry3d hmd_chest_pose_raw_;
    Eigen::Isometry3d hmd_pelv_pose_raw_;

    Eigen::Isometry3d hmd_head_pose_raw_last_;
    Eigen::Isometry3d hmd_lshoulder_pose_raw_last_;
    Eigen::Isometry3d hmd_lupperarm_pose_raw_last_;
    Eigen::Isometry3d hmd_lhand_pose_raw_last_;
    Eigen::Isometry3d hmd_rshoulder_pose_raw_last_;
    Eigen::Isometry3d hmd_rupperarm_pose_raw_last_;
    Eigen::Isometry3d hmd_rhand_pose_raw_last_;
    Eigen::Isometry3d hmd_chest_pose_raw_last_;
    Eigen::Isometry3d hmd_pelv_pose_raw_last_;

    Eigen::Isometry3d hmd_head_pose_;
    Eigen::Isometry3d hmd_lshoulder_pose_;
    Eigen::Isometry3d hmd_lupperarm_pose_;
    Eigen::Isometry3d hmd_lhand_pose_;
    Eigen::Isometry3d hmd_rshoulder_pose_;
    Eigen::Isometry3d hmd_rupperarm_pose_;
    Eigen::Isometry3d hmd_rhand_pose_;
    Eigen::Isometry3d hmd_chest_pose_;
    Eigen::Isometry3d hmd_pelv_pose_;

    Eigen::Isometry3d hmd_head_pose_pre_;
    Eigen::Isometry3d hmd_lshoulder_pose_pre_;
    Eigen::Isometry3d hmd_lupperarm_pose_pre_;
    Eigen::Isometry3d hmd_lhand_pose_pre_;
    Eigen::Isometry3d hmd_rshoulder_pose_pre_;
    Eigen::Isometry3d hmd_rupperarm_pose_pre_;
    Eigen::Isometry3d hmd_rhand_pose_pre_;
    Eigen::Isometry3d hmd_chest_pose_pre_;
    Eigen::Isometry3d hmd_pelv_pose_pre_;

    Eigen::Isometry3d hmd_head_pose_init_;  
    Eigen::Isometry3d hmd_lshoulder_pose_init_; 
    Eigen::Isometry3d hmd_lupperarm_pose_init_;
    Eigen::Isometry3d hmd_lhand_pose_init_;
    Eigen::Isometry3d hmd_rshoulder_pose_init_; 
    Eigen::Isometry3d hmd_rupperarm_pose_init_;
    Eigen::Isometry3d hmd_rhand_pose_init_;
    Eigen::Isometry3d hmd_chest_pose_init_;
    Eigen::Isometry3d hmd_pelv_pose_init_;

    Eigen::Vector3d hmd2robot_lhand_pos_mapping_;
	Eigen::Vector3d hmd2robot_rhand_pos_mapping_;
    Eigen::Vector3d hmd2robot_lelbow_pos_mapping_;
    Eigen::Vector3d hmd2robot_relbow_pos_mapping_;

    Eigen::Vector3d hmd2robot_lhand_pos_mapping_init_;
    Eigen::Vector3d hmd2robot_rhand_pos_mapping_init_;

    Eigen::Vector3d hmd_still_cali_lhand_pos_;
    Eigen::Vector3d hmd_still_cali_rhand_pos_;
    Eigen::Vector3d hmd_tpose_cali_lhand_pos_;
    Eigen::Vector3d hmd_tpose_cali_rhand_pos_;
    Eigen::Vector3d hmd_forward_cali_lhand_pos_;
    Eigen::Vector3d hmd_forward_cali_rhand_pos_;
    
    Eigen::Vector3d hmd_lshoulder_center_pos_;
    Eigen::Vector3d hmd_rshoulder_center_pos_;

    Eigen::Vector3d hmd_chest_2_lshoulder_center_pos_;
    Eigen::Vector3d hmd_chest_2_rshoulder_center_pos_;

    //global frame of vive tracker
    Eigen::Vector6d hmd_head_vel_global_;
    Eigen::Vector6d hmd_lupperarm_vel_global_;
    Eigen::Vector6d hmd_lhand_vel_global_;
    Eigen::Vector6d hmd_rupperarm_vel_global_;
    Eigen::Vector6d hmd_rhand_vel_global_;
    Eigen::Vector6d hmd_chest_vel_global_;
    Eigen::Vector6d hmd_pelv_vel_global_; 

    //pelvis frame
    Eigen::Vector6d hmd_head_vel_;
    Eigen::Vector6d hmd_lshoulder_vel_;
    Eigen::Vector6d hmd_lupperarm_vel_;
    Eigen::Vector6d hmd_lhand_vel_;
    Eigen::Vector6d hmd_rshoulder_vel_;
    Eigen::Vector6d hmd_rupperarm_vel_;
    Eigen::Vector6d hmd_rhand_vel_;
    Eigen::Vector6d hmd_chest_vel_;
    Eigen::Vector6d hmd_pelv_vel_; 
    
    int hmd_head_abrupt_motion_count_;
    int hmd_lupperarm_abrupt_motion_count_;
    int hmd_lhand_abrupt_motion_count_;
    int hmd_rupperarm_abrupt_motion_count_;
    int hmd_rhand_abrupt_motion_count_;
    int hmd_chest_abrupt_motion_count_;
    int hmd_pelv_abrupt_motion_count_;

    ///////////QPIK///////////////////////////
    Eigen::Vector3d lhand_pos_error_;
    Eigen::Vector3d rhand_pos_error_;
    Eigen::Vector3d lhand_ori_error_;
    Eigen::Vector3d rhand_ori_error_;

    Eigen::Vector3d lelbow_ori_error_;
    Eigen::Vector3d relbow_ori_error_;
    Eigen::Vector3d lshoulder_ori_error_;
    Eigen::Vector3d rshoulder_ori_error_;

    Eigen::Vector6d lhand_vel_error_;
    Eigen::Vector6d rhand_vel_error_;
    Eigen::Vector3d lelbow_vel_error_;
    Eigen::Vector3d relbow_vel_error_;
    Eigen::Vector3d lacromion_vel_error_;
    Eigen::Vector3d racromion_vel_error_;
    //////////////////////////////////////////

    /////////////QPIK UPPERBODY /////////////////
    const int hierarchy_num_upperbody_ = 4;
    const int variable_size_upperbody_ = 21;
	const int constraint_size1_upperbody_ = 21;	//[lb <=	x	<= 	ub] form constraints
	const int constraint_size2_upperbody_ = 12;	//[lb <=	Ax 	<=	ub] from constraints
   	const int control_size_upperbody_[4] = {3, 14, 4, 4};		//1: upperbody, 2: head + hand, 3: upperarm, 4: shoulder

	// const int control_size_hand = 12;		//2
	// const int control_size_upperbody = 3;	//1
	// const int control_size_head = 2;		//2
	// const int control_size_upperarm = 4; 	//3
	// const int control_size_shoulder = 4;	//4

    double w1_upperbody_;
    double w2_upperbody_;
    double w3_upperbody_;
    double w4_upperbody_;
    double w5_upperbody_;
    double w6_upperbody_;

    Eigen::MatrixXd H_upperbody_, A_upperbody_;
    Eigen::MatrixXd J_upperbody_[4];
    Eigen::VectorXd g_upperbody_, u_dot_upperbody_[4], qpres_upperbody_, ub_upperbody_, lb_upperbody_, ubA_upperbody_, lbA_upperbody_;
    Eigen::VectorXd q_dot_upperbody_;

    Eigen::MatrixXd N1_upperbody_, N2_aug_upperbody_, N3_aug_upperbody_;
    Eigen::MatrixXd J2_aug_upperbody_, J2_aug_pinv_upperbody_, J3_aug_upperbody_, J3_aug_pinv_upperbody_, J1_pinv_upperbody_,  J2N1_upperbody_, J3N2_aug_upperbody_, J4N3_aug_upperbody_;
    Eigen::MatrixXd I3_upperbody_, I6_upperbody_, I15_upperbody_, I21_upperbody_;
    /////////////////////////////////////////////

    /////////////HQPIK//////////////////////////
    const unsigned int hierarchy_num_hqpik_ = 4;
    const unsigned int variable_size_hqpik_ = 21;
	const unsigned int constraint_size1_hqpik_ = 21;	//[lb <=	x	<= 	ub] form constraints
	const unsigned int constraint_size2_hqpik_[4] = {12, 15, 17, 21};	//[lb <=	Ax 	<=	ub] or [Ax = b]
	const unsigned int control_size_hqpik_[4] = {3, 14, 4, 4};		//1: upperbody, 2: head + hand, 3: upperarm, 4: shoulder

    double w1_hqpik_[4];
    double w2_hqpik_[4];
    double w3_hqpik_[4];
    double w4_hqpik_[4];
    double w5_hqpik_[4];
    double w6_hqpik_[4];
    
    Eigen::MatrixXd H_hqpik_[4], A_hqpik_[4];
    Eigen::MatrixXd J_hqpik_[4], J_temp_;
    Eigen::VectorXd g_hqpik_[4], u_dot_hqpik_[4], qpres_hqpik_, ub_hqpik_[4],lb_hqpik_[4], ubA_hqpik_[4], lbA_hqpik_[4];
    Eigen::VectorXd q_dot_hqpik_[4];

    int last_solved_hierarchy_num_;
    const double equality_condition_eps_ = 1e-8;
    const double damped_puedoinverse_eps_ = 1e-5;
    ///////////////////////////////////////////////////
    
    /////////////HQPIK2//////////////////////////
    const int hierarchy_num_hqpik2_ = 5;
    const int variable_size_hqpik2_ = 21;
	const int constraint_size1_hqpik2_ = 21;	//[lb <=	x	<= 	ub] form constraints
	const int constraint_size2_hqpik2_[5] = {12, 16, 19, 19, 23};	//[lb <=	Ax 	<=	ub] or [Ax = b]
	const int control_size_hqpik2_[5] = {4, 3, 12, 4, 4};		//1: head ori(2)+pos(2), 2: hand, 3: upper body ori, 4: upper arm ori(2) 5: shoulder ori(2)

    double w1_hqpik2_[5];
    double w2_hqpik2_[5];
    double w3_hqpik2_[5];
    double w4_hqpik2_[5];
    double w5_hqpik2_[5];
    double w6_hqpik2_[5];
    
    Eigen::MatrixXd H_hqpik2_[5], A_hqpik2_[5];
    Eigen::MatrixXd J_hqpik2_[5];
    Eigen::VectorXd g_hqpik2_[5], u_dot_hqpik2_[5], qpres_hqpik2_, ub_hqpik2_[5],lb_hqpik2_[5], ubA_hqpik2_[5], lbA_hqpik2_[5];
    Eigen::VectorXd q_dot_hqpik2_[5];
    /////////////////////////////////////////////////////

    ////////////QP RETARGETING//////////////////////////////////
    const int variable_size_retargeting_ = 6;
	const int constraint_size1_retargeting_ = 6;	//[lb <=	x	<= 	ub] form constraints
	const int constraint_size2_retargeting_[3] = {6, 6, 9};	//[lb <=	Ax 	<=	ub] from constraints
   	const int control_size_retargeting_[3] = {3, 3, 3};		//1: left hand, 2: right hand, 3: relative hand

    Eigen::Vector3d robot_still_pose_lhand_, robot_t_pose_lhand_, robot_forward_pose_lhand_, robot_still_pose_rhand_, robot_t_pose_rhand_, robot_forward_pose_rhand_;
    Eigen::Vector3d lhand_mapping_vector_, rhand_mapping_vector_, lhand_mapping_vector_pre_, rhand_mapping_vector_pre_, lhand_mapping_vector_dot_, rhand_mapping_vector_dot_;
    Eigen::MatrixXd lhand_master_ref_stack_, lhand_robot_ref_stack_, rhand_master_ref_stack_, rhand_robot_ref_stack_, lhand_master_ref_stack_pinverse_, rhand_master_ref_stack_pinverse_;
    
    Eigen::MatrixXd H_retargeting_lhand_,  A_retargeting_lhand_, H_retargeting_rhand_, A_retargeting_rhand_;
    Eigen::VectorXd qpres_retargeting_[3], g_retargeting_lhand_, g_retargeting_rhand_, ub_retargeting_, lb_retargeting_, ubA_retargeting_[3], lbA_retargeting_[3];
    Eigen::MatrixXd E1_, E2_, E3_, H_retargeting_, A_retargeting_[3];
    Eigen::VectorXd g_retargeting_, u1_, u2_, u3_;

    Eigen::Vector3d h_d_lhand_, h_d_rhand_, h_pre_lhand_, h_pre_rhand_, r_pre_lhand_, r_pre_rhand_;
    double w1_retargeting_, w2_retargeting_, w3_retargeting_, human_shoulder_width_; 
    const double control_gain_retargeting_ = 100;
    const double human_vel_min_ = -2;
    const double human_vel_max_ = 2;
    const double w_dot_min_ = -30;
    const double w_dot_max_ = 30;

    ////////////////////////////////////////////////////////////
    Eigen::VectorXd stepping_input;
    Eigen::VectorXd stepping_input_;

    /////////////MPC-MJ//////////////////////////
    Eigen::Vector3d x_hat_;
    Eigen::Vector3d y_hat_;
    Eigen::Vector3d x_hat_p_;
    Eigen::Vector3d y_hat_p_;

    Eigen::Vector3d x_hat_thread_;
    Eigen::Vector3d y_hat_thread_;
    Eigen::Vector3d x_hat_p_thread_;
    Eigen::Vector3d y_hat_p_thread_;

    Eigen::Vector3d x_hat_thread2_;
    Eigen::Vector3d y_hat_thread2_;
    Eigen::Vector3d x_hat_p_thread2_;
    Eigen::Vector3d y_hat_p_thread2_;

    Eigen::VectorXd MPC_input_x_;
    Eigen::VectorXd MPC_input_y_;
    Eigen::Matrix3d A_mpc_;
    Eigen::Vector3d B_mpc_;
    Eigen::Vector3d C_mpc_transpose_;
    Eigen::MatrixXd P_ps_mpc_; 
    Eigen::MatrixXd P_pu_mpc_;
    Eigen::MatrixXd P_vs_mpc_; 
    Eigen::MatrixXd P_vu_mpc_;
    Eigen::MatrixXd P_zs_mpc_; 
    Eigen::MatrixXd P_zu_mpc_;
    Eigen::MatrixXd Q_prime_;
    Eigen::MatrixXd Q_mpc_;

    Eigen::VectorXd x_com_pos_recur_;
    Eigen::VectorXd x_com_vel_recur_;
    Eigen::VectorXd x_zmp_recur_;
    Eigen::VectorXd y_com_pos_recur_;
    Eigen::VectorXd y_com_vel_recur_;
    Eigen::VectorXd y_zmp_recur_;
    
    Eigen::VectorXd x_cp_recur_;
    Eigen::VectorXd y_cp_recur_;

    Eigen::MatrixXd H_cp_control_x_;
    Eigen::MatrixXd H_cp_control_y_;
    Eigen::MatrixXd H_change_regul_;
    Eigen::MatrixXd H_damping_Nsize_x_, H_damping_Nsize_y_;
    Eigen::MatrixXd H_damping_x_, H_damping_y_;
    Eigen::MatrixXd F_cp_;
    Eigen::MatrixXd diff_matrix_;
    Eigen::MatrixXd F_zmp_;
    Eigen::MatrixXd H_cpmpc_;
    Eigen::MatrixXd H_cpStepping_mpc_;
    Eigen::MatrixXd weighting_cp_;
    Eigen::MatrixXd weighting_zmp_diff_;
    Eigen::VectorXd e1_cpmpc_;
    Eigen::VectorXd cpmpc_input_x_;
    Eigen::VectorXd cpmpc_deszmp_x_;
    Eigen::VectorXd cpmpc_input_y_;
    Eigen::VectorXd cpmpc_deszmp_y_;
    double force_temp_ = 0, theta_temp_ = 0;
    double ssp_flag_ = 0;
    double w_ux_temp_ = 0, w_uy_temp_ = 0, w_time_temp_ = 0, w_bx_temp_ = 0, w_by_temp_ = 0;
    Eigen::Vector2d dsp_scaler_dot_;
    Eigen::Vector2d dsp_scaler_;
    double dsp_scaler_x_dot_ = 0;
    double dsp_scaler_x_ = 0;
    double dsp_scaler_y_dot_ = 0;
    double dsp_scaler_y_ = 0;  
    double dsp_time_reducer_ = 0;
    double dsp_time_reducer_fixed_ = 0;
    double del_F_x_next_ = 0;
    double del_F_y_next_ = 0;
    double del_F_x_ = 0;
    double del_F_x_prev_ = 0;
    double del_F_x_LPF_ = 0;
    double del_F_y_ = 0;
    double del_F_x_thread_ = 0;
    double del_F_y_thread_ = 0;
    double del_F_x_next_thread_ = 0;
    double del_F_y_next_thread_ = 0;
    
    double cpmpc_des_zmp_x_thread_ = 0;
    double cpmpc_des_zmp_x_thread2_ = 0;

    double cpmpc_des_zmp_y_thread_ = 0;    
    double cpmpc_des_zmp_y_thread2_ = 0;
    
    double cp_des_zmp_x_ = 0;
    double cp_des_zmp_x_prev_ = 0;
    double des_zmp_x_stepchange_ = 0;
    double des_zmp_x_prev_stepchange_ = 0;

    double cp_des_zmp_y_ = 0;
    double cp_des_zmp_y_prev_ = 0;
    double des_zmp_y_stepchange_ = 0;
    double des_zmp_y_prev_stepchange_ = 0;

    Eigen::Vector2d opt_F_;

    Eigen::Vector2d des_zmp_interpol_;

    // New CPMPC
    Eigen::MatrixXd F_cp_new_; 
    Eigen::MatrixXd F_cmp_new_;
    Eigen::MatrixXd diff_matrix_new_;
    Eigen::MatrixXd integral_matrix_;
    Eigen::MatrixXd e1_cpmpc_new_;
    Eigen::MatrixXd weighting_cp_new_x_;
    Eigen::MatrixXd weighting_cp_new_y_;
    Eigen::MatrixXd weighting_cmp_diff_new_;
    Eigen::MatrixXd H_cpmpc_new_x_;
    Eigen::MatrixXd H_cpmpc_new_y_;
    Eigen::MatrixXd H_cpStepping_mpc_new_x_;
    Eigen::MatrixXd H_cpStepping_mpc_new_y_;

    Eigen::MatrixXd damping_integral_mat_;
    Eigen::MatrixXd cam_damping_mat_;

    Eigen::MatrixXd desZmp_sel_;
    Eigen::MatrixXd Tau_sel_;
    Eigen::MatrixXd weighting_zmp_regul_;
    Eigen::MatrixXd weighting_tau_regul_;
    Eigen::MatrixXd weighting_tau_damping_;
    Eigen::MatrixXd weighting_tau_damping_x_;
    Eigen::MatrixXd weighting_tau_damping_y_;

    Eigen::VectorXd cpmpc_deszmp_x_new_;
    Eigen::VectorXd cpmpc_deszmp_y_new_;
    Eigen::VectorXd cpmpc_destau_x_new_;
    Eigen::VectorXd cpmpc_destau_y_new_;
    Eigen::VectorXd cpmpc_output_x_new_;
    Eigen::VectorXd cpmpc_output_y_new_;

    Eigen::VectorXd cpmpc_input_x_new_;
    Eigen::VectorXd cpmpc_input_y_new_;

    Eigen::VectorXd Z_x_ref_cpmpc_only_;
    Eigen::VectorXd Z_y_ref_cpmpc_only_;

    double cp_eos_x_mpc_, cp_eos_y_mpc_;
    double cp_eos_x_cpmpc_, cp_eos_y_cpmpc_;

    double des_tau_x_thread_ = 0.0;
    double des_tau_x_ = 0.0;
    double des_tau_y_thread_ = 0.0;
    double des_tau_y_ = 0.0;
    //
    // Thread 3
    Eigen::VectorXd U_x_mpc_;
    Eigen::VectorXd U_y_mpc_; 
    // Thread 2
    
    Eigen::Vector2d del_F_;
    Eigen::Vector3d x_hat_r_;
    Eigen::Vector3d x_hat_r_sc_;
    Eigen::Vector3d x_hat_r_p_sc_;
    Eigen::Vector3d y_hat_r_;
    Eigen::Vector3d y_hat_r_sc_;    
    Eigen::Vector3d y_hat_r_p_sc_;
    Eigen::Vector3d x_hat_r_p_;
    Eigen::Vector3d y_hat_r_p_;
    Eigen::Vector3d x_mpc_i_;
    Eigen::Vector3d y_mpc_i_; 
    Eigen::Vector3d z_mpc_i_; 
    double w_mpc_i_;
    double w_dot_mpc_i_;
    double lambda_mpc_i_;
    Eigen::Vector3d x_diff_;
    Eigen::Vector3d y_diff_;
    Eigen::Vector3d z_diff_;
    double w_diff_;
    double w_dot_diff_;
    double lambda_diff_;
    Eigen::Vector2d cpmpc_diff_;
    Eigen::Vector2d cpStepping_diff_;

    int wieber_interpol_cnt_x_ = 0;
    int wieber_interpol_cnt_y_ = 0;
    int cpmpc_interpol_cnt_x_ = 0;
    int cpmpc_interpol_cnt_y_ = 0;
    bool mpc_x_update_ {false}, mpc_y_update_ {false}, mpc_z_update_ {false}, mpc_qcqp_update_ {false};
    bool mpc_cp_update_{false}, mpc_dcm_xyz_update_{false};
    bool cpmpc_x_update_ {false}, cpmpc_y_update_ {false} ;
    double W1_mpc_ = 0, W2_mpc_ = 0, W3_mpc_ = 0;
    int alpha_step_mpc_ = 0;
    int alpha_step_mpc_thread_ = 0;

    Eigen::VectorXd alpha_mpc_;
    Eigen::VectorXd F_diff_mpc_x_;
    Eigen::VectorXd F_diff_mpc_y_;
    double alpha_lpf_ = 0;
    double temp_pos_y_ = 0;
    double F0_F1_mpc_x_ = 0, F1_F2_mpc_x_ = 0, F2_F3_mpc_x_ = 0, F0_F1_mpc_y_ = 0, F1_F2_mpc_y_ = 0, F2_F3_mpc_y_ = 0;
    Eigen::Vector2d foot_diff_current2next_;
    Eigen::Vector2d foot_diff_next2Nnext_;
    Eigen::Vector2d foot_diff_current2next_thread_;
    Eigen::Vector2d foot_diff_next2Nnext_thread_;
    Eigen::Vector2d foot_diff_current2next_mpc_;
    Eigen::Vector2d foot_diff_next2Nnext_mpc_;

    // Eigen::Vector2d foot_diff_currentTonext_;
    Eigen::Vector6d target_swing_foot;
    Eigen::Vector6d desired_swing_foot;
    Eigen::Vector6d desired_swing_foot_LPF_;
    Eigen::Vector2d del_F_LPF_;
    Eigen::Vector6d fixed_swing_foot;
    Eigen::Vector6d fixed_swing_foot_del_F_;
    Eigen::MatrixXd modified_del_zmp_; 
    Eigen::MatrixXd m_del_zmp_x;
    Eigen::MatrixXd m_del_zmp_y;
    int first_current_step_flag_ = 0;
    int first_current_step_number_ = 0;
    
    double zmp_modif_time_margin_ = 0; 
    ////////////////////////////////////////////////////////////
    
    /////////////CAM-HQP//////////////////////////
    const int hierarchy_num_camhqp_ = 2;
    const int variable_size_camhqp_ = 10; // original number -> 6 (DG) // IROS -> 8 (MJ)
    const int constraint_size1_camhqp_ = 10; //[lb <= x <=	ub] form constraints // original number -> 6 (DG)  // IROS -> 8 (MJ)
    //const int constraint_size2_camhqp_[2] = {0, 3};	//[lb <=	Ax 	<=	ub] or [Ax = b]/  
    const int constraint_size2_camhqp_[2] = {0, 2};	//[lb <=	Ax 	<=	ub] or [Ax = b] 
    //const int control_size_camhqp_[2] = {3, 8}; //1: CAM control, 2: init pose // original number -> 6 (DG)
    const int control_size_camhqp_[2] = {2, 10}; //1: CAM control, 2: init pose // original number -> 6 (DG) // IROS -> 8 (MJ)  

    double w1_camhqp_[2];
    double w2_camhqp_[2]; 
    
    Eigen::MatrixXd H_camhqp_[2], A_camhqp_[2];
    Eigen::MatrixXd J_camhqp_[2];
    Eigen::VectorXd g_camhqp_[2], u_dot_camhqp_[2], qpres_camhqp_, ub_camhqp_[2],lb_camhqp_[2], ubA_camhqp_[2], lbA_camhqp_[2];
    Eigen::VectorXd q_dot_camhqp_[2];

    int control_joint_idx_camhqp_[10]; // original number -> 6 (DG) // IROS -> 8 (MJ)
    int last_solved_hierarchy_num_camhqp_;
    unsigned int torque_flag_x = 0, torque_flag_y = 0; 
    ///////////////////////////////////////////////////

    /////////////////////////MOMENTUM OBSERVER////////////////////////////////////////////////
    Eigen::VectorVQd mob_integral_;
    Eigen::VectorVQd mob_residual_;
    ////////////////////////////////////////////////////////////////////////////////////////////

    //fallDetection variables
    Eigen::VectorQd fall_init_q_;
    double fall_start_time_;
    int foot_lift_count_;
    int foot_landing_count_;
    ///////////////////////////////////////////////////////////////

private:
    Eigen::VectorQd ControlVal_;
        
    void initWalkingParameter();

    //Arm controller
    Eigen::VectorXd joint_limit_l_;
    Eigen::VectorXd joint_limit_h_;
    Eigen::VectorXd joint_vel_limit_l_;
    Eigen::VectorXd joint_vel_limit_h_;

    Eigen::Vector3d left_x_traj_pre_;
    Eigen::Vector3d right_x_traj_pre_;
    Eigen::Matrix3d left_rotm_pre_;
    Eigen::Matrix3d right_rotm_pre_;

    Eigen::VectorQVQd q_virtual_clik_;
    Eigen::Vector8d integral;

    bool first_loop_larm_;
    bool first_loop_rarm_;
    bool first_loop_upperbody_;
    bool first_loop_hqpik_;
    bool first_loop_hqpik2_;
    bool first_loop_qp_retargeting_;
    bool first_loop_camhqp_;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////MJ CustomCuntroller//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
    //////////////////////////////// Myeong-Ju
    void circling_motion();
    void computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);
    void Joint_gain_set_MJ();
    void updateInitialState();
    void updateNextStepTime();
    void parameterSetting();
    void getRobotState();
    void calculateFootStepTotal();
    void calculateFootStepTotal_MJ();
    void supportToFloatPattern();
    void floatToSupportFootstep();
    void GravityCalculate_MJ();
    
    void getZmpTrajectory();
    void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
    void onestepZmp_wo_offset(unsigned int current_step_number, double t_total_zmp, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py, Eigen::VectorXd& temp_px_wo_offset, Eigen::VectorXd& temp_py_wo_offset);
    void getComTrajectory();
    void getFootTrajectory();
    void getFootTrajectory_stepping();
    void getPelvTrajectory();
    void previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
    Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD);  
    void preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C);
    void addZmpOffset();
    void hip_compensator();
    void Compliant_control(Eigen::Vector12d desired_leg_q);
    
    void SC_err_compen(double x_des, double y_des);

    void CP_compen_MJ();
    void CP_compen_MJ_FT();
    void CLIPM_ZMP_compen_MJ(double XZMP_ref, double YZMP_ref);
    double U_ZMP_y_ssp = 0;
    double U_ZMP_y_ssp_LPF = 0;
    double U_ZMP_x_ssp = 0;
    double U_ZMP_x_ssp_LPF = 0;
    double damping_x = 0;
    double damping_y = 0;
    Eigen::Vector2d Tau_R;
    Eigen::Vector2d Tau_L;

    Eigen::VectorQd Tau_CP;

    Eigen::Vector12d pre_motor_q_leg_;
    Eigen::Vector12d current_motor_q_leg_;
    Eigen::Vector12d d_hat_b;
    Eigen::Vector12d DOB_IK_output_b_;
    Eigen::Vector12d DOB_IK_output_;
    Eigen::VectorQd ref_q_;
    Eigen::VectorQd ref_q_fast_;
    Eigen::VectorQd Kp;
    Eigen::VectorQd Kd;
    Eigen::VectorQd desired_q_not_compensated_;
    
    Eigen::VectorQd q_prev_MJ_;

    Eigen::Vector12d q_des_;
    
    Eigen::Isometry3d pelv_trajectory_support_; //local frame
    
    Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
    Eigen::Isometry3d lfoot_trajectory_support_;
    Eigen::Vector3d rfoot_trajectory_euler_support_;
    Eigen::Vector3d lfoot_trajectory_euler_support_;

    Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame
    Eigen::Isometry3d rfoot_trajectory_float_;
    Eigen::Isometry3d lfoot_trajectory_float_;

    Eigen::Isometry3d lfoot_trajectory_float_fast_;
    Eigen::Isometry3d lfoot_trajectory_float_slow_;

    Eigen::Isometry3d rfoot_trajectory_float_fast_;
    Eigen::Isometry3d rfoot_trajectory_float_slow_;

    Eigen::Vector3d pelv_support_euler_init_;
    Eigen::Vector3d lfoot_support_euler_init_;
    Eigen::Vector3d rfoot_support_euler_init_;

    Eigen::Vector2d cam_mpc_init_;
    Eigen::Vector2d cam_thread_;
    Eigen::Vector2d del_cmp;
    Eigen::Vector3d del_tau_;
    Eigen::Vector3d del_ang_momentum_;
    Eigen::Vector3d del_ang_momentum_prev_;

    Eigen::Vector3d del_ang_momentum_slow_;
    Eigen::Vector3d del_ang_momentum_fast_;

    Eigen::VectorQd del_cmm_q_;
    unsigned int cmp_control_mode = 0;

    double des_zmp_ssp_mpc_x_ = 0;
    double des_zmp_ssp_mpc_y_ = 0;

    Eigen::Isometry3d pelv_support_start_;
    Eigen::Isometry3d pelv_support_init_;
    Eigen::Vector2d del_zmp;
    double del_lambda0_;
    double del_lambda1_;
    Eigen::Vector2d cp_desired_;
    Eigen::Vector3d dcm_desired_;
    Eigen::Vector2d cp_measured_;
    Eigen::Vector3d dcm_measured_;
    Eigen::Vector3d dcm_measured_thread_;
    Eigen::Vector3d dcm_measured_mpc_;

    Eigen::Vector3d com_measured_;
    Eigen::Vector3d com_measured_thread_;
    Eigen::Vector3d com_measured_mpc_;

    Eigen::Vector3d com_dot_measured_;
    Eigen::Vector3d com_dot_measured_thread_;
    Eigen::Vector3d com_dot_measured_mpc_;

    double lambda_desired_;
    double lambda_measured_;
    double w_desired_;
    double w_dot_desired_;
    double w_dot_measured_;
    double w_measured_;
    Eigen::Vector3d com_imu_acc_;
    Eigen::Vector3d com_imu_acc_lpf_;
    Eigen::Vector2d cp_measured_LPF;
    Eigen::Vector2d cp_measured_thread_;
    Eigen::Vector2d cp_measured_mpc_;
    Eigen::Vector3d com_support_init_;
    Eigen::Vector3d com_float_init_;
    Eigen::Vector3d com_float_current_;
    Eigen::Vector3d com_support_current_;
    Eigen::Vector3d com_support_current_dot_;
    Eigen::Vector3d com_support_current_ddot_;
    Eigen::Vector3d com_support_current_LPF;
    Eigen::Vector3d com_float_current_LPF;
    Eigen::Vector3d com_support_current_prev;
    Eigen::Vector3d com_support_cp_;

    Eigen::Vector3d com_float_current_dot;
    Eigen::Vector3d com_float_current_dot_prev;
    Eigen::Vector3d com_float_current_dot_LPF;
    Eigen::Vector3d com_support_current_dot_LPF;

    Eigen::Vector3d com_float_current_ddot;
    Eigen::Vector3d com_float_current_ddot_prev;
    Eigen::Vector3d com_float_current_ddot_LPF;
    Eigen::Vector3d com_support_current_ddot_LPF;

    Eigen::Vector3d pelv_rpy_current_mj_;
    Eigen::Vector3d rfoot_rpy_current_;
    Eigen::Vector3d lfoot_rpy_current_;
    Eigen::Isometry3d pelv_yaw_rot_current_from_global_mj_;
    Eigen::Isometry3d rfoot_roll_rot_;
    Eigen::Isometry3d lfoot_roll_rot_;
    Eigen::Isometry3d rfoot_pitch_rot_;
    Eigen::Isometry3d lfoot_pitch_rot_;

    Eigen::Isometry3d pelv_float_current_;
    Eigen::Isometry3d lfoot_float_current_;
    Eigen::Isometry3d rfoot_float_current_;
    Eigen::Isometry3d pelv_float_init_;
    Eigen::Isometry3d lfoot_float_init_;
    Eigen::Isometry3d rfoot_float_init_;
    double wn = 0;

    Eigen::Vector2d sc_err_before;
    Eigen::Vector2d sc_err_after;
    Eigen::Vector2d SC_com;
    Eigen::Vector2d sc_err;

    Eigen::Vector12d sc_joint_before;
    Eigen::Vector12d sc_joint_after;
    Eigen::Vector12d SC_joint;
    Eigen::Vector12d sc_joint_err;

    double walking_end_flag = 0;
    
    Eigen::Isometry3d supportfoot_float_current_; 

    Eigen::Isometry3d pelv_support_current_;
    Eigen::Isometry3d lfoot_support_current_;
    Eigen::Isometry3d rfoot_support_current_;

    Eigen::Isometry3d lfoot_support_init_;
    Eigen::Isometry3d rfoot_support_init_;
    
    Eigen::Vector6d supportfoot_support_init_offset_;
    Eigen::Vector6d supportfoot_float_init_;
    Eigen::Vector6d supportfoot_support_init_;
    Eigen::Vector6d swingfoot_float_init_;
    Eigen::Vector6d swingfoot_support_init_;

    Eigen::MatrixXd ref_zmp_mj_;
    Eigen::MatrixXd ref_zmp_mj_wo_offset_;
    Eigen::MatrixXd ref_zmp_wo_offset_mpc_;
    Eigen::MatrixXd ref_zmp_wo_offset_thread_;
    Eigen::MatrixXd ref_zmp_e_max_;
    Eigen::MatrixXd ref_zmp_e_min_;

    Eigen::MatrixXd ref_zmp_mpc_;
    Eigen::MatrixXd ref_zmp_thread_;

    Eigen::Vector3d xs_mj_;
    Eigen::Vector3d ys_mj_;
    Eigen::Vector3d xd_mj_;
    Eigen::Vector3d yd_mj_; 
    Eigen::Vector3d preview_x_mj, preview_y_mj, preview_x_b_mj, preview_y_b_mj;

    Eigen::MatrixXd Gi_mj_;
    Eigen::MatrixXd Gx_mj_;
    Eigen::VectorXd Gd_mj_;
    Eigen::MatrixXd A_mj_;
    Eigen::VectorXd B_mj_;
    Eigen::MatrixXd C_mj_;

    Eigen::VectorQd Gravity_MJ_fast_;
    Eigen::VectorQd Gravity_MJ_;
    Eigen::VectorQd Gravity_DSP_;
    Eigen::VectorQd Gravity_DSP_last_;
    Eigen::VectorQd Gravity_SSP_;
    Eigen::VectorQd Gravity_SSP_last_;
    Eigen::VectorQd q_dot_LPF_MJ;

    Eigen::Vector6d r_ft_mj_;
    Eigen::Vector6d l_ft_mj_;
    Eigen::Vector2d zmp_measured_mj_;
    Eigen::Vector2d zmp_err_;
    Eigen::Vector2d zmp_measured_LPF_;

    double P_angle_i = 0;
    double P_angle = 0;
    double P_angle_input_dot = 0;
    double P_angle_input = 0;
    double R_angle = 0;
    double R_angle_input_dot = 0;
    double R_angle_input = 0;
    double aa = 0; 

    double del_t = 0.0005;
    double xi_mj_;
    double yi_mj_;
    double zc_mj_;

    double ZMP_X_REF_;
    double ZMP_Y_REF_;
    double ZMP_Y_REF_alpha_ = 0;

    double t_last_;
    double t_start_;
    double t_start_real_;
    double t_temp_;  
    double t_rest_init_;
    double t_rest_last_;
    double t_double1_;
    double t_double2_;
    double t_total_;
    double t_total_const_;
    double t_total_thread_;
    double t_rest_init_thread_;
    double t_rest_last_thread_;
    double t_total_mpc_;
    double t_rest_init_mpc_;
    double t_rest_last_mpc_;
    double foot_height_;
    int total_step_num_;
    int total_step_num_mpc_;
    int total_step_num_thread_;
    int current_step_num_;
    int current_step_num_mpc_;
    int current_step_num_thread_;
    int current_step_num_thread2_;
    int current_step_num_z_mpc_;
    int current_step_num_z_thread_;
    int current_step_num_z_thread2_;
    int current_step_num_mpc_prev_;   
    int current_step_num_mpc_new_prev_;   
    double step_length_x_;
    double step_length_y_;
    double target_theta_;
    double target_x_;
    double target_y_;
    double target_z_;
    double com_height_;
    int is_right_foot_swing_;

    Eigen::Isometry3d lfoot_support_current_thread_;
    Eigen::Isometry3d rfoot_support_current_thread_;
    Eigen::Isometry3d lfoot_support_current_mpc_;
    Eigen::Isometry3d rfoot_support_current_mpc_;

    double zmp_start_time_mj_;
    double zmp_start_time_mj_mpc_;
    double zmp_start_time_mj_thread_;
    double UX_mj_, UY_mj_; 
    Eigen::Vector3d com_desired_;
    Eigen::MatrixXd foot_step_;
    Eigen::MatrixXd foot_step_support_frame_;
    Eigen::MatrixXd foot_step_support_frame_mpc_;
    Eigen::MatrixXd foot_step_support_frame_thread_;
    Eigen::MatrixXd foot_step_support_frame_offset_;
    Eigen::MatrixXd foot_step_support_frame_offset_mpc_;
    Eigen::MatrixXd foot_step_support_frame_offset_mpc_p_;
    Eigen::MatrixXd foot_step_support_frame_offset_thread_;
    
    // Com damping control - ZMP tracking controller
    Eigen::MatrixXd A_y_ssp;
    Eigen::MatrixXd B_y_ssp;
    Eigen::MatrixXd Ad_y_ssp;
    Eigen::MatrixXd Bd_y_ssp;
    Eigen::MatrixXd C_y_ssp;
    Eigen::MatrixXd D_y_ssp;
    Eigen::MatrixXd K_y_ssp;
    Eigen::MatrixXd Y_y_ssp;
    Eigen::Vector2d X_y_ssp;
    
    Eigen::MatrixXd A_x_ssp;
    Eigen::MatrixXd B_x_ssp;

    Eigen::MatrixXd Ad_x_ssp;
    Eigen::MatrixXd Bd_x_ssp;
    Eigen::MatrixXd C_x_ssp;
    Eigen::MatrixXd D_x_ssp;
    Eigen::MatrixXd K_x_ssp;
    Eigen::MatrixXd Y_x_ssp;
    Eigen::Vector2d X_x_ssp;
    Eigen::MatrixXd ff_gain_y_ssp;
    Eigen::MatrixXd ff_gain_x_ssp;
    //
    Eigen::VectorQd contact_torque_MJ;
    Eigen::VectorQd Initial_ref_q_;
    Eigen::VectorQd Initial_ref_upper_q_;
    Eigen::VectorQd Initial_current_q_;
    Eigen::VectorQd Initial_ref_q_walk_;
    bool walking_enable_ ;

    //pedal_
    ros::NodeHandle nh;
    ros::Subscriber pedal_command;
    void PedalCommandCallback(const tocabi_msgs::WalkingCommandConstPtr &msg);
    Eigen::Vector4d joystick_input;
    Eigen::Vector4d joystick_input_;

    //// joystick&pedal Footstep
    void updateInitialStateJoy();
    void calculateFootStepTotal_MJoy();
    void calculateFootStepTotal_MJoy_End();
    void updateNextStepTimeJoy();
    int joy_index_ = 0;
    Eigen::MatrixXd foot_step_joy_temp_;
    bool joy_enable_ = false;
    bool joy_input_enable_ = false;

    Eigen::VectorQd q_mj;
    Eigen::VectorQd q_mj_prev;

    //////////////////////////////// Econom2 function
    void writeDataTxt();
    void getComTrajectory_Z_e();
    void comRefGenerator_Z_e();
    void comRefGenerator_Z_e_thread();
    void comGenerator_MPC_Z_e(double MPC_z_freq, double T_z_, double preview_window_z_, int MPC_synchro_hz_);
    void footTerrainInt(Eigen::MatrixXd& foot_terrain_int);
    void footTerrainInt_thread(Eigen::MatrixXd& foot_terrain_int);
    void comGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void comGenerator_thread(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepComz(unsigned int current_step_number, Eigen::VectorXd& temp_cz);
    void onestepComz_thread(unsigned int current_step_number, Eigen::VectorXd& temp_cz);
    void footPrevGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void footPrevGenerator_thread(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepFootPrev(unsigned int current_step_number, Eigen::VectorXd& temp_lfx, Eigen::VectorXd& temp_lfy, Eigen::VectorXd& temp_lfz,
                                                           Eigen::VectorXd& temp_rfx, Eigen::VectorXd& temp_rfy, Eigen::VectorXd& temp_rfz);
    void onestepFootPrev_thread(unsigned int current_step_number, Eigen::VectorXd& temp_lfx, Eigen::VectorXd& temp_lfy, Eigen::VectorXd& temp_lfz,
                                                                  Eigen::VectorXd& temp_rfx, Eigen::VectorXd& temp_rfy, Eigen::VectorXd& temp_rfz);
    //////////////////////////////// Econom2 variables
    Eigen::Vector2d zmp_desired_;
    Eigen::Vector2d zmp_max_;
    Eigen::Vector2d zmp_min_;

    int param_qcqp_int_;
    double param_Qcpz_;
    double param_Qcvz_;
    double param_Qcaz_;
    double param_Czref_calc_;
    double param_target_x_;
    double param_step_x_;
    double ext_force_;
    double ext_theta_;
    double param_Q_dcm_x_;
    double param_R_dcm_x_;
    double param_R_f_x_;
    double param_R_df_x_;
    double param_Q_dcm_y_;
    double param_R_dcm_y_;
    double param_R_f_y_;
    double param_R_df_y_;
    double param_Q_dcm_z_;
    double param_R_dcm_z_;
    double param_ext_force_time_;
    int param_ext_step_num_;
    double econom2_calc;
    double zmp_x_max = 0.13;
    double zmp_x_min = 0.07;
    double zmp_y_max = 0.07;
    double zmp_y_min = 0.07;
    double height_diff = 0.0;
    double angle_diff = 0.0;
    Eigen::MatrixXd ref_com_z_e_;
    Eigen::MatrixXd ref_com_z_mpc_;
    Eigen::MatrixXd ref_com_z_thread_;
    Eigen::MatrixXd lfoot_prev_e_;
    Eigen::MatrixXd rfoot_prev_e_;
    Eigen::MatrixXd lfoot_prev_mpc_;
    Eigen::MatrixXd rfoot_prev_mpc_;
    Eigen::MatrixXd lfoot_prev_thread_;
    Eigen::MatrixXd rfoot_prev_thread_;
    Eigen::MatrixXd foot_terrain_int_;
    
    ///z mpc
    Eigen::Vector3d MPC_z_;
    Eigen::Vector3d MPC_z_p_;
    Eigen::Vector3d MPC_z_r_;
    Eigen::Vector3d MPC_z_r_sc_;
    Eigen::Vector3d MPC_z_r_p_;
    Eigen::Vector3d MPC_z_r_p_sc_;
    Eigen::Vector3d MPC_z_thread_;
    Eigen::Vector3d MPC_z_thread2_;
    Eigen::Vector3d MPC_z_p_thread_;
    Eigen::Vector3d MPC_z_p_thread2_;
    Eigen::Vector3d MPC_z_r_thread_;
    Eigen::Vector3d MPC_z_r_p_thread_;
    Eigen::Vector3d MPC_z_mpc_;
    Eigen::Vector3d MPC_z_p_mpc_;
    Eigen::Vector3d MPC_z_r_mpc_;
    Eigen::Vector3d MPC_z_r_p_mpc_;
    Eigen::MatrixXd Az_mpc_;
    Eigen::MatrixXd Bz_mpc_;
    Eigen::MatrixXd Czp_mpc_;
    Eigen::MatrixXd Czv_mpc_;
    Eigen::MatrixXd Cza_mpc_;
    Eigen::MatrixXd Czmp_mpc_;
    Eigen::MatrixXd Qzp_mpc_;
    Eigen::MatrixXd Qzv_mpc_;
    Eigen::MatrixXd Qza_mpc_;
    Eigen::MatrixXd Rz_mpc_;
    Eigen::MatrixXd Pzps_mpc_;
    Eigen::MatrixXd Pzvs_mpc_;
    Eigen::MatrixXd Pzas_mpc_;
    Eigen::MatrixXd Pzmps_mpc_e_;
    Eigen::MatrixXd Pzpu_mpc_;
    Eigen::MatrixXd Pzvu_mpc_;
    Eigen::MatrixXd Pzau_mpc_;
    Eigen::MatrixXd Pzmpu_mpc_e_;
    Eigen::VectorXd z_com_pos_recur_;
    Eigen::VectorXd z_com_vel_recur_;
    Eigen::VectorXd z_com_acc_recur_;
    Eigen::MatrixXd Qcalcz_mpc_;
    Eigen::MatrixXd gcalcz_mpc_;
    CQuadraticProgram QP_mpc_z_;
    Eigen::VectorXd MPC_input_z_;
    Eigen::VectorXd U_z_mpc_;
    double com_start_tick_e_;
    double com_start_tick_e_mpc_;
    double com_start_tick_e_thread_;
    int Z_e_interpol_cnt_ = 0;

    double MPC_w_;
    double MPC_w_p_;
    double MPC_w_r_;
    double MPC_w_r_sc_;
    double MPC_w_r_p_;
    double MPC_w_r_p_sc_;
    double MPC_w_thread_;
    double MPC_w_thread2_;
    double MPC_w_p_thread_;
    double MPC_w_p_thread2_;
    double MPC_w_r_thread_;
    double MPC_w_r_p_thread_;
    double MPC_w_mpc_;
    double MPC_w_p_mpc_;
    double MPC_w_r_mpc_;
    double MPC_w_r_p_mpc_;

    double MPC_w_dot_;
    double MPC_w_dot_p_;
    double MPC_w_dot_r_;
    double MPC_w_dot_r_sc_;
    double MPC_w_dot_r_p_;
    double MPC_w_dot_r_p_sc_;
    double MPC_w_dot_thread_;
    double MPC_w_dot_thread2_;
    double MPC_w_dot_p_thread_;
    double MPC_w_dot_p_thread2_;
    double MPC_w_dot_r_thread_;
    double MPC_w_dot_r_p_thread_;
    double MPC_w_dot_mpc_;
    double MPC_w_dot_p_mpc_;
    double MPC_w_dot_r_mpc_;
    double MPC_w_dot_r_p_mpc_;

    double MPC_lambda_;
    double MPC_lambda_p_;
    double MPC_lambda_r_;
    double MPC_lambda_r_sc_;
    double MPC_lambda_r_p_;
    double MPC_lambda_r_p_sc_;
    double MPC_lambda_thread_;
    double MPC_lambda_thread2_;
    double MPC_lambda_p_thread_;
    double MPC_lambda_p_thread2_;
    double MPC_lambda_r_thread_;
    double MPC_lambda_r_p_thread_;
    double MPC_lambda_mpc_;
    double MPC_lambda_p_mpc_;
    double MPC_lambda_r_mpc_;
    double MPC_lambda_r_p_mpc_;

    double MPC_wf_;
    int qcqp_int;

    void onestepZmp_e(unsigned int current_step_number, Eigen::VectorXd& temp_px_max, Eigen::VectorXd& temp_py_max, Eigen::VectorXd& temp_px_min, Eigen::VectorXd& temp_py_min);
    //3D DCM

    //Van RAL QCQP
    void comGenerator_MPC_qcqp(double mpc_qcqp_freq, double dt_qcqp_, double preview_window_qcqp_, int MPC_synchro_hz_);
    void comGenerator_MPC_ding(double mpc_qcqp_freq, double dt_qcqp_, double preview_window_qcqp_, int MPC_synchro_hz_);
    void dcmcontroller_MPC_e(double mpc_freq, double preview_window);
    void dcmcontroller_MPC_qcqp_e(double mpc_freq, double preview_window);
    void dcmcontroller_MPC_foot_e(double mpc_freq, double preview_window);
    void dcmcontroller_MPC_foot_e2(double mpc_freq, double preview_window);
    void cpcontroller_MPC_foot_e(double mpc_freq, double preview_window);
    void getComFootTrajectory_mpc_qcqp();
    void footRefGenerator();
    void footGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepFoot(unsigned int current_step_number, Eigen::VectorXd& temp_fx, Eigen::VectorXd& temp_fy, Eigen::VectorXd& temp_fz, int foot_int);

    int run_planner_ = 1;
    CQuadraticProgram QCQP_mpc_;
    CQuadraticProgram QCQP_mpc_temp_;
    CQuadraticProgram CP_mpc_;
    CQuadraticProgram DCM_mpc_t_;
    CQuadraticProgram DCM_qcqp_mpc_t_;

    //Matrix
    Eigen::MatrixXd A_qcqp_;
    Eigen::MatrixXd calc_A_qcqp_;
    Eigen::MatrixXd A_dcm_;
    Eigen::MatrixXd A_dcm_c_;
    Eigen::MatrixXd A_dcm_qcqp_;
    Eigen::MatrixXd A_cp_;
    Eigen::MatrixXd B_qcqp_;
    Eigen::MatrixXd calc_B_qcqp_;
    Eigen::MatrixXd B_dcm_;
    Eigen::MatrixXd B_dcm_c_;
    Eigen::MatrixXd B_dcm_qcqp_;
    Eigen::MatrixXd B_cp_;
    Eigen::MatrixXd C_dcm_;
    Eigen::MatrixXd C_dcm_dcm_;
    Eigen::MatrixXd C_dcm_c_;
    Eigen::MatrixXd C_dcm_cd_;
    Eigen::MatrixXd C_dcm_qcqp_;
    Eigen::MatrixXd C_dcm_dcm_qcqp_;
    Eigen::MatrixXd C_dcm_c_qcqp_;
    Eigen::MatrixXd C_dcm_cd_qcqp_;
    Eigen::MatrixXd Czmp_qcqp_;
    Eigen::MatrixXd Cp_qcqp_;
    Eigen::MatrixXd Cv_qcqp_;
    Eigen::MatrixXd Ca_qcqp_;
    Eigen::MatrixXd Pzmps_qcqp_;
    Eigen::MatrixXd F_psi_dcm_;
    Eigen::MatrixXd F_psi_dcm_dcm_;
    Eigen::MatrixXd F_psi_dcm_c_;
    Eigen::MatrixXd F_psi_dcm_cd_;
    Eigen::MatrixXd F_psi_dcm_qcqp_;
    Eigen::MatrixXd F_psi_cp_;
    Eigen::MatrixXd Pps_qcqp_;
    Eigen::MatrixXd Pvs_qcqp_;
    Eigen::MatrixXd Pas_qcqp_;
    Eigen::MatrixXd Pzmpu_qcqp_;
    Eigen::MatrixXd F_p_dcm_;
    Eigen::MatrixXd F_p_dcm_dcm_;
    Eigen::MatrixXd F_p_dcm_c_;
    Eigen::MatrixXd F_p_dcm_cd_;
    Eigen::MatrixXd F_p_dcm_qcqp_;
    Eigen::MatrixXd F_p_cp_;
    Eigen::MatrixXd Ppu_qcqp_;
    Eigen::MatrixXd Pvu_qcqp_;
    Eigen::MatrixXd Pau_qcqp_;
    Eigen::MatrixXd Qmat_qcqp_;
    Eigen::MatrixXd Qpy_mat_qcqp;
    Eigen::MatrixXd Qvy_mat_qcqp; 
    Eigen::MatrixXd Qay_mat_qcqp;
    Eigen::MatrixXd Ry_mat_qcqp;
    Eigen::MatrixXd Qmat_dcm_;
    Eigen::MatrixXd Qmat_cp_;
    Eigen::MatrixXd Qmat_f_qcqp_;
    Eigen::MatrixXd theta_dcm_;
    Eigen::MatrixXd theta_cp_;
    Eigen::MatrixXd e1_dcm_;
    Eigen::MatrixXd e1_cp_;
    Eigen::MatrixXd Qcalc_th4_;
    Eigen::MatrixXd Qxcalc_qcqp_;
    Eigen::MatrixXd Qxcalc_f_qcqp_;
    Eigen::MatrixXd Qycalc_qcqp_;
    Eigen::MatrixXd Qycalc_f_qcqp_;
    Eigen::MatrixXd Qzcalc_qcqp_;
    Eigen::MatrixXd Qzcalc_f_qcqp_;
    Eigen::MatrixXd Qcalc_qcqp_;
    Eigen::MatrixXd Qcalc_f_qcqp_;
    Eigen::MatrixXd Qcalc_dcm_;
    Eigen::MatrixXd Qcalc_dcm_qcqp_;
    Eigen::MatrixXd Qcalc_cp_;
    Eigen::MatrixXd deldel_Cgoal_;
    Eigen::MatrixXd deldel_Fgoal_;
    Eigen::MatrixXd deldel_Tgoal_;
    Eigen::MatrixXd deldel_Dgoal_;
    Eigen::MatrixXd deldel_th4_;
    Eigen::MatrixXd gxyzcalc_qcqp_;
    Eigen::MatrixXd gcalc_th4_;
    Eigen::MatrixXd gxcalc_qcqp_;
    Eigen::MatrixXd gxcalc_f_qcqp_;
    Eigen::MatrixXd gycalc_qcqp_;
    Eigen::MatrixXd gycalc_f_qcqp_;
    Eigen::MatrixXd gzcalc_qcqp_;
    Eigen::MatrixXd gzcalc_f_qcqp_;
    Eigen::MatrixXd gcalc_qcqp_;
    Eigen::MatrixXd gcalc_dcm_;
    Eigen::MatrixXd gcalc_dcm_qcqp_;
    Eigen::MatrixXd gcalc_cp_;
    Eigen::MatrixXd gcalc_f_qcqp_;
    Eigen::MatrixXd del_th4_;
    Eigen::MatrixXd del_cgoal_;
    Eigen::MatrixXd del_fgoal_;
    Eigen::MatrixXd del_lfgoal_;
    Eigen::MatrixXd del_rfgoal_;
    Eigen::MatrixXd del_tgoal_;
    Eigen::MatrixXd del_dgoal_;
    Eigen::MatrixXd SUx;
    Eigen::MatrixXd SUy;
    Eigen::MatrixXd SUz;
    Eigen::MatrixXd dcm_SUx;
    Eigen::MatrixXd dcm_SUy;
    Eigen::MatrixXd dcm_SUz;
    Eigen::MatrixXd SUc;
    Eigen::MatrixXd dcm_SUc;
    Eigen::MatrixXd SUf;
    Eigen::MatrixXd dcm_SUf;
    Eigen::MatrixXd SUlf;
    Eigen::MatrixXd SUrf;
    Eigen::MatrixXd dcm_Si3_;
    Eigen::MatrixXd dcm_Sfx;
    Eigen::MatrixXd dcm_Sfy;
    Eigen::MatrixXd Si6;
    Eigen::MatrixXd dcm_Si;
    Eigen::MatrixXd Sx;
    Eigen::MatrixXd Sy;
    Eigen::MatrixXd Sz;
    Eigen::MatrixXd ssx;
    Eigen::MatrixXd ssy;
    Eigen::MatrixXd ssz;
    Eigen::MatrixXd dcm_ssx;
    Eigen::MatrixXd dcm_ssy;
    Eigen::MatrixXd dcm_ssz;
    Eigen::MatrixXd sx6;
    Eigen::MatrixXd sy6;
    Eigen::MatrixXd sz6;
    Eigen::MatrixXd sx3;
    Eigen::MatrixXd sy3;
    Eigen::MatrixXd sz3;
    Eigen::MatrixXd sfx;
    Eigen::MatrixXd sfy;
    Eigen::MatrixXd sc;
    Eigen::MatrixXd foot_constr_i_;
    Eigen::MatrixXd dcm_Sf_i_;
    int dcm_delf_fix_;
    Eigen::MatrixXd Sip_foot_x_;
    Eigen::MatrixXd Sip_foot_y_;
    Eigen::MatrixXd Sip_foot_z_;
    Eigen::MatrixXd Siv_foot_x_;
    Eigen::MatrixXd Siv_foot_y_;
    Eigen::MatrixXd Siv_foot_z_;
    Eigen::MatrixXd Sia_foot_x_;
    Eigen::MatrixXd Sia_foot_y_;
    Eigen::MatrixXd Sia_foot_z_;
    Eigen::MatrixXd si_foot_x_;
    Eigen::MatrixXd si_foot_y_;
    Eigen::MatrixXd si_foot_z_;

    Eigen::VectorXd foot_desired_;

    //Constraint
    //zmp
    Eigen::MatrixXd Const_Psi_zx;
    Eigen::MatrixXd Const_Psi_zy;

    Eigen::MatrixXd Const_Pi_zx_u;
    Eigen::MatrixXd Const_Pi_zx_l;
    Eigen::MatrixXd Const_Pi_zy_u;
    Eigen::MatrixXd Const_Pi_zy_l;

    Eigen::MatrixXd Const_pi_zx_calc;
    Eigen::MatrixXd Const_pi_zx_u_tp;
    Eigen::MatrixXd Const_pi_zx_l_tp;
    Eigen::MatrixXd Const_pi_zy_calc;
    Eigen::MatrixXd Const_pi_zy_u_tp;
    Eigen::MatrixXd Const_pi_zy_l_tp;

    Eigen::MatrixXd Const_ri_zx_calc;
    Eigen::MatrixXd Const_ri_zx_u;
    Eigen::MatrixXd Const_ri_zx_l;
    Eigen::MatrixXd Const_ri_zy_calc;
    Eigen::MatrixXd Const_ri_zy_u;
    Eigen::MatrixXd Const_ri_zy_l;
    
    Eigen::MatrixXd Const_del_hi_zx_u;
    Eigen::MatrixXd Const_del_hi_zx_l;
    Eigen::MatrixXd Const_del_hi_zy_u;
    Eigen::MatrixXd Const_del_hi_zy_l;

    Eigen::MatrixXd Const_hi_zx_u;
    Eigen::MatrixXd Const_hi_zx_l;
    Eigen::MatrixXd Const_hi_zy_u;
    Eigen::MatrixXd Const_hi_zy_l;

    Eigen::MatrixXd Const_del_zmp_;
    Eigen::MatrixXd Const_zmp_;

    //foot
    Eigen::MatrixXd Const_Psi_fz;

    Eigen::MatrixXd Const_Pi_fz_u;
    Eigen::MatrixXd Const_Pi_fz_l;

    Eigen::MatrixXd Const_pi_fz_calc;
    Eigen::MatrixXd Const_pi_fz_u_tp;
    Eigen::MatrixXd Const_pi_fz_l_tp;

    Eigen::MatrixXd Const_ri_fz_calc;
    Eigen::MatrixXd Const_ri_fz_u;
    Eigen::MatrixXd Const_ri_fz_l;
    
    Eigen::MatrixXd Const_del_hi_fz_u;
    Eigen::MatrixXd Const_del_hi_fz_l;

    Eigen::MatrixXd Const_hi_fz_u;
    Eigen::MatrixXd Const_hi_fz_l;

    Eigen::MatrixXd Const_del_fz_;
    Eigen::MatrixXd Const_fz_;

    Eigen::MatrixXd Const_del_z_;
    Eigen::MatrixXd Const_z_;

    //leg length
    Eigen::MatrixXd Const_Psi_ll;

    Eigen::MatrixXd Const_Pi_ll_u;
    Eigen::MatrixXd Const_Pi_ll_l;

    Eigen::MatrixXd Const_pi_ll_calc;
    Eigen::MatrixXd Const_pi_ll_u_tp;
    Eigen::MatrixXd Const_pi_ll_l_tp;
    
    Eigen::MatrixXd Const_ri_ll_calc;
    Eigen::MatrixXd Const_ri_ll_u;
    Eigen::MatrixXd Const_ri_ll_l;

    Eigen::MatrixXd Const_del_hi_ll_u;
    Eigen::MatrixXd Const_del_hi_ll_l;

    Eigen::MatrixXd Const_hi_ll_u;
    Eigen::MatrixXd Const_hi_ll_l;

    Eigen::MatrixXd Const_del_ll_;
    Eigen::MatrixXd Const_ll_;

    //delf
    Eigen::MatrixXd Const_Psi_delf;

    Eigen::MatrixXd Const_Pi_delf_u;
    Eigen::MatrixXd Const_Pi_delf_l;

    Eigen::MatrixXd Const_pi_delf_calc;
    Eigen::MatrixXd Const_pi_delf_u_tp;
    Eigen::MatrixXd Const_pi_delf_l_tp;
    
    Eigen::MatrixXd Const_ri_delf_calc;
    Eigen::MatrixXd Const_ri_delf_u;
    Eigen::MatrixXd Const_ri_delf_l;

    Eigen::MatrixXd Const_del_hi_delf_u;
    Eigen::MatrixXd Const_del_hi_delf_l;

    Eigen::MatrixXd Const_hi_delf_u;
    Eigen::MatrixXd Const_hi_delf_l;

    Eigen::MatrixXd Const_del_delf_;
    Eigen::MatrixXd Const_delf_;

    //dcm qcqp
    Eigen::MatrixXd Const_Psi_dcm_zmpx;
    Eigen::MatrixXd Const_Psi_dcm_zmpy;
    Eigen::MatrixXd Const_Psi_dcm_ll;

    Eigen::MatrixXd Const_Pi_dcm_zmpx_u;
    Eigen::MatrixXd Const_Pi_dcm_zmpx_l;
    Eigen::MatrixXd Const_Pi_dcm_zmpy_u;
    Eigen::MatrixXd Const_Pi_dcm_zmpy_l;
    Eigen::MatrixXd Const_Pi_dcm_ll_u;
    Eigen::MatrixXd Const_Pi_dcm_ll_l;

    Eigen::MatrixXd Const_pi_dcm_zmpx_calc;
    Eigen::MatrixXd Const_pi_dcm_zmpx_u_tp;
    Eigen::MatrixXd Const_pi_dcm_zmpx_l_tp;
    Eigen::MatrixXd Const_pi_dcm_zmpy_calc;
    Eigen::MatrixXd Const_pi_dcm_zmpy_u_tp;
    Eigen::MatrixXd Const_pi_dcm_zmpy_l_tp;
    Eigen::MatrixXd Const_pi_dcm_ll_calc;
    Eigen::MatrixXd Const_pi_dcm_ll_u_tp;
    Eigen::MatrixXd Const_pi_dcm_ll_l_tp;
    
    Eigen::MatrixXd Const_ri_dcm_zmpx_calc;
    Eigen::MatrixXd Const_ri_dcm_zmpx_u;
    Eigen::MatrixXd Const_ri_dcm_zmpx_l;
    Eigen::MatrixXd Const_ri_dcm_zmpy_calc;
    Eigen::MatrixXd Const_ri_dcm_zmpy_u;
    Eigen::MatrixXd Const_ri_dcm_zmpy_l;
    Eigen::MatrixXd Const_ri_dcm_ll_calc;
    Eigen::MatrixXd Const_ri_dcm_ll_u;
    Eigen::MatrixXd Const_ri_dcm_ll_l;

    Eigen::MatrixXd Const_del_hi_dcm_zmpx_u;
    Eigen::MatrixXd Const_del_hi_dcm_zmpx_l;
    Eigen::MatrixXd Const_del_hi_dcm_zmpy_u;
    Eigen::MatrixXd Const_del_hi_dcm_zmpy_l;
    Eigen::MatrixXd Const_del_hi_dcm_ll_u;
    Eigen::MatrixXd Const_del_hi_dcm_ll_l;

    Eigen::MatrixXd Const_hi_dcm_zmpx_u;
    Eigen::MatrixXd Const_hi_dcm_zmpx_l;
    Eigen::MatrixXd Const_hi_dcm_zmpy_u;
    Eigen::MatrixXd Const_hi_dcm_zmpy_l;
    Eigen::MatrixXd Const_hi_dcm_ll_u;
    Eigen::MatrixXd Const_hi_dcm_ll_l;

    Eigen::MatrixXd Const_del_dcm_zmp_;
    Eigen::MatrixXd Const_del_dcm_ll_;
    Eigen::MatrixXd Const_dcm_zmp_;
    Eigen::MatrixXd Const_dcm_ll_;


    //state
    Eigen::VectorXd MPC_qcqp_dcm_foot_u_;
    Eigen::VectorXd MPC_qcqp_;
    Eigen::VectorXd MPC_th4_;
    Eigen::VectorXd MPC_qcqp_gurobi_;

    Eigen::VectorXd MPC_qcqp_sqp_gurobi_;          
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_thread_;   
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_thread2_;  
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_p_;        
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_p_thread_; 
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_p_thread2_;
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_n_;        
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_n_thread_; 
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_n_thread2_;
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_r_;        
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_r_p_;    
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_r_n_;
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_r_sc_;     
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_r_p_sc_;
    Eigen::Vector3d MPC_qcqp_sqp_gurobi_u_;
    Eigen::Vector3d MPC_qcqp_sqp_gurobi_u_thread_;
    Eigen::Vector3d MPC_qcqp_sqp_gurobi_u_thread2_;
    Eigen::VectorXd qcqp_sqp_com_calc_;
    int MPC_qcqp_sqp_planner_hz_;

    Eigen::Vector2d MPC_qcqp_zmp_;
    Eigen::Vector2d MPC_qcqp_zmp_gurobi_;
    Eigen::Vector2d MPC_qcqp_sqp_gurobi_zmp_;
    Eigen::Vector2d MPC_qcqp_sqp_gurobi_zmp_p_;
    Eigen::Vector3d MPC_qcqp_sqp_gurobi_dcm_;
    double MPC_qcqp_sqp_gurobi_lambda_;
    double MPC_qcqp_sqp_gurobi_b_;
    Eigen::MatrixXd MPC_qcqp_sqp_gurobi_dcm_prev_;
    Eigen::MatrixXd MPC_qcqp_sqp_gurobi_cp_prev_;
    Eigen::MatrixXd MPC_qcqp_sqp_gurobi_com_prev_;
    Eigen::MatrixXd MPC_qcqp_sqp_gurobi_com_prev_p_;
    Eigen::VectorXd MPC_qcqp_sqp_u_;
    Eigen::VectorXd MPC_qcqp_sqp_u_p_;
    Eigen::VectorXd th4_u_;
    Eigen::VectorXd MPC_qcqp_input_;
    Eigen::VectorXd MPC_dcm_qcqp_sqp_del_u_;
    Eigen::VectorXd MPC_dcm_qcqp_sqp_u_;
    Eigen::Vector3d MPC_dcm_vrp_;
    Eigen::Vector3d MPC_dcm_vrp_p_;
    Eigen::Vector3d MPC_dcm_vrp_r_;
    Eigen::Vector3d MPC_dcm_vrp_r_p_;
    Eigen::Vector3d MPC_dcm_vrp_r_sc_;
    Eigen::Vector3d MPC_dcm_vrp_r_p_sc_;
    Eigen::Vector3d MPC_dcm_vrp_thread_;
    Eigen::Vector3d MPC_dcm_vrp_p_thread_;
    Eigen::Vector3d MPC_dcm_vrp_thread2_;
    Eigen::Vector3d MPC_dcm_vrp_p_thread2_;
    Eigen::Vector3d vrp_desired_;
    Eigen::VectorXd vrp_u_b_;

    double MPC_dcm_lambda_;
    double MPC_dcm_lambda_p_;
    double MPC_dcm_lambda_r_;
    double MPC_dcm_lambda_r_p_;
    double MPC_dcm_lambda_r_sc_;
    double MPC_dcm_lambda_r_p_sc_;
    double MPC_dcm_lambda_thread_;
    double MPC_dcm_lambda_p_thread_;
    double MPC_dcm_lambda_thread2_;
    double MPC_dcm_lambda_p_thread2_;
    Eigen::Vector2d MPC_dcm_zmp_;
    Eigen::Vector2d MPC_dcm_zmp_p_;
    Eigen::Vector2d MPC_dcm_zmp_r_;
    Eigen::Vector2d MPC_dcm_zmp_r_p_;
    Eigen::Vector2d MPC_dcm_zmp_r_sc_;
    Eigen::Vector2d MPC_dcm_zmp_r_p_sc_;
    Eigen::Vector2d MPC_dcm_zmp_thread_;
    Eigen::Vector2d MPC_dcm_zmp_p_thread_;
    Eigen::Vector2d MPC_dcm_zmp_thread2_;
    Eigen::Vector2d MPC_dcm_zmp_p_thread2_;
    Eigen::VectorXd MPC_dcm_delf_thread_;
    Eigen::VectorXd MPC_dcm_delf_;
    Eigen::VectorXd MPC_dcm_delf_fix_;
    Eigen::VectorXd MPC_dcm_delf_fix_p_;
    Eigen::VectorXd MPC_dcm_delf_p_;
    Eigen::VectorXd delf_prev_step_;
    Eigen::VectorXd MPC_dcm_delf_r_;
    Eigen::VectorXd MPC_dcm_delf_r_p_;
    
    Eigen::Vector2d MPC_cp_zmp_;
    Eigen::Vector2d MPC_cp_zmp_p_;
    Eigen::Vector2d MPC_cp_zmp_r_;
    Eigen::Vector2d MPC_cp_zmp_r_p_;
    Eigen::Vector2d MPC_cp_zmp_thread2_;

    //foot state
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_;     
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_thread_;   
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_thread2_;  
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_p_;        
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_p_thread_; 
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_p_thread2_;
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_n_;        
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_n_thread_; 
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_n_thread2_;
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_r_;        
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_r_p_;      
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_r_n_;      
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_r_sc_;     
    Eigen::VectorXd MPC_qcqp_sqp_gurobi_foot_r_p_sc_;

    Eigen::VectorXd MPC_qcqp_sqp_u_foot_;
    Eigen::VectorXd MPC_qcqp_sqp_u_lfoot_;
    Eigen::VectorXd MPC_qcqp_sqp_u_rfoot_;

    std::atomic<bool> atb_mpc_qcqp_update_{false};
    std::atomic<bool> atb_mpc_main_qcqp_update_{false};
    std::atomic<bool> atb_mpc_cp_update_{false};
    std::atomic<bool> atb_mpc_dcm_xyz_update_{false};

    double com_start_tick_e_qcqp_;
    double com_start_tick_e_qcqp_mpc_;
    double com_start_tick_e_qcqp_thread_;

    int current_step_num_qcqp_mpc_;
    int current_step_num_qcqp_thread_;
    int current_step_num_qcqp_thread2_;

    Eigen::MatrixXd ref_zmp_e_;
    Eigen::MatrixXd ref_zmp_e_mpc_;
    Eigen::MatrixXd ref_zmp_e_thread_;

    Eigen::MatrixXd ref_com_e_;
    Eigen::MatrixXd ref_com_e_mpc_;
    Eigen::MatrixXd ref_com_e_thread_;

    Eigen::MatrixXd ref_lfoot_e_;
    Eigen::MatrixXd ref_lfoot_e_mpc_;
    Eigen::MatrixXd ref_lfoot_e_thread_;

    Eigen::MatrixXd ref_rfoot_e_;
    Eigen::MatrixXd ref_rfoot_e_mpc_;
    Eigen::MatrixXd ref_rfoot_e_thread_;

    Eigen::MatrixXd max_zmp_e_mpc_;
    Eigen::MatrixXd max_zmp_e_thread_;
    Eigen::MatrixXd min_zmp_e_mpc_;
    Eigen::MatrixXd min_zmp_e_thread_;

    int qcqp_e_interpol_cnt_ = 0;
    int dcm_x_interpol_cnt_ = 0;
    int dcm_y_interpol_cnt_ = 0;
    int dcm_z_interpol_cnt_ = 0;
    int dcm_xyz_interpol_cnt_ = 0;

    Eigen::VectorXd qcqp_diff_;
    double dcm_lambda_diff_;
    Eigen::Vector2d dcm_zmp_diff_;

    Eigen::VectorXd qcqp_foot_diff_;

    Eigen::VectorXd qcqp_mpc_i_;
    double dcm_lambda_i_;
    Eigen::Vector2d dcm_zmp_i_;

    Eigen::VectorXd qcqp_foot_mpc_i_;

    Eigen::VectorXd qcqp_foot_mpc_cubic_calc_;
    Eigen::VectorXd qcqp_foot_mpc_cubic_dot_calc_;

    double qcqp_mpc_i_z_acc_b_ = 0.0;

    double foot_step_end_cubic = 0.0;

private:    
    //////////////////////////////// Myeong-Ju
    unsigned int walking_tick_mj = 0;
    unsigned int walking_tick_mj_mpc_ = 0;
    unsigned int walking_tick_mj_thread_ = 0;
    unsigned int walking_tick_e_thread_ = 0;
    unsigned int walking_tick_e_qcqp_mpc_ = 0;
    unsigned int walking_tick_e_qcqp_thread_ = 0;
    unsigned int initial_tick_mj = 0;
    unsigned int initial_flag = 0;
    const double hz_ = 2000.0;  
};
