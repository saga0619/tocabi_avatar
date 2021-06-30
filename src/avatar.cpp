#include "avatar.h"

using namespace TOCABI;

AvatarController::AvatarController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();

    nh_avatar_.setCallbackQueue(&queue_avatar_);
    sub_1 = nh_avatar_.subscribe("/tocabi/avatar_test", 1, &AvatarController::avatar_callback, this);

    std::cout<<"register AVATAR!"<<std::endl;
}

void AvatarController::avatar_callback(const std_msgs::StringConstPtr &msg)
{
    std::cout << "received : " << msg->data << std::endl;
}

Eigen::VectorQd AvatarController::getControl()
{
    return ControlVal_;
}

// void AvatarController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void AvatarController::computeSlow()
{   
    queue_avatar_.callAvailable(ros::WallDuration());

    if (rd_.tc_.mode == 10)
    {

        if (rd_.tc_init)
        {
            //Initialize settings for Task Control!

            std::cout<<"avatar mode 10 init !"<<std::endl;
            rd_.tc_init = false;

            //rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
        }

        WBC::SetContact(rd_, 1, 1);

        rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac();
        rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

        rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
        rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

        rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll) * DyrosMath::rotateWithY(rd_.tc_.pitch) * DyrosMath::rotateWithZ(rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

        Eigen::VectorXd fstar;
        rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        fstar.setZero(9);
        fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id]);
        fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

        rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));
    }
}

void AvatarController::computeFast()
{
    // if (tc.mode == 10)
    // {
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void AvatarController::computePlanner()
{
}

void AvatarController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}