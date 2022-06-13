#include <string.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <numeric>

#define ROOT_DIR "/home/dyros/catkin_ws/src/tocabi_avatar/model/target/"

enum class Wrench
{
    num_clusters = 4,
    input_dim = 1
};

enum class Pose
{
    num_clusters = 3,
    input_dim = 1
};

class Indicator
{
    public:
        Indicator(std::string name);    

        void loadModel();
        void loadNominalWalkingData();
        int defineState(Eigen::VectorXd &x);
        int getInputDim();
        int getNumCluster();
        void publishResult(float in_data, int out_data);
        
        std::string name_;
        std::string filename_list_[4] = {"_mu.txt","_sigma.txt","_coeff.txt","_info.txt"};

        Eigen::MatrixXd mean_;
        Eigen::MatrixXd sigma_;
        Eigen::VectorXd coeff_;
        Eigen::VectorXd info_;
        Eigen::VectorXd nominal_w_mean_;
        Eigen::MatrixXd nominal_w_cov_;
        Eigen::VectorXd responsibility_;

        ros::NodeHandle nh_indicator_;
        ros::Publisher indicator_out_pub_, indicator_in_pub_;

    private:

        double computeMVN(Eigen::VectorXd &mean, Eigen::MatrixXd &sigma, Eigen::VectorXd &x);
        Eigen::VectorXd computeResponsibility(Eigen::MatrixXd &mean,
                                            Eigen::MatrixXd &sigma,
                                            Eigen::VectorXd &coeff,
                                            Eigen::VectorXd &x);
        int num_clusters_;
        int input_dim_;
        std::string in_topic_name_, out_topic_name_;

};

class Discriminator
{
    public:
        Discriminator();
        int predict(int w, int p);
        void publishResult(int cs);
        
        double pause_count_;
        bool pause_flag_;
        Eigen::VectorXd freezing_q_;
        double freezing_tick_;

        ros::NodeHandle nh_discriminator_;
        ros::Publisher discriminator_pub_;

    private:
        int rule(int w, int p);
        std::vector<int> filter(std::vector<int> &cs_buffer, int target_cs, int buffer_size);
        std::vector<int> cs_buffer_;   
};