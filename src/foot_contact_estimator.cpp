#include "foot_contact_estimator.h"

Indicator::Indicator(std::string name) : name_(name)
{
    if(name_ == "wrench")
    {
        num_clusters_ = static_cast<int>(Wrench::num_clusters);
        input_dim_ = static_cast<int>(Wrench::input_dim);

        for(int i = 0; i < 4; i ++)
            filename_list_[i] = "w" + filename_list_[i];      
    }
    else if(name_ == "pose")
    {
        num_clusters_ = static_cast<int>(Pose::num_clusters);
        input_dim_ = static_cast<int>(Pose::input_dim);

        for(int i = 0; i < 4; i ++)
            filename_list_[i] = "p" + filename_list_[i];        
    }
    Indicator::loadNominalWalkingData();
    Indicator::loadModel();

    in_topic_name_ = "/foot_contact_estimator/"+name_+"_indicator/input";
    out_topic_name_ = "/foot_contact_estimator/"+name_+"_indicator/output";
    indicator_in_pub_ = nh_indicator_.advertise<std_msgs::Float32>(in_topic_name_, 100);
    indicator_out_pub_ = nh_indicator_.advertise<std_msgs::Int32>(out_topic_name_, 100);


    // responsibility_ = Indicator::computeResponsibility(mean_, sigma_, coeff_, input_);
    // int x;
    // Eigen::VectorXd input;
    // input.resize(input_dim_,1);
    // input << 0.02;
    // x = defineState(mean_, sigma_, coeff_, input);
}

void Indicator::loadModel()
{
    mean_.resize(num_clusters_, input_dim_);
    sigma_.resize(num_clusters_ * input_dim_, num_clusters_ * input_dim_);
    coeff_.resize(num_clusters_);
    info_.resize(num_clusters_);
    responsibility_.resize(num_clusters_);
    
    mean_.setZero();
    sigma_.setZero();
    coeff_.setZero();
    info_.setZero();
    responsibility_.setZero();
    
    for (int i = 0; i < 4; i ++)
    {   
        // read text as in the order "mu", "sigma", "coeff", "info"
        std::ifstream file(ROOT_DIR+filename_list_[i]);
        if(file.is_open())
        {
            std::cout<<ROOT_DIR+filename_list_[i]<<std::endl;
            // read mu.txt
            if(i == 0)
            {
                for(int row = 0; row < num_clusters_; row++)
                    for(int col = 0; col < input_dim_; col++)
                        file >> mean_(row, col);                
            }
            // read sigma.txt
            else if(i == 1)
            {
                for(int dim=0; dim<num_clusters_; dim++)
                    for(int row = 0; row < input_dim_; row++)
                        for(int col=0; col<input_dim_; col++){
                            file>>sigma_(dim*input_dim_+row, dim*input_dim_+col);
                        }
            }
            // read coeff.txt
            else if(i == 2)
            {
                for(int col=0; col<num_clusters_; col++)
                    file>>coeff_(col);

            }
            // read info.txt
            else
            {
                for(int col=0; col<num_clusters_; col++)
                    file>>info_(col);                
            }
        }
        file.close();
    }
    std::cout<<"Load "<< name_<<" indicator!!"<<std::endl;
}

void Indicator::loadNominalWalkingData()   
{
    nominal_w_mean_.resize(6,1);
    nominal_w_cov_.resize(6,6);

    nominal_w_mean_.setZero();
    nominal_w_cov_.setZero();
    
    std::ifstream nominal_mean(ROOT_DIR+std::string("nominal_mean.txt"));
    std::ifstream nominal_cov(ROOT_DIR+std::string("nominal_cov.txt"));
    if(nominal_mean.is_open())
    {
        for(int i=0; i<6; i++)
            nominal_mean>>nominal_w_mean_(i);      
    }
    if(nominal_cov.is_open()){
        for(int row = 0; row < 6; row++){
            for(int col=0; col<6; col++){
                nominal_cov>>nominal_w_cov_(row, col);
            }
        }
    }    
    std::cout<<nominal_w_cov_<<std::endl;
}

int Indicator::defineState(Eigen::VectorXd &x)
{
    int max_resp_idx, state_idx;
    responsibility_ = Indicator::computeResponsibility(mean_, sigma_, coeff_, x);    
    responsibility_.maxCoeff(&max_resp_idx);            
    for(int i=0; i<num_clusters_; i++){
        if(info_(i) == max_resp_idx+1.0) {
            state_idx = i;
            break;
        }            
    }    
    return state_idx;
}

int Indicator::getInputDim()
{
    return input_dim_;
}

int Indicator::getNumCluster()
{
    return num_clusters_;
}

double Indicator::computeMVN(Eigen::VectorXd &mean,
                             Eigen::MatrixXd &sigma,
                             Eigen::VectorXd &x)
{
    double det_sigma;
    float k;
    double alpha, beta, p;
    
    det_sigma = sigma.determinant();
    k = input_dim_;

    alpha = pow(2*M_PI, k/2)*sqrt(det_sigma);    
    beta = (-1.0/2)*(x-mean).transpose()*sigma.inverse()*(x-mean);
    p = 1/alpha*exp(beta);

    
    return p;
}

Eigen::VectorXd Indicator::computeResponsibility(Eigen::MatrixXd &mean,
                                                 Eigen::MatrixXd &sigma,
                                                 Eigen::VectorXd &coeff,
                                                 Eigen::VectorXd &x)
{
    Eigen::VectorXd resp, pred;

    resp.resize(num_clusters_, 1);
    pred.resize(num_clusters_, 1);
    resp.setZero();
    pred.setZero();
    
    for(int i=0; i<num_clusters_; i++)
    {
        Eigen::VectorXd m;
        Eigen::MatrixXd s;
        
        m.resize(num_clusters_, 1);        
        s.resize(input_dim_, input_dim_);
        
        m = mean.block(i,0,1,input_dim_).transpose();            
        s = sigma.block(i*input_dim_, i*input_dim_, input_dim_, input_dim_);
        
        pred(i) = Indicator::computeMVN(m,s,x);        
    }
    
    double sum;
    sum = coeff.transpose()*pred;
    for(int i=0; i<num_clusters_; i++)
    {
        resp(i) = coeff(i)*pred(i)/sum;
    }

    // if(name_ == "wrench")
    // {
    //     std::cout<<"MVN sum : "<< sum<<std::endl;
    //     std::cout<<"MVN resp: "<< resp.transpose()<<std::endl;
    // }
        

    return resp;
}

void Indicator::publishResult(float in_data, int out_data)
{
    std_msgs::Int32 int_msg;
    std_msgs::Float32 float_msg;

    float_msg.data = in_data;
    int_msg.data = out_data;

    indicator_in_pub_.publish(float_msg);
    indicator_out_pub_.publish(int_msg);
}

Discriminator::Discriminator(){
    pause_count_ = 0.0;
    pause_flag_ = false;
    discriminator_pub_ = nh_discriminator_.advertise<std_msgs::Int32>("/foot_contact_estimator/discriminator/output",100);
}

int Discriminator::predict(int w, int p)
{
    int cs, filtered_cs;
    std::vector<int> filtered_cs_buffer;
    int buffer_size = 200;

    cs = Discriminator::rule(w, p);

    if(cs_buffer_.size() < buffer_size){
        cs_buffer_.push_back(cs);
        filtered_cs = cs;
    }
    else{
        cs_buffer_.erase(cs_buffer_.begin()); // erase the first element!
        cs_buffer_.push_back(cs);

        filtered_cs_buffer = Discriminator::filter(cs_buffer_, 1, buffer_size);
        filtered_cs_buffer = Discriminator::filter(filtered_cs_buffer, 2, buffer_size);
        filtered_cs_buffer = Discriminator::filter(filtered_cs_buffer, 3, 100);
        filtered_cs = filtered_cs_buffer.back();       
    }
    return filtered_cs;
}

void Discriminator::publishResult(int cs)
{
    std_msgs::Int32 msg;
    msg.data = cs;
    discriminator_pub_.publish(msg);
}

int Discriminator::rule(int w, int p)
{
    // For w : [0, 1, 2, 3] = [Fn, Fs, Fm, Fl]
    // For p : [0, 1, 2] = [Pn, Ps, Pl]
    // For the final contact state : 
    // 0 : stable withouth any disturbance
    // 1 : stable with small disturbance
    // 2 : moderately stable even with large disturbance
    // 3 : unstable due to the excessive disturbance --> going to be falling!!
    int cs;

    if(w==0 && p==0) cs = 0; // stable
    else if(w==0 && p==1) cs = 1; // disturbed
    else if(w==0 && p==2) cs = 2; // hazard

    else if(w==1 && p==0) cs = 0; // stable
    else if(w==1 && p==1) cs = 1; // disturbed
    else if(w==1 && p==2) cs = 2; // hazard
    
    else if(w==2 && p==0) cs = 1; // disturbed
    else if(w==2 && p==1) cs = 2; // hazard
    else if(w==2 && p==2) cs = 2; // hazard
    
    else cs = 3; // falling

    return cs;
}

std::vector<int> Discriminator::filter(std::vector<int> &cs_buffer, 
                                        int target_cs,
                                        int buffer_size)
{
    int filtered_cs;
    int sum_of_element;
    std::vector<int> filtered_cs_buffer;
    std::vector<int> v(cs_buffer.end()-buffer_size, cs_buffer.end());


    if(cs_buffer.back() == target_cs){
        sum_of_element = std::accumulate(v.begin(), v.end(), 0);
        if(sum_of_element == target_cs * buffer_size){
            filtered_cs = target_cs;
        }
        else{
            if(sum_of_element > target_cs * buffer_size){
                filtered_cs = v.back();            
            }
            else{
                filtered_cs = v.back() - 1; // down grade the degree of the cs            
            }
        }
    }
    else
        filtered_cs = cs_buffer.back();

    filtered_cs_buffer.assign(cs_buffer.begin()+1, cs_buffer.end());
    filtered_cs_buffer.push_back(filtered_cs);

    return filtered_cs_buffer;
}