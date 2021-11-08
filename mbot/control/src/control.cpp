/*
    controller
*/

# include <string>
# include <math.h>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <std_msgs/Header.h>
# include <std_msgs/Float64.h>

double zeit = 0.02;
double pi = 3.1415926;
double g = 9.8;
      
double theta = 0.1;
double m_pend = 0.2;
double l_pend = 0.3;
double J_pend = 0.018;
double v_pend = 0.0;
double a_pend = 0.0;

double v_cart = 0.0;
double m_cart = 0.5;      
double a_cart = 0.0;
double s_cart = 0.0;
       
double P = 1000.0;
double I = 100.0;
double D = 300.0;
double F = 0.0;
double error_sum = 0.0;
double error_past = 0.0;

class tl1{

public:
    tl1();
    void value();
    class controller{
        public:
            double PID(double P, double I, double D, double theta);
        private:
    };
    
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void fCallback(const sensor_msgs::JointState::ConstPtr& msg); 
    
private:
    ros::Publisher pub_f64;
    
    ros::Subscriber sub_f64;
    
    ros::NodeHandle nh;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "talker_listener2");
    
    ros::NodeHandle nh;
    
    tl1 pubSub1;
    
    pubSub1.registerNodeHandle(nh);
    
    pubSub1.value();
    
    pubSub1.registerPubSub();
    
    ros::spin();
}

tl1::tl1(){};

void tl1::registerNodeHandle(ros::NodeHandle& _nh){

    nh = _nh;
};

void tl1::value(){

    double zeit;
    double pi;
    double g;
   
    double theta;
    double m_pend;
    double l_pend;
    double J_pend;
    double v_pend;
    double a_pend;
   
    double v_cart;
    double m_cart;
    double a_cart;
    double s_cart;
    
    double P;
    double I;
    double D;
    double F;
    
    nh.getParam("zeit", zeit);
    nh.getParam("pi", pi);
    nh.getParam("g", g);
   
    nh.getParam("theta", theta);
    nh.getParam("m_pend", m_pend);
    nh.getParam("l_pend", l_pend);
    nh.getParam("J_pend", J_pend);
    nh.getParam("v_pend", v_pend);
    nh.getParam("a_pend", a_pend);
   
    nh.getParam("v_cart", v_cart);
    nh.getParam("m_cart", m_cart);
    nh.getParam("a_cart", a_cart);
    nh.getParam("s_cart", s_cart);
    
    nh.getParam("P", P);
    nh.getParam("I", I);
    nh.getParam("D", D);
    nh.getParam("F", F);
};

void tl1::registerPubSub(){

    pub_f64 = nh.advertise<std_msgs::Float64>("chatter2to1",1500);   // save all but only use one can man use 10 or 1(currently) 
    
    sub_f64 = nh.subscribe("/joint_states",1000,&tl1::fCallback, this); // this pointer here means the class itself
};

void tl1::fCallback(const sensor_msgs::JointState::ConstPtr& msg){
    
    double theta;
    double s_cart;
    
    double F;
    double P;
    P = 10.0;
    double I;
    I = 4.0;
    double D;
    D = 3.0;
    
    msg->position;
    
    std_msgs::Float64 pubData;
    controller controller;
    
    s_cart = msg->position[0];
    theta = msg->position[1];
    
    F = controller.PID(P,I,D,theta);
   
    pubData.data = F;
 
    ros::Rate sr(50);
    
    pub_f64.publish(pubData);
    
    sr.sleep();
};

double tl1::controller::PID(double P, double I, double D, double theta){
    
    double error;
    double error_sum;
    double error_past;
    
    error = 0.0 - theta;
    
    error_sum = error_sum + error;
    
    F = P * error + I * error_sum + D * (error - error_past);
    
    error_past = error;
    
    return -F;
};















