/*
    dynamic
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
       
double P = 100.0;
double I = 0.0;
double D = 0.0;
double F = 0.0;

class tl1{

public:
    tl1();
    void value();
    class dynamic{
        public:
            double get_A_pend(double theta, double v_pend, double F);
            double getPendSpeed(double a_pend, double v_pend);
            double getPendPosition(double v_pend, double theta);
            
            double get_A_cart(double theta, double v_pend, double a_pend, double F);
            double getCartSpeed(double a_cart, double v_cart);
            double getCartPosition(double v_cart, double s_cart);
        private:
    };
    
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void iniPub();
    
    void fCallback(const std_msgs::Float64::ConstPtr& msg);
    
private:
    ros::Publisher pub_f64;
    
    ros::Subscriber sub_f64;
    
    ros::NodeHandle nh;
    // the code will throw the pin after the function runs
};

int main(int argc, char **argv){

    ros::init(argc, argv, "talker_listener1");
    
    ros::NodeHandle nh;
    
    tl1 pubSub1;
    
    pubSub1.registerNodeHandle(nh);
    
    pubSub1.value();
    
    pubSub1.registerPubSub();
    
    pubSub1.iniPub();
    
    ros::spin();
    
}

tl1::tl1(){}

void tl1::registerNodeHandle(ros::NodeHandle& _nh){

    nh = _nh;
};

void tl1::value(){
   
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
   
   nh.getParam("F", F);
};

void tl1::registerPubSub(){
    
    pub_f64 = nh.advertise<sensor_msgs::JointState>("/joint_states",1000);
    
    sub_f64 = nh.subscribe("/chatter2to1", 500, &tl1::fCallback, this); //this pointer here means the class itself
};

void tl1::fCallback(const std_msgs::Float64::ConstPtr& msg){

    msg->data;
    
    sensor_msgs::JointState pubData;
    dynamic dynamic;
       
    F = msg->data;
    
    a_pend = dynamic.get_A_pend(theta,v_pend,F);
    a_cart = dynamic.get_A_cart(theta,v_pend,a_pend,F);
    
    v_pend = dynamic.getPendSpeed(a_pend,v_pend);
    theta = dynamic.getPendPosition(v_pend,theta);
    
    v_cart = dynamic.getCartSpeed(a_cart, v_cart);
    s_cart = dynamic.getCartPosition(v_cart,s_cart);

    pubData.header.stamp = ros::Time::now();            //callback save the data 
    
    pubData.name.resize(2);
    pubData.name[0] = "base_link_to_cart";
    pubData.name[1] = "pend_joint";
    
    pubData.position.resize(2);
    pubData.position[0] = s_cart;
    pubData.position[1] = theta;
    
    ros::Rate sr(1000);
    
    pub_f64.publish(pubData);
    
    sr.sleep();
}; 


void tl1::iniPub(){
    
    sensor_msgs::JointState pubData;
    
    pubData.header.stamp = ros::Time::now();
    
    pubData.name.resize(2);
    pubData.name[0] = "base_link_to_cart";
    pubData.name[1] = "pend_joint";
        
    pubData.position.resize(2);
    pubData.position[0] = 0.0;
    pubData.position[1] = 0.1;
    
    usleep(500000);//wait for connection
    
    pub_f64.publish(pubData);
};

double tl1::dynamic::get_A_pend(double theta, double v_pend, double F){

    a_pend = (m_pend*g*l_pend*sin(theta) - m_pend*m_pend*l_pend*l_pend*v_pend*v_pend*sin(theta)*cos(theta) / (m_cart+m_pend) - F*m_pend*l_pend*cos(theta) / (m_cart+m_pend)) / (J_pend + m_cart*m_pend*l_pend*l_pend / (m_cart+m_pend) + m_pend*m_pend*l_pend*l_pend*sin(theta)*sin(theta) / (m_cart+m_pend));
  
    return a_pend;  
};

double tl1::dynamic::getPendSpeed(double a_pend, double v_pend){

    v_pend = v_pend + zeit * a_pend;
    
    return v_pend;
};

double tl1::dynamic::getPendPosition(double v_pend, double theta){

    theta = theta + v_pend * zeit;
    
    return theta;
};

double tl1::dynamic::get_A_cart(double theta, double v_pend, double a_pend, double F){

    a_cart = (F+m_pend*l_pend*(v_pend*v_pend*sin(theta) - a_pend*cos(theta))) / (m_cart + m_pend);
    
    return a_cart;
};

double tl1::dynamic::getCartSpeed(double a_cart, double v_cart){

    v_cart = v_cart + a_cart * zeit;
    
    return v_cart;
};

double tl1::dynamic::getCartPosition(double v_cart, double s_cart){

    s_cart = s_cart + zeit * v_cart;
    
    return s_cart;
};





