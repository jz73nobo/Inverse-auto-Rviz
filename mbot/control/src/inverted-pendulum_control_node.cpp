/*
     inverse pendulum Robot
*/

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>


class Dynamic
{
    public:
       double time = 0.02;
       double pi = 3.1415926;
       double g = 9.8;
      
       double theta = 0.15;
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
       double I = 1.0;
       double D = 30.0;
       double F = 0.0;
       
       double controller(double theta, double P, double I, double D, double v_pend);
       
       double get_A_pend(double theta, double v_pend, double F);
       double getPendSpeed(double a_pend);
       double getPendPosition(double v_pend);
       
       double get_A_cart(double theta, double v_pend, double F, double a_pend);
       double getCartSpeed(double a_cart);
       double getCartPosition(double v_cart);
       
       
       
};

double Dynamic::controller(double theta, double P, double I, double D, double v_pend)
{

        F = P * theta + I * theta * time + D * v_pend;
        
        return F;
}

double Dynamic::get_A_pend(double theta, double v_pend, double F)
{
        a_pend = (m_pend*g*l_pend*sin(theta)-m_pend*l_pend*l_pend*v_pend*v_pend*cos(theta)*sin(theta)-F*l_pend*cos(theta)+(F*m_cart*l_pend*cos(theta))/(m_cart+m_pend)+(m_cart*m_pend*l_pend*l_pend*v_pend*v_pend*sin(theta)*cos(theta))/(m_cart+m_pend)) / (J_pend+m_pend*l_pend*l_pend*sin(theta)*sin(theta)+(m_cart*m_pend*l_pend*l_pend*cos(theta)*cos(theta))/(m_cart+m_pend));
        
        return a_pend;
}

double Dynamic::getPendSpeed(double a_pend)
{
         v_pend = v_pend + time * a_pend;
                 
         return v_pend;
}

double Dynamic::getPendPosition(double v_pend)
{
         theta = theta + v_pend * time;
         
         return theta;
}

double Dynamic::get_A_cart(double theta, double v_pend, double F, double a_pend)
{
         a_cart = (F+m_pend*l_pend*v_pend*v_pend*sin(theta)-m_pend*l_pend*a_pend*cos(theta))/(m_cart+m_pend);
         
         return a_cart;
}

double Dynamic::getCartSpeed(double a_cart)
{
         v_cart = v_cart + time * a_cart;
                                         
         return v_cart;  
}
double Dynamic::getCartPosition(double v_cart)
{
         s_cart = s_cart + time * v_cart;
                                         
         return s_cart;  
}


int main(int argc, char **argv)
{
         ros::init(argc, argv, "Pend");
         
         ros::NodeHandle n;
         
         ros::Publisher pend_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
                  
         ros::Rate loop_rate(100);
                  
         
         while (ros::ok())
         {         
                 Dynamic Dynamic;
                 
                 double theta = Dynamic.theta; 
                 double v_cart = Dynamic.v_cart;
                 double s_cart = Dynamic.s_cart;
                 double v_pend = Dynamic.v_pend;  
                 double a_cart = Dynamic.a_cart;
                 double a_pend = Dynamic.a_pend;
                 
                 double P = Dynamic.P;
                 double I = Dynamic.I;
                 double D = Dynamic.D;
                 double F;
                   
                 sensor_msgs::JointState joint_msg;
                 joint_msg.header.stamp = ros::Time::now();
                 
                 while(theta < 1.57)
                 {
                      joint_msg.name.resize(2);
                      joint_msg.name[0] = "base_link_to_cart";
                      joint_msg.name[1] = "pend_joint";
                      
                      F = Dynamic.controller(theta,P,I,D,v_pend);
                      a_pend = Dynamic.get_A_pend(theta,v_pend,F);
                      a_cart = Dynamic.get_A_cart(theta,v_pend,F,a_pend);
                      
                      v_pend = Dynamic.getPendSpeed(a_pend);
                      theta = Dynamic.getPendPosition(v_pend);
                      
                      v_cart = Dynamic.getCartSpeed(a_cart);
                      s_cart = Dynamic.getPendSpeed(v_cart);
                 
                      joint_msg.position.resize(2);                 
                      joint_msg.position[0] = s_cart;
                      joint_msg.position[1] = theta;
                 
                      pend_pub.publish(joint_msg);
                      
                      s_cart = joint_msg.position[0];
                      theta = joint_msg.position[1];
                      
                 }
              loop_rate.sleep();
              
         }
         
         return 0;
}
