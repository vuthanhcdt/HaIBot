#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


// vi tri cua mang button
#define nhan 0
#define tron 1
#define tam_giac 2
#define vuong 3
#define trai_tren 4
#define phai_tren 5
#define trai_duoi 6
#define phai_duoi 7
#define select 8
#define start 9
#define connect 10
#define analog_trai 11
#define analog_phai 12
#define up 13
#define down 14
#define left 15
#define right 16
// vi tri cua mang axes
#define trai_sang 0
#define trai_len 1
#define nut_duoi_trai 2
#define phai_sang 3
#define phai_len 4
#define nut_duoi_phai 5


ros::Publisher pub;
geometry_msgs::Twist data;
double vtdai_xe = 0, vtgoc_xe = 0;


std::string topic_vdai_vgoc;

void joyTwistCallback(const sensor_msgs::JoyConstPtr &joy)
{
    if(joy->axes[trai_len]>0.5)//tien
    {
        if(joy->buttons[trai_tren])
        {
            data.angular.z=vtgoc_xe/3;
        }
        else  if(joy->buttons[phai_tren])
        {
            data.angular.z=-vtgoc_xe/3;
        }
        else
        {
            data.angular.z=0;
        }
        data.linear.x=vtdai_xe;
        
    }
    else if(joy->axes[trai_len]<-0.5)//lui
    {
        if(joy->buttons[trai_tren])
        {
            data.angular.z=-vtgoc_xe/3;
        }
        else  if(joy->buttons[phai_tren])
        {
            data.angular.z=vtgoc_xe/3;
        }
        else
        {
            data.angular.z=0;
        }
        data.linear.x=-vtdai_xe;
    }
    else
    {
        if(joy->buttons[trai_tren])
        {
            data.linear.x=0;
            data.angular.z=vtgoc_xe;
        }
        else if(joy->buttons[phai_tren])
        {
            data.linear.x=0;
            data.angular.z=-vtgoc_xe;
        }
        else
        {
            data.linear.x=0;
            data.angular.z=0;
        }
        
    }
    
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "ps3_node");
 ros::NodeHandle node_handle;
 ros::NodeHandle private_node_handle("~");//node param
 ros::Subscriber joy_subsriber = node_handle.subscribe<sensor_msgs::Joy>("joy",10,joyTwistCallback) ;
 private_node_handle.param<std::string>("topic_vdai_vgoc", topic_vdai_vgoc, "Twist");//cong usb
 private_node_handle.param<double>("vtdai_xe", vtdai_xe, 0);
 private_node_handle.param<double>("vtgoc_xe", vtgoc_xe, 0);
 pub = node_handle.advertise<geometry_msgs::Twist>(topic_vdai_vgoc, 2);
 ROS_INFO("Start ");
 ros::Rate loop_rate(50);//khai bong vong lap 10hz
 while(ros::ok())
 {
    pub.publish(data);
    ros::spinOnce();
    loop_rate.sleep();
 }
 return 0;
}