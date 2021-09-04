/*******************************************************************/
/****************************HaIBot Lab****************************/
/**************************Vu Cong Thanh**************************/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16MultiArray.h"

std::string topic_v;
std::string topic_vdai_vgoc;
ros::Publisher  Pub_v;
std_msgs::Int16MultiArray msg;
int v_banh_trai,v_banh_phai;

double r_banh=0.05;  //ban kinh banh xe metter
double kc_hai_banh=0.38; //khoang cach 2 banh meter
double pi = 3.141592653589793238462643;
double vtgoc_trai = 0, vtgoc_phai = 0; 
double vtdai_xe = 0, vtgoc_xe = 0;
double hop_giam_toc=71.2;
int v_max_dong_co=10000,pwm_max=255;

double v_thuc;
double t_gia_toc=1;
double t_giam_toc=0.5;
double gia_toc,giam_toc;
double t_thuc_te;
double dt;
uint8_t dem_tang,dem_giam;

ros::Time measurement_time;
ros::Time pre_measurement_time;

int16_t convert_pwm(double v_banh)
{
    int16_t pwm;
    pwm=v_banh*hop_giam_toc;
    pwm=((double)pwm/v_max_dong_co)*(pwm_max);
    return pwm;
}


void donghocnghich(double v_dai,double v_goc)
{

    vtgoc_trai = (1.0/(2.0*r_banh))*(2.0*v_dai - kc_hai_banh*v_goc); //rad/s
    vtgoc_phai = (1.0/(2.0*r_banh))*(2.0*v_dai + kc_hai_banh*v_goc);
    v_banh_trai = convert_pwm(vtgoc_trai*(60/(2*pi))); //vong/phut  
    v_banh_phai = convert_pwm(vtgoc_phai*(60/(2*pi)));
    ROS_INFO("%f,%f,%d,%d",vtgoc_trai,vtgoc_phai,v_banh_trai,v_banh_phai);

}


void tinh_v()
{
    msg.data.resize(0);
    msg.data.push_back(-v_banh_trai);
    msg.data.push_back(v_banh_phai);
    Pub_v.publish(msg);
}


//================can chuyen thanh phuong trinh gia toc=====================//


void gia_toc_giam_toc(double v_dai)
{
    if(v_thuc!=v_dai&&v_dai!=0)//gia toc
    {
        dem_giam=0;
        measurement_time=ros::Time::now();
        dt=(measurement_time-pre_measurement_time).toSec();
        if(dem_tang<2)
        {
            t_thuc_te=0;
            gia_toc=v_dai/t_gia_toc;
            dem_tang++;
        }
        if(dem_tang>1)
        {
            t_thuc_te=t_thuc_te+dt;
        }
        if(t_thuc_te>t_gia_toc)
         t_thuc_te=t_gia_toc;
       
        v_thuc=(gia_toc*t_thuc_te);
        pre_measurement_time=measurement_time;
        ROS_INFO("%f,%f,%f",v_thuc,gia_toc,t_thuc_te);
    }
    else if(v_thuc!=v_dai&&v_dai==0)
    {
        dem_tang=0;
        measurement_time=ros::Time::now();
        dt=(measurement_time-pre_measurement_time).toSec();
        if(dem_giam<2)
        {
            t_thuc_te=0;
            giam_toc=v_thuc/t_giam_toc;
            dem_giam++;
        }
        if(dem_giam>1)
        {
            t_thuc_te=t_thuc_te+dt;
        }
        if(t_thuc_te>t_giam_toc)
        {
            t_thuc_te=t_giam_toc;
            v_thuc=0;
        }
        else
            v_thuc=giam_toc*t_giam_toc-(giam_toc*t_thuc_te);
        pre_measurement_time=measurement_time;
        ROS_INFO("%f,%f,%f",v_thuc,gia_toc,t_thuc_te);
    }
    else
    {
        dt=0;
        t_thuc_te=0;
        dem_tang=0;
        dem_giam=0;
    }
    
}

void control(double v_dai,double v_goc)
{
    gia_toc_giam_toc(v_dai);
    donghocnghich(v_thuc,v_goc);
    tinh_v();
}

void read_vdai_vgoc(const geometry_msgs::Twist &v_xe)
{
    //  ROS_INFO("%f,%f",v_xe.angular.z,v_xe.linear.x);
     control(v_xe.linear.x,v_xe.angular.z);
}

int main(int argc, char **argv)
{
    //========================khai bao ban dau=====================================//
    ros::init(argc, argv, "kinetic_rv");//khai bao ten node 
    ros::NodeHandle node_xu_ly;
    ros::NodeHandle private_node_handle("~");//node param
    ros::NodeHandle n;
    private_node_handle.param<std::string>("topic_v", topic_v, "van_toc_banh");//cong usb
    private_node_handle.param<std::string>("topic_vdai_vgoc", topic_vdai_vgoc, "Twist");//cong usb
    private_node_handle.param<int>("v_banh_trai", v_banh_trai, 0);
    private_node_handle.param<int>("v_banh_phai", v_banh_phai, 0);
    private_node_handle.param<double>("hop_giam_toc", hop_giam_toc, 71.2);
    private_node_handle.param<double>("vtdai_xe", vtdai_xe, 0);
    private_node_handle.param<double>("vtgoc_xe", vtgoc_xe, 0);
    private_node_handle.param<double>("kc_hai_banh", kc_hai_banh, 0.38);
    private_node_handle.param<double>("t_gia_toc", t_gia_toc, 1);
    private_node_handle.param<double>("t_giam_toc", t_giam_toc, 0.);
    private_node_handle.param<int>("v_max_dong_co", v_max_dong_co, 10000);
    private_node_handle.param<int>("pwm_max", pwm_max, 255);
    Pub_v = n.advertise<std_msgs::Int16MultiArray>(topic_v, 10); //publish tọa độ x, y, theta cho follow qua topic odom
    ros::Subscriber sub = n.subscribe(topic_vdai_vgoc, 1, read_vdai_vgoc);
    ros::spin();
}