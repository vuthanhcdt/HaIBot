/*******************************************************************/
/****************************MRobot Lab****************************/
/****************************11/09/2020***************************/
/**********************doc encoder tu mcu sau do pub ra**********/
/*************************Vu Cong Thanh*************************/

#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int16MultiArray.h"

//khai bao serial
std::string port;
serial::Serial ser;
std::string topic_v;

uint8_t rec_data,count;
uint8_t buff_data[8];
int16_t goc,en1,en2,pre_goc;
uint8_t reset[3]={'a','a','a'};

double delta_theta=0,delta_s=0; 
double wr=0, wl=0; //quãng đường bánh phải, trái 
double pi = 3.141592653589793238462643;//chuyen doi sang radian
double rad_to_dr = 180/3.141592653589793238462643;//chuyen doi sang radian
double rad=3.14159265359/1800;
double the_ta=0,x=0,y=0; //tọa độ
double pre_the_ta=0,pre_x=0,pre_y=0; 
double so_xung_1_vong=2000; //4 canh 1 vong ca bo truyen
double odom_goc;
double ban_kinh_banh=0.05,khoang_cach_banh=0.38;
double he_so,dt;


std::string topic_odom,topic_imu;
ros::Time measurement_time,measurement_time_imu;
std::string fixed_frame_odom;
ros::Time pre_measurement_time,pre_measurement_time_imu;
ros::Publisher odom_pub; //node Publisher
sensor_msgs::Imu imu;
ros::Publisher imu_pub;

void delta(void)
{
    //tính quãng đường bánh trái và phải 
    he_so=(2.0*pi*ban_kinh_banh)/so_xung_1_vong;

	wr= (en1)*he_so;
	wl= (en2)*he_so;
    
	delta_theta=(wr+wl)/khoang_cach_banh;
	delta_s=(wr-wl)/2.0;

}

/**************tinh odometry*************/
void odometry()
{
	the_ta = pre_the_ta + delta_theta;
	odom_goc=the_ta*rad_to_dr*10;
	x = pre_x + (cos(the_ta)*(delta_s));
	y = pre_y + (sin(the_ta)*delta_s);
	pre_the_ta=the_ta;  //gán lại tọa độ 
	pre_x=x; 
	pre_y=y;
	
}

void imu_data()
{
     //=====================imu====================//
    measurement_time_imu = ros::Time::now();
    dt=(measurement_time-pre_measurement_time_imu).toSec();
    geometry_msgs::Quaternion imu_quad= tf::createQuaternionMsgFromYaw(goc*rad);
    imu.header.stamp = measurement_time_imu;
    imu.header.frame_id = "base_link";
    imu.orientation=imu_quad;
    imu.angular_velocity.z=((goc-pre_goc)*rad)/dt;
    imu_pub.publish(imu);
    pre_measurement_time_imu=measurement_time_imu;
    pre_goc=goc;
    ROS_INFO("%d",goc);
}

void odom()
{
    measurement_time=ros::Time::now(); 
    delta();
    odometry();
    dt=(measurement_time-pre_measurement_time).toSec();
    geometry_msgs::Quaternion odom_quat= tf::createQuaternionMsgFromYaw(odom_goc*rad);

    nav_msgs::Odometry odom;
    odom.header.stamp=measurement_time;
    odom.header.frame_id = fixed_frame_odom;

    odom.pose.pose.position.x=x;
    odom.pose.pose.position.y=y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    
    odom.twist.twist.linear.y=0;
    odom.twist.twist.angular.z=delta_theta/dt;
    if((en1>0&&en2>0)||(en1<0&&en2<0))
    {
        odom.twist.twist.linear.x=0;
    }
    else
    {
       odom.twist.twist.linear.x=delta_s/dt;
    }
    
   
    odom.child_frame_id="base_link";

    odom_pub.publish(odom);
   
    pre_measurement_time=measurement_time;
   
}


//ket noi serial
void ket_noi_serial()
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
        ros::Duration(5).sleep();
    }
    if (ser.isOpen())
    {
        ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
    }
}



void read_serial()
{
        while (ser.available())
        {
            ser.read(&rec_data,1);
            if(count==0&&rec_data!=0x02)
                break;
            if(count==7&&rec_data!=0x03)
                break;
            if(count==7&&rec_data==0x03)
            {
                goc=buff_data[1]<<8|buff_data[2];
                en1=buff_data[3]<<8|buff_data[4];
                en2=buff_data[5]<<8|buff_data[6];
                odom();
                imu_data();
                count=0;
            }
            else
            {
                buff_data[count]=rec_data;
                count++;
            }
        }  
}


uint8_t byte_control[7]={0x02,0,0,0,0,0,0x03};

void write_driver(uint8_t byte_reset, int16_t v_banh_trai,int16_t v_banh_phai)
{
    uint8_t dir_trai,dir_phai;
    
    if(v_banh_trai<0)
    {
        dir_trai=1;
        v_banh_trai=-v_banh_trai;

    }
    else if(v_banh_trai>0)
    {
        dir_trai=0;
    }
    if(v_banh_trai>255)
        v_banh_trai=255;

    if(v_banh_phai<0)
    {
        dir_phai=1;
        v_banh_phai=-v_banh_phai;

    }
    else if(v_banh_phai>0)
    {
        dir_phai=0;
    }
    if(v_banh_phai>255)
        v_banh_phai=255;


    byte_control[1]=byte_reset;

    byte_control[2]=(dir_trai<<7)|0x01;
    byte_control[3]=v_banh_trai;

    byte_control[4]=(dir_phai<<7)|0x02;
    byte_control[5]=v_banh_phai;

    ser.write(byte_control,7);

}



void read_v(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    ROS_INFO("%d,%d",msg->data[0],msg->data[1]);
    write_driver(0,msg->data[0],msg->data[1]);
}


int main(int argc, char **argv)
{
    //========================khai bao ban dau=====================================//
    ros::init(argc, argv, "kinetic_iv");//khai bao ten node 
    ros::NodeHandle node_xu_ly;
    ros::NodeHandle private_node_handle("~");//node param
    ros::NodeHandle n;
    

    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");//cong usb
    private_node_handle.param<std::string>("topic_odom", topic_odom, "odom_encoder");//cong usb
    private_node_handle.param<std::string>("topic_imu", topic_imu, "IMU");//cong usb
    private_node_handle.param<std::string>("fixed_frame_odom", fixed_frame_odom, "odom");

    private_node_handle.param<double>("ban_kinh_banh", ban_kinh_banh, 0.05);
    private_node_handle.param<double>("khoang_cach_banh", khoang_cach_banh, 0.38);
    private_node_handle.param<double>("so_xung_1_vong", so_xung_1_vong, 2000.0);
    private_node_handle.param<std::string>("topic_v", topic_v, "van_toc_banh");//cong usb

    odom_pub = n.advertise<nav_msgs::Odometry>(topic_odom, 10); //publish tọa độ x, y, theta cho follow qua topic odom
    imu_pub = n.advertise<sensor_msgs::Imu>(topic_imu, 10);
    ros::Subscriber sub = n.subscribe(topic_v, 1, read_v);

    ket_noi_serial();
    ros::Rate rate_sleep(100);
    write_driver(0x61,0,0);
  
    while (ros::ok())
    {
        read_serial();
        rate_sleep.sleep();
        ros::spinOnce();
       
    }

}
