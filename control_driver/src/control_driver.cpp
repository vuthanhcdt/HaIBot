/*******************************************************************/
/****************************HaIBot Lab****************************/
/**************************Vu Cong Thanh**************************/

#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int16MultiArray.h"

//khai bao serial
std::string port;
serial::Serial ser;
std::string topic_v;




void write_driver(int16_t van_toc, uint8_t diachi)
{
    uint8_t chieu_quay;
    uint8_t byte_gui[3]={0,0,0xff};
    if(van_toc<0)
    {
        chieu_quay=1;
        van_toc=-van_toc;

    }
    else if(van_toc>0)
    {
        chieu_quay=0;
    }
    if(van_toc>255)
        van_toc=255;
    byte_gui[0]=chieu_quay<<7|diachi;
    byte_gui[1]=van_toc;
    ser.write(byte_gui,3);

}

void read_v(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    ROS_INFO("%d,%d",msg->data[0],msg->data[1]);
    write_driver(msg->data[0],1);
    write_driver(msg->data[1],2);
}


//ket noi serial
void ket_noi_serial()
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(19200);
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




int main(int argc, char **argv)
{
    //========================khai bao ban dau=====================================//
    ros::init(argc, argv, "control_driver");//khai bao ten node 
    ros::NodeHandle node_xu_ly;
    ros::NodeHandle private_node_handle("~");//node param
    ros::NodeHandle n;
    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");//cong usb
    private_node_handle.param<std::string>("topic_v", topic_v, "van_toc_banh");//cong usb
    ros::Rate rate_sleep(20);
    ros::Subscriber sub = n.subscribe(topic_v, 1, read_v);
    ket_noi_serial();
    ros::spin();

}