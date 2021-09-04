#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

std::string topic_pose,topic_v;
double toa_do_x,toa_do_y,angle,z,w;
double rad_to_dr = 180/3.141592653589793238462643;//chuyen doi sang radian
double ka;
double van_toc_dai,van_toc_goc, v_max_xe, timeup;
double rad=3.14159265359/180;
ros::Publisher  pub_v;
geometry_msgs::Twist data; 

uint8_t i =0;

double r,l;
double Xg, Yg, _Xg, _Yg;
double a_r, b_r, c_r;
double a_c, b_c, c_c;
double chieu_quay;

double x_c, y_c, r_c;
double xg1,xg2,yg1,yg2;

void convert_to_euler()
{
  angle=atan2(2*w*z,1-2*z*z)*rad_to_dr;
}

// void read_pose(const nav_msgs::Odometry::ConstPtr& msg)
// {

//   toa_do_x=msg->pose.pose.position.x;
//   toa_do_y=msg->pose.pose.position.y;
//   z=msg->pose.pose.orientation.z;
//   w=msg->pose.pose.orientation.w;
//   convert_to_euler();

// }

void read_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
  toa_do_x=msg->position.x;
  toa_do_y=msg->position.y;
  z=msg->orientation.z;
  w=msg->orientation.w;
  convert_to_euler();

}

void pt_duong_tron_tam_r(double x_r, double y_r)
{
    a_r = -2*x_r; b_r=-2*y_r; c_r=x_r*x_r + y_r*y_r - l*l;
    // ROS_INFO("a_r=%lf, b_r=%lf, c_r=%lf",a_r,b_r,c_r);
}

void pt_duong_tron_qd()
{
    a_c= -2*x_c; b_c=-2*y_c; c_c=x_c*x_c + y_c*y_c - r_c*r_c;
    // ROS_INFO("a_r=%lf, b_r=%lf, c_r=%lf",a_c,b_c,c_c);
}

void find_point_ahead()
{

    double la = 1 + ((a_r-a_c)/(b_r-b_c))*((a_r-a_c)/(b_r-b_c));
    double lb = -2*((c_c-c_r)/(b_r-b_c))*((a_r-a_c)/(b_r-b_c)) + a_c - b_c*((a_r-a_c)/(b_r-b_c));
    double lc = ((c_c-c_r)/(b_r-b_c))*((c_c-c_r)/(b_r-b_c)) + b_c*((c_c-c_r)/(b_r-b_c)) + c_c;
    double denlta=lb*lb-4*la*lc;
    // ROS_INFO("la=%lf, lb=%lf, lc=%lf",la, lb, lc);

    xg1=(-lb+sqrt(denlta))/(2*la);
    yg1=((c_c-c_r)/(b_r-b_c)) - xg1*((a_r-a_c)/(b_r-b_c));
   

    xg2=(-lb-sqrt(denlta))/(2*la);
    yg2=((c_c-c_r)/(b_r-b_c)) - xg2*((a_r-a_c)/(b_r-b_c));
    // ROS_INFO("x_1=%lf, y_1=%lf, x_2=%lf, y_2=%lf",xg1, yg1, xg2, yg2);

    // loai nghiem
}

void chuyenhetoado(double x_hien_tai, double y_hien_tai, double angle_hien_tai)
{
  angle_hien_tai=-angle_hien_tai;
  
  double _Xg1=(xg1-x_hien_tai)*cosf(angle_hien_tai*rad)-(yg1-y_hien_tai)*sinf(angle_hien_tai*rad);
  double _Yg1=(xg1-x_hien_tai)*sinf(angle_hien_tai*rad)+(yg1-y_hien_tai)*cosf(angle_hien_tai*rad);

  double _Xg2=(xg2-x_hien_tai)*cosf(angle_hien_tai*rad)-(yg2-y_hien_tai)*sinf(angle_hien_tai*rad);
  double _Yg2=(xg2-x_hien_tai)*sinf(angle_hien_tai*rad)+(yg2-y_hien_tai)*cosf(angle_hien_tai*rad);
 
  if(_Xg1>0 and _Xg2>0)
  {
      if(_Xg1>_Xg2) {_Xg=_Xg1; _Yg=_Yg1;}
      else {_Xg=_Xg2; _Yg=_Yg2;}
  }
  else if(_Xg1>0 and _Xg2<0)
  {
      _Xg=_Xg1; _Yg=_Yg1;
  }
  else if(_Xg1<0 and _Xg2>0)
  {
      _Xg=_Xg2; _Yg=_Yg2;
  }
  
  l=sqrt(_Xg*_Xg+_Yg*_Yg);
}

void pure_pursuit()
{
  van_toc_dai=v_max_xe;
  if(_Yg==0) _Yg = 0.0001;
  r=(l*l)/(2*fabs(_Yg));
  van_toc_goc=van_toc_dai/r;
  if(_Yg>0) van_toc_goc=van_toc_goc;
  if(_Yg<0) van_toc_goc=-van_toc_goc;

}

void bamquydao()
{
 for( i; i<1; i++)
  {
    pt_duong_tron_qd();
  }
  pt_duong_tron_tam_r(toa_do_x,toa_do_y);
  find_point_ahead();
  chuyenhetoado(toa_do_x,toa_do_y,angle);
  pure_pursuit();
  data.linear.x=van_toc_dai;
  data.angular.z=van_toc_goc;
  ROS_INFO("X_robot = %lf, Y_robot = %lf, theta_robot=%lf, vt_dai=%lf, vt_goc=%lf",toa_do_x, toa_do_y, angle, van_toc_dai,van_toc_goc);
  pub_v.publish(data);

}

int main(int argc, char ** argv)
{
  
  ros::init(argc, argv, "go_to_point");
  ros::NodeHandle n;
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("topic_pose", topic_pose, "pose_robot");
  private_node_handle.param<std::string>("topic_v", topic_v, "cmd_vel");
  private_node_handle.param<double>("ka",ka,1.2);
  private_node_handle.param<double>("khoang_nhin_truoc",l,4);
  private_node_handle.param<double>("toa_do_tam_x",x_c,3);
  private_node_handle.param<double>("toa_do_tam_y",y_c,3);
  private_node_handle.param<double>("ban_kinh_duong_tron",r_c,1);
  private_node_handle.param<double>("v_max_xe",v_max_xe,0.3);
  private_node_handle.param<double>("timeup",timeup,0.5);
  ros::Subscriber sub = n.subscribe(topic_pose, 10, read_pose);
  pub_v = n.advertise<geometry_msgs::Twist>(topic_v, 10); 
  ros::Rate rate_sleep(500);

  pt_duong_tron_tam_r(0,0);
  pt_duong_tron_qd();
  find_point_ahead();
  while (ros::ok())
   {
    bamquydao();
    rate_sleep.sleep();
    ros::spinOnce(); 
   }

}