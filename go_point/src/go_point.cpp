#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


std::string topic_pose,topic_v;
double toa_do_x,toa_do_y,angle,z,w;
double rad_to_dr = 180/3.14159265359;//chuye141592653589793238462643n doi sang radian
double ka;
double van_toc_dai,van_toc_goc, v_max_xe, timeup;
double rad=3.14159265359/180;
ros::Publisher  pub_v;
geometry_msgs::Twist data; 

double L = 0.3, kc, r, l;
double a,b,c;
double an,bn,cn;
double Xn, Yn, Xg, Yg, _Xg, _Yg;
uint8_t m=0;

#define number_line  5
// toa do quanh a10
double toado[number_line][2]={{0,0},
                     {7,0},
                     {7,5},
                     {0,5},
                     {0,0}
                         };
//toa do 1 loi a10

// double toado[number_line][2]={{0,-0.5},
//                      {26,-0.5},
//                      {26,0.5},
//                      {0,0.5},
//                      {0,-0.5}
//                          };                         

uint8_t i=0;

void convert_to_euler()
{
  angle=atan2(2*w*z,1-2*z*z)*rad_to_dr;
}



void read_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
  toa_do_x=msg->position.x;
  toa_do_y=msg->position.y;
  z=msg->orientation.z;
  w=msg->orientation.w;
  convert_to_euler();
}

// void read_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//   toa_do_x=msg->pose.position.x;
//   toa_do_y=msg->pose.position.y;
//   z=msg->pose.orientation.z;
//   w=msg->pose.orientation.w;
//   convert_to_euler();
// }


void pt_duong_thang(double x1, double y1, double x2, double y2)
{
  a=y1-y2; b=x2-x1; c= -x1*a-y1*b;
}

void pt_duong_thang_RN(double x, double y)
{
  an=b; bn=-a; cn= -x*an-y*bn;
}

void toadodiemN()
{
  Xn=((cn*b)-(c*bn))/((a*bn)-(b*an));
  Yn=((cn*a)-(c*an))/((an*b)-(bn*a));
}

void find_point_ahead(double x_c, double y_c)
{
  double xg1,xg2,yg1,yg2;
  if(b==0)
  {
    Xg=xg1=xg2=-c/a;
    yg1=-sqrt(L*L-(Xg-Xn)*(Xg-Xn))+Yn;
    yg2=sqrt(L*L-(Xg-Xn)*(Xg-Xn))+Yn;
  }
  else
  {
    double la=(1+(a/b)*(a/b));
    double lb=-2*(Xn-(a/b)*((c/b)+Yn));
    double lc=Xn*Xn+((c/b)+Yn)*((c/b)+Yn)-L*L;
    double denlta=lb*lb-4*la*lc;

    xg1=(-lb+sqrt(denlta))/(2*la);
    xg2=(-lb-sqrt(denlta))/(2*la);

    yg1=(-c-a*xg1)/b;
    yg2=(-c-a*xg2)/b;
  }

  double kc1=sqrt((x_c-xg1)*(x_c-xg1)+(y_c-yg1)*(y_c-yg1));
  double kc2=sqrt((x_c-xg2)*(x_c-xg2)+(y_c-yg2)*(y_c-yg2));
  
  if(kc1<kc2)
  {
    Xg=xg1; Yg=yg1;
  }
  else
  {
    Xg=xg2; Yg=yg2;
  }

  kc=sqrt((x_c-Xn)*(x_c-Xn)+(y_c-Yn)*(y_c-Yn));

  if(kc<L and m==(number_line-2))
  {
    Xg=toado[m][0];
    Yg=toado[m][1];
    van_toc_dai = ka*kc;
    if(van_toc_dai > v_max_xe) van_toc_dai=v_max_xe;
    if(kc<0.03) {van_toc_dai = 0;van_toc_goc=0;}
  }
  else
  {
    //pt_giatoc();
     van_toc_dai=v_max_xe;

  } 
}

void chuyenhetoado(double x_hien_tai, double y_hien_tai, double angle_hien_tai)
{
  angle_hien_tai=-angle_hien_tai;
  
  _Xg=(Xg-x_hien_tai)*cosf(angle_hien_tai*rad)-(Yg-y_hien_tai)*sinf(angle_hien_tai*rad);
  _Yg=(Xg-x_hien_tai)*sinf(angle_hien_tai*rad)+(Yg-y_hien_tai)*cosf(angle_hien_tai*rad);

  l=sqrt(_Xg*_Xg+_Yg*_Yg);
}

void pure_pursuit()
{
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
    pt_duong_thang(toado[m][0], toado[m][1], toado[m+1][0], toado[m+1][1]);
  }
  pt_duong_thang_RN(toa_do_x,toa_do_y);
  toadodiemN();
  find_point_ahead(toado[m+1][0], toado[m+1][1]);
  if(kc<L and m!=(number_line-2))
  {
    m++; i=0;
  }
  else
  {
    chuyenhetoado(toa_do_x, toa_do_y, angle);
    pure_pursuit();
    data.linear.x=van_toc_dai;
    data.angular.z=van_toc_goc;
    ROS_INFO("X_robot = %lf, Y_robot = %lf, theta_robot=%lf, vt_dai=%lf, vt_goc=%lf",toa_do_x, toa_do_y, angle, van_toc_dai,van_toc_goc);
    pub_v.publish(data);
  }
   
}

int main(int argc, char ** argv)
{
  
  ros::init(argc, argv, "go_to_point");
  ros::NodeHandle n;
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("topic_pose", topic_pose, "pose_robot");
  private_node_handle.param<std::string>("topic_v", topic_v, "cmd_vel");
  private_node_handle.param<double>("ka",ka,1.2);
  private_node_handle.param<double>("v_max_xe",v_max_xe,0.3);
  private_node_handle.param<double>("timeup",timeup,0.5);
  ros::Subscriber sub = n.subscribe(topic_pose, 10, read_pose);
  pub_v = n.advertise<geometry_msgs::Twist>(topic_v, 10); 
  ros::Rate rate_sleep(500);
  while (ros::ok())
   {
    bamquydao();
    rate_sleep.sleep();
    ros::spinOnce(); 
   }

}

