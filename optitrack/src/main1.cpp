#include "ros/ros.h"
#include "std_msgs/String.h"
#include <optitrack/mymulticastoptitrack.h>
#include <cstring>
#include <sstream>
#include <string.h>

#include "optitrack/Vel_Alt.h"

#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <time.h>  
#include <string.h>  
#include <stdio.h>  
#include <unistd.h>  
#include <stdlib.h>  
#include <sys/ioctl.h>  
  
#define MSGBUFSIZE 256*2
//#define TRAM_STATUS_ADDR "192.168.0.190" 
#define TRAM_STATUS_ADDR "239.255.42.99" 
#define TRAM_STATUS_RECV_PORT 1513

optitrack::Vel_Alt opt_speed;
void uavSpeed(MyMultiCastOptitrack OptitrackData);
void speedRestrict(float &vx,float &vy);
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "mymulticastoptitrack");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<optitrack::Vel_Alt>("/optitrack/Vel_Alt", 1);

  ros::Rate loop_rate(120);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

 struct sockaddr_in addr;  
  int fd, nbytes, addrlen;  
  struct ip_mreq mreq; 
  char msgbuf[MSGBUFSIZE];  
  int on; 
   opt_speed.Aim_X=1;
   opt_speed.Aim_Y=1;
   opt_speed.Aim_Z=1.5;
 MyMultiCastOptitrack OptitrackData;
       /* 初始化地址 */  
  memset(&addr, 0, sizeof(addr));  
  addr.sin_family = AF_INET;  
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  
  addr.sin_port = htons(TRAM_STATUS_RECV_PORT); 
   
      /*加入多播组*/  
    mreq.imr_multiaddr.s_addr = inet_addr(TRAM_STATUS_ADDR);  
    mreq.imr_interface.s_addr = htonl(INADDR_ANY); 

if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)  
    {  
        perror("socket ");  
        return -1;  
    }  

  // 将套接字绑定到服务器的网络地址上  
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)  
    {  
        perror("bind");  
        return -1;  
    } 
 on = 1;
/* 设置地址复用许可, 根据具体情况判断是否增加此功能 */  
   if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)  
   {  
       perror("SO_REUSEADDR");  
        return -1;  
   } 

   
    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) 
    {  
        perror("setsockopt");  
        return -1;  
    } 





  while (ros::ok())
  {
  count++;
   //memset(msgbuf,0,MSGBUFSIZE);
   addrlen = sizeof(addr);   
   if ((nbytes = recvfrom(fd, msgbuf, MSGBUFSIZE, 0, (struct sockaddr *)&addr, (socklen_t *)&addrlen)) < 0)  
    {  
        perror("recvfrom");  
        return -1;  
    } 
//if(count%50==0)
//{
 //  printf("%d   \n",count);
//}//printf("recv ok!   ");  

   OptitrackData.SetUpMyDataTableWidget(msgbuf);
   opt_speed.X=OptitrackData.UavData.x;
   opt_speed.Y=OptitrackData.UavData.y;
   opt_speed.Z=OptitrackData.UavData.z;
   if(abs(opt_speed.X - opt_speed.Aim_X)<0.1&&abs(opt_speed.Y - opt_speed.Aim_Y)<0.1)
   {
	   opt_speed.Aim_X = 2.2;
	   opt_speed.Aim_Y = 2.2;
   }
   uavSpeed(OptitrackData);
   if(count%50==0)
{
   printf("%f %f %f %f\n",opt_speed.X,opt_speed.Y,opt_speed.Vx,opt_speed.Vy); 
}

 if(count%3==0)
   {
   chatter_pub.publish(opt_speed);
   ros::spinOnce();
   }

   loop_rate.sleep();
  }

/*退出多播组*/ 
   setsockopt(fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));  
   close(fd);   
return 0;
  }

void uavSpeed(MyMultiCastOptitrack OptitrackData)
{
float delta_x=-OptitrackData.UavData.x+opt_speed.Aim_X;
float delta_y=-OptitrackData.UavData.y+opt_speed.Aim_Y;
float delta_z=-OptitrackData.UavData.z+opt_speed.Aim_Z;
if(abs(delta_x)<0.1)
 {
  opt_speed.Vx=0.0;
if(abs(delta_y)<0.1)
 {
  opt_speed.Vy=0.0;
 }
else if(abs(delta_y)<0.3)
 {
  opt_speed.Vy=delta_y*0.3;
 }
else
{
	opt_speed.Vy = delta_y*0.5;
}
 }

else if(abs(delta_x)<0.3)
 {
  opt_speed.Vx=delta_x*0.3;
  opt_speed.Vy=0.0;
 }

else
{
	opt_speed.Vx = delta_x*0.5;
        opt_speed.Vy=0.0;
}

speedRestrict(opt_speed.Vx,opt_speed.Vy);
}

void speedRestrict(float &vx,float &vy)
{
 if(vx>0.5)
   {
   vx=0.5;
    }
 if(vx<-0.5)
{
vx=-0.5;
}
if(vy>0.5)
	{
	 vy=0.5;
    }
if(vy<-0.5)
	{
	 vy=-0.5;
	}
}
