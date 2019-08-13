

#include <algorithm>
#include <functional>
#include <queue>
#include <fstream> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <vector>

#include <image_transport/image_transport.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


//dji msg
//msgs
#include <dji_sdk/A3RTK.h>
#include <dji_sdk/A3GPS.h>
#include <dji_sdk/Acceleration.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/Compass.h>
#include <dji_sdk/FlightControlInfo.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/GlobalPosition.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/PowerStatus.h> 
#include <dji_sdk/RCChannels.h>
#include <dji_sdk/Velocity.h>
#include <dji_sdk/Waypoint.h>
#include <dji_sdk/WaypointList.h>
#include <dji_sdk/TransparentTransmissionData.h>
#include <dji_sdk/TimeStamp.h>
#include <optitrack/Vel_Alt.h>

using namespace std;
using namespace cv;
using namespace ros;



//
int timestamp;
float pitch, yaw, roll;
/*
void djigimbal_callback(const dji_sdk::Gimbal& rc_gimbal)
{
  timestamp= rc_gimbal.ts;
  pitch = rc_gimbal.pitch;
 yaw = rc_gimbal.yaw;
 roll = rc_gimbal.roll;
cout<<"timestamp: "<<timestamp<<"pitch: "<< pitch<<"yaw:"<<yaw<<"roll: "<<roll<<endl;
}*/
float position_x,position_y, position_z;
void djiposition_callback(const dji_sdk::LocalPosition& rc_position)
{
  timestamp= rc_position.header.stamp;
  position_x = rc_position.x;
 position_y = rc_position.y;
 position_z = rc_position.z;
cout<<"timestamp: "<<timestamp<<"position_x: "<< position_x <<"position_y :"<<position_y <<"position_z : "<<position_z <<endl;
}
float optiposition_x,optiposition_y, optiposition_z;
void optitrack_callback(const optitrack::Vel_Alt& rc_optiposition)
{
  //timestamp= rc_position.header.stamp;
  optiposition_x = rc_optiposition.X;
 optiposition_y = rc_optiposition.Y;
 optiposition_z = rc_optiposition.Z;
cout<<"optiposition_x: "<< optiposition_x<<"optiposition_x :"<<optiposition_y <<"optiposition_x : "<<optiposition_z <<endl;
}

int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "data");
  ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe("sonar",10,sonarcallback);
//  ros::Subscriber subimu = n.subscribe("xsens_data",1000,IMUcallback); 
 // ros::Subscriber   sub_height = n.subscribe("height",1,heightCallback); 
  image_transport::ImageTransport it_image0(n);
  //image_transport::Subscriber sub0 = it_image0.subscribe("Camera/IDS_RGB_image",3,IDS_image_callback0);
  //image_transport::Subscriber sub0 = it_image0.subscribe("dji_sdk/image_raw",3,manifold_image_callback);
  ros::Subscriber   sub_optitrack  = n.subscribe("/optitrack/Vel_Alt",10,optitrack_callback);
  ros::Subscriber   sub_position  = n.subscribe("dji_sdk/local_position",10,djiposition_callback);

  ros::Rate loop_rate(20);
  int Num = 0;
  
   time_t tt = time(NULL);
    tm* t = localtime(&tt);
    FILE* fp;
    char fname[50];
    sprintf(fname,"DATA-%d%d%d%d.txt",t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min);
   // printf("%s\n",fname);
    if((fp = fopen(fname,"w+"))==NULL)
         printf("open failed\n");
    fclose(fp);
    ofstream write;
    write.open(fname);
   while(ros::ok())
   {
     
   
  write<<timestamp<<" "<<position_x<<"  "<<position_y<<"  "<<position_z<<"  "<<optiposition_x<<" "<<optiposition_y<<"     "<<optiposition_z<<endl;

    ros::spinOnce();
    loop_rate.sleep(); 

  }//end while(ros::ok()) 
 
   
   return 0;
}




















