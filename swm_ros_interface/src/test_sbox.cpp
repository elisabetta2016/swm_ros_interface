//ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geographic_msgs/GeoPose.h"
#include "geographic_msgs/GeoPoint.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <zyre.h>
#include <jansson.h>
#include <uuid/uuid.h>
#include <string.h>
#include <sys/time.h>

//Messages
#include <custom_msgs/gnssSample.h>
#include <donkey_rover/Rover_Power_Data.h>
#include <sherpa_msgs/SboxStatus.h>

extern "C" {
  #include "swmzyre.h"
}

//C++
#include <vector>
#include <string>
#include <math.h>
#include "boost/thread.hpp"

using namespace std;
char* str2char( string str ) {
  char *c = new char[ str.length()+1 ];
  strcpy( c, str.c_str());
  return c;
}


class swmAgentTester
{
public:
   swmAgentTester(string mode, ros::NodeHandle& node)
   {

     node = nh_;
     rate = 5.0;
     //---SWM
     char* pPath;
     pPath = getenv ("UBX_ROBOTSCENEGRAPH_DIR");
     if (pPath==NULL) {
       ROS_ERROR("UBX_ROBOTSCENEGRAPH_DIR env not set!");
       ROS_WARN("Please set the environment variable using the following example \n ### export UBX_ROBOTSCENEGRAPH_DIR=/home/sherpa/swm/ubx_robotscenegraph  ###");
       exit(0);
     }
     string ubx_conf_path(pPath);
     if(mode.compare("sbox0") == 0)
     {

       ubx_conf_path += "/examples/zyre/swm_zyre_sbox.json"; //the json name could be a param; it should be ;-)
       config = load_config_file(str2char(ubx_conf_path));//"swm_zyre_config.json");

     }


     if (config == NULL) {
       ROS_INFO("Unable to load config");
       return;
     }
     self = new_component(config);
     if (self == NULL) {
       ROS_INFO("Unable to initialize component");
       return;
     }
     //---

     agent_initialized = false;

     //Initialize agent
     double matrix[16] = { 1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1}; // y,x,z,1 remember this is column-major!

     //string agent_name = "busy_genius";
     //assert(add_agent(self, matrix, 0.0, str2char(ns) ));
     add_agent(self, matrix, 0.0, str2char(ns) );

     cout << "NS: " << ns << endl;
   }
   void set_sbox()
   {
     //Just an example
     sbox_stat.completed = 0;
     sbox_stat.idle = 1;
     sbox_stat.executeId = 31;
     sbox_stat.waspDockLeft = false;
     sbox_stat.waspDockRight = true;
     sbox_stat.waspLockedLeft = false;
     sbox_stat.waspLockedRight = true;
   }

   void run()
   {

     ros::Rate r(rate);

     //---msgs
     geographic_msgs::GeoPose gp;
     //---

     set_sbox();
     while( nh_.ok() ) {

       ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
       utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
       if(!add_sherpa_box_status(self,sbox_stat,str2char("sbox0")))// the code crashes here, invalid pointer
          ROS_ERROR("could not add sbox data");

       //Mohsen Stuff
       ros::spinOnce();
       r.sleep();
     }
   }


protected:
   ros::NodeHandle nh_;
   double utcTimeInMiliSec;
   double rate;
   string ns;

   struct timeval tp;

   // swm
   char config_folder[255];
   char config_name[];
   char config_file;
   json_t * config;
   component_t *self;
   sbox_status sbox_stat;
   bool agent_initialized;
   //

   uint16_t counter_print;
private:

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "sbox_test");
  string mode = "sbox0";
  ros::NodeHandle node;
  swmAgentTester tester(mode,node);
  tester.run();

  return 0;
}
