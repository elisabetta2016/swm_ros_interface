//ROS
#include "ros/ros.h"
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

extern "C" {
	#include "swmzyre.h"
}

//C++
#include <vector>
#include <string>
#include <math.h>
#include "boost/thread.hpp"


using namespace std;

enum publishers_code {
	BG_GEOPOSE,
	WASP_GEOPOSE
};

class SwmRosInterfaceNodeClass {
	public:
		SwmRosInterfaceNodeClass();
		void run();
		void main_loop();

		//---Callbacks
		void readGeopose_publishSwm(const geographic_msgs::GeoPose::ConstPtr& msg);
		void readGeopose_publishSwm_wasp(const geographic_msgs::GeoPose::ConstPtr& msg);
    void readGeopose_publishSwm_donkey(const custom_msgs::gnssSample::ConstPtr& msg);
    //void readCameraObservations_publishSwm(const camera_handler_sherpa::Camera::ConstPtr& msg);
		//---

	protected:
		
		/*state here*/
		ros::NodeHandle _nh;
		ros::Subscriber subSelfGeopose_;
		ros::Subscriber subWaspGeopose_;
		ros::Subscriber subWaspCamera_;
    ros::Subscriber subDonkeyGPS_;

		ros::Publisher pubBgGeopose_;
		std::vector<publishers_code> publishers;
		std::vector<uint16_t> rate_publishers;
		std::vector<uint16_t> counter_publishers;

		double utcTimeInMiliSec;
		std::string ns;

		struct timeval tp;

		// swm
		char config_folder[255];
		char config_name[];
		char config_file;
		json_t * config;
		component_t *self;
		bool agent_initialized;
		//

    double rate;
		uint16_t counter_print;

private:

};


//---helper functions
void quat2DCM(double (&rot_matrix)[9], geometry_msgs::Quaternion quat);
//---
