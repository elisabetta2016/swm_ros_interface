#include "swm_ros_interface.h"

using namespace std;


char* str2char( string str ) {
	char *c = new char[ str.length()+1 ];
	strcpy( c, str.c_str());
	return c;
}

SwmRosInterfaceNodeClass::SwmRosInterfaceNodeClass() {

	counter_print = 0;
	
	//---params:
	// pub: subscription to rostopic, advertising on the SWM
	// sub: subscription to the SWM, publishing on rostopic
	//--
	ns = ros::this_node::getNamespace();
	string nodename = ros::this_node::getName();
  pubSboxState    = _nh.advertise<sherpa_msgs::SboxStatus>("/Sbox_status",1);
  subDonkeyGPS_   = _nh.subscribe("/mti/sensor/gnssPvt", 0, &SwmRosInterfaceNodeClass::readGeopose_publishSwm_donkey,this);
  subDonkeyPower_ = _nh.subscribe("/RoverPowerInfo", 0, &SwmRosInterfaceNodeClass::readPower_publishSwm_donkey,this);

	//---publishers (TO SWM)
	if (_nh.hasParam(nodename + "/pub/publish_operator_geopose")) { //send the position of the bg to the SWM 
		subSelfGeopose_ = _nh.subscribe("/CREATE/human_pose", 0, &SwmRosInterfaceNodeClass::readGeopose_publishSwm,this);
		cout << "Subscribing: \t [operator geopose]: /CREATE/human_pose" << endl; 
	}

	if (_nh.hasParam(nodename + "/pub/wasp_geopose")) { //send the position of the wasp to the SWM 
		subWaspGeopose_ = _nh.subscribe("geopose", 0, &SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp,this);
		cout << "Subscribing: \t [wasp geopose]" << endl; 
	}


	//---
	if (_nh.hasParam(nodename + "/sub/publish_operator_geopose")){
		pubBgGeopose_ = _nh.advertise<geographic_msgs::GeoPose>("/bg/geopose",10);
		publishers.push_back(BG_GEOPOSE);
		rate_publishers.push_back(5);	//rate in Hz at which we want to read from SWM
		counter_publishers.push_back(0);
	} 

  rate = 5;	//TODO maybe pick rate of node as twice the highest rate of publishers

	//gettimeofday(&tp, NULL);

	//---SWM
	char* pPath;
	pPath = getenv ("UBX_ROBOTSCENEGRAPH_DIR");
	if (pPath==NULL) {
  	ROS_ERROR("UBX_ROBOTSCENEGRAPH_DIR env not set!");
  	exit(0);
  }
  
  string ubx_conf_path(pPath);
  ubx_conf_path += "/examples/zyre/swm_zyre_donkey2.json"; //the json name could be a param
	config = load_config_file(str2char(ubx_conf_path));//"swm_zyre_config.json");
	
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

void SwmRosInterfaceNodeClass::readGeopose_publishSwm(const geographic_msgs::GeoPose::ConstPtr& msg){
	//ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
	//utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
  gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!
	
	string agent_name = "busy_genius";
	update_pose(self, matrix, utcTimeInMiliSec, str2char(agent_name) );
}

void SwmRosInterfaceNodeClass::readGeopose_publishSwm_wasp(const geographic_msgs::GeoPose::ConstPtr& msg){
  // Mohsen Comments: I need to adapt it with rover, the structure remains the same
	gettimeofday(&tp, NULL);
	utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000; //get current timestamp in milliseconds
	//ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
	//utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;
	double rot_matrix[9];
	quat2DCM(rot_matrix, msg->orientation);
	double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
						   					rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
						   					rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
						   					msg->position.latitude, msg->position.longitude, msg->position.altitude, 1}; // y,x,z,1 remember this is column-major!
	
	update_pose(self, matrix, utcTimeInMiliSec, str2char(ns) );
}

void SwmRosInterfaceNodeClass::readGeopose_publishSwm_donkey(const custom_msgs::gnssSample::ConstPtr& msg)
{
  gettimeofday(&tp, NULL);
  utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  double rot_matrix[9] = {1,0,0,0,1,0,0,0,1}; // To Be Done
  double matrix[16] = { rot_matrix[0], rot_matrix[1], rot_matrix[2], 0,
                        rot_matrix[3], rot_matrix[4], rot_matrix[5], 0,
                        rot_matrix[6], rot_matrix[7], rot_matrix[8], 0,
                        msg->latitude, msg->longitude, msg->hEll, 1};
  update_pose(self, matrix, utcTimeInMiliSec, str2char(ns) );
  //ROS_INFO("Sending donkey pose to SWM");
}

void SwmRosInterfaceNodeClass::readPower_publishSwm_donkey(const donkey_rover::Rover_Power_Data::ConstPtr& msg)
{
  gettimeofday(&tp, NULL);
  utcTimeInMiliSec = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  ROS_INFO("BATTERY UPDATE");
  string batt_status = "HIGH";
  if(msg->Battery_Voltage < 51) batt_status = "MID";
  if(msg->Battery_Voltage < 47) batt_status = "LOW";
  add_battery(self, msg->Battery_Voltage,str2char(batt_status),utcTimeInMiliSec,str2char(ns));
}

void SwmRosInterfaceNodeClass::readSWM_publishRos()
{
   char* sbox_name = NULL;
   if(get_sherpa_box_status(self, sbox_stat, sbox_name))
   {
     sherpa_msgs::SboxStatus sbox_msg;
     sbox_msg.header.stamp = ros::Time::now();
     sbox_msg.commandStep = sbox_stat->commandStep;
     sbox_msg.completed = sbox_stat->completed;
     sbox_msg.executeId = sbox_stat->executeId;
     sbox_msg.idle = sbox_stat->idle;
     sbox_msg.linActuatorPosition = sbox_stat->linActuatorPosition;
     sbox_msg.waspDockLeft = sbox_stat->waspDockLeft;
     sbox_msg.waspDockRight = sbox_stat->waspDockRight;
     sbox_msg.waspLockedLeft = sbox_stat->waspLockedLeft;
     sbox_msg.waspLockedRight = sbox_stat->waspLockedRight;
     pubSboxState.publish(sbox_msg);
   }
   else
     ROS_ERROR("acquiring Sbox status from SWM was unsuccessful");
}


void quat2DCM(double (&rot_matrix)[9], geometry_msgs::Quaternion quat){
	
	rot_matrix[0] = 1-2*(quat.y*quat.y+quat.z*quat.z);
	rot_matrix[1] = 2*(quat.x*quat.y-quat.w*quat.z);
	rot_matrix[2] = 2*(quat.x*quat.z+quat.w*quat.y);
	rot_matrix[3] = 2*(quat.x*quat.y+quat.w*quat.z);
	rot_matrix[4] = 1-2*(quat.x*quat.x+quat.z*quat.z);
	rot_matrix[5] = 2*(quat.y*quat.z-quat.w*quat.x);
	rot_matrix[6] = 2*(quat.x*quat.z-quat.w*quat.y);
	rot_matrix[7] = 2*(quat.y*quat.z+quat.w*quat.x);
	rot_matrix[8] = 1-2*(quat.x*quat.x+quat.y*quat.y);
	
}

void SwmRosInterfaceNodeClass::main_loop()
{

	ros::Rate r(rate);

	//---msgs
	geographic_msgs::GeoPose gp;
	//---

	while( ros::ok() ) {

		ros::Time time = ros::Time::now();	//TODO probably this is not system time but node time...to check
		utcTimeInMiliSec = time.sec*1000000.0 + time.nsec/1000.0;

		counter_print++;
    readSWM_publishRos();

		for (int i=0; i<publishers.size(); i++){
			
			counter_publishers[i]++;

			switch (publishers[i]){
				case BG_GEOPOSE:
					if (counter_publishers[i] >= rate/rate_publishers[i]){
						string agent_name = "busy_genius";
						double x,y,z;
						//get_position(self, &gp.position.latitude, &gp.position.longitude, &gp.position.altitude, utcTimeInMiliSec, str2char(agent_name));	
						//get_position(self, &x, &y, &z, utcTimeInMiliSec, "test");
						pubBgGeopose_.publish( gp );
						counter_publishers[i] = 0;
					}
					break;
			}						
		}


    //Mohsen Stuff
		r.sleep();
	}
}




void SwmRosInterfaceNodeClass::run() {

	ros::Rate loop_rate(rate);	
	ROS_INFO_ONCE("SIM: RUNNING");

	//start loop handle as a thread. The execution time is specified by rate parameter
	boost::thread loop_handle_t( &SwmRosInterfaceNodeClass::main_loop, this);
  ros::spin();

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "swm_ros_interface");	
	SwmRosInterfaceNodeClass swm_ros_interfaceNode;
	swm_ros_interfaceNode.run();

	return 0;
}
