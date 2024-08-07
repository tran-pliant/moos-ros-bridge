//* The MOOS ROS Bridge
/**
 *
 * @file
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 *
 * @version 1.0
 *
 * @date May 25th, 2012
 *
 * @section LICENSE
 *
 * Georgia Tech Research Institute (GTRI)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * The MOOS ROS Bridge allows for communication between a
 * MOOS Database and a ROS Core.
 */

#include <moos-ros-bridge/MOOSNode.h>
#include <moos-ros-bridge/MsgContainer.h>
#include "base64_utils/base64.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CompressedImage.h"
#include <vector>
#include <chrono>
#include <ctime>
#include <cmath>

#define _USE_MATH_DEFINES


//default constructor
MOOSNode::MOOSNode()
{
}

void MOOSNode::AssignPublisher(vector<MsgContainer> *new_msgVec){
     msgVec = new_msgVec;
}

//default (virtual) destructor
MOOSNode::~MOOSNode(){
}

bool MOOSNode::toMOOS(std::string moosName, double data){
     m_Comms.Notify(moosName,data,MOOSTime());
     return true;
}

bool MOOSNode::toMOOS(std::string moosName, std::string myString){
     m_Comms.Notify(moosName,myString,MOOSTime());
     return true;
}

//Support for binary-string from ROS to MOOS
bool MOOSNode::toMOOSBinaryString(std::string moosName, std::string myString){
    m_Comms.Notify(moosName,(void *)myString.c_str(),myString.size(),MOOSTime());
	return true;
}

bool MOOSNode::OnNewMail (MOOSMSG_LIST &NewMail){
     MOOSMSG_LIST::iterator p;
     for( p = NewMail.begin() ; p != NewMail.end() ; p++ ){

	  CMOOSMsg & rMsg = *p;
	  string key = rMsg.GetKey();

	  // These are systems key checks, the msgvec iterator below assigns publishers to each
	  // msg found, we don't need that for these...?
	  // Special cases:
	  if( key == "NAV_X") {
		if (rMsg.GetDouble() != 0) { // need to check if nav_x has proper value
			// std::cout << "received nav_x" << std::endl;
			nav_x = rMsg.GetDouble();
			MOOSNode::handle_odom_msg(key);
		}
	  }
	  else if( key == "NAV_Y") {
		if (rMsg.GetDouble() != 0) { // need to check if nav_y has proper value
			// std::cout << "received nav_y" << std::endl;
			nav_y = rMsg.GetDouble();
			MOOSNode::handle_odom_msg(key);
		}
	  }
	  else if( key == "NAV_DEPTH") {
		if (rMsg.GetDouble() != 0) {
			// std::cout << "received nav_depth" << std::endl;
			nav_z = -(rMsg.GetDouble()); // depth is positive down
			MOOSNode::handle_odom_msg(key);
		}
	  }
	  else if( key == "NAV_ROLL") {
		if (rMsg.GetDouble() != 0) {
			// std::cout << "received nav_roll" << std::endl;
			nav_roll = rMsg.GetDouble();
			MOOSNode::handle_odom_msg(key);
		}
	  }
	  else if( key == "NAV_PITCH") {
		if (rMsg.GetDouble() != 0) {
			// std::cout << "received nav_pitch" << std::endl;
			nav_pitch = rMsg.GetDouble();
			MOOSNode::handle_odom_msg(key);
		}
	  }
	  else if( key == "NAV_HEADING") {
		if (rMsg.GetDouble() != 0) {
			// std::cout << "received nav_yaw" << std::endl;
			nav_yaw = rMsg.GetDouble();
			MOOSNode::handle_odom_msg(key);
		}
	  }
	//   else if (key == "RAW_IMU_DATA") {
    //     // cray::protobuf::CRayFrontseat raw_msg_;
    //     // cray::util::parse_from_moos_message(rMsg.GetString(), &raw_msg_);
    //     // if (raw_msg_.has_imu_data()){
    //     //   MOOSNode::handle_raw_imu_data(raw_msg_.time(), raw_msg_.imu_data());
	// 	//   MOOSNode::handle_odom_msg(key);
    //     // }
    //   }
	//   else if( key == "RAW_DEPTH_DATA"){
	//   	// cray::protobuf::CRayFrontseat raw_msg_;
	// 	// cray::util::parse_from_moos_message(rMsg.GetString(), &raw_msg_);
	// 	// if (raw_msg_.has_depth_data()){
	// 	// 	MOOSNode::handle_raw_depth_data(raw_msg_.time(), raw_msg_.depth_data());
	// 	// 	MOOSNode::handle_odom_msg(key);
	// 	// }
	//   }

	  //Every time a new type is added, a new IF statement is
	  //required here to handle the conversion


	  // right now, this publishes everything in moos network to ROS
	  // Need to handle incoming messages, and package accordingly

	  vector<MsgContainer>::iterator it;

	  for ( it = msgVec->begin() ; it < msgVec->end() ; it++ ){
	       if(MOOSStrCmp(rMsg.GetKey(), it->moosName)){
		    if(rMsg.IsDouble() && (strcmp(it->moosType.c_str(),"Double") == 0) ){
			 if( it->rosType == "std_msgs/Float64"){
				if (rMsg.GetKey() == "SONAR_RANGE_RES") {
				std_msgs::Float64 sonar_range_res;
				sonar_range_res.data = rMsg.GetDouble();
				it->pub.publish(sonar_range_res);
				}
			 }
			//  }else if( it->rosType == "std_msgs/Float32"){
			//       std_msgs::Float32 dataFloat32;
			//       dataFloat32.data = rMsg.GetDouble();
			//       it->pub.publish(dataFloat32);
			//  }else if( it->rosType == "std_msgs/Int32"){
			//       std_msgs::Int32 dataInt32;
			//       dataInt32.data = rMsg.GetDouble();
			//       it->pub.publish(dataInt32);
			//  }else if( it->rosType == "std_msgs/Int64"){
			//       std_msgs::Int64 dataInt64;
			//       dataInt64.data = rMsg.GetDouble();
			//       it->pub.publish(dataInt64);
			//  }
		    }
		    if(rMsg.IsString() && (it->moosType  == "String")){
			  if( it->rosType == "nav_msgs/Odometry"){
				//   std::cout << "received dummy_var" << std::endl;
				  // only publish when odom_msg is filled out, then clear entries
				  // flip all switches to off so we don't send this if unfilled with new
				  if (std::all_of( std::begin(updatedOdom), std::end(updatedOdom), []( const bool v){ return v; } )) {
						// std::cout << "full odom check passed" << std::endl;
						it->pub.publish(odom_msg);
						// empty odom_msg updated check
						for (auto i : updatedOdom) {
							i = false;
					}
				  }
			  }
			  if(rMsg.GetKey() == "SONAR_REMAP_IMAGE"){
				  string sonarBin = base64_decode(rMsg.GetString());
				  // this is a vector returned from cv::imencode
				  std::vector<uint8_t> remapPingData(sonarBin.begin(), sonarBin.end());
				  sensor_msgs::CompressedImage remap_ping_msg;
				  remap_ping_msg.header.stamp = ros::Time::now();
				  remap_ping_msg.data = remapPingData;
				  remap_ping_msg.format += "; jpeg compressed";
				  // copy result to a compressed image msg type
			      // it->pub.publish(ping image here);
				  // callback takes in image then does processing, then remaps
				  // we could probably just process the remapped from the get go...?
				  it->pub.publish(remap_ping_msg);
			  }
		    }

	       }//end if MOOSStrCmp
	  }//end Iterator for
     }//end Mailbox for
     return true;
}

/*
   called by the base class when the application has made contact with
   the MOOSDB and a channel has been opened . Place code to specify what
   notifications you want to receive here .
*/
bool MOOSNode::OnConnectToServer()
{
     DoRegistrations();
     return true;
}

/*
  Called by the base class periodically. This is where you place code
  which does the work of the application
*/
bool MOOSNode::Iterate(){
	// std::cout << "nax_x: " << updatedOdom[0] << std::endl;
	// std::cout << "nax_y: " << updatedOdom[1] << std::endl;
	// std::cout << "nax_z: " << updatedOdom[2] << std::endl;
	// std::cout << "nax_roll: " << updatedOdom[3] << std::endl;
	// std::cout << "nax_pitch: " << updatedOdom[4] << std::endl;
	// std::cout << "nax_yaw: " << updatedOdom[5] << std::endl;
	// Notify("DUMMY_VAR", "sendOdom");
    return true;
}

/*
   called by the base class before the first :: Iterate is called . Place
   startup code here − especially code which reads configuration data from the
   mission file
*/
bool MOOSNode::OnStartUp()
{
     appTick = 1;
     commsTick = 1;

     if(!m_MissionReader.GetConfigurationParam("AppTick",appTick)){
	  MOOSTrace("Warning, AppTick not set.\n");
     }

     if(!m_MissionReader.GetConfigurationParam("CommsTick",commsTick)){
	  MOOSTrace("Warning, CommsTick not set.\n");
     }

     SetAppFreq(appTick);
     SetCommsFreq(commsTick);

     DoRegistrations();

     return true;
}

void MOOSNode::DoRegistrations(){
     vector<MsgContainer>::iterator it;
     for ( it = msgVec->begin() ; it < msgVec->end() ; it++ ){
	  m_Comms.Register(it->moosName,0);

	  char buf[512];
	  sprintf(buf,"Subscribing to %s\n",it->moosName.c_str());
	  MOOSTrace(buf);
     }

	 Register("NAV_X", 0);
	 Register("NAV_Y", 0);
	 Register("NAV_DEPTH", 0);
	 Register("NAV_ROLL", 0);
	 Register("NAV_PITCH", 0);
	 Register("NAV_HEADING", 0);
	 Register("RAW_IMU_DATA", 0);
	 Register("RAW_DEPTH_DATA", 0);
}

// C-Ray protobuf helper stuff needs to be included somehow...

// //---------------------------------------------------------
// // Handle RAW_IMU_DATA data
// void MOOSNode::handle_raw_imu_data(double msg_time, const cray::protobuf::IMU& imu_raw_)
// {
//   if(imu_raw_.has_roll() && !std::isnan(imu_raw_.roll())){
//     rawIMURoll = imu_raw_.roll();
//   }
//   if(imu_raw_.has_pitch() && !std::isnan(imu_raw_.pitch())){
//     rawIMUPitch = imu_raw_.pitch();
//   }
//   if(imu_raw_.has_heading() && !std::isnan(imu_raw_.heading())){
//     rawIMUHeading = imu_raw_.heading();
//   }
// }

// //---------------------------------------------------------
// // Handle RAW_DEPTH data
// void MOOSNode::handle_raw_depth_data(double msg_time, const cray::protobuf::DEPTH& dep_raw_)
// {
//   if(dep_raw_.has_depth() && !std::isnan(dep_raw_.depth()))
//     nav_z = dep_raw_.depth();
// }

void MOOSNode::e2q(double y, double p, double r) {
	qx = sin(r/2) * cos(p/2) * cos(y/2) - cos(r/2) * sin(p/2) * sin(y/2);
	qy = cos(r/2) * sin(p/2) * cos(y/2) + sin(r/2) * cos(p/2) * sin(y/2);
	qz = cos(r/2) * cos(p/2) * sin(y/2) - sin(r/2) * sin(p/2) * cos(y/2);
	qw = cos(r/2) * cos(p/2) * cos(y/2) + sin(r/2) * sin(p/2) * sin(y/2);
}

void MOOSNode::handle_odom_msg(string arg){
	// parse pose, then construct odom msg to get arg fields
	// Construct odom msg
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";

	// fill out odom_msg
	
	// handling if we get raw imu data
	// if (arg == "IMU") {
	// 	if (updatedOdom[3] == false) {
	// 		MOOSNode::e2q(rawIMUHeading, rawIMUPitch, rawIMURoll);
	// 		odom_msg.pose.pose.orientation.x = qx;
	// 		odom_msg.pose.pose.orientation.y = qy;
	// 		odom_msg.pose.pose.orientation.z = qz;
	// 		odom_msg.pose.pose.orientation.w = qw;
	// 		updatedOdom[3] = true;
	// 	}
	// }
	if (arg == "NAV_X") {
		if (updatedOdom[0] == false) {
			odom_msg.pose.pose.position.x = nav_x;
			updatedOdom[0] = true;
			// std::cout << "set nav_x" << std::endl;
		}
	}
	else if (arg == "NAV_Y") {
		if (updatedOdom[1] == false) {
			odom_msg.pose.pose.position.y = nav_y;
			updatedOdom[1] = true;
			// std::cout << "set nav_y" << std::endl;
		}
	}
	// else if (arg == "RAW_DEPTH_DATA") {
	// 	if (updatedOdom[2] = false) {
	// 		odom_msg.pose.pose.position.z = nav_z;
	// 		updatedOdom[2] = true;
	// 	}
	// }
	else if (arg == "NAV_DEPTH") {
		if (updatedOdom[2] == false) {
			odom_msg.pose.pose.position.z = nav_z;
			updatedOdom[2] = true;
			// std::cout << "set nav_z" << std::endl;
		}
	}
	else  {
		if (arg == "NAV_ROLL") {
			if (updatedOdom[3] == false) {
				updatedOdom[3] = true;
				// std::cout << "set nav_roll" << std::endl;
			}
		}
		if (arg == "NAV_PITCH") {
			if (updatedOdom[4] == false) {
				updatedOdom[4] = true;
				// std::cout << "set nav_pitch" << std::endl;
			}
		}
		if (arg == "NAV_HEADING") {
			if (updatedOdom[5] == false) {
				updatedOdom[5] = true;
				// std::cout << "set nav_yaw" << std::endl;
			}
		}
		// handling if we get piecewise IMU data (e.g. nav_roll, nav_pitch, nav_yaw)
		if (updatedOdom[3] && updatedOdom[4] && updatedOdom[5]) {
			MOOSNode::e2q(nav_yaw*M_PI/180, nav_pitch*M_PI/180, nav_roll*M_PI/180);
			odom_msg.pose.pose.orientation.x = qx;
			odom_msg.pose.pose.orientation.y = qy;
			odom_msg.pose.pose.orientation.z = qz;
			odom_msg.pose.pose.orientation.w = qw;
			// std::cout << "set quaternion" << std::endl;
		}
	}
	
	// Apparently result is worse with local planner...?
	odom_msg.twist.twist.linear.x = 0;
	odom_msg.twist.twist.linear.y = 0;
	odom_msg.twist.twist.linear.z = 0;
	odom_msg.twist.twist.angular.x = 0;
	odom_msg.twist.twist.angular.y = 0;
	odom_msg.twist.twist.angular.z = 0;

	if (std::all_of( std::begin(updatedOdom), std::end(updatedOdom), []( const bool v){ return v; } )) {
		// why does this need to be static? doesn't publish otherwise
		static tf2_ros::TransformBroadcaster tf;
		geometry_msgs::TransformStamped	tfs;
		tfs.header.stamp = odom_msg.header.stamp;
		tfs.header.frame_id = "odom";
		tfs.child_frame_id = "base_link";
		tfs.transform.translation.x = odom_msg.pose.pose.position.x;
		tfs.transform.translation.y = odom_msg.pose.pose.position.y;
		tfs.transform.translation.z = odom_msg.pose.pose.position.z;
		tfs.transform.rotation.x = odom_msg.pose.pose.orientation.x;
		tfs.transform.rotation.y = odom_msg.pose.pose.orientation.y;
		tfs.transform.rotation.z = odom_msg.pose.pose.orientation.z;
		tfs.transform.rotation.w = odom_msg.pose.pose.orientation.w;

		tf.sendTransform(tfs);
	}
}
