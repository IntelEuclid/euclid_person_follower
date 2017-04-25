/******************************************************************************
Copyright (c) 2016, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

//includes for person tracking stuff
#include <realsense_person/PersonDetection.h>
#include <realsense_person/Person.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

#include <visualization_msgs/Marker.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>

#include "follower_nodelet.h"

//Nodelet dependencies
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(realsense_person_follower::PersonFollowerNodelet, nodelet::Nodelet)

namespace realsense_person_follower
{

	//******************************
	// Public Methods
	//******************************

	PersonFollowerNodelet::~PersonFollowerNodelet()
	{
		ROS_INFO_STREAM("Done - PersonFollowerNodelet");
	}


	void PersonFollowerNodelet::onInit()
	{
		ROS_INFO_STREAM("Starting Follower node");

		fillConfigMap();

		mTrackedPersonId = -1;

		ros::NodeHandle& nh = getNodeHandle();

		//subscribe
		mSub = nh.subscribe("camera/person/detection_data", 1, &PersonFollowerNodelet::personTrackingCallback, this);

		//advertise
		mMarkerPub = nh.advertise<visualization_msgs::Marker>("person_follower/marker",1);
		mGoalPub = nh.advertise<geometry_msgs::PointStamped>("person_follower/goal", 1);

		// Initialize dynamic reconfigure
		mReconfigureServer.reset(new dynamic_reconfigure::Server<realsense_person_follower::person_followerConfig>(getPrivateNodeHandle()));
		mReconfigureServer->setCallback(boost::bind(&PersonFollowerNodelet::ConfigureCallback, this, _1, _2));

	}

	//===================================
	//	Member Functions
	//===================================

	void PersonFollowerNodelet::fillConfigMap()
	{
		std::vector<std::string> args = getMyArgv();
		while (args.size() > 1)
		{
			mConfig[args[0]] = args[1];
			args.erase(args.begin());
			args.erase(args.begin());
		}

		mEnabled = true;

	}

	void PersonFollowerNodelet::process(double x, double y, double z)
	{

		//if the point is not valid
		if (x != -1 || y !=-1 || z != -1)
		{
			publishMarker(x, y, z);
			publishGoal(x, y, z);
		}

	  }


	//***********************************
	// Debug/Visualize functions
	//***********************************

	void PersonFollowerNodelet::publishGoal(double x, double y, double z)
	{
		geometry_msgs::PointStamped goal;
		goal.header.frame_id = mFrame;
		goal.header.stamp = ros::Time();
		goal.point.x = x;
		goal.point.y = y;
		goal.point.z = z;
		mGoalPub.publish(goal);
	}

	void PersonFollowerNodelet::publishMarker(double x,double y,double z)
	  {
	    visualization_msgs::Marker marker;
	    marker.header.frame_id = mFrame;
	    marker.header.stamp = ros::Time();
	    marker.ns = "my_namespace";
	    marker.id = 0;
	    marker.type = visualization_msgs::Marker::SPHERE;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x = x;
	    marker.pose.position.y = y;
	    marker.pose.position.z = z;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;
	    marker.scale.x = 0.1;
	    marker.scale.y = 0.1;
	    marker.scale.z = 0.1;
	    marker.color.a = 1.0;
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.0;
	    mMarkerPub.publish( marker );

	 }

	//***********************************
	// Callback functions
	//***********************************

	void PersonFollowerNodelet::personTrackingCallback(const realsense_person::PersonDetection& msg_userdata)
	{

		if (mEnabled)
		{
			mPersonCount = msg_userdata.detected_person_count;
			
			if(mPersonCount > 0)
    		{
				if (mTrackedPersonId == -1)
				{
					realsense_person::Person personItem = msg_userdata.persons[0];
					mTrackedPersonId = personItem.person_id.tracking_id;
					mFrame = msg_userdata.header.frame_id;
       				process(personItem.center_of_mass.image.x/1000.0, personItem.center_of_mass.image.y/1000.0, personItem.center_of_mass.image.z/1000.0); 
				}	 
				else 
				{
					for (int i = 0; i < mPersonCount; i++)
					{
						if (msg_userdata.persons[i].person_id.tracking_id == mTrackedPersonId)
						{
							realsense_person::Person personItem = msg_userdata.persons[i];
							mFrame = msg_userdata.header.frame_id;//personItem.header.frame_id;
       						process(personItem.center_of_mass.image.x/1000.0, personItem.center_of_mass.image.y/1000.0, personItem.center_of_mass.image.z/1000.0);
						}
					}
       				mTrackedPersonId = -1;
				}				          			
    		}
    		else
    		{
    			process(-1,-1,-1);
    		}
		}

	}

	void PersonFollowerNodelet::ConfigureCallback(realsense_person_follower::person_followerConfig &config, uint32_t level)
	{
		
		mEnabled = config.mEnabled;

	}

	//***********************************
	// Utils functions
	//***********************************

	bool PersonFollowerNodelet::to_bool(std::string str)
	{
		std::transform(str.begin(), str.end(), str.begin(), ::tolower);
		std::istringstream is(str);

		bool b;

		is >> std::boolalpha >> b;

		return b;
	}

}

