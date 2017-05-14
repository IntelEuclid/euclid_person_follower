# Intel&reg; Euclid&trade; Person follower sample.

This nodelet register to person tracking messages and publishes a goal for the robot movement control to handle.

http://www.intel.com/Euclid_XXX

http://wiki.ros.org/EuclidWiki_XXX

## Subscribed Topics

	camera/person/detection_data (realsense_person::PersonDetection)
		Person Detection/Tracking data
	
## Published Topics

Point Cloud

    depth_follower/goal (geometry_msgs::PointStamped)
		X,Y,Z goal of where to follow
	depth_follower/marker (visualization_msgs::Marker)
		X,Y,Z for rviz visualization 

## Parameters

	Enabled(bool, default: true)
		Enable flag for the algorithm

## Contributing to the Project

The Intel&reg; Euclid&trade; Person follower sample is developed and distributed under
a BSD-3 license as noted in [licenses/License.txt](licenses/License.txt).

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
have the right to submit it under the open source license
indicated in the file; or

(b) The contribution is based upon previous work that, to the best
of my knowledge, is covered under an appropriate open source
license and I have the right under that license to submit that
work with modifications, whether created in whole or in part
by me, under the same open source license (unless I am
permitted to submit under a different license), as indicated
in the file; or

(c) The contribution was provided directly to me by some other
person who certified (a), (b) or (c) and I have not modified
it.

(d) I understand and agree that this project and the contribution
are public and that a record of the contribution (including all
personal information I submit with it, including my sign-off) is
maintained indefinitely and may be redistributed consistent with
this project or the open source license(s) involved.

## Configuration:

| Version        | Best Known           |
|:-------------- |:---------------------|
| OS             | Ubuntu 16.04 LTS     |
| ROS            | Kinetic              |
