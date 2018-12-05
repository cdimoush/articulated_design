#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include <math.h>
//#include "mech_members.h"

namespace articulated
{
class mech
{
public:
	mech();

	struct link_struct
	{
		double length;
		double joint_pos;
		double weight;
		double coord[2][2];

	};
	struct joint_struct
	{
		double weight;
		double torque;
		double angle;
		double coord[2];

	};
	struct servo_struct
	{
		double angle;
		double hub; //This is important for joint 2 mechanism, hub var is the radius 
		double torque;
		double coord[2];

	};
private:
	//functions
	void defineLinkPositions();
	void defineLinkProperties();
	void publishLinks();
	void rotateServo1(std_msgs::Float64 degrees);
	void rotateServo2(std_msgs::Float64 degrees);
	void updateJointPositions();
	void calcTorque();
	//publish/subscribe
	ros::Publisher link_pub_;
	ros::Publisher joint_pub_;
	ros::Publisher torque1_pub_;
	ros::Publisher torque2_pub_;
	ros::Publisher servo1_pub_;
	ros::Publisher servo2_pub_;
	ros::Subscriber servo1_sub_;
	ros::Subscriber servo2_sub_;
	//variables
	mech::link_struct link1_, link2_, link3_;
	mech::joint_struct joint1_, joint2_, ee_;
	mech::servo_struct servo1_, servo2_;
	geometry_msgs::PoseArray link_coord_array_;
	std_msgs::Float64 servo1_torque_;
	std_msgs::Float64 servo2_torque_;


	ros::NodeHandle n_;
};
}
