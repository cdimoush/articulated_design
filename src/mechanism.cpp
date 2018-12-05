#include "mechanism.h"
#include <ros/console.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mechanism_node");
	articulated::mech arm_mechanism;
	
	return 0;
}

articulated::mech::mech()
{
	link_pub_ = n_.advertise<geometry_msgs::PoseArray>("/arm/links", 1000);
	torque1_pub_ = n_.advertise<std_msgs::Float64>("/arm/servo1/torque", 1);
	torque2_pub_ = n_.advertise<std_msgs::Float64>("/arm/servo2/torque", 1);
	servo1_pub_ = n_.advertise<std_msgs::Float64>("/arm/servo1/angle", 1);
	servo2_pub_ = n_.advertise<std_msgs::Float64>("/arm/servo2/angle", 1);
	servo1_sub_ = n_.subscribe("arm/servo1", 1000, &mech::rotateServo1, this);
	servo2_sub_ = n_.subscribe("arm/servo2", 1000, &mech::rotateServo2, this);
	defineLinkPositions();
	defineLinkProperties();
	while ( ros::ok() )
  	{
  		//link_pub_.publish(link_coord_array_);
  		//torque1_pub_.publish(servo1_torque_);
  		//torque2_pub_.publish(servo2_torque_);

    	ros::spinOnce();
    	ros::Duration(0.1).sleep();
	}
}

void articulated::mech::rotateServo1(std_msgs::Float64 degrees)
{
	//Change angle of servo1, then calculate and update all links points

	//Servo angle 
	servo1_.angle += degrees.data * M_PI / 180;
	//Link1(only xy2 changes)
	link1_.coord[1][0] = servo1_.coord[0] + sin(servo1_.angle)*link1_.length;
	link1_.coord[1][1] = servo1_.coord[1] + cos(servo1_.angle)*link1_.length;
	
	//Link2-xy1 and Link3-xy2
	//Point calculated using the law of cosine
	//////////////////////////////////////////
	//a - lenght of the link2 small lever
	//b - link3 lenght
	//c - distance between link31 and link12
	double a; 
	n_.getParam("/arm/link2_joint_pos", a);
	double b = link3_.length;
	double c1 = link1_.coord[1][0] - link3_.coord[0][0];
	double c2 = link1_.coord[1][1] - link3_.coord[0][1];
	double c  = sqrt(pow(c1, 2) + pow(c2, 2));
	//phi - angle btw c and a
	//gamma - angle btw c and horizontal 
	//theta - difference btw phi and gamma
	double phi = acos((c*c + a*a - b*b)/(2*c*a));
	double gamma = asin((link1_.coord[1][1] - link3_.coord[0][1])/c);
	double theta = phi - gamma;
	
	///////////////////////////////////////////

	//Calculate Link2/Link3 shared point
	link2_.coord[0][0] = link1_.coord[1][0] - a*cos(theta);
	link2_.coord[0][1] = link1_.coord[1][1] + a*sin(theta);
	link3_.coord[1][0] = link2_.coord[0][0];
	link3_.coord[1][1] = link2_.coord[0][1];
	//Calculate Link2-xy2
	//alpha is angle between link2-xy1 and link1_-xy2
	double alpha = asin((link2_.coord[0][1] - link1_.coord[1][1])/a);
	link2_.coord[1][0] = link2_.coord[0][0] + cos(alpha)*link2_.length;
	link2_.coord[1][1] = link2_.coord[0][1] - sin(alpha)*link2_.length;


	//double l3 = sqrt(pow((link3_.coord[1][0] - link3_.coord[0][0]), 2) + pow((link3_.coord[1][1] - link3_.coord[0][1]),2));

	updateJointPositions();

	publishLinks();

}

void articulated::mech::rotateServo2(std_msgs::Float64 degrees)
{
	//Change angle of servo2, then calculate and update all links points

	//Servo angle 
	servo2_.angle += degrees.data * M_PI / 180;
	//Link3-xy1
	link3_.coord[0][0] = servo2_.coord[0] - cos(servo2_.angle)*servo2_.hub;
	link3_.coord[0][1] = servo2_.coord[1] + sin(servo2_.angle)*servo2_.hub;

	//Link2-xy1 and Link3-xy2
	//Point calculated using the law of cosine
	//////////////////////////////////////////
	//a - lenght of the link2 small lever
	//b - link3 lenght
	//c - distance between link31 and link1_2
	double a; 
	n_.getParam("/arm/link2_joint_pos", a);
	double b = link3_.length;
	double c1 = link1_.coord[1][0] - link3_.coord[0][0];
	double c2 = link1_.coord[1][1] - link3_.coord[0][1];
	double c  = sqrt(pow(c1, 2) + pow(c2, 2));
	//phi - angle btw c and a
	//gamma - angle btw c and horizontal 
	//theta - difference btw phi and gamma
	double phi = acos((c*c + a*a - b*b)/(2*c*a));
	double gamma = asin((link1_.coord[1][1] - link3_.coord[0][1])/c);
	double theta = phi - gamma;
	
	///////////////////////////////////////////

	//Calculate Link2/Link3 shared point
	link2_.coord[0][0] = link1_.coord[1][0] - a*cos(theta);
	link2_.coord[0][1] = link1_.coord[1][1] + a*sin(theta);
	link3_.coord[1][0] = link2_.coord[0][0];
	link3_.coord[1][1] = link2_.coord[0][1];
	//Calculate Link2-xy2
	//alpha is angle between link2-xy1 and link1_-xy2
	double alpha = asin((link2_.coord[0][1] - link1_.coord[1][1])/a);
	link2_.coord[1][0] = link2_.coord[0][0] + cos(alpha)*link2_.length;
	link2_.coord[1][1] = link2_.coord[0][1] - sin(alpha)*link2_.length;


	//double l3 = sqrt(pow((link3_.coord[1][0] - link3_.coord[0][0]), 2) + pow((link3_.coord[1][1] - link3_.coord[0][1]),2));
	

	updateJointPositions();

	publishLinks();
}

void articulated::mech::updateJointPositions()
{
	//Update Joints Positions 
	joint1_.coord[0] = link1_.coord[0][0];
	joint1_.coord[1] = link1_.coord[0][1];
	joint2_.coord[0] = link1_.coord[1][0];
	joint2_.coord[1] = link1_.coord[1][1];
	ee_.coord[0] = link2_.coord[1][0];
	ee_.coord[1] = link2_.coord[1][1];

	joint1_.angle = servo1_.angle;
	joint2_.angle = acos((link2_.coord[1][0] - link2_.coord[0][0]) / link2_.length);
}

void articulated::mech::calcTorque()
{
	//Calculate the Torques created at each joint and at servos 

	//HOLDING TORQUE
	///////////////////////////////////

	//Get all weight forces and x coord
	double w_array[4][2];
	//l1
	w_array[0][0] = link1_.weight;
	w_array[0][1] = link1_.coord[1][0] / 2;
	//j2
	w_array[1][0] = joint2_.weight;
	w_array[1][1] = link1_.coord[1][0];
	//w2
	w_array[2][0] = link2_.weight;
	w_array[2][1] = link2_.coord[0][0] + cos(joint2_.angle)*link2_.length/2;
	//wee
	w_array[3][0] = ee_.weight;
	w_array[3][1] = link2_.coord[1][0];
	//f3, The force applied on link3 (found by solving moment equation about joint2)
	double beta;
	if ((link3_.coord[1][1] - link3_.coord[0][1]) > link3_.length)
	{
		//BUG!!!!!
		//Link 3 is calculated to be longer than real length
		beta = M_PI / 2;
	}
	else
	{
		beta = asin((link3_.coord[1][1] - link3_.coord[0][1])/link3_.length);
	}
	
	double j2_x = joint2_.coord[0]; double j2_y = joint2_.coord[1];
	//0 = f3x*dx + f3y*dy - ws*dx - wee*dx
	//add w2 and wee torques
	double f3 = w_array[2][0] * (w_array[2][1] - j2_x) + w_array[3][0] * (w_array[3][1] - j2_x);
	
	//divide sinb*dx + cosb*dy
	f3 = f3 / (fabs(sin(beta)*(j2_x - link2_.coord[0][0])) + fabs(cos(beta)*(j2_y - link2_.coord[0][1])));
	//DEBUG!!!!!!!!1
	double t3_x = f3 * sin(beta) * fabs(j2_x - link3_.coord[1][0]);
	double t3_y = f3 * cos(beta) * fabs(j2_y - link3_.coord[1][1]);
	
	//SERVO1
	double torque1 = 0;
	//for all weights in w_array
	double t = 0; 
	for(int i = 0; i < 4; i++)
	{
		//negative if right of servo (joint1)
		torque1 += -1*(w_array[i][0] * (w_array[i][1] - joint1_.coord[0]));
		//Debug if statements
		//ROS_ERROR("Link1 Torques: %f", torque1);
		if (i == 1)
		{
			//ROS_ERROR("Link1 Torques: %f", torque1);
			t = torque1;
		}
		if (i == 3)
		{
			t = torque1 - t;
			//ROS_ERROR("Link2 Torques: %f", t);
			
		}
	}
	//ROS_ERROR("Weight Torques: %f", torque1);
	//torques created from f3
	//f3x
	torque1 += -1*(fabs(f3*sin(beta)) * (link3_.coord[1][0] - joint1_.coord[0]));
	//f3y
	torque1 += fabs(f3*cos(beta)) * (link3_.coord[1][1] - joint1_.coord[1]);
	//ROS_ERROR("Total Torques: %f", torque1);
	
	servo1_torque_.data = torque1;
	torque1_pub_.publish(servo1_torque_);

	//SERVO2
	double alpha = M_PI/2 - servo2_.angle; //Angle btw Hub tangent and Horizontal
	double gamma = fabs(beta - alpha); //Angle between link3 and hub tangent 
	
	//get tangent component of f3 on hub tangent
	double f3_t = f3*cos(gamma);
	
	double torque2 = -1 * f3_t * servo2_.hub;
	servo2_torque_.data = torque2;
	torque2_pub_.publish(servo2_torque_);
}

void articulated::mech::defineLinkPositions()
{
	//Setup position of mechanism

	//SET SERVOS
	////////////////////////////////////////
	//Servo1 is at 0,0
	//Both servos start at angle=0
	servo1_.coord[0] = 0; servo1_.coord[1] = 0;
	servo1_.angle = 0;
	n_.getParam("/arm/s2_x", servo2_.coord[0]);
	n_.getParam("/arm/s2_y", servo2_.coord[1]);
	servo2_.angle = 0;
	n_.getParam("/arm/s2_hub_rad", servo2_.hub);


	//SET LINKS 1&2
	/////////////////////////////////////////
	//Length
	n_.getParam("/arm/link1_length", link1_.length);
	n_.getParam("/arm/link2_length", link2_.length);
	//Coordinates (2 end points)
	link1_.coord[0][0] = servo1_.coord[0];
	link1_.coord[0][1] = servo1_.coord[1];
	link1_.coord[1][0] = servo1_.coord[0] + sin(servo1_.angle)*link1_.length;
	link1_.coord[1][1] = servo1_.coord[1] + cos(servo1_.angle)*link1_.length;
	//Link2 will always start parralel to horizontal
	//Link2 is connected to end of link1 at the joint position
	double link2_joint_pos;
	n_.getParam("/arm/link2_joint_pos", link2_joint_pos);
	n_.getParam("/arm/joint2_angle", joint2_.angle);
	joint2_.angle = joint2_.angle * M_PI / 180;
	link2_.coord[0][0] = link1_.coord[1][0] - link2_joint_pos * cos(joint2_.angle);
	link2_.coord[0][1] = link1_.coord[1][1] + link2_joint_pos * sin(joint2_.angle);
	link2_.coord[1][0] = link2_.coord[0][0] + link2_.length * cos(joint2_.angle);
	link2_.coord[1][1] = link2_.coord[0][1] - link2_.length * sin(joint2_.angle);

	//SET LINK 3
	////////////////////////////////////////
	//Connected to hub and link2 coord[0]
	link3_.coord[0][0] = servo2_.coord[0] - servo2_.hub;
	ROS_ERROR("Link3 x coord: %f", link3_.coord[0][0]);
	link3_.coord[0][1] = servo2_.coord[1];
	link3_.coord[1][0] = link2_.coord[0][0];
	link3_.coord[1][1] = link2_.coord[0][1];
	//Calculate link3 length 
	link3_.length = sqrt(pow((link3_.coord[1][0] - link3_.coord[0][0]), 2) + pow((link3_.coord[1][1] - link3_.coord[0][1]),2));
	ROS_ERROR("Link3 Vaule: %f", link3_.length);

	updateJointPositions();

	publishLinks();
}

void articulated::mech::defineLinkProperties()
{
	//Define Weight of all links. If components on arm have significant mass
	//and effect the preformance this should be noted here

	double g = 9.81;
	double m1;
	double m2;
	double m3;
	double mee;
	n_.getParam("/arm/link1_mass", m1);
	n_.getParam("/arm/link2_mass", m2);
	n_.getParam("/arm/link3_mass", m3);
	n_.getParam("/arm/ee_mass", mee);

	link1_.weight = m1*g;
	link2_.weight = m2*g;
	link3_.weight = m3*g;
	ee_.weight = mee*g;
}

void articulated::mech::publishLinks()
{
	//Fill link points msg
	link_coord_array_.poses.clear();
	geometry_msgs::Pose new_pose;
	for (int i=0; i<2; i ++)
	{
		
		new_pose.position.x = link1_.coord[i][0];
		new_pose.position.y = link1_.coord[i][1];
		link_coord_array_.poses.push_back(new_pose);
	}
	for (int i=0; i<2; i ++)
	{
		new_pose.position.x = link2_.coord[i][0];
		new_pose.position.y = link2_.coord[i][1];
		link_coord_array_.poses.push_back(new_pose);
	}
	for (int i=0; i<2; i ++)
	{
		new_pose.position.x = link3_.coord[i][0];
		new_pose.position.y = link3_.coord[i][1];
		link_coord_array_.poses.push_back(new_pose);
	}

	link_pub_.publish(link_coord_array_);

	std_msgs::Float64 s1_msg;
	std_msgs::Float64 s2_msg;
	s1_msg.data = servo1_.angle * 180 / M_PI;
	s2_msg.data = servo2_.angle * 180 / M_PI; 
	servo1_pub_.publish(s1_msg);
	servo2_pub_.publish(s2_msg);

	//Position has been changed, update torque
	calcTorque();
}