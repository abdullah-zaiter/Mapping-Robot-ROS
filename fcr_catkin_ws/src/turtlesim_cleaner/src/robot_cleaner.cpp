#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
ros::Publisher velocity_publisher;
using namespace std;

void move(double speed, double distance, bool isForward);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_cleaner");
    ros::NodeHandle n;
    double speed, distance;
	bool isForward;
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    cout<<"enter speed: ";
    cin>> speed;
    cout<<"enter distance: ";
    cin>>distance;
    cout<<"Forward?: ";
    cin>>isForward;
    move(speed, distance, isForward);

}


void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;
	double current_distance=0,t0=0,t1=0;

	if (isForward)
	{
		vel_msg.linear.x = abs(speed);
	}else
	{
		vel_msg.linear.x = -abs(speed);
	}
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance <= distance);
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}
