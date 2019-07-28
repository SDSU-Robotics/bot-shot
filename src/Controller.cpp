#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>

using namespace std;

const float FAST_SPEED = 0.99;
const float SLOW_SPEED = 0.2;

class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;

private:
    bool _buttons[12] = { 0 };
	double _axes[6] = { 0 };
};


void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 6; i++)
        _axes[i] = Joy->axes[i];
}

void Listener::getJoyVals(bool buttons[], double axes[]) const
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("joy", 100, &Listener::joyListener, &listener);

	bool buttons[12];
	double axes[6];

	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float64>("l_speed", 1000);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float64>("r_speed", 1000);
	ros::Publisher intake_pub = n.advertise<std_msgs::Float64>("set_intake", 1000);
	ros::Publisher angle_pub = n.advertise<std_msgs::Float64>("set_angle", 1000);

    std_msgs::Float64 l_speed_msg;
    std_msgs::Float64 r_speed_msg;
	std_msgs::Float64 intake_msg;
	std_msgs::Float64 angle_msg;

	while (ros::ok())
	{
        listener.getJoyVals(buttons, axes);

		// get controller values
		float speed = axes[1]; // left Y
		float turn = -1 * axes[3]; // right X

		float speedFactor;

		

		if (buttons[0]) // A
			intake_msg.data = 1.0;
		else
			intake_msg.data = 0.0;

		if (buttons[5]) // RB
		{
			angle_msg.data = 0.5 * axes[1];
			l_speed_msg.data = 0.0;
        	r_speed_msg.data = 0.0;
		}
		else
		{
			angle_msg.data = 0.0;
			if (axes[2] < 0.0 && axes[5] < 0.0) // left and right triggers
				speedFactor = FAST_SPEED;
			else
				speedFactor = SLOW_SPEED;
		
			l_speed_msg.data = speedFactor * 0.5 * speed + 0.25 * turn;
        	r_speed_msg.data = speedFactor * 0.5 * speed - 0.25 * turn;
		}
			

		l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);
		intake_pub.publish(intake_msg);
		angle_pub.publish(angle_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
