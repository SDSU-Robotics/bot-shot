#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <SDL2/SDL.h>

#include "Enables.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

const float FAST_SPEED = 0.99;
const float SLOW_SPEED = 0.2;

void inline sleepApp(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void updateDrive();
void updateLaunchWheels();
void updateLaunchAngle();
void updateComAngle();

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
	ctre::phoenix::platform::can::SetCANInterface("can0");

	ros::NodeHandle n;

	ros::Rate loop_rate(1000);
	Listener listener;

    ros::Publisher l_speed_pub = n.advertise<std_msgs::Float64>("l_speed", 1000);
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float64>("r_speed", 1000);

	ros::Subscriber joySub = n.subscribe("joy", 100, &Listener::joyListener, &listener);

	// wait for Talons to get ready
	sleepApp(2000);

	while (ros::ok())
	{
	    bool buttons[12];
	    double axes[6];

	    std_msgs::Float64 l_speed_msg;
        std_msgs::Float64 r_speed_msg;

        listener.getJoyVals(buttons, axes);

		// get controller values
		float speed = axes[1]; // left Y
		float turn = -1 * axes[4]; // right Y

		float speedFactor;

		if (axes[2] < 0.0 && axes[5] < 0.0)
			speedFactor = FAST_SPEED;
		else
			speedFactor = SLOW_SPEED;
		
		l_speed_msg.data = speedFactor * 0.5 * speed + 0.25 * turn;
        r_speed_msg.data = speedFactor * 0.5 * speed - 0.25 * turn;

		l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);

		ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}