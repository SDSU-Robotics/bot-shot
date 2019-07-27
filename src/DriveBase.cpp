#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include <chrono>
#include <thread>

#include "DeviceIDs.h"
#include "Enables.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <iostream>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

void inline sleepApp(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

class Listener
{
public:
	void leftCallback(const std_msgs::Float64 msg);
	void rightCallback(const std_msgs::Float64 msg);

private:
	TalonSRX _motorL = {DeviceIDs::driveL};
	TalonSRX _motorR = {DeviceIDs::driveR};
};

void Listener::leftCallback(const std_msgs::Float64 msg)
{
	#ifndef DRIVE
		return;
	#endif // DRIVE

	// limit values
	float percentOutput = msg.data;
	if (percentOutput < -1.0f)
		percentOutput = -1.0f;
	else if (percentOutput > 1.0f)
		percentOutput = 1.0f;

	_motorL.Set(ControlMode::PercentOutput, percentOutput);
}

void Listener::rightCallback(const std_msgs::Float64 msg)
{
	#ifndef DRIVE
		return;
	#endif // DRIVE
	
	// limit values
	float percentOutput = msg.data;
	if (percentOutput < -1.0f)
		percentOutput = -1.0f;
	else if (percentOutput > 1.0f)
		percentOutput = 1.0f;

	_motorR.Set(ControlMode::PercentOutput, -1 * percentOutput);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "DriveBase");

	ctre::phoenix::platform::can::SetCANInterface("can0");
	
	ros::NodeHandle n;

	ros::Rate loop_rate(1000);
	
	Listener listener;
	
	// wait for Talons to get ready
	sleepApp(2000);

	ros::Subscriber l_speed_sub = n.subscribe("l_speed", 1000, &Listener::leftCallback, &listener);
	ros::Subscriber r_speed_sub = n.subscribe("r_speed", 1000, &Listener::rightCallback, &listener);

	while (ros::ok())
	{
		ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
