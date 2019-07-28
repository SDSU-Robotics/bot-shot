#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"
#include "Enables.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Listener
{
public:
	void setLSpeed(const std_msgs::Float64 msg);
	void setRSpeed(const std_msgs::Float64 msg);

private:
	TalonSRX _motorL = {DeviceIDs::driveL};
	TalonSRX _motorR = {DeviceIDs::driveR};
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "DriveBase");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	
	ctre::phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber l_speed_sub = n.subscribe("l_speed", 1000, &Listener::setLSpeed, &listener);
	ros::Subscriber r_speed_sub = n.subscribe("r_speed", 1000, &Listener::setRSpeed, &listener);

	while(ros::ok())
	{
		ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


void Listener::setLSpeed(const std_msgs::Float64 msg)
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

void Listener::setRSpeed(const std_msgs::Float64 msg)
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
