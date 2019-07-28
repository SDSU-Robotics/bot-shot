#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"
#include "Enables.h"
#include "Conversions.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

const int MAX_RPM = 2000;

class Listener
{
public:
	Listener();
	void setRPM(const std_msgs::Float64 msg);
	void setAngle(const std_msgs::Float64 msg);
	void setIntake(const std_msgs::Float64 msg);

private:
	TalonSRX _topWheel = {DeviceIDs::launcherTop};
	TalonSRX _bottomWheel = {DeviceIDs::launcherBottom};
	TalonSRX _comArm = {DeviceIDs::commencementArm};
	TalonSRX _angleMotor = {DeviceIDs::launcherAngle};
	TalonSRX _intakeLeft = {DeviceIDs::intakeLeft};
	TalonSRX _intakeRight = {DeviceIDs::intakeRight};

	float _rpmSetpoint = 0.0;
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "Launcher");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	ctre::phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber set_RPM_sub = n.subscribe("set_RPM", 1000, &Listener::setRPM, &listener);
	ros::Subscriber set_angle_sub = n.subscribe("set_angle", 1000, &Listener::setAngle, &listener);
	ros::Subscriber set_intake_sub = n.subscribe("set_intake", 1000, &Listener::setIntake, &listener);

	//ros::Publisher top_RPM_pub = n.advertise<std_msgs::Float64>("top_RPM_reading", 1000);
    //ros::Publisher bot_RPM_pub = n.advertise<std_msgs::Float64>("bot_RPM_reading", 1000);

	//std_msgs::Float64 top_RPM_msg;
    //std_msgs::Float64 bot_RPM_msg;

	while (ros::ok())
	{
		//top_RPM_msg.data = Conversions::toRpm( listener._bottomWheel.GetSelectedSensorVelocity() );
		//bot_RPM_msg.data = Conversions::toRpm( listener._topWheel.GetSelectedSensorVelocity() );

		//top_RPM_pub.publish(top_RPM_msg);
		//bot_RPM_pub.publish(bot_RPM_msg);

		ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


Listener::Listener()
{
	// ============================== Top Wheel ==============================
	TalonSRXConfiguration topProfile;

	//Threshold for zero-motion for the neutral position.
	topProfile.neutralDeadband = 0.01;
			
	//Peak Speed Config
	topProfile.peakOutputForward = 1.0;
	topProfile.peakOutputReverse = -1.0;
	
	//Ramp Config
	topProfile.closedloopRamp = 1.5f;
			
	//PID Config
	topProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
	topProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	//PID Constants
	topProfile.slot0.kP                       = 0.027f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	topProfile.slot0.kI                       = 0.01f; //Integral Constant.     Controls the steady-state error correction.
	topProfile.slot0.kD                       = 0.01f; //Derivative Constant.   Controls error oscillation.
	topProfile.slot0.kF                       = 0.0083f; //Feed Forward Constant. (IDK what this does)
	topProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	topProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. (IDK what this does)
	topProfile.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
	topProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	topProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	TalonSRXPIDSetConfiguration profile;

	_topWheel.ConfigAllSettings(topProfile);

	_topWheel.SetNeutralMode(NeutralMode::Brake);
	_topWheel.SetInverted(false);
	_topWheel.SetSensorPhase(false);

	// ============================== Bottom Wheel ==============================

	TalonSRXConfiguration bottomProfile;

	//Threshold for zero-motion for the neutral position.
	bottomProfile.neutralDeadband = 0.01;
			
	//Peak Speed Config
	bottomProfile.peakOutputForward = 1.0;
	bottomProfile.peakOutputReverse = -1.0;
			
	//Ramp Config
	bottomProfile.closedloopRamp = 1.5f;
			
	//PID Config
	bottomProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
	bottomProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	//PID Constants
	bottomProfile.slot0.kP                       = 0.027f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	bottomProfile.slot0.kI                       = 0.01f; //Integral Constant.     Controls the steady-state error correction.
	bottomProfile.slot0.kD                       = 0.01f; //Derivative Constant.   Controls error oscillation.
	bottomProfile.slot0.kF                       = 0.0084; //Feed Forward Constant. For velocity
	bottomProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	bottomProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. Biggest Error for I
	bottomProfile.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
	bottomProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	bottomProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	_bottomWheel.ConfigAllSettings(bottomProfile);

	_bottomWheel.SetNeutralMode(NeutralMode::Brake);
	_bottomWheel.SetInverted(false);
	_bottomWheel.SetSensorPhase(true);


	// =========================== Commencement Arm =============================
	_comArm.SetInverted(true);

	// ============================== Angle Motor ===============================
	_angleMotor.SetNeutralMode(NeutralMode::Brake);
}


void Listener::setRPM(const std_msgs::Float64 msg)
{
	float rpm = msg.data;
	if (rpm > 0.01)
	{
		if (rpm > MAX_RPM)
			rpm = MAX_RPM;
			
		_rpmSetpoint = rpm;
		_topWheel.Set(ControlMode::Velocity, Conversions::fromRpm(rpm - 100));
		_bottomWheel.Set(ControlMode::Velocity, Conversions::fromRpm(rpm + 100));
	}
	else
	{
		_rpmSetpoint = 0.0;
		_topWheel.Set(ControlMode::PercentOutput, 0.0);
		_bottomWheel.Set(ControlMode::PercentOutput, 0.0);
	}
}

void Listener::setAngle(const std_msgs::Float64 msg)
{
	_angleMotor.Set(ControlMode::PercentOutput, msg.data);
}

void Listener::setIntake(const std_msgs::Float64 msg)
{
	_intakeLeft.Set(ControlMode::PercentOutput, -1 * msg.data);
	_intakeRight.Set(ControlMode::PercentOutput, msg.data);
}