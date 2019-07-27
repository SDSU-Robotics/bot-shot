#include "Launcher.h"
#include "Display.h"
#include "DriveBase.h"
#include "Enables.h"

#include <thread>
#include <chrono>

TalonSRX Launcher::_topWheel = {DeviceIDs::launcherTop};
TalonSRX Launcher::_bottomWheel = {DeviceIDs::launcherBottom};
TalonSRX Launcher::_comArm = {DeviceIDs::commencementArm};
TalonSRX Launcher::_angleMotor = {DeviceIDs::launcherAngle};

float Launcher::_rpmSetpoint = 0.0;

void Launcher::init()
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

	_comArm.SetInverted(true);
	_angleMotor.SetNeutralMode(NeutralMode::Brake);
}


void Launcher::setRPM(float rpm)
{
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

void Launcher::stop()
{
	_angleMotor.Set(ControlMode::PercentOutput, 0.0);
	_comArm.Set(ControlMode::PercentOutput, 0.0);
}