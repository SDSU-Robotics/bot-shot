#include "Launcher.h"
#include "Display.h"
#include "Arduino.h"
#include "PixyController.h"
#include "DriveBase.h"

#include <thread>
#include <chrono>

TalonSRX Launcher::_topWheel = {DeviceIDs::launcherTop};
TalonSRX Launcher::_bottomWheel = {DeviceIDs::launcherBottom};
TalonSRX Launcher::_comArm = {DeviceIDs::commencementArm};
TalonSRX Launcher::_angleMotor = {DeviceIDs::launcherAngle};

PIDController Launcher::_launchAnglePID = PIDController();
PIDController Launcher::_comArmPID = PIDController();
PIDController Launcher::_horizontalPixyPID = PIDController();
PIDController Launcher::_verticalPixyPID = PIDController();

ControlMode Launcher::_launchAngleControlMode = ControlMode::PercentOutput;
ControlMode Launcher::_comAngleControlMode = ControlMode::PercentOutput;

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
	topProfile.slot0.kP                       = 0.04f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	topProfile.slot0.kI                       = 0.01f; //Integral Constant.     Controls the steady-state error correction.
	topProfile.slot0.kD                       = 0.75f; //Derivative Constant.   Controls error oscillation.
	topProfile.slot0.kF                       = 0.0105f; //Feed Forward Constant. (IDK what this does)
	topProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	topProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. (IDK what this does)
	topProfile.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
	topProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	topProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	TalonSRXPIDSetConfiguration profile;

	_topWheel.ConfigAllSettings(topProfile);

	_topWheel.SetNeutralMode(NeutralMode::Brake);
	_topWheel.SetInverted(false);
	_topWheel.SetSensorPhase(true);

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
	bottomProfile.slot0.kP                       = 0.04f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	bottomProfile.slot0.kI                       = 0.01f; //Integral Constant.     Controls the steady-state error correction.
	bottomProfile.slot0.kD                       = 0.75f; //Derivative Constant.   Controls error oscillation.
	bottomProfile.slot0.kF                       = 0.0105f; //Feed Forward Constant. For velocity
	bottomProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	bottomProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. Biggest Error for I
	bottomProfile.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
	bottomProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	bottomProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	_bottomWheel.ConfigAllSettings(bottomProfile);

	_bottomWheel.SetNeutralMode(NeutralMode::Brake);
	_bottomWheel.SetInverted(false);
	_bottomWheel.SetSensorPhase(true);


	// ============================== Launcher Angle ==============================

	_launchAnglePID.setKP(0.08);
	_launchAnglePID.setKI(0.0005);
	_launchAnglePID.setKD(0.01);
	_launchAnglePID.setILimit(1000.0);
	_launchAnglePID.setMaxOut(0.7);

	// ============================== Commencement Arm ==============================

	_comArm.SetNeutralMode(NeutralMode::Brake); 
	_comArm.SetInverted(false);

	_comArmPID.setKP(0.006);
	_comArmPID.setKI(0.0001);
	_comArmPID.setKD(0.01);
	_comArmPID.setILimit(1000.0);
	_comArmPID.setMaxOut(0.2);

	// ============================== Horizontal Pixy PID ==============================

	_horizontalPixyPID.setKP(0.006);
	_horizontalPixyPID.setKI(0.0001);
	_horizontalPixyPID.setKD(0.01);
	_horizontalPixyPID.setILimit(1000.0);
	_horizontalPixyPID.setMaxOut(0.2);

	// ============================== Vertical Pixy PID ==============================

	_verticalPixyPID.setKP(0.001);
	_verticalPixyPID.setKI(0.0001);
	_verticalPixyPID.setKD(0.01);
	_verticalPixyPID.setILimit(1000.0);
	_verticalPixyPID.setMaxOut(0.5);
}


void Launcher::setRPM(float rpm)
{
	if (rpm > 0.01)
	{
		if (rpm > 2500)
			rpm = 2500;
			
		_rpmSetpoint = rpm;
		_topWheel.Set(ControlMode::Velocity, -1 * Conversions::fromRpm(rpm - 100));
		_bottomWheel.Set(ControlMode::Velocity, Conversions::fromRpm(rpm + 100));
	}
	else
	{
		_rpmSetpoint = 0.0;
		_topWheel.Set(ControlMode::PercentOutput, 0.0);
		_bottomWheel.Set(ControlMode::PercentOutput, 0.0);
	}
}

void Launcher::setLaunchAngle(float setAngle)
{
	bool success = false;
	float angle, output;

	switch(_launchAngleControlMode)
	{
		case ControlMode::Position:
			if (setAngle < MIN_LAUNCH_ANGLE)
				setAngle = MIN_LAUNCH_ANGLE;
			
			if (setAngle > MAX_LAUNCH_ANGLE)
				setAngle = MAX_LAUNCH_ANGLE;

			_launchAnglePID.setSetpoint(setAngle);

			//Get IMU values
			if (Arduino::getLaunchAngle(angle))
			{
				Display::debug("Launch angle: " + to_string(angle));
				output = -1.0 * _launchAnglePID.calcOutput(angle);
				Display::debug("Output: " + to_string(output));
				_angleMotor.Set(ControlMode::PercentOutput, output);
			}

			break;

		case ControlMode::PercentOutput:
			_angleMotor.Set(ControlMode::PercentOutput, setAngle);
			break;

		default:
			Display::debug("[Launcher, setLaunchAngle] Error: Invalid control mode for launcher!");
	}
};

void Launcher::setComAngle(float setAngle)
{
	switch(_comAngleControlMode)
	{
		case ControlMode::Position:
			_comArmPID.setSetpoint(setAngle);
			break;
		case ControlMode::PercentOutput:
			_comArm.Set(ControlMode::PercentOutput, -1 * setAngle);
			break;
		default:
			Display::debug("[Launcher, setComAngle] Error: Invalid control mode for launcher!");
	}
	
};


void Launcher::centerHorizontal()
{
	// give the servo time to move into place
	if (pixy_rcs_get_position(0) != 999)
	{
		Display::print("[Launcher, centerHorizontal] Centering on target...");
		pixy_rcs_set_position(0, 999);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// set brightness
	pixy_cam_set_brightness(LAUNCH_PIXY_BRIGHTNESS);

	struct Block block;
	int count = 0;

	do
	{
		block = PixyController::getLatestBlock();
		++count;
	} while (block.signature != HOOP_SIG && count < 10);
	
	float output = _horizontalPixyPID.calcOutput(block.x);

	DriveBase::setLeftPercent(output * -1);
	DriveBase::setRightPercent(output);
}