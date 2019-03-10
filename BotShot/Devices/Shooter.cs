using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using BotShot.Utility;
using BotShot.Config;

namespace BotShot.Devices{
	public class Shooter {

		//Objects associated with the shooter
		public TalonSRX topWheel = new TalonSRX(DeviceIDs.ShooterTop);
		public TalonSRX bottomWheel = new TalonSRX(DeviceIDs.ShooterBottom);
		public TalonSRX angleMotor = new TalonSRX(DeviceIDs.ShooterAngle);
		public TalonSRX comArm = new TalonSRX(DeviceIDs.ShooterComArm);

		// constructor
		public Shooter()
		{
			topWheel.ConfigAllSettings(Motors.DriveL1());
			bottomWheel.ConfigAllSettings(Motors.DriveL1());
			angleMotor.ConfigAllSettings(Motors.ShooterAngle());
			comArm.ConfigAllSettings(Motors.ShooterComArm());
		}

		//=== Functionality ==============================

		public void SetLaunchAngle(float angle)
		{
			angleMotor.Set(ControlMode.PercentOutput, angle);
		}

		public void SetShotVelocity(float velocity)
		{
			topWheel.Set(ControlMode.PercentOutput, velocity);
			bottomWheel.Set(ControlMode.PercentOutput, -1 * velocity);
		}

		public void Launch()
		{
			// break wheels
			// push forward commencement arm
		}

		//================================================
	}
}
