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
			topWheel.ConfigAllSettings(Motors.ShooterTop());
			bottomWheel.ConfigAllSettings(Motors.ShooterBottom());
			angleMotor.ConfigAllSettings(Motors.ShooterAngle());
			comArm.ConfigAllSettings(Motors.ShooterComArm());

            topWheel.SetNeutralMode(NeutralMode.Brake);
            bottomWheel.SetNeutralMode(NeutralMode.Brake);

            topWheel.SetInverted(false);
            topWheel.SetSensorPhase(true);
            bottomWheel.SetInverted(false);
            bottomWheel.SetSensorPhase(true);
        }

		//=== Functionality ==============================

		public void SetLaunchAngle(float angle)
		{
			angleMotor.Set(ControlMode.PercentOutput, angle);
		}

		public void SetShotVelocity(float velocity)
		{
            if (velocity > 0.01)
            {
                topWheel.Set(ControlMode.Velocity, -1 * Conversion.FromRpm(velocity));
                bottomWheel.Set(ControlMode.Velocity, Conversion.FromRpm(velocity));
            }
            else
            {
                topWheel.Set(ControlMode.PercentOutput, 0.0f);
                bottomWheel.Set(ControlMode.PercentOutput, 0.0f);
            }
        }

		public void Launch()
		{
			// break wheels
			// push forward commencement arm
		}

		//================================================
	}
}
