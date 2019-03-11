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
		private TalonSRX topWheel = new TalonSRX(DeviceIDs.ShooterTop);
		private TalonSRX bottomWheel = new TalonSRX(DeviceIDs.ShooterBottom);
		private TalonSRX angleMotor = new TalonSRX(DeviceIDs.ShooterAngle);
		private TalonSRX comArm = new TalonSRX(DeviceIDs.ShooterComArm);

		private PigeonIMU pigeon = new PigeonIMU(DeviceIDs.ShooterIMU);

		private float launchAngleSP;

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
			launchAngleSP = angle;
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

		public void ControlLoop()
		{
			float[] tiltAngles = new float[3];
			pigeon.GetAccelerometerAngles(tiltAngles);

			Debug.Print("Shooter Angle: " + tiltAngles[0].ToString());
		}

		//================================================
	}
}
