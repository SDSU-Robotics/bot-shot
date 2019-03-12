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

		private const float kp = 0.006f;
		private const float ki = 0.0001f;
		private const float kd = 0.01f;
		private const float ILimit = 1000.0f;
		private const float maxOut = 0.2f;

		private float error = 0.0f;
		private float integral = 0;
		private float derivative = 0;
		private float lastError = 0;
		private float lastAngle = 0;
		private float output = 0;

		private const int IMAGE_WIDTH = 319;

		// constructor
		public Shooter()
		{
			topWheel.ConfigAllSettings(Motors.ShooterTop());
			bottomWheel.ConfigAllSettings(Motors.ShooterBottom());
			angleMotor.ConfigAllSettings(Motors.ShooterAngle());
			comArm.ConfigAllSettings(Motors.ShooterComArm());

            topWheel.SetNeutralMode(NeutralMode.Brake);
            bottomWheel.SetNeutralMode(NeutralMode.Brake);
			comArm.SetNeutralMode(NeutralMode.Brake);

            topWheel.SetInverted(false);
            topWheel.SetSensorPhase(true);
            bottomWheel.SetInverted(false);
            bottomWheel.SetSensorPhase(true);
			comArm.SetInverted(false);
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
			// launch angle
			float[] tiltAngles = new float[3];
			pigeon.GetAccelerometerAngles(tiltAngles);
		}

		public float centeringPID(uint targetX)
		{
			error = IMAGE_WIDTH / 2 - targetX;

			integral = integral + error;
			if (integral > ILimit)
			{
				integral = ILimit;
			}
			if (integral < -ILimit)
			{
				integral = -ILimit;
			}

			derivative = error - lastError;

			output = kp * error + ki * integral + kd * derivative;

			if (output > maxOut)
			{
				output = maxOut;
			}
			if (output < -1 * maxOut)
			{
				output = -1 * maxOut;
			}

			lastError = error;

			return -1 * output;
		}
	}

		//================================================
}
