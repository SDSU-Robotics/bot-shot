using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using BotShot.Utility;
using BotShot.Config;

namespace BotShot.Devices
{
	public class Pickup
	{

		//Objects associated with the shooter
		private TalonSRX motor = new TalonSRX(DeviceIDs.Pickup);

		private PigeonIMU pigeon = new PigeonIMU(DeviceIDs.PickupIMU);

		private float angleSP;

        private const float kp = 0.01f;
        private const float ki = 0.0001f;
        private const float kd = 0.0f;
        private const float ILimit = 1000.0f;
        private const float maxOut = 0.4f;  

        private float error = 0.0f;
        private float integral = 0;
        private float derivative = 0;
        private float lastError = 0;
        private float lastAngle = 0;
        private float output = 0;

        // constructor
        public Pickup()
		{
			motor.ConfigAllSettings(Motors.DriveL());
		}

		//=== Functionality ==============================

		public void SetPickupAngle(float angle)
		{
			angleSP = angle;
		}

		public void ControlLoop()
		{
            float[] tiltAngles = new float[3];
            pigeon.GetAccelerometerAngles(tiltAngles);
            float angle = tiltAngles[2];

            Debug.Print("Pickup Angle: " + angle.ToString());

            if (angle < 0.001)
                motor.Set(ControlMode.PercentOutput, 0.0f);
            else
            {
                error = angleSP - tiltAngles[2];

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

                motor.Set(ControlMode.PercentOutput, -1 * output);

                lastError = error;
            }
		}

		//================================================
	}
}
