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

			Debug.Print("Pickup Angle: " + tiltAngles[2].ToString());
		}
		

		//================================================
	}
}
