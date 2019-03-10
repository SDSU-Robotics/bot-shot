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
		public TalonSRX motor = new TalonSRX(DeviceIDs.Pickup);

		// constructor
		public Pickup()
		{
			motor.ConfigAllSettings(Motors.DriveL1());
		}

		//=== Functionality ==============================

		public void SetPickupAngle(float power)
		{
			motor.Set(ControlMode.PercentOutput, power);
		}
		

		//================================================
	}
}
