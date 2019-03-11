using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using BotShot.Utility;
using BotShot.Config;

namespace BotShot.Devices {
	public class DriveBase {
		// Objects associated with the drive base
		private TalonSRX motorL = new TalonSRX(DeviceIDs.DriveL);
		private TalonSRX motorR = new TalonSRX(DeviceIDs.DriveR);

		// constructor
		public DriveBase()
		{
			motorL.ConfigAllSettings(Motors.DriveL());
			motorR.ConfigAllSettings(Motors.DriveR());
		}

		//=== Functionality ==============================

		//================================================

		public void SetLeftPercent(float percentOutput)
		{
			// limit values
			if (percentOutput < -1.0f)
				percentOutput = -1.0f;
			else if (percentOutput > 1.0f)
				percentOutput = 1.0f;

			motorL.Set(ControlMode.PercentOutput, percentOutput);
		}

		public void SetRightPercent(float percentOutput)
		{
			// limit values
			if (percentOutput < -1.0f)
				percentOutput = -1.0f;
			else if (percentOutput > 1.0f)
				percentOutput = 1.0f;

			motorR.Set(ControlMode.PercentOutput, percentOutput);
		}

		public void Stop()
		{
			motorL.Set(ControlMode.PercentOutput, 0);
			motorR.Set(ControlMode.PercentOutput, 0);
		}
	}
}
