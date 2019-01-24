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
		private TalonSRX motorL1 = new TalonSRX(DeviceIDs.DriveL1);
		private TalonSRX motorL2 = new TalonSRX(DeviceIDs.DriveL2);
		private TalonSRX motorR1 = new TalonSRX(DeviceIDs.DriveR1);
		private TalonSRX motorR2 = new TalonSRX(DeviceIDs.DriveR2);

		// initializer
        public DriveBase()
		{
            motorL1.ConfigAllSettings(Motors.driveL1());
            motorL2.ConfigAllSettings(Motors.driveL2());
            motorR1.ConfigAllSettings(Motors.driveR1());
            motorR2.ConfigAllSettings(Motors.driveR2());
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

			motorL1.Set(ControlMode.PercentOutput, percentOutput);
			motorL2.Set(ControlMode.PercentOutput, percentOutput);
		}

		public void SetRightPercent(float percentOutput)
		{
			// limit values
			if (percentOutput < -1.0f)
				percentOutput = -1.0f;
			else if (percentOutput > 1.0f)
				percentOutput = 1.0f;

			motorR1.Set(ControlMode.PercentOutput, percentOutput);
			motorR2.Set(ControlMode.PercentOutput, percentOutput);
		}

		public void Stop()
		{
			motorL1.Set(ControlMode.PercentOutput, 0);
			motorL2.Set(ControlMode.PercentOutput, 0);
			motorR1.Set(ControlMode.PercentOutput, 0);
			motorR2.Set(ControlMode.PercentOutput, 0);
		}
	}
}
