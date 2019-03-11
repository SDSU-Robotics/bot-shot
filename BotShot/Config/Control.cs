using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using BotShot.Devices;
using BotShot.Utility;

namespace BotShot.Config
{
	public static class Control
	{
		public static float MAX_SPEED = 0.4f;

		public static class AXIS_ID
		{
			public const uint LEFT_X = 0;
			public const uint LEFT_Y = 1;
			public const uint RIGHT_X = 2;
			public const uint RIGHT_Y = 5;
		}

		public static class Button_ID
		{
			public const uint X = 1;
			public const uint A = 2;
			public const uint B = 3;
			public const uint Y = 4;
			public const uint LB = 5;
			public const uint RB = 6;
			public const uint LT = 7;
			public const uint RT = 8;
			public const uint SEL = 9;
			public const uint START = 10;

		}

		// private attributes
		static CTRE.Phoenix.Controller.GameController gp;
		static DriveBase driveBase;
		static Shooter shooter;
		static Pickup pickup;

		// initialization procedures
		public static void Initialize()
		{
			gp = new CTRE.Phoenix.Controller.GameController(new CTRE.Phoenix.UsbHostDevice(0));
			driveBase = new DriveBase();
			shooter = new Shooter();
			pickup = new Pickup();
			Display.UartPrint("[ Control initialization complete. ]\r\n");
		}

		// is the controller connected
		public static bool IsConnected()
		{
			return gp.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected;
		}

		// driving mode
		public static void ManualMode()
		{
			// drive mode
			if (gp.GetButton(Button_ID.LT) && gp.GetButton(Button_ID.RT))
			{
				float speed = -1.0f * gp.GetAxis(AXIS_ID.LEFT_Y); // left vertical
				float turn = gp.GetAxis(AXIS_ID.RIGHT_X); // right horizontal

				float lSpeed = MAX_SPEED * 0.5f * speed + MAX_SPEED * 0.5f * turn;
				float rSpeed = MAX_SPEED * 0.5f * speed - MAX_SPEED * 0.5f * turn;

				driveBase.SetLeftPercent(lSpeed);
				driveBase.SetRightPercent(rSpeed);
			}
			// pickup/shoot mode
			else
			{
				// stop
				driveBase.SetLeftPercent(0.0f);
				driveBase.SetRightPercent(0.0f);

				float pickupSP = gp.GetAxis(AXIS_ID.LEFT_Y) * 30.0f + 90.0f;
				pickup.SetPickupAngle(pickupSP);

				float shooterAngleSpeed = gp.GetAxis(AXIS_ID.RIGHT_Y) / 2.0f;
				shooter.SetLaunchAngle(shooterAngleSpeed);
			}

			if (gp.GetButton(4))
				shooter.SetShotVelocity(1000.0f);
			else
				shooter.SetShotVelocity(0.0f);

			// update control loops
			shooter.ControlLoop();
			pickup.ControlLoop();
		}
	}
}
