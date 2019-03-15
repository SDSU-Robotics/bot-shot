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
		static PixyCam pixyCam = new PixyCam(10000, DeviceIDs.pixyCam);

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

				float lSpeed = MAX_SPEED * 0.99f * speed + MAX_SPEED * 0.5f * turn;
				float rSpeed = MAX_SPEED * 0.99f * speed - MAX_SPEED * 0.5f * turn;

				driveBase.SetLeftPercent(lSpeed);
				driveBase.SetRightPercent(rSpeed);
			}
			else if (gp.GetButton(Button_ID.LB))
				AutoPickup();
			else if (gp.GetButton(Button_ID.RB))
				AutoAim();
			// pickup/shoot mode
			else
			{
				// stop
				driveBase.SetLeftPercent(0.0f);
				driveBase.SetRightPercent(0.0f);

				float pickupSP = gp.GetAxis(AXIS_ID.LEFT_Y) * 0.5f;
				pickup.SetPickupAngle(pickupSP);

				float shooterAngleSpeed = gp.GetAxis(AXIS_ID.RIGHT_Y) / 0.5f;
				shooter.SetLaunchAngle(shooterAngleSpeed);
			}

			if (gp.GetButton(4))
				shooter.SetShotVelocity(1300);
			else
				shooter.SetShotVelocity(0.0f);

            if (gp.GetButton(1))
                shooter.moveComArm(-0.25f);
            else if (gp.GetButton(2))
                shooter.moveComArm(0.25f);
            else
                shooter.moveComArm(0.0f);

            // update control loops
            shooter.ControlLoop();
			pickup.ControlLoop();
		}

		// print out computer vision info
		public static void DisplayCV()
		{
			PixyBlock pixyData = new PixyBlock();

			pixyCam.Process();

			pixyCam.GetBlock(pixyData);

			Debug.Print(pixyData.ToString());
		}

		public static void AutoAim()
        {
			Debug.Print("------AutoAim------");
			// centering

			pixyCam.SetBrightness(0x22);

			PixyBlock pixyData = new PixyBlock();

			int count = 0;
			do
			{
				pixyCam.Process();
				pixyCam.GetBlock(pixyData);
				++count;
				if (count > 100)
					break;
			} while (pixyData.Signature != 2 || pixyData.Area < 20); // discard other data

			float adjustment = shooter.centeringPID(pixyData.X);
			driveBase.SetLeftPercent(adjustment);
			driveBase.SetRightPercent(-1 * adjustment);

			Debug.Print("Adjustment: " + adjustment.ToString() + "\n");
			DisplayCV();
        }

        public static void AutoPickup()
        {
			Debug.Print("------AutoPickup------");
			// centering

			pixyCam.SetBrightness(0x70);

			PixyBlock pixyData = new PixyBlock();

			int count = 0;
			do
			{
				pixyCam.Process();
				pixyCam.GetBlock(pixyData);
				++count;
				if (count > 100)
					break;
			} while (pixyData.Signature != 1 || pixyData.Area < 20); // discard other data

			float adjustment = pickup.centeringPID(pixyData.X);
			driveBase.SetLeftPercent(adjustment);
			driveBase.SetRightPercent(-1 * adjustment);

			Debug.Print("Adjustment: " + adjustment.ToString() + "\n");
			DisplayCV();
		}
       
	}
}
