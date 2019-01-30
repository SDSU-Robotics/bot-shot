using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using BotShot.Devices;

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

		[Flags]
		public enum Button_ID
		{
			A = 0x01,
			B = 0x02,
			X = 0x04,
			Y = 0x08
		}

		// private attributes
		static CTRE.Phoenix.Controller.GameController gp;
		static DriveBase driveBase;
		static Shooter shooter;

		// initialization procedures
		public static void Initialize()
		{
			gp = new CTRE.Phoenix.Controller.GameController(new CTRE.Phoenix.UsbHostDevice(0));
			driveBase = new DriveBase();
			shooter = new Shooter();
			Display.UartPrint("[ Control initialization complete. ]\r\n");
		}

		// is the controller connected
		public static bool IsConnected()
		{
			return gp.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected;
		}

		// driving mode
		public static void DriveMode()
		{
			float speed = -1.0f * gp.GetAxis(AXIS_ID.LEFT_Y); // left vertical
			float turn = gp.GetAxis(AXIS_ID.RIGHT_X); // right horizontal

			float lSpeed = MAX_SPEED * 0.5f * speed + MAX_SPEED * 0.5f * turn;
			float rSpeed = MAX_SPEED * 0.5f * speed - MAX_SPEED * 0.5f * turn;

			driveBase.SetLeftPercent(lSpeed);
			driveBase.SetRightPercent(rSpeed);
		}
	}
}
