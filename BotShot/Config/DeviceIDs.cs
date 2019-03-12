using System;
using Microsoft.SPOT;

namespace BotShot.Config
{
	class DeviceIDs
	{
		public static int PDP = 0;

		public static int DriveL = 1;
		public static int DriveR = 2;

		public static int ShooterTop = 5;
		public static int ShooterBottom = 6;
		public static int ShooterAngle = 8;
		public static int ShooterComArm = 7;

		public static int Pickup = 9;

		public static int ShooterIMU = 10;
		public static int PickupIMU = 11;

        public static Microsoft.SPOT.Hardware.Cpu.Pin ShooterCamPin6 = CTRE.HERO.IO.Port8.Pin6;
        public static Microsoft.SPOT.Hardware.Cpu.Pin PickupCamPin6 = CTRE.HERO.IO.Port1.Pin6;

    }
}
