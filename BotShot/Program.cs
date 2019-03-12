// based on PID velocity control example
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/HERO%20C%23/VelocityClosedLoopAuxiliary%5BFeedForward%5D

using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using System.Diagnostics;

//using VelociyClosedLoopAuxiliary.Platform;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using BotShot.Devices;
using BotShot.Utility;
using BotShot.Config;

//=== GPIO PIN SETTINGS ==============================
// Pin map:     https://www.ctr-electronics.com/downloads/pdf/HERO%20User's%20Guide.pdf#page=10&zoom=100,0,621
// Guide:       https://www.ctr-electronics.com/downloads/pdf/HERO%20User's%20Guide.pdf#page=53&zoom=0,0,0
//====================================================

//=== CLOCK PROGRAMMING ==============================
// https://www.ctr-electronics.com/downloads/pdf/HERO%20User's%20Guide.pdf#page=59&zoom=0,0,0
//====================================================

//=== DEVICE PIN PROGRAMMING =========================
// https://www.ctr-electronics.com/downloads/pdf/HERO%20User's%20Guide.pdf#page=36&zoom=0,0,0
//====================================================

//=== MANUAL DRIVER INSTALLATION =====================
// Drivers:             https://www.ctr-electronics.com/downloads/pdf/HERO%20User's%20Guide.pdf#page=29&zoom=0,0,0
// NETMF-Enabling:      https://www.ctr-electronics.com/downloads/pdf/HERO%20User's%20Guide.pdf#page=22&zoom=0,0,0
// LIFEBOAT Imaging:    https://www.ctr-electronics.com/downloads/pdf/HERO%20User's%20Guide.pdf#page=25&zoom=0,0,0
//====================================================

//=== PIDF DOCUMENTATION =============================
// https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html
//====================================================

//=== WIFI-CONTROL METHOD WE SHOULD PROBABLY USE =====
// https://phoenix-documentation.readthedocs.io/en/latest/ch06_PrepRobot.html
//====================================================

namespace BotShot {
	public static class Program {
		public static void Main(){
			Display.Initialize();
			Control.Initialize();				

			//Controller Loop
			while(true)
			{
				Thread.Sleep(10); //Command Unstaler

				if(Control.IsConnected())
				{
					Display.ConnectionSuccess();
					CTRE.Phoenix.Watchdog.Feed(); //Refresh E-stop unlock (Allows the motors to move)

                    // add mode switching in future
                    //Control.ManualMode();
                    Control.AutoAim();
				}
				else
					Display.ConnectionError();
			}
		}
	}
}