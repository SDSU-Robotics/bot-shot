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

using Basic_Drive.Devices;
using Basic_Drive.Utility;

namespace Basic_Drive{
    public class Program{

        private static Shooter              shooter;
        private static GameController       controller;
        private static GameControllerValues gcValues;

        public static void Initialization(){
            shooter    = new Shooter(Constants.MotorID.SHOOTER_L, Constants.MotorID.SHOOTER_R);
            controller = new GameController(new UsbHostDevice(0));
            gcValues   = new GameControllerValues();
            }
        public static void DisplayMOTD(){
            //If there are any display lights or sounds that the robot can make to show that it is working.
            }
        public static void DisplayConnectionError(){
            //If there are any display lights or sounds that the robot can make to show that it has connection issues.
            }
        public static void DisplayConnectionSuccess(){
            //If there are any display lights or sounds that the robot can make to show that it has connection issues.
            }
        public static void ControllerButtons(){

            controller.GetAllValues(ref gcValues);
            Constants.Button_ID keydown = (Constants.Button_ID) gcValues.flagBits;

            //Switch based on button bitflags (Button mapping might be wrong)
            switch(keydown){
                case Constants.Button_ID.X:
                    break;
                case (Constants.Button_ID.X & Constants.Button_ID.Y):
                    break;
                default:
                    break;
                }
            }
        public static void ControllerAxes(){
            
            }

        public static void Main(){

            Initialization(); //Initialize components
            DisplayMOTD();    //Bootup Display (Means that we have entered the control loop in case of in-competition reboot)

            //Control Loop
            while(true){
                
                switch(controller.GetConnectionStatus()){
                    case UsbDeviceConnection.Connected:
                        DisplayConnectionSuccess();
                        ControllerButtons();
                        ControllerAxes();
                        break;
                    case UsbDeviceConnection.NotConnected:
                        DisplayConnectionError();
                        Thread.Sleep(10); //Might be outside the case statement to wait for input for every command.
                        break;
                    }

                }
            }
        }
    }

/*
		    //Finalize settings and pass settings to motor object
		    sT.ConfigAllSettings(MotorGroups.Shooter.Settings);
			
		    //Config Constants
		    int timeoutSec = 30;
			
		    // Set status frame periods to ensure we don't have stale data
		    sT.SetStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, timeoutSec);
			
		    //Set neutral mode
		    sT.SetNeutralMode(NeutralMode.Brake);
			
		    sT.SetInverted(false);
		    sT.SetSensorPhase(true);
            

			float SetPoint = 10000; //RPM

			// loop forever
			while (true)
			{
				//if (CTRE.Phoenix.) { 
				if (gp.GetConnectionStatus() == UsbDeviceConnection.Connected)
				{

					//sT.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, .1);
					//sT.Set(ControlMode.Velocity, target_unitsPer100ms);
						
					if ( gp.GetButton(0x1) )
					{
						sT.Set(ControlMode.Velocity, SetPoint);//TransFn(SetPoint) );
					}
					else
					{
						sT.Set(ControlMode.Velocity, 0);
					}
						
						// GetSelectedSensorVelocity() returns counts per 100 ms

						//Debug.Print("RPM:\t"           + ToRpm( sT.GetSelectedSensorVelocity(0)).ToString() );
						//Debug.Print("Velocity:\t"      + sT.GetSelectedSensorVelocity(0).ToString() );
						//Debug.Print("SetPointasRpm:\t" + ToRpm( SetPoint ) );

						//Debug.Print("Position:\t" + sT.GetSelectedSensorPosition(0).ToString());

						CTRE.Phoenix.Watchdog.Feed();
				}
				//}
                //wait a bit
                System.Threading.Thread.Sleep(10);
            }
        }
    }
}
*/