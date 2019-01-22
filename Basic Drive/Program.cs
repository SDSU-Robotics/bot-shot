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

namespace Basic_Drive
{
    public class Program
    {

		public static float ToRpm(float targetRpm)
		{
			//float UnitsPer100ms = targetRpm * 4096.0f / 125.0f;
			float UnitsPer100ms = targetRpm / 4096.0f * 600.0f;
			return UnitsPer100ms;
		}
		public static float FromRpm(float input)
		{
			//float UnitsPer100ms = targetRpm * 4096.0f / 125.0f;
			float output = input * 4096.0f / 600.0f;
			return output;
		}

		private static TalonSRXConfiguration shooterCfg = new TalonSRXConfiguration();

        public static void Main()
        {
			//Create Objects (Controller and MotorController)
            GameController gp = new GameController(new UsbHostDevice(0));
            TalonSRX       sT = new TalonSRX(5);

			//Threshold for zero-motion for the neutral position.
			shooterCfg.neutralDeadband = 0.01f;
			
			//Peak Speed Config
			shooterCfg.peakOutputForward = 1f;
			shooterCfg.peakOutputReverse = -1f;
			
			//Ramp Config
			shooterCfg.closedloopRamp = 1.5f;
			
			//PID Config
			shooterCfg.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
			shooterCfg.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

			//PID Constants
			shooterCfg.slot_0.kP                       = 1.00f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
			shooterCfg.slot_0.kI                       = 0.00f; //Integral Constant.     Controls the steady-state error correction.
			shooterCfg.slot_0.kD                       = 0.00f; //Derivative Constant.   Controls error oscillation.
			shooterCfg.slot_0.kF                       = 0.00f; //Feed Forward Constant. (IDK what this does)
			shooterCfg.slot_0.integralZone             = 900;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
			shooterCfg.slot_0.maxIntegralAccumulator   = 900;   //Maximum value for the integral error accumulator. (IDK what this does)
			shooterCfg.slot_0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
			shooterCfg.slot_0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
			shooterCfg.slot_0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)
			
			//shooterCfg.auxPIDPolarity = false;
			
			//Finalize settings and pass settings to motor object
			sT.ConfigAllSettings(shooterCfg);
			
			//Config Constants
			int timeoutSec = 30;
			
			// Set status frame periods to ensure we don't have stale data
			sT.SetStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, timeoutSec);
			
			//Set neutral mode
			sT.SetNeutralMode(NeutralMode.Brake);
			
			sT.SetInverted(false);
			sT.SetSensorPhase(true);

			float SetPoint = 10000; //RPM
			Stopwatch sw = new Stopwatch();
			sw.Start();

			/* loop forever */
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

					Debug.Print(sw.DurationMs.ToString() + ", " + sT.GetSelectedSensorVelocity(0).ToString());
						
						// GetSelectedSensorVelocity() returns counts per 100 ms

						//Debug.Print("RPM:\t"           + ToRpm( sT.GetSelectedSensorVelocity(0)).ToString() );
						//Debug.Print("Velocity:\t"      + sT.GetSelectedSensorVelocity(0).ToString() );
						//Debug.Print("SetPointasRpm:\t" + ToRpm( SetPoint ) );

						//Debug.Print("Position:\t" + sT.GetSelectedSensorPosition(0).ToString());

						CTRE.Phoenix.Watchdog.Feed();
				}
				//}
                /* wait a bit */
                System.Threading.Thread.Sleep(10);
            }
        }
    }
}
