// based on PID velocity control example
// https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/HERO%20C%23/VelocityClosedLoopAuxiliary%5BFeedForward%5D

using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

//using VelociyClosedLoopAuxiliary.Platform;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Sensors;

namespace Basic_Drive
{
    public class Program
    {

        const int kTimeoutMs = 30;
        const float kNeutralDeadband = 0.01f;
        const float velocitykP = 0.5f;
        const float velocitykI = 0f;
        const float velocitykD = 0.01f;
        const float velocitykF = 0.0f;
        const int kSlot_Velocit = 0; //?
        const int kIzone = 10;  //intergration zone, ms
        const float kPeakOutput = .9f;       


        public static void Main()
        {
            /* create a gamepad object */
            CTRE.Phoenix.Controller.GameController gp = new CTRE.Phoenix.Controller.GameController(new CTRE.Phoenix.UsbHostDevice(0));

            // Create Talon objects
            CTRE.Phoenix.MotorControl.CAN.TalonSRX sT = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(5);

            sT.SetNeutralMode(NeutralMode.Coast);

      
            /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
            sT.ConfigSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, kTimeoutMs); //0 or 1?

            /* Scale the Feedback Sensor using a coefficient */
            sT.ConfigSelectedFeedbackCoefficient(0.5f,                       // Coefficient
                                                                    0,      // PID Slot of Source
                                                                    kTimeoutMs);		// Configuration Timeout

            /* Configure output and sensor direction */
            sT.SetInverted(false);
            sT.SetSensorPhase(true);

            /* Set status frame periods to ensure we don't have stale data */
            sT.SetStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, kTimeoutMs);


            /* Configure Neutral Deadband */
            sT.ConfigNeutralDeadband(0.01f, kTimeoutMs);

            //sT.ConfigSelectedFeedbackCoefficient(0.5f, 0, 30);

            /* FPID Gains for turn closed loop */
            sT.Config_kP(kSlot_Velocit, velocitykP, kTimeoutMs);
            sT.Config_kI(kSlot_Velocit, velocitykI, kTimeoutMs);
            sT.Config_kD(kSlot_Velocit, velocitykD, kTimeoutMs);
            sT.Config_kF(kSlot_Velocit, velocitykF, kTimeoutMs);
            sT.Config_IntegralZone(kSlot_Velocit, kIzone, kTimeoutMs);
            sT.ConfigClosedLoopPeakOutput(kSlot_Velocit, kPeakOutput, kTimeoutMs);
            sT.ConfigAllowableClosedloopError(kSlot_Velocit, 0, kTimeoutMs);

            double target_RPM;
            double target_unitsPer100ms;

            /* loop forever */
            while (true)
            {
                if (gp.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                {

                    target_RPM = 1000;
                    target_unitsPer100ms = target_RPM * 4096.0 / 600.0;


                    //sT.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, .1);
                    sT.Set(ControlMode.Velocity, target_unitsPer100ms);

                    Debug.Print((sT.GetSelectedSensorVelocity(0) / 4096.0 * 600.0).ToString());

                    CTRE.Phoenix.Watchdog.Feed();
                }
                
                /* wait a bit */
                System.Threading.Thread.Sleep(10);
            }
        }
    }
}
