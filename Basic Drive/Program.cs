using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace Basic_Drive
{
    public class Program
    {
        public static void Main()
        {
            /* create a gamepad object */
            CTRE.Phoenix.Controller.GameController gp = new CTRE.Phoenix.Controller.GameController(new CTRE.Phoenix.UsbHostDevice(0));

            // Create Talong objects
            CTRE.Phoenix.MotorControl.CAN.TalonSRX l1 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(1);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX l2 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(2);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX r1 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(3);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX r2 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(4);

            /* loop forever */
            while (true)
            {
                if (gp.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                {
                    double L_speed = gp.GetAxis(1) / -3.0;
                    double R_speed = gp.GetAxis(5) / -3.0;

                    //Debug.Print("axis:" + R_speed);
                    l1.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, L_speed);
                    l2.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, L_speed);
                    r1.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, R_speed);
                    r2.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, R_speed);

                    CTRE.Phoenix.Watchdog.Feed();
                }
                
                /* wait a bit */
                System.Threading.Thread.Sleep(10);
            }
        }
    }
}
