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
            // constants
            const double MAX_SPEED = 0.2;

            /* create a gamepad object */
            CTRE.Phoenix.Controller.GameController gp = new CTRE.Phoenix.Controller.GameController(new CTRE.Phoenix.UsbHostDevice(0));

            // Create Talong objects
            CTRE.Phoenix.MotorControl.CAN.TalonSRX l1 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(1);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX l2 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(2);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX r1 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(3);
            CTRE.Phoenix.MotorControl.CAN.TalonSRX r2 = new CTRE.Phoenix.MotorControl.CAN.TalonSRX(4);

            // speeds
            double l_speed = 0.0;
            double r_speed = 0.0;
            double speed = 0.0;
            double turn = 0.0;

            /* loop forever */
            while (true)
            {	
                if (gp.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                {
                    speed = -1.0 * gp.GetAxis(1); // left vertical
                    turn = gp.GetAxis(2); // right horizontal
					
					

                    l_speed = MAX_SPEED * 0.5 * speed + MAX_SPEED * 0.5 * turn;
                    r_speed = MAX_SPEED * 0.5 * speed - MAX_SPEED * 0.5 * turn;

					//Debug.Print("Left: " + l_speed + "   Right: " + r_speed);
					
                    l1.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, l_speed);
                    l2.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, l_speed);
                    r1.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, r_speed);
                    r2.Set(CTRE.Phoenix.MotorControl.ControlMode.PercentOutput, r_speed);

                    CTRE.Phoenix.Watchdog.Feed();
                }
                
                /* wait a bit */
                System.Threading.Thread.Sleep(10);
            }
        }
    }
}
