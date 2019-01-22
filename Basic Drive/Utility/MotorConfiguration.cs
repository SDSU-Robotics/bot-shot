using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

namespace Basic_Drive.Utility {
    class MotorConfiguration {

        //Pin assignment (Change in )
        public enum MotorID {
            DRIVE_FL = 0,
            DRIVE_FR = 1,
            DRIVE_RL = 2,
            DRIVE_RR = 3,
            SHOOTER_L = 4,
            SHOOTER_R = 5
            }

        //Motor Profiles
        public static TalonSRXConfiguration LeftShooter(){

            TalonSRXConfiguration profile = new TalonSRXConfiguration();

		    //Threshold for zero-motion for the neutral position.
		    profile.neutralDeadband = 0.01f;
			
		    //Peak Speed Config
		    profile.peakOutputForward = 1f;
		    profile.peakOutputReverse = -1f;
			
		    //Ramp Config
		    profile.closedloopRamp = 1.5f;
			
		    //PID Config
		    profile.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
		    profile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

		    //PID Constants
		    profile.slot_0.kP                       = 1.00f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
		    profile.slot_0.kI                       = 0.00f; //Integral Constant.     Controls the steady-state error correction.
		    profile.slot_0.kD                       = 0.00f; //Derivative Constant.   Controls error oscillation.
		    profile.slot_0.kF                       = 0.00f; //Feed Forward Constant. (IDK what this does)
		    profile.slot_0.integralZone             = 900;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
		    profile.slot_0.maxIntegralAccumulator   = 900;   //Maximum value for the integral error accumulator. (IDK what this does)
		    profile.slot_0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
		    profile.slot_0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
		    profile.slot_0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

            return profile;
			}
        public static TalonSRXConfiguration RightShooter(){

            TalonSRXConfiguration profile = new TalonSRXConfiguration();

		    //Threshold for zero-motion for the neutral position.
		    profile.neutralDeadband = 0.01f;
			
		    //Peak Speed Config
		    profile.peakOutputForward = 1f;
		    profile.peakOutputReverse = -1f;
			
		    //Ramp Config
		    profile.closedloopRamp = 1.5f;
			
		    //PID Config
		    profile.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
		    profile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

		    //PID Constants
		    profile.slot_0.kP                       = 1.00f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
		    profile.slot_0.kI                       = 0.00f; //Integral Constant.     Controls the steady-state error correction.
		    profile.slot_0.kD                       = 0.00f; //Derivative Constant.   Controls error oscillation.
		    profile.slot_0.kF                       = 0.00f; //Feed Forward Constant. (IDK what this does)
		    profile.slot_0.integralZone             = 900;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
		    profile.slot_0.maxIntegralAccumulator   = 900;   //Maximum value for the integral error accumulator. (IDK what this does)
		    profile.slot_0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
		    profile.slot_0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
		    profile.slot_0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

            return profile;
			}
        //public static TalonSRXConfiguration WheelFR(){
        //    
        //    }
        //public static TalonSRXConfiguration WheelFL(){
        //    
        //    }
        //public static TalonSRXConfiguration WheelBR(){
        //    
        //    }
        //public static TalonSRXConfiguration WheelBL(){
        //    
        //    }
        }
    }
