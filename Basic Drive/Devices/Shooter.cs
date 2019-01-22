using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using Basic_Drive.Utility;


//=== PIDF DOCUMENTATION =====================================================
// https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html
//============================================================================

//=== WIFI-CONTROL METHOD WE SHOULD PROBABLY USE =============================
// https://phoenix-documentation.readthedocs.io/en/latest/ch06_PrepRobot.html
//============================================================================


namespace Basic_Drive.Devices{
    public class Shooter{

        //Backing-fields
        public TalonSRXConfiguration leftSettings;
        public TalonSRXConfiguration rightSettings;

        //Settings (Automatically reflects on motors)
        public TalonSRXConfiguration LeftSettings{
            get{
                return leftSettings;
                } 
            private set{
                leftSettings = value;
                LeftSpinner.ConfigAllSettings(leftSettings);
                }
            }
        public TalonSRXConfiguration RightSettings{
            get{
                return rightSettings;
                } 
            private set{
                rightSettings = value;
                RightSpinner.ConfigAllSettings(rightSettings);
                }
            }

        public TalonSRX LeftSpinner{get; private set;}
        public TalonSRX RightSpinner{get; private set;}
        


        public Shooter(Constants.MotorID LeftDeviceId, Constants.MotorID RightDeviceId){
            LeftSettings = LeftMotorProfile();
            RightSettings = RightMotorProfile();
            }

        //Motor Profiles
        private TalonSRXConfiguration LeftMotorProfile(){

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
        private TalonSRXConfiguration RightMotorProfile(){

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

        //Functionality
        public void SetArmAngle(float angle){

            }
        public void SetShotVelocity(float velocity){
            
            }
        public void LoadBall(){
            //Maybe return success value?
            }

        }
    }
