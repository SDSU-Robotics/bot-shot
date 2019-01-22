using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using Basic_Drive.Utility;

namespace Basic_Drive.Devices{
    public class Shooter{

        //Objects associated with the shooter
        public TalonSRX LeftSpinner{get; private set;}
        public TalonSRX RightSpinner{get; private set;}

        public Shooter(){
            LeftSpinner.ConfigAllSettings(MotorConfiguration.LeftShooter());
            LeftSpinner.ConfigAllSettings(MotorConfiguration.RightShooter());
            }

        //=== Functionality ==============================
        public void SetArmAngle(float angle){

            }
        public void SetShotVelocity(float velocity){
            
            }
        public void LoadBall(){
            //Maybe return success value?
            }
        //================================================

        }
    }
