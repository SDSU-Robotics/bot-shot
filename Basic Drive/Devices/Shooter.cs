using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using Basic_Drive.Utility;
using Basic_Drive.Config;

namespace Basic_Drive.Devices{
    public class Shooter{

        //Objects associated with the shooter
        public TalonSRX LeftSpinner{get; private set;}
        public TalonSRX RightSpinner{get; private set;}

        public Shooter(){
            LeftSpinner.ConfigAllSettings(Motors.LeftShooter());
            LeftSpinner.ConfigAllSettings(Motors.RightShooter());
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
