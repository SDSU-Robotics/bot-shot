using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

using Basic_Drive.Utility;

namespace Basic_Drive.Devices {
    class Drive {

        //Objects associated with the shooter
        public TalonSRX WheelFR{get; private set;}
        public TalonSRX WheelFL{get; private set;}
        public TalonSRX WheelBR{get; private set;}
        public TalonSRX WheelBL{get; private set;}

        public Drive(){
            WheelFR.ConfigAllSettings(MotorConfiguration.WheelFR());
            WheelFL.ConfigAllSettings(MotorConfiguration.WheelFL());
            WheelBR.ConfigAllSettings(MotorConfiguration.WheelBR());
            WheelBL.ConfigAllSettings(MotorConfiguration.WheelBL());
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
