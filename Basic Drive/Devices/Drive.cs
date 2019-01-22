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
    public class Drive {

        //Objects associated with the shooter
        public TalonSRX WheelFR{get; private set;}
        public TalonSRX WheelFL{get; private set;}
        public TalonSRX WheelBR{get; private set;}
        public TalonSRX WheelBL{get; private set;}

        public Drive(){
            WheelFR.ConfigAllSettings(Motors.WheelFR());
            WheelFL.ConfigAllSettings(Motors.WheelFL());
            WheelBR.ConfigAllSettings(Motors.WheelBR());
            WheelBL.ConfigAllSettings(Motors.WheelBL());
            }

        //=== Functionality ==============================

        //================================================

        }
    }
