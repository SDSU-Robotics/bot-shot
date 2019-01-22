using System;
using Microsoft.SPOT;

using CTRE.Phoenix;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Sensors;

namespace Basic_Drive.Config {
    class FieldControls : GameController{

        private Button_ID Keydown;
        private JoyStick LeftJoyStick = new JoyStick();
        private JoyStick RightStick   = new JoyStick();
        private Trigger  LeftTrigger  = new Trigger();
        private Trigger  RightTrigger = new Trigger();

        [Flags] public enum Button_ID {
            A = 0x01,
            B = 0x02,
            X = 0x04,
            Y = 0x08
            }

        public FieldControls(IGameControllerValuesProvider provider, uint idx = 0) : base(provider, idx){
            }

        public void UpdateValues(){

            GameControllerValues gcValues = new GameControllerValues();
            GetAllValues(ref gcValues);

            Keydown                 = (Button_ID) gcValues.flagBits;
            LeftJoyStick.Horizontal = GetAxis(Xbox360Gamepad.kAxis_LeftX);
            LeftJoyStick.Vertical   = GetAxis(Xbox360Gamepad.kAxis_LeftY);
            RightStick.Horizontal   = GetAxis(Xbox360Gamepad.kAxis_RightX);
            RightStick.Vertical     = GetAxis(Xbox360Gamepad.kAxis_RightY);
            LeftTrigger.Value       = GetAxis(Xbox360Gamepad.kAxis_LeftShoulder);
            RightTrigger.Value      = GetAxis(Xbox360Gamepad.kAxis_RightShoulder);
            }

        public void ExecuteButtons(){

            //Switch based on button bitflags (Button mapping might be wrong)
            switch(Keydown){
                case Button_ID.X:
                    break;
                case (Button_ID.X & Button_ID.Y):
                    break;
                default:
                    break;
                }

            }
        public void ExecuteAxes(){
            
            }

        public class JoyStick {
            public float Vertical{get;set;}
            public float Horizontal{get;set;}
            }
        public class Trigger {
            public float Value{get;set;}
            }

        }
    }
