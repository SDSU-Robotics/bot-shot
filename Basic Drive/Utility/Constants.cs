using System;
using Microsoft.SPOT;

namespace Basic_Drive.Utility {

    public static class Constants {

        public enum MotorID {
            DRIVE_FL = 0,
            DRIVE_FR = 1,
            DRIVE_RL = 2,
            DRIVE_RR = 3,
            SHOOTER_L = 4,
            SHOOTER_R = 5
            }

        [Flags]
        public enum Button_ID {
            A = 0x01,
            B = 0x02,
            X = 0x04,
            Y = 0x08
            }

        }

    }
