using System;
using Microsoft.SPOT;

namespace Basic_Drive.Utility {

    public static class Constants {

        [Flags]
        public enum Button_ID {
            A = 0x01,
            B = 0x02,
            X = 0x04,
            Y = 0x08
            }

        }

    }
