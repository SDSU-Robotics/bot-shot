using System;
using Microsoft.SPOT;

namespace Basic_Drive.Utility {
    public static class Conversion {
		public static float ToRpm(float targetRpm){
			    //float UnitsPer100ms = targetRpm * 4096.0f / 125.0f;
			    float UnitsPer100ms = targetRpm / 4096.0f * 600.0f;
			    return UnitsPer100ms;
		    }
		public static float FromRpm(float input){
			    //float UnitsPer100ms = targetRpm * 4096.0f / 125.0f;
			    float output = input * 4096.0f / 600.0f;
			    return output;
		    }
        }
    }
