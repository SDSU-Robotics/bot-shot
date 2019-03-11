using System;
using Microsoft.SPOT;

namespace BotShot.Utility {
	public static class Conversion {
		public static float ToRpm(float targetRpm){
				float UnitsPer100ms = targetRpm / 27.30667f;
				return UnitsPer100ms;
			}
		public static float FromRpm(float input){
				float output = input * 27.30667f;
				return output;
			}
		}
	}
