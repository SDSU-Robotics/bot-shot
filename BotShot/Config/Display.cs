using System;
using Microsoft.SPOT;

namespace BotShot.Config {
    static public class Display {
        public static void MOTD(){
            //If there are any display lights or sounds that the robot can make to show that it is working.
            Debug.Print("[POWER ON]");
            }
        public static void ConnectionError(){
            //If there are any display lights or sounds that the robot can make to show that it has connection issues.
            }
        public static void ConnectionSuccess(){
            //If there are any display lights or sounds that the robot can make to show that it has connection issues.
            }
        }
    }
