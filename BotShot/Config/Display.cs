using System;
using Microsoft.SPOT;

namespace BotShot.Config {
	static public class Display {
		static System.IO.Ports.SerialPort uart;
		static bool connected = true;

		public static void Initialize()
		{
			uart = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port1.UART, 9600);
			uart.Open();

			UartPrint("\x1b[2J[H");
			Display.UartPrint("[ Display initialization complete. ]\r\n");
		}

		public static void ConnectionError()
		{
			if (connected)
			{
				connected = false;
				UartPrint("Controller connection lost");
			}
			else
				UartPrint(".");
		}

		public static void ConnectionSuccess()
		{
			if (!connected)
			{
				connected = true;
				UartPrint("Connection restored\r\n");
			}
		}

		public static void UartPrint(String msg)
		{
			byte[] data = new byte[msg.Length];
			for (int i = 0; i < msg.Length; ++i)
				data[i] = (byte)msg[i];

			uart.Write(data, 0, data.Length);
		}
	}
}
