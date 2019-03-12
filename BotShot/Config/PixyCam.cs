
using System;
using Microsoft.SPOT;

namespace BotShot.Config
{
    public class PixyCam {
        static System.IO.Ports.SerialPort uart;
        static bool connected = true;
        static byte[] _rx = new byte[1];
        static byte[] _tx = new byte[1];
        static int _txCnt = 0;




        public PixyCam(string port)
        {
            uart = new System.IO.Ports.SerialPort(port, 19200);
            uart.Open();
        }

        public byte UartReadByte()
        {
            if (uart.BytesToRead > 0)
            {
                int readCnt = uart.Read(_rx, 0, CalcRemainingCap());
            }
            System.Threading.Thread.Sleep(10);
            Debug.Print("Output:  ");
            Debug.Print(_rx[5].ToString());
            return _rx[0];
        }

        public byte UartReadWord()
        {
            if (uart.BytesToRead > 0)
            {
                int readCnt = uart.Read(_rx, 0, CalcRemainingCap());
            }
            System.Threading.Thread.Sleep(10);
            Debug.Print("Output:  ");
            Debug.Print(_rx[5].ToString());
            return _rx[0];
        }

        private int CalcRemainingCap()
        {
            /* firs calc the remaining capacity in the ring buffer */
            int rem = _tx.Length - _txCnt;
            /* cap the return to the maximum capacity of the rx array */
            if (rem > _rx.Length)
                rem = _rx.Length;
            return rem;
        }
        
        public int getStart()
        {
            UInt16 w, lastw;

            lastw = 0xffff;

            while(true)
            {
                w = UartRead();
                if (w == 0 && lastw == 0)
                    return 0;
                else if (w==0xaa55 && lastw==0xaa55)
                {
                    g_blockType = NORMAL_BLOCK;
                    return 1;
                }
                else if (w==0xaa56 && lastw--0xaa55)
                {
                    return 1;
                }
                else if (w==0x55aa)
                {

                }
                }

            }
        }
                 
    }   
}
