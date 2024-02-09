using System;
using System.Runtime.InteropServices;
using dist;

namespace CVE {
    class Mouse {
        static IntPtr Input = IntPtr.Zero;
        static Properties.IO_STATUS_BLOCK io = new Properties.IO_STATUS_BLOCK();

        const int FILE_SYNCHRONOUS_IO_NONALERT = 0x00000020;
        const int FILE_NON_DIRECTORY_FILE = 0x00000040;
        const int FILE_ATTRIBUTE_NORMAL = 0x00000080;
        const int SYNCHRONIZE = 0x00100000;
        const int GENERIC_WRITE = 0x40000000;

        public static int Initialize(string name) {
            Properties.OBJECT_ATTRIBUTES Attributes = new Properties.OBJECT_ATTRIBUTES(name, 0);
            int Status = WinAPI.NtCreateFile(out Input, GENERIC_WRITE | SYNCHRONIZE, ref Attributes, ref io, IntPtr.Zero, FILE_ATTRIBUTE_NORMAL, 0, 3, FILE_NON_DIRECTORY_FILE | FILE_SYNCHRONOUS_IO_NONALERT, IntPtr.Zero, 0);
            Attributes.Dispose();
            return Status;
        }

        public static bool Open() {
            int Status = 0;

            if (Input == IntPtr.Zero) {
                for (int num = 9; num >= 0; num--) {
                    Status = Initialize("\\??\\ROOT#SYSTEM#000" + num + "#{1abc05c0-c378-41b9-9cef-df1aba82b015}");
                    if (Status >= 0) break;
                }
            }
            return Status == 0;
        }

        public static void Close() {
            if (Input != IntPtr.Zero) {
                WinAPI.ZwClose(Input);
                Input = IntPtr.Zero;
            }
        }

        public static bool Call(Struct.MOUSE_IO buffer) {
            Properties.IO_STATUS_BLOCK block = new Properties.IO_STATUS_BLOCK();
            return 0 == WinAPI.NtDeviceIoControlFile(Input, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero, ref block, 0x2a2010, ref buffer, Marshal.SizeOf(typeof(Struct.MOUSE_IO)), IntPtr.Zero, 0);
        }

        public static void Move(int button, int x, int y, int wheel) {
            Struct.MOUSE_IO io;
            io.Unk1 = 0;
            io.Button = (byte)button;
            io.X = (byte)x;
            io.Y = (byte)y;
            io.Wheel = (byte)wheel;

            if (!Call(io)) {
                Close();
                Open();
            }
        }
    }
}
