using System.Runtime.InteropServices;

namespace dist {
    class Struct {
        [StructLayout(LayoutKind.Sequential)]
        public struct MOUSE_IO {
            public byte Button;
            public byte X;
            public byte Y;
            public byte Wheel;
            public byte Unk1;
        }
    }
}
