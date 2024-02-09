using System;
using System.Runtime.InteropServices;

namespace dist {
    class Properties {
        [StructLayout(LayoutKind.Sequential, Pack = 0)]
        public struct OBJECT_ATTRIBUTES : IDisposable {
            public int Length;
            public IntPtr RootDirectory;
            private IntPtr objectName;
            public uint Attributes;
            public IntPtr SecurityDescriptor;
            public IntPtr SecurityQualityOfService;

            public OBJECT_ATTRIBUTES(string name, uint attributes) {
                Length = 0;
                RootDirectory = IntPtr.Zero;
                objectName = IntPtr.Zero;
                Attributes = attributes;
                SecurityDescriptor = IntPtr.Zero;
                SecurityQualityOfService = IntPtr.Zero;

                Length = Marshal.SizeOf(this);
                objectName = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Properties.UNICODE_STRING)));
                WinAPI.RtlInitUnicodeString(objectName, name);
            }

            public void Dispose() {
                if (objectName != IntPtr.Zero) {
                    Marshal.FreeHGlobal(objectName);
                    objectName = IntPtr.Zero;
                }
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 0)]
        public struct UNICODE_STRING {
            public ushort Length;
            public ushort MaximumLength;
            public IntPtr Buffer;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 0)]
        public struct IO_STATUS_BLOCK {
            public uint Status;
            public IntPtr Information;
        }
    }
}
