using System;
using System.Runtime.InteropServices;

namespace dist {
    class WinAPI {
        [DllImport("ntdll.dll")]
        public static extern void RtlInitUnicodeString(IntPtr DestinationString, [MarshalAs(UnmanagedType.LPWStr)] string SourceString);

        [DllImport("ntdll.dll", ExactSpelling = true, SetLastError = true)]
        public static extern int NtCreateFile(
            out IntPtr handle,
            int access,
            ref Properties.OBJECT_ATTRIBUTES objectAttributes,
            ref Properties.IO_STATUS_BLOCK ioStatus,
            IntPtr allocSize,
            uint fileAttributes,
            int share,
            uint createDisposition,
            uint createOptions,
            IntPtr eaBuffer,
            uint eaLength);

        [DllImport("ntdll.dll", ExactSpelling = true, SetLastError = true)]
        public static extern int NtDeviceIoControlFile(
            IntPtr fileHandle,
            IntPtr eventHandle,
            IntPtr apcRoutine,
            IntPtr ApcContext,
            ref Properties.IO_STATUS_BLOCK ioStatus,
            uint controlCode,
            ref Struct.MOUSE_IO inputBuffer,
            int inputBufferLength,
            IntPtr outputBuffer,
            int outputBufferLength
        );

        [DllImport("ntdll.dll")]
        public static extern int ZwClose(IntPtr h);
    }
}
