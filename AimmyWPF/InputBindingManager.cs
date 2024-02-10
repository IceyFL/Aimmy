using Gma.System.MouseKeyHook;
using System;
using System.Windows.Forms;

public class InputBindingManager
{
    private IKeyboardMouseEvents _mEvents;
    private bool isSettingBinding = false;

    public string CurrentBinding { get; private set; }

    public event Action<string> OnBindingSet;

    public event Action<string> OnBindingPressed;

    public event Action<string> OnBindingReleased;

    public void Setup()
    {
        _mEvents = Hook.GlobalEvents();
        _mEvents.KeyDown += GlobalHookKeyDown;
        _mEvents.MouseDown += GlobalHookMouseDown;
        _mEvents.KeyUp += GlobalHookKeyUp;
        _mEvents.MouseUp += GlobalHookMouseUp;
    }

    private void GlobalHookKeyDown(object sender, KeyEventArgs e)
    {
        OnBindingPressed?.Invoke(e.KeyCode.ToString());
    }

    private void GlobalHookMouseDown(object sender, MouseEventArgs e)
    {
        OnBindingPressed?.Invoke(e.Button.ToString());
    }

    private void GlobalHookKeyUp(object sender, KeyEventArgs e)
    {
        OnBindingReleased?.Invoke(e.KeyCode.ToString());
    }

    private void GlobalHookMouseUp(object sender, MouseEventArgs e)
    {
        OnBindingReleased?.Invoke(e.Button.ToString());
    }

    public void StopListening()
    {
        if (_mEvents == null) return;

        _mEvents.KeyDown -= GlobalHookKeyDown;
        _mEvents.MouseDown -= GlobalHookMouseDown;
        _mEvents.KeyUp -= GlobalHookKeyUp;
        _mEvents.MouseUp -= GlobalHookMouseUp;
        _mEvents.Dispose();
        _mEvents = null;
    }
}