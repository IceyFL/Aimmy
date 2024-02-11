using AimmyAimbot;
using AimmyWPF.Class;
using AimmyWPF.UserController;
using Class;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Visualization;
using static AimmyWPF.PredictionManager;

namespace AimmyWPF
{
    public partial class MainWindow : Window
    {
        private PredictionManager predictionManager;
        private OverlayWindow FOVOverlay;
        private PlayerDetectionWindow DetectedPlayerOverlay;
        private FileSystemWatcher fileWatcher;
        private FileSystemWatcher ConfigfileWatcher;

        private string lastLoadedModel = "N/A";
        private string lastLoadedConfig = "N/A";

        private readonly BrushConverter brushcolor = new();

        private int TimeSinceLastClick = 0;
        private DateTime LastClickTime = DateTime.MinValue;

        private const uint MOUSEEVENTF_LEFTDOWN = 0x0002;
        private const uint MOUSEEVENTF_LEFTUP = 0x0004;
        private const uint MOUSEEVENTF_MOVE = 0x0001; // Movement flag

        private static int ScreenWidth = System.Windows.Forms.Screen.PrimaryScreen.Bounds.Width;
        private static int ScreenHeight = System.Windows.Forms.Screen.PrimaryScreen.Bounds.Height;

        private AIModel _onnxModel;
        private InputBindingManager bindingManager;
        public bool Key1Change = false;
        public bool Key2Change = false;
        public bool RecoilKeyChange = false;
        public bool RecoilToggleKeyChange = false;
        public bool ToggleKeyChange = false;
        string key1 = "Right";
        string key2 = "Left";
        string RecoilKey = "Left";
        string RecoilToggleKey = "P";
        string ToggleKey = "O";
        private CancellationTokenSource cts;

        private enum MenuPosition
        {
            AimMenu,
            MiscMenu,
            SelectorMenu,
            SettingsMenu
        }

        private DateTime lastExecutionTime = DateTime.MinValue;



        public List<string> KeyDown = new List<string> { };

        // Changed to Dynamic from Double because it was making the Config System hard to rework :/
        public Dictionary<string, dynamic> aimmySettings = new()
        {
            { "Suggested_Model", ""},
            { "FOV_Size", 640 },
            { "Mouse_Sens", 0.80 },
            { "Mouse_SensY", 0.90 },
            { "Mouse_Jitter", 4 },
            { "Y_Offset", 0 },
            { "X_Offset", 0 },
            { "Trigger_Delay", 0.1 },
            { "AI_Min_Conf", 50 },
            { "AimMethod", "Closest Detection" },
            { "RecoilStrength", 10 },
            { "RecoilDelay", 200 },
            { "RecoilActivationDelay", 10 },
            { "AIFPS", 40 },
            { "MouseMoveMethod", "Mouse Event" }
        };

        private Dictionary<string, bool> toggleState = new()
        {
            { "AimbotToggle", false },
            { "AlwaysOn", false },
            { "PredictionToggle", false },
            { "AimViewToggle", false },
            { "TriggerBot", false },
            { "CollectData", false },
            { "TopMost", false }
        };

        // PDW == PlayerDetectionWindow
        public Dictionary<string, dynamic> OverlayProperties = new()
        {
            { "FOV_Color", "#712fc6"},
            { "PDW_Size", 50 },
            { "PDW_CornerRadius", 0 },
            { "PDW_BorderThickness", 1 },
            { "PDW_Opacity", 1 }
        };

        //setup needed buttons and toggles
        AKeyChanger RecoilKeyChanger;
        AKeyChanger RecoilToggleKeyChanger;
        AKeyChanger Change_KeyPress;
        AKeyChanger Change_KeyPress2;
        AKeyChanger ToggleKeyChanger;
        AToggle RecoilState;
        AToggle Enable_AIAimAligner;

        private Thickness WinTooLeft = new(-1680, 0, 1680, 0);
        private Thickness WinVeryLeft = new(-1120, 0, 1120, 0);
        private Thickness WinLeft = new(-560, 0, 560, 0);

        private Thickness WinCenter = new(0, 0, 0, 0);

        private Thickness WinRight = new(560, 0, -560, 0);
        private Thickness WinVeryRight = new(1120, 0, -1120, 0);
        private Thickness WinTooRight = new(1680, 0, -1680, 0);

        public MainWindow()
        {
            InitializeComponent();
            MainBorder.MouseMove += MainBorder_MouseMove;
            this.Title = Path.GetFileNameWithoutExtension(Assembly.GetExecutingAssembly().Location);

            // Check to see if certain items are installed
            RequirementsManager RM = new();
            if (!RM.IsVCRedistInstalled())
            {
                System.Windows.Forms.MessageBox.Show("Visual C++ Redistributables x64 are not installed on this device, please install them before using Aimmy to avoid issues.", "Load Error");
                Process.Start("https://aka.ms/vs/17/release/vc_redist.x64.exe");
                Application.Current.Shutdown();
            }
            //if(!RM.IsDotNetInstalled()) not working
            //{
            //    MessageBox.Show("DotNet 7.0 is not installed on this device, please install them before using Aimmy to avoid issues.", "Load Error");
            //    Process.Start("https://dotnet.microsoft.com/download/dotnet/7.0");
            //    Application.Current.Shutdown();
            //}

            // Check for required folders
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            string[] dirs = { "bin", "bin/models", "bin/images", "bin/configs", "bin/labels" };

            try
            {
                foreach (string dir in dirs)
                {
                    string fullPath = Path.Combine(baseDir, dir);
                    if (!Directory.Exists(fullPath))
                    {
                        // Create the directory
                        Directory.CreateDirectory(fullPath);
                    }
                }
            }
            catch (Exception ex)
            {
                System.Windows.Forms.MessageBox.Show($"Error creating a required directory: {ex}");
                Application.Current.Shutdown(); // We don't want to continue running without that folder.
            }


            // Setup key/mouse hook
            bindingManager = new InputBindingManager();
            bindingManager.Setup();
            bindingManager.OnBindingPressed += (binding) =>
            {
                if ((KeyDown.Contains(binding) == false))
                {
                    KeyDown.Add(binding);
                    if (binding == RecoilToggleKey){ ToggleRecoil(); }
                    else if (binding == ToggleKey) { ToggleAim(); }
                }
                if (Key1Change) { key1 = binding; Key1Change = false; Change_KeyPress.KeyNotifier.Content = binding; Change_Keysize(Change_KeyPress, binding); }
                else if (Key2Change) { key2 = binding; Key2Change = false; Change_KeyPress2.KeyNotifier.Content = binding; Change_Keysize(Change_KeyPress2, binding); }
                else if (RecoilKeyChange) { RecoilKey = binding; RecoilKeyChange = false; RecoilKeyChanger.KeyNotifier.Content = binding; Change_Keysize(RecoilKeyChanger, binding); }
                else if (RecoilToggleKeyChange) { RecoilToggleKey = binding; RecoilToggleKeyChange = false; RecoilToggleKeyChanger.KeyNotifier.Content = binding; Change_Keysize(RecoilToggleKeyChanger, binding); }
                else if (ToggleKeyChange) { ToggleKey = binding; ToggleKeyChange = false; ToggleKeyChanger.KeyNotifier.Content = binding; Change_Keysize(ToggleKeyChanger, binding); }
            };
            bindingManager.OnBindingReleased += (binding) => { try { KeyDown.Remove(binding); } catch {} };

            // Load UI
            InitializeMenuPositions();
            //LoadAimMenu();
            //LoadTriggerMenu();
            //LoadSettingsMenu();
            ReloadMenu();
            InitializeFileWatcher();
            InitializeConfigWatcher();

            // Load PredictionManager
            predictionManager = new PredictionManager();

            // Load all models into listbox
            LoadModelsIntoListBox();
            LoadConfigsIntoListBox();

            SelectorListBox.SelectionChanged += new SelectionChangedEventHandler(SelectorListBox_SelectionChanged);
            ConfigSelectorListBox.SelectionChanged += new SelectionChangedEventHandler(ConfigSelectorListBox_SelectionChanged);

            // Create FOV Overlay
            FOVOverlay = new OverlayWindow();
            FOVOverlay.Hide();
            FOVOverlay.FovSize = (int)aimmySettings["FOV_Size"];
            AwfulPropertyChanger.PostNewFOVSize();
            AwfulPropertyChanger.PostTravellingFOV(false);

            // Create Current Detected Player Overlay
            DetectedPlayerOverlay = new PlayerDetectionWindow();
            DetectedPlayerOverlay.Hide();

            // Start the loop that runs the model
            Task.Run(() => StartModelCaptureLoop());
            Task.Run(() => RecoilLoop());
        }

        private HashSet<string> AvailableModels = new();
        private HashSet<string> AvailableConfigs = new();

        private void Change_Keysize(AKeyChanger Current, string Key) 
        {
            int Width = 20;
            int bindingwidth = Key.Length * 6;
            Width = Width + bindingwidth;
            Current.Box.Width = Width;
        }

        private void ToggleRecoil() 
        {
            RecoilState.Reader.RaiseEvent(new RoutedEventArgs(Button.ClickEvent));
        }

        private void ToggleAim()
        {
            Enable_AIAimAligner.Reader.RaiseEvent(new RoutedEventArgs(Button.ClickEvent));
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            await LoadOverlayPropertiesAsync("bin/Overlay.cfg");
            await LoadConfigAsync("bin/configs/Default.cfg");
            try
            {
                Task modelTask = RetrieveAndAddFilesAsync("models", "bin\\models", AvailableModels);
                Task configTask = RetrieveAndAddFilesAsync("configs", "bin\\configs", AvailableConfigs);
                await Task.WhenAll(modelTask, configTask);
                LoadStoreMenu();
            }
            catch
            {
                System.Windows.Forms.MessageBox.Show("Github is irretrieveable right now, the Downloadable Model menu will not work right now, sorry!");
            }
        }

        private async Task RetrieveAndAddFilesAsync(string repositoryPath, string localPath, HashSet<string> availableFiles)
        {
            IEnumerable<string> results = await RetrieveGithubFiles.ListContents(repositoryPath);

            foreach (var file in results)
            {
                string filePath = Path.Combine(localPath, file);
                if (!availableFiles.Contains(file) && !File.Exists(filePath))
                {
                    availableFiles.Add(file);
                }
            }
        }

        #region Mouse Movement / Clicking Handler

        [DllImport("user32.dll")]
        private static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, int dwExtraInfo);

        private static Random MouseRandom = new();

        private static Point CubicBezier(Point start, Point end, Point control1, Point control2, double t)
        {
            double u = 1 - t;
            double tt = t * t;
            double uu = u * u;
            double uuu = uu * u;
            double ttt = tt * t;

            double x = uuu * start.X + 3 * uu * t * control1.X + 3 * u * tt * control2.X + ttt * end.X;
            double y = uuu * start.Y + 3 * uu * t * control1.Y + 3 * u * tt * control2.Y + ttt * end.Y;

            return new Point((int)x, (int)y);
        }

        private async Task DoTriggerClick()
        {
            int TimeSinceLastClick = (int)(DateTime.Now - LastClickTime).TotalMilliseconds;
            int Trigger_Delay_Milliseconds = (int)(aimmySettings["Trigger_Delay"] * 1000);

            if (TimeSinceLastClick >= Trigger_Delay_Milliseconds || LastClickTime == DateTime.MinValue)
            {
                mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);
                await Task.Delay(20);
                mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                LastClickTime = DateTime.Now;
            }

            return;
        }

        private void MoveCrosshair(int detectedX, int detectedY)
        {
            double Alpha = aimmySettings["Mouse_Sens"];
            double Beta = aimmySettings["Mouse_SensY"];

            int halfScreenWidth = ScreenWidth / 2;
            int halfScreenHeight = ScreenHeight / 2;

            int targetX = detectedX - halfScreenWidth;
            int targetY = detectedY - halfScreenHeight;

            // Aspect ratio correction factor
            double aspectRatioCorrection = (double)ScreenWidth / ScreenHeight;
            targetY = (int)(targetY * aspectRatioCorrection);

            // Introduce random jitter
            int MouseJitter = (int)aimmySettings["Mouse_Jitter"];
            int jitterX = MouseRandom.Next(-MouseJitter, MouseJitter);
            int jitterY = MouseRandom.Next(-MouseJitter, MouseJitter);

            targetX += jitterX;
            targetY += jitterY;

            // Define Bezier curve control points
            Point start = new(0, 0); // Current cursor position (locked to center screen)
            Point end = new(targetX, targetY);
            Point control1 = new Point(start.X + (end.X - start.X) / 3, 0);
            Point control2 = new Point(start.X + 2 * (end.X - start.X) / 3, 0);
            Point control3 = new Point(0, start.Y + (end.Y - start.Y) / 3);
            Point control4 = new Point(0, start.Y + 2 * (end.Y - start.Y) / 3);

            // Calculate new position along the Bezier curve
            Point newPosition = CubicBezier(start, end, control1, control2, 1 - Alpha);
            Point newPosition2 = CubicBezier(start, end, control3, control4, 1 - Beta);

            mouseMove((int)newPosition.X, (int)newPosition2.Y);

            if (toggleState["TriggerBot"])
            {
                Task.Run(DoTriggerClick);
            }
        }

        #endregion Mouse Movement / Clicking Handler

        #region Aim Aligner Main and Loop

        public async Task ModelCapture(bool TriggerOnly = false)
        {
            int AimMethod = 0;
            if (aimmySettings["AimMethod"].Contains("Highest Confidence Detection")) { AimMethod = 1; }
            var closestPrediction = await _onnxModel.GetClosestPredictionToCenterAsync((int)AimMethod);
            if (closestPrediction == null)
            {
                return;
            }
            else if (TriggerOnly)
            {
                Task.Run(DoTriggerClick);
                return;
            }

            float scaleX = (float)ScreenWidth / 640f;
            float scaleY = (float)ScreenHeight / 640f;

            double YOffset = aimmySettings["Y_Offset"];
            double XOffset = aimmySettings["X_Offset"];
            int detectedX = (int)((closestPrediction.Rectangle.X + closestPrediction.Rectangle.Width / 2) * scaleX + XOffset);
            int detectedY = (int)((closestPrediction.Rectangle.Y + closestPrediction.Rectangle.Height / 2) * scaleY + YOffset);

            Console.WriteLine(AIModel.AIConfidence.ToString());

            bool IsHolding_Binding = (KeyDown.Contains(key1) || (Bools.SecondKey && KeyDown.Contains(key2)));
            // Handle Prediction
            if (toggleState["PredictionToggle"])
            {
                Detection detection = new()
                {
                    X = detectedX,
                    Y = detectedY,
                    Timestamp = DateTime.UtcNow
                };

                predictionManager.UpdateKalmanFilter(detection);
                var predictedPosition = predictionManager.GetEstimatedPosition();

                if ((Bools.AimOnlyWhenBindingHeld && IsHolding_Binding) || Bools.AimOnlyWhenBindingHeld == false)
                    MoveCrosshair(predictedPosition.X, predictedPosition.Y);
                //MoveCrosshair(predictedPosition.X, predictedPosition.Y);

                if (Bools.ShowDetectedPlayerWindow && Bools.ShowPrediction)
                {
                    this.Dispatcher.Invoke(() =>
                    {
                        DetectedPlayerOverlay.PredictionFocus.Margin = new Thickness(
                            predictedPosition.X - (OverlayProperties["PDW_Size"] / (OverlayProperties["PDW_Size"] / 2)),
                            predictedPosition.Y - (OverlayProperties["PDW_Size"] / (OverlayProperties["PDW_Size"] / 2)),
                            0, 0);
                    });
                }
            }
            else
            {
                if ((Bools.AimOnlyWhenBindingHeld && IsHolding_Binding) || Bools.AimOnlyWhenBindingHeld == false)
                    MoveCrosshair(detectedX, detectedY);
            }

            if (Bools.ShowDetectedPlayerWindow)
            {
                this.Dispatcher.Invoke(() =>
                {
                    if (Bools.ShowCurrentDetectedPlayer)
                        DetectedPlayerOverlay.DetectedPlayerFocus.Margin = new Thickness(
                            detectedX - (OverlayProperties["PDW_Size"] / (OverlayProperties["PDW_Size"] / 2)),
                            detectedY - (OverlayProperties["PDW_Size"] / (OverlayProperties["PDW_Size"] / 2)),
                            0, 0);

                    if (Bools.ShowUnfilteredDetectedPlayer)
                        DetectedPlayerOverlay.UnfilteredPlayerFocus.Margin = new Thickness(
                            (int)((closestPrediction.Rectangle.X + closestPrediction.Rectangle.Width / 2) * scaleX) - (OverlayProperties["PDW_Size"] / (OverlayProperties["PDW_Size"] / 2)),
                            (int)((closestPrediction.Rectangle.Y + closestPrediction.Rectangle.Height / 2) * scaleY) - (OverlayProperties["PDW_Size"] / (OverlayProperties["PDW_Size"] / 2)),
                            0, 0);
                });
            }
        }

        private async Task StartModelCaptureLoop()
        {
            // Create a new CancellationTokenSource
            cts = new CancellationTokenSource();

            while (!cts.Token.IsCancellationRequested)
            {
                bool IsHolding_Binding = (KeyDown.Contains(key1) || (Bools.SecondKey && KeyDown.Contains(key2)));
                TimeSpan elapsed = DateTime.Now - lastExecutionTime;
                if (elapsed.TotalMilliseconds > 1000.0 / aimmySettings["AIFPS"])
                {
                    lastExecutionTime = DateTime.Now;
                    if (toggleState["AimbotToggle"] && Bools.ConstantTracking==false && IsHolding_Binding)
                    {
                        await ModelCapture();
                    }
                    else if (toggleState["AimbotToggle"] && Bools.ConstantTracking)
                    {
                        await ModelCapture();
                    }
                    else if (!toggleState["AimbotToggle"] && toggleState["TriggerBot"] && IsHolding_Binding && Bools.ConstantTracking == false) // Triggerbot Only
                    {
                        await ModelCapture(true);
                    }
                }

                //if (toggleState["AimbotToggle"] && (IsHolding_Binding || toggleState["AlwaysOn"]))
                //{
                //    await ModelCapture();
                //}
                //else if (!toggleState["AimbotToggle"] && toggleState["TriggerBot"] && IsHolding_Binding) // Triggerbot Only
                //{
                //    await ModelCapture(true);
                //}

                // We have to have some sort of delay here to not overload the CPU / reduce CPU usage.
                await Task.Delay(1);
            }
        }

        private async Task mouseMove(int x, int y) 
        {
            if (aimmySettings["MouseMoveMethod"].Contains("Mouse Event"))
            {
                mouse_event(MOUSEEVENTF_MOVE, (uint)x, (uint)y, 0, 0);
            }
            else if (aimmySettings["MouseMoveMethod"].Contains("Logitech Mouse Driver"))
            {
                CVE.Mouse.Move(0, x, y, 0);
            }
        }

        private async Task RecoilLoop()
        {
            cts = new CancellationTokenSource();

            while (!cts.Token.IsCancellationRequested)
            {
                bool RecoilKeyHeld = (KeyDown.Contains(RecoilKey));
                if (Bools.Recoil && RecoilKeyHeld)
                {
                    while (RecoilKeyHeld)
                    {
                        await mouseMove(0, (int)aimmySettings["RecoilStrength"]);
                        await Task.Delay((int)aimmySettings["RecoilDelay"]);
                        RecoilKeyHeld = (KeyDown.Contains(RecoilKey));
                    }
                }
                await System.Windows.Application.Current.Dispatcher.InvokeAsync(() =>
                {
                    Random random = new Random();
                    int randomNumber = random.Next(1, 10000);
                    string title = randomNumber.ToString("D4");
                    this.Title = title;
                });
                await Task.Delay((int)aimmySettings["RecoilActivationDelay"]);
            }
        }

        public void StopModelCaptureLoop()
        {
            if (cts != null)
            {
                cts?.Cancel();
                cts = null;
            }
        }

        #endregion Aim Aligner Main and Loop

        #region Menu Initialization and Setup

        private void InitializeMenuPositions()
        {
            AimMenu.Margin = new Thickness(0, 0, 0, 0);
            MiscMenu.Margin = new Thickness(560, 0, -560, 0);
            SelectorMenu.Margin = new Thickness(560, 0, -560, 0);
            SettingsMenu.Margin = new Thickness(1680, 0, -1680, 0);
        }

        private void SetupToggle(AToggle toggle, Action<bool> action, bool initialState)
        {
            toggle.Reader.Tag = initialState;
            if (initialState == true)
            {
                toggle.ToggleCheckBox.IsChecked = true;
            }

            toggle.Reader.Click += (s, x) =>
            {
                bool currentState = (bool)toggle.Reader.Tag;
                toggle.Reader.Tag = !currentState;
                action.Invoke(!currentState);
                SetToggleState(toggle);
            };
        }

        private void MainBorder_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            UpdateGradientDirection(e.GetPosition(MainBorder));
        }

        private void UpdateGradientDirection(Point mousePosition)
        {
            double xPercentage = mousePosition.X / MainBorder.ActualWidth;
            double yPercentage = mousePosition.Y / MainBorder.ActualHeight;

            // Calculate the angle of the gradient based on the mouse position
            double angle = Math.Atan2(yPercentage - 0.5, xPercentage - 0.5) * (180 / Math.PI);

            GradientRotation.Angle = angle;
        }
        private void SetToggleState(AToggle toggle)
        {
            bool state = (bool)toggle.Reader.Tag;

            // Stop them from turning on anything until the model has been selected.
            if ((toggle.Reader.Name == "AimbotToggle" || toggle.Reader.Name == "AimOnlyWhenBindingHeld" || toggle.Reader.Name == "ConstantAITracking" || toggle.Reader.Name == "TriggerBot" || toggle.Reader.Name == "CollectData") && lastLoadedModel == "N/A")
            {
                SetToggleStatesOnModelNotSelected();
                System.Windows.Forms.MessageBox.Show("Please select a model in the Model Selector before toggling.", "Toggle Error");
                return;
            }

            toggleState[toggle.Reader.Name] = state;

            HandleToggleSpecificActions(toggle);
        }

        private void SetToggleStatesOnModelNotSelected()
        {
            Bools.AIAimAligner = false;
            Bools.Triggerbot = false;
            Bools.CollectDataWhilePlaying = false;
        }

        private void HandleToggleSpecificActions(AToggle toggle)
        {
            string toggleName = toggle.Reader.Name;

            switch (toggleName)
            {
                case "CollectData":
                    _onnxModel.CollectData = (bool)toggle.Reader.Tag;
                    break;

                case "ShowFOV":
                    ((bool)toggle.Reader.Tag ? (Action)FOVOverlay.Show : (Action)FOVOverlay.Hide)();
                    break;

                case "TravellingFOV":
                    AwfulPropertyChanger.PostTravellingFOV((bool)toggle.Reader.Tag);
                    break;

                case "ShowDetectedPlayerWindow":
                    ((bool)toggle.Reader.Tag ? (Action)DetectedPlayerOverlay.Show : (Action)DetectedPlayerOverlay.Hide)();
                    break;

                case "ShowCurrentDetectedPlayer":
                    ((bool)toggle.Reader.Tag ? (Action)(() => DetectedPlayerOverlay.DetectedPlayerFocus.Visibility = Visibility.Visible) : () => DetectedPlayerOverlay.DetectedPlayerFocus.Visibility = Visibility.Collapsed)();
                    break;

                case "ShowUnfilteredDetectedPlayer":
                    ((bool)toggle.Reader.Tag ? (Action)(() => DetectedPlayerOverlay.UnfilteredPlayerFocus.Visibility = Visibility.Visible) : () => DetectedPlayerOverlay.UnfilteredPlayerFocus.Visibility = Visibility.Collapsed)();
                    break;

                case "ShowAIPrediction":
                    ((bool)toggle.Reader.Tag ? (Action)(() => DetectedPlayerOverlay.PredictionFocus.Visibility = Visibility.Visible) : () => DetectedPlayerOverlay.PredictionFocus.Visibility = Visibility.Collapsed)();
                    break;

                case "TopMost":
                    Topmost = (bool)toggle.Reader.Tag;
                    break;
            }
        }

        #endregion Menu Initialization and Setup

        #region Menu Controls

        private async void Selection_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button clickedButton)
            {
                MenuPosition position = (MenuPosition)Enum.Parse(typeof(MenuPosition), clickedButton.Tag.ToString());

                ApplyMenuAnimations(position);
                UpdateMenuVisibility(position);

                Thickness LeftMargin;
                Thickness RightMargin;
                switch (clickedButton.Name)
                {
                    case "Selection1":
                        LeftMargin = new Thickness(25, 25, -25, 335);
                        RightMargin = new Thickness(-25, 25, 25, 335);
                        break;
                    case "Selection2":
                        LeftMargin = new Thickness(25, 90, -25, 300);
                        RightMargin = new Thickness(-25, 90, 25, 300);
                        break;
                    case "Selection3":
                        LeftMargin = new Thickness(25, 130, -25, 237);
                        RightMargin = new Thickness(-25, 130, 25, 237);
                        break;
                    default:
                        LeftMargin = new Thickness(25, 175, -25, 180);
                        RightMargin = new Thickness(-25, 175, 25, 180);
                        break;
                }
                LeftDot.Margin = LeftMargin;
                RightDot.Margin = RightMargin;
            }
        }

        private void ApplyMenuAnimations(MenuPosition position)
        {
            Thickness highlighterMargin = new(0, 30, 414, 0);
            switch (position)
            {
                case MenuPosition.AimMenu:
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), AimMenu, AimMenu.Margin, WinCenter);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), MiscMenu, MiscMenu.Margin, WinRight);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SelectorMenu, SelectorMenu.Margin, WinVeryRight);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SettingsMenu, SettingsMenu.Margin, WinTooRight);
                    break;

                case MenuPosition.MiscMenu:
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), AimMenu, AimMenu.Margin, WinLeft);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), MiscMenu, MiscMenu.Margin, WinCenter);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SelectorMenu, SelectorMenu.Margin, WinRight);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SettingsMenu, SettingsMenu.Margin, WinVeryRight);
                    break;

                case MenuPosition.SelectorMenu:
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), AimMenu, AimMenu.Margin, WinVeryLeft);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), MiscMenu, MiscMenu.Margin, WinLeft);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SelectorMenu, SelectorMenu.Margin, WinCenter);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SettingsMenu, SettingsMenu.Margin, WinRight);
                    break;

                case MenuPosition.SettingsMenu:
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), AimMenu, AimMenu.Margin, WinTooLeft);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), MiscMenu, MiscMenu.Margin, WinVeryLeft);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SelectorMenu, SelectorMenu.Margin, WinLeft);
                    Animator.ObjectShift(TimeSpan.FromMilliseconds(500), SettingsMenu, SettingsMenu.Margin, WinCenter);
                    break;
            }
        }

        private void UpdateMenuVisibility(MenuPosition position)
        {
            AimMenu.Visibility = (position == MenuPosition.AimMenu) ? Visibility.Visible : Visibility.Collapsed;
            MiscMenu.Visibility = (position == MenuPosition.MiscMenu) ? Visibility.Visible : Visibility.Collapsed;
            SelectorMenu.Visibility = (position == MenuPosition.SelectorMenu) ? Visibility.Visible : Visibility.Collapsed;
            SettingsMenu.Visibility = (position == MenuPosition.SettingsMenu) ? Visibility.Visible : Visibility.Collapsed;
        }

        #endregion Menu Controls

        #region More Info Function

        public void ActivateMoreInfo(string info)
        {
            SetMenuState(false);
            Animator.ObjectShift(TimeSpan.FromMilliseconds(1000), MoreInfoBox, MoreInfoBox.Margin, new Thickness(5, 0, 5, 5));
            MoreInfoBox.Visibility = Visibility.Visible;
            InfoText.Text = info;
        }

        private async void MoreInfoExit_Click(object sender, RoutedEventArgs e)
        {
            Animator.ObjectShift(TimeSpan.FromMilliseconds(1000), MoreInfoBox, MoreInfoBox.Margin, new Thickness(5, 0, 5, -180));
            await Task.Delay(1000);
            MoreInfoBox.Visibility = Visibility.Collapsed;
            SetMenuState(true);
        }

        private void SetMenuState(bool state)
        {
            AimMenu.IsEnabled = state;
            MiscMenu.IsEnabled = state;
            SelectorMenu.IsEnabled = state;
            SettingsMenu.IsEnabled = state;
        }

        #endregion More Info Function
        private void LoadAimMenu()
        {
            SectionPanel leftPanel = new SectionPanel();
            SectionPanel rightPanel = new SectionPanel();

            // Add controls to the aimingConfigurationPanel instead of AimScroller
            leftPanel.Children.Add(new ALabel("Aim Assist"));


            Enable_AIAimAligner = new("Aim Assist");
            Enable_AIAimAligner.Reader.Name = "AimbotToggle";
            SetupToggle(Enable_AIAimAligner, state => Bools.AIAimAligner = state, Bools.AIAimAligner);
            leftPanel.Children.Add(Enable_AIAimAligner);

            AToggle Enable_ConstantAITracking = new("Constant AI Tracking");
            Enable_ConstantAITracking.Reader.Name = "ConstantAITracking";
            SetupToggle(Enable_ConstantAITracking, state => Bools.ConstantTracking = state, Bools.ConstantTracking);
            leftPanel.Children.Add(Enable_ConstantAITracking);

            ASlider AIMinimumConfidence = new("AI Minimum Confidence", 1, "confidence");

            AIMinimumConfidence.Slider.Minimum = 1;
            AIMinimumConfidence.Slider.Maximum = 100;
            AIMinimumConfidence.Slider.Value = aimmySettings["AI_Min_Conf"];
            AIMinimumConfidence.Slider.TickFrequency = 1;
            AIMinimumConfidence.Slider.ValueChanged += (s, x) =>
            {
                if (lastLoadedModel != "N/A")
                {
                    double ConfVal = ((double)AIMinimumConfidence.Slider.Value);
                    aimmySettings["AI_Min_Conf"] = ConfVal;
                    _onnxModel.ConfidenceThreshold = (float)(ConfVal / 100.0f);
                }
                else
                {
                    // Prevent double messageboxes..
                    if (AIMinimumConfidence.Slider.Value != aimmySettings["AI_Min_Conf"])
                    {
                        System.Windows.Forms.MessageBox.Show("Unable to set confidence, please select a model and try again.", "Slider Error");
                        AIMinimumConfidence.Slider.Value = aimmySettings["AI_Min_Conf"];
                    }
                }
            };

             leftPanel.Children.Add(AIMinimumConfidence);

            ASlider FPS = new("AI FPS", 1, "FPS");

            FPS.Slider.Minimum = 0;
            FPS.Slider.Maximum = 200;
            FPS.Slider.Value = aimmySettings["AIFPS"];
            FPS.Slider.TickFrequency = 1;
            FPS.Slider.ValueChanged += (s, x) =>
            {
                double method = FPS.Slider.Value;
                aimmySettings["AIFPS"] = method;
            };

            leftPanel.Children.Add(FPS);

            Change_KeyPress = new("Aim Keybind", "Right");
            Change_KeyPress.Reader.Click += (s, x) =>
            {
                Change_KeyPress.KeyNotifier.Content = "Listening..";
                Change_Keysize(Change_KeyPress, "Listening..");
                Key1Change = true;
            };

            leftPanel.Children.Add(Change_KeyPress);

            AToggle SecondKey = new("Enable Second Key");
            SecondKey.Reader.Name = "SecondKey";
            SetupToggle(SecondKey, state => Bools.SecondKey = state, Bools.SecondKey);

            leftPanel.Children.Add(SecondKey);


            Change_KeyPress2 = new("Second Aim Keybind", "Left");
            Change_KeyPress2.Reader.Click += (s, x) =>
            {
                Change_KeyPress2.KeyNotifier.Content = "Listening..";
                Change_Keysize(Change_KeyPress2, "Listening..");
                Key2Change = true;
            };

            leftPanel.Children.Add(Change_KeyPress2);

            //AToggle Enable_AlwaysOn = new AToggle(this, "Aim Align Always On",
            //   "This will keep the aim aligner on 24/7 so you don't have to hold a toggle.");
            //Enable_AlwaysOn.Reader.Name = "AlwaysOn";
            //SetupToggle(Enable_AlwaysOn, state => Bools.AIAlwaysOn = state, Bools.AIAlwaysOn);
            //AimScroller.Children.Add(Enable_AlwaysOn);

            AToggle Enable_AIPredictions = new("Predictions");
            Enable_AIPredictions.Reader.Name = "PredictionToggle";
            SetupToggle(Enable_AIPredictions, state => Bools.AIPredictions = state, Bools.AIPredictions);
            leftPanel.Children.Add(Enable_AIPredictions);

            ToggleKeyChanger = new("Aim Toggle Keybind", "O");
            ToggleKeyChanger.Reader.Click += (s, x) =>
            {
                ToggleKeyChanger.KeyNotifier.Content = "Listening..";
                Change_Keysize(ToggleKeyChanger, "Listening..");
                ToggleKeyChange = true;
            };

            leftPanel.Children.Add(ToggleKeyChanger);


            rightPanel.Children.Add(new ALabel("Aim Config"));

            List<string> MouseMoveMethod = new List<string>
            {
                { "Mouse Event" },
                { "Logitech Mouse Driver" }
            };

            ADropdown MouseMethod = new("Mouse Movement", MouseMoveMethod);

            MouseMethod.DropdownBox.SelectionChanged += (s, x) =>
            {
                aimmySettings["MouseMoveMethod"] = MouseMethod.DropdownBox.SelectedItem.ToString();
            };

            rightPanel.Children.Add(MouseMethod);

            List<string> MouseDrivers = new List<string>
            {
                { "Closest Detection" },
                { "Highest Confidence Detection" }
            };

            ADropdown AimMethod = new("Aim Method", MouseDrivers);

            AimMethod.DropdownBox.SelectionChanged += (s, x) =>
            {
                aimmySettings["AimMethod"] = AimMethod.DropdownBox.SelectedItem.ToString();
            };

            rightPanel.Children.Add(AimMethod);


            ASlider MouseSensitivtyX = new ASlider("Mouse Sensitivity X", 0.01, "Sens");

            MouseSensitivtyX.Slider.Minimum = 0.01;
            MouseSensitivtyX.Slider.Maximum = 1;
            MouseSensitivtyX.Slider.Value = aimmySettings["Mouse_Sens"];
            MouseSensitivtyX.Slider.TickFrequency = 0.01;
            MouseSensitivtyX.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Mouse_Sens"] = MouseSensitivtyX.Slider.Value;
            };

            rightPanel.Children.Add(MouseSensitivtyX);

            ASlider MouseSensitivtyY = new ASlider("Mouse Sensitivity Y", 0.01, "Sens");

            MouseSensitivtyY.Slider.Minimum = 0.01;
            MouseSensitivtyY.Slider.Maximum = 1;
            MouseSensitivtyY.Slider.Value = aimmySettings["Mouse_SensY"];
            MouseSensitivtyY.Slider.TickFrequency = 0.01;
            MouseSensitivtyY.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Mouse_SensY"] = MouseSensitivtyY.Slider.Value;
            };

            rightPanel.Children.Add(MouseSensitivtyY);

            ASlider MouseJitter = new("Mouse Jitter", 0.01, "Jitter");

            MouseJitter.Slider.Minimum = 0;
            MouseJitter.Slider.Maximum = 15;
            MouseJitter.Slider.Value = aimmySettings["Mouse_Jitter"];
            MouseJitter.Slider.TickFrequency = 1;
            MouseJitter.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Mouse_Jitter"] = MouseJitter.Slider.Value;
            };
            rightPanel.Children.Add(MouseJitter);

            ASlider YOffset = new("Y Offset (Up/Down)", 1, "Offset");

            YOffset.Slider.Minimum = -150;
            YOffset.Slider.Maximum = 150;
            YOffset.Slider.Value = aimmySettings["Y_Offset"];
            YOffset.Slider.TickFrequency = 1;
            YOffset.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Y_Offset"] = YOffset.Slider.Value;
            };

            rightPanel.Children.Add(YOffset);

            ASlider XOffset = new("X Offset (Left/Right)", 1, "Offset");

            XOffset.Slider.Minimum = -150;
            XOffset.Slider.Maximum = 150;
            XOffset.Slider.Value = aimmySettings["X_Offset"];
            XOffset.Slider.TickFrequency = 1;
            XOffset.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["X_Offset"] = XOffset.Slider.Value;
            };

            rightPanel.Children.Add(XOffset);

            SectionPanel leftPanel1 = new SectionPanel();

            leftPanel1.Children.Add(new ALabel("Auto Trigger"));

            AToggle Enable_TriggerBot = new("Enable Auto Trigger");
            Enable_TriggerBot.Reader.Name = "TriggerBot";
            SetupToggle(Enable_TriggerBot, state => Bools.Triggerbot = state, Bools.Triggerbot);
            leftPanel1.Children.Add(Enable_TriggerBot);

            ASlider TriggerBot_Delay = new("Auto Trigger Delay", 0.1, "ms");

            TriggerBot_Delay.Slider.Minimum = 0.01;
            TriggerBot_Delay.Slider.Maximum = 1;
            TriggerBot_Delay.Slider.Value = aimmySettings["Trigger_Delay"];
            TriggerBot_Delay.Slider.TickFrequency = 0.01;
            TriggerBot_Delay.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Trigger_Delay"] = TriggerBot_Delay.Slider.Value;
            };

            leftPanel1.Children.Add(TriggerBot_Delay);

            SectionPanel leftPanel2 = new SectionPanel();


            leftPanel2.Children.Add(new ALabel("Anti Recoil"));

            RecoilState = new("Anti Recoil");
            RecoilState.Reader.Name = "RecoilToggle";
            SetupToggle(RecoilState, state => Bools.Recoil = state, Bools.Recoil);
            leftPanel2.Children.Add(RecoilState);

            ASlider RecoilStrength = new("Anti Recoil Strength", 1, "Strength");

            RecoilStrength.Slider.Minimum = 1;
            RecoilStrength.Slider.Maximum = 50;
            RecoilStrength.Slider.Value = aimmySettings["RecoilStrength"];
            RecoilStrength.Slider.TickFrequency = 1;
            RecoilStrength.Slider.ValueChanged += (s, x) =>
            {
                double method = RecoilStrength.Slider.Value;
                aimmySettings["RecoilStrength"] = method;
            };

            leftPanel2.Children.Add(RecoilStrength);

            RecoilKeyChanger = new("Recoil Keybind", "Left");
            RecoilKeyChanger.Reader.Click += (s, x) =>
            {
                RecoilKeyChanger.KeyNotifier.Content = "Listening..";
                Change_Keysize(RecoilKeyChanger, "Listening..");
                RecoilKeyChange = true;
            };

            leftPanel2.Children.Add(RecoilKeyChanger);

            RecoilToggleKeyChanger = new("Recoil Toggle Keybind", "P");
            RecoilToggleKeyChanger.Reader.Click += (s, x) =>
            {
                RecoilToggleKeyChanger.KeyNotifier.Content = "Listening..";
                Change_Keysize(RecoilToggleKeyChanger, "Listening..");
                RecoilToggleKeyChange = true;
            };

            leftPanel2.Children.Add(RecoilToggleKeyChanger);

            ASlider RecoilDelay = new("Fire Rate", 1, "ms");

            RecoilDelay.Slider.Minimum = 1;
            RecoilDelay.Slider.Maximum = 2000;
            RecoilDelay.Slider.Value = aimmySettings["RecoilDelay"];
            RecoilDelay.Slider.TickFrequency = 1;
            RecoilDelay.Slider.ValueChanged += (s, x) =>
            {
                double method = RecoilDelay.Slider.Value;
                aimmySettings["RecoilDelay"] = method;
            };

            leftPanel2.Children.Add(RecoilDelay);

            ASlider RecoilActivationDelay = new("Activation Delay", 1, "ms");

            RecoilActivationDelay.Slider.Minimum = 1;
            RecoilActivationDelay.Slider.Maximum = 2000;
            RecoilActivationDelay.Slider.Value = aimmySettings["RecoilActivationDelay"];
            RecoilActivationDelay.Slider.TickFrequency = 1;
            RecoilActivationDelay.Slider.ValueChanged += (s, x) =>
            {
                double method = RecoilActivationDelay.Slider.Value;
                aimmySettings["RecoilActivationDelay"] = method;
            };

            leftPanel2.Children.Add(RecoilActivationDelay);

            #region FOV System

            SectionPanel rightPanel1 = new SectionPanel();

            rightPanel1.Children.Add(new ALabel("FOV Config"));

            AToggle Show_FOV = new("Show FOV");
            Show_FOV.Reader.Name = "ShowFOV";
            SetupToggle(Show_FOV, state => Bools.ShowFOV = state, Bools.ShowFOV);
            rightPanel1.Children.Add(Show_FOV);

            AToggle Travelling_FOV = new("Dynamic FOV");
            Travelling_FOV.Reader.Name = "TravellingFOV";
            SetupToggle(Travelling_FOV, state => Bools.TravellingFOV = state, Bools.TravellingFOV);
            rightPanel1.Children.Add(Travelling_FOV);

            AColorChanger Change_FOVColor = new("FOV Color");
            Change_FOVColor.ColorChangingBorder.Background = (Brush)new BrushConverter().ConvertFromString(OverlayProperties["FOV_Color"]);
            Change_FOVColor.Reader.Click += (s, x) =>
            {
                System.Windows.Forms.ColorDialog colorDialog = new();
                if (colorDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    Change_FOVColor.ColorChangingBorder.Background = new SolidColorBrush(Color.FromArgb(colorDialog.Color.A, colorDialog.Color.R, colorDialog.Color.G, colorDialog.Color.B));
                    OverlayProperties["FOV_Color"] = Color.FromArgb(colorDialog.Color.A, colorDialog.Color.R, colorDialog.Color.G, colorDialog.Color.B).ToString();
                    AwfulPropertyChanger.PostColor(Color.FromArgb(colorDialog.Color.A, colorDialog.Color.R, colorDialog.Color.G, colorDialog.Color.B));
                }
            };
            rightPanel1.Children.Add(Change_FOVColor);

            ASlider FovSlider = new("FOV Size", 1, "");

            FovSlider.Slider.Minimum = 10;
            FovSlider.Slider.Maximum = 640;
            FovSlider.Slider.Value = aimmySettings["FOV_Size"];
            FovSlider.Slider.TickFrequency = 1;
            FovSlider.Slider.ValueChanged += (s, x) =>
            {
                double FovSize = FovSlider.Slider.Value;
                aimmySettings["FOV_Size"] = FovSize;
                if (_onnxModel != null)
                {
                    _onnxModel.FovSize = (int)FovSize;
                }

                FOVOverlay.FovSize = (int)FovSize;
                AwfulPropertyChanger.PostNewFOVSize();
            };

            rightPanel1.Children.Add(FovSlider);

            #endregion FOV System

            #region Visual Debugging

            SectionPanel rightPanel2 = new SectionPanel();

            rightPanel2.Children.Add(new ALabel("Visual Debugging"));

            AToggle Show_DetectedPlayerWindow = new("Detected Player Window");
            Show_DetectedPlayerWindow.Reader.Name = "ShowDetectedPlayerWindow";
            SetupToggle(Show_DetectedPlayerWindow, state => Bools.ShowDetectedPlayerWindow = state, Bools.ShowDetectedPlayerWindow);
            rightPanel2.Children.Add(Show_DetectedPlayerWindow);

            AToggle Show_CurrentDetectedPlayer = new("Current Detection [Red]");
            Show_CurrentDetectedPlayer.Reader.Name = "ShowCurrentDetectedPlayer";
            SetupToggle(Show_CurrentDetectedPlayer, state => Bools.ShowCurrentDetectedPlayer = state, Bools.ShowCurrentDetectedPlayer);
            rightPanel2.Children.Add(Show_CurrentDetectedPlayer);

            AToggle Show_UnfilteredDetectedPlayer = new("Unflitered Current Detection [Purple]");
            Show_UnfilteredDetectedPlayer.Reader.Name = "ShowUnfilteredDetectedPlayer";
            SetupToggle(Show_UnfilteredDetectedPlayer, state => Bools.ShowUnfilteredDetectedPlayer = state, Bools.ShowUnfilteredDetectedPlayer);
            rightPanel2.Children.Add(Show_UnfilteredDetectedPlayer);

            AToggle Show_Prediction = new("AI Prediction [Green]");
            Show_Prediction.Reader.Name = "ShowAIPrediction";
            SetupToggle(Show_Prediction, state => Bools.ShowPrediction = state, Bools.ShowPrediction);
            rightPanel2.Children.Add(Show_Prediction);

            #endregion Visual Debugging

            #region Visual Debugging Customizer

            SectionPanel rightPanel3 = new SectionPanel();

            rightPanel3.Children.Add(new ALabel("Visual Debugging Customization"));

            ASlider Change_PDW_Size = new("Detection Window Size", 1, "");

            Change_PDW_Size.Slider.Minimum = 10;
            Change_PDW_Size.Slider.Maximum = 100;
            Change_PDW_Size.Slider.Value = OverlayProperties["PDW_Size"];
            Change_PDW_Size.Slider.TickFrequency = 1;
            Change_PDW_Size.Slider.ValueChanged += (s, x) =>
            {
                int PDWSize = (int)Change_PDW_Size.Slider.Value;
                OverlayProperties["PDW_Size"] = PDWSize;
                AwfulPropertyChanger.PostPDWSize(PDWSize);
            };

            rightPanel3.Children.Add(Change_PDW_Size);

            ASlider Change_PDW_CornerRadius = new("Detection Window Corner Radius", 1, "px");

            Change_PDW_CornerRadius.Slider.Minimum = 0;
            Change_PDW_CornerRadius.Slider.Maximum = 100;
            Change_PDW_CornerRadius.Slider.Value = OverlayProperties["PDW_CornerRadius"];
            Change_PDW_CornerRadius.Slider.TickFrequency = 1;
            Change_PDW_CornerRadius.Slider.ValueChanged += (s, x) =>
            {
                int CornerRadiusSize = (int)Change_PDW_CornerRadius.Slider.Value;
                OverlayProperties["PDW_CornerRadius"] = CornerRadiusSize;
                AwfulPropertyChanger.PostPDWCornerRadius(CornerRadiusSize);
            };

            rightPanel3.Children.Add(Change_PDW_CornerRadius);

            ASlider Change_PDW_BorderThickness = new("Detection Window Border Thickness", 1, "px");

            Change_PDW_BorderThickness.Slider.Minimum = 0.1;
            Change_PDW_BorderThickness.Slider.Maximum = 10;
            Change_PDW_BorderThickness.Slider.Value = OverlayProperties["PDW_BorderThickness"];
            Change_PDW_BorderThickness.Slider.TickFrequency = 0.1;
            Change_PDW_BorderThickness.Slider.ValueChanged += (s, x) =>
            {
                double BorderThicknessSize = (double)Change_PDW_BorderThickness.Slider.Value;
                OverlayProperties["PDW_BorderThickness"] = BorderThicknessSize;
                AwfulPropertyChanger.PostPDWBorderThickness(BorderThicknessSize);
            };

            rightPanel3.Children.Add(Change_PDW_BorderThickness);

            ASlider Change_PDW_Opacity = new("Detection Window Opacity", 0.1, "");

            Change_PDW_Opacity.Slider.Minimum = 0;
            Change_PDW_Opacity.Slider.Maximum = 1;
            Change_PDW_Opacity.Slider.Value = OverlayProperties["PDW_Opacity"];
            Change_PDW_Opacity.Slider.TickFrequency = 0.1;
            Change_PDW_Opacity.Slider.ValueChanged += (s, x) =>
            {
                double WindowOpacity = (double)Change_PDW_Opacity.Slider.Value;
                OverlayProperties["PDW_Opacity"] = WindowOpacity;
                AwfulPropertyChanger.PostPDWOpacity(WindowOpacity);
            };

            rightPanel3.Children.Add(Change_PDW_Opacity);

            #endregion Visual Debugging Customizer

            // Set leftPanel to column 0

            // Create a StackPanel to hold both rightPanel and rightPanel2 vertically
            StackPanel rightPanelsStack = new StackPanel();
            StackPanel leftPanelsStack = new StackPanel();

            leftPanelsStack.Children.Add(leftPanel);
            leftPanelsStack.Children.Add(leftPanel1);
            leftPanelsStack.Children.Add(leftPanel2);

            rightPanelsStack.Children.Add(rightPanel);
            rightPanelsStack.Children.Add(rightPanel1);
            rightPanelsStack.Children.Add(rightPanel2);
            rightPanelsStack.Children.Add(rightPanel3);

            rightPanelsStack.VerticalAlignment = VerticalAlignment.Top;
            Grid.SetColumn(rightPanelsStack, 2);
            AimScroller.Children.Add(rightPanelsStack);

            leftPanelsStack.VerticalAlignment = VerticalAlignment.Top;
            Grid.SetColumn(leftPanelsStack, 0);
            AimScroller.Children.Add(leftPanelsStack);


            Change_Keysize(Change_KeyPress, "Right");
            Change_Keysize(Change_KeyPress2, "Left");
            Change_Keysize(RecoilKeyChanger, "Left");
            Change_Keysize(RecoilToggleKeyChanger, "P");
            Change_Keysize(ToggleKeyChanger, "O");

        }

        private void LoadMiscMenu()
        {
            SectionPanel leftPanel = new SectionPanel();

            AButton SaveConfigSystem = new("Save Current Config",
"This will save the current config for the purposes of publishing.");

            SaveConfigSystem.Reader.Click += (s, e) =>
            {
                new SecondaryWindows.ConfigSaver(aimmySettings, lastLoadedModel).ShowDialog();
            };

            leftPanel.Children.Add(SaveConfigSystem);


            leftPanel.VerticalAlignment = VerticalAlignment.Top;
            Grid.SetColumn(leftPanel, 0);
            MiscScroller.Children.Add(leftPanel);
        }

        private void FileWatcher_Reload(object sender, FileSystemEventArgs e)
        {
            Dispatcher.Invoke(() =>
            {
                LoadModelsIntoListBox();

                // Maybe I broke something removing this, fix it, because it was causing a bug where ListBox stops working when something was added =)
                // nori
                //InitializeModel();
            });
        }

        private void InitializeFileWatcher()
        {
            fileWatcher = new FileSystemWatcher();
            fileWatcher.Path = "bin/models";
            fileWatcher.Filter = "*.onnx";
            fileWatcher.EnableRaisingEvents = true;
            fileWatcher.Created += FileWatcher_Reload;
            fileWatcher.Deleted += FileWatcher_Reload;
            fileWatcher.Renamed += FileWatcher_Reload;
        }

        private bool ModelLoadDebounce = false;

        private void InitializeModel()
        {
            if (!ModelLoadDebounce)
            {
                if (!(Bools.ConstantTracking && Bools.AIAimAligner))
                {
                    ModelLoadDebounce = true;

                    string selectedModel = SelectorListBox.SelectedItem?.ToString();
                    if (selectedModel == null) return;

                    string modelPath = Path.Combine("bin/models", selectedModel);

                    _onnxModel?.Dispose();
                    Task.Delay(100);
                    Task.Run(() => _onnxModel = new AIModel(modelPath)
                    {
                        ConfidenceThreshold = (float)(aimmySettings["AI_Min_Conf"] / 100.0f),
                        CollectData = toggleState["CollectData"],
                        FovSize = (int)aimmySettings["FOV_Size"]
                    });

                    SelectedModelNotifier.Content = "Loaded Model: " + selectedModel;
                    lastLoadedModel = selectedModel;

                    ModelLoadDebounce = false;
                }
            }
        }

        private void LoadModelsIntoListBox()
        {
            string[] onnxFiles = Directory.GetFiles("bin/models", "*.onnx");
            SelectorListBox.Items.Clear();

            foreach (string filePath in onnxFiles)
            {
                SelectorListBox.Items.Add(Path.GetFileName(filePath));
            }

            if (SelectorListBox.Items.Count > 0)
            {
                if (!SelectorListBox.Items.Contains(lastLoadedModel) && lastLoadedModel != "N/A")
                {
                    SelectorListBox.SelectedIndex = 0;
                    lastLoadedModel = SelectorListBox.Items[0].ToString();
                }
                else
                {
                    SelectorListBox.SelectedItem = lastLoadedModel;
                }
                SelectedModelNotifier.Content = "Loaded Model: " + lastLoadedModel;
            }
            ModelLoadDebounce = false;
        }

        private void SelectorListBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            InitializeModel();
        }

        private async Task LoadConfigAsync(string path)
        {
            if (File.Exists(path))
            {
                string json = await File.ReadAllTextAsync(path);

                var config = JsonConvert.DeserializeObject<Dictionary<string, dynamic>>(json);
                if (config != null)
                {
                    foreach (var (key, value) in config)
                    {
                        if (aimmySettings.TryGetValue(key, out var currentValue))
                        {
                            aimmySettings[key] = value;
                        }
                        else if (key == "TopMost" && value is bool topMostValue)
                        {
                            toggleState["TopMost"] = topMostValue;
                            this.Topmost = topMostValue;
                        }
                    }
                }
            }
            if (aimmySettings["Suggested_Model"] != string.Empty)
            {
                System.Windows.Forms.MessageBox.Show("The creator of this model suggests you use this model:" +
                    "\n" +
                    aimmySettings["Suggested_Model"], "Suggested Model - Aimmy");
            }

            // We'll attempt to update the AI Settings but it may not be loaded yet.
            try
            {
                int fovSize = (int)aimmySettings["FOV_Size"];
                FOVOverlay.FovSize = fovSize;
                AwfulPropertyChanger.PostNewFOVSize();

                _onnxModel.FovSize = fovSize;
                _onnxModel.ConfidenceThreshold = (float)(aimmySettings["AI_Min_Conf"] / 100.0f);
            }
            catch { }

            string fileName = Path.GetFileName(path);
            if (ConfigSelectorListBox.Items.Contains(fileName))
            {
                ConfigSelectorListBox.SelectedItem = fileName;
                lastLoadedConfig = ConfigSelectorListBox.SelectedItem.ToString();
                SetSelectedConfig();
            }

            ReloadMenu();
        }

        private async Task LoadOverlayPropertiesAsync(string path)
        {
            if (File.Exists(path))
            {
                string json = await File.ReadAllTextAsync(path);

                var config = JsonConvert.DeserializeObject<Dictionary<string, dynamic>>(json);
                if (config != null)
                {
                    foreach (var (key, value) in config)
                    {
                        if (OverlayProperties.TryGetValue(key, out var currentValue))
                        {
                            OverlayProperties[key] = value;
                        }
                    }
                }
            }

            try
            {
                AwfulPropertyChanger.PostColor((Color)ColorConverter.ConvertFromString(OverlayProperties["FOV_Color"]));
                AwfulPropertyChanger.PostPDWSize((int)OverlayProperties["PDW_Size"]);
                AwfulPropertyChanger.PostPDWCornerRadius((int)OverlayProperties["PDW_CornerRadius"]);
                AwfulPropertyChanger.PostPDWBorderThickness((int)OverlayProperties["PDW_BorderThickness"]);
                AwfulPropertyChanger.PostPDWOpacity((Double)OverlayProperties["PDW_Opacity"]);
            }
            catch (Exception ex)
            {
                System.Windows.Forms.MessageBox.Show(ex.ToString());
            }

            //ReloadMenu();
        }

        private void ReloadMenu()
        {
            AimScroller.Children.Clear();
            MiscScroller.Children.Clear();
            SettingsScroller.Children.Clear();

            LoadAimMenu();
            LoadMiscMenu();
            LoadSettingsMenu();
        }

        private void ConfigWatcher_Reload(object sender, FileSystemEventArgs e)
        {
            this.Dispatcher.Invoke(LoadConfigsIntoListBox);
        }

        private void InitializeConfigWatcher()
        {
            ConfigfileWatcher = new FileSystemWatcher();
            ConfigfileWatcher.Path = "bin/configs";
            ConfigfileWatcher.Filters.Add("*.json");
            ConfigfileWatcher.Filters.Add("*.cfg");
            ConfigfileWatcher.EnableRaisingEvents = true;
            ConfigfileWatcher.Created += ConfigWatcher_Reload;
            ConfigfileWatcher.Deleted += ConfigWatcher_Reload;
            ConfigfileWatcher.Renamed += ConfigWatcher_Reload;
        }

        private void LoadConfigsIntoListBox()
        {
            ConfigSelectorListBox.Items.Clear();

            foreach (string filePath in Directory.GetFiles("bin/configs"))
            {
                string fileName = Path.GetFileName(filePath);
                ConfigSelectorListBox.Items.Add(fileName);
            }

            //SetSelectedConfig();
        }

        private void SetSelectedConfig()
        {
            if (ConfigSelectorListBox.Items.Count > 0)
            {
                if (!ConfigSelectorListBox.Items.Contains(lastLoadedConfig) && lastLoadedConfig != "N/A")
                {
                    ConfigSelectorListBox.SelectedIndex = 0;
                    lastLoadedConfig = ConfigSelectorListBox.Items[0].ToString();
                }
                else
                {
                    ConfigSelectorListBox.SelectedItem = lastLoadedConfig;
                }
                SelectedConfigNotifier.Content = "Loaded Config: " + lastLoadedConfig;
            }
        }

        private async void ConfigSelectorListBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (ConfigSelectorListBox.SelectedItem != null)
            {
                await LoadConfigAsync($"bin/configs/{ConfigSelectorListBox.SelectedItem.ToString()}");
            }
        }

        private void LoadStoreMenu()
        {
            DownloadGateway(ModelStoreScroller, AvailableModels, "models");
            DownloadGateway(ConfigStoreScroller, AvailableConfigs, "configs");
        }

        private void DownloadGateway(StackPanel Scroller, HashSet<string> entries, string folder)
        {
            if (entries.Count > 0)
            {
                foreach (var entry in entries)
                    Scroller.Children.Add(new ADownloadGateway(entry, folder));
            }
            else
            {
                Scroller.Children.Clear();
                LackOfConfigsText.Visibility = Visibility.Visible;
            }
        }

        private void LoadSettingsMenu()
        {
            SectionPanel aimingConfigurationPanel = new SectionPanel();

            SettingsScroller.Children.Add(new AInfoSection());

            AToggle CollectDataWhilePlaying = new("Collect Data While Playing");
            CollectDataWhilePlaying.Reader.Name = "CollectData";
            SetupToggle(CollectDataWhilePlaying, state => Bools.CollectDataWhilePlaying = state, Bools.CollectDataWhilePlaying);
            aimingConfigurationPanel.Children.Add(CollectDataWhilePlaying);

            bool topMostInitialState = toggleState.ContainsKey("TopMost") ? toggleState["TopMost"] : false;

            AToggle TopMost = new("UI TopMost");
            TopMost.Reader.Name = "TopMost";
            SetupToggle(TopMost, state => Bools.TopMost = state, topMostInitialState);

            aimingConfigurationPanel.Children.Add(TopMost);
            SettingsScroller.Children.Add(aimingConfigurationPanel);
        }

        #region Window Controls

        private void Exit_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.Application.Current.Shutdown();
        }

        private void Minimize_Click(object sender, RoutedEventArgs e)
        {
            WindowState = WindowState.Minimized;
        }

        private void Border_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            DragMove();
        }

        private static bool SavedData = false;

        private void Window_Closing(object sender, CancelEventArgs e)
        {
            // Prevent saving overwrite
            if (SavedData) return;

            // Save to Default Config
            try
            {
                var extendedSettings = new Dictionary<string, object>();
                foreach (var kvp in aimmySettings)
                {
                    extendedSettings[kvp.Key] = kvp.Value;
                }

                // Add topmost
                extendedSettings["TopMost"] = this.Topmost ? true : false;

                string json = JsonConvert.SerializeObject(extendedSettings, Formatting.Indented);
                File.WriteAllText("bin/configs/Default.cfg", json);
            }
            catch (Exception x)
            {
                Console.WriteLine("Error saving configuration: " + x.Message);
            }

            // Save Overlay Properties Data
            // Nori
            try
            {
                var OverlaySettings = new Dictionary<string, object>();
                foreach (var kvp in OverlayProperties)
                {
                    OverlaySettings[kvp.Key] = kvp.Value;
                }

                string json = JsonConvert.SerializeObject(OverlaySettings, Formatting.Indented);
                File.WriteAllText("bin/Overlay.cfg", json);
            }
            catch (Exception x)
            {
                Console.WriteLine("Error saving configuration: " + x.Message);
            }

            SavedData = true;

            // Unhook keybind hooker
            bindingManager.StopListening();
            FOVOverlay.Close();
            DetectedPlayerOverlay.Close();

            // Dispose (best practice)
            fileWatcher.Dispose();
            ConfigfileWatcher.Dispose();
            cts.Dispose();

            // Close
            System.Windows.Application.Current.Shutdown();
        }

        #endregion Window Controls
    }
}