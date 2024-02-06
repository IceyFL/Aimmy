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
        private InputBindingManager RightClick;
        private InputBindingManager LeftClick;
        private bool IsHolding_Binding = false;
        private bool IsHolding_Left = false;
        private bool IsHolding_Right = false;
        private CancellationTokenSource cts;

        private enum MenuPosition
        {
            AimMenu,
            MiscMenu,
            SelectorMenu,
            SettingsMenu
        }

        private DateTime lastExecutionTime = DateTime.MinValue;

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
            { "AimMethod", 0 },
            { "RecoilStrength", 10 },
            { "AIFPS", 40 }
        };

        private Dictionary<string, bool> toggleState = new()
        {
            { "AimbotToggle", false },
            { "AlwaysOn", false },
            { "PredictionToggle", false },
            { "AimViewToggle", false },
            { "TriggerBot", false },
            { "CollectData", false },
            { "TopMost", false },
            { "Recoil", false }
        };

        // PDW == PlayerDetectionWindow
        public Dictionary<string, dynamic> OverlayProperties = new()
        {
            { "FOV_Color", "#ff0000"},
            { "PDW_Size", 50 },
            { "PDW_CornerRadius", 0 },
            { "PDW_BorderThickness", 1 },
            { "PDW_Opacity", 1 }
        };

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
            this.Title = Path.GetFileNameWithoutExtension(Assembly.GetExecutingAssembly().Location);

            // Check to see if certain items are installed
            RequirementsManager RM = new();
            if (!RM.IsVCRedistInstalled())
            {
                MessageBox.Show("Visual C++ Redistributables x64 are not installed on this device, please install them before using Aimmy to avoid issues.", "Load Error");
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
                MessageBox.Show($"Error creating a required directory: {ex}");
                Application.Current.Shutdown(); // We don't want to continue running without that folder.
            }

            // Setup key/mouse hook
            bindingManager = new InputBindingManager();
            bindingManager.SetupDefault("Right");
            bindingManager.OnBindingPressed += (binding) => { IsHolding_Binding = true; };
            bindingManager.OnBindingReleased += (binding) => { IsHolding_Binding = false; };

            RightClick = new InputBindingManager();
            RightClick.SetupDefault("Right");
            RightClick.OnBindingPressed += (binding) => { IsHolding_Right = true; };
            RightClick.OnBindingReleased += (binding) => { IsHolding_Right = false; };

            LeftClick = new InputBindingManager();
            LeftClick.SetupDefault("Left");
            LeftClick.OnBindingPressed += (binding) => { IsHolding_Left = true; };
            LeftClick.OnBindingReleased += (binding) => { IsHolding_Left = false; };

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
            Task.Run(() => TitleLoop());
            Task.Run(() => RecoilLoop());
        }

        private HashSet<string> AvailableModels = new();
        private HashSet<string> AvailableConfigs = new();

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
                MessageBox.Show("Github is irretrieveable right now, the Downloadable Model menu will not work right now, sorry!");
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

            mouse_event(MOUSEEVENTF_MOVE, (uint)newPosition.X, (uint)newPosition2.Y, 0, 0);

            if (toggleState["TriggerBot"])
            {
                Task.Run(DoTriggerClick);
            }
        }

        #endregion Mouse Movement / Clicking Handler

        #region Aim Aligner Main and Loop

        public async Task ModelCapture(bool TriggerOnly = false)
        {
            var closestPrediction = await _onnxModel.GetClosestPredictionToCenterAsync((int)aimmySettings["AimMethod"]);
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
                TimeSpan elapsed = DateTime.Now - lastExecutionTime;
                if (elapsed.TotalMilliseconds > 1000.0 / aimmySettings["AIFPS"])
                {
                    lastExecutionTime = DateTime.Now;
                    if (toggleState["AimbotToggle"] && Bools.ConstantTracking == false)
                    {
                        if (IsHolding_Binding)
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

        private async Task TitleLoop()
        {
            cts = new CancellationTokenSource();

            while (!cts.Token.IsCancellationRequested)
            {
                await Application.Current.Dispatcher.InvokeAsync(() =>
                {
                    Random random = new Random();
                    int randomNumber = random.Next(1, 10000);
                    string title = randomNumber.ToString("D4");
                    this.Title = title;
                });
                await Task.Delay(2000);
            }
        }

        private async Task RecoilLoop()
        {
            cts = new CancellationTokenSource();

            while (!cts.Token.IsCancellationRequested)
            {
                if (Bools.Recoil && IsHolding_Right && IsHolding_Left)
                {
                    mouse_event(MOUSEEVENTF_MOVE, 0, (uint)aimmySettings["RecoilStrength"], 0, 0);
                }
                await Task.Delay(250);
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

        private void SetToggleState(AToggle toggle)
        {
            bool state = (bool)toggle.Reader.Tag;

            // Stop them from turning on anything until the model has been selected.
            if ((toggle.Reader.Name == "AimbotToggle" || toggle.Reader.Name == "AimOnlyWhenBindingHeld" || toggle.Reader.Name == "ConstantAITracking" || toggle.Reader.Name == "TriggerBot" || toggle.Reader.Name == "CollectData") && lastLoadedModel == "N/A")
            {
                SetToggleStatesOnModelNotSelected();
                MessageBox.Show("Please select a model in the Model Selector before toggling.", "Toggle Error");
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
                ResetMenuColors();
                clickedButton.Foreground = (Brush)brushcolor.ConvertFromString("White");
                ApplyMenuAnimations(position);
                UpdateMenuVisibility(position);
            }
        }

        private void ResetMenuColors()
        {
            Selection1.Foreground = Selection2.Foreground = Selection3.Foreground = Selection4.Foreground =
                (Brush)brushcolor.ConvertFromString("#ffffff");
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
            leftPanel.Children.Add(new ALabel("Aiming Configuration"));


            AToggle Enable_AIAimAligner = new(this, "Enable AI Aim Aligner",
                "This will enable the AI's ability to align the aim.");
            Enable_AIAimAligner.Reader.Name = "AimbotToggle";
            SetupToggle(Enable_AIAimAligner, state => Bools.AIAimAligner = state, Bools.AIAimAligner);
            leftPanel.Children.Add(Enable_AIAimAligner);

            AToggle Enable_ConstantAITracking = new(this, "Enable Constant AI Aligner",
    "This will let the AI run 24/7 to let Visual Debugging run.");
            Enable_ConstantAITracking.Reader.Name = "ConstantAITracking";
            SetupToggle(Enable_ConstantAITracking, state => Bools.ConstantTracking = state, Bools.ConstantTracking);
            leftPanel.Children.Add(Enable_ConstantAITracking);

            ASlider AimMethod = new(this, "Aim Method", "Aim Method",
                "This setting controls how the AI decides which detection to aim at.",
                1);

            Dictionary<double, string> weaponNames = new Dictionary<double, string>
            {
                { 0, "Closest Detection" },
                { 1, "Highest Conf Detection" }
            };
            AimMethod.Slider.Minimum = 0;
            AimMethod.Slider.Maximum = 1;
            AimMethod.Slider.Value = aimmySettings["AimMethod"];
            AimMethod.Slider.TickFrequency = 1;
            AimMethod.Slider.ValueChanged += (s, x) =>
            AimMethod.AdjustNotifier.Content = weaponNames[AimMethod.Slider.Value];
            {
                double method = AimMethod.Slider.Value;
                aimmySettings["AimMethod"] = method;
                AimMethod.AdjustNotifier.Content = weaponNames[AimMethod.Slider.Value];
            };

            leftPanel.Children.Add(AimMethod);

            ASlider FPS = new(this, "AI FPS", "AI FPS",
            "This setting controls how many scans the AI does per second.",
            1);

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

            AToggle AimOnlyWhenBindingHeld = new(this, "Aim only when Trigger Button is held", // this can be simplifed with a toggle between constant and hold (toggle/hold), ill do it later.
"This will stop the AI from aiming unless the Trigger Button is held.");
            AimOnlyWhenBindingHeld.Reader.Name = "AimOnlyWhenBindingHeld";
            SetupToggle(AimOnlyWhenBindingHeld, state => Bools.AimOnlyWhenBindingHeld = state, Bools.AimOnlyWhenBindingHeld);
            leftPanel.Children.Add(AimOnlyWhenBindingHeld);

            AKeyChanger Change_KeyPress = new("Change Keybind", "Right");
            Change_KeyPress.Reader.Click += (s, x) =>
            {
                Change_KeyPress.KeyNotifier.Content = "Listening..";
                bindingManager.StartListeningForBinding();
            };

            bindingManager.OnBindingSet += (binding) =>
            {
                Change_KeyPress.KeyNotifier.Content = binding;
            };

            leftPanel.Children.Add(Change_KeyPress);

            //AToggle Enable_AlwaysOn = new AToggle(this, "Aim Align Always On",
            //   "This will keep the aim aligner on 24/7 so you don't have to hold a toggle.");
            //Enable_AlwaysOn.Reader.Name = "AlwaysOn";
            //SetupToggle(Enable_AlwaysOn, state => Bools.AIAlwaysOn = state, Bools.AIAlwaysOn);
            //AimScroller.Children.Add(Enable_AlwaysOn);

            AToggle Enable_AIPredictions = new(this, "Enable Predictions",
               "This will use a KalmanFilter algorithm to predict aim patterns for better tracing of enemies.");
            Enable_AIPredictions.Reader.Name = "PredictionToggle";
            SetupToggle(Enable_AIPredictions, state => Bools.AIPredictions = state, Bools.AIPredictions);
            leftPanel.Children.Add(Enable_AIPredictions);

            ASlider MouseSensitivtyX = new ASlider(this, "Mouse Sensitivty X", "Sensitivty",
    "This setting controls how fast your mouse moves to a detection, if it moves too fast you need to set it to a higher number.",
    0.01);

            MouseSensitivtyX.Slider.Minimum = 0.01;
            MouseSensitivtyX.Slider.Maximum = 1;
            MouseSensitivtyX.Slider.Value = aimmySettings["Mouse_Sens"];
            MouseSensitivtyX.Slider.TickFrequency = 0.01;
            MouseSensitivtyX.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Mouse_Sens"] = MouseSensitivtyX.Slider.Value;
            };

            leftPanel.Children.Add(MouseSensitivtyX);

            ASlider MouseSensitivtyY = new ASlider(this, "Mouse Sensitivty Y", "Sensitivty",
                "This setting controls how fast your mouse moves to a detection, if it moves too fast you need to set it to a higher number.",
                0.01);

            MouseSensitivtyY.Slider.Minimum = 0.01;
            MouseSensitivtyY.Slider.Maximum = 1;
            MouseSensitivtyY.Slider.Value = aimmySettings["Mouse_SensY"];
            MouseSensitivtyY.Slider.TickFrequency = 0.01;
            MouseSensitivtyY.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Mouse_SensY"] = MouseSensitivtyY.Slider.Value;
            };

            leftPanel.Children.Add(MouseSensitivtyY);

            ASlider MouseJitter = new(this, "Mouse Jitter", "Jitter",
                "This setting controls how much fake jitter is added to the mouse movements. Aim is almost never steady so this adds a nice layer of humanizing onto aim.",
                0.01);

            MouseJitter.Slider.Minimum = 0;
            MouseJitter.Slider.Maximum = 15;
            MouseJitter.Slider.Value = aimmySettings["Mouse_Jitter"];
            MouseJitter.Slider.TickFrequency = 1;
            MouseJitter.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Mouse_Jitter"] = MouseJitter.Slider.Value;
            };
            leftPanel.Children.Add(MouseJitter);

            ASlider YOffset = new(this, "Y Offset (Up/Down)", "Offset",
                "This setting controls how high / low you aim. A lower number will result in a higher aim. A higher number will result in a lower aim.",
                1);

            YOffset.Slider.Minimum = -150;
            YOffset.Slider.Maximum = 150;
            YOffset.Slider.Value = aimmySettings["Y_Offset"];
            YOffset.Slider.TickFrequency = 1;
            YOffset.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Y_Offset"] = YOffset.Slider.Value;
            };

            leftPanel.Children.Add(YOffset);

            ASlider XOffset = new(this, "X Offset (Left/Right)", "Offset",
                "This setting controls which way your aim leans. A lower number will result in an aim that leans to the left. A higher number will result in an aim that leans to the right",
                1);

            XOffset.Slider.Minimum = -150;
            XOffset.Slider.Maximum = 150;
            XOffset.Slider.Value = aimmySettings["X_Offset"];
            XOffset.Slider.TickFrequency = 1;
            XOffset.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["X_Offset"] = XOffset.Slider.Value;
            };

            leftPanel.Children.Add(XOffset);

            #region FOV System

            rightPanel.Children.Add(new ALabel("FOV System"));

            AToggle Show_FOV = new(this, "Show FOV",
                "This will show a circle around your screen that show what the AI is considering on the screen at a given moment.");
            Show_FOV.Reader.Name = "ShowFOV";
            SetupToggle(Show_FOV, state => Bools.ShowFOV = state, Bools.ShowFOV);
            rightPanel.Children.Add(Show_FOV);

            AToggle Travelling_FOV = new(this, "Travelling FOV",
    "This will allow the FOV circle to travel alongside your mouse.\n" +
    "[PLEASE NOTE]: This does not have any effect on the AI's personal FOV, this feature is only for the visual effect.");
            Travelling_FOV.Reader.Name = "TravellingFOV";
            SetupToggle(Travelling_FOV, state => Bools.TravellingFOV = state, Bools.TravellingFOV);
            rightPanel.Children.Add(Travelling_FOV);

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
            rightPanel.Children.Add(Change_FOVColor);

            ASlider FovSlider = new(this, "FOV Size", "Size of FOV",
                "This setting controls how much of your screen is considered in the AI's decision making and how big the circle on your screen will be.",
                1);

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

            rightPanel.Children.Add(FovSlider);

            #endregion FOV System

            #region Visual Debugging

            SectionPanel rightPanel2 = new SectionPanel();

            rightPanel2.Children.Add(new ALabel("Visual Debugging"));

            AToggle Show_DetectedPlayerWindow = new(this, "Detected Player Window",
                "Shows the Detected Player Overlay, the options below will not work if this is enabled!");
            Show_DetectedPlayerWindow.Reader.Name = "ShowDetectedPlayerWindow";
            SetupToggle(Show_DetectedPlayerWindow, state => Bools.ShowDetectedPlayerWindow = state, Bools.ShowDetectedPlayerWindow);
            rightPanel2.Children.Add(Show_DetectedPlayerWindow);

            AToggle Show_CurrentDetectedPlayer = new(this, "Current Detection [Red]",
    "This will show a rectangle on the player that the AI is considering on the screen at a given moment.");
            Show_CurrentDetectedPlayer.Reader.Name = "ShowCurrentDetectedPlayer";
            SetupToggle(Show_CurrentDetectedPlayer, state => Bools.ShowCurrentDetectedPlayer = state, Bools.ShowCurrentDetectedPlayer);
            rightPanel2.Children.Add(Show_CurrentDetectedPlayer);

            AToggle Show_UnfilteredDetectedPlayer = new(this, "Unflitered Current Detection [Purple]",
                "This will show a rectangle on the player that the AI is considering on the screen at a given moment without considering the adjusted X and Y axis.");
            Show_UnfilteredDetectedPlayer.Reader.Name = "ShowUnfilteredDetectedPlayer";
            SetupToggle(Show_UnfilteredDetectedPlayer, state => Bools.ShowUnfilteredDetectedPlayer = state, Bools.ShowUnfilteredDetectedPlayer);
            rightPanel2.Children.Add(Show_UnfilteredDetectedPlayer);

            AToggle Show_Prediction = new(this, "AI Prediction [Green]",
                "This will show a rectangle on where the AI assumes the player will be on the screen at a given moment.");
            Show_Prediction.Reader.Name = "ShowAIPrediction";
            SetupToggle(Show_Prediction, state => Bools.ShowPrediction = state, Bools.ShowPrediction);
            rightPanel2.Children.Add(Show_Prediction);

            #endregion Visual Debugging

            #region Visual Debugging Customizer

            SectionPanel rightPanel3 = new SectionPanel();

            rightPanel3.Children.Add(new ALabel("Visual Debugging Customization"));

            ASlider Change_PDW_Size = new(this, "Detection Window Size", "Size",
                "This setting controls the size of your Detected Player Windows.",
                1);

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

            ASlider Change_PDW_CornerRadius = new(this, "Detection Window Corner Radius", "Corner Radius",
                "This setting controls the corner radius of your Detected Player Windows.",
                1);

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

            ASlider Change_PDW_BorderThickness = new(this, "Detection Window Border Thickness", "Border Thickness",
                "This setting controls the Border Thickness of your Detected Player Windows.",
                1);

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

            ASlider Change_PDW_Opacity = new(this, "Detection Window Opacity", "Opacity",
                "This setting controls the Opacity of your Detected Player Windows.",
                0.1);

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
            leftPanel.VerticalAlignment = VerticalAlignment.Top;
            Grid.SetColumn(leftPanel, 0);
            AimScroller.Children.Add(leftPanel);

            // Create a StackPanel to hold both rightPanel and rightPanel2 vertically
            StackPanel rightPanelsStack = new StackPanel();

            // Add rightPanel to the StackPanel
            rightPanelsStack.Children.Add(rightPanel);

            // Add rightPanel2 to the StackPanel
            rightPanelsStack.Children.Add(rightPanel2);

            // Add rightPanel2 to the StackPanel
            rightPanelsStack.Children.Add(rightPanel3);

            // Set the StackPanel to column 1
            Grid.SetColumn(rightPanelsStack, 2);
            AimScroller.Children.Add(rightPanelsStack);

        }

        private void LoadMiscMenu()
        {
            SectionPanel leftPanel = new SectionPanel();
            SectionPanel RightPanel = new SectionPanel();

            leftPanel.Children.Add(new ALabel("Auto Trigger"));

            AToggle Enable_TriggerBot = new(this, "Enable Auto Trigger",
                "This will enable the AI's ability to shoot whenever it sees a target.");
            Enable_TriggerBot.Reader.Name = "TriggerBot";
            SetupToggle(Enable_TriggerBot, state => Bools.Triggerbot = state, Bools.Triggerbot);
            leftPanel.Children.Add(Enable_TriggerBot);

            ASlider TriggerBot_Delay = new(this, "Auto Trigger Delay", "Seconds",
                "This slider will control how many miliseconds it will take to initiate a trigger.",
                0.1);

            TriggerBot_Delay.Slider.Minimum = 0.01;
            TriggerBot_Delay.Slider.Maximum = 1;
            TriggerBot_Delay.Slider.Value = aimmySettings["Trigger_Delay"];
            TriggerBot_Delay.Slider.TickFrequency = 0.01;
            TriggerBot_Delay.Slider.ValueChanged += (s, x) =>
            {
                aimmySettings["Trigger_Delay"] = TriggerBot_Delay.Slider.Value;
            };

            leftPanel.Children.Add(TriggerBot_Delay);

            RightPanel.Children.Add(new ALabel("Anti Recoil"));

            AToggle RecoilState = new(this, "Anti Recoil",
"This will counter recoil.");
            RecoilState.Reader.Name = "RecoilToggle";
            SetupToggle(RecoilState, state => Bools.Recoil = state, Bools.Recoil);
            RightPanel.Children.Add(RecoilState);

            ASlider RecoilStrength = new(this, "Anti Recoil Strength", "Anti Recoil Strength",
            "This setting controls the strength of the anti recoil.",
            1);

            RecoilStrength.Slider.Minimum = 1;
            RecoilStrength.Slider.Maximum = 50;
            RecoilStrength.Slider.Value = aimmySettings["RecoilStrength"];
            RecoilStrength.Slider.TickFrequency = 1;
            RecoilStrength.Slider.ValueChanged += (s, x) =>
            {
                double method = RecoilStrength.Slider.Value;
                aimmySettings["RecoilStrength"] = method;
            };

            RightPanel.Children.Add(RecoilStrength);

            leftPanel.VerticalAlignment = VerticalAlignment.Top;
            Grid.SetColumn(leftPanel, 0);
            MiscScroller.Children.Add(leftPanel);

            RightPanel.VerticalAlignment = VerticalAlignment.Top;
            Grid.SetColumn(RightPanel, 2);
            MiscScroller.Children.Add(RightPanel);
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
                    _onnxModel = new AIModel(modelPath)
                    {
                        ConfidenceThreshold = (float)(aimmySettings["AI_Min_Conf"] / 100.0f),
                        CollectData = toggleState["CollectData"],
                        FovSize = (int)aimmySettings["FOV_Size"]
                    };

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
                MessageBox.Show("The creator of this model suggests you use this model:" +
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
                MessageBox.Show(ex.ToString());
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

            AButton SaveConfigSystem = new(this, "Save Current Config",
"This will save the current config for the purposes of publishing.");

            SaveConfigSystem.Reader.Click += (s, e) =>
            {
                new SecondaryWindows.ConfigSaver(aimmySettings, lastLoadedModel).ShowDialog();
            };

            aimingConfigurationPanel.Children.Add(SaveConfigSystem);

            AToggle CollectDataWhilePlaying = new(this, "Collect Data While Playing",
                "This will enable the AI's ability to take a picture of your screen when the trigger key is pressed.");
            CollectDataWhilePlaying.Reader.Name = "CollectData";
            SetupToggle(CollectDataWhilePlaying, state => Bools.CollectDataWhilePlaying = state, Bools.CollectDataWhilePlaying);
            aimingConfigurationPanel.Children.Add(CollectDataWhilePlaying);


            ASlider AIMinimumConfidence = new(this, "AI Minimum Confidence", "% Confidence",
                "This setting controls how confident the AI needs to be before making the decision to aim.",
                1);

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
                        MessageBox.Show("Unable to set confidence, please select a model and try again.", "Slider Error");
                        AIMinimumConfidence.Slider.Value = aimmySettings["AI_Min_Conf"];
                    }
                }
            };

            aimingConfigurationPanel.Children.Add(AIMinimumConfidence);

            bool topMostInitialState = toggleState.ContainsKey("TopMost") ? toggleState["TopMost"] : false;

            AToggle TopMost = new(this, "UI TopMost",
                "This will toggle the UI's TopMost, meaning it can hide behind other windows vs always being on top.");
            TopMost.Reader.Name = "TopMost";
            SetupToggle(TopMost, state => Bools.TopMost = state, topMostInitialState);

            aimingConfigurationPanel.Children.Add(TopMost);
            SettingsScroller.Children.Add(aimingConfigurationPanel);
        }

        #region Window Controls

        private void Exit_Click(object sender, RoutedEventArgs e)
        {
            Application.Current.Shutdown();
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
            Application.Current.Shutdown();
        }

        #endregion Window Controls
    }
}