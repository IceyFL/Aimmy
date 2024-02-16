using System.Reflection;
using System.Windows;
using AimmyWPF.Class;

namespace Visualization
{
    public partial class PlayerDetectionWindow : Window
    {
        public PlayerDetectionWindow()
        {
            InitializeComponent();

            this.Title = System.IO.Path.GetFileNameWithoutExtension(Assembly.GetExecutingAssembly().Location);

            AwfulPropertyChanger.ReceivePDWSize = ChangeSize;
            AwfulPropertyChanger.ReceivePDWCornerRadius = ChangeCornerRadius;
            AwfulPropertyChanger.ReceivePDWBorderThickness = ChangeBorderThickness;
            AwfulPropertyChanger.ReceivePDWOpacity = ChangeOpacity;
        }

        private void ChangeSize(int newint)
        {
            DetectedPlayerFocus.Width = newint;
            DetectedPlayerFocus.Height = newint;

            UnfilteredPlayerFocus.Width = newint;
            UnfilteredPlayerFocus.Height = newint;

            PredictionFocus.Width = newint;
            PredictionFocus.Height = newint;
        }

        private void ChangeCornerRadius(int newint)
        {
            DetectedPlayerFocus.CornerRadius = new CornerRadius(newint);
            UnfilteredPlayerFocus.CornerRadius = new CornerRadius(newint);
            PredictionFocus.CornerRadius = new CornerRadius(newint);
        }

        void ChangeBorderThickness(double newdouble)
        {
            DetectedPlayerFocus.BorderThickness = new Thickness(newdouble);
            UnfilteredPlayerFocus.BorderThickness = new Thickness(newdouble);
            PredictionFocus.BorderThickness = new Thickness(newdouble);

        }

        private void ChangeOpacity(double newdouble)
        {
            DetectedPlayerFocus.Opacity = newdouble;
            UnfilteredPlayerFocus.Opacity = newdouble;
            PredictionFocus.Opacity = newdouble;
        }


        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            e.Cancel = true;
            this.Hide();
        }
    }
}