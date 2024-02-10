using System;
using System.Windows.Controls;

namespace AimmyWPF.UserController
{
    /// <summary>
    /// Interaction logic for ASlider.xaml
    /// </summary>
    public partial class ASlider : UserControl
    {
        public ASlider(string Text, double ButtonSteps)
        {
            InitializeComponent();
            Title.Content = Text;

            Slider.ValueChanged += (s, e) =>
            {
                if (AdjustNotifier != null)
                    AdjustNotifier.Content = $"{Slider.Value.ToString()}";

                // Added by Nori
                Slider.Value = Math.Round(Slider.Value, 2);
            };

            // Added by Nori
            SubtractOne.Click += (s, e) => Slider.Value = Slider.Value - ButtonSteps;
            AddOne.Click += (s, e) => Slider.Value = Slider.Value + ButtonSteps;
        }
    }
}