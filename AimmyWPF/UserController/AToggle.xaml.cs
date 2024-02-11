using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace AimmyWPF.UserController
{
    /// <summary>
    /// Interaction logic for AToggle.xaml
    /// </summary>
    public partial class AToggle : UserControl
    {
        public AToggle(string Text)
        {
            ToggleBorderColor = (Brush)new BrushConverter().ConvertFromString("#712fc6");
            InitializeComponent();
            Title.Content = Text;
        }

        // Define the ToggleBorderColor property
        public Brush ToggleBorderColor
        {
            get { return (Brush)GetValue(ToggleBorderColorProperty); }
            set { SetValue(ToggleBorderColorProperty, value); }
        }

        public static readonly DependencyProperty ToggleBorderColorProperty =
            DependencyProperty.Register("ToggleBorderColor", typeof(Brush), typeof(AToggle), new PropertyMetadata(null));

        public void Reader_Click(object sender, RoutedEventArgs e)
        {
            // Toggle the IsChecked property of the CheckBox
            ToggleCheckBox.IsChecked = !ToggleCheckBox.IsChecked;
        }

        // Reference: https://stackoverflow.com/questions/34815532/start-storyboard-from-c-sharp-code
    }
}
