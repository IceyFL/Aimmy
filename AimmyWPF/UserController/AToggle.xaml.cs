using System.Windows;
using System.Windows.Controls;

namespace AimmyWPF.UserController
{
    /// <summary>
    /// Interaction logic for AToggle.xaml
    /// </summary>
    public partial class AToggle : UserControl
    {
        public AToggle(string Text)
        {
            InitializeComponent();
            Title.Content = Text;

        }

        public void Reader_Click(object sender, RoutedEventArgs e)
        {
            // Toggle the IsChecked property of the CheckBox
            ToggleCheckBox.IsChecked = !ToggleCheckBox.IsChecked;
        }

        // Reference: https://stackoverflow.com/questions/34815532/start-storyboard-from-c-sharp-code
    }
}