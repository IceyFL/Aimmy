using System.Windows;
using System.Windows.Controls;
using Microsoft.Win32;

namespace AimmyWPF.UserController
{
    /// <summary>
    /// Interaction logic for ASlider.xaml
    /// </summary>
    public partial class AFilePicker : UserControl
    {
        public AFilePicker(string Text)
        {
            InitializeComponent();
            Title.Content = Text;
            FilePath = "";
        }

        public string FilePath
        {
            get { return (string)GetValue(FilePathProperty); }
            set { SetValue(FilePathProperty, value); }
        }


        // Using a DependencyProperty as the backing store for FilePath. This enables animation, styling, binding, etc...
        public static readonly DependencyProperty FilePathProperty =
            DependencyProperty.Register("FilePath", typeof(string), typeof(AFilePicker), new PropertyMetadata(""));

        public void BrowseButton_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.Filter = "DLL files (*.dll)|*.dll|All files (*.*)|*.*"; // Filter for DLL files only
            openFileDialog.FilterIndex = 1; // Set the default filter to DLL files
            if (openFileDialog.ShowDialog() == true)
            {
                FilePath = openFileDialog.FileName;
                FilePathTextBox.Text = FilePath; // Update the TextBox with the selected file path
            }
        }
    }
}
