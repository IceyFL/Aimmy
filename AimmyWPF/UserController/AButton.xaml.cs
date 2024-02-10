using System.Windows.Controls;

namespace AimmyWPF.UserController
{
    /// <summary>
    /// Interaction logic for AButton.xaml
    /// </summary>
    public partial class AButton : UserControl
    {
        public AButton(string Text, string Info)
        {
            InitializeComponent();
            Title.Content = Text;

        }
    }
}