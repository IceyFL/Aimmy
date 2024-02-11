using System.Collections.Generic;
using System.Windows.Controls;

namespace AimmyWPF.UserController
{
    /// <summary>
    /// Interaction logic for ASlider.xaml
    /// </summary>
    public partial class ADropdown : UserControl
    {
        public ADropdown(string Text, List<string> options)
        {
            InitializeComponent();
            Title.Content = Text;
            foreach (string option in options)
            {
                ComboBoxItem item = new ComboBoxItem();
                item.Content = option;
                DropdownBox.Items.Add(item);
            }
        }
    }
}