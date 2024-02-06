using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;

namespace AimmyWPF.UserController
{
    public partial class SectionPanel : UserControl
    {
        // Dependency properties
        public static readonly DependencyProperty TitleProperty = DependencyProperty.Register("Title", typeof(string), typeof(SectionPanel));
        public static readonly DependencyProperty ContentProperty = DependencyProperty.Register("Content", typeof(object), typeof(SectionPanel));

        // Public properties
        public string Title
        {
            get { return (string)GetValue(TitleProperty); }
            set { SetValue(TitleProperty, value); }
        }

        public object Content
        {
            get { return GetValue(ContentProperty); }
            set { SetValue(ContentProperty, value); }
        }

        // Children property to hold multiple child controls
        public List<UIElement> Children { get; } = new List<UIElement>();

        public SectionPanel()
        {
            InitializeComponent();
            DataContext = this;
            Loaded += SectionPanel_Loaded;
        }

        // Add child controls to the ChildrenPanel StackPanel
        private void SectionPanel_Loaded(object sender, RoutedEventArgs e)
        {
            foreach (var child in Children)
            {
                ChildrenPanel.Children.Add(child);
            }
        }
    }
}
