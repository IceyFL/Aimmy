using AimmyWPF.Class;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;

namespace SecondaryWindows
{
    /// <summary>
    /// Interaction logic for NoticeBar.xaml
    /// </summary>
    public partial class NoticeBar : Window
    {
        private static int openNoticeCount = 0;
        private static List<NoticeBar> openNoticeBars = new List<NoticeBar>();
        private const int NoticeHeight = 40; // Height of each notice
        private const int Spacing = 5;       // Spacing between notices
        private const int BaseMargin = 100;  // Base margin from the bottom

        public NoticeBar(string text, string hexColor = "#0f0333")
        {
            InitializeComponent();
            ContentText.Content = text;
            System.Windows.Media.LinearGradientBrush brush = this.Wassuh;
            System.Windows.Media.Color color = (System.Windows.Media.Color)System.Windows.Media.ColorConverter.ConvertFromString(hexColor);
            brush.GradientStops[1].Color = color;
            this.Notice.Width = (text.Length * 6.75) + 25;
            Loaded += OnLoaded;
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            AdjustMargin();
            ShowNotice();
        }

        private async void ShowNotice()
        {
            openNoticeBars.Add(this);
            openNoticeCount++;
            Animator.Fade(Notice);
            await Task.Delay(4000);
            Animator.FadeOut(Notice);
            await Task.Delay(1000);
            CloseNotice();
        }

        private void CloseNotice()
        {
            openNoticeBars.Remove(this);
            openNoticeCount--;
            AdjustMarginsForAll();
            this.Close();
        }

        private void AdjustMargin()
        {
            int bottomMargin = BaseMargin + (openNoticeCount * (NoticeHeight + Spacing));
            if (openNoticeCount >= 2) { bottomMargin = BaseMargin + (2 * (NoticeHeight + Spacing)); }
            Notice.Margin = new Thickness(0, 0, 0, bottomMargin);
        }

        private void AdjustMarginsForAll()
        {
            try
            {
                Application.Current.Dispatcher.InvokeAsync(() =>
                {
                    NoticeBar firstNoticeBar = openNoticeBars[1];
                    firstNoticeBar.Notice.Margin = new Thickness(0, 0, 0, 100);
                    NoticeBar secondsNoticeBar = openNoticeBars[2];
                    secondsNoticeBar.Notice.Margin = new Thickness(0, 0, 0, 145);
                });
            }
            catch { }
        }
    }
}