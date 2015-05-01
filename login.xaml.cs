using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Microsoft.Samples.Kinect.BodyBasics
{
    /// <summary>
    /// Interaction logic for login.xaml
    /// </summary>
    public partial class login : Window
    {
        public login()
        {
            InitializeComponent();
        }
        private void login_clicked(object sender, RoutedEventArgs e)
        {
            MainWindow obj = new MainWindow();

            login objmain = new login();

            obj.Show();
            objmain.Hide();
            //App.UserEmail = EmailTextBox.Text;
            App.UserNumber = PhoneTextBox.Text; // CleanUp so it can be automatically be used
            if (!this.Username.Text.Equals(""))
            {
                App.UserName = this.Username.Text;

            }
            else App.UserName = "User";


        }
    }
}
