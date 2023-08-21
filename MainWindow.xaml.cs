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
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace ConventionalIK {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {
        private readonly DispatcherTimer _timer;

        public MainWindow() {
            InitializeComponent();

            // Set up the rotation timer
            _timer = new DispatcherTimer(DispatcherPriority.Normal, Dispatcher) {
                Interval = TimeSpan.FromMilliseconds(30)
            };
            _timer.Tick += TimerOnTick;
            _timer.Start();
        }

        private void TimerOnTick(object sender, EventArgs eventArgs) {
            var rotation = new AxisAngleRotation3D(new Vector3D(0, 1, 0), 1);
            var transform = new RotateTransform3D(rotation, new Point3D(0, 0, 0));
            cube.Transform = transform;
        }
    }
}
