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
        public Point3DCollection ArcPath { get; private set; }

        public MainWindow() {
            InitializeComponent();
            DataContext = this;

            // Create a 3D arc path
            ArcPath = new Point3DCollection(Create3DArcPath(new Point3D(0, 0, 0), 2, 45, 135, 0.1));
        }

        private List<Point3D> Create3DArcPath(Point3D center, double radius, double startAngle, double endAngle, double step) {
            List<Point3D> path = new List<Point3D>();
            for (double angle = startAngle; angle <= endAngle; angle += step) {
                double radian = angle * (Math.PI / 180);
                double x = center.X + radius * Math.Cos(radian);
                double y = center.Y + radius * Math.Sin(radian);
                path.Add(new Point3D(x, y, center.Z));
            }
            return path;
        }
    }
}
