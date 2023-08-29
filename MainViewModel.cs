using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Xml;
using CommunityToolkit.Mvvm;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using HelixToolkit.Wpf;
using MathNet.Numerics.LinearAlgebra;
using OpenTK.Mathematics;

namespace ConventionalIK {
    public class Helper {
        public static MatrixTransform3D ConvertToMatrixTransform3D(Matrix<double> matrix) {
            if (matrix.RowCount != 4 || matrix.ColumnCount != 4)
                throw new ArgumentException("Matrix must be 4x4 to convert to MatrixTransform3D");

            return new MatrixTransform3D(new Matrix3D(
                matrix[0, 0], matrix[0, 1], matrix[0, 2], matrix[3, 0],
                matrix[1, 0], matrix[1, 1], matrix[1, 2], matrix[3, 1],
                matrix[2, 0], matrix[2, 1], matrix[2, 2], matrix[3, 2],
                matrix[0, 3], matrix[1, 3], matrix[2, 3], matrix[3, 3]
                                                                                       ));
        }

    }

    public partial class MainViewModel : ObservableObject {
        [ObservableProperty]
        private Point3D currentPose;

        [RelayCommand]
        private void MakeJoints() {
            Objects.Add(new JointModel3D());

            var table = Matrix<double>.Build.DenseOfArray(new double[,] {

            });
            //var T = Kinematics.MakeDHTransformMatrix()
            
        }

        [RelayCommand]
        private void MakeAllJoints() {
            var list = Objects.OfType<JointModel3D>().ToList();

            list.ForEach(e => Objects.Remove(e));

            var theta0 = OpenTK.Mathematics.MathHelper.DegreesToRadians(90);

            var theta1 = OpenTK.Mathematics.MathHelper.DegreesToRadians(30);
            var theta2 = OpenTK.Mathematics.MathHelper.DegreesToRadians(30);
            var d = 2;

            var j0 = new JointModel3D("Joint0");
            var j1 = new JointModel3D("Joint1");
            var j2 = new JointModel3D("Joint2");
            var j3 = new JointModel3D("Joint3");
            var j4 = new JointModel3D("Joint4");
                
            var A01 = Kinematics.MakeDHTransformMatrix(0, 0, 0, Math.PI/2);
            var A12 = Kinematics.MakeDHTransformMatrix(0, d, 0, 0);
            var A23 = Kinematics.MakeDHTransformMatrix(0, 0, d, 0);
            var A34 = Kinematics.MakeDHTransformMatrix(theta2, 0, 0, Math.PI/2);

            var A04 = A01 * A12 * A23 * A34;
            var A03 = A01 * A12 * A23;
            var A02 = A01 * A12;

            var TA04 = Helper.ConvertToMatrixTransform3D(A04);
            var TA03 = Helper.ConvertToMatrixTransform3D(A03);
            var TA02 = Helper.ConvertToMatrixTransform3D(A02);
            var TA01 = Helper.ConvertToMatrixTransform3D(A01);

            var t1 = Matrix3D.Identity;
            t1.Rotate(new System.Windows.Media.Media3D.Quaternion(new Vector3D(0, 0, 1), -30));

            var t2 = t1;
            t2.Rotate(new System.Windows.Media.Media3D.Quaternion(new Vector3D(1, 0, 0), -90));
            var t3 = t2;
            t3.Translate(new Vector3D(d, 0, 0));

            //t2.Children.Add(t1);
            //t2.Children.Add(new TranslateTransform3D(d, 0, 0));
            //t2.Children.Add(new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 30)));

            // Draw a reference arc
            j1.Transform = new MatrixTransform3D(t1);
            //j1.Transform = TA01;
            j2.Transform = new MatrixTransform3D(t2);
            j3.Transform = new MatrixTransform3D(t3);
            //j4.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0,0), -90));


            // add all joints into Objects
            //Objects.Add(j0);
            //Objects.Add(j1);
            Objects.Add(j2);
            Objects.Add(j3);
            //Objects.Add(j4);
        }

        
        public ObservableCollection<Visual3D> Objects { get; set; }

        public MainViewModel() {
            Objects = new ObservableCollection<Visual3D> {
                new SunLight(),
                //new SphereVisual3D {
                //    Center = new Point3D(),
                //    Radius = 0.1,
                //    Fill = Brushes.White
                //},
                //new ArrowVisual3D {
                //    Point1 = new Point3D(),
                //    Point2 = new Point3D(1, 0, 0),
                //    Diameter = 0.03,
                //    Fill = Brushes.Red
                //},
                //new ArrowVisual3D {
                //    Point1 = new Point3D(),
                //    Point2 = new Point3D(0, 1, 0),
                //    Diameter = 0.03,
                //    Fill = Brushes.Green
                //},
                //new ArrowVisual3D {
                //    Point1 = new Point3D(),
                //    Point2 = new Point3D(0, 0, 1),
                //    Diameter = 0.03,
                //    Fill = Brushes.Blue
                //},
                new GridLinesVisual3D {
                    Width = 10,
                    Length = 10,
                    MinorDistance = 0.5,
                    MajorDistance = 1,
                    Thickness = 0.01,
                    Fill = Brushes.DimGray
                },
            };
        }
    }


    public class JointModel3D : ModelVisual3D {
        private TubeVisual3D jointTube;
        private BoundingBoxVisual3D boundingBox;
        private CoordinateSystemVisual3D coordinateSystem;
        private BillboardTextVisual3D jointLabel;

        public JointModel3D(string label = "Joint", double labelOffset = 0.6) {
            // 1. Joint represented by a tube (cylinder-like)
            jointTube = new TubeVisual3D {
                Path = new Point3DCollection { new Point3D(0, 0, -0.2), new Point3D(0, 0, 0.2) },
                Diameter = 0.05,
                Fill = Brushes.Gray
            };

            // 2. Bounding Box
            boundingBox = new BoundingBoxVisual3D {
                BoundingBox = new Rect3D(-0.2, -0.2, -0.6, 0.4, 0.4, 1.2),
                Fill = new SolidColorBrush(Color.FromArgb(25, 200, 200, 200)),  // semi-transparent
                Diameter = 0.01,
            };

            // 3. Coordinate System (Axis Visual)
            coordinateSystem = new CoordinateSystemVisual3D {
                ArrowLengths = 0.4,
                
                //AxisLabels = new[] { "X", "Y", "Z" }
            };

            // 4. Joint Label
            jointLabel = new BillboardTextVisual3D {
                Text = label,
                Position = new Point3D(0, 0, labelOffset),
                Foreground = Brushes.White,
                Background = Brushes.Black,
                Width=4,
                FontSize = 20,
            };

            // Add all the visual components to the main visual
            this.Children.Add(jointTube);
            this.Children.Add(boundingBox);
            this.Children.Add(coordinateSystem);
            this.Children.Add(jointLabel);
        }
    }

}
