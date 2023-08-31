using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
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

            var t = matrix.Transpose();

            return new MatrixTransform3D(new Matrix3D(
                t[0, 0], t[0, 1], t[0, 2], t[0, 3],
                t[1, 0], t[1, 1], t[1, 2], t[1, 3],
                t[2, 0], t[2, 1], t[2, 2], t[2, 3],
                t[3, 0], t[3, 1], t[3, 2], t[3, 3]
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

        public List<Visual3D> JointObjects { get; set; } = new List<Visual3D>();

        [RelayCommand]
        private void MakeAllJoints() {
            JointObjects.ForEach(e => Objects.Remove(e));

            var arc = Kinematics.MakeTransformMatrixF2(Vector<double>.Build.DenseOfArray(new double[] { 4, 4, 2 }), 1, 2);

            //var theta0 = OpenTK.Mathematics.MathHelper.DegreesToRadians(90);
            var s = arc[0];
            var kappa = arc[1];
            var radius = 1 / kappa;
            var phi = arc[2];
            var theta = s / radius;

            Debug.WriteLine($"S: {s}, Kappa: {kappa}, phi: {phi}");

            var theta1 = Math.PI / 2 - theta / 2;
            var theta2 = -theta / 2;
            var d = Math.Sin(theta / 2) / kappa * 2;

            Debug.WriteLine($"Theta: {theta}");

            var j0 = new JointModel3D("Joint0");
            var j1 = new JointModel3D("Joint1");
            var j2 = new JointModel3D("Joint2");
            var j3 = new JointModel3D("Joint3");
            var j4 = new JointModel3D("Joint4");
                
            var A01 = Kinematics.MakeTransformMatrixDH(phi, 0, 0, Math.PI / 2);
            var A12 = Kinematics.MakeTransformMatrixDH(theta1, 0, d, 0);
            var A23 = Kinematics.MakeTransformMatrixDH(theta2, 0, 0, -Math.PI / 2);
            var A34 = Kinematics.MakeTransformMatrixDH(0, 0, 0, 0);

            var A04 = A01 * A12 * A23 * A34;
            var A03 = A01 * A12 * A23;
            var A02 = A01 * A12;

            var TA04 = Helper.ConvertToMatrixTransform3D(A04);
            var TA03 = Helper.ConvertToMatrixTransform3D(A03);
            var TA02 = Helper.ConvertToMatrixTransform3D(A02);
            var TA01 = Helper.ConvertToMatrixTransform3D(A01);

            // Draw a reference arc
            j1.Transform = TA01;
            j2.Transform = TA02;
            j3.Transform = TA03;
            //j4.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0,0), -90));

            // add all joints into Objects
            Objects.Add(j0);
            Objects.Add(j1);
            Objects.Add(j2);
            Objects.Add(j3);
            //Objects.Add(j4);

            JointObjects.Add(j0);
            JointObjects.Add(j1);
            JointObjects.Add(j2);
            JointObjects.Add(j3);

            var arcPoints = new List<Vector3d>();
            Point3DCollection path;
            if (kappa == 0)
                path = new Point3DCollection(new Point3D[] { new Point3D(0, 0, 0), new Point3D(0, 0, s) });
            else {
                int numSegments = 10;  // The number of segments to use to draw the arc. Adjust as needed.
                double angleIncrement = s / radius / numSegments;

                // Compute the normal to the arc's plane which is the cross product of the startVector and the y-axis
                //var normal = new Vector3d(0, -1, 0);
                //phi = OpenTK.Mathematics.MathHelper.DegreesToRadians(30);
                var rotationMatrix = Quaterniond.FromAxisAngle(new Vector3d(0, 0, 1), phi);


                for (int i = 0; i <= numSegments; i++) {
                    double alpha = i * angleIncrement;

                    Vector3d point = new Vector3d(radius * (1 - Math.Cos(alpha)), 0, radius * Math.Sin(alpha));

                    point = Vector3d.Transform(point, rotationMatrix);

                    arcPoints.Add(point);
                }

                path = new Point3DCollection(arcPoints.Select(e1 => new Point3D(e1.X, e1.Y, e1.Z)));
            }

            var tube = new TubeVisual3D {
                Path = path,
                Material = new DiffuseMaterial {
                    Brush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0)),
                },
                BackMaterial = new DiffuseMaterial {
                    Brush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0)),
                },
                Diameter = 0.5,
            };

            Objects.Add(tube);
            JointObjects.Add(tube);

            var sp = new SphereVisual3D {
                Center = new Point3D(0, 0, 8),
                Fill = Brushes.Red,
                Radius = 0.1,
            };
            Objects.Add(sp);
            JointObjects.Add(sp);
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

        public double Opacity { get; set; } = 1.0d;

        public JointModel3D(string label = "Joint", double labelOffset = 0.6) {
            // Adjust opacity with Opacity


            // 1. Joint represented by a tube (cylinder-like)
            jointTube = new TubeVisual3D {
                Path = new Point3DCollection { new Point3D(0, 0, -0.2), new Point3D(0, 0, 0.2) },
                Diameter = 0.05,
                Fill = Brushes.Gray,
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
                Foreground = Brushes.Black,
                //Background = Brushes.Black,
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
