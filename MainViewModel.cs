using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Xml;
using CommunityToolkit.Mvvm;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using HelixToolkit.Wpf;
using MathNet.Numerics.LinearAlgebra;
using MNVector3D = MathNet.Spatial.Euclidean.Vector3D;
using OpenTK.Mathematics;
using System.Xml.Serialization;

namespace ConventionalIK {
    public static class Helper {
        public static Vector<double> TransformationMatrixToPoseVector(Matrix<double> matrix) {
            var R = matrix.SubMatrix(0, 3, 0, 3);
            var T = new double[] { matrix[0, 3], matrix[1, 3], matrix[2, 3] };

            // Rotation matrix to Euler angles step by step

            var (yaw, pitch, roll) = Helper.RotationMatrixToEulerAngles(R);

            return CreateVector.DenseOfArray(new double[] {
                     matrix[0, 3], matrix[1, 3], matrix[2, 3],
                     yaw,pitch,roll
                });
        }


        public static Matrix<double> EulerAnglesToRotationMatrix(double yaw, double pitch, double roll) {
            // Convert degrees to radians
            yaw = yaw * (Math.PI / 180.0);
            pitch = pitch * (Math.PI / 180.0);
            roll = roll * (Math.PI / 180.0);

            // Z-axis (Yaw) rotation matrix
            var Rz = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { Math.Cos(yaw), -Math.Sin(yaw), 0 },
                { Math.Sin(yaw), Math.Cos(yaw),  0 },
                { 0,             0,              1 }
            });

            // Y-axis (Pitch) rotation matrix
            var Ry = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { Math.Cos(pitch),  0, Math.Sin(pitch) },
                { 0,               1, 0               },
                { -Math.Sin(pitch), 0, Math.Cos(pitch) }
            });

            // X-axis (Roll) rotation matrix
            var Rx = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                { 1, 0,              0               },
                { 0, Math.Cos(roll), -Math.Sin(roll) },
                { 0, Math.Sin(roll), Math.Cos(roll)  }
            });

            // Combined rotation matrix
            return Rz * Ry * Rx;
        }

        public static (double yaw, double pitch, double roll) RotationMatrixToEulerAngles(Matrix<double> m) {
            if (m.RowCount != 3 || m.ColumnCount != 3)
                throw new ArgumentException("Expected a 3x3 matrix.");

            double yaw, pitch, roll;

            if (Math.Abs(m[1, 0] - 1) < 1e-6 || Math.Abs(m[1, 0] + 1) < 1e-6) {
                // Gimbal lock cases
                yaw = 0;  // Yaw is set to 0
                pitch = Math.Atan2(-m[1, 0], m[0, 0]);
                roll = Math.Atan2(-m[2, 1], m[1, 1]);
            }
            else {
                yaw = Math.Atan2(m[0, 1], m[0, 0]);
                pitch = Math.Asin(-m[0, 2]);
                roll = Math.Atan2(m[1, 2], m[2, 2]);
            }

            // Convert radians to degrees
            yaw *= (180.0 / Math.PI);
            pitch *= (180.0 / Math.PI);
            roll *= (180.0 / Math.PI);

            return (yaw, pitch, roll);
        }

        public static MatrixTransform3D ToMatrixTransform3D(this Matrix<double> matrix) {
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

        public static Matrix<double> ToMathNetMatrix(this Matrix3D mat) {
            return CreateMatrix.DenseOfArray(new double[,] {
                { mat.M11, mat.M12, mat.M13, mat.M14, },
                { mat.M21, mat.M22, mat.M23, mat.M24, },
                { mat.M31, mat.M32, mat.M33, mat.M34, },
                { mat.OffsetX, mat.OffsetY, mat.OffsetZ, mat.M44, } }).Transpose();
        }

        public static Vector<double> ToVector4(this MNVector3D vector) {
            return CreateVector.Dense(new double[] { vector.X, vector.Y, vector.Z, 1 });
        }

        public static Vector<double> CrossProduct(this Vector<double> it, Vector<double> other) {
            double x = it[1] * other[2] - it[2] * other[1];
            double y = it[2] * other[0] - it[0] * other[2];
            double z = it[0] * other[1] - it[1] * other[0];

            return CreateVector.Dense(new double[] { x, y, z});
        }
    }

    public partial class MainViewModel : ObservableObject {
        [ObservableProperty]
        private Point3D currentPose;
        [ObservableProperty]
        private double l1 = 4;
        [ObservableProperty]
        private double l2 = 4;
        [ObservableProperty]
        private double l3 = 4;

        [ObservableProperty]
        private double min = 0.5;
        [ObservableProperty]
        private double max = 4;

        partial void OnL1Changed(double value) {
            Robot.L1 = value;

            Robot.Invalidate();
        }

        partial void OnL2Changed(double value) {
            Robot.L2 = value;

            Robot.Invalidate();
        }
        partial void OnL3Changed(double value) {
            Robot.L3 = value;

            Robot.Invalidate();
        }

        [RelayCommand]
        private void ComputeIK() {
            //Robot.Joints.ForEach(e => Objects.Remove(e));
            
            ////Objects.Remove(Robot.Manipulator);
            //Objects.Remove(Robot.RobotObject);

            //// Draw robot
            //Objects.Add(Robot.RobotObject);
            ////Objects.Add(Robot.Manipulator);

            //Robot.Joints.ForEach(Objects.Add);

            Robot.ComputeIK();
            //var T = Kinematics.MakeDHTransformMatrix()

        }

        [RelayCommand]
        private void Initialize() {
            Robot.Joints.ForEach(e => Objects.Remove(e));
            Objects.Remove(Robot.RobotObject);
            Objects.Remove(Robot.TargetObject);

            Objects.Add(Robot.RobotObject);
            Objects.Add(Robot.TargetObject);
            Robot.Joints.ForEach(Objects.Add);

        }

        [RelayCommand]
        private void ComputeJoints() {
            Robot.ComputeEEPose();
            Robot.ComputeJoints();
        }

        

        public List<Visual3D> JointObjects { get; set; } = new List<Visual3D>();

        public SoftRobot Robot { get; set; } = new SoftRobot();

        [RelayCommand]
        private void MakeSmallDisplacement() {

        }

        [RelayCommand]
        private void MakeAllJoints() {
            Robot.Joints.ForEach(e => Objects.Remove(e));
            Objects.Remove(Robot.RobotObject);

            // Draw robot
            Objects.Add(Robot.RobotObject);

            Robot.Joints.ForEach(Objects.Add);

            ////Objects.Clear();

            //var lengths = Vector<double>.Build.DenseOfArray(new double[] { L1,L2,L3 });

            //Robot.ComputeEEPose(lengths);

            //var arc = Robot.Arc;

            ////var arc = Kinematics.MakeTransformMatrixF2(, 1, 2);

            ////var theta0 = OpenTK.Mathematics.MathHelper.DegreesToRadians(90);
            //var s = arc[0];
            //var kappa = arc[1];
            //var radius = 1 / kappa;
            //var phi = arc[2];
            //var theta = s / radius;


            //var theta1 = Math.PI / 2 - theta / 2;
            //var theta2 = -theta / 2;
            //var r = Math.Sin(theta / 2) / kappa * 2;

            ////Debug.WriteLine($"S: {s}, Kappa: {kappa}, phi: {phi}, r: {r}");
            ////Debug.WriteLine($"Theta: {theta}");

            //var j0 = new JointModel3D("Joint0");
            //var j1 = new JointModel3D("Joint1");
            //var j2 = new JointModel3D("Joint2");
            //var j3 = new JointModel3D("Joint3");
            //var j4 = new JointModel3D("Joint4");
                
            //var A01 = Kinematics.MakeTransformMatrixDH(phi, 0, 0, Math.PI / 2);
            //var A12 = Kinematics.MakeTransformMatrixDH(theta1, 0, r, 0);
            //var A23 = Kinematics.MakeTransformMatrixDH(theta2, 0, 0, -Math.PI / 2);
            //var A34 = Kinematics.MakeTransformMatrixDH(0, 0, 0, 0);

            //var A04 = A01 * A12 * A23 * A34;
            //var A03 = A01 * A12 * A23;
            //var A02 = A01 * A12;

            ////Debug.WriteLine($"T: {A03}");


            //var TA04 = Helper.ConvertToMatrixTransform3D(A04);
            //var TA03 = Helper.ConvertToMatrixTransform3D(A03);
            //var TA02 = Helper.ConvertToMatrixTransform3D(A02);
            //var TA01 = Helper.ConvertToMatrixTransform3D(A01);

            //// Draw a reference arc
            //j1.Transform = TA01;
            //j2.Transform = TA02;
            //j3.Transform = TA03;
            ////j4.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0,0), -90));

            //// add all joints into Objects
            //Objects.Add(j0);
            ////Objects.Add(j1);
            ////Objects.Add(j2);
            //Objects.Add(j3);
            ////Objects.Add(j4);

            //JointObjects.Add(j0);
            //JointObjects.Add(j1);
            //JointObjects.Add(j2);
            //JointObjects.Add(j3);

            //var arcPoints = new List<Vector3d>();
            //Point3DCollection path;
            //if (kappa == 0)
            //    path = new Point3DCollection(new Point3D[] { new Point3D(0, 0, 0), new Point3D(0, 0, s) });
            //else {
            //    int numSegments = 10;  // The number of segments to use to draw the arc. Adjust as needed.
            //    double angleIncrement = s / radius / numSegments;

            //    // Compute the normal to the arc's plane which is the cross product of the startVector and the y-axis
            //    //var normal = new Vector3d(0, -1, 0);
            //    //phi = OpenTK.Mathematics.MathHelper.DegreesToRadians(30);
            //    var rotationMatrix = Quaterniond.FromAxisAngle(new Vector3d(0, 0, 1), phi);


            //    for (int i = 0; i <= numSegments; i++) {
            //        double alpha = i * angleIncrement;

            //        Vector3d point = new Vector3d(radius * (1 - Math.Cos(alpha)), 0, radius * Math.Sin(alpha));

            //        point = Vector3d.Transform(point, rotationMatrix);

            //        arcPoints.Add(point);
            //    }

            //    path = new Point3DCollection(arcPoints.Select(e1 => new Point3D(e1.X, e1.Y, e1.Z)));
            //}

            //var tube = new TubeVisual3D {
            //    Path = path,
            //    Material = new DiffuseMaterial {
            //        Brush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0)),
            //    },
            //    BackMaterial = new DiffuseMaterial {
            //        Brush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0)),
            //    },
            //    Diameter = 0.5,
            //};

            //Objects.Add(tube);
            //JointObjects.Add(tube);

            //var eeController = new JointModel3D("EE") {

            //    Transform = Helper.ConvertToMatrixTransform3D(Robot.EEPose),
            //};
            //Objects.Add(eeController);
            //JointObjects.Add(eeController);

            //var manipulator = new CombinedManipulator() {
            //};

            //BindingOperations.SetBinding(
            //    manipulator,
            //    CombinedManipulator.TargetTransformProperty,
            //    new Binding("Transform") { Source = eeController });

            //Objects.Add(manipulator);
            //JointObjects.Add(manipulator);
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
        private CombinedManipulator combinedManipulator;

        public new Transform3D Transform {
            get { return base.Transform; }
            set {
                base.Transform = value;

                if (jointTube != null) jointTube.Transform = value;
                if (boundingBox != null) boundingBox.Transform = value;
                if (coordinateSystem != null) coordinateSystem.Transform = value;
                if (jointLabel != null) jointLabel.Transform = value;
            }
        }

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
            //combinedManipulator = new CombinedManipulator();

            //BindingOperations.SetBinding(
            //   combinedManipulator,
            //   CombinedManipulator.TargetTransformProperty,
            //   new Binding("Transform") { Source = this });

            // Add all the visual components to the main visual
            this.Children.Add(jointTube);
            this.Children.Add(boundingBox);
            this.Children.Add(coordinateSystem);
            this.Children.Add(jointLabel);
            //Children.Add(combinedManipulator);
        }
    }

}
