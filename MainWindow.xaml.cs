using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
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
using HelixToolkit.Wpf;
using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using OpenTK.Mathematics;
using MathHelper = OpenTK.Mathematics.MathHelper;
using Quaternion = System.Windows.Media.Media3D.Quaternion;
using Quaterniond = OpenTK.Mathematics.Quaterniond;

namespace ConventionalIK {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {
        public Vector3D GridNormal { get; set; } = new Vector3D(0, 1, 0);

        public string CurrentPose { get; set; } = "";
        
        public Point3DCollection ArcPath { get; private set; }
        public MeshGeometry3D YourMesh { get; private set; }

        public MainWindow() {
            InitializeComponent();
            //DataContext = this;

            //var l = Vector<double>.Build.DenseOfArray(new double[] { 8, 2, 8 });
            //var l = Vector<double>.Build.DenseOfArray(new double[] { 2, 8, 8 });
            //var l = Vector<double>.Build.DenseOfArray(new double[] { 8, 8, 2 });

            //var arc = Kinematics.GetSingleSectionArcParameters(l, 1, 4);
            //var s = arc[0];
            //var kappa = arc[1];
            //var phi = arc[2];
            //var radius = 1 / kappa;

            //// Create a 3D arc path
            //if (kappa == 0)
            //    ArcPath = new Point3DCollection(new Point3D[] { new Point3D(0, 0, 0), new Point3D(0, 0, 4) });
            //else
            //    ComputeArcPoints(kappa, s, phi);
            //    //ArcPath = new Point3DCollection(ComputeArcPoints(new Vector3d(radius, 0, 0), kappa, new Vector3d(0,0,0), s,phi).Select(e => new Point3D(e.X, e.Y, e.Z)));

            //CreatePlaneWithNormal(phi);
            //var visual = new MeshGeometryVisual3D() {
            //    MeshGeometry = CreatePlaneWithNormal(phi*180/Math.PI),
            //    Material = Materials.Blue,
            //};

            //view1.Children.Add(visual);

            //var plane = new Plane3D(new Point3D(0.5, 0.5, 0), new Vector3D(0, 0, 1));
            //var planeVisual = new ModelVisual3D();
            //var group = new Model3DGroup();

            //group.Children.Add()

            //view1.Children.Add(planeVisual);

            //planeVisual.Content = 
        }

        public void ComputeArcPoints(double s, double kappa, double phi) {
            var arcPoints = new List<Vector3d>();
            var radius = 1 / kappa;
            var center = new Vector3d(radius, 0, 0);
            var start = new Vector3d(0, 0, 0);

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
                    double theta = i * angleIncrement;

                    Vector3d point = new Vector3d(radius * (1 - Math.Cos(theta)), 0, radius * Math.Sin(theta));

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

            view1.Children.Add(tube);
            Models.Add(tube);
        }

        public List<Vector3d> ComputeArcPoints(Vector3d center, double kappa, Vector3d startPoint, double arcLength, double phi) {
            var arcPoints = new List<Vector3d>();
            var radius = 1 / kappa;
            var startVector = startPoint - center;

            // Ensure startVector is normalized
            startVector.Normalize();

            //phi = 30;
            // Compute the normal to the arc's plane
            var normal = new Vector3d(0, 1, 0);
            var zAxis = new Vector3d(0, 0, 1);
            var rotation = Quaterniond.FromAxisAngle(zAxis, phi);

            normal = rotation * normal;

            // Rotate the normal by phi around the z-axis
            //phi = 0;
            //normal = Quaterniond.FromAxisAngle(zAxis, phi) * normal;

            int numSegments = 100;  // The number of segments to use to draw the arc. Adjust as needed.
            double angleIncrement = arcLength / radius / numSegments;

            for (int i = 0; i <= numSegments; i++) {
                double alpha = i * angleIncrement;
                Vector3d rotatedVector = Quaterniond.FromAxisAngle(normal, alpha) * startVector;
                arcPoints.Add(center + rotatedVector * radius);
            }


            //return arcPoints;
            return arcPoints.Select(e => rotation * e).ToList();
        }

        private List<ModelVisual3D> Models { get; set; } = new List<ModelVisual3D>();

        public void CreatePlaneWithNormal(double angle) {
            // A unit square in the XZ-plane

            Point3D p1 = new(-0.5, 0, -0.5);
            Point3D p2 = new Point3D(-0.5, 0, 0.5);
            Point3D p3 = new Point3D(0.5, 0, 0.5);
            Point3D p4 = new Point3D(0.5, 0, -0.5);

            // Find rotation that aligns Z-axis (0, 0, 1) to the given normal
            var zAxis = new Vector3D(0, 0, 1);
            var rotation = new Quaternion(zAxis, MathHelper.RadiansToDegrees(angle));

            // Rotate the points
            var rotateTransform = new RotateTransform3D(new QuaternionRotation3D(rotation));
            p1 = rotateTransform.Transform(p1);
            p2 = rotateTransform.Transform(p2);
            p3 = rotateTransform.Transform(p3);
            p4 = rotateTransform.Transform(p4);

            // Scale all points to 400% of original size
            var scaleTransform = new ScaleTransform3D(4, 4, 4);
            p1 = scaleTransform.Transform(p1);
            p2 = scaleTransform.Transform(p2);
            p3 = scaleTransform.Transform(p3);
            p4 = scaleTransform.Transform(p4);


            // Create geometry using MeshBuilder
            var meshBuilder = new MeshBuilder();
            meshBuilder.AddQuad(p1, p2, p3, p4);

            var visual = new MeshGeometryVisual3D() {
                MeshGeometry = meshBuilder.ToMesh(),
                Material = new DiffuseMaterial {
                    Brush = new SolidColorBrush(Colors.Blue) {
                        Opacity = 0.5
                    }
                }
            };

            view1.Children.Add(visual);
            Models.Add(visual);

            var normalVisual = new ArrowVisual3D {
                Point1 = new Point3D(),
                Point2 = rotateTransform.Transform(new Point3D(0, 1, 0)),
                Diameter = 0.08,
                Fill = Brushes.DarkCyan
            };

            view1.Children.Add(normalVisual);
            Models.Add(normalVisual);

        }

        private int Index { get; set; } = 0;
        private void Window_KeyDown(object sender, KeyEventArgs e) {
            if (e.Key == Key.K) {
                foreach (var item in Models) {
                    view1.Children.Remove(item);
                }

                var selection = new List<Vector<double>> {
                    Vector<double>.Build.DenseOfArray(new double[] { 8, 8, 8 }),
                    Vector<double>.Build.DenseOfArray(new double[] { 4, 8, 8 }),
                    Vector<double>.Build.DenseOfArray(new double[] { 8, 4, 8 }),
                    Vector<double>.Build.DenseOfArray(new double[] { 8, 8, 4 }),
                    Vector<double>.Build.DenseOfArray(new double[] { 8, 4, 4 }),
                };

                // increase index, if index is greater than selection count, reset to 0
                Index = Index >= selection.Count - 1 ? 0 : Index + 1;

                // select a element according to idx from selection
                var l = selection[Index];

                var arc = Kinematics.GetSingleSectionArcParameters(l, 1, 4);
                var s = arc[0];
                var kappa = arc[1];
                var phi = arc[2];
                var radius = 1 / kappa;

                // Create a 3D arc path
                ComputeArcPoints(s, kappa, phi);

                CreatePlaneWithNormal(phi);

                var pose = Kinematics.ForwardKinematics(l, 4, 1);

                CurrentPose = pose.ToString();

                var sphere = new SphereVisual3D {
                    Material = new DiffuseMaterial {
                        Brush = new SolidColorBrush(Colors.Blue) {
                            Opacity = 0.5
                        }
                    },
                    Center = new Point3D(pose[0], pose[1], pose[2]),
                    Radius = 1,
                };

                var visual = view1.Children.OfType<SphereVisual3D>().FirstOrDefault();
                
                view1.Children.Remove(visual);

                view1.Children.Add(sphere);


                InvalidateVisual();
            }
        }
    }
}
