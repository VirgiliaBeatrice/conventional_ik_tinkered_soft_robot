﻿using System;
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
using HelixToolkit.Wpf;
using MathNet.Numerics.LinearAlgebra;
using OpenTK.Mathematics;
using Quaternion = System.Windows.Media.Media3D.Quaternion;
using Quaterniond = OpenTK.Mathematics.Quaterniond;

namespace ConventionalIK {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {
        public Point3DCollection ArcPath { get; private set; }
        public MeshGeometry3D YourMesh { get; private set; }

        public MainWindow() {
            InitializeComponent();
            DataContext = this;

            var l = Vector<double>.Build.DenseOfArray(new double[] { 4, 2, 4 });

            var arc = Kinematics.GetSingleSectionArcParameters(l, 1, 4);
            var s = arc[0];
            var kappa = arc[1];
            var phi = arc[2];
            var radius = 1 / kappa;

            // Create a 3D arc path
            if (kappa == 0)
                ArcPath = new Point3DCollection(new Point3D[] { new Point3D(0, 0, 0), new Point3D(0, 0, 4) });
            else
                ArcPath = new Point3DCollection(ComputeArcPoints(new Vector3d(radius, 0, 0), kappa, new Vector3d(0,0,0), s,phi).Select(e => new Point3D(e.X, e.Y, e.Z)));

            var visual = new MeshGeometryVisual3D() {
                MeshGeometry = CreatePlaneWithNormal(phi*180/Math.PI),
                Material = Materials.Blue,
            };

            view1.Children.Add(visual);

            //var plane = new Plane3D(new Point3D(0.5, 0.5, 0), new Vector3D(0, 0, 1));
            //var planeVisual = new ModelVisual3D();
            //var group = new Model3DGroup();

            //group.Children.Add()

            //view1.Children.Add(planeVisual);

            //planeVisual.Content = 
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

        //private Point3DCollection CreateTrunk() {
        //    var origin = new Point3D(0, 0, 0);
        //    var trunk = new Point3DCollection();
        //    var l = Vector<double>.Build.DenseOfArray(new double[] { 10, 10, 10 });

        //    var arc = Kinematics.GetSingleSectionArcParameters(l, 1, 5);
        //    var s = arc[0];
        //    var kappa = arc[1];
        //    var phi = arc[2];
        //    var radius = 1 / kappa;

        //    origin.X = radius;

        //    // Create a 3D arc path from center, radius, startPoint, endPoint and step
        //    var arcPath = Create3DArcPath(origin, radius, phi * (180 / Math.PI), 180, 0.1);
        //}

        public List<Vector3d> ComputeArcPoints(Vector3d center, double kappa, Vector3d startPoint, double arcLength, double phi) {
            var arcPoints = new List<Vector3d>();
            var radius = 1 / kappa;
            var startVector = startPoint - center;

            // Ensure startVector is normalized
            startVector.Normalize();

            // Compute the normal to the arc's plane
            var normal = new Vector3d(0, 1, 0);
            var zAxis = new Vector3d(0, 0, 1);

            // Rotate the normal by phi around the z-axis
            phi = 0;
            normal = Quaterniond.FromAxisAngle(zAxis, phi) * normal;

            int numSegments = 100;  // The number of segments to use to draw the arc. Adjust as needed.
            double angleIncrement = arcLength / radius / numSegments;

            for (int i = 0; i <= numSegments; i++) {
                double alpha = i * angleIncrement;
                Vector3d rotatedVector = Quaterniond.FromAxisAngle(normal, alpha) * startVector;
                arcPoints.Add(center + rotatedVector * radius);
            }

            return arcPoints;
            //return arcPoints.Select(e => rotation * e).ToList();
        }

        public MeshGeometry3D CreatePlaneWithNormal(double angle) {
            // A unit square in the XZ-plane

            Point3D p1 = new Point3D(-0.5, 0, -0.5);
            Point3D p2 = new Point3D(-0.5, 0, 0.5);
            Point3D p3 = new Point3D(0.5, 0, 0.5);
            Point3D p4 = new Point3D(0.5, 0, -0.5);

            // Find rotation that aligns Z-axis (0, 0, 1) to the given normal
            var zAxis = new Vector3D(0, 0, 1);
            var rotation = new Quaternion(zAxis, angle);

            // Rotate the points
            var rotateTransform = new RotateTransform3D(new QuaternionRotation3D(rotation));
            p1 = rotateTransform.Transform(p1);
            p2 = rotateTransform.Transform(p2);
            p3 = rotateTransform.Transform(p3);
            p4 = rotateTransform.Transform(p4);

            // Create geometry using MeshBuilder
            var meshBuilder = new MeshBuilder();
            meshBuilder.AddQuad(p1, p2, p3, p4);

            return meshBuilder.ToMesh();
        }

    }
}
