using MathNet.Numerics.Integration;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Threading.Tasks.Dataflow;
using System.Windows;
using MathNet.Spatial;
using MathNet.Spatial.Euclidean;
using MathNet.Spatial.Units;
using System.Threading;
using MathNet.Numerics.LinearAlgebra.Double;
using HelixToolkit.Wpf;
using System.Diagnostics;
using OpenTK.Mathematics;
using System.Linq;
using System.Windows.Media.Media3D;
using System.Windows.Media;
using MNVector3D = MathNet.Spatial.Euclidean.Vector3D;
using Point3D = System.Windows.Media.Media3D.Point3D;
using NMMatrix3D = MathNet.Spatial.Euclidean.Matrix3D;

namespace ConventionalIK {
    public class SoftRobot {
        public double L1 { get; set; }
        public double L2 { get; set; }
        public double L3 { get; set; }

        public int NumSegments { get; set; } = 1;
        public double Diameter { get; set; } = 1;

        public double StaticLength { get; set; } = 4;
        public double Max => StaticLength;
        public double Min { get; set; } = 0.5;

        public Vector<double> Lengths { get; set; } = CreateVector.Dense(3, 4d);
        public Vector<double> Arc { get; set; } = CreateVector.Dense(3, 0d);
        public Matrix<double> Q { get; set; } = CreateMatrix.Dense(3, 4, 0d);
        public Matrix<double> EEPose { get; set; } = CreateMatrix.DenseIdentity<double>(4,4);


        public List<JointModel3D> Joints { get; set; } = new List<JointModel3D>();
        public TubeVisual3D RobotObject { get; set; } = null;
        public CombinedManipulator Manipulator = new CombinedManipulator();

        public SoftRobot() => Initialize();

        public void Initialize() {
            Joints.Clear();

            L1 = StaticLength;
            L2 = StaticLength;
            L3 = 2;

            var lens = CreateVector.Dense(new double[] { L1, L2, L3 });

            ComputeEEPose(lens);
            ComputeAllJoints();
            ComputeRobotObject();
        }

        public void Invalidate() {
            var lens = CreateVector.Dense(new double[] { L1, L2, L3 });

            ComputeEEPose(lens);
            InvalidateAllJoints();
            ComputeRobotObject();
        }

        public void Compute() {
            //Matrix<double> goalEE = Manipulator.TargetTransform.Value.ToMathNetMatrix();

            var lens = CreateVector.Dense(new double[] { L1, L2, L3 });

            ComputeEEPose(lens);

            // goalEE for test
            var goalEE = (Matrix<double>)Kinematics.ForwardKinematics(CreateVector.Dense(new double[] { 4, 4, 1 }),Diameter, NumSegments)[2];

            var currL = Kinematics.InverseKinematics(goalEE, EEPose, lens, Diameter, NumSegments);

            L1 = currL[0];
            L2 = currL[1];
            L3 = currL[2];

            Invalidate();
        }

        public void ComputeIK() {
            // Update current pose from current lengths
            var lens = CreateVector.Dense(new double[] { L1, L2, L3 });

            ComputeEEPose(lens);

            // goalEE for test
            var goalEE = (Matrix<double>)Kinematics.ForwardKinematics(CreateVector.Dense(new double[] { 4, 4, 1 }),Diameter, NumSegments)[2];
            var currEE = EEPose;
            var currL = lens;

            int maxIter = 100;
            int iter = 0;
            do {
                var deltaMatEE = goalEE - currEE;
                var deltaVEE = Helper.TransformationMatrixToPoseVector(deltaMatEE) * 0.1;

                deltaVEE[3] = 0;
                deltaVEE[4] = 0;
                deltaVEE[5] = 0;

                var J2 = Kinematics.MakeJacobianF2(currL, Diameter, NumSegments);
                var J1 = Kinematics.MakeJacobianF1(Arc[0], Arc[1], Arc[2]);
                var JDH = Kinematics.MakeJacobianDH(Q);

                var J = JDH * J1 * J2;

                var deltaL = Kinematics.IKStep(deltaVEE, J);

                currL += deltaL;
                ComputeEEPose(currL);

                InvalidateAllJoints();
                Invalidate();
                iter++;
            } while (iter <= maxIter);
        }

        public void InvalidateAllJoints() {
            var s = Arc[0];
            var kappa = Arc[1];
            var phi = Arc[2];
            var theta = s * kappa;

            var j1 = Joints[1];
            var j2 = Joints[2];
            var j3 = Joints[3];

            if (kappa == 0) {
                var T03 = new TranslateTransform3D(0, 0, s);

                j3.Transform = T03;
            }
            else {
                var theta1 = Math.PI / 2 - theta / 2;
                var theta2 = -theta / 2;
                var r = Math.Sin(theta / 2) / kappa * 2;

                var A01 = Kinematics.MakeTransformMatrixDH(phi, 0, 0, Math.PI / 2);
                var A12 = Kinematics.MakeTransformMatrixDH(theta1, 0, r, 0);
                var A23 = Kinematics.MakeTransformMatrixDH(theta2, 0, 0, -Math.PI / 2);

                var A03 = A01 * A12 * A23;
                var A02 = A01 * A12;

                var TA03 = A03.ToMatrixTransform3D();
                var TA02 = A02.ToMatrixTransform3D();
                var TA01 = A01.ToMatrixTransform3D();

                j1.Transform = TA01;
                j2.Transform = TA02;
                j3.Transform = TA03;
            }
        }

        public void ComputeAllJoints() {
            var j0 = new JointModel3D("Joint0");
            var j1 = new JointModel3D("Joint1");
            var j2 = new JointModel3D("Joint2");
            var j3 = new JointModel3D("Joint3");

            var s = Arc[0];
            var kappa = Arc[1];
            var phi = Arc[2];
            var theta = s * kappa;

            if (kappa == 0) {
                var T03 = new TranslateTransform3D(0, 0, s);

                j3.Transform = T03;
            }
            else {
                var theta1 = Math.PI / 2 - theta / 2;
                var theta2 = -theta / 2;
                var r = Math.Sin(theta / 2) / kappa * 2;

                var A01 = Kinematics.MakeTransformMatrixDH(phi, 0, 0, Math.PI / 2);
                var A12 = Kinematics.MakeTransformMatrixDH(theta1, 0, r, 0);
                var A23 = Kinematics.MakeTransformMatrixDH(theta2, 0, 0, -Math.PI / 2);

                var A03 = A01 * A12 * A23;
                var A02 = A01 * A12;

                var TA03 = A03.ToMatrixTransform3D();
                var TA02 = A02.ToMatrixTransform3D();
                var TA01 = A01.ToMatrixTransform3D();

                j1.Transform = TA01;
                j2.Transform = TA02;
                j3.Transform = TA03;

            }

            Joints.Add(j0);
            Joints.Add(j1);
            Joints.Add(j2);
            Joints.Add(j3);

            Manipulator.TargetTransform = j3.Transform;
        }


        private DiffuseMaterial tRed = new DiffuseMaterial {
            Brush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0)),
        };
        private DiffuseMaterial tBlue = new DiffuseMaterial {
            Brush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0)),
        };
        public void ComputeRobotObject() {
            var arcPoints = new List<Vector3d>();
            Point3DCollection path;

            var s = Arc[0];
            var kappa = Arc[1];
            var phi = Arc[2];
            var radius = 1 / kappa;

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
                Material = tRed,
                BackMaterial = tRed,
                Diameter = Diameter,
            };

            RobotObject = tube;
        }

        public void ComputeEEPose(Vector<double> lengths) {
            Lengths = lengths;
            //Debug.WriteLine($"Len: {lengths}");

            var fkResults = Kinematics.ForwardKinematics(lengths, Diameter, NumSegments);

            //Debug.WriteLine(Arc);

            var q = (Vector<double>)fkResults[1];

            Q = CreateMatrix.DenseOfArray(new double[,] {
                { q[0], q[1], q[2], q[3] },
                { q[4], q[5], q[6], q[7] },
                { q[8], q[9], q[10], q[11] },
            });

            Arc = (Vector<double>)fkResults[0];
            EEPose = (Matrix<double>)fkResults[2];
        }
    }

    public class Kinematics {
        public Kinematics() {
        }

        // Forward Kinematics
        // Kappa: Curvature
        // Phi: angle
        // S: Length
        public static Vector<double> MakeTransformMatrixF2(Vector<double> l, int numOfSegments, double diameter) {
            var l0 = l[0];
            var l1 = l[1];
            var l2 = l[2];

            // number of segments
            var n = numOfSegments;

            // diameter of the trunk frame
            var d = diameter;

            // root of sum of all lengths
            var f0 = Math.Sqrt(l0 * l0 + l1 * l1 + l2 * l2 - l0 * l1 - l0 * l2 - l1 * l2);

            var s = n * d * (l0 + l1 + l2) / f0 * Math.Asin(f0 / (3 * n * d));
            var kappa = 2 * f0 / (d * (l0 + l1 + l2));

            if (kappa == 0) {
                return Vector<double>.Build.DenseOfArray(new double[] { (l0 + l1 + l2) / 3, kappa, double.NaN });
            }
            else {
                double phi = 0;

                if (Math.Sign(l2 - l1) == 1) {
                    phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2)) + Math.PI;
                }
                else if (Math.Sign(l2 - l1) == 0) {
                    phi = Math.Sign(l1 - l0) == 1 ? 90 * Math.PI / 180 : 270 * Math.PI / 180;
                }
                else if (Math.Sign(l2 - l1) == -1) {
                    phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2));
                }

                return Vector<double>.Build.DenseOfArray(new double[] { s, kappa, phi });
            }
        }

        public static Matrix<double> MakeJacobianF2(Vector<double> lengths, double diameter, int numSegments) {
            double[] CalculateS(Vector<double> lengths, double d, int n) {
                double l1, l2, l3;
                l1 = lengths[0];
                l2 = lengths[1];
                l3 = lengths[2];

                // if l0=l1=l2, then B=0

                var A = l1 + l2 + l3;
                var B = Math.Sqrt(l1 * l1 + l2 * l2 + l3 * l3 - l1 * l2 - l1 * l3 - l2 * l3);

                var C1 = 2 * l1 - l2 - l3;
                var C2 = 2 * l2 - l1 - l3;
                var C3 = 2 * l3 - l1 - l2;

                var theta = B / (3 * n * d);

                var partial_s_l1 = n * d * (2 * B * B + A * C1) / (2 * B * B * B) * Math.Asin(theta) + n * d * A * C1 / (2 * B * B * Math.Sqrt(9 * n * n * d * d - B * B));
                var partial_s_l2 = n * d * (2 * B * B + A * C2) / (2 * B * B * B) * Math.Asin(theta) + n * d * A * C2 / (2 * B * B * Math.Sqrt(9 * n * n * d * d - B * B));
                var partial_s_l3 = n * d * (2 * B * B + A * C3) / (2 * B * B * B) * Math.Asin(theta) + n * d * A * C3 / (2 * B * B * Math.Sqrt(9 * n * n * d * d - B * B));
                return new double[] { partial_s_l1, partial_s_l2, partial_s_l3 };
            }

            double[] partial_s = CalculateS(lengths, diameter, numSegments);

            // k: Double checked
            double[] CalculateK(Vector<double> lengths, double diameter, double numSegments) {
                double l1, l2, l3;
                l1 = lengths[0];
                l2 = lengths[1];
                l3 = lengths[2];

                var d = diameter;
                var n = numSegments;

                var A = l1 * l1 + l2 * l2 + l3 * l3 - l1 * l2 - l1 * l3 - l2 * l3;
                var B = l1 + l2 + l3;

                var partial_a_l1 = 2 * l1 - l2 - l3;
                var partial_a_l2 = 2 * l2 - l1 - l3;
                var partial_a_l3 = 2 * l3 - l1 - l2;

                // Singularity: A == 0
                var partial_k_a = 1 / (d * B * Math.Sqrt(A));
                var partial_k_b = -2 * Math.Sqrt(A) / (d * B * B);

                return new double[] {
                    partial_a_l1 / (d * B * Math.Sqrt(A)) + partial_k_b,
                    partial_a_l2 / (d * B * Math.Sqrt(A)) + partial_k_b,
                    partial_a_l3 / (d * B * Math.Sqrt(A)) + partial_k_b,
                };
            }

            double[] partial_k = CalculateK(lengths, diameter, numSegments);

            // phi
            double[] CalculatePhi(Vector<double> lengths) {
                double l1, l2, l3;
                l1 = lengths[0];
                l2 = lengths[1];
                l3 = lengths[2];

                var A = l3 + l2 - 2 * l1;
                var B = l2 - l3;

                var partial_phi_l0 = -6 * Math.Sqrt(3) * B / (3 * B * B + A * A);
                var partial_phi_l1 = (Math.Sqrt(3) * (3 * B - A)) / (3 * B * B + A * A);
                var partial_phi_l2 = (Math.Sqrt(3) * (3 * B + A)) / (3 * B * B + A * A);

                return new double[] { partial_phi_l0, partial_phi_l1, partial_phi_l2 };
            }

            double[] partial_phi = CalculatePhi(lengths);

            return CreateMatrix.DenseOfRowArrays(new double[][] { partial_s, partial_k, partial_phi });
        }


        public static Matrix<double> MakeJacobianF1(double s, double kappa, double phi) {
            var e11 = 0;
            var e12 = 0;
            var e13 = 1;
            var e21 = -kappa / 2;
            var e22 = -s / 2;
            var e23 = 0;
            var e31 = -kappa / 2;
            var e32 = -s / 2;
            var e33 = 0;
            var e41 = 2 * Math.Cos(s * kappa / 2);
            var e42 = 2 * Math.Cos(s * kappa / 2) * s / kappa;
            var e43 = 0;

            return CreateMatrix.DenseOfArray(new double[,] {
                { 0, 0, 1},
                {-kappa/2, -s/2,0},
                {-kappa/2, -s/2,0 }
            });
        }

        public static Matrix<double> MakeJacobianDH(Matrix<double> table) {
            var TDH = MakeTransformMatrixDH(table);
            var Oee = TDH.Column(3).SubVector(0, 4);
            var jointTypes = new int[] { 0, 0, 0 };

            Vector<double> MakeJacobianDHForRevoluteJoint(Vector<double> Zprev, Vector<double> Oprev) {
                var JiV = Zprev.CrossProduct(Oee - Oprev);
                var JiW = Zprev;

                // return JiV appends JiW
                return Vector<double>.Build.DenseOfArray(new double[] { JiV[0], JiV[1], JiV[2], JiW[0], JiW[1], JiW[2] });
            }

            Vector<double> MakeDHForPrismaticJoint(Vector<double> Zprev) {
                var JiV = Zprev;
                var JiW = CreateVector.Dense(3, 0d);

                return Vector<double>.Build.DenseOfArray(new double[] { JiV[0], JiV[1], JiV[2], JiW[0], JiW[1], JiW[2] });
            }

            var Z0 = CreateVector.Dense(new double[] { 0, 0, 1, 1 });
            var O0 = CreateVector.Dense(4, 0d);

            Vector<double> Zprev = Z0;
            Vector<double> Oprev = O0;

            Matrix<double> JDH = CreateMatrix.Dense<double>(6, table.RowCount);

            for(int i = 0; i<table.RowCount; i++) {

                //var type = table[i, 0];
                var theta = table[i, 0];
                var d = table[i, 1];
                var a = table[i, 2];
                var alpha = table[i, 3];

                var Ti = MakeTransformMatrixDH(theta, d, a, alpha);

                Vector<double> Ji;

                if (jointTypes[i] == 0)
                    Ji = MakeJacobianDHForRevoluteJoint(Zprev, Oprev);
                else
                    Ji = MakeDHForPrismaticJoint(Zprev);

                Zprev = Ti * Zprev;
                Oprev = Ti.Column(3).SubVector(0, 4);

                JDH.SetColumn(i, Ji);
            }

            return JDH;
        }

        public static object[] ForwardKinematics(Vector<double> lengths, double diameter, int numOfSegments) {
            double l0, l1, l2;
            l0 = lengths[0];
            l1 = lengths[1];
            l2 = lengths[2];

            var d = diameter;
            var n = numOfSegments;

            var A = l0 + l1 + l2;
            var B = l0 * l0 + l1 * l1 + l2 * l2 - l0 * l1 - l0 * l2 - l1 * l2;

            var f1 = Math.Sqrt(B) / (3 * n * d);
            // clamp f1 in [-1,1]
            if (f1 > 1)
                f1 = 1;
            else if (f1 < -1)
                f1 = -1;


            double kappa = 2 * Math.Sqrt(B) / d / A;
            double s = kappa == 0? l0 : n * d * A / (Math.Sqrt(B)) * Math.Asin(f1);
            double phi = 0d;

            if (l1 > l2) {
                phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2));
            }
            else if (l1 < l2) {
                phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2)) + Math.PI;
            }
            else if (l1 == l2) {
                if (l0 > l1) {
                    phi = 3 * Math.PI / 2;
                }
                else if (l0 < l1) {
                    phi = Math.PI / 2;
                }
            }
            else
                phi = 0d;

            var theta0 = phi;
            var theta1 = (Math.PI - s * kappa) / 2;
            var theta2 = -s * kappa / 2;
            var r = kappa == 0 ? s : 2 * Math.Sin(s * kappa / 2) / kappa;
            
            //Debug.WriteLine($"S: {s}, Kappa: {kappa}, phi: {phi}, r: {r}");


            var q = CreateMatrix.DenseOfArray(new double[,] {
                { theta0,0,0, Math.PI / 2 },
                { theta1,0, r, 0 },
                { theta2,0,0,-Math.PI / 2 }
            });

            var Vq = CreateVector.DenseOfArray(new double[] {
                theta0,0,0, Math.PI / 2,
                theta1, 0, r, 0,
                theta2,0,0,-Math.PI / 2
            });

            var TransformationEE = MakeTransformMatrixDH(q);
            //Debug.WriteLine($"TransformationEE: {TransformationEE}");

            var Ree = TransformationEE.SubMatrix(0, 3, 0, 3);
            var Tee = TransformationEE.SubMatrix(0, 3, 3, 1);

            return new object[] {
                CreateVector.DenseOfArray(new double[] { s, kappa, phi}),
                Vq,
                TransformationEE
            };
        }

        public static Vector<double> IKStep(Vector<double> deltaEE, Matrix<double> J) {
            var Jdaggar = J.PseudoInverse();
            var deltaL = Jdaggar * deltaEE;

            return deltaL;
        }

        public static Vector<double> InverseKinematics(Matrix<double> goalEE, Matrix<double> currEE, Vector<double> currL, double diameter, int numSegments) {
            var fkResults = ForwardKinematics(currL, diameter, numSegments);

            var arc = (Vector<double>)fkResults[0];
            var q = (Vector<double>)fkResults[1];
            //var currEE = (Matrix<double>)fkResults[2];
            var Q = CreateMatrix.DenseOfArray(new double[,] {
                { q[0], q[1], q[2], q[3] },
                { q[4], q[5], q[6], q[7] },
                { q[8], q[9], q[10], q[11] },
            });

            // step1
            int maxIter = 100;
            int iter = 0;

            Matrix<double> deltaEE;
            Vector<double> deltaVEE;
            
            Vector<double> TransformationMatrixToPoseVector(Matrix<double> matrix) {
                var R = matrix.SubMatrix(0, 3, 0, 3);
                var T = new double[] { matrix[0, 3], matrix[1, 3], matrix[2, 3] };

                // Rotation matrix to Euler angles step by step

                var (yaw, pitch, roll) = Helper.RotationMatrixToEulerAngles(R);

                return CreateVector.DenseOfArray(new double[] {
                     matrix[0, 3], matrix[1, 3], matrix[2, 3],
                     yaw,pitch,roll
                });
            }

            do {
                deltaEE = goalEE - currEE;

                deltaVEE = TransformationMatrixToPoseVector(deltaEE);

                var a = 0.1;

                // step2
                var vEE = a * deltaVEE;

                // step3
                var JF2 = MakeJacobianF2(currL, diameter, numSegments);
                var JF1 = MakeJacobianF1(arc[0], arc[1], arc[2]);
                var JDH = MakeJacobianDH(Q);

                var J = JDH * JF1 * JF2;

                // step4
                var Jdaggar = J.PseudoInverse();

                // step5
                var vL = Jdaggar * vEE;

                currL += vL;
                fkResults = ForwardKinematics(currL, diameter, numSegments);
                currEE = (Matrix<double>)fkResults[2];

                iter++;

            } while (iter <= maxIter | deltaEE.FrobeniusNorm() < 0.0001);

            return currL;
        }

        public IEnumerable<Vector<double>> IK_JacobianTranspose(Matrix<double> desiredE, Vector<double> currL, double diameter, int numOfSegments) {

            var fkResults = ForwardKinematics(currL, diameter, numOfSegments);

            var arc = (Vector<double>)fkResults[0];
            var q = (Vector<double>)fkResults[1];

            var JF2 = MakeJacobianF2(currL, diameter, numOfSegments);
            var JF1 = MakeJacobianF1(arc[0], arc[1], arc[2]);
            var JDH = MakeJacobianDH(q.ToRowMatrix().Resize(3,4));

            var J = JDH * JF1 * JF2;
            var JT = J.Transpose();

            double alpha = 0.1;
            double thereshold = 0.01;

            Matrix<double> currE = (Matrix<double>)fkResults[2];
            Vector<double> nextL = Vector<double>.Build.Dense(4, 0.0);

            while ((desiredE - currE).FrobeniusNorm() < thereshold) {
                currE = (Matrix<double>)ForwardKinematics(currL, diameter, numOfSegments)[2];
                nextL = currL + Vector<double>.Build.DenseOfArray((alpha * JT * (desiredE - currE)).ToColumnMajorArray());

                yield return nextL;
            }

            yield break;
        }


        public static Matrix<double> MakeTransformMatrixDH(double theta, double d, double r, double alpha) {
            var s_theta = Math.Sin(theta);
            var c_theta = Math.Cos(theta);
            var s_alpha = Math.Sin(alpha);
            var c_alpha = Math.Cos(alpha);

            var Z = CreateMatrix.DenseOfArray(new double[,] {
                {c_theta, -s_theta, 0, 0 },
                { s_theta, c_theta, 0, 0 },
                { 0, 0, 1, d },
                {0, 0, 0, 1}
            });
            var X = CreateMatrix.DenseOfArray(new double[,] {
                {1,0,0,r },
                {0, c_alpha, -s_alpha, 0},
                {0, s_alpha, c_alpha, 0},
                {0, 0, 0, 1 }
            });

            return Z * X;
        }

        public static Matrix<double> MakeTransformMatrixDH(Matrix<double> table) {
            Matrix<double> T = CreateMatrix.DenseIdentity<double>(4, 4);

            for (int i = 0; i < table.RowCount; i++) {
                var Ti = MakeTransformMatrixDH(table[i, 0], table[i, 1], table[i, 2], table[i, 3]);

                T *= Ti;
            }

            return T;
        }
    }
}