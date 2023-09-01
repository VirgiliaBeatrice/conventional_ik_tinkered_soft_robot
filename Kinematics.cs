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

namespace ConventionalIK {
    public class SoftRobot {
        public int NumSegments { get; set; } = 1;
        public double Diameter { get; set; } = 1;

        public double Max { get; set; } = 4;
        public double Min { get; set; } = 0.5;

        public Vector<double> Lengths { get; set; } = Vector<double>.Build.Dense(3, 4d);
        public Vector<double> Arc { get; set; } = Vector<double>.Build.Dense(3, 0d);
        public Vector<double> Q { get; set; } = Vector<double>.Build.Dense(3*4, 0d);
        public Matrix<double> EEPose { get; set; } = CreateMatrix.DenseIdentity<double>(4,4);

        public Vector3D EEPosePosition => new Vector3D(EEPose[0,3], EEPose[2, 3], EEPose[3, 3]);
        public Matrix<double> EEPoseRotation => EEPose.SubMatrix(0, 3, 0, 3);

        public SoftRobot() { }

        public void ComputeEEPose(Vector<double> lengths) {
            Lengths = lengths;
            Debug.WriteLine($"Len: {lengths}");

            var fkResults = Kinematics.ForwardKinematics(lengths, Diameter, NumSegments);

            Debug.WriteLine(Arc);

            Arc = (Vector<double>)fkResults[0];
            Q = (Vector<double>)fkResults[1];
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

        //public static Vector<double> GetLengths(Vector<double> arcParameters, int numOfSegments, double diameter) {
        //    var s = arcParameters[0];
        //    var kappa = arcParameters[1];
        //    var phi = arcParameters[2];

        //    var d = diameter;
        //    var n = numOfSegments;

        //    var l0 = 2 * n * Math.Sin(kappa * s / (2 * n)) * (1 / kappa - d * Math.Sin(phi));
        //    var l1 = 2 * n * Math.Sin(kappa * s / (2 * n)) * (1 / kappa + d * Math.Sin(phi + Math.PI / 3));
        //    var l2 = 2 * n * Math.Sin(kappa * s / (2 * n)) * (1 / kappa - d * Math.Cos(phi + Math.PI / 6));

        //    if (kappa == 0) {
        //        l0 = s;
        //        l1 = s;
        //        l2 = s;
        //    }

        //    return Vector<double>.Build.DenseOfArray(new double[] { l0, l1, l2 });
        //}

        public static Matrix<double> MakeJacobianF2(Vector<double> lengths, double diameter, int numOfSegments) {
            double[] CalculateS(Vector<double> lengths, double d, int n) {
                double l0, l1, l2;
                l0 = lengths[0];
                l1 = lengths[1];
                l2 = lengths[2];

                var A = l0 + l1 + l2;
                var B = l0 * l0 + l1 * l1 + l2 * l2 - l0 * l1 - l0 * l2 - l1 * l2;
                var u = n * d * A / Math.Sqrt(B);
                var v = Math.Asin(Math.Sqrt(B) / (3 * n * d));

                var partial_a_l0 = 1;
                var partial_a_l1 = 1;
                var partial_a_l2 = 1;
                var partial_b_l0 = 2 * l0 - l1 - l2;
                var partial_b_l1 = 2 * l1 - l0 - l2;
                var partial_b_l2 = 2 * l2 - l0 - l1;

                double partial_u_l0 = (n * d * Math.Sqrt(B) * partial_a_l0 - (n * d * A) * (1 / 2 / Math.Sqrt(B)) * partial_b_l0) / B;
                double partial_u_l1 = (n * d * Math.Sqrt(B) * partial_a_l1 - (n * d * A) * (1 / 2 / Math.Sqrt(B)) * partial_b_l1) / B;
                double partial_u_l2 = (n * d * Math.Sqrt(B) * partial_a_l2 - (n * d * A) * (1 / 2 / Math.Sqrt(B)) * partial_b_l2) / B;

                double partial_v_l0 = (1 / Math.Sqrt(1 - Math.Pow(Math.Sqrt(B) / (3 * n * d), 2))) * (1 / (3 * n * d)) * (1 / (2 * Math.Sqrt(B))) * partial_b_l0;
                double partial_v_l1 = (1 / Math.Sqrt(1 - Math.Pow(Math.Sqrt(B) / (3 * n * d), 2))) * (1 / (3 * n * d)) * (1 / (2 * Math.Sqrt(B))) * partial_b_l1;
                double partial_v_l2 = (1 / Math.Sqrt(1 - Math.Pow(Math.Sqrt(B) / (3 * n * d), 2))) * (1 / (3 * n * d)) * (1 / (2 * Math.Sqrt(B))) * partial_b_l2;

                double partial_s_l0 = partial_u_l0 * v + partial_v_l0 * u;
                double partial_s_l1 = partial_u_l1 * v + partial_v_l1 * u;
                double partial_s_l2 = partial_u_l2 * v + partial_v_l2 * u;

                return new double[] { partial_s_l0, partial_s_l1, partial_s_l2 };
            }

            double[] partial_s = CalculateS(lengths, diameter, numOfSegments);

            // k
            double[] CalculateK(Vector<double> lengths, double diameter) {
                double l0, l1, l2;
                l0 = lengths[0];
                l1 = lengths[1];
                l2 = lengths[2];

                var d = diameter;

                var partial_b_l0 = 2 * l0 - l1 - l2;
                var partial_b_l1 = 2 * l1 - l0 - l2;
                var partial_b_l2 = 2 * l2 - l0 - l1;

                return new double[] {
                    1 / (Math.Sqrt(partial_b_l0) * d),
                    1 / (Math.Sqrt(partial_b_l1) * d),
                    1 / (Math.Sqrt(partial_b_l2) * d)
                };
            }

            double[] partial_k = CalculateK(lengths, diameter);

            // phi
            double[] CalculatePhi(Vector<double> lengths) {
                double l0, l1, l2;
                l0 = lengths[0];
                l1 = lengths[1];
                l2 = lengths[2];

                var A = Math.Sqrt(3) * (l2 + l1 - 2 * l0);
                var B = 3 * (l2 - l1);

                var partial_phi_l0 = -2 * Math.Sqrt(3) * B / (A + Math.Pow(B, 2));
                var partial_phi_l1 = (Math.Sqrt(3) * B - 3 * A) / (A + Math.Pow(B, 2));
                var partial_phi_l2 = (Math.Sqrt(3) * B + 3 * A) / (A + Math.Pow(B, 2));

                return new double[] { partial_phi_l0, partial_phi_l1, partial_phi_l2 };
            }

            double[] partial_phi = CalculatePhi(lengths);

            return CreateMatrix.DenseOfRowArrays(new double[][] { partial_s, partial_k, partial_phi });
        }

        //public static Matrix<double> MakeJacobianMatrix_DH_F1(Vector<double> lengths, double diameter, int numOfSegments) {
        //    double l0, l1, l2;
        //    l0 = lengths[0];
        //    l1 = lengths[1];
        //    l2 = lengths[2];

        //    var d = diameter;
        //    var n = numOfSegments;

        //    var A = l0 + l1 + l2;
        //    var B = l0 * l0 + l1 * l1 + l2 * l2 - l0 * l1 - l0 * l2 - l1 * l2;

        //    double s = n * d * A / (Math.Sqrt(B)) * Math.Asin(Math.Sqrt(B) / (3 * n * d));
        //    double kappa = 2 * Math.Sqrt(B) / d / A;
        //    double phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2));
        //    double radius = 1 / kappa;
        //    double theta = s * kappa;
        //    var ks = kappa * s;
        //    var Rad90 = Math.PI / 2;
        //    var theta1 = Rad90 - theta / 2;
        //    var theta2 = -theta / 2;
        //    var di = Math.Sin(theta / 2) / kappa * 2;

        //    var DHTable = CreateMatrix.DenseOfArray(new double[,] {
        //        {phi, 0,0, Rad90},
        //        {theta1, 0, di,0 },
        //        {theta2,0,0,-Rad90 },
        //    });

        //    var T_DH = MakeTransformMatrixDH(DHTable);

        //    var C_ks = Math.Cos(ks);
        //    var S_ks = Math.Sin(ks);
        //    var C_phi = Math.Cos(phi);
        //    var S_phi = Math.Sin(phi);

        //    var e11 = -S_phi * (C_ks - 1) / kappa;
        //    var e12 = -C_phi * (ks * S_ks + C_ks - 1) / (kappa * kappa);
        //    var e13 = -C_phi * S_ks;

        //    var e21 = 0.0;
        //    var e22 = (ks * C_ks - S_ks) / (kappa * kappa);
        //    var e23 = C_ks;

        //    var e31 = C_phi * (C_ks - 1) / kappa;
        //    var e32 = S_phi * (ks * S_ks + C_ks - 1) / (kappa * kappa);
        //    var e33 = S_phi * S_ks;

        //    var e41 = C_phi * S_ks;
        //    var e42 = s * S_phi;
        //    var e43 = kappa * S_phi;

        //    var e51 = 1 - C_ks;
        //    var e52 = 0.0;
        //    var e53 = 0.0;

        //    var e61 = -S_phi * S_ks;
        //    var e62 = s * C_phi;
        //    var e63 = kappa * C_phi;

        //    return CreateMatrix.DenseOfArray(new double[,] {
        //        { e11, e12, e13 },
        //        { e21, e22, e23 },
        //        { e31, e32, e33 },
        //        { e41, e42, e43 },
        //        { e51, e52, e53 },
        //        { e61, e62, e63 },
        //    });
        ////}

        //public static Matrix<double> MakeJacobian(Vector<double> lengths, double diameter, int numOfSegments) {
        //    var arc = MakeTransformMatrixF2(lengths, numOfSegments, diameter);

        //    Matrix<double> JDH = MakeJacobianDH(arc[0], arc[1], arc[2]);
        //    Matrix<double> JF1 = MakeJacobianF1(arc[0], arc[1], arc[2]);
        //    Matrix<double> J2 = MakeJacobianF2(lengths, diameter, numOfSegments);

        //    return J1 * J2;
        //}

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
                { e11, e12, e13 },
                { 0, 0, 0 },
                { 0, 0, 0 },
                { 0, 0, 0 },

                { e21, e22, e23 },
                { 0, 0, 0 },
                { e41, e42, e43 },
                { 0, 0, 0 },

                { e31, e32, e33 },
                { 0, 0, 0 },
                { 0, 0, 0 },
                { 0, 0, 0 },
            });
        }

        public static Matrix<double> MakeJacobianDH(Matrix<double> table) {
            var Tee = MakeTransformMatrixDH(table.SubMatrix(0, table.RowCount, 1, table.ColumnCount - 1));
            var Oee = Vector3D.OfVector(Tee.Column(3).SubVector(0, 3));

            Vector<double> MakeJacobianDHForRevoluteJoint(Vector3D Zprev, Vector3D Oprev) {
                var JiV = Zprev.CrossProduct(Oee - Oprev);
                var JiW = Zprev;

                // return JiV appends JiW
                return Vector<double>.Build.DenseOfArray(new double[] { JiV.X, JiV.Y, JiV.Z, JiW.X, JiW.Y, JiW.Z });
            }

            Vector<double> MakeDHForPrismaticJoint(Vector3D Zprev) {
                var JiV = Zprev;
                var JiW = new Vector3D();

                return Vector<double>.Build.DenseOfArray(new double[] { JiV.X, JiV.Y, JiV.Z, JiW.X, JiW.Y, JiW.Z });
            }

            var Zprev = new Vector3D(0, 0, 1);
            var Oprev = new Vector3D(0, 0, 0);

            Matrix<double> JDH = CreateMatrix.Dense<double>(6, table.RowCount);

            for(int i = 0; i<table.RowCount; i++) {

                var type = table[i, 0];
                var theta = table[i, 1];
                var d = table[i, 2];
                var a = table[i, 3];
                var alpha = table[i, 4];
                var Ti = MakeTransformMatrixDH(theta, d, a, alpha);

                Vector<double> Ji;

                if (type == 0)
                    Ji = MakeJacobianDHForRevoluteJoint(Zprev, Oprev);
                else
                    Ji = MakeDHForPrismaticJoint(Zprev);

                Zprev = Vector3D.OfVector(Ti * Zprev.ToVector());
                Oprev = Vector3D.OfVector(Ti.Column(3).SubVector(0, 3));

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


            double s = n * d * A / (Math.Sqrt(B)) * Math.Asin(f1);
            double kappa = 2 * Math.Sqrt(B) / d / A;
            double phi = double.NaN;

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
                phi = double.NaN;

            var theta0 = phi;
            var theta1 = (Math.PI - s * kappa) / 2;
            var theta2 = -s * kappa / 2;
            var r = 2 * Math.Sin(s * kappa / 2) / kappa;
            
            //Debug.WriteLine($"S: {s}, Kappa: {kappa}, phi: {phi}, r: {r}");


            var q = CreateMatrix.DenseOfArray(new double[,] {
                { 0, theta0,0,0, Math.PI / 2 },
                { 0, theta1,0, r, 0 },
                { 0, theta2,0,0,-Math.PI / 2 }
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
                var Ti = MakeTransformMatrixDH(table[i, 1], table[i, 2], table[i, 3], table[i, 4]);

                if (i == 0)
                    T = Ti;
                else
                    T = T * Ti;
            }

            return T;
        }
    }
}