using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Security.Permissions;

namespace ConventionalIK {
    public class Kinematics {

        public Kinematics() { }

        // Forward Kinematics
        // Kappa: Curvature
        // Phi: angle
        // S: Length
        public static Vector<double> GetSingleSectionArcParameters(Vector<double> l, int numOfSegments, double diameter) {
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

        public static Vector<double> GetLengths(Vector<double> arcParameters, int numOfSegments, double diameter) {
            var s = arcParameters[0];
            var kappa = arcParameters[1];
            var phi = arcParameters[2];

            var d = diameter;
            var n = numOfSegments;

            var l0 = 2 * n * Math.Sin(kappa * s / (2 * n)) * (1 / kappa - d * Math.Sin(phi));
            var l1 = 2 * n * Math.Sin(kappa * s / (2 * n)) * (1 / kappa + d * Math.Sin(phi + Math.PI / 3));
            var l2 = 2 * n * Math.Sin(kappa * s / (2 * n)) * (1 / kappa - d * Math.Cos(phi + Math.PI / 6));

            if (kappa == 0) {
                l0 = s;
                l1 = s;
                l2 = s;
            }

            return Vector<double>.Build.DenseOfArray(new double[] { l0, l1, l2 });
        }

        public static Matrix<double> MakeJacobianMatrix_F2(Vector<double> lengths, double diameter, int numOfSegments) {
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

            return Matrix<double>.Build.DenseOfRowArrays(new double[][] { partial_s, partial_k, partial_phi });
        }

        public static Matrix<double> MakeJacobianMatrix_DH_F1(Vector<double> lengths, double diameter, int numOfSegments) {
            double l0, l1, l2;
            l0 = lengths[0];
            l1 = lengths[1];
            l2 = lengths[2];

            var d = diameter;
            var n = numOfSegments;

            var A = l0 + l1 + l2;
            var B = l0 * l0 + l1 * l1 + l2 * l2 - l0 * l1 - l0 * l2 - l1 * l2;

            double s = n * d * A / (Math.Sqrt(B)) * Math.Asin(Math.Sqrt(B) / (3 * n * d));
            double kappa = 2 * Math.Sqrt(B) / d / A;
            double phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2));

            var ks = kappa * s;
            var C_ks = Math.Cos(ks);
            var S_ks = Math.Sin(ks);
            var C_phi = Math.Cos(phi);
            var S_phi = Math.Sin(phi);

            var e11 = -S_phi * (C_ks - 1) / kappa;
            var e12 = -C_phi * (ks * S_ks + C_ks - 1) / (kappa * kappa);
            var e13 = -C_phi * S_ks;

            var e21 = 0.0;
            var e22 = (ks * C_ks - S_ks) / (kappa * kappa);
            var e23 = C_ks;

            var e31 = C_phi * (C_ks - 1) / kappa;
            var e32 = S_phi * (ks * S_ks + C_ks - 1) / (kappa * kappa);
            var e33 = S_phi * S_ks;

            var e41 = C_phi * S_ks;
            var e42 = s * S_phi;
            var e43 = kappa * S_phi;

            var e51 = 1 - C_ks;
            var e52 = 0.0;
            var e53 = 0.0;

            var e61 = -S_phi * S_ks;
            var e62 = s * C_phi;
            var e63 = kappa * C_phi;

            return Matrix<double>.Build.DenseOfArray(new double[,] {
                { e11, e12, e13 },
                { e21, e22, e23 },
                { e31, e32, e33 },
                { e41, e42, e43 },
                { e51, e52, e53 },
                { e61, e62, e63 },
            });
        }

        public static Matrix<double> MakeJacobian(Vector<double> lengths, double diameter, int numOfSegments) {
            Matrix<double> J1 = MakeJacobianMatrix_DH_F1(lengths, diameter, numOfSegments);
            Matrix<double> J2 = MakeJacobianMatrix_F2(lengths, diameter, numOfSegments);

            return J1 * J2;
        }

        public static Vector<double> ForwardKinematics(Vector<double> lengths, double diameter, int numOfSegments) {
            double l0, l1, l2;
            l0 = lengths[0];
            l1 = lengths[1];
            l2 = lengths[2];

            var d = diameter;
            var n = numOfSegments;

            var A = l0 + l1 + l2;
            var B = l0 * l0 + l1 * l1 + l2 * l2 - l0 * l1 - l0 * l2 - l1 * l2;

            double s = n * d * A / (Math.Sqrt(B)) * Math.Asin(Math.Sqrt(B) / (3 * n * d));
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

            //if (l1 < l2) {
            //    phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2));
            //}
            //else if (l1 > l2) {
            //    phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2)) + Math.PI;
            //}
            //else if (l1 == l2) {
            //    if (l0 < l1) {
            //        phi = 3 * Math.PI / 2;
            //    }
            //    else if (l0 > l1) {
            //        phi = Math.PI / 2;
            //    }
            //}
            //else
            //    phi = double.NaN;

            double c_phi = Math.Cos(phi);
            double c_ks = Math.Cos(kappa * s);
            double s_phi = Math.Sin(phi);
            double s_ks = Math.Sin(kappa * s);

            Matrix<double> T = Matrix<double>.Build.DenseOfArray(new double[,] {
                { c_phi * c_phi * (c_ks - 1) + 1, s_phi * c_phi * (c_ks - 1), -c_phi * s_ks, c_phi * (c_ks - 1) / kappa },
                { s_phi * c_phi * (c_ks - 1), c_phi * c_phi * (1 - c_ks) + c_ks, -s_phi * s_ks, s_phi * (c_ks - 1) / kappa },
                { c_phi * s_ks, s_phi * s_ks, c_ks, s_ks / kappa },
                { 0, 0, 0, 1 }
            });

            return T * Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 0, 1 });
        }

        public IEnumerable<Vector<double>> IK_JacobianTranspose(Vector<double> desiredE, Vector<double> currL, double diameter, int numOfSegments) {
            Matrix<double> J = MakeJacobian(currL, diameter, numOfSegments);
            Matrix<double> JT = J.Clone();

            JT.Transpose();
            double alpha = 0.1;
            double thereshold = 0.01;

            Vector<double> currE = ForwardKinematics(currL, diameter, numOfSegments);
            Vector<double> nextL = Vector<double>.Build.Dense(4, 0.0);

            while ((desiredE - currE).Norm(2) < thereshold) {
                currE = ForwardKinematics(currL, diameter, numOfSegments);
                nextL = currL + alpha * JT * (desiredE - currE);

                yield return nextL;
            }

            yield break;
        }

        //public IEnumerable<Vector<double>> IK_JacobianTranspose_r(Vector<double> desiredE, Vector<double> currL, double diameter, int numOfSegments) {

        //}

        public static Matrix<double> MakeDHTransformMatrix(double theta, double d, double r, double alpha) {
            var s_theta = Math.Sin(theta);
            var c_theta = Math.Cos(theta);
            var s_alpha = Math.Sin(alpha);
            var c_alpha = Math.Cos(alpha);


            return Matrix<double>.Build.DenseOfArray(new double[,] {
                {c_theta, -s_theta*c_alpha, s_theta * s_alpha, r * c_theta},
                {s_theta, c_theta * c_alpha, -c_theta * s_alpha, r * s_theta},
                {0, s_alpha, c_alpha, d},
                {0, 0, 0, 1}
            });
        }

        public static Matrix<double> MakeDHTransformMatrix(Matrix<double> table) {
            Matrix<double> T = Matrix<double>.Build.DenseIdentity(4);

            for(int i=0;i<table.RowCount;i++) {
                var Ti = MakeDHTransformMatrix(table[i, 0], table[i, 1], table[i, 2], table[i, 3]);

                T *= Ti;
            }

            return T;
        }
    }

}
