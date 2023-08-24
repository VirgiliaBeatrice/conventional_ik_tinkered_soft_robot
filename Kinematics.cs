using HelixToolkit.Wpf;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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
            } else {
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

        public static Matrix<double> MakeJacobianMatrix_F2(Vector<double> lengths, int numOfSegments, double diameter) {
            double l0, l1, l2;
            
            l0 = lengths[0];
            l1 = lengths[1];
            l2 = lengths[2];

            int n = numOfSegments;
            double d = diameter;


            // s
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


            // k 
            double partial_k_l0 = 1 / (Math.Sqrt(partial_b_l0) * d);
            double partial_k_l1 = 1 / (Math.Sqrt(partial_b_l1) * d);
            double partial_k_l2 = 1 / (Math.Sqrt(partial_b_l2) * d);

            // phi: TODO
            double partial_phi_l0 = 0;
            double partial_phi_l1 = 0;
            double partial_phi_l2 = 0;

            return Matrix<double>.Build.DenseOfArray(new double[,] {
                { partial_s_l0, partial_k_l0, partial_phi_l0 },
                { partial_s_l1, partial_k_l1, partial_phi_l1 },
                { partial_s_l2, partial_k_l2, partial_phi_l2 }
            }).Transpose();
        }

        public static Matrix<double> MakeJacobianMatrix_DH_F1() {
            double s = 0, kappa= 0, phi = 0;

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
    }

}
