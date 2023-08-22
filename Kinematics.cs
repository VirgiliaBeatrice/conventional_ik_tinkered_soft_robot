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
            var phi = Math.Atan(Math.Sqrt(3) / 3 * (l2 + l1 - 2 * l0) / (l1 - l2));

            return Vector<double>.Build.DenseOfArray(new double[] { s, kappa, phi });
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
    }

}
