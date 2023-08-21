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
        public Vector<double> GetPCCParameters(Vector<double> l) {

            return Vector<double>.Build.DenseOfArray(new double[] { 0.0, 0.0, 0.0 });
        }
    }

}
