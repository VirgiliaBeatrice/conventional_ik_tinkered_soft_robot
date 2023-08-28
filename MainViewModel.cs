using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
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

namespace ConventionalIK {

    public partial class MainViewModel : ObservableObject {
        [ObservableProperty]
        private Point3D currentPose;

        [RelayCommand]
        private void MakeJoints() {
            Objects.Add(new JointVisual3D());

            var T = Kinematics.MakeDHTransformMatrix()
            
        }

        
        public ObservableCollection<Visual3D> Objects { get; set; }

        public MainViewModel() {
            Objects = new ObservableCollection<Visual3D> {
                new SunLight(),
                new SphereVisual3D {
                    Center = new Point3D(),
                    Radius = 0.1,
                    Fill = Brushes.White
                },
                new ArrowVisual3D {
                    Point1 = new Point3D(),
                    Point2 = new Point3D(1, 0, 0),
                    Diameter = 0.03,
                    Fill = Brushes.Red
                },
                new ArrowVisual3D {
                    Point1 = new Point3D(),
                    Point2 = new Point3D(0, 1, 0),
                    Diameter = 0.03,
                    Fill = Brushes.Green
                },
                new ArrowVisual3D {
                    Point1 = new Point3D(),
                    Point2 = new Point3D(0, 0, 1),
                    Diameter = 0.03,
                    Fill = Brushes.Blue
                },
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

    public class JointVisual3D : ModelVisual3D {
        public new Transform3D Transform {
            get { return base.Transform; }
            set {
                base.Transform = value;

                // Apply the transformation to each axis arrow
                xAxisArrow.Transform = value;
                yAxisArrow.Transform = value;
                zAxisArrow.Transform = value;
            }
        }

        private ArrowVisual3D xAxisArrow;
        private ArrowVisual3D yAxisArrow;
        private ArrowVisual3D zAxisArrow;

        public JointVisual3D() {
            xAxisArrow = new ArrowVisual3D {
                Point1 = new Point3D(0, 0, 0),
                Point2 = new Point3D(1, 0, 0), // Unit vector in X direction
                Diameter = 0.1,
                Fill = Brushes.Red
            };

            yAxisArrow = new ArrowVisual3D {
                Point1 = new Point3D(0, 0, 0),
                Point2 = new Point3D(0, 1, 0), // Unit vector in Y direction
                Diameter = 0.1,
                Fill = Brushes.Green
            };

            zAxisArrow = new ArrowVisual3D {
                Point1 = new Point3D(0, 0, 0),
                Point2 = new Point3D(0, 0, 1), // Unit vector in Z direction
                Diameter = 0.1,
                Fill = Brushes.Blue
            };

            // Add arrows to the visual
            Children.Add(xAxisArrow);
            Children.Add(yAxisArrow);
            Children.Add(zAxisArrow);
        }


    }
}
