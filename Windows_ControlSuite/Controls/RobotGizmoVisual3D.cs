using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace PAROLC_ControlSuite.Controls
{
    /// <summary>
    /// Interactive 3D Cartesian Translation & Rotation Gizmo positioned at the robot TCP.
    /// Supports raycast hit-testing and dragging along X (Red), Y (Green), and Z (Blue) axes.
    /// </summary>
    public class RobotGizmoVisual3D : ModelVisual3D
    {
        private readonly TranslateTransform3D _positionTransform = new TranslateTransform3D();
        private readonly RotateTransform3D _rotationTransform = new RotateTransform3D();
        private readonly Transform3DGroup _rootTransform = new Transform3DGroup();

        // Hit-testable geometries
        public GeometryModel3D HandleX { get; private set; }
        public GeometryModel3D HandleY { get; private set; }
        public GeometryModel3D HandleZ { get; private set; }
        public GeometryModel3D CenterHandle { get; private set; }

        public Point3D Position
        {
            get => new Point3D(_positionTransform.OffsetX, _positionTransform.OffsetY, _positionTransform.OffsetZ);
            set
            {
                _positionTransform.OffsetX = value.X;
                _positionTransform.OffsetY = value.Y;
                _positionTransform.OffsetZ = value.Z;
            }
        }

        public double AxisLength { get; set; } = 60.0; // mm
        public double ArrowRadius { get; set; } = 4.0; // mm
        public double ConeHeight { get; set; } = 14.0; // mm
        public double ConeRadius { get; set; } = 8.0;  // mm

        public RobotGizmoVisual3D()
        {
            _rootTransform.Children.Add(_rotationTransform);
            _rootTransform.Children.Add(_positionTransform);
            this.Transform = _rootTransform;

            BuildGizmoGeometry();
        }

        private void BuildGizmoGeometry()
        {
            var modelGroup = new Model3DGroup();

            // Center Origin Sphere
            var centerBuilder = new MeshBuilder(false, false);
            centerBuilder.AddSphere(new Point3D(0, 0, 0), ArrowRadius * 1.5, 16, 16);
            CenterHandle = new GeometryModel3D(centerBuilder.ToMesh(), MaterialHelper.CreateMaterial(Colors.Gold));
            modelGroup.Children.Add(CenterHandle);

            // X Axis Handle (Red)
            var xBuilder = new MeshBuilder(false, false);
            xBuilder.AddCylinder(new Point3D(0, 0, 0), new Point3D(AxisLength - ConeHeight, 0, 0), ArrowRadius, 16);
            xBuilder.AddCone(new Point3D(AxisLength - ConeHeight, 0, 0), new Vector3D(1, 0, 0), ConeRadius, 0.0, ConeHeight, true, false, 16);
            HandleX = new GeometryModel3D(xBuilder.ToMesh(), MaterialHelper.CreateMaterial(Colors.Crimson));
            modelGroup.Children.Add(HandleX);

            // Y Axis Handle (Green)
            var yBuilder = new MeshBuilder(false, false);
            yBuilder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, AxisLength - ConeHeight, 0), ArrowRadius, 16);
            yBuilder.AddCone(new Point3D(0, AxisLength - ConeHeight, 0), new Vector3D(0, 1, 0), ConeRadius, 0.0, ConeHeight, true, false, 16);
            HandleY = new GeometryModel3D(yBuilder.ToMesh(), MaterialHelper.CreateMaterial(Colors.LimeGreen));
            modelGroup.Children.Add(HandleY);

            // Z Axis Handle (Blue)
            var zBuilder = new MeshBuilder(false, false);
            zBuilder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, AxisLength - ConeHeight), ArrowRadius, 16);
            zBuilder.AddCone(new Point3D(0, 0, AxisLength - ConeHeight), new Vector3D(0, 0, 1), ConeRadius, 0.0, ConeHeight, true, false, 16);
            HandleZ = new GeometryModel3D(zBuilder.ToMesh(), MaterialHelper.CreateMaterial(Colors.DodgerBlue));
            modelGroup.Children.Add(HandleZ);

            // Rotation Rings (Toroids) for visualization
            var ringX = new MeshBuilder(false, false);
            ringX.AddTorus(AxisLength * 0.7, 1.2, 24, 12);
            var ringXModel = new GeometryModel3D(ringX.ToMesh(), MaterialHelper.CreateMaterial(Color.FromArgb(120, 220, 20, 60)));
            var rotX = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 90));
            ringXModel.Transform = rotX;
            modelGroup.Children.Add(ringXModel);

            var ringY = new MeshBuilder(false, false);
            ringY.AddTorus(AxisLength * 0.7, 1.2, 24, 12);
            var ringYModel = new GeometryModel3D(ringY.ToMesh(), MaterialHelper.CreateMaterial(Color.FromArgb(120, 50, 205, 50)));
            var rotY = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 90));
            ringYModel.Transform = rotY;
            modelGroup.Children.Add(ringYModel);

            var ringZ = new MeshBuilder(false, false);
            ringZ.AddTorus(AxisLength * 0.7, 1.2, 24, 12);
            var ringZModel = new GeometryModel3D(ringZ.ToMesh(), MaterialHelper.CreateMaterial(Color.FromArgb(120, 30, 144, 255)));
            modelGroup.Children.Add(ringZModel);

            this.Content = modelGroup;
        }

        public void SetPose(double xMm, double yMm, double zMm, double rxDeg = 0, double ryDeg = 0, double rzDeg = 0)
        {
            Position = new Point3D(xMm, yMm, zMm);
        }
    }
}
