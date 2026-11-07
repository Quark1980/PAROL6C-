using System;
using System.IO;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace PAROLC_ControlSuite.Models
{
    /// <summary>
    /// Loads and manages the 3D visual mesh hierarchy for the PAROL6 arm.
    /// Manages dual representation: Solid Live Robot (actual Pi feedback) and Ghost Robot (commanded target).
    /// </summary>
    public class RobotMeshVisualizer
    {
        private readonly RobotKinematics _kinematics;
        private readonly ModelVisual3D _liveRobotGroup = new ModelVisual3D();
        private readonly ModelVisual3D _ghostRobotGroup = new ModelVisual3D();

        // 7 Visual links: index 0 = Base, 1 = L1, 2 = L2, 3 = L3, 4 = L4, 5 = L5, 6 = L6
        private readonly ModelVisual3D[] _liveLinks = new ModelVisual3D[7];
        private readonly ModelVisual3D[] _ghostLinks = new ModelVisual3D[7];

        public ModelVisual3D LiveVisual => _liveRobotGroup;
        public ModelVisual3D GhostVisual => _ghostRobotGroup;

        private bool _ghostVisible = true;
        public bool GhostVisible
        {
            get => _ghostVisible;
            set
            {
                _ghostVisible = value;
                _ghostRobotGroup.Children.Clear();
                if (_ghostVisible)
                {
                    foreach (var link in _ghostLinks)
                    {
                        if (link != null) _ghostRobotGroup.Children.Add(link);
                    }
                }
            }
        }

        public RobotMeshVisualizer(RobotKinematics kinematics)
        {
            _kinematics = kinematics ?? throw new ArgumentNullException(nameof(kinematics));
        }

        /// <summary>
        /// Loads the STL mesh files from the application Assets directory.
        /// </summary>
        public void LoadMeshes(string assetsPath)
        {
            string meshesDir = Path.Combine(assetsPath, "meshes");
            if (!Directory.Exists(meshesDir))
            {
                // Fallback to checking executable directory
                meshesDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "meshes");
            }

            string[] fileNames = new[]
            {
                "base_link_simplified.stl",
                "L1_simplified.stl",
                "L2_simplified.stl",
                "L3_simplified.stl",
                "L4_simplified.stl",
                "L5_simplified.stl",
                "L6_simplified.stl"
            };

            string[] fallbackFileNames = new[]
            {
                "base_link.STL",
                "L1.STL",
                "L2.STL",
                "L3.STL",
                "L4.STL",
                "L5.STL",
                "L6.STL"
            };

            // Colors for solid live robot
            Color[] liveColors = new[]
            {
                Color.FromRgb(44, 62, 80),     // Base: Dark Anthracite
                Color.FromRgb(0, 168, 255),    // L1: Electric Cyan
                Color.FromRgb(149, 165, 166),  // L2: Metallic Silver
                Color.FromRgb(0, 168, 255),    // L3: Electric Cyan
                Color.FromRgb(149, 165, 166),  // L4: Metallic Silver
                Color.FromRgb(52, 73, 94),     // L5: Deep Slate
                Color.FromRgb(230, 126, 34)    // L6: Burnished Flange Gold
            };

            // Translucent glowing cyan for target ghost robot
            Color ghostColor = Color.FromArgb(90, 0, 220, 255);

            for (int i = 0; i < 7; i++)
            {
                var reader = new StLReader();
                string filePath = Path.Combine(meshesDir, fileNames[i]);
                if (!File.Exists(filePath))
                {
                    filePath = Path.Combine(meshesDir, fallbackFileNames[i]);
                }

                Model3DGroup loadedMesh = null;
                if (File.Exists(filePath))
                {
                    try
                    {
                        loadedMesh = reader.Read(filePath);
                    }
                    catch
                    {
                        loadedMesh = null;
                    }
                }

                // If loading failed, generate fallback geometric block
                if (loadedMesh == null)
                {
                    loadedMesh = CreateFallbackGeometry(i);
                }

                // STL coordinates are in METERS. Scale by 1000 to convert to MILLIMETERS
                // Also L6 has a visual origin offset of -21 mm (-0.021 m) along Z
                var meshTransform = new Transform3DGroup();
                meshTransform.Children.Add(new ScaleTransform3D(1000, 1000, 1000));
                if (i == 6)
                {
                    meshTransform.Children.Add(new TranslateTransform3D(0, 0, -21.0));
                }

                // 1. Build Solid Live Link
                var liveModel = CloneModelWithMaterial(loadedMesh, MaterialHelper.CreateMaterial(new SolidColorBrush(liveColors[i]), 0.8, 60));
                liveModel.Transform = meshTransform;

                var liveVisual = new ModelVisual3D { Content = liveModel };
                liveVisual.Transform = new Transform3DGroup();
                _liveLinks[i] = liveVisual;
                _liveRobotGroup.Children.Add(liveVisual);

                // 2. Build Ghost Target Link (Semi-Transparent)
                var ghostMat = new DiffuseMaterial(new SolidColorBrush(ghostColor));
                var ghostModel = CloneModelWithMaterial(loadedMesh, ghostMat);
                ghostModel.Transform = meshTransform;

                var ghostVisual = new ModelVisual3D { Content = ghostModel };
                ghostVisual.Transform = new Transform3DGroup();
                _ghostLinks[i] = ghostVisual;
                if (_ghostVisible)
                {
                    _ghostRobotGroup.Children.Add(ghostVisual);
                }
            }
        }

        /// <summary>
        /// Updates the 3D joint poses of both the interactive digital twin (solid) and physical feedback (ghost).
        /// </summary>
        public void UpdatePoses(double[] liveJointsDeg, double[] targetJointsDeg)
        {
            // 1. Update Solid Interactive Target Twin (CAD / Commanded Target)
            // Responds immediately to sliders, 3D gizmo, and jog controls
            if (targetJointsDeg != null)
            {
                var targetTransforms = _kinematics.ComputeAllLinkTransforms(targetJointsDeg);
                for (int i = 0; i < 7; i++)
                {
                    if (_liveLinks[i] != null)
                    {
                        var mat = targetTransforms[i];
                        _liveLinks[i].Transform = ToWpfMatrixTransform(mat);
                    }
                }
            }

            // 2. Update Cyan Ghost Robot (Actual Physical Robot Telemetry feedback from Pi)
            // Displays real-time physical arm position in space
            if (liveJointsDeg != null)
            {
                var liveTransforms = _kinematics.ComputeAllLinkTransforms(liveJointsDeg);
                for (int i = 0; i < 7; i++)
                {
                    if (_ghostLinks[i] != null)
                    {
                        var mat = liveTransforms[i];
                        _ghostLinks[i].Transform = ToWpfMatrixTransform(mat);
                    }
                }
            }
        }

        private static MatrixTransform3D ToWpfMatrixTransform(in RobotKinematics.Mat4 m)
        {
            // Note: WPF Matrix3D uses row-vector convention (v * M)
            var wpfMat = new Matrix3D(
                m.M00, m.M10, m.M20, 0,
                m.M01, m.M11, m.M21, 0,
                m.M02, m.M12, m.M22, 0,
                m.M03, m.M13, m.M23, 1
            );
            return new MatrixTransform3D(wpfMat);
        }

        private static Model3DGroup CloneModelWithMaterial(Model3DGroup original, Material material)
        {
            var group = new Model3DGroup();
            foreach (var child in original.Children)
            {
                if (child is GeometryModel3D geom)
                {
                    var clone = new GeometryModel3D
                    {
                        Geometry = geom.Geometry,
                        Material = material,
                        BackMaterial = material
                    };
                    group.Children.Add(clone);
                }
                else if (child is Model3DGroup subGroup)
                {
                    group.Children.Add(CloneModelWithMaterial(subGroup, material));
                }
            }
            return group;
        }

        private static Model3DGroup CreateFallbackGeometry(int linkIndex)
        {
            var builder = new MeshBuilder(false, false);
            double h = 0.05 + linkIndex * 0.02;
            builder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, h), 0.03, 16);
            var mesh = builder.ToMesh();

            var group = new Model3DGroup();
            group.Children.Add(new GeometryModel3D(mesh, MaterialHelper.CreateMaterial(Colors.DarkGray)));
            return group;
        }
    }
}
