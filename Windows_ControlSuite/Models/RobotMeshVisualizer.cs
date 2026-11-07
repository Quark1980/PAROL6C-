using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace PAROLC_ControlSuite.Models
{
    /// <summary>
    /// Metadata descriptor for a selectable End Effector / Gripper in Assets/meshes.
    /// </summary>
    public class EndEffectorDefinition
    {
        public string Id { get; set; }
        public string DisplayName { get; set; }
        public string BodyMeshFile { get; set; }
        public string LeftJawMeshFile { get; set; }
        public string RightJawMeshFile { get; set; }
        public double ToolLengthMm { get; set; } = 118.0;
        public string JawSlideAxis { get; set; } = "Y"; // "Y", "X", or "NONE"
        public double MaxJawStrokeMm { get; set; } = 20.0;
        public bool IsMillimeterUnits { get; set; } = false;
        public bool IsTd8820Custom { get; set; } = false;
        public bool HasTopCamera30Deg { get; set; } = false;

        public override string ToString() => DisplayName;
    }

    /// <summary>
    /// Loads and manages the 3D visual mesh hierarchy for the PAROL6 arm and its selectable End Effectors.
    /// Manages dual representation: Solid Interactive Twin (target) and Cyan Ghost Robot (actual feedback),
    /// including animated sliding gripper fingers, rotating pinion gear, and 30° downward top camera.
    /// </summary>
    public class RobotMeshVisualizer
    {
        private readonly RobotKinematics _kinematics;
        private readonly ModelVisual3D _liveRobotGroup = new ModelVisual3D();
        private readonly ModelVisual3D _ghostRobotGroup = new ModelVisual3D();

        // 7 Visual links: index 0 = Base, 1 = L1, 2 = L2, 3 = L3, 4 = L4, 5 = L5, 6 = L6
        private readonly ModelVisual3D[] _liveLinks = new ModelVisual3D[7];
        private readonly ModelVisual3D[] _ghostLinks = new ModelVisual3D[7];

        // End-Effector sub-visuals attached to L6 flange frame
        private readonly ModelVisual3D _liveToolRoot = new ModelVisual3D();
        private readonly ModelVisual3D _ghostToolRoot = new ModelVisual3D();

        private Model3DGroup _liveBodyGroup;
        private Model3DGroup _liveLeftJawGroup;
        private Model3DGroup _liveRightJawGroup;
        private Model3DGroup _livePinionGroup;
        private Model3DGroup _liveCameraFovGroup;

        private Model3DGroup _ghostBodyGroup;
        private Model3DGroup _ghostLeftJawGroup;
        private Model3DGroup _ghostRightJawGroup;
        private Model3DGroup _ghostPinionGroup;

        private string _meshesDir;
        private EndEffectorDefinition _activeEndEffector;
        private double _lastTargetGripperDeg = 90.0;
        private double _lastActualGripperDeg = 90.0;
        private double[] _lastLiveJoints = (double[])RobotKinematics.CadHomeDeg.Clone();
        private double[] _lastTargetJoints = (double[])RobotKinematics.CadHomeDeg.Clone();

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
                    _ghostRobotGroup.Children.Add(_ghostToolRoot);
                }
            }
        }

        private bool _showCameraFovCone = true;
        public bool ShowCameraFovCone
        {
            get => _showCameraFovCone;
            set
            {
                _showCameraFovCone = value;
                if (_activeEndEffector != null)
                {
                    SetEndEffector(_activeEndEffector);
                }
            }
        }

        public RobotMeshVisualizer(RobotKinematics kinematics)
        {
            _kinematics = kinematics ?? throw new ArgumentNullException(nameof(kinematics));
        }

        /// <summary>
        /// Builds the catalog of available end effectors present in Assets/meshes.
        /// Places End_effector_TD-8820MG.stl (118 mm tool length, animated fingers + 30° top camera) first as default.
        /// </summary>
        public static List<EndEffectorDefinition> GetAvailableEndEffectors(string assetsPath = null)
        {
            string meshesDir = null;
            if (!string.IsNullOrEmpty(assetsPath))
            {
                meshesDir = Path.Combine(assetsPath, "meshes");
            }
            if (string.IsNullOrEmpty(meshesDir) || !Directory.Exists(meshesDir))
            {
                meshesDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "meshes");
            }

            var list = new List<EndEffectorDefinition>
            {
                new EndEffectorDefinition
                {
                    Id = "td8820mg",
                    DisplayName = "End_effector_TD-8820MG (118 mm | Animated + 30° Cam)",
                    BodyMeshFile = "End_effector_TD-8820MG.stl",
                    ToolLengthMm = 118.0,
                    JawSlideAxis = "Y",
                    MaxJawStrokeMm = 20.0,
                    IsMillimeterUnits = true,
                    IsTd8820Custom = true,
                    HasTopCamera30Deg = true
                },
                new EndEffectorDefinition
                {
                    Id = "ssg48_standard",
                    DisplayName = "SSG-48 Electric Gripper — Standard Fingers (105 mm)",
                    BodyMeshFile = "ssg48_body_simplified.stl",
                    LeftJawMeshFile = "ssg48_finger_left_simplified.stl",
                    RightJawMeshFile = "ssg48_finger_right_simplified.stl",
                    ToolLengthMm = 105.0,
                    JawSlideAxis = "Y",
                    MaxJawStrokeMm = 18.0
                },
                new EndEffectorDefinition
                {
                    Id = "ssg48_pinch",
                    DisplayName = "SSG-48 Electric Gripper — Pinch Fingers (105 mm)",
                    BodyMeshFile = "ssg48_body_simplified.stl",
                    LeftJawMeshFile = "ssg48_pinch_left_simplified.stl",
                    RightJawMeshFile = "ssg48_pinch_right_simplified.stl",
                    ToolLengthMm = 105.0,
                    JawSlideAxis = "Y",
                    MaxJawStrokeMm = 18.0
                },
                new EndEffectorDefinition
                {
                    Id = "msg_ai_100",
                    DisplayName = "MSG-AI-100 Servo Gripper (104 mm)",
                    BodyMeshFile = "msg_ai_100_body_simplified.stl",
                    LeftJawMeshFile = "msg_ai_100_left_jaw_simplified.stl",
                    RightJawMeshFile = "msg_ai_100_right_jaw_simplified.stl",
                    ToolLengthMm = 104.0,
                    JawSlideAxis = "Y",
                    MaxJawStrokeMm = 16.0
                },
                new EndEffectorDefinition
                {
                    Id = "msg_ai_150",
                    DisplayName = "MSG-AI-150 Servo Gripper (104 mm)",
                    BodyMeshFile = "msg_ai_150_body_simplified.stl",
                    LeftJawMeshFile = "msg_ai_150_left_jaw_simplified.stl",
                    RightJawMeshFile = "msg_ai_150_right_jaw_simplified.stl",
                    ToolLengthMm = 104.0,
                    JawSlideAxis = "Y",
                    MaxJawStrokeMm = 18.0
                },
                new EndEffectorDefinition
                {
                    Id = "msg_ai_200",
                    DisplayName = "MSG-AI-200 Servo Gripper (104 mm)",
                    BodyMeshFile = "msg_ai_200_body_simplified.stl",
                    LeftJawMeshFile = "msg_ai_200_left_jaw_simplified.stl",
                    RightJawMeshFile = "msg_ai_200_right_jaw_simplified.stl",
                    ToolLengthMm = 104.0,
                    JawSlideAxis = "Y",
                    MaxJawStrokeMm = 20.0
                },
                new EndEffectorDefinition
                {
                    Id = "pneumatic_horizontal",
                    DisplayName = "Pneumatic Gripper — Horizontal Jaws (82 mm)",
                    BodyMeshFile = "pneumatic_gripper_horizontal_body_simplified.stl",
                    LeftJawMeshFile = "pneumatic_gripper_horizontal_left_jaw_simplified.stl",
                    RightJawMeshFile = "pneumatic_gripper_horizontal_right_jaw_simplified.stl",
                    ToolLengthMm = 82.0,
                    JawSlideAxis = "X",
                    MaxJawStrokeMm = 8.0
                },
                new EndEffectorDefinition
                {
                    Id = "pneumatic_vertical",
                    DisplayName = "Pneumatic Gripper — Vertical Jaws (55 mm)",
                    BodyMeshFile = "pneumatic_gripper_vertical_body_simplified.stl",
                    LeftJawMeshFile = "pneumatic_gripper_vertical_left_jaw_simplified.stl",
                    RightJawMeshFile = "pneumatic_gripper_vertical_right_jaw_simplified.stl",
                    ToolLengthMm = 55.0,
                    JawSlideAxis = "Y",
                    MaxJawStrokeMm = 8.0
                },
                new EndEffectorDefinition
                {
                    Id = "pneumatic_assembly",
                    DisplayName = "Pneumatic Gripper — Full Assembly (85 mm)",
                    BodyMeshFile = "pneumatic_gripper_assembly_simplified.stl",
                    ToolLengthMm = 85.0,
                    JawSlideAxis = "NONE",
                    MaxJawStrokeMm = 0.0
                },
                new EndEffectorDefinition
                {
                    Id = "vacuum_gripper",
                    DisplayName = "Vacuum Suction Gripper (60 mm)",
                    BodyMeshFile = "vacuum_gripper_body_simplified.stl",
                    ToolLengthMm = 60.0,
                    JawSlideAxis = "NONE",
                    MaxJawStrokeMm = 0.0
                },
                new EndEffectorDefinition
                {
                    Id = "none",
                    DisplayName = "None — L6 Flange Only (0 mm)",
                    BodyMeshFile = null,
                    ToolLengthMm = 0.0,
                    JawSlideAxis = "NONE",
                    MaxJawStrokeMm = 0.0
                }
            };

            // Also auto-discover any custom STL files placed in Assets/meshes that aren't standard links or already listed
            if (Directory.Exists(meshesDir))
            {
                var knownFiles = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
                {
                    "base_link.stl", "base_link_simplified.stl",
                    "l1.stl", "l1_simplified.stl",
                    "l2.stl", "l2_simplified.stl",
                    "l3.stl", "l3_simplified.stl",
                    "l4.stl", "l4_simplified.stl",
                    "l5.stl", "l5_simplified.stl",
                    "l6.stl", "l6_simplified.stl",
                    "end_effector_td-8820mg.stl"
                };

                foreach (var item in list)
                {
                    if (!string.IsNullOrEmpty(item.BodyMeshFile)) knownFiles.Add(item.BodyMeshFile);
                    if (!string.IsNullOrEmpty(item.LeftJawMeshFile)) knownFiles.Add(item.LeftJawMeshFile);
                    if (!string.IsNullOrEmpty(item.RightJawMeshFile)) knownFiles.Add(item.RightJawMeshFile);
                }

                foreach (string file in Directory.GetFiles(meshesDir, "*.stl"))
                {
                    string fileName = Path.GetFileName(file);
                    string lower = fileName.ToLowerInvariant();
                    if (knownFiles.Contains(fileName) ||
                        lower.StartsWith("ssg48_") ||
                        lower.StartsWith("msg_ai_") ||
                        lower.StartsWith("pneumatic_gripper_") ||
                        lower.StartsWith("vacuum_gripper_"))
                    {
                        continue;
                    }

                    list.Insert(list.Count - 1, new EndEffectorDefinition
                    {
                        Id = Path.GetFileNameWithoutExtension(fileName),
                        DisplayName = $"{Path.GetFileNameWithoutExtension(fileName)} (Custom STL | 118 mm)",
                        BodyMeshFile = fileName,
                        ToolLengthMm = 118.0,
                        JawSlideAxis = "Y",
                        MaxJawStrokeMm = 18.0,
                        IsMillimeterUnits = true
                    });
                }
            }

            return list;
        }

        /// <summary>
        /// Loads the STL mesh files from the application Assets directory and builds the default TD-8820MG end effector.
        /// </summary>
        public void LoadMeshes(string assetsPath)
        {
            _meshesDir = Path.Combine(assetsPath, "meshes");
            if (!Directory.Exists(_meshesDir))
            {
                _meshesDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "meshes");
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

            Color ghostColor = Color.FromArgb(90, 0, 220, 255);

            _liveRobotGroup.Children.Clear();
            _ghostRobotGroup.Children.Clear();

            for (int i = 0; i < 7; i++)
            {
                var reader = new StLReader();
                string filePath = Path.Combine(_meshesDir, fileNames[i]);
                if (!File.Exists(filePath))
                {
                    filePath = Path.Combine(_meshesDir, fallbackFileNames[i]);
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

                if (loadedMesh == null)
                {
                    loadedMesh = CreateFallbackGeometry(i);
                }

                var meshTransform = new Transform3DGroup();
                meshTransform.Children.Add(new ScaleTransform3D(1000, 1000, 1000));
                if (i == 6)
                {
                    meshTransform.Children.Add(new TranslateTransform3D(0, 0, -21.0));
                }

                var liveModel = CloneModelWithMaterial(loadedMesh, MaterialHelper.CreateMaterial(new SolidColorBrush(liveColors[i]), 0.8, 60));
                liveModel.Transform = meshTransform;

                var liveVisual = new ModelVisual3D { Content = liveModel };
                liveVisual.Transform = new Transform3DGroup();
                _liveLinks[i] = liveVisual;
                _liveRobotGroup.Children.Add(liveVisual);

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

            _liveRobotGroup.Children.Add(_liveToolRoot);
            if (_ghostVisible)
            {
                _ghostRobotGroup.Children.Add(_ghostToolRoot);
            }

            var catalog = GetAvailableEndEffectors(assetsPath);
            SetEndEffector(catalog.FirstOrDefault());
        }

        /// <summary>
        /// Switches the active 3D end-effector visual and updates the kinematic tool offset (e.g. -118 mm for TD-8820MG).
        /// </summary>
        public void SetEndEffector(EndEffectorDefinition def)
        {
            if (def == null) return;
            _activeEndEffector = def;
            _kinematics.ToolOffsetZ = -Math.Abs(def.ToolLengthMm);

            _liveBodyGroup = new Model3DGroup();
            _liveLeftJawGroup = new Model3DGroup();
            _liveRightJawGroup = new Model3DGroup();
            _livePinionGroup = new Model3DGroup();
            _liveCameraFovGroup = new Model3DGroup();

            _ghostBodyGroup = new Model3DGroup();
            _ghostLeftJawGroup = new Model3DGroup();
            _ghostRightJawGroup = new Model3DGroup();
            _ghostPinionGroup = new Model3DGroup();

            Material ghostMat = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(85, 0, 220, 255)));

            if (def.IsTd8820Custom)
            {
                BuildTd8820CustomEndEffector(ghostMat);
            }
            else if (!string.IsNullOrEmpty(def.BodyMeshFile))
            {
                BuildStandardStlEndEffector(def, ghostMat);
            }

            var liveComposite = new Model3DGroup();
            liveComposite.Children.Add(_liveBodyGroup);
            liveComposite.Children.Add(_livePinionGroup);
            liveComposite.Children.Add(_liveLeftJawGroup);
            liveComposite.Children.Add(_liveRightJawGroup);
            if (_showCameraFovCone)
            {
                liveComposite.Children.Add(_liveCameraFovGroup);
            }
            _liveToolRoot.Content = liveComposite;

            var ghostComposite = new Model3DGroup();
            ghostComposite.Children.Add(_ghostBodyGroup);
            ghostComposite.Children.Add(_ghostPinionGroup);
            ghostComposite.Children.Add(_ghostLeftJawGroup);
            ghostComposite.Children.Add(_ghostRightJawGroup);
            _ghostToolRoot.Content = ghostComposite;

            UpdateGripperTransforms(_lastTargetGripperDeg, _lastActualGripperDeg);
            UpdatePoses(_lastLiveJoints, _lastTargetJoints);
        }

        /// <summary>
        /// Builds the custom TD-8820MG end-effector visual:
        /// 1. Loads End_effector_TD-8820MG.stl and centers its 32 mm mounting flange at (0, 0, 0) on L6 (+22.57, +59.16, +8.50 mm).
        /// 2. Decomposes the 5 connected shells inside the STL:
        ///    - Shell 0: Main TD-8820MG Housing
        ///    - Shell 1: Central Pinion Gear (rotates with servo angle)
        ///    - Shell 2 & 3: Right (+Y) & Left (-Y) Rack-and-Pinion Sliding Fingers
        ///    - Shell 4: Top Camera Module mounted at 30° downward
        /// 3. Adds animated high-contrast sliding finger tip pads (reaching Z = -118 mm) and a 30° downward camera lens + FOV frustum cone.
        /// </summary>
        private void BuildTd8820CustomEndEffector(Material ghostMat)
        {
            Material bodyMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(46, 56, 70)), 0.7, 50);
            Material pinionMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(235, 170, 45)), 0.9, 80);
            Material fingerMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(195, 208, 220)), 0.85, 70);
            Material padMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(0, 195, 255)), 0.8, 60);
            Material camBodyMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(24, 30, 38)), 0.6, 40);
            Material camLensMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(0, 230, 255)), 0.95, 100);
            Material camFovMat = new DiffuseMaterial(new SolidColorBrush(Color.FromArgb(42, 0, 230, 255)));

            bool loadedFromStl = false;
            string stlPath = Path.Combine(_meshesDir ?? "", "End_effector_TD-8820MG.stl");

            if (File.Exists(stlPath))
            {
                try
                {
                    var reader = new StLReader();
                    var rawGroup = reader.Read(stlPath);
                    var shells = SplitConnectedShellsWithOffset(rawGroup, 22.57, 59.16, 8.50);

                    if (shells.Count >= 4)
                    {
                        loadedFromStl = true;
                        // Sort shells by triangle count descending
                        shells.Sort((a, b) => b.TriangleIndices.Count.CompareTo(a.TriangleIndices.Count));

                        // Shell 0 (largest): Main TD-8820MG Housing
                        AddMeshToPair(shells[0], bodyMat, ghostMat, _liveBodyGroup, _ghostBodyGroup);

                        // Classify remaining shells by bounding box
                        for (int i = 1; i < shells.Count; i++)
                        {
                            var m = shells[i];
                            Rect3D b = m.Bounds;

                            if (b.Z + b.SizeZ > -65.0 && b.Z > -65.0 && Math.Abs(b.X + b.SizeX * 0.5) < 15.0)
                            {
                                // Central Pinion Gear (X in [-11.2, 11.2], Y in [-11.2, 11.2], Z in [-62, -54])
                                AddMeshToPair(m, pinionMat, ghostMat, _livePinionGroup, _ghostPinionGroup);
                            }
                            else if (b.X > 22.0)
                            {
                                // Top Camera Module (X in [30.6, 57.9], looking 30° downward)
                                AddMeshToPair(m, camBodyMat, ghostMat, _liveBodyGroup, _ghostBodyGroup);
                            }
                            else
                            {
                                // Sliding Finger & Rack: classify +Y vs -Y by average Y of fingertip vertices (Z < -85)
                                double sumY = 0;
                                int cntY = 0;
                                foreach (var pt in m.Positions)
                                {
                                    if (pt.Z < -85.0)
                                    {
                                        sumY += pt.Y;
                                        cntY++;
                                    }
                                }
                                double avgY = cntY > 0 ? sumY / cntY : (b.Y + b.SizeY * 0.5);
                                if (avgY >= 0)
                                {
                                    AddMeshToPair(m, fingerMat, ghostMat, _liveRightJawGroup, _ghostRightJawGroup);
                                }
                                else
                                {
                                    AddMeshToPair(m, fingerMat, ghostMat, _liveLeftJawGroup, _ghostLeftJawGroup);
                                }
                            }
                        }
                    }
                }
                catch
                {
                    loadedFromStl = false;
                }
            }

            if (!loadedFromStl)
            {
                // Clean procedural TD-8820MG Gripper Body + Sliding Fingers if STL is missing
                var bodyBuilder = new MeshBuilder(false, false);
                bodyBuilder.AddCylinder(new Point3D(0, 0, 0), new Point3D(0, 0, -14), 16.0, 24);
                bodyBuilder.AddBox(new Point3D(8, 0, -44), 52, 76, 52);
                var bodyMesh = bodyBuilder.ToMesh();
                AddMeshToPair(bodyMesh, bodyMat, ghostMat, _liveBodyGroup, _ghostBodyGroup);

                var pinionBuilder = new MeshBuilder(false, false);
                pinionBuilder.AddCylinder(new Point3D(0, 0, -54), new Point3D(0, 0, -62), 11.0, 20);
                AddMeshToPair(pinionBuilder.ToMesh(), pinionMat, ghostMat, _livePinionGroup, _ghostPinionGroup);

                var leftBuilder = new MeshBuilder(false, false);
                leftBuilder.AddBox(new Point3D(0, -6.0, -90), 22, 9.0, 56);
                AddMeshToPair(leftBuilder.ToMesh(), fingerMat, ghostMat, _liveLeftJawGroup, _ghostLeftJawGroup);

                var rightBuilder = new MeshBuilder(false, false);
                rightBuilder.AddBox(new Point3D(0, 6.0, -90), 22, 9.0, 56);
                AddMeshToPair(rightBuilder.ToMesh(), fingerMat, ghostMat, _liveRightJawGroup, _ghostRightJawGroup);
            }

            // Add high-contrast animated inner fingertip grip pads reaching exact Z = -118.0 mm TCP
            var leftPadBuilder = new MeshBuilder(false, false);
            leftPadBuilder.AddBox(new Point3D(0, -1.6, -104.0), 18.0, 2.4, 28.0);
            AddMeshToPair(leftPadBuilder.ToMesh(), padMat, ghostMat, _liveLeftJawGroup, _ghostLeftJawGroup);

            var rightPadBuilder = new MeshBuilder(false, false);
            rightPadBuilder.AddBox(new Point3D(0, 1.6, -104.0), 18.0, 2.4, 28.0);
            AddMeshToPair(rightPadBuilder.ToMesh(), padMat, ghostMat, _liveRightJawGroup, _ghostRightJawGroup);

            // Add 30° downward-looking optical camera lens & sight frustum on top of the gripper (+X side)
            // Camera optical origin at (46.2, 0, -68.0), optical axis pointing 30° inward/downward toward (0, 0, -118.0)
            double tiltRad = 30.0 * Math.PI / 180.0;
            Vector3D camDir = new Vector3D(-Math.Sin(tiltRad), 0, -Math.Cos(tiltRad)); // (-0.5, 0, -0.866)
            Point3D lensStart = new Point3D(46.2, 0, -66.0);
            Point3D lensEnd = lensStart + camDir * 7.0;

            var lensBuilder = new MeshBuilder(false, false);
            lensBuilder.AddCylinder(lensStart, lensEnd, 6.5, 24);
            AddMeshToPair(lensBuilder.ToMesh(), camLensMat, ghostMat, _liveBodyGroup, _ghostBodyGroup);

            // Translucent 30° camera FOV cone showing camera sightline toward the fingertips
            var fovBuilder = new MeshBuilder(false, false);
            Point3D fovEnd = lensEnd + camDir * 52.0; // Reaches right to the gripping zone at Z ≈ -118 mm
            fovBuilder.AddCone(lensEnd, camDir, 4.5, 22.0, 52.0, false, false, 24);
            var fovGeom = new GeometryModel3D(fovBuilder.ToMesh(), camFovMat) { BackMaterial = camFovMat };
            _liveCameraFovGroup.Children.Add(fovGeom);
        }

        private void BuildStandardStlEndEffector(EndEffectorDefinition def, Material ghostMat)
        {
            double scale = def.IsMillimeterUnits ? 1.0 : 1000.0;
            Material bodyMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(46, 56, 70)), 0.75, 50);
            Material jawMat = MaterialHelper.CreateMaterial(new SolidColorBrush(Color.FromRgb(0, 190, 255)), 0.85, 65);

            var bodyMesh = LoadScaledMesh(def.BodyMeshFile, scale);
            if (bodyMesh != null)
            {
                AddMeshToPair(bodyMesh, bodyMat, ghostMat, _liveBodyGroup, _ghostBodyGroup);
            }

            if (!string.IsNullOrEmpty(def.LeftJawMeshFile))
            {
                var leftMesh = LoadScaledMesh(def.LeftJawMeshFile, scale);
                if (leftMesh != null)
                {
                    AddMeshToPair(leftMesh, jawMat, ghostMat, _liveLeftJawGroup, _ghostLeftJawGroup);
                }
            }

            if (!string.IsNullOrEmpty(def.RightJawMeshFile))
            {
                var rightMesh = LoadScaledMesh(def.RightJawMeshFile, scale);
                if (rightMesh != null)
                {
                    AddMeshToPair(rightMesh, jawMat, ghostMat, _liveRightJawGroup, _ghostRightJawGroup);
                }
            }
        }

        private MeshGeometry3D LoadScaledMesh(string fileName, double scale)
        {
            if (string.IsNullOrEmpty(fileName) || string.IsNullOrEmpty(_meshesDir)) return null;
            string path = Path.Combine(_meshesDir, fileName);
            if (!File.Exists(path)) return null;

            try
            {
                var reader = new StLReader();
                var group = reader.Read(path);
                var merged = new MeshGeometry3D();
                int baseIdx = 0;

                foreach (var child in group.Children)
                {
                    if (child is GeometryModel3D gm && gm.Geometry is MeshGeometry3D mg)
                    {
                        foreach (var pt in mg.Positions)
                        {
                            merged.Positions.Add(new Point3D(pt.X * scale, pt.Y * scale, pt.Z * scale));
                        }
                        foreach (int idx in mg.TriangleIndices)
                        {
                            merged.TriangleIndices.Add(baseIdx + idx);
                        }
                        baseIdx = merged.Positions.Count;
                    }
                }
                return merged;
            }
            catch
            {
                return null;
            }
        }

        private static void AddMeshToPair(
            MeshGeometry3D mesh,
            Material liveMat,
            Material ghostMat,
            Model3DGroup liveTarget,
            Model3DGroup ghostTarget)
        {
            if (mesh == null) return;
            liveTarget.Children.Add(new GeometryModel3D(mesh, liveMat) { BackMaterial = liveMat });
            ghostTarget.Children.Add(new GeometryModel3D(mesh, ghostMat) { BackMaterial = ghostMat });
        }

        /// <summary>
        /// Splits a multi-body STL Model3DGroup into disconnected triangle shells while applying a translation offset.
        /// Used to isolate the 5 mechanical parts inside End_effector_TD-8820MG.stl in < 8 ms.
        /// </summary>
        private static List<MeshGeometry3D> SplitConnectedShellsWithOffset(Model3DGroup group, double dx, double dy, double dz)
        {
            var allPts = new List<Point3D>();
            foreach (var child in group.Children)
            {
                if (child is GeometryModel3D gm && gm.Geometry is MeshGeometry3D mg)
                {
                    if (mg.TriangleIndices != null && mg.TriangleIndices.Count >= 3)
                    {
                        for (int i = 0; i + 2 < mg.TriangleIndices.Count; i += 3)
                        {
                            Point3D p0 = mg.Positions[mg.TriangleIndices[i]];
                            Point3D p1 = mg.Positions[mg.TriangleIndices[i + 1]];
                            Point3D p2 = mg.Positions[mg.TriangleIndices[i + 2]];
                            allPts.Add(new Point3D(p0.X + dx, p0.Y + dy, p0.Z + dz));
                            allPts.Add(new Point3D(p1.X + dx, p1.Y + dy, p1.Z + dz));
                            allPts.Add(new Point3D(p2.X + dx, p2.Y + dy, p2.Z + dz));
                        }
                    }
                    else
                    {
                        foreach (var p in mg.Positions)
                        {
                            allPts.Add(new Point3D(p.X + dx, p.Y + dy, p.Z + dz));
                        }
                    }
                }
            }

            int numTris = allPts.Count / 3;
            var vertToTris = new Dictionary<(int, int, int), List<int>>(numTris);
            for (int t = 0; t < numTris; t++)
            {
                for (int k = 0; k < 3; k++)
                {
                    Point3D p = allPts[t * 3 + k];
                    var key = ((int)Math.Round(p.X * 50.0), (int)Math.Round(p.Y * 50.0), (int)Math.Round(p.Z * 50.0));
                    if (!vertToTris.TryGetValue(key, out var list))
                    {
                        list = new List<int>(4);
                        vertToTris[key] = list;
                    }
                    list.Add(t);
                }
            }

            bool[] visited = new bool[numTris];
            var shells = new List<MeshGeometry3D>();
            var queue = new Queue<int>();

            for (int i = 0; i < numTris; i++)
            {
                if (visited[i]) continue;
                visited[i] = true;
                queue.Enqueue(i);

                var shell = new MeshGeometry3D();
                int idxCounter = 0;

                while (queue.Count > 0)
                {
                    int cur = queue.Dequeue();
                    for (int k = 0; k < 3; k++)
                    {
                        Point3D p = allPts[cur * 3 + k];
                        shell.Positions.Add(p);
                        shell.TriangleIndices.Add(idxCounter++);

                        var key = ((int)Math.Round(p.X * 50.0), (int)Math.Round(p.Y * 50.0), (int)Math.Round(p.Z * 50.0));
                        if (vertToTris.TryGetValue(key, out var neighbors))
                        {
                            foreach (int nb in neighbors)
                            {
                                if (!visited[nb])
                                {
                                    visited[nb] = true;
                                    queue.Enqueue(nb);
                                }
                            }
                        }
                    }
                }

                shells.Add(shell);
            }

            return shells;
        }

        /// <summary>
        /// Updates the sliding finger transforms and pinion rotation for both the Target Twin and Live Feedback Ghost.
        /// 0° = Closed (0 mm shift), 180° = Fully Open (MaxJawStrokeMm shift per finger).
        /// </summary>
        public void UpdateGripperTransforms(double targetGripperDeg, double actualGripperDeg)
        {
            _lastTargetGripperDeg = Math.Clamp(targetGripperDeg, 0.0, 180.0);
            _lastActualGripperDeg = Math.Clamp(actualGripperDeg, 0.0, 180.0);

            if (_activeEndEffector == null) return;

            double maxStroke = _activeEndEffector.MaxJawStrokeMm;
            double targetRatio = _lastTargetGripperDeg / 180.0;
            double actualRatio = _lastActualGripperDeg / 180.0;

            double targetShift = targetRatio * maxStroke;
            double actualShift = actualRatio * maxStroke;

            if (_activeEndEffector.JawSlideAxis == "Y")
            {
                if (_liveLeftJawGroup != null) _liveLeftJawGroup.Transform = new TranslateTransform3D(0, -targetShift, 0);
                if (_liveRightJawGroup != null) _liveRightJawGroup.Transform = new TranslateTransform3D(0, +targetShift, 0);
                if (_ghostLeftJawGroup != null) _ghostLeftJawGroup.Transform = new TranslateTransform3D(0, -actualShift, 0);
                if (_ghostRightJawGroup != null) _ghostRightJawGroup.Transform = new TranslateTransform3D(0, +actualShift, 0);
            }
            else if (_activeEndEffector.JawSlideAxis == "X")
            {
                if (_liveLeftJawGroup != null) _liveLeftJawGroup.Transform = new TranslateTransform3D(-targetShift, 0, 0);
                if (_liveRightJawGroup != null) _liveRightJawGroup.Transform = new TranslateTransform3D(+targetShift, 0, 0);
                if (_ghostLeftJawGroup != null) _ghostLeftJawGroup.Transform = new TranslateTransform3D(-actualShift, 0, 0);
                if (_ghostRightJawGroup != null) _ghostRightJawGroup.Transform = new TranslateTransform3D(+actualShift, 0, 0);
            }

            if (_livePinionGroup != null)
            {
                _livePinionGroup.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), _lastTargetGripperDeg - 90.0));
            }
            if (_ghostPinionGroup != null)
            {
                _ghostPinionGroup.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), _lastActualGripperDeg - 90.0));
            }
        }

        /// <summary>
        /// Updates the 3D joint poses of both the interactive digital twin (solid) and physical feedback (ghost).
        /// </summary>
        public void UpdatePoses(double[] liveJointsDeg, double[] targetJointsDeg)
        {
            if (targetJointsDeg != null)
            {
                _lastTargetJoints = (double[])targetJointsDeg.Clone();
                var targetTransforms = _kinematics.ComputeAllLinkTransforms(targetJointsDeg);
                for (int i = 0; i < 7; i++)
                {
                    if (_liveLinks[i] != null)
                    {
                        _liveLinks[i].Transform = ToWpfMatrixTransform(targetTransforms[i]);
                    }
                }
                _liveToolRoot.Transform = ToWpfMatrixTransform(targetTransforms[6]);
            }

            if (liveJointsDeg != null)
            {
                _lastLiveJoints = (double[])liveJointsDeg.Clone();
                var liveTransforms = _kinematics.ComputeAllLinkTransforms(liveJointsDeg);
                for (int i = 0; i < 7; i++)
                {
                    if (_ghostLinks[i] != null)
                    {
                        _ghostLinks[i].Transform = ToWpfMatrixTransform(liveTransforms[i]);
                    }
                }
                _ghostToolRoot.Transform = ToWpfMatrixTransform(liveTransforms[6]);
            }
        }

        private static MatrixTransform3D ToWpfMatrixTransform(in RobotKinematics.Mat4 m)
        {
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
