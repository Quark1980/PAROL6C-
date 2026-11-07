using System;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using PAROLC_ControlSuite.Controls;
using PAROLC_ControlSuite.Models;
using PAROLC_ControlSuite.ViewModels;

namespace PAROLC_ControlSuite
{
    public partial class MainWindow : Window
    {
        private readonly MainViewModel _vm;
        private readonly RobotMeshVisualizer _visualizer;
        private readonly RobotGizmoVisual3D _gizmo;
        private readonly TrajectoryTrailVisual3D _trailVisual;
        private Storyboard _estopStoryboard;

        // Gizmo interactive dragging state
        private bool _isDraggingGizmo = false;
        private string _activeDragAxis = null;
        private Point _dragStartMouse;
        private Point3D _dragStartGizmoPos;

        public MainWindow()
        {
            InitializeComponent();

            _vm = new MainViewModel();
            this.DataContext = _vm;

            // Initialize 3D robot mesh visualizer
            _visualizer = new RobotMeshVisualizer(_vm.Kinematics);
            string assetsDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets");
            _visualizer.LoadMeshes(assetsDir);

            // Add 3D models to Helix viewport
            Viewport3D.Children.Add(_visualizer.LiveVisual);
            Viewport3D.Children.Add(_visualizer.GhostVisual);
            _visualizer.GhostVisible = true;

            // Add 3D Trajectory Trail / Planned Path Visual
            _trailVisual = new TrajectoryTrailVisual3D();
            Viewport3D.Children.Add(_trailVisual);

            // Add 3D TCP Transform Gizmo
            _gizmo = new RobotGizmoVisual3D();
            Viewport3D.Children.Add(_gizmo);

            // Subscribe to pose and trail updates from ViewModel
            _vm.VisualsUpdated += OnVisualsUpdated;
            _vm.TrailUpdated += (pts, isFollow) => Dispatcher.Invoke(() => _trailVisual.UpdateTrail(pts, isFollow));
            _vm.PropertyChanged += OnViewModelPropertyChanged;

            // Setup default camera looking at the robot
            ResetCameraIsometric();

            // Initial visual update
            _visualizer.UpdatePoses(_vm.GetActualJoints(), _vm.GetTargetJoints());
            _gizmo.SetPose(_vm.TargetX, _vm.TargetY, _vm.TargetZ);

            // Spacebar emergency stop hotkey
            this.PreviewKeyDown += OnWindowPreviewKeyDown;

            // Pre-load E-Stop Storyboard
            _estopStoryboard = TryFindResource("EstopPulseAnimation") as Storyboard;
        }

        private void OnViewModelPropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(MainViewModel.IsEstopActive))
            {
                Dispatcher.Invoke(() =>
                {
                    if (_vm.IsEstopActive)
                    {
                        _estopStoryboard?.Begin(this, true);
                    }
                    else
                    {
                        _estopStoryboard?.Stop(this);
                        if (EstopPulsingOverlay != null) EstopPulsingOverlay.Opacity = 0.0;
                        if (EstopPulseBorder != null) EstopPulseBorder.Opacity = 0.0;
                    }
                });
            }
        }

        private void OnVisualsUpdated(double[] liveJoints, double[] targetJoints)
        {
            _visualizer.UpdatePoses(liveJoints, targetJoints);
            _gizmo.SetPose(_vm.TargetX, _vm.TargetY, _vm.TargetZ);
        }

        private async void OnWindowPreviewKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Space)
            {
                e.Handled = true;
                await _vm.TriggerEstopAsync();
            }
        }

        #region Camera Navigation

        private void ResetCameraIsometric()
        {
            var camera = new PerspectiveCamera
            {
                Position = new Point3D(550, -650, 480),
                LookDirection = new Vector3D(-550, 650, -320),
                UpDirection = new Vector3D(0, 0, 1),
                FieldOfView = 50,
                NearPlaneDistance = 1.0,
                FarPlaneDistance = 5000.0
            };
            Viewport3D.Camera = camera;
        }

        private void BtnCamIso_Click(object sender, RoutedEventArgs e) => ResetCameraIsometric();

        private void BtnCamFront_Click(object sender, RoutedEventArgs e)
        {
            Viewport3D.Camera = new PerspectiveCamera
            {
                Position = new Point3D(0, -900, 250),
                LookDirection = new Vector3D(0, 900, -50),
                UpDirection = new Vector3D(0, 0, 1),
                FieldOfView = 50,
                NearPlaneDistance = 1.0,
                FarPlaneDistance = 5000.0
            };
        }

        private void BtnCamTop_Click(object sender, RoutedEventArgs e)
        {
            Viewport3D.Camera = new PerspectiveCamera
            {
                Position = new Point3D(0, 0, 950),
                LookDirection = new Vector3D(0, 0, -950),
                UpDirection = new Vector3D(0, 1, 0),
                FieldOfView = 50,
                NearPlaneDistance = 1.0,
                FarPlaneDistance = 5000.0
            };
        }

        private void BtnCamSide_Click(object sender, RoutedEventArgs e)
        {
            Viewport3D.Camera = new PerspectiveCamera
            {
                Position = new Point3D(900, 0, 250),
                LookDirection = new Vector3D(-900, 0, -50),
                UpDirection = new Vector3D(0, 0, 1),
                FieldOfView = 50,
                NearPlaneDistance = 1.0,
                FarPlaneDistance = 5000.0
            };
        }

        private void ChkShowGhost_Changed(object sender, RoutedEventArgs e)
        {
            if (_visualizer != null)
            {
                _visualizer.GhostVisible = (ChkShowGhost.IsChecked == true);
            }
        }

        #endregion

        #region Interactive 3D Transform Gizmo Raycasting & Dragging

        private void Viewport3D_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed && !_vm.IsEstopActive)
            {
                Point mousePos = e.GetPosition(Viewport3D);

                // Raycast hit-test against gizmo handles
                HitTestResult hitResult = VisualTreeHelper.HitTest(Viewport3D, mousePos);
                if (hitResult is RayMeshGeometry3DHitTestResult rayHit)
                {
                    var hitModel = rayHit.ModelHit as GeometryModel3D;
                    if (hitModel != null)
                    {
                        if (hitModel == _gizmo.HandleX) _activeDragAxis = "X";
                        else if (hitModel == _gizmo.HandleY) _activeDragAxis = "Y";
                        else if (hitModel == _gizmo.HandleZ) _activeDragAxis = "Z";
                        else if (hitModel == _gizmo.CenterHandle) _activeDragAxis = "XYZ";

                        if (_activeDragAxis != null)
                        {
                            _isDraggingGizmo = true;
                            _dragStartMouse = mousePos;
                            _dragStartGizmoPos = _gizmo.Position;
                            _vm.OnInteractiveDragStarted();
                            Viewport3D.CaptureMouse();
                            e.Handled = true;
                        }
                    }
                }
            }
        }

        private void Viewport3D_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            if (_isDraggingGizmo && _activeDragAxis != null && !_vm.IsEstopActive)
            {
                Point curPos = e.GetPosition(Viewport3D);
                double deltaScreenX = curPos.X - _dragStartMouse.X;
                double deltaScreenY = curPos.Y - _dragStartMouse.Y;

                // Scale factor from screen pixels to millimeters based on camera distance
                double mmPerPixel = 0.8;

                double newX = _dragStartGizmoPos.X;
                double newY = _dragStartGizmoPos.Y;
                double newZ = _dragStartGizmoPos.Z;

                if (_activeDragAxis == "X")
                {
                    newX += deltaScreenX * mmPerPixel;
                }
                else if (_activeDragAxis == "Y")
                {
                    newY += deltaScreenX * mmPerPixel;
                }
                else if (_activeDragAxis == "Z")
                {
                    newZ -= deltaScreenY * mmPerPixel; // Screen Y is inverted
                }
                else if (_activeDragAxis == "XYZ")
                {
                    newX += deltaScreenX * mmPerPixel;
                    newZ -= deltaScreenY * mmPerPixel;
                }

                _vm.OnGizmoMoved(newX, newY, newZ);
                e.Handled = true;
            }
        }

        private void Viewport3D_PreviewMouseUp(object sender, MouseButtonEventArgs e)
        {
            if (_isDraggingGizmo)
            {
                _isDraggingGizmo = false;
                _activeDragAxis = null;
                Viewport3D.ReleaseMouseCapture();
                _vm.OnInteractiveDragCompleted();
                e.Handled = true;
            }
        }

        private void Slider_DragStarted(object sender, System.Windows.Controls.Primitives.DragStartedEventArgs e)
        {
            _vm.OnInteractiveDragStarted();
        }

        private void Slider_DragCompleted(object sender, System.Windows.Controls.Primitives.DragCompletedEventArgs e)
        {
            _vm.OnInteractiveDragCompleted();
        }

        private void BtnToolVertical_Click(object sender, RoutedEventArgs e)
        {
            _vm.SetToolVerticalDown();
        }

        private void BtnToolHorizontal_Click(object sender, RoutedEventArgs e)
        {
            _vm.SetToolHorizontalForward();
        }

        private void BtnLockCurrentAngle_Click(object sender, RoutedEventArgs e)
        {
            _vm.LockCurrentToolOrientation();
        }

        #endregion

        #region User Actions & Button Handlers

        private async void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            await _vm.ToggleConnectAsync();
        }

        private async void BtnEstop_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            await _vm.TriggerEstopAsync();
        }

        private async void BtnReset_Click(object sender, RoutedEventArgs e)
        {
            await _vm.TriggerResetAsync();
        }

        private async void BtnHomeAll_Click(object sender, RoutedEventArgs e)
        {
            await _vm.TriggerHomeAsync();
        }

        private async void BtnSetCadHome_Click(object sender, RoutedEventArgs e)
        {
            await _vm.TriggerSetCadHomeAsync();
        }

        private Views.DebugStreamWindow _debugWindow;

        private void BtnDebugStream_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_debugWindow == null)
                {
                    _debugWindow = new Views.DebugStreamWindow(_vm.Client);
                    _debugWindow.Owner = this;
                    _debugWindow.Closed += (s, args) => _debugWindow = null;
                    _debugWindow.Show();
                }
                else
                {
                    _debugWindow.Activate();
                }
            }
            catch (Exception ex)
            {
                _debugWindow = null;
                _vm.AppendLog($"Debug Stream Window Error: {ex.Message}");
            }
        }

        private async void BtnExecutePose_Click(object sender, RoutedEventArgs e)
        {
            await _vm.SendExecutePoseAsync();
        }

        private void BtnSyncGhost_Click(object sender, RoutedEventArgs e)
        {
            _vm.SyncTargetToActual();
        }

        private void BtnCadHome_Click(object sender, RoutedEventArgs e)
        {
            _vm.MoveToCadHome();
        }

        private async void BtnJogJoint_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn && btn.Tag is string tag)
            {
                var parts = tag.Split(',');
                if (parts.Length == 2 && int.TryParse(parts[0], out int j) && double.TryParse(parts[1], out double dir))
                {
                    await _vm.JogJointAsync(j, dir);
                }
            }
        }

        private async void BtnJogCart_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn && btn.Tag is string tag)
            {
                var parts = tag.Split(',');
                if (parts.Length == 2 && double.TryParse(parts[1], out double dir))
                {
                    await _vm.JogCartesianAsync(parts[0], dir);
                }
            }
        }

        private void RdoJointStep_Click(object sender, RoutedEventArgs e)
        {
            if (sender is RadioButton rdo && rdo.Tag is string tag && double.TryParse(tag, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out double val))
            {
                _vm.JointStepSize = val;
            }
        }

        private void RdoCartStep_Click(object sender, RoutedEventArgs e)
        {
            if (sender is RadioButton rdo && rdo.Tag is string tag && double.TryParse(tag, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out double val))
            {
                _vm.CartesianStepSize = val;
            }
        }

        private void BtnTrimNudge_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn && btn.Tag is string tag)
            {
                var parts = tag.Split(',');
                if (parts.Length == 2 && int.TryParse(parts[0], out int j) && double.TryParse(parts[1], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out double delta))
                {
                    _vm.NudgeTrim(j, delta);
                }
            }
        }

        private async void BtnSaveTrims_Click(object sender, RoutedEventArgs e)
        {
            await _vm.SaveTrimsAsync();
        }

        private async void BtnRefreshTrims_Click(object sender, RoutedEventArgs e)
        {
            await _vm.RefreshTrimsAsync();
        }

        private void BtnGripperClose_Click(object sender, RoutedEventArgs e) => _vm.SetGripperImmediate(0);
        private void BtnGripperMid_Click(object sender, RoutedEventArgs e) => _vm.SetGripperImmediate(90);
        private void BtnGripperOpen_Click(object sender, RoutedEventArgs e) => _vm.SetGripperImmediate(180);
        private void GripperSlider_DragCompleted(object sender, System.Windows.Controls.Primitives.DragCompletedEventArgs e) => _vm.OnGripperDragCompleted();

        private void BtnClearLog_Click(object sender, RoutedEventArgs e) => _vm.ClearAllLogs();

        #endregion
    }
}
