using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using PAROLC_ControlSuite.Models;
using PAROLC_ControlSuite.Network;

namespace PAROLC_ControlSuite.ViewModels
{
    public class RelayCommand : ICommand
    {
        private readonly Action<object> _execute;
        private readonly Predicate<object> _canExecute;

        public RelayCommand(Action<object> execute, Predicate<object> canExecute = null)
        {
            _execute = execute ?? throw new ArgumentNullException(nameof(execute));
            _canExecute = canExecute;
        }

        public RelayCommand(Action execute, Func<bool> canExecute = null)
            : this(_ => execute(), canExecute == null ? (Predicate<object>)null : _ => canExecute())
        {
        }

        public bool CanExecute(object parameter) => _canExecute == null || _canExecute(parameter);
        public void Execute(object parameter) => _execute(parameter);
        public event EventHandler CanExecuteChanged
        {
            add => CommandManager.RequerySuggested += value;
            remove => CommandManager.RequerySuggested -= value;
        }
    }

    public class MainViewModel : INotifyPropertyChanged
    {
        private readonly RobotWebSocketClient _client;
        private readonly RobotKinematics _kinematics;
        private readonly Timer _streamingTimer;

        public RobotWebSocketClient Client => _client;
        public RobotKinematics Kinematics => _kinematics;

        #region UI Properties

        private string _piAddress = "192.168.0.159:8765";
        public string PiAddress
        {
            get => _piAddress;
            set => SetProperty(ref _piAddress, value);
        }

        private bool _isConnected;
        public bool IsConnected
        {
            get => _isConnected;
            set
            {
                if (SetProperty(ref _isConnected, value))
                {
                    OnPropertyChanged(nameof(ConnectButtonText));
                    OnPropertyChanged(nameof(IsMotionAllowed));
                }
            }
        }

        public string ConnectButtonText => IsConnected ? "DISCONNECT" : "CONNECT";

        private string _statusText = "Disconnected";
        public string StatusText
        {
            get => _statusText;
            set => SetProperty(ref _statusText, value);
        }

        private double _latencyMs;
        public double LatencyMs
        {
            get => _latencyMs;
            set => SetProperty(ref _latencyMs, value);
        }

        private double _updateRateHz;
        public double UpdateRateHz
        {
            get => _updateRateHz;
            set => SetProperty(ref _updateRateHz, value);
        }

        private string _robotState = "IDLE";
        public string RobotState
        {
            get => _robotState;
            set
            {
                if (SetProperty(ref _robotState, value))
                {
                    IsEstopActive = (value == "ESTOP");
                    OnPropertyChanged(nameof(IsMotionAllowed));
                }
            }
        }

        private bool _isEstopActive = false;
        public bool IsEstopActive
        {
            get => _isEstopActive;
            set
            {
                if (SetProperty(ref _isEstopActive, value))
                {
                    OnPropertyChanged(nameof(IsMotionAllowed));
                }
            }
        }

        private bool _klipperReady = false;
        public bool KlipperReady
        {
            get => _klipperReady;
            set
            {
                if (SetProperty(ref _klipperReady, value))
                {
                    OnPropertyChanged(nameof(IsMotionAllowed));
                }
            }
        }

        private string _klipperStatus = "Klipper: Connecting...";
        public string KlipperStatus
        {
            get => _klipperStatus;
            set => SetProperty(ref _klipperStatus, value);
        }

        private bool _isHomed;
        public bool IsHomed
        {
            get => _isHomed;
            set
            {
                if (SetProperty(ref _isHomed, value))
                {
                    OnPropertyChanged(nameof(IsMotionAllowed));
                }
            }
        }

        public bool IsMotionAllowed => !IsEstopActive && (IsSimulateOnly || (IsConnected && IsHomed && KlipperReady));
        public bool CanSendPose => IsConnected && IsHomed && !IsEstopActive && KlipperReady;

        private string _branchJumpWarning = "";
        public string BranchJumpWarning
        {
            get => _branchJumpWarning;
            set
            {
                if (SetProperty(ref _branchJumpWarning, value))
                {
                    OnPropertyChanged(nameof(HasWarning));
                }
            }
        }

        public bool HasWarning => !string.IsNullOrEmpty(BranchJumpWarning);

        private uint _mcuClock;
        public uint McuClock
        {
            get => _mcuClock;
            set => SetProperty(ref _mcuClock, value);
        }

        // Actual Telemetry Joint Angles (deg) - defaults to CAD Home
        private double _actualJ1 = 0.0, _actualJ2 = -90.0, _actualJ3 = 180.0, _actualJ4 = 0.0, _actualJ5 = 0.0, _actualJ6 = 180.0;
        public double ActualJ1 { get => _actualJ1; set => SetProperty(ref _actualJ1, value); }
        public double ActualJ2 { get => _actualJ2; set => SetProperty(ref _actualJ2, value); }
        public double ActualJ3 { get => _actualJ3; set => SetProperty(ref _actualJ3, value); }
        public double ActualJ4 { get => _actualJ4; set => SetProperty(ref _actualJ4, value); }
        public double ActualJ5 { get => _actualJ5; set => SetProperty(ref _actualJ5, value); }
        public double ActualJ6 { get => _actualJ6; set => SetProperty(ref _actualJ6, value); }

        // Target Joint Angles (deg)
        private double _targetJ1 = 0.0, _targetJ2 = -90.0, _targetJ3 = 180.0, _targetJ4 = 0.0, _targetJ5 = 0.0, _targetJ6 = 180.0;
        public double TargetJ1 { get => _targetJ1; set { if (SetProperty(ref _targetJ1, value)) OnTargetJointsChanged(); } }
        public double TargetJ2 { get => _targetJ2; set { if (SetProperty(ref _targetJ2, value)) OnTargetJointsChanged(); } }
        public double TargetJ3 { get => _targetJ3; set { if (SetProperty(ref _targetJ3, value)) OnTargetJointsChanged(); } }
        public double TargetJ4 { get => _targetJ4; set { if (SetProperty(ref _targetJ4, value)) OnTargetJointsChanged(); } }
        public double TargetJ5 { get => _targetJ5; set { if (SetProperty(ref _targetJ5, value)) OnTargetJointsChanged(); } }
        public double TargetJ6 { get => _targetJ6; set { if (SetProperty(ref _targetJ6, value)) OnTargetJointsChanged(); } }

        // Actual Cartesian Pose (mm & deg)
        private double _actualX = 341.8, _actualY = 0.0, _actualZ = 334.0, _actualRoll = 0.0, _actualPitch = -90.0, _actualYaw = 0.0;
        public double ActualX { get => _actualX; set => SetProperty(ref _actualX, value); }
        public double ActualY { get => _actualY; set => SetProperty(ref _actualY, value); }
        public double ActualZ { get => _actualZ; set => SetProperty(ref _actualZ, value); }
        public double ActualRoll { get => _actualRoll; set => SetProperty(ref _actualRoll, value); }
        public double ActualPitch { get => _actualPitch; set => SetProperty(ref _actualPitch, value); }
        public double ActualYaw { get => _actualYaw; set => SetProperty(ref _actualYaw, value); }

        // Target Cartesian Pose (mm & deg)
        private double _targetX = 341.8, _targetY = 0.0, _targetZ = 334.0, _targetRoll = 0.0, _targetPitch = -90.0, _targetYaw = 0.0;
        public double TargetX { get => _targetX; set => SetProperty(ref _targetX, value); }
        public double TargetY { get => _targetY; set => SetProperty(ref _targetY, value); }
        public double TargetZ { get => _targetZ; set => SetProperty(ref _targetZ, value); }
        public double TargetRoll { get => _targetRoll; set => SetProperty(ref _targetRoll, value); }
        public double TargetPitch { get => _targetPitch; set => SetProperty(ref _targetPitch, value); }
        public double TargetYaw { get => _targetYaw; set => SetProperty(ref _targetYaw, value); }

        // Persistent Homing Trims (deg)
        private double _trimJ1, _trimJ2, _trimJ3, _trimJ4, _trimJ5, _trimJ6;
        public double TrimJ1 { get => _trimJ1; set => SetProperty(ref _trimJ1, value); }
        public double TrimJ2 { get => _trimJ2; set => SetProperty(ref _trimJ2, value); }
        public double TrimJ3 { get => _trimJ3; set => SetProperty(ref _trimJ3, value); }
        public double TrimJ4 { get => _trimJ4; set => SetProperty(ref _trimJ4, value); }
        public double TrimJ5 { get => _trimJ5; set => SetProperty(ref _trimJ5, value); }
        public double TrimJ6 { get => _trimJ6; set => SetProperty(ref _trimJ6, value); }

        // Gripper (Debounced 25 Hz max + Stationary Interlock) & End Effector Selection
        private double _gripperAngle = 90.0;
        private double _actualGripperAngle = 90.0;
        private long _lastGripperSendTicks = 0;
        private double? _pendingGripperAngle = null;

        public List<EndEffectorDefinition> AvailableEndEffectors { get; } = RobotMeshVisualizer.GetAvailableEndEffectors();

        private EndEffectorDefinition _selectedEndEffector;
        public EndEffectorDefinition SelectedEndEffector
        {
            get => _selectedEndEffector;
            set
            {
                if (SetProperty(ref _selectedEndEffector, value) && value != null)
                {
                    ToolLengthMm = value.ToolLengthMm;
                    EndEffectorChanged?.Invoke(value);
                    AppendLog($"Selected End Effector: {value.DisplayName} (Tool Offset Z = -{value.ToolLengthMm:F1} mm)");
                }
            }
        }

        private double _toolLengthMm = 118.0;
        public double ToolLengthMm
        {
            get => _toolLengthMm;
            set
            {
                double clamped = Math.Clamp(value, 0.0, 300.0);
                if (SetProperty(ref _toolLengthMm, clamped))
                {
                    _kinematics.ToolOffsetZ = -clamped;
                    double[] actualFk = _kinematics.ComputeFkPose(GetActualJoints());
                    ActualX = Math.Round(actualFk[0], 2);
                    ActualY = Math.Round(actualFk[1], 2);
                    ActualZ = Math.Round(actualFk[2], 2);
                    ActualRoll = Math.Round(actualFk[3], 2);
                    ActualPitch = Math.Round(actualFk[4], 2);
                    ActualYaw = Math.Round(actualFk[5], 2);

                    UpdateTargetFk();
                    _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
                    VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
                }
            }
        }

        public double GripperAngle
        {
            get => _gripperAngle;
            set
            {
                double clamped = Math.Clamp(value, 0.0, 180.0);
                if (SetProperty(ref _gripperAngle, clamped))
                {
                    GripperMm = Math.Round(62.5 * (clamped / 180.0), 1);
                    GripperVisualsUpdated?.Invoke(_gripperAngle, _actualGripperAngle);
                    RequestGripperDispatch(clamped, forceImmediate: false);
                }
            }
        }

        public void SetGripperImmediate(double angleDeg)
        {
            double clamped = Math.Clamp(angleDeg, 0.0, 180.0);
            _gripperAngle = clamped;
            OnPropertyChanged(nameof(GripperAngle));
            GripperMm = Math.Round(62.5 * (clamped / 180.0), 1);
            GripperVisualsUpdated?.Invoke(_gripperAngle, _actualGripperAngle);
            RequestGripperDispatch(clamped, forceImmediate: true);
        }

        public void OnGripperDragCompleted()
        {
            RequestGripperDispatch(_gripperAngle, forceImmediate: true);
        }

        private void RequestGripperDispatch(double angleDeg, bool forceImmediate)
        {
            if (IsSimulateOnly || !IsConnected || IsEstopActive)
                return;

            // Stationary Interlock: only actuate servo when arm is stationary
            bool armMoving = _isExecutingBlendedMove || (IsContinuousStreaming && _isUserDragging) || RobotState == "MOVING" || RobotState == "STREAMING";
            if (armMoving && !forceImmediate)
            {
                _pendingGripperAngle = angleDeg;
                return;
            }

            long nowMs = Environment.TickCount64;
            if (forceImmediate || (nowMs - _lastGripperSendTicks >= 45)) // Max ~22 Hz throttle
            {
                _lastGripperSendTicks = nowMs;
                _pendingGripperAngle = null;
                _ = _client.SendSetGripperAsync(angleDeg);
            }
            else
            {
                _pendingGripperAngle = angleDeg;
            }
        }

        private double _gripperMm = 31.2;
        public double GripperMm
        {
            get => _gripperMm;
            set => SetProperty(ref _gripperMm, value);
        }

        // Jog Configuration
        private double _jointStepSize = 1.0; // deg
        public double JointStepSize
        {
            get => _jointStepSize;
            set => SetProperty(ref _jointStepSize, value);
        }

        private double _cartesianStepSize = 10.0; // mm
        public double CartesianStepSize
        {
            get => _cartesianStepSize;
            set => SetProperty(ref _cartesianStepSize, value);
        }

        private bool _isSimulateOnly = false;
        public bool IsSimulateOnly
        {
            get => _isSimulateOnly;
            set
            {
                if (SetProperty(ref _isSimulateOnly, value) && value)
                {
                    _isSendOnRelease = false;
                    _isContinuousStreaming = false;
                    _isExecutingBlendedMove = false;
                    _streamingTimer.Change(Timeout.Infinite, Timeout.Infinite);
                    OnPropertyChanged(nameof(IsSendOnRelease));
                    OnPropertyChanged(nameof(IsContinuousStreaming));
                    OnPropertyChanged(nameof(IsMotionAllowed));
                    OnPropertyChanged(nameof(CanSendPose));
                    UpdatePlannedPathPreview();
                    AppendLog("Mode: SIMULATE ONLY — Virtual twin & path preview active; zero network packets dispatched.");
                }
            }
        }

        private bool _isSendOnRelease = true;
        public bool IsSendOnRelease
        {
            get => _isSendOnRelease;
            set
            {
                if (SetProperty(ref _isSendOnRelease, value) && value)
                {
                    _isSimulateOnly = false;
                    _isContinuousStreaming = false;
                    OnPropertyChanged(nameof(IsSimulateOnly));
                    OnPropertyChanged(nameof(IsContinuousStreaming));
                    OnPropertyChanged(nameof(IsMotionAllowed));
                    OnPropertyChanged(nameof(CanSendPose));
                    UpdatePlannedPathPreview();
                    AppendLog("Mode: ON RELEASE — Stages target pose & draws planned path; dispatches trajectory on mouse release.");
                }
            }
        }

        private bool _isContinuousStreaming = false;
        public bool IsContinuousStreaming
        {
            get => _isContinuousStreaming;
            set
            {
                if (SetProperty(ref _isContinuousStreaming, value))
                {
                    if (_isContinuousStreaming)
                    {
                        if (IsEstopActive || !IsHomed || !IsConnected)
                        {
                            _isContinuousStreaming = false;
                            OnPropertyChanged(nameof(IsContinuousStreaming));
                            AppendLog("⚠️ Follow Mode rejected: Robot must be connected, homed, and E-Stop cleared first!");
                            return;
                        }
                        _isSimulateOnly = false;
                        _isSendOnRelease = false;
                        OnPropertyChanged(nameof(IsSimulateOnly));
                        OnPropertyChanged(nameof(IsSendOnRelease));
                        OnPropertyChanged(nameof(IsMotionAllowed));
                        SyncTargetToActual();
                        InitializeBlenderFromActual();
                        lock (_trailLock)
                        {
                            _followTrailPoints.Clear();
                            _followSplineWindow.Clear();
                        }
                        TrailUpdated?.Invoke(Array.Empty<System.Windows.Media.Media3D.Point3D>(), true);
                        _streamingTimer.Change(0, 20); // 50 Hz Cubic B-Spline + Flange-Locked Stream
                        AppendLog("Mode: FOLLOW STREAM (50 Hz Sectional Cubic B-Spline Smoothing & Strict Flange Orientation Lock)");
                    }
                    else
                    {
                        if (!_isSimulateOnly && !_isSendOnRelease)
                        {
                            _isSendOnRelease = true;
                            OnPropertyChanged(nameof(IsSendOnRelease));
                        }
                        if (!_isExecutingBlendedMove)
                        {
                            _streamingTimer.Change(Timeout.Infinite, Timeout.Infinite);
                        }
                        OnPropertyChanged(nameof(IsMotionAllowed));
                        UpdatePlannedPathPreview();
                    }
                }
            }
        }

        // IK Orientation Mode: Default TRUE so Flange World Orientation R_start is strictly locked across linear moves
        private bool _isIkLockOrientation = true;
        public bool IsIkLockOrientation
        {
            get => _isIkLockOrientation;
            set
            {
                if (SetProperty(ref _isIkLockOrientation, value))
                {
                    OnPropertyChanged(nameof(IsIkPositionOnly));
                    OnPropertyChanged(nameof(IkModeStatusText));
                    if (_isIkLockOrientation)
                    {
                        LockCurrentToolOrientation();
                    }
                    else
                    {
                        AppendLog("IK Mode: TIP POSITION (Probe / Free Angle) — Keeps tooltip on coordinate with flexible wrist.");
                    }
                }
            }
        }

        public bool IsIkPositionOnly
        {
            get => !_isIkLockOrientation;
            set => IsIkLockOrientation = !value;
        }

        private string _lockedOrientationLabel = "STEADY FLANGE (R:0° P:-90° Y:0°)";
        public string LockedOrientationLabel
        {
            get => _lockedOrientationLabel;
            set => SetProperty(ref _lockedOrientationLabel, value);
        }

        public string IkModeStatusText => IsIkLockOrientation
            ? $"LOCKED TOOL: {LockedOrientationLabel}"
            : "PROBE TIP (Free Angle)";

        // Smoothing & Blending Factor (0.05 = ultra-smooth, 0.30 = snappy)
        private double _smoothingTimeConstant = 0.12;
        public double SmoothingTimeConstant
        {
            get => _smoothingTimeConstant;
            set => SetProperty(ref _smoothingTimeConstant, Math.Clamp(value, 0.04, 0.40));
        }

        // UDP Streaming Network Metrics
        private string _udpRttText = "UDP RTT: --";
        public string UdpRttText { get => _udpRttText; set => SetProperty(ref _udpRttText, value); }

        private string _udpJitterText = "JITTER: --";
        public string UdpJitterText { get => _udpJitterText; set => SetProperty(ref _udpJitterText, value); }

        private string _udpLossText = "LOSS: 0.0%";
        public string UdpLossText { get => _udpLossText; set => SetProperty(ref _udpLossText, value); }

        private string _udpRateText = "UDP RX: --";
        public string UdpRateText { get => _udpRateText; set => SetProperty(ref _udpRateText, value); }

        private double _moveDuration = 0.5;
        public double MoveDuration
        {
            get => _moveDuration;
            set => SetProperty(ref _moveDuration, value);
        }

        // Adjustable Speed Limits (deg/s) & Master Speed Scale
        private double _speedLimitJ1 = 60.0;
        public double SpeedLimitJ1 { get => _speedLimitJ1; set { if (SetProperty(ref _speedLimitJ1, value)) SyncSpeedLimits(); } }
        private double _speedLimitJ2 = 30.0;
        public double SpeedLimitJ2 { get => _speedLimitJ2; set { if (SetProperty(ref _speedLimitJ2, value)) SyncSpeedLimits(); } }
        private double _speedLimitJ3 = 30.0;
        public double SpeedLimitJ3 { get => _speedLimitJ3; set { if (SetProperty(ref _speedLimitJ3, value)) SyncSpeedLimits(); } }
        private double _speedLimitJ4 = 40.0;
        public double SpeedLimitJ4 { get => _speedLimitJ4; set { if (SetProperty(ref _speedLimitJ4, value)) SyncSpeedLimits(); } }
        private double _speedLimitJ5 = 40.0;
        public double SpeedLimitJ5 { get => _speedLimitJ5; set { if (SetProperty(ref _speedLimitJ5, value)) SyncSpeedLimits(); } }
        private double _speedLimitJ6 = 60.0;
        public double SpeedLimitJ6 { get => _speedLimitJ6; set { if (SetProperty(ref _speedLimitJ6, value)) SyncSpeedLimits(); } }

        private double _speedScale = 0.4;
        public double SpeedScale { get => _speedScale; set { if (SetProperty(ref _speedScale, value)) SyncSpeedLimits(); } }

        public void SyncSpeedLimits()
        {
            if (IsConnected)
            {
                double[] speeds = new[] { SpeedLimitJ1, SpeedLimitJ2, SpeedLimitJ3, SpeedLimitJ4, SpeedLimitJ5, SpeedLimitJ6 };
                _ = _client.SendSpeedLimitsAsync(speeds, SpeedScale);
            }
        }

        private string _logFeed = "";
        public string LogFeed
        {
            get => _logFeed;
            set => SetProperty(ref _logFeed, value);
        }

        private string _suiteRpiLogFeed = "";
        public string SuiteRpiLogFeed
        {
            get => _suiteRpiLogFeed;
            set => SetProperty(ref _suiteRpiLogFeed, value);
        }

        private string _rpiMcuLogFeed = "";
        public string RpiMcuLogFeed
        {
            get => _rpiMcuLogFeed;
            set => SetProperty(ref _rpiMcuLogFeed, value);
        }

        private string _lastMcuError = "";
        public string LastMcuError
        {
            get => _lastMcuError;
            set
            {
                if (SetProperty(ref _lastMcuError, value))
                {
                    OnPropertyChanged(nameof(HasMcuError));
                }
            }
        }

        public bool HasMcuError => !string.IsNullOrEmpty(_lastMcuError);
        private int _lastProcessedCommLogId = 0;

        #endregion

        // Action invoked when visual 3D poses need update on UI thread
        public event Action<double[], double[]> VisualsUpdated;

        // Action invoked when active End Effector selection changes
        public event Action<EndEffectorDefinition> EndEffectorChanged;

        // Action invoked when Gripper jaw angle changes: (targetGripperDeg, actualGripperDeg)
        public event Action<double, double> GripperVisualsUpdated;

        // Action invoked when 3D viewport planned path or Follow Mode dynamic trail updates: (waypoints, isFollowTrail)
        public event Action<IReadOnlyList<System.Windows.Media.Media3D.Point3D>, bool> TrailUpdated;

        private bool _initialSyncDone = false;
        private bool _isSyncingTarget = false;
        private bool _isUserDragging = false;
        private bool _isExecutingBlendedMove = false;
        private readonly object _blenderLock = new object();
        private readonly object _trailLock = new object();

        // Follow Mode sliding-window sectional B-spline buffer & 3D trail waypoints
        private readonly List<(double x, double y, double z, long tickMs)> _followSplineWindow = new List<(double x, double y, double z, long tickMs)>();
        private readonly List<(System.Windows.Media.Media3D.Point3D pt, long tickMs)> _followTrailPoints = new List<(System.Windows.Media.Media3D.Point3D pt, long tickMs)>();

        // Client-side trajectory smoothing & second-order velocity/acceleration blender state
        private double[] _blendedJoints = new double[] { 0.0, -90.0, 180.0, 0.0, 0.0, 180.0 };
        private double[] _blendedJointVel = new double[6];
        private double[] _blendedCartPos = new double[] { 341.8, 0.0, 334.0 };
        private double[] _blendedCartVel = new double[3];
        private double _blendedCartSpeed = 0.0;
        private RobotKinematics.Mat4 _lockedRotationMatrix = RobotKinematics.Mat4.Identity;

        private double[] _lastStreamedJoints = new double[] { 0.0, -90.0, 180.0, 0.0, 0.0, 180.0 };
        private int _streamIdleCounter = 0;

        public MainViewModel()
        {
            _kinematics = new RobotKinematics();
            _selectedEndEffector = AvailableEndEffectors.Count > 0 ? AvailableEndEffectors[0] : null;
            if (_selectedEndEffector != null)
            {
                _toolLengthMm = _selectedEndEffector.ToolLengthMm;
                _kinematics.ToolOffsetZ = -_toolLengthMm;
            }
            _client = new RobotWebSocketClient();

            _client.StateChanged += OnConnectionStateChanged;
            _client.TelemetryReceived += OnTelemetryReceived;
            _client.TrimsReceived += OnTrimsReceived;
            _client.LogMessage += AppendLog;
            _client.UdpMetricsUpdated += OnUdpMetricsUpdated;

            // Setup 50 Hz (20 ms) client-side trajectory smoothing & blending timer
            _streamingTimer = new Timer(OnStreamingTick, null, Timeout.Infinite, Timeout.Infinite);

            // Compute initial actual & target poses & locked rotation matrix
            double[] initActualFk = _kinematics.ComputeFkPose(GetActualJoints());
            _actualX = Math.Round(initActualFk[0], 2);
            _actualY = Math.Round(initActualFk[1], 2);
            _actualZ = Math.Round(initActualFk[2], 2);
            _actualRoll = Math.Round(initActualFk[3], 2);
            _actualPitch = Math.Round(initActualFk[4], 2);
            _actualYaw = Math.Round(initActualFk[5], 2);

            UpdateTargetFk();
            _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
        }

        private void InitializeBlenderFromActual()
        {
            lock (_blenderLock)
            {
                double[] actual = GetActualJoints();
                for (int i = 0; i < 6; i++)
                {
                    _blendedJoints[i] = actual[i];
                    _blendedJointVel[i] = 0.0;
                }
                var tcp = _kinematics.ComputeTcpMatrix(actual);
                _blendedCartPos[0] = tcp.M03;
                _blendedCartPos[1] = tcp.M13;
                _blendedCartPos[2] = tcp.M23;
                _blendedCartVel[0] = 0.0;
                _blendedCartVel[1] = 0.0;
                _blendedCartVel[2] = 0.0;
                _blendedCartSpeed = 0.0;
            }
        }

        /// <summary>
        /// Captures and locks the current 3D tool orientation matrix so Cartesian moves and intermediate waypoints
        /// keep the tool steady at this exact angle.
        /// </summary>
        public void LockCurrentToolOrientation(string customLabel = null)
        {
            _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
            var pose = _lockedRotationMatrix.ToPoseMmDeg();
            LockedOrientationLabel = customLabel ?? $"R:{pose.rx:F0}° P:{pose.ry:F0}° Y:{pose.rz:F0}°";
            if (!_isIkLockOrientation)
            {
                _isIkLockOrientation = true;
                OnPropertyChanged(nameof(IsIkLockOrientation));
                OnPropertyChanged(nameof(IsIkPositionOnly));
            }
            OnPropertyChanged(nameof(IkModeStatusText));
            AppendLog($"🔒 Tool Orientation Locked: {LockedOrientationLabel} (Steady attitude active across waypoints)");
        }

        /// <summary>
        /// Orients the tool straight vertically downward (-Z world axis, Roll=0°, Pitch=0°, Yaw=0°) for pen/pick manipulation
        /// and locks the orientation.
        /// </summary>
        public void SetToolVerticalDown()
        {
            if (IsEstopActive) return;

            var vertMat = RobotKinematics.Mat4.FromTranslationAndRpy(TargetX, TargetY, TargetZ, 0.0, 0.0, TargetYaw * Math.PI / 180.0);
            // Seed with J5 = -90° so wrist points straight down cleanly
            double[] seed = GetTargetJoints();
            seed[4] = -(seed[1] + seed[2]); // Approx vertical wrist pitch compensation
            var (ok, solvedQ) = _kinematics.SolveIKWithMatrix(TargetX, TargetY, TargetZ, vertMat, seed, maxIterations: 80, toleranceMm: 0.8, positionOnly: false);
            if (!ok)
            {
                // Fallback to reachable vertical home position (X=220, Y=0, Z=200) if current X=341.8 is too far for vertical tool
                double safeX = Math.Clamp(TargetX, 140.0, 250.0);
                double safeZ = Math.Clamp(TargetZ, 120.0, 260.0);
                (ok, solvedQ) = _kinematics.SolveIKWithMatrix(safeX, TargetY, safeZ, vertMat, new[] { seed[0], -90.0, 180.0, 0.0, -90.0, 180.0 }, maxIterations: 80, toleranceMm: 1.0, positionOnly: false);
            }

            if (solvedQ != null)
            {
                ApplySolvedJointsToTarget(solvedQ);
                _lockedRotationMatrix = vertMat;
                LockedOrientationLabel = "VERTICAL DOWN (-Z)";
                if (!_isIkLockOrientation)
                {
                    _isIkLockOrientation = true;
                    OnPropertyChanged(nameof(IsIkLockOrientation));
                    OnPropertyChanged(nameof(IsIkPositionOnly));
                }
                OnPropertyChanged(nameof(IkModeStatusText));
                AppendLog("🔒 Tool set to VERTICAL DOWN (Pen / Object Manipulation Mode) and locked.");
                DispatchIfAutoOrFollow();
            }
        }

        /// <summary>
        /// Orients the tool horizontally forward (+X world axis, Roll=0°, Pitch=-90°, Yaw=0°) and locks the orientation.
        /// </summary>
        public void SetToolHorizontalForward()
        {
            if (IsEstopActive) return;

            var horizMat = RobotKinematics.Mat4.FromTranslationAndRpy(TargetX, TargetY, TargetZ, 0.0, -Math.PI / 2.0, TargetYaw * Math.PI / 180.0);
            var (_, solvedQ) = _kinematics.SolveIKWithMatrix(TargetX, TargetY, TargetZ, horizMat, GetTargetJoints(), maxIterations: 80, toleranceMm: 0.8, positionOnly: false);
            if (solvedQ != null)
            {
                ApplySolvedJointsToTarget(solvedQ);
                _lockedRotationMatrix = horizMat;
                LockedOrientationLabel = "HORIZONTAL (+X)";
                if (!_isIkLockOrientation)
                {
                    _isIkLockOrientation = true;
                    OnPropertyChanged(nameof(IsIkLockOrientation));
                    OnPropertyChanged(nameof(IsIkPositionOnly));
                }
                OnPropertyChanged(nameof(IkModeStatusText));
                AppendLog("🔒 Tool set to HORIZONTAL FORWARD (+X) and locked.");
                DispatchIfAutoOrFollow();
            }
        }

        private void ApplySolvedJointsToTarget(double[] solvedQ)
        {
            _isSyncingTarget = true;
            _targetJ1 = Math.Round(solvedQ[0], 2);
            _targetJ2 = Math.Round(solvedQ[1], 2);
            _targetJ3 = Math.Round(solvedQ[2], 2);
            _targetJ4 = Math.Round(solvedQ[3], 2);
            _targetJ5 = Math.Round(solvedQ[4], 2);
            _targetJ6 = Math.Round(solvedQ[5], 2);

            OnPropertyChanged(nameof(TargetJ1));
            OnPropertyChanged(nameof(TargetJ2));
            OnPropertyChanged(nameof(TargetJ3));
            OnPropertyChanged(nameof(TargetJ4));
            OnPropertyChanged(nameof(TargetJ5));
            OnPropertyChanged(nameof(TargetJ6));
            _isSyncingTarget = false;

            UpdateTargetFk();
            VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
        }

        #region Network & Telemetry Event Handlers

        private void OnConnectionStateChanged(ConnectionState state)
        {
            App.Current?.Dispatcher.Invoke(() =>
            {
                IsConnected = (state == ConnectionState.Connected);
                StatusText = state.ToString();
                if (!IsConnected)
                {
                    _initialSyncDone = false;
                    KlipperReady = false;
                    KlipperStatus = "Klipper: Disconnected";
                    IsContinuousStreaming = false;
                }
            });
        }

        private void OnTelemetryReceived(TelemetryData data)
        {
            App.Current?.Dispatcher.Invoke(() =>
            {
                bool homingJustFinished = !_isHomed && data.Homed;
                RobotState = data.State;
                IsHomed = data.Homed;
                McuClock = data.McuClock;
                LatencyMs = Math.Round(data.RttMs, 2);
                UpdateRateHz = Math.Round(_client.UpdateRateHz, 1);
                KlipperReady = data.KlipperReady;

                if (data.State == "ESTOP")
                {
                    IsEstopActive = true;
                    if (IsContinuousStreaming)
                    {
                        IsContinuousStreaming = false;
                    }
                    KlipperStatus = "Klipper: EMERGENCY STOP (M112 Active)";
                }
                else if (data.KlipperReady)
                {
                    KlipperStatus = "Klipper: 6-Axis Core (Ready)";
                }
                else if (data.KlipperConnected)
                {
                    KlipperStatus = "Klipper: MCU Connected (Standby)";
                }
                else
                {
                    KlipperStatus = "Klipper: MCU Disconnected";
                }

                if (!string.IsNullOrEmpty(data.Warning))
                {
                    BranchJumpWarning = data.Warning;
                }

                LastMcuError = data.LastMcuError ?? "";
                if (data.CommLogs != null && data.CommLogs.Count > 0)
                {
                    int newestId = data.CommLogs[data.CommLogs.Count - 1].Id;
                    if (newestId != _lastProcessedCommLogId)
                    {
                        var suiteSb = new System.Text.StringBuilder();
                        var mcuSb = new System.Text.StringBuilder();
                        for (int i = data.CommLogs.Count - 1; i >= 0; i--)
                        {
                            var entry = data.CommLogs[i];
                            string line = $"[{entry.Ts}] [{entry.Dir}] {entry.Msg}";
                            if (entry.Link == "SUITE_RPI")
                            {
                                suiteSb.AppendLine(line);
                            }
                            else
                            {
                                mcuSb.AppendLine(line);
                            }

                            if (entry.Id > _lastProcessedCommLogId && entry.Dir == "ERR")
                            {
                                AppendLog($"🚨 [{entry.Link}] {entry.Msg}");
                            }
                        }
                        _lastProcessedCommLogId = newestId;
                        SuiteRpiLogFeed = suiteSb.ToString();
                        RpiMcuLogFeed = mcuSb.ToString();
                    }
                }

                ActualJ1 = Math.Round(data.JointsDeg[0], 2);
                ActualJ2 = Math.Round(data.JointsDeg[1], 2);
                ActualJ3 = Math.Round(data.JointsDeg[2], 2);
                ActualJ4 = Math.Round(data.JointsDeg[3], 2);
                ActualJ5 = Math.Round(data.JointsDeg[4], 2);
                ActualJ6 = Math.Round(data.JointsDeg[5], 2);

                // Always compute live TCP pose from actual joint angles so 50 Hz binary UDP frames never zero ActualX/Y/Z to (0,0,0)
                double[] actualFk = _kinematics.ComputeFkPose(GetActualJoints());
                ActualX = Math.Round(actualFk[0], 2);
                ActualY = Math.Round(actualFk[1], 2);
                ActualZ = Math.Round(actualFk[2], 2);
                ActualRoll = Math.Round(actualFk[3], 2);
                ActualPitch = Math.Round(actualFk[4], 2);
                ActualYaw = Math.Round(actualFk[5], 2);

                if (homingJustFinished || (!_initialSyncDone && data.Homed))
                {
                    _initialSyncDone = true;
                    SyncTargetToActual();
                    InitializeBlenderFromActual();
                    if (homingJustFinished)
                    {
                        AppendLog("Homing complete: Target twin synchronized to CAD reference pose.");
                    }
                }
                OnPropertyChanged(nameof(CanSendPose));
                OnPropertyChanged(nameof(IsMotionAllowed));

                // Flush any staged Gripper command once the physical arm is stationary (IDLE)
                if (_pendingGripperAngle.HasValue && data.State == "IDLE" && !_isExecutingBlendedMove && !_isUserDragging)
                {
                    double pendingAngle = _pendingGripperAngle.Value;
                    _pendingGripperAngle = null;
                    _lastGripperSendTicks = Environment.TickCount64;
                    _ = _client.SendSetGripperAsync(pendingAngle);
                }

                // Update 3D Viewport Trail / Planned Path Line
                if (IsContinuousStreaming)
                {
                    UpdateFollowTrailWithActualTcp();
                }
                else
                {
                    UpdatePlannedPathPreview();
                }

                // Notify 3D Visualizer
                if (data.GripperMm >= 0)
                {
                    _actualGripperAngle = Math.Clamp((1.0 - (data.GripperMm / 62.5)) * 180.0, 0.0, 180.0);
                    GripperVisualsUpdated?.Invoke(_gripperAngle, _actualGripperAngle);
                }
                VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
            });
        }

        private void OnTrimsReceived(Dictionary<string, double> trims)
        {
            App.Current?.Dispatcher.Invoke(() =>
            {
                if (trims.TryGetValue("j1", out double v1)) TrimJ1 = Math.Round(v1, 3);
                if (trims.TryGetValue("j2", out double v2)) TrimJ2 = Math.Round(v2, 3);
                if (trims.TryGetValue("j3", out double v3)) TrimJ3 = Math.Round(v3, 3);
                if (trims.TryGetValue("j4", out double v4)) TrimJ4 = Math.Round(v4, 3);
                if (trims.TryGetValue("j5", out double v5)) TrimJ5 = Math.Round(v5, 3);
                if (trims.TryGetValue("j6", out double v6)) TrimJ6 = Math.Round(v6, 3);
                AppendLog("Received persistent homing trims from robot engine.");
            });
        }

        public void AppendLog(string message)
        {
            App.Current?.Dispatcher.Invoke(() =>
            {
                string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
                LogFeed = $"[{timestamp}] {message}\n" + (LogFeed.Length > 2000 ? LogFeed.Substring(0, 2000) : LogFeed);
            });
        }

        public void ClearAllLogs()
        {
            LogFeed = "";
            SuiteRpiLogFeed = "";
            RpiMcuLogFeed = "";
            LastMcuError = "";
            _lastProcessedCommLogId = -1;
            if (IsConnected)
            {
                _ = _client.SendClearLogsAsync();
            }
        }

        #endregion

        #region Motion & Streaming

        private void OnUdpMetricsUpdated(double rttMs, double jitterMs, double lossPercent, double rxRateHz)
        {
            App.Current?.Dispatcher.Invoke(() =>
            {
                UdpRttText = $"UDP RTT: {rttMs:F1} ms";
                UdpJitterText = $"JITTER: ±{jitterMs:F1} ms";
                UdpLossText = $"LOSS: {lossPercent:F1}%";
                UdpRateText = $"UDP: {rxRateHz:F0} Hz";
            });
        }

        public double[] GetActualJoints() => new[] { ActualJ1, ActualJ2, ActualJ3, ActualJ4, ActualJ5, ActualJ6 };
        public double[] GetTargetJoints() => new[] { TargetJ1, TargetJ2, TargetJ3, TargetJ4, TargetJ5, TargetJ6 };

        public void OnInteractiveDragStarted()
        {
            _isUserDragging = true;

            // Strictly capture and lock the current flange world orientation R_start at the beginning of a drag
            _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
            if (IsIkLockOrientation)
            {
                var pose = _lockedRotationMatrix.ToPoseMmDeg();
                LockedOrientationLabel = $"LOCKED R:{pose.rx:F0}° P:{pose.ry:F0}° Y:{pose.rz:F0}°";
                OnPropertyChanged(nameof(IkModeStatusText));
            }

            if (IsContinuousStreaming)
            {
                long nowMs = Environment.TickCount64;
                lock (_trailLock)
                {
                    _followSplineWindow.Clear();
                    _followSplineWindow.Add((TargetX, TargetY, TargetZ, nowMs));
                    _followTrailPoints.Clear();
                    _followTrailPoints.Add((new System.Windows.Media.Media3D.Point3D(TargetX, TargetY, TargetZ), nowMs));
                }
                UpdateFollowTrailWithActualTcp();
            }
            else
            {
                UpdatePlannedPathPreview();
            }
        }

        public void OnInteractiveDragCompleted()
        {
            _isUserDragging = false;
            if (IsSendOnRelease && IsConnected && IsHomed && !IsEstopActive)
            {
                StartBlendedMoveToTarget();
            }
            else if (IsSimulateOnly)
            {
                UpdatePlannedPathPreview();
            }
        }

        /// <summary>
        /// Renders the planned linear trajectory path in the 3D viewport for OnRelease and Simulate modes
        /// strictly between the live tooltip and the staged target tooltip (never from the robot base).
        /// </summary>
        public void UpdatePlannedPathPreview()
        {
            if (IsContinuousStreaming)
                return;

            double[] actualFk = _kinematics.ComputeFkPose(GetActualJoints());
            double startX = actualFk[0];
            double startY = actualFk[1];
            double startZ = actualFk[2];

            double dx = TargetX - startX;
            double dy = TargetY - startY;
            double dz = TargetZ - startZ;
            double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (dist < 1.5)
            {
                TrailUpdated?.Invoke(Array.Empty<System.Windows.Media.Media3D.Point3D>(), false);
                return;
            }

            int segments = Math.Clamp((int)Math.Ceiling(dist / 12.0), 4, 24);
            var pts = new List<System.Windows.Media.Media3D.Point3D>(segments + 1);
            for (int i = 0; i <= segments; i++)
            {
                double alpha = (double)i / segments;
                pts.Add(new System.Windows.Media.Media3D.Point3D(
                    startX + dx * alpha,
                    startY + dy * alpha,
                    startZ + dz * alpha));
            }
            TrailUpdated?.Invoke(pts, false);
        }

        /// <summary>
        /// Updates the Follow Mode 3D trail and dynamically trims/fades segments as the physical robot catches up
        /// and reaches the final pose. Never connects to the robot base.
        /// </summary>
        private void UpdateFollowTrailWithActualTcp()
        {
            List<System.Windows.Media.Media3D.Point3D> snapshot;
            lock (_trailLock)
            {
                long nowMs = Environment.TickCount64;
                double[] actualFk = _kinematics.ComputeFkPose(GetActualJoints());
                var actualPt = new System.Windows.Media.Media3D.Point3D(actualFk[0], actualFk[1], actualFk[2]);
                var targetPt = new System.Windows.Media.Media3D.Point3D(TargetX, TargetY, TargetZ);

                // 1. Prune trail points that have aged out (> 850 ms) or that the physical robot TCP has reached/passed
                while (_followTrailPoints.Count > 0)
                {
                    long ageMs = nowMs - _followTrailPoints[0].tickMs;
                    double d0 = (_followTrailPoints[0].pt - actualPt).Length;
                    bool passedByActual = _followTrailPoints.Count > 1 &&
                                          ((_followTrailPoints[1].pt - actualPt).Length <= d0 || d0 < 10.0);

                    if (ageMs > 850 || passedByActual || (!_isUserDragging && ageMs > 180))
                    {
                        _followTrailPoints.RemoveAt(0);
                    }
                    else
                    {
                        break;
                    }
                }

                // 2. When not dragging, also progressively fade/pop the tail every tick so the ribbon shrinks into the tooltip
                if (!_isUserDragging && _followTrailPoints.Count > 0)
                {
                    if ((targetPt - actualPt).Length < 4.5 || _followTrailPoints.Count <= 2)
                    {
                        _followTrailPoints.Clear();
                    }
                    else
                    {
                        _followTrailPoints.RemoveAt(0);
                    }
                }

                if (_followTrailPoints.Count < 2)
                {
                    snapshot = new List<System.Windows.Media.Media3D.Point3D>();
                }
                else
                {
                    snapshot = new List<System.Windows.Media.Media3D.Point3D>(_followTrailPoints.Count);
                    for (int i = 0; i < _followTrailPoints.Count; i++)
                    {
                        snapshot.Add(_followTrailPoints[i].pt);
                    }
                }
            }

            App.Current?.Dispatcher.BeginInvoke(new Action(() =>
            {
                TrailUpdated?.Invoke(snapshot, true);
            }));
        }

        /// <summary>
        /// Evaluates a smooth Cubic B-Spline / Catmull-Rom curve over the recent 70 ms sliding window of mouse setpoints
        /// to filter out raw mouse jitter before feeding the flange-locked trajectory generator.
        /// </summary>
        private (double x, double y, double z) EvaluateSlidingWindowSpline(double fallbackX, double fallbackY, double fallbackZ)
        {
            lock (_trailLock)
            {
                long nowMs = Environment.TickCount64;
                // Keep points within the last 90 ms (minimum 4 control points if available)
                while (_followSplineWindow.Count > 4 && (nowMs - _followSplineWindow[0].tickMs) > 90)
                {
                    _followSplineWindow.RemoveAt(0);
                }

                int n = _followSplineWindow.Count;
                if (n == 0)
                    return (fallbackX, fallbackY, fallbackZ);
                if (n < 4)
                {
                    // Exponentially weighted moving average over short window
                    double sx = 0, sy = 0, sz = 0, wSum = 0;
                    for (int i = 0; i < n; i++)
                    {
                        double w = i + 1;
                        sx += _followSplineWindow[i].x * w;
                        sy += _followSplineWindow[i].y * w;
                        sz += _followSplineWindow[i].z * w;
                        wSum += w;
                    }
                    return (sx / wSum, sy / wSum, sz / wSum);
                }

                // Uniform Cubic B-Spline evaluation on the latest 4 control points P0, P1, P2, P3 at u = 0.75
                var p0 = _followSplineWindow[n - 4];
                var p1 = _followSplineWindow[n - 3];
                var p2 = _followSplineWindow[n - 2];
                var p3 = _followSplineWindow[n - 1];

                const double u = 0.75;
                const double u2 = u * u;
                const double u3 = u2 * u;

                double b0 = (1.0 - u) * (1.0 - u) * (1.0 - u) / 6.0;
                double b1 = (3.0 * u3 - 6.0 * u2 + 4.0) / 6.0;
                double b2 = (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0) / 6.0;
                double b3 = u3 / 6.0;

                // Blend 65% B-Spline + 35% latest endpoint so steady-state converges to exact P3
                double bx = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
                double by = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
                double bz = b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z;

                return (0.65 * bx + 0.35 * p3.x, 0.65 * by + 0.35 * p3.y, 0.65 * bz + 0.35 * p3.z);
            }
        }

        private void DispatchIfAutoOrFollow()
        {
            if (!IsConnected || !IsHomed || IsEstopActive || IsSimulateOnly)
                return;

            if (IsSendOnRelease && !_isUserDragging)
            {
                StartBlendedMoveToTarget();
            }
        }

        private void StartBlendedMoveToTarget()
        {
            if (!IsConnected || !IsHomed || IsEstopActive) return;

            if (!_isExecutingBlendedMove && !IsContinuousStreaming)
            {
                InitializeBlenderFromActual();
                _isExecutingBlendedMove = true;
                _streamingTimer.Change(0, 20); // 50 Hz smoothed & blended trajectory execution
            }
            else
            {
                _isExecutingBlendedMove = true;
            }
        }

        private void OnTargetJointsChanged()
        {
            UpdateTargetFk();
            _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
            if (IsIkLockOrientation)
            {
                var p = _lockedRotationMatrix.ToPoseMmDeg();
                LockedOrientationLabel = $"R:{p.rx:F0}° P:{p.ry:F0}° Y:{p.rz:F0}°";
                OnPropertyChanged(nameof(IkModeStatusText));
            }
            VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
            UpdatePlannedPathPreview();

            if (_isSyncingTarget || !IsConnected || !IsHomed || IsEstopActive || IsSimulateOnly)
                return;

            // In Follow Mode, OnStreamingTick (50 Hz) automatically smooths and blends toward TargetJ1..6.
            // In Send-On-Release Mode, if user clicked a slider track (not dragging thumb), trigger blended move.
            if (IsSendOnRelease && !_isUserDragging)
            {
                StartBlendedMoveToTarget();
            }
        }

        private void UpdateTargetFk()
        {
            double[] pose = _kinematics.ComputeFkPose(GetTargetJoints());
            _targetX = Math.Round(pose[0], 2);
            _targetY = Math.Round(pose[1], 2);
            _targetZ = Math.Round(pose[2], 2);
            _targetRoll = Math.Round(pose[3], 2);
            _targetPitch = Math.Round(pose[4], 2);
            _targetYaw = Math.Round(pose[5], 2);

            OnPropertyChanged(nameof(TargetX));
            OnPropertyChanged(nameof(TargetY));
            OnPropertyChanged(nameof(TargetZ));
            OnPropertyChanged(nameof(TargetRoll));
            OnPropertyChanged(nameof(TargetPitch));
            OnPropertyChanged(nameof(TargetYaw));
        }

        public void OnGizmoMoved(double newX, double newY, double newZ)
        {
            if (IsEstopActive) return;

            double reqX = Math.Round(newX, 2);
            double reqY = Math.Round(newY, 2);
            double reqZ = Math.Round(newZ, 2);

            bool success;
            double[] solvedQ;

            if (IsIkLockOrientation)
            {
                // Mode 2: Keep Tool Steady at Locked 3D Rotation Matrix while moving X/Y/Z
                // SolveIKWithMatrix sub-steps from current target joints and stops before any impossible/flipped pose
                (success, solvedQ) = _kinematics.SolveIKWithMatrix(
                    reqX, reqY, reqZ,
                    _lockedRotationMatrix,
                    GetTargetJoints(),
                    maxIterations: 70,
                    toleranceMm: 0.45,
                    positionOnly: false);
            }
            else
            {
                // Mode 1: Probe / Free Tip Mode (keep tooltip on coordinate, flexible wrist angle)
                double[] targetPose = new[] { reqX, reqY, reqZ, TargetRoll, TargetPitch, TargetYaw };
                (success, solvedQ) = _kinematics.SolveIK(targetPose, GetTargetJoints(), positionOnly: true);
            }

            if (success && solvedQ != null && RobotKinematics.IsWithinSafeLimits(solvedQ))
            {
                _isSyncingTarget = true;
                _targetJ1 = Math.Round(solvedQ[0], 2);
                _targetJ2 = Math.Round(solvedQ[1], 2);
                _targetJ3 = Math.Round(solvedQ[2], 2);
                _targetJ4 = Math.Round(solvedQ[3], 2);
                _targetJ5 = Math.Round(solvedQ[4], 2);
                _targetJ6 = Math.Round(solvedQ[5], 2);

                OnPropertyChanged(nameof(TargetJ1));
                OnPropertyChanged(nameof(TargetJ2));
                OnPropertyChanged(nameof(TargetJ3));
                OnPropertyChanged(nameof(TargetJ4));
                OnPropertyChanged(nameof(TargetJ5));
                OnPropertyChanged(nameof(TargetJ6));
                _isSyncingTarget = false;

                // Update TargetX/Y/Z/R/P/Y from the actual validated FK of solvedQ so the gizmo clamps to reachable workspace
                UpdateTargetFk();

                if (!IsIkLockOrientation)
                {
                    _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
                }

                if (IsContinuousStreaming)
                {
                    long nowMs = Environment.TickCount64;
                    lock (_trailLock)
                    {
                        _followSplineWindow.Add((TargetX, TargetY, TargetZ, nowMs));
                        var newPt = new System.Windows.Media.Media3D.Point3D(TargetX, TargetY, TargetZ);
                        if (_followTrailPoints.Count == 0 || (_followTrailPoints[_followTrailPoints.Count - 1].pt - newPt).Length > 2.0)
                        {
                            _followTrailPoints.Add((newPt, nowMs));
                            if (_followTrailPoints.Count > 120)
                            {
                                _followTrailPoints.RemoveAt(0);
                            }
                        }
                    }
                    UpdateFollowTrailWithActualTcp();
                }
                else
                {
                    UpdatePlannedPathPreview();
                }

                VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
            }
        }

        public async Task SendExecutePoseAsync()
        {
            if (!IsConnected)
            {
                AppendLog("Cannot send pose: Not connected to robot engine.");
                return;
            }
            if (IsEstopActive)
            {
                AppendLog("⚠️ EMERGENCY STOP ACTIVE: Motion is locked. Reset drivers first.");
                return;
            }
            if (!IsHomed)
            {
                AppendLog("⚠️ SAFETY INTERLOCK: Robot must be homed before motion! Click HOME ALL first.");
                return;
            }

            if (IsIkLockOrientation)
            {
                AppendLog($"Executing orientation-locked straight-line trajectory to [{TargetX:F1}, {TargetY:F1}, {TargetZ:F1}] ({LockedOrientationLabel})...");
                StartBlendedMoveToTarget();
            }
            else
            {
                AppendLog($"Executing smoothed 6-axis pose: [{TargetJ1:F1}°, {TargetJ2:F1}°, {TargetJ3:F1}°, {TargetJ4:F1}°, {TargetJ5:F1}°, {TargetJ6:F1}°]");
                StartBlendedMoveToTarget();
                await Task.CompletedTask;
            }
        }

        private void OnStreamingTick(object state)
        {
            if ((!IsContinuousStreaming && !_isExecutingBlendedMove) || !IsConnected || !IsHomed || IsEstopActive)
                return;

            const double dt = 0.020; // 50 Hz deterministic step
            double[] targetJoints = GetTargetJoints();
            double[] speedLimits = new[] { SpeedLimitJ1, SpeedLimitJ2, SpeedLimitJ3, SpeedLimitJ4, SpeedLimitJ5, SpeedLimitJ6 };
            double scale = Math.Clamp(SpeedScale, 0.1, 1.0);
            double tau = Math.Clamp(SmoothingTimeConstant, 0.05, 0.35);

            double[] cmdJoints = new double[6];
            bool reachedTarget = true;

            lock (_blenderLock)
            {
                if (IsIkLockOrientation)
                {
                    // Straight-line / Sectional B-Spline Cartesian interpolation with STRICT Flange World Orientation Lock (R_start).
                    // Every single interpolated step is solved directly via SolveIKWithMatrix with synchronized 6-axis velocity scaling
                    // so the tool rotation NEVER changes during intermediate steps.
                    var targetTcp = _kinematics.ComputeTcpMatrix(targetJoints);
                    double goalX = targetTcp.M03;
                    double goalY = targetTcp.M13;
                    double goalZ = targetTcp.M23;

                    if (IsContinuousStreaming && _isUserDragging)
                    {
                        (goalX, goalY, goalZ) = EvaluateSlidingWindowSpline(goalX, goalY, goalZ);
                    }

                    double dx = goalX - _blendedCartPos[0];
                    double dy = goalY - _blendedCartPos[1];
                    double dz = goalZ - _blendedCartPos[2];
                    double cartDist = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                    double maxCartSpeed = 150.0 * scale; // mm/s
                    double maxCartAccel = 450.0 * scale; // mm/s^2

                    if (cartDist > 0.15)
                    {
                        reachedTarget = false;
                        double desSpeed = Math.Min(maxCartSpeed, cartDist / tau);
                        double dv = Math.Clamp(desSpeed - _blendedCartSpeed, -maxCartAccel * dt, maxCartAccel * dt);
                        _blendedCartSpeed = Math.Max(1.0, _blendedCartSpeed + dv);

                        double stepDist = Math.Min(cartDist, _blendedCartSpeed * dt);
                        double dirX = dx / cartDist;
                        double dirY = dy / cartDist;
                        double dirZ = dz / cartDist;

                        double nextX = _blendedCartPos[0] + dirX * stepDist;
                        double nextY = _blendedCartPos[1] + dirY * stepDist;
                        double nextZ = _blendedCartPos[2] + dirZ * stepDist;

                        var (ok, ikQ) = _kinematics.SolveIKWithMatrix(
                            nextX, nextY, nextZ,
                            _lockedRotationMatrix,
                            _blendedJoints,
                            maxIterations: 50,
                            toleranceMm: 0.4,
                            positionOnly: false);

                        if (ok && ikQ != null && RobotKinematics.IsWithinSafeLimits(ikQ))
                        {
                            // Synchronized 6-axis joint speed scaling & 3.0° delta interceptor:
                            // scale the Cartesian step proportionally so all 6 joints remain 100% orientation-locked
                            // and never exceed 3.0° per tick.
                            double syncScale = 1.0;
                            for (int i = 0; i < 6; i++)
                            {
                                double vMax = Math.Max(5.0, speedLimits[i] * scale);
                                double maxStepDeg = Math.Min(3.0, vMax * dt);
                                double deltaDeg = Math.Abs(ikQ[i] - _blendedJoints[i]);
                                if (deltaDeg > maxStepDeg && deltaDeg > 1e-4)
                                {
                                    syncScale = Math.Min(syncScale, maxStepDeg / deltaDeg);
                                }
                            }

                            if (syncScale < 0.999)
                            {
                                stepDist *= syncScale;
                                nextX = _blendedCartPos[0] + dirX * stepDist;
                                nextY = _blendedCartPos[1] + dirY * stepDist;
                                nextZ = _blendedCartPos[2] + dirZ * stepDist;

                                var (okScaled, ikQScaled) = _kinematics.SolveIKWithMatrix(
                                    nextX, nextY, nextZ,
                                    _lockedRotationMatrix,
                                    _blendedJoints,
                                    maxIterations: 45,
                                    toleranceMm: 0.4,
                                    positionOnly: false);

                                if (okScaled && ikQScaled != null && RobotKinematics.IsWithinSafeLimits(ikQScaled))
                                {
                                    ikQ = ikQScaled;
                                }
                            }

                            if (RobotKinematics.ValidateAndLockBranch(ikQ, _blendedJoints, 3.0, out string violationReason))
                            {
                                for (int i = 0; i < 6; i++)
                                {
                                    _blendedJointVel[i] = (ikQ[i] - _blendedJoints[i]) / dt;
                                    _blendedJoints[i] = ikQ[i];
                                    cmdJoints[i] = Math.Round(_blendedJoints[i], 3);
                                }

                                var achievedTcp = _kinematics.ComputeTcpMatrix(_blendedJoints);
                                _blendedCartPos[0] = achievedTcp.M03;
                                _blendedCartPos[1] = achievedTcp.M13;
                                _blendedCartPos[2] = achievedTcp.M23;
                            }
                            else
                            {
                                // Safety interceptor rejected candidate frame: hold previous valid state
                                _blendedCartSpeed = 0.0;
                                reachedTarget = true;
                                for (int i = 0; i < 6; i++)
                                {
                                    _blendedJointVel[i] = 0.0;
                                    cmdJoints[i] = Math.Round(_blendedJoints[i], 3);
                                }
                                AppendLog($"[SAFETY GATE] Frame rejected: {violationReason}");
                            }
                        }
                        else
                        {
                            // Reached boundary of valid orientation-locked path; stop cleanly at current valid pose
                            _blendedCartSpeed = 0.0;
                            reachedTarget = true;
                            for (int i = 0; i < 6; i++)
                            {
                                _blendedJointVel[i] = 0.0;
                                cmdJoints[i] = Math.Round(_blendedJoints[i], 3);
                            }
                        }
                    }
                    else
                    {
                        _blendedCartSpeed = 0.0;
                        for (int i = 0; i < 6; i++)
                        {
                            _blendedJointVel[i] = 0.0;
                            cmdJoints[i] = Math.Round(_blendedJoints[i], 3);
                        }
                    }
                }
                else
                {
                    // Free Wrist / Probe Mode: Synchronized second-order velocity & acceleration joint blender
                    for (int i = 0; i < 6; i++)
                    {
                        double err = targetJoints[i] - _blendedJoints[i];
                        if (Math.Abs(err) > 0.03 || Math.Abs(_blendedJointVel[i]) > 0.2)
                        {
                            reachedTarget = false;
                        }

                        double vMax = Math.Max(5.0, speedLimits[i] * scale);
                        double aMax = vMax * 3.2;
                        double vDes = Math.Clamp(err / tau, -vMax, vMax);
                        double dv = Math.Clamp(vDes - _blendedJointVel[i], -aMax * dt, aMax * dt);
                        _blendedJointVel[i] += dv;
                        double nextQ = _blendedJoints[i] + _blendedJointVel[i] * dt;
                        _blendedJoints[i] = Math.Clamp(nextQ, RobotKinematics.HardwareLimitsDeg[i].Min, RobotKinematics.HardwareLimitsDeg[i].Max);

                        if (Math.Abs(targetJoints[i] - _blendedJoints[i]) < 0.02 && Math.Abs(vDes) < 0.5)
                        {
                            _blendedJoints[i] = targetJoints[i];
                            _blendedJointVel[i] = 0.0;
                        }

                        cmdJoints[i] = Math.Round(_blendedJoints[i], 3);
                    }

                    var achievedTcp = _kinematics.ComputeTcpMatrix(_blendedJoints);
                    _blendedCartPos[0] = achievedTcp.M03;
                    _blendedCartPos[1] = achievedTcp.M13;
                    _blendedCartPos[2] = achievedTcp.M23;
                }
            }

            byte grip = (byte)Math.Clamp(GripperAngle, 0.0, 180.0);
            bool changed = false;
            for (int i = 0; i < 6; i++)
            {
                if (Math.Abs(cmdJoints[i] - _lastStreamedJoints[i]) > 0.008)
                {
                    changed = true;
                    break;
                }
            }

            _streamIdleCounter++;
            if (changed || (IsContinuousStreaming && _streamIdleCounter >= 10))
            {
                _streamIdleCounter = 0;
                Array.Copy(cmdJoints, _lastStreamedJoints, 6);
                _client.SendUdpServoJoints(cmdJoints, grip);
            }

            if (IsContinuousStreaming)
            {
                UpdateFollowTrailWithActualTcp();
            }

            if (!IsContinuousStreaming && _isExecutingBlendedMove && reachedTarget)
            {
                _isExecutingBlendedMove = false;
                _streamingTimer.Change(Timeout.Infinite, Timeout.Infinite);
                _ = _client.SendMoveJointsAsync(cmdJoints, 0.25);
                if (_pendingGripperAngle.HasValue)
                {
                    double pendingAngle = _pendingGripperAngle.Value;
                    _pendingGripperAngle = null;
                    _ = _client.SendSetGripperAsync(pendingAngle);
                }
            }
        }

        #endregion

        #region Control Actions & Commands

        public async Task ToggleConnectAsync()
        {
            if (IsConnected)
            {
                _client.Disconnect();
            }
            else
            {
                await _client.ConnectAsync(PiAddress);
            }
        }

        public async Task TriggerEstopAsync()
        {
            AppendLog("🚨 EMERGENCY STOP (M112) DISPATCHED!");
            IsEstopActive = true;
            _isExecutingBlendedMove = false;
            IsContinuousStreaming = false;
            _streamingTimer.Change(Timeout.Infinite, Timeout.Infinite);
            await _client.SendEstopAsync();
        }

        public async Task TriggerResetAsync()
        {
            AppendLog("Resetting system via Klipper FIRMWARE_RESTART...");
            IsEstopActive = false;
            await _client.SendResetAsync();
        }

        public async Task TriggerHomeAsync()
        {
            if (IsEstopActive)
            {
                AppendLog("⚠️ Clear E-Stop before homing!");
                return;
            }
            AppendLog("Triggering sequential hardware collision-avoidance homing...");
            await _client.SendHomeAsync("ALL");
        }

        public async Task TriggerSetCadHomeAsync()
        {
            if (IsEstopActive)
            {
                AppendLog("⚠️ Clear E-Stop before calibrating CAD home!");
                return;
            }
            AppendLog("Invoking SET_CAD_HOME calibration macro...");
            await _client.SendSetCadHomeAsync();
            SyncTargetToActual();
            InitializeBlenderFromActual();
        }

        public async Task JogJointAsync(int joint, double direction)
        {
            if (IsEstopActive || (!IsSimulateOnly && (!IsConnected || !IsHomed)))
            {
                AppendLog("⚠️ Motion locked: Verify connection, homed state, and E-Stop (or enable SIMULATE mode).");
                return;
            }

            double delta = direction * JointStepSize;
            _isSyncingTarget = true;

            switch (joint)
            {
                case 1: _targetJ1 = Math.Clamp(_targetJ1 + delta, RobotKinematics.HardwareLimitsDeg[0].Min, RobotKinematics.HardwareLimitsDeg[0].Max); break;
                case 2: _targetJ2 = Math.Clamp(_targetJ2 + delta, RobotKinematics.HardwareLimitsDeg[1].Min, RobotKinematics.HardwareLimitsDeg[1].Max); break;
                case 3: _targetJ3 = Math.Clamp(_targetJ3 + delta, RobotKinematics.HardwareLimitsDeg[2].Min, RobotKinematics.HardwareLimitsDeg[2].Max); break;
                case 4: _targetJ4 = Math.Clamp(_targetJ4 + delta, RobotKinematics.HardwareLimitsDeg[3].Min, RobotKinematics.HardwareLimitsDeg[3].Max); break;
                case 5: _targetJ5 = Math.Clamp(_targetJ5 + delta, RobotKinematics.HardwareLimitsDeg[4].Min, RobotKinematics.HardwareLimitsDeg[4].Max); break;
                case 6: _targetJ6 = Math.Clamp(_targetJ6 + delta, RobotKinematics.HardwareLimitsDeg[5].Min, RobotKinematics.HardwareLimitsDeg[5].Max); break;
            }

            OnPropertyChanged(nameof(TargetJ1));
            OnPropertyChanged(nameof(TargetJ2));
            OnPropertyChanged(nameof(TargetJ3));
            OnPropertyChanged(nameof(TargetJ4));
            OnPropertyChanged(nameof(TargetJ5));
            OnPropertyChanged(nameof(TargetJ6));
            _isSyncingTarget = false;

            UpdateTargetFk();
            if (IsIkLockOrientation)
            {
                _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
                var p = _lockedRotationMatrix.ToPoseMmDeg();
                LockedOrientationLabel = $"R:{p.rx:F0}° P:{p.ry:F0}° Y:{p.rz:F0}°";
                OnPropertyChanged(nameof(IkModeStatusText));
            }
            VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
            UpdatePlannedPathPreview();

            if (!IsSimulateOnly && IsConnected && IsHomed)
            {
                StartBlendedMoveToTarget();
            }
            await Task.CompletedTask;
        }

        public async Task JogCartesianAsync(string axis, double direction)
        {
            if (IsEstopActive || (!IsSimulateOnly && (!IsConnected || !IsHomed)))
            {
                AppendLog("⚠️ Motion locked: Verify connection, homed state, and E-Stop (or enable SIMULATE mode).");
                return;
            }

            string axUpper = axis.ToUpper();
            bool isRot = (axUpper == "RX" || axUpper == "RY" || axUpper == "RZ");
            double delta = direction * (isRot ? 5.0 : CartesianStepSize);

            var currentTcpMat = IsIkLockOrientation ? _lockedRotationMatrix : _kinematics.ComputeTcpMatrix(GetTargetJoints());
            currentTcpMat.M03 = _targetX;
            currentTcpMat.M13 = _targetY;
            currentTcpMat.M23 = _targetZ;

            if (!isRot)
            {
                switch (axUpper)
                {
                    case "X": _targetX = Math.Round(_targetX + delta, 2); break;
                    case "Y": _targetY = Math.Round(_targetY + delta, 2); break;
                    case "Z": _targetZ = Math.Round(_targetZ + delta, 2); break;
                }
            }
            else
            {
                currentTcpMat = RobotKinematics.ApplyWorldRotation(currentTcpMat, axUpper, delta);
                _lockedRotationMatrix = currentTcpMat;
                var rotPose = currentTcpMat.ToPoseMmDeg();
                _targetRoll = Math.Round(rotPose.rx, 2);
                _targetPitch = Math.Round(rotPose.ry, 2);
                _targetYaw = Math.Round(rotPose.rz, 2);
                if (IsIkLockOrientation)
                {
                    LockedOrientationLabel = $"R:{_targetRoll:F0}° P:{_targetPitch:F0}° Y:{_targetYaw:F0}°";
                    OnPropertyChanged(nameof(IkModeStatusText));
                }
            }

            OnPropertyChanged(nameof(TargetX));
            OnPropertyChanged(nameof(TargetY));
            OnPropertyChanged(nameof(TargetZ));
            OnPropertyChanged(nameof(TargetRoll));
            OnPropertyChanged(nameof(TargetPitch));
            OnPropertyChanged(nameof(TargetYaw));

            bool success;
            double[] solvedQ;
            if (IsIkLockOrientation || isRot)
            {
                (success, solvedQ) = _kinematics.SolveIKWithMatrix(
                    _targetX, _targetY, _targetZ,
                    currentTcpMat,
                    GetTargetJoints(),
                    maxIterations: 75,
                    toleranceMm: 0.5,
                    positionOnly: false);
            }
            else
            {
                (success, solvedQ) = _kinematics.SolveIKWithMatrix(
                    _targetX, _targetY, _targetZ,
                    currentTcpMat,
                    GetTargetJoints(),
                    maxIterations: 65,
                    toleranceMm: 0.5,
                    positionOnly: false);
                if (!success)
                {
                    (success, solvedQ) = _kinematics.SolveIKWithMatrix(
                        _targetX, _targetY, _targetZ,
                        currentTcpMat,
                        GetTargetJoints(),
                        maxIterations: 50,
                        toleranceMm: 0.5,
                        positionOnly: true);
                }
            }

            if (success && solvedQ != null)
            {
                _isSyncingTarget = true;
                _targetJ1 = Math.Round(solvedQ[0], 2);
                _targetJ2 = Math.Round(solvedQ[1], 2);
                _targetJ3 = Math.Round(solvedQ[2], 2);
                _targetJ4 = Math.Round(solvedQ[3], 2);
                _targetJ5 = Math.Round(solvedQ[4], 2);
                _targetJ6 = Math.Round(solvedQ[5], 2);

                OnPropertyChanged(nameof(TargetJ1));
                OnPropertyChanged(nameof(TargetJ2));
                OnPropertyChanged(nameof(TargetJ3));
                OnPropertyChanged(nameof(TargetJ4));
                OnPropertyChanged(nameof(TargetJ5));
                OnPropertyChanged(nameof(TargetJ6));
                _isSyncingTarget = false;

                UpdateTargetFk();
                VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
                UpdatePlannedPathPreview();
            }

            if (!IsSimulateOnly && IsConnected && IsHomed)
            {
                StartBlendedMoveToTarget();
            }
            await Task.CompletedTask;
        }

        public void SyncTargetToActual()
        {
            _isSyncingTarget = true;
            _targetJ1 = ActualJ1;
            _targetJ2 = ActualJ2;
            _targetJ3 = ActualJ3;
            _targetJ4 = ActualJ4;
            _targetJ5 = ActualJ5;
            _targetJ6 = ActualJ6;

            OnPropertyChanged(nameof(TargetJ1));
            OnPropertyChanged(nameof(TargetJ2));
            OnPropertyChanged(nameof(TargetJ3));
            OnPropertyChanged(nameof(TargetJ4));
            OnPropertyChanged(nameof(TargetJ5));
            OnPropertyChanged(nameof(TargetJ6));
            _isSyncingTarget = false;

            UpdateTargetFk();
            _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
            InitializeBlenderFromActual();
            VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
            UpdatePlannedPathPreview();
            AppendLog("Synced target ghost to live robot angles.");
        }

        public void MoveToCadHome()
        {
            if (IsEstopActive) return;

            _isSyncingTarget = true;
            _targetJ1 = RobotKinematics.CadHomeDeg[0];
            _targetJ2 = RobotKinematics.CadHomeDeg[1];
            _targetJ3 = RobotKinematics.CadHomeDeg[2];
            _targetJ4 = RobotKinematics.CadHomeDeg[3];
            _targetJ5 = RobotKinematics.CadHomeDeg[4];
            _targetJ6 = RobotKinematics.CadHomeDeg[5];
            OnPropertyChanged(nameof(TargetJ1));
            OnPropertyChanged(nameof(TargetJ2));
            OnPropertyChanged(nameof(TargetJ3));
            OnPropertyChanged(nameof(TargetJ4));
            OnPropertyChanged(nameof(TargetJ5));
            OnPropertyChanged(nameof(TargetJ6));
            _isSyncingTarget = false;

            UpdateTargetFk();
            _lockedRotationMatrix = _kinematics.ComputeTcpMatrix(GetTargetJoints());
            VisualsUpdated?.Invoke(GetActualJoints(), GetTargetJoints());
            UpdatePlannedPathPreview();
            if (!IsSimulateOnly && IsConnected && IsHomed)
            {
                StartBlendedMoveToTarget();
            }
        }

        public void NudgeTrim(int joint, double delta)
        {
            switch (joint)
            {
                case 1: TrimJ1 = Math.Round(TrimJ1 + delta, 3); break;
                case 2: TrimJ2 = Math.Round(TrimJ2 + delta, 3); break;
                case 3: TrimJ3 = Math.Round(TrimJ3 + delta, 3); break;
                case 4: TrimJ4 = Math.Round(TrimJ4 + delta, 3); break;
                case 5: TrimJ5 = Math.Round(TrimJ5 + delta, 3); break;
                case 6: TrimJ6 = Math.Round(TrimJ6 + delta, 3); break;
            }
        }

        public async Task SaveTrimsAsync()
        {
            var dict = new Dictionary<string, double>
            {
                ["j1"] = TrimJ1,
                ["j2"] = TrimJ2,
                ["j3"] = TrimJ3,
                ["j4"] = TrimJ4,
                ["j5"] = TrimJ5,
                ["j6"] = TrimJ6
            };
            await _client.SendSetTrimsAsync(dict, save: true);
            AppendLog("Saved updated trims to Raspberry Pi persistent storage.");
        }

        public async Task RefreshTrimsAsync()
        {
            await _client.SendGetTrimsAsync();
        }

        #endregion

        #region INotifyPropertyChanged

        public event PropertyChangedEventHandler PropertyChanged;
        protected bool SetProperty<T>(ref T storage, T value, [CallerMemberName] string propertyName = null)
        {
            if (EqualityComparer<T>.Default.Equals(storage, value)) return false;
            storage = value;
            OnPropertyChanged(propertyName);
            return true;
        }

        protected void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        #endregion
    }
}
