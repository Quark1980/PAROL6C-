using System;
using System.Collections.ObjectModel;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using PAROLC_ControlSuite.Network;

namespace PAROLC_ControlSuite.Views
{
    public class PacketLogEntry
    {
        public string Timestamp { get; set; }
        public string Direction { get; set; }
        public string JsonText { get; set; }

        public Brush BadgeBackground { get; set; }
        public Brush BadgeForeground { get; set; }
        public Brush BackgroundBrush { get; set; }
        public Brush BorderBrush { get; set; }
        public Brush ContentForeground { get; set; }
    }

    public partial class DebugStreamWindow : Window
    {
        private readonly RobotWebSocketClient _client;
        private readonly ObservableCollection<PacketLogEntry> _allPackets = new ObservableCollection<PacketLogEntry>();
        private readonly ObservableCollection<PacketLogEntry> _displayedPackets = new ObservableCollection<PacketLogEntry>();
        private const int MaxStoredPackets = 1200;

        private bool _isInitialized = false;
        private bool _isPaused = false;
        private string _activeFilter = "ALL";
        private string _searchQuery = "";

        // Cached brushes
        private static readonly Brush RxBadgeBg = new SolidColorBrush(Color.FromRgb(14, 116, 144));      // Cyan-700
        private static readonly Brush RxBadgeFg = new SolidColorBrush(Color.FromRgb(103, 232, 249));     // Cyan-300
        private static readonly Brush TxBadgeBg = new SolidColorBrush(Color.FromRgb(109, 40, 217));      // Purple-700
        private static readonly Brush TxBadgeFg = new SolidColorBrush(Color.FromRgb(216, 180, 254));     // Purple-300
        private static readonly Brush RxRowBg = new SolidColorBrush(Color.FromArgb(40, 15, 23, 42));
        private static readonly Brush TxRowBg = new SolidColorBrush(Color.FromArgb(40, 30, 27, 75));
        private static readonly Brush AckRowBg = new SolidColorBrush(Color.FromArgb(50, 6, 78, 59));
        private static readonly Brush BorderSubtle = new SolidColorBrush(Color.FromRgb(30, 41, 59));
        private static readonly Brush TextDefault = new SolidColorBrush(Color.FromRgb(226, 232, 240));
        private static readonly Brush TextAck = new SolidColorBrush(Color.FromRgb(52, 211, 153));

        // Endstop visual indicators
        private static readonly Brush EndstopTrigLed = new SolidColorBrush(Color.FromRgb(16, 185, 129));   // Green-500
        private static readonly Brush EndstopTrigBg = new SolidColorBrush(Color.FromArgb(50, 16, 185, 129));
        private static readonly Brush EndstopTrigBorder = new SolidColorBrush(Color.FromRgb(16, 185, 129));
        private static readonly Brush EndstopOpenLed = new SolidColorBrush(Color.FromRgb(100, 116, 139));  // Slate-500
        private static readonly Brush EndstopOpenBg = new SolidColorBrush(Color.FromRgb(30, 41, 59));     // Slate-800
        private static readonly Brush EndstopOpenBorder = new SolidColorBrush(Color.FromRgb(51, 65, 85));

        public DebugStreamWindow(RobotWebSocketClient client)
        {
            InitializeComponent();
            _client = client ?? throw new ArgumentNullException(nameof(client));

            LstPackets.ItemsSource = _displayedPackets;

            // Subscribe to network events
            _client.RawPacketLogged += OnRawPacketLogged;
            _client.TelemetryReceived += OnTelemetryReceived;
            _client.StateChanged += OnStateChanged;

            UpdateConnectionStatus(_client.State);

            this.Closed += OnWindowClosed;

            // Mark initialization complete and build packet view safely
            _isInitialized = true;
            RebuildDisplayedPackets();
        }

        private void OnWindowClosed(object sender, EventArgs e)
        {
            _client.RawPacketLogged -= OnRawPacketLogged;
            _client.TelemetryReceived -= OnTelemetryReceived;
            _client.StateChanged -= OnStateChanged;
        }

        private void OnRawPacketLogged(string direction, string rawJson)
        {
            if (_isPaused) return;

            Dispatcher.InvokeAsync(() =>
            {
                bool isAck = rawJson.Contains("\"type\":\"ACK\"") || rawJson.Contains("\"status\":\"OK\"");

                bool isTx = direction.Contains("TX");
                Brush badgeBg = isTx ? TxBadgeBg : RxBadgeBg;
                Brush badgeFg = isTx ? TxBadgeFg : RxBadgeFg;
                Brush rowBg = isAck ? AckRowBg : (isTx ? TxRowBg : RxRowBg);
                Brush contentFg = isAck ? TextAck : TextDefault;

                var entry = new PacketLogEntry
                {
                    Timestamp = DateTime.Now.ToString("HH:mm:ss.fff"),
                    Direction = direction,
                    JsonText = rawJson,
                    BadgeBackground = badgeBg,
                    BadgeForeground = badgeFg,
                    BackgroundBrush = rowBg,
                    BorderBrush = BorderSubtle,
                    ContentForeground = contentFg
                };

                _allPackets.Add(entry);
                if (_allPackets.Count > MaxStoredPackets)
                {
                    _allPackets.RemoveAt(0);
                }

                if (MatchesFilter(entry))
                {
                    _displayedPackets.Add(entry);
                    if (_displayedPackets.Count > MaxStoredPackets)
                    {
                        _displayedPackets.RemoveAt(0);
                    }

                    if (ChkAutoScroll?.IsChecked == true && LstPackets?.Items.Count > 0)
                    {
                        LstPackets.ScrollIntoView(entry);
                    }
                }

                if (TxtRxCount != null) TxtRxCount.Text = $"RX: {_client.TotalPacketsRx:N0} pkts";
                if (TxtTxCount != null) TxtTxCount.Text = $"TX: {_client.TotalPacketsTx:N0} pkts";
                if (TxtRateHz != null) TxtRateHz.Text = $"Rate: {_client.UpdateRateHz:F1} Hz";
            }, DispatcherPriority.Background);
        }

        private void OnTelemetryReceived(TelemetryData data)
        {
            Dispatcher.InvokeAsync(() =>
            {
                if (TxtRttMs != null) TxtRttMs.Text = $"RTT: {data.RttMs:F1} ms";
                if (TxtMcuClock != null) TxtMcuClock.Text = $"CLK: {data.McuClock:N0}";

                // Update Free Slots display
                if (TxtSlotSummary != null && data.FreeSlots != null && data.FreeSlots.Length >= 6)
                {
                    TxtSlotSummary.Text = $"[{string.Join(", ", data.FreeSlots)}]";
                }

                // Update Invert Masks
                if (TxtDirInvert != null) TxtDirInvert.Text = $"DIR: 0x{data.DirInvertMask:X2}";
                if (TxtEndstopInvert != null) TxtEndstopInvert.Text = $"STOP: 0x{data.EndstopInvertMask:X2}";

                // Update Joint Angles
                if (TxtLiveJoints != null && data.JointsDeg != null && data.JointsDeg.Length >= 6)
                {
                    TxtLiveJoints.Text = $"J1: {data.JointsDeg[0]:F1}°  J2: {data.JointsDeg[1]:F1}°  J3: {data.JointsDeg[2]:F1}°  J4: {data.JointsDeg[3]:F1}°  J5: {data.JointsDeg[4]:F1}°  J6: {data.JointsDeg[5]:F1}°";
                }

                // Update Endstops individual visual badges
                if (data.Endstops != null && data.Endstops.Length >= 6)
                {
                    UpdateEndstopBadge(0, data.Endstops[0], BorderEndstopJ1, LedEndstopJ1, TxtEndstopJ1, "J1 (PE5)");
                    UpdateEndstopBadge(1, data.Endstops[1], BorderEndstopJ2, LedEndstopJ2, TxtEndstopJ2, "J2 (PE4)");
                    UpdateEndstopBadge(2, data.Endstops[2], BorderEndstopJ3, LedEndstopJ3, TxtEndstopJ3, "J3 (PJ1)");
                    UpdateEndstopBadge(3, data.Endstops[3], BorderEndstopJ4, LedEndstopJ4, TxtEndstopJ4, "J4 (PJ0)");
                    UpdateEndstopBadge(4, data.Endstops[4], BorderEndstopJ5, LedEndstopJ5, TxtEndstopJ5, "J5 (PD3)");
                    UpdateEndstopBadge(5, data.Endstops[5], BorderEndstopJ6, LedEndstopJ6, TxtEndstopJ6, "J6 (PD2)");

                    var sb = new StringBuilder("[");
                    for (int i = 0; i < 6; i++)
                    {
                        sb.Append(data.Endstops[i] ? "1" : "0");
                        if (i < 5) sb.Append(" ");
                    }
                    sb.Append("]");
                    if (TxtEndstops != null) TxtEndstops.Text = sb.ToString();
                }
            }, DispatcherPriority.Background);
        }

        private void UpdateEndstopBadge(int idx, bool triggered, Border border, Ellipse led, TextBlock txt, string label)
        {
            if (border == null || led == null || txt == null) return;
            if (triggered)
            {
                led.Fill = EndstopTrigLed;
                border.Background = EndstopTrigBg;
                border.BorderBrush = EndstopTrigBorder;
                txt.Foreground = EndstopTrigLed;
                txt.Text = $"{label}: TRIG";
            }
            else
            {
                led.Fill = EndstopOpenLed;
                border.Background = EndstopOpenBg;
                border.BorderBrush = EndstopOpenBorder;
                txt.Foreground = TextDefault;
                txt.Text = $"{label}: OPEN";
            }
        }

        private void OnStateChanged(ConnectionState state)
        {
            Dispatcher.InvokeAsync(() => UpdateConnectionStatus(state));
        }

        private void UpdateConnectionStatus(ConnectionState state)
        {
            if (TxtConnStatus == null) return;
            switch (state)
            {
                case ConnectionState.Connected:
                    TxtConnStatus.Text = "[CONNECTED]";
                    TxtConnStatus.Foreground = new SolidColorBrush(Color.FromRgb(16, 185, 129));
                    break;
                case ConnectionState.Connecting:
                    TxtConnStatus.Text = "[CONNECTING]";
                    TxtConnStatus.Foreground = new SolidColorBrush(Color.FromRgb(245, 158, 11));
                    break;
                case ConnectionState.Reconnecting:
                    TxtConnStatus.Text = "[RECONNECTING]";
                    TxtConnStatus.Foreground = new SolidColorBrush(Color.FromRgb(245, 158, 11));
                    break;
                default:
                    TxtConnStatus.Text = "[DISCONNECTED]";
                    TxtConnStatus.Foreground = new SolidColorBrush(Color.FromRgb(239, 68, 68));
                    break;
            }
        }

        #region Filters

        private bool MatchesFilter(PacketLogEntry entry)
        {
            if (_activeFilter == "RX" && !entry.Direction.Contains("RX")) return false;
            if (_activeFilter == "TX" && !entry.Direction.Contains("TX")) return false;
            if (_activeFilter == "ACK" && !entry.JsonText.Contains("\"type\":\"ACK\"") && !entry.JsonText.Contains("\"status\":\"OK\"")) return false;

            if (!string.IsNullOrEmpty(_searchQuery))
            {
                if (entry.JsonText.IndexOf(_searchQuery, StringComparison.OrdinalIgnoreCase) < 0 &&
                    entry.Direction.IndexOf(_searchQuery, StringComparison.OrdinalIgnoreCase) < 0)
                {
                    return false;
                }
            }

            return true;
        }

        private void RebuildDisplayedPackets()
        {
            if (!_isInitialized) return;

            _displayedPackets.Clear();
            foreach (var p in _allPackets)
            {
                if (MatchesFilter(p))
                {
                    _displayedPackets.Add(p);
                }
            }
            if (ChkAutoScroll?.IsChecked == true && _displayedPackets.Count > 0 && LstPackets != null)
            {
                LstPackets.ScrollIntoView(_displayedPackets[_displayedPackets.Count - 1]);
            }
        }

        private void Filter_Changed(object sender, RoutedEventArgs e)
        {
            if (!_isInitialized) return;

            if (RdoRx?.IsChecked == true) _activeFilter = "RX";
            else if (RdoTx?.IsChecked == true) _activeFilter = "TX";
            else if (RdoAck?.IsChecked == true) _activeFilter = "ACK";
            else _activeFilter = "ALL";

            RebuildDisplayedPackets();
        }

        private void TxtSearch_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_isInitialized) return;

            _searchQuery = TxtSearch?.Text?.Trim() ?? "";
            RebuildDisplayedPackets();
        }

        #endregion

        #region Actions

        private async void BtnRestoreDefaults_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                await _client.SendResetInvertMasksAsync();
                MessageBox.Show("Polarity masks successfully restored to printer.cfg defaults!\nDIR MASK: 0x03 (J1, J2 only)\nENDSTOP MASK: 0x00 (Normal active-high)", "PAROL6 Hardware Configuration", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Failed restoring defaults: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void BtnPauseResume_Click(object sender, RoutedEventArgs e)
        {
            _isPaused = !_isPaused;
            if (_isPaused)
            {
                BtnPauseResume.Content = "RESUME";
                BtnPauseResume.Background = new SolidColorBrush(Color.FromRgb(180, 83, 9));
            }
            else
            {
                BtnPauseResume.Content = "PAUSE";
                BtnPauseResume.Background = new SolidColorBrush(Color.FromRgb(30, 41, 59));
            }
        }

        private void BtnClear_Click(object sender, RoutedEventArgs e)
        {
            _allPackets.Clear();
            _displayedPackets.Clear();
        }

        private void BtnCopy_Click(object sender, RoutedEventArgs e)
        {
            var sb = new StringBuilder();
            foreach (var item in _displayedPackets)
            {
                sb.AppendLine($"[{item.Timestamp}] [{item.Direction}] {item.JsonText}");
            }
            if (sb.Length > 0)
            {
                Clipboard.SetText(sb.ToString());
                MessageBox.Show("Packets copied to clipboard!", "PAROL6 Debug Monitor", MessageBoxButton.OK, MessageBoxImage.Information);
            }
        }

        #endregion
    }
}
