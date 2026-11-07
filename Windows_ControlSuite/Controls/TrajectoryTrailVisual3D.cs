using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace PAROLC_ControlSuite.Controls
{
    /// <summary>
    /// Renders 3D trajectory path lines (OnRelease / Simulate planned path) and dynamic fading
    /// real-time TCP ribbon trails (Follow Mode) inside the HelixViewport3D.
    /// </summary>
    public class TrajectoryTrailVisual3D : ModelVisual3D
    {
        private readonly LinesVisual3D _plannedPathLines;
        private readonly LinesVisual3D _followTrailCoreLines;
        private readonly LinesVisual3D _followTrailGlowLines;
        private readonly PointsVisual3D _waypointMarkers;

        public TrajectoryTrailVisual3D()
        {
            _plannedPathLines = new LinesVisual3D
            {
                Color = Color.FromArgb(230, 0, 230, 118), // Tactical Emerald Green (#00E676)
                Thickness = 3.0
            };

            _followTrailGlowLines = new LinesVisual3D
            {
                Color = Color.FromArgb(110, 0, 210, 255), // Outer Cyan Glow
                Thickness = 6.0
            };

            _followTrailCoreLines = new LinesVisual3D
            {
                Color = Color.FromArgb(255, 255, 145, 0), // Inner High-Contrast Orange/Gold Trail
                Thickness = 2.8
            };

            _waypointMarkers = new PointsVisual3D
            {
                Color = Color.FromArgb(240, 0, 229, 255),
                Size = 5.5
            };

            this.Children.Add(_plannedPathLines);
            this.Children.Add(_followTrailGlowLines);
            this.Children.Add(_followTrailCoreLines);
            this.Children.Add(_waypointMarkers);
        }

        /// <summary>
        /// Updates the 3D viewport trajectory visualization.
        /// </summary>
        /// <param name="waypoints">Ordered list of 3D TCP points (in mm) from physical robot TCP to target TCP.</param>
        /// <param name="isFollowTrail">True when rendering the dynamic Follow Mode trail; False when rendering OnRelease/Simulate planned path.</param>
        public void UpdateTrail(IReadOnlyList<Point3D> waypoints, bool isFollowTrail)
        {
            var plannedPts = new Point3DCollection();
            var followPts = new Point3DCollection();
            var markerPts = new Point3DCollection();

            if (waypoints != null && waypoints.Count >= 2)
            {
                for (int i = 0; i < waypoints.Count - 1; i++)
                {
                    Point3D p0 = waypoints[i];
                    Point3D p1 = waypoints[i + 1];
                    if ((p1 - p0).LengthSquared > 0.01)
                    {
                        if (isFollowTrail)
                        {
                            followPts.Add(p0);
                            followPts.Add(p1);
                        }
                        else
                        {
                            plannedPts.Add(p0);
                            plannedPts.Add(p1);
                        }
                    }
                }

                // Add start, intermediate, and end markers
                markerPts.Add(waypoints[0]);
                for (int i = 1; i < waypoints.Count - 1; i += 2)
                {
                    markerPts.Add(waypoints[i]);
                }
                markerPts.Add(waypoints[waypoints.Count - 1]);
            }

            _plannedPathLines.Points = plannedPts;
            _followTrailCoreLines.Points = followPts;
            _followTrailGlowLines.Points = followPts;
            _waypointMarkers.Points = markerPts;
        }

        public void Clear()
        {
            _plannedPathLines.Points = new Point3DCollection();
            _followTrailCoreLines.Points = new Point3DCollection();
            _followTrailGlowLines.Points = new Point3DCollection();
            _waypointMarkers.Points = new Point3DCollection();
        }
    }
}
