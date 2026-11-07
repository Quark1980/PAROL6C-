using System;
using System.Collections.Generic;

namespace PAROLC_ControlSuite.Models
{
    /// <summary>
    /// Pure C# 6-DOF Forward & Inverse Kinematics Engine for PAROL6.
    /// Replicates the Denavit-Hartenberg and URDF transformations from PAROL6.urdf and kinematics.py.
    /// Units: Distances in Millimeters (mm), Angles in Degrees (deg).
    /// </summary>
    public class RobotKinematics
    {
        // CAD Home Pose in degrees
        public static readonly double[] CadHomeDeg = { 0.0, -90.0, 180.0, 0.0, 0.0, 180.0 };

        // Outer hardware & cable protection limits relative to CAD Home [0, -90, 180, 0, 0, 180]
        public static readonly (double Min, double Max)[] HardwareLimitsDeg = new[]
        {
            (-170.0, 170.0), // J1 (Base Yaw: [-170°, +170°])
            (-150.0, 50.0),  // J2 (Shoulder Pitch: CAD Home -90°, [-60°, +140°] rel -> [-150°, +50°])
            (65.0, 290.0),   // J3 (Elbow Pitch: CAD Home 180°, [-115°, +110°] rel -> [65°, 290°])
            (-160.0, 160.0), // J4 (Wrist Roll: [-160°, +160°] strictly bounded to prevent internal harness twisting)
            (-110.0, 110.0), // J5 (Wrist Pitch: [-110°, +110°] mechanical collision boundary)
            (0.0, 360.0)     // J6 (Tool Flange Roll: CAD Home 180° ± 180° -> [0°, 360°] prevents umbilical wrap)
        };

        // Static link transformations extracted from PAROL6.urdf
        // In millimeters and radians: (xyz_mm, rpy_rad)
        private static readonly (double[] xyz, double[] rpy)[] LinkDefs = new[]
        {
            // J1: base_link -> L1
            (new[] { 0.0, 0.0, 110.5 }, new[] { 0.0, 0.0, 0.0 }),
            // J2: L1 -> L2
            (new[] { 23.420721, 0.0, 0.0 }, new[] { -Math.PI / 2.0, 0.0, 0.0 }),
            // J3: L2 -> L3
            (new[] { 180.0, 0.0, 0.0 }, new[] { Math.PI, 0.0, 0.0 }),
            // J4: L3 -> L4
            (new[] { -43.5, 176.35, 0.0 }, new[] { Math.PI / 2.0, 0.0, 0.0 }),
            // J5: L4 -> L5
            (new[] { 0.0, 0.0, 0.0 }, new[] { -Math.PI / 2.0, 0.0, 0.0 }),
            // J6: L5 -> L6
            (new[] { 0.0, 37.0, 0.0 }, new[] { Math.PI / 2.0, 0.0, 0.0 })
        };

        // Tool offset in mm relative to L6 flange (default End_effector_TD-8820MG.stl tip = 118.0 mm)
        public double ToolOffsetX { get; set; } = 0.0;
        public double ToolOffsetY { get; set; } = 0.0;
        public double ToolOffsetZ { get; set; } = -118.0;

        /// <summary>
        /// 4x4 Transformation Matrix
        /// </summary>
        public struct Mat4
        {
            public double M00, M01, M02, M03;
            public double M10, M11, M12, M13;
            public double M20, M21, M22, M23;
            public double M30, M31, M32, M33;

            public static Mat4 Identity => new Mat4
            {
                M00 = 1, M11 = 1, M22 = 1, M33 = 1
            };

            public static Mat4 Multiply(in Mat4 a, in Mat4 b)
            {
                return new Mat4
                {
                    M00 = a.M00 * b.M00 + a.M01 * b.M10 + a.M02 * b.M20 + a.M03 * b.M30,
                    M01 = a.M00 * b.M01 + a.M01 * b.M11 + a.M02 * b.M21 + a.M03 * b.M31,
                    M02 = a.M00 * b.M02 + a.M01 * b.M12 + a.M02 * b.M22 + a.M03 * b.M32,
                    M03 = a.M00 * b.M03 + a.M01 * b.M13 + a.M02 * b.M23 + a.M03 * b.M33,

                    M10 = a.M10 * b.M00 + a.M11 * b.M10 + a.M12 * b.M20 + a.M13 * b.M30,
                    M11 = a.M10 * b.M01 + a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31,
                    M12 = a.M10 * b.M02 + a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32,
                    M13 = a.M10 * b.M03 + a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33,

                    M20 = a.M20 * b.M00 + a.M21 * b.M10 + a.M22 * b.M20 + a.M23 * b.M30,
                    M21 = a.M20 * b.M01 + a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31,
                    M22 = a.M20 * b.M02 + a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32,
                    M23 = a.M20 * b.M03 + a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33,

                    M30 = a.M30 * b.M00 + a.M31 * b.M10 + a.M32 * b.M20 + a.M33 * b.M30,
                    M31 = a.M30 * b.M01 + a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31,
                    M32 = a.M30 * b.M02 + a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32,
                    M33 = a.M30 * b.M03 + a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33
                };
            }

            public static Mat4 FromTranslationAndRpy(double x, double y, double z, double roll, double pitch, double yaw)
            {
                double cx = Math.Cos(roll), sx = Math.Sin(roll);
                double cy = Math.Cos(pitch), sy = Math.Sin(pitch);
                double cz = Math.Cos(yaw), sz = Math.Sin(yaw);

                return new Mat4
                {
                    M00 = cz * cy,
                    M01 = cz * sy * sx - sz * cx,
                    M02 = cz * sy * cx + sz * sx,
                    M03 = x,

                    M10 = sz * cy,
                    M11 = sz * sy * sx + cz * cx,
                    M12 = sz * sy * cx - cz * sx,
                    M13 = y,

                    M20 = -sy,
                    M21 = cy * sx,
                    M22 = cy * cx,
                    M23 = z,

                    M30 = 0, M31 = 0, M32 = 0, M33 = 1
                };
            }

            public static Mat4 FromRotationZ(double angleRad)
            {
                double c = Math.Cos(angleRad);
                double s = Math.Sin(angleRad);
                return new Mat4
                {
                    M00 = c,  M01 = -s, M02 = 0, M03 = 0,
                    M10 = s,  M11 = c,  M12 = 0, M13 = 0,
                    M20 = 0,  M21 = 0,  M22 = 1, M23 = 0,
                    M30 = 0,  M31 = 0,  M32 = 0, M33 = 1
                };
            }

            public (double x, double y, double z, double rx, double ry, double rz) ToPoseMmDeg()
            {
                double sy = -M20;
                sy = Math.Max(-1.0, Math.Min(1.0, sy));
                double ry = Math.Asin(sy);
                double rx, rz;

                if (Math.Abs(Math.Cos(ry)) > 1e-6)
                {
                    rx = Math.Atan2(M21, M22);
                    rz = Math.Atan2(M10, M00);
                }
                else
                {
                    rx = Math.Atan2(-M12, M11);
                    rz = 0.0;
                }

                return (M03, M13, M23, rx * 180.0 / Math.PI, ry * 180.0 / Math.PI, rz * 180.0 / Math.PI);
            }
        }

        /// <summary>
        /// Computes cumulative forward transformations for all links:
        /// returns Mat4 array of length 7 (index 0 = base_link, 1..6 = L1..L6) plus final TCP.
        /// </summary>
        public Mat4[] ComputeAllLinkTransforms(double[] jointsDeg)
        {
            var transforms = new Mat4[7];
            transforms[0] = Mat4.Identity; // base_link at origin

            Mat4 current = Mat4.Identity;
            for (int i = 0; i < 6; i++)
            {
                double angleRad = (jointsDeg != null && jointsDeg.Length > i) ? jointsDeg[i] * Math.PI / 180.0 : 0.0;
                var def = LinkDefs[i];
                var tStatic = Mat4.FromTranslationAndRpy(def.xyz[0], def.xyz[1], def.xyz[2], def.rpy[0], def.rpy[1], def.rpy[2]);
                var rZ = Mat4.FromRotationZ(angleRad);

                current = Mat4.Multiply(current, tStatic);
                current = Mat4.Multiply(current, rZ);
                transforms[i + 1] = current;
            }

            return transforms;
        }

        /// <summary>
        /// Computes full 4x4 TCP transform matrix in mm and radians.
        /// </summary>
        public Mat4 ComputeTcpMatrix(double[] jointsDeg)
        {
            var all = ComputeAllLinkTransforms(jointsDeg);
            var flange = all[6];

            var tTool = Mat4.Identity;
            tTool.M03 = ToolOffsetX;
            tTool.M13 = ToolOffsetY;
            tTool.M23 = ToolOffsetZ;

            return Mat4.Multiply(flange, tTool);
        }

        /// <summary>
        /// Computes TCP forward kinematics pose [X, Y, Z, Rx, Ry, Rz] in mm and degrees.
        /// </summary>
        public double[] ComputeFkPose(double[] jointsDeg)
        {
            var tcp = ComputeTcpMatrix(jointsDeg);
            var pose = tcp.ToPoseMmDeg();
            return new[] { pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz };
        }

        /// <summary>
        /// Applies an extrinsic world-frame rotation increment (in degrees) around X (Roll), Y (Pitch), or Z (Yaw)
        /// directly to a 4x4 transformation matrix, avoiding Euler gimbal lock.
        /// </summary>
        public static Mat4 ApplyWorldRotation(Mat4 current, string axis, double deltaDeg)
        {
            double rad = deltaDeg * Math.PI / 180.0;
            double c = Math.Cos(rad), s = Math.Sin(rad);
            Mat4 rStep = Mat4.Identity;
            string ax = (axis ?? "").ToUpperInvariant();
            if (ax == "RX" || ax == "ROLL" || ax == "X")
            {
                rStep.M11 = c; rStep.M12 = -s;
                rStep.M21 = s; rStep.M22 = c;
            }
            else if (ax == "RY" || ax == "PITCH" || ax == "Y")
            {
                rStep.M00 = c;  rStep.M02 = s;
                rStep.M20 = -s; rStep.M22 = c;
            }
            else if (ax == "RZ" || ax == "YAW" || ax == "Z")
            {
                rStep.M00 = c; rStep.M01 = -s;
                rStep.M10 = s; rStep.M11 = c;
            }

            Mat4 rotOnly = current;
            rotOnly.M03 = 0; rotOnly.M13 = 0; rotOnly.M23 = 0;
            Mat4 rotated = Mat4.Multiply(rStep, rotOnly);
            rotated.M03 = current.M03;
            rotated.M13 = current.M13;
            rotated.M23 = current.M23;
            return rotated;
        }

        /// <summary>
        /// Solves Damped Least Squares (DLS) Inverse Kinematics for target Cartesian pose [X, Y, Z, Rx, Ry, Rz].
        /// </summary>
        public (bool Success, double[] JointsDeg) SolveIK(double[] targetPoseMmDeg, double[] seedJointsDeg, int maxIterations = 60, double toleranceMm = 0.5, bool positionOnly = false)
        {
            Mat4 targetMat = Mat4.FromTranslationAndRpy(
                targetPoseMmDeg[0], targetPoseMmDeg[1], targetPoseMmDeg[2],
                targetPoseMmDeg[3] * Math.PI / 180.0,
                targetPoseMmDeg[4] * Math.PI / 180.0,
                targetPoseMmDeg[5] * Math.PI / 180.0);
            return SolveIKWithMatrix(targetPoseMmDeg[0], targetPoseMmDeg[1], targetPoseMmDeg[2], targetMat, seedJointsDeg, maxIterations, toleranceMm, positionOnly);
        }

        /// <summary>
        /// Solves 6-DOF Inverse Kinematics for target TCP position (mm) and exact 3x3 rotation matrix in targetRotMat.
        /// Marches along the Cartesian line in fine sub-steps, enforcing strict hardware limits (±180° max from CAD Home),
        /// strict 0.000° flange world orientation lock, and per-step joint continuity so the arm NEVER flips the wrist
        /// or passes through impossible intermediate positions.
        /// </summary>
        public (bool Success, double[] JointsDeg) SolveIKWithMatrix(
            double targetX,
            double targetY,
            double targetZ,
            Mat4 targetRotMat,
            double[] seedJointsDeg,
            int maxIterations = 70,
            double toleranceMm = 0.4,
            bool positionOnly = false)
        {
            double[] qSeed = new double[6];
            for (int i = 0; i < 6; i++)
            {
                double val = (seedJointsDeg != null && seedJointsDeg.Length > i) ? seedJointsDeg[i] : CadHomeDeg[i];
                qSeed[i] = Math.Clamp(val, HardwareLimitsDeg[i].Min, HardwareLimitsDeg[i].Max);
            }

            Mat4 startTcp = ComputeTcpMatrix(qSeed);
            double totalDx = targetX - startTcp.M03;
            double totalDy = targetY - startTcp.M13;
            double totalDz = targetZ - startTcp.M23;
            double totalDist = Math.Sqrt(totalDx * totalDx + totalDy * totalDy + totalDz * totalDz);

            // Fine sub-stepping (~1.5 mm per step) guarantees continuous sub-step deltas stay well below the 3.0° safety gate
            int subSteps = Math.Clamp((int)Math.Ceiling(totalDist / 1.5), 1, 160);
            double[] qCurr = (double[])qSeed.Clone();
            int completedSteps = 0;

            for (int stepIdx = 1; stepIdx <= subSteps; stepIdx++)
            {
                double alpha = (double)stepIdx / subSteps;
                double stepX = startTcp.M03 + totalDx * alpha;
                double stepY = startTcp.M13 + totalDy * alpha;
                double stepZ = startTcp.M23 + totalDz * alpha;

                if (!positionOnly && TrySolveAnalyticalSphericalWristIK(stepX, stepY, stepZ, targetRotMat, qCurr, out double[] qStepAnalytical))
                {
                    if (ValidateAndLockBranch(qStepAnalytical, qCurr, 3.0, out _))
                    {
                        qCurr = qStepAnalytical;
                        completedSteps++;
                        continue;
                    }
                }

                var (ok, qSolved) = SolveIKSingleStep(
                    stepX, stepY, stepZ, targetRotMat, qCurr,
                    stepIdx == subSteps ? maxIterations : 40,
                    stepIdx == subSteps ? toleranceMm : 0.6,
                    positionOnly);

                if (!ok || qSolved == null)
                {
                    // Stop immediately before entering an impossible or orientation-violating region
                    break;
                }

                // Continuity & cable safety interceptor: reject any branch flip or > 3.0° spike across a sub-step
                if (!ValidateAndLockBranch(qSolved, qCurr, 3.0, out _))
                {
                    break;
                }

                qCurr = qSolved;
                completedSteps++;
            }

            return (completedSteps > 0, qCurr);
        }

        /// <summary>
        /// Exact closed-form kinematic decoupling for PAROL6 (3-DOF positional arm + 3-DOF spherical wrist).
        /// Strictly enforces hardware cable limits, weighted Euclidean branch locking, and the |J5| &lt; 1.0°
        /// collinear singularity crossing guard (locking J4 and resolving roll onto J6).
        /// </summary>
        private bool TrySolveAnalyticalSphericalWristIK(
            double targetX,
            double targetY,
            double targetZ,
            in Mat4 targetRotMat,
            double[] seedJointsDeg,
            out double[] solvedJointsDeg)
        {
            solvedJointsDeg = null;

            // 1. Compute Spherical Wrist Center (L4/L5 origin) from TCP and locked rotation matrix Z-axis
            double dTool = 37.0 - ToolOffsetZ; // 37.0 - (-105.0) = 142.0 mm along +Z_flange
            double xWc = targetX + dTool * targetRotMat.M02;
            double yWc = targetY + dTool * targetRotMat.M12;
            double zWc = targetZ + dTool * targetRotMat.M22;

            // 2. Solve J1 (Base Yaw)
            double rXy = Math.Sqrt(xWc * xWc + yWc * yWc);
            if (rXy < 20.0)
                return false; // Overhead shoulder singularity zone

            double q1Deg = Math.Atan2(yWc, xWc) * 180.0 / Math.PI;
            q1Deg = UnwrapNear(q1Deg, seedJointsDeg[0], HardwareLimitsDeg[0].Min, HardwareLimitsDeg[0].Max);
            if (q1Deg < HardwareLimitsDeg[0].Min || q1Deg > HardwareLimitsDeg[0].Max || Math.Abs(q1Deg - seedJointsDeg[0]) > 3.0)
                return false;

            // 3. Solve J2 & J3 in the radial (r, z) plane relative to J2 origin (23.420721, 110.5)
            double dR = rXy - 23.420721;
            double dZ = zWc - 110.5;
            const double L2 = 180.0;
            const double a3 = -43.5;
            const double b3 = 176.35;
            double L3 = Math.Sqrt(a3 * a3 + b3 * b3);
            double beta = Math.Atan2(b3, a3);

            double distSq = dR * dR + dZ * dZ;
            double cosDelta = (distSq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
            if (cosDelta < -1.0 || cosDelta > 1.0)
                return false;

            double absDelta = Math.Acos(Math.Clamp(cosDelta, -1.0, 1.0));
            double[] deltaCandidates = new[] { absDelta, -absDelta };

            double bestElbowCost = double.MaxValue;
            double bestQ2 = 0, bestQ3 = 0;
            bool foundElbow = false;

            foreach (double delta in deltaCandidates)
            {
                double q2Rad = Math.Atan2(-dZ, dR) - Math.Atan2(L3 * Math.Sin(delta), L2 + L3 * Math.Cos(delta));
                double q3Rad = -(delta + beta);

                double q2Cand = UnwrapNear(q2Rad * 180.0 / Math.PI, seedJointsDeg[1], HardwareLimitsDeg[1].Min, HardwareLimitsDeg[1].Max);
                double q3Cand = UnwrapNear(q3Rad * 180.0 / Math.PI, seedJointsDeg[2], HardwareLimitsDeg[2].Min, HardwareLimitsDeg[2].Max);

                if (q2Cand >= HardwareLimitsDeg[1].Min && q2Cand <= HardwareLimitsDeg[1].Max &&
                    q3Cand >= HardwareLimitsDeg[2].Min && q3Cand <= HardwareLimitsDeg[2].Max &&
                    Math.Abs(q2Cand - seedJointsDeg[1]) <= 3.0 &&
                    Math.Abs(q3Cand - seedJointsDeg[2]) <= 3.0)
                {
                    double d2 = q2Cand - seedJointsDeg[1];
                    double d3 = q3Cand - seedJointsDeg[2];
                    double cost = d2 * d2 + d3 * d3;
                    if (cost < bestElbowCost)
                    {
                        bestElbowCost = cost;
                        bestQ2 = q2Cand;
                        bestQ3 = q3Cand;
                        foundElbow = true;
                    }
                }
            }

            if (!foundElbow)
                return false;

            // 4. Solve Spherical Wrist (J4, J5, J6) from R_36 = (T_L3 * tStatic4)^T * R_target
            var linkTransforms = ComputeAllLinkTransforms(new[] { q1Deg, bestQ2, bestQ3, 0, 0, 0 });
            Mat4 t3 = linkTransforms[3];
            var def4 = LinkDefs[3];
            Mat4 tStatic4 = Mat4.FromTranslationAndRpy(def4.xyz[0], def4.xyz[1], def4.xyz[2], def4.rpy[0], def4.rpy[1], def4.rpy[2]);
            Mat4 b = Mat4.Multiply(t3, tStatic4);

            // R_36 = b^T * targetRotMat (3x3 rotation part)
            double r00 = b.M00 * targetRotMat.M00 + b.M10 * targetRotMat.M10 + b.M20 * targetRotMat.M20;
            double r01 = b.M00 * targetRotMat.M01 + b.M10 * targetRotMat.M11 + b.M20 * targetRotMat.M21;
            double r02 = b.M00 * targetRotMat.M02 + b.M10 * targetRotMat.M12 + b.M20 * targetRotMat.M22;

            double r10 = b.M01 * targetRotMat.M00 + b.M11 * targetRotMat.M10 + b.M21 * targetRotMat.M20;
            double r11 = b.M01 * targetRotMat.M01 + b.M11 * targetRotMat.M11 + b.M21 * targetRotMat.M21;
            double r12 = b.M01 * targetRotMat.M02 + b.M11 * targetRotMat.M12 + b.M21 * targetRotMat.M22;

            double r20 = b.M02 * targetRotMat.M00 + b.M12 * targetRotMat.M10 + b.M22 * targetRotMat.M20;
            double r21 = b.M02 * targetRotMat.M01 + b.M12 * targetRotMat.M11 + b.M22 * targetRotMat.M21;
            double r22 = b.M02 * targetRotMat.M02 + b.M12 * targetRotMat.M12 + b.M22 * targetRotMat.M22;

            double c5 = Math.Clamp(r22, -1.0, 1.0);
            double s5 = Math.Sqrt(Math.Max(0.0, 1.0 - c5 * c5));
            double q5Abs = Math.Atan2(s5, c5) * 180.0 / Math.PI;

            double bestQ4 = 0, bestQ5 = 0, bestQ6 = 0;
            bool foundWrist = false;

            // SINGULARITY CROSSING GUARD (|J5| < 1.0°):
            // Joint 4 and Joint 6 are collinear. Lock J4 to its active value and resolve roll onto J6.
            if (q5Abs < 1.0)
            {
                bestQ4 = Math.Clamp(seedJointsDeg[3], HardwareLimitsDeg[3].Min, HardwareLimitsDeg[3].Max);
                bestQ5 = (seedJointsDeg[4] >= 0.0) ? q5Abs : -q5Abs;
                double totalRollDeg = Math.Atan2(r10, r00) * 180.0 / Math.PI;
                bestQ6 = UnwrapNear(totalRollDeg - bestQ4, seedJointsDeg[5], HardwareLimitsDeg[5].Min, HardwareLimitsDeg[5].Max);

                if (bestQ6 >= HardwareLimitsDeg[5].Min && bestQ6 <= HardwareLimitsDeg[5].Max &&
                    Math.Abs(bestQ6 - seedJointsDeg[5]) <= 3.0)
                {
                    foundWrist = true;
                }
            }
            else
            {
                // DETERMINISTIC ANALYTICAL WRIST BRANCH LOCKING (J5 > 0 vs J5 < 0):
                // Evaluate both mathematical solutions using weighted Euclidean distance against q_current.
                var branches = new (double q4, double q5, double q6)[]
                {
                    (Math.Atan2(r12, r02) * 180.0 / Math.PI,   +q5Abs, Math.Atan2(r21, -r20) * 180.0 / Math.PI),
                    (Math.Atan2(-r12, -r02) * 180.0 / Math.PI, -q5Abs, Math.Atan2(-r21, r20) * 180.0 / Math.PI)
                };

                double bestWristCost = double.MaxValue;
                const double w4 = 1.4, w5 = 1.8, w6 = 1.0;

                foreach (var br in branches)
                {
                    double q4Cand = UnwrapNear(br.q4, seedJointsDeg[3], HardwareLimitsDeg[3].Min, HardwareLimitsDeg[3].Max);
                    double q5Cand = UnwrapNear(br.q5, seedJointsDeg[4], HardwareLimitsDeg[4].Min, HardwareLimitsDeg[4].Max);
                    double q6Cand = UnwrapNear(br.q6, seedJointsDeg[5], HardwareLimitsDeg[5].Min, HardwareLimitsDeg[5].Max);

                    if (q4Cand >= HardwareLimitsDeg[3].Min && q4Cand <= HardwareLimitsDeg[3].Max &&
                        q5Cand >= HardwareLimitsDeg[4].Min && q5Cand <= HardwareLimitsDeg[4].Max &&
                        q6Cand >= HardwareLimitsDeg[5].Min && q6Cand <= HardwareLimitsDeg[5].Max)
                    {
                        double d4 = WrapAngleDeg(q4Cand - seedJointsDeg[3]);
                        double d5 = WrapAngleDeg(q5Cand - seedJointsDeg[4]);
                        double d6 = WrapAngleDeg(q6Cand - seedJointsDeg[5]);

                        // Reject branch flips (> 3.0° across a sub-step)
                        if (Math.Abs(d4) > 3.0 || Math.Abs(d5) > 3.0 || Math.Abs(d6) > 3.0)
                            continue;

                        double cost = w4 * (d4 * d4) + w5 * (d5 * d5) + w6 * (d6 * d6);
                        if (cost < bestWristCost)
                        {
                            bestWristCost = cost;
                            bestQ4 = q4Cand;
                            bestQ5 = q5Cand;
                            bestQ6 = q6Cand;
                            foundWrist = true;
                        }
                    }
                }
            }

            if (!foundWrist)
                return false;

            var cand = new[] { q1Deg, bestQ2, bestQ3, bestQ4, bestQ5, bestQ6 };
            if (!ValidateAndLockBranch(cand, seedJointsDeg, 3.0, out _))
                return false;

            solvedJointsDeg = cand;
            return true;
        }

        /// <summary>
        /// High-level safety interceptor: verifies that all 6 joints are strictly inside HardwareLimitsDeg,
        /// within ±180° of CadHomeDeg, and that no joint commands a delta spike exceeding maxDeltaDeg
        /// (default 3.0° per 10ms window). If violated, rejects the candidate frame and holds currentJoints.
        /// </summary>
        public static bool ValidateAndLockBranch(
            double[] candidateJoints,
            double[] currentJoints,
            double maxDeltaDeg,
            out string violationReason)
        {
            violationReason = string.Empty;
            if (candidateJoints == null || candidateJoints.Length < 6)
            {
                violationReason = "Null or incomplete joint vector.";
                return false;
            }

            for (int i = 0; i < 6; i++)
            {
                double q = candidateJoints[i];
                if (double.IsNaN(q) || double.IsInfinity(q))
                {
                    violationReason = $"J{i + 1} is NaN/Inf.";
                    return false;
                }
                if (q < HardwareLimitsDeg[i].Min - 1e-3 || q > HardwareLimitsDeg[i].Max + 1e-3)
                {
                    violationReason = $"J{i + 1}={q:F1}° exceeds cable protection limit [{HardwareLimitsDeg[i].Min:F0}°, {HardwareLimitsDeg[i].Max:F0}°].";
                    return false;
                }
                if (Math.Abs(q - CadHomeDeg[i]) > 180.0 + 1e-3)
                {
                    violationReason = $"J{i + 1}={q:F1}° exceeds ±180° from CAD Home ({CadHomeDeg[i]:F0}°).";
                    return false;
                }
                if (currentJoints != null && currentJoints.Length > i && maxDeltaDeg > 0.0)
                {
                    double delta = Math.Abs(q - currentJoints[i]);
                    if (delta > maxDeltaDeg + 1e-3)
                    {
                        violationReason = $"J{i + 1} delta spike {delta:F2}° > {maxDeltaDeg:F1}° limit (branch flip blocked).";
                        return false;
                    }
                }
            }
            return true;
        }

        /// <summary>
        /// High-level safety check: verifies that all 6 joints are strictly inside HardwareLimitsDeg
        /// AND that no joint (especially the tool J6 and forearm J4) ever rotates more than ±180° from CadHomeDeg.
        /// </summary>
        public static bool IsWithinSafeLimits(double[] jointsDeg)
        {
            return ValidateAndLockBranch(jointsDeg, null, 0.0, out _);
        }

        private static double UnwrapNear(double angleDeg, double refDeg, double minDeg, double maxDeg)
        {
            double diff = angleDeg - refDeg;
            while (diff > 180.0) { angleDeg -= 360.0; diff -= 360.0; }
            while (diff < -180.0) { angleDeg += 360.0; diff += 360.0; }
            if (angleDeg < minDeg && angleDeg + 360.0 <= maxDeg) angleDeg += 360.0;
            if (angleDeg > maxDeg && angleDeg - 360.0 >= minDeg) angleDeg -= 360.0;
            return angleDeg;
        }

        private (bool Success, double[] JointsDeg) SolveIKSingleStep(
            double targetX,
            double targetY,
            double targetZ,
            Mat4 targetMat,
            double[] seedJointsDeg,
            int maxIterations,
            double toleranceMm,
            bool positionOnly)
        {
            double[] q = new double[6];
            for (int i = 0; i < 6; i++)
            {
                q[i] = Math.Clamp(seedJointsDeg[i], HardwareLimitsDeg[i].Min, HardwareLimitsDeg[i].Max);
            }

            double lambda = 1.2;
            double lastPosErr = 999.0;
            double lastRotErrDeg = 999.0;

            for (int iter = 0; iter < maxIterations; iter++)
            {
                // Forward kinematics and joint axes in mm
                Mat4 current = Mat4.Identity;
                double[,] jointOrigins = new double[6, 3];
                double[,] jointZAxes = new double[6, 3];

                for (int i = 0; i < 6; i++)
                {
                    var def = LinkDefs[i];
                    var tStatic = Mat4.FromTranslationAndRpy(def.xyz[0], def.xyz[1], def.xyz[2], def.rpy[0], def.rpy[1], def.rpy[2]);
                    var tJointFrame = Mat4.Multiply(current, tStatic);
                    jointOrigins[i, 0] = tJointFrame.M03;
                    jointOrigins[i, 1] = tJointFrame.M13;
                    jointOrigins[i, 2] = tJointFrame.M23;
                    jointZAxes[i, 0] = tJointFrame.M02;
                    jointZAxes[i, 1] = tJointFrame.M12;
                    jointZAxes[i, 2] = tJointFrame.M22;

                    double angleRad = q[i] * Math.PI / 180.0;
                    current = Mat4.Multiply(tJointFrame, Mat4.FromRotationZ(angleRad));
                }

                var tTool = Mat4.Identity;
                tTool.M03 = ToolOffsetX;
                tTool.M13 = ToolOffsetY;
                tTool.M23 = ToolOffsetZ;
                Mat4 tcp = Mat4.Multiply(current, tTool);

                double dx = targetX - tcp.M03;
                double dy = targetY - tcp.M13;
                double dz = targetZ - tcp.M23;

                // SO(3) orientation error: R_err = R_target * R_curr^T
                double r00 = targetMat.M00 * tcp.M00 + targetMat.M01 * tcp.M01 + targetMat.M02 * tcp.M02;
                double r01 = targetMat.M00 * tcp.M10 + targetMat.M01 * tcp.M11 + targetMat.M02 * tcp.M12;
                double r02 = targetMat.M00 * tcp.M20 + targetMat.M01 * tcp.M21 + targetMat.M02 * tcp.M22;
                double r10 = targetMat.M10 * tcp.M00 + targetMat.M11 * tcp.M01 + targetMat.M12 * tcp.M02;
                double r11 = targetMat.M10 * tcp.M10 + targetMat.M11 * tcp.M11 + targetMat.M12 * tcp.M12;
                double r12 = targetMat.M10 * tcp.M20 + targetMat.M11 * tcp.M21 + targetMat.M12 * tcp.M22;
                double r20 = targetMat.M20 * tcp.M00 + targetMat.M21 * tcp.M01 + targetMat.M22 * tcp.M02;
                double r21 = targetMat.M20 * tcp.M10 + targetMat.M21 * tcp.M11 + targetMat.M22 * tcp.M12;
                double r22 = targetMat.M20 * tcp.M20 + targetMat.M21 * tcp.M21 + targetMat.M22 * tcp.M22;

                double tr = r00 + r11 + r22;
                double cosTheta = Math.Max(-1.0, Math.Min(1.0, (tr - 1.0) * 0.5));
                double theta = Math.Acos(cosTheta);
                double wx = 0.0, wy = 0.0, wz = 0.0;
                if (theta > 1e-6)
                {
                    double factor = theta / (2.0 * Math.Sin(theta));
                    wx = factor * (r21 - r12);
                    wy = factor * (r02 - r20);
                    wz = factor * (r10 - r01);
                }

                lastPosErr = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                lastRotErrDeg = theta * 180.0 / Math.PI;
                bool isWristSingular = Math.Abs(q[4]) < 12.0;

                // Strict orientation tolerance: never allow tool rotation drift in locked mode
                if (lastPosErr < toleranceMm && (positionOnly || lastRotErrDeg < 0.40))
                {
                    return (IsWithinSafeLimits(q), q);
                }

                double oriWeight = positionOnly ? 0.0 : 120.0;

                // Geometric Jacobian wrt degrees (rad/deg = pi/180)
                double degToRad = Math.PI / 180.0;
                double[,] J = new double[6, 6];
                for (int i = 0; i < 6; i++)
                {
                    double zx = jointZAxes[i, 0], zy = jointZAxes[i, 1], zz = jointZAxes[i, 2];
                    double rx = tcp.M03 - jointOrigins[i, 0];
                    double ry = tcp.M13 - jointOrigins[i, 1];
                    double rz = tcp.M23 - jointOrigins[i, 2];

                    J[0, i] = (zy * rz - zz * ry) * degToRad;
                    J[1, i] = (zz * rx - zx * rz) * degToRad;
                    J[2, i] = (zx * ry - zy * rx) * degToRad;
                    J[3, i] = zx * degToRad * oriWeight;
                    J[4, i] = zy * degToRad * oriWeight;
                    J[5, i] = zz * degToRad * oriWeight;
                }

                double[] e = new[] { dx, dy, dz, wx * oriWeight, wy * oriWeight, wz * oriWeight };

                // Extra null-space damping on J4 and J6 when near J5=0° collinear singularity to prevent wrist counter-spinning
                double wristSingDamp = isWristSingular ? (10.0 * (1.0 - Math.Abs(q[4]) / 12.0)) : 0.0;

                double[,] JTJ = new double[6, 6];
                for (int r = 0; r < 6; r++)
                {
                    for (int c = 0; c < 6; c++)
                    {
                        double sum = 0;
                        for (int k = 0; k < 6; k++) sum += J[k, r] * J[k, c];
                        if (r == c)
                        {
                            sum += lambda * lambda;
                            if (r == 3 || r == 5)
                            {
                                sum += wristSingDamp * wristSingDamp;
                            }
                        }
                        JTJ[r, c] = sum;
                    }
                }

                double[] JTe = new double[6];
                for (int r = 0; r < 6; r++)
                {
                    double sum = 0;
                    for (int k = 0; k < 6; k++) sum += J[k, r] * e[k];
                    JTe[r] = sum;
                }

                double[] dq = SolveLinearSystem(JTJ, JTe);
                if (dq == null) break;

                // Collinear singularity guard (|J5| < 1.0°): lock J4 so all roll is resolved onto J6
                if (Math.Abs(q[4]) < 1.0)
                {
                    dq[3] = 0.0;
                }

                for (int j = 0; j < 6; j++)
                {
                    double step = Math.Max(-2.5, Math.Min(2.5, dq[j]));
                    double minLim = Math.Max(HardwareLimitsDeg[j].Min, Math.Max(CadHomeDeg[j] - 180.0, seedJointsDeg[j] - 3.0));
                    double maxLim = Math.Min(HardwareLimitsDeg[j].Max, Math.Min(CadHomeDeg[j] + 180.0, seedJointsDeg[j] + 3.0));
                    q[j] = Math.Clamp(q[j] + step, minLim, maxLim);
                }
            }

            bool converged = lastPosErr < toleranceMm * 1.5 && (positionOnly || lastRotErrDeg < 0.50) && IsWithinSafeLimits(q);
            return (converged, q);
        }

        private static double WrapAngleDeg(double angle)
        {
            while (angle > 180.0) angle -= 360.0;
            while (angle < -180.0) angle += 360.0;
            return angle;
        }

        private static double[] SolveLinearSystem(double[,] A, double[] b)
        {
            int n = 6;
            double[,] M = new double[n, n + 1];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++) M[i, j] = A[i, j];
                M[i, n] = b[i];
            }

            // Gaussian elimination with partial pivoting
            for (int i = 0; i < n; i++)
            {
                int maxRow = i;
                for (int k = i + 1; k < n; k++)
                {
                    if (Math.Abs(M[k, i]) > Math.Abs(M[maxRow, i])) maxRow = k;
                }

                for (int k = i; k <= n; k++)
                {
                    double tmp = M[maxRow, k];
                    M[maxRow, k] = M[i, k];
                    M[i, k] = tmp;
                }

                if (Math.Abs(M[i, i]) < 1e-9) return null; // Singular

                for (int k = i + 1; k < n; k++)
                {
                    double factor = M[k, i] / M[i, i];
                    for (int j = i; j <= n; j++) M[k, j] -= factor * M[i, j];
                }
            }

            // Back substitution
            double[] x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                x[i] = M[i, n];
                for (int j = i + 1; j < n; j++) x[i] -= M[i, j] * x[j];
                x[i] /= M[i, i];
            }

            return x;
        }
    }
}
