# 6-Axis Kinematics for PAROL6 Robotic Arm (Klipper 6-Axis Motion Core)
#
# Supports native coordinated 6-axis motion planning (XYZABC)
# directly through Klipper's trapq and itersolve engines.
#
# Copyright (C) 2026 PAROL6 Ecosystem Project
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math
import stepper

class Parol6Kinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        self.axes = 'xyzabc'

        # Bind all 6 primary joint rails (stepper_x -> J1 ... stepper_c -> J6)
        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_' + n))
            for n in self.axes
        ]

        # Allocate C-helper itersolve for all 6 axes
        for rail, axis in zip(self.rails, self.axes):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())

        # Bind steppers to Klipper's primary lookahead trapq
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        # Register motor disable event
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)

        # Velocity & Acceleration limits
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_speed_x = config.getfloat('max_speed_x', max_velocity, above=0.)
        self.max_speed_y = config.getfloat('max_speed_y', max_velocity, above=0.)
        self.max_speed_z = config.getfloat('max_speed_z', max_velocity, above=0.)
        self.max_speed_a = config.getfloat('max_speed_a', max_velocity, above=0.)
        self.max_speed_b = config.getfloat('max_speed_b', max_velocity, above=0.)
        self.max_speed_c = config.getfloat('max_speed_c', max_velocity, above=0.)

        # Boundaries & Limits per axis
        self.limits = [(1.0, -1.0)] * len(self.axes)
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

    def get_steppers(self):
        """Return all steppers bound to all 6 rails."""
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        """Map stepper positions to axis coordinate list [x, y, z, a, b, c]."""
        return [stepper_positions[rail.get_name()] for rail in self.rails]

    def calc_tag_position(self):
        """Direct 1:1 joint angle retrieval in degrees from rails."""
        return [rail.get_commanded_position() for rail in self.rails]

    def calc_move_distance(self, start_pos, end_pos):
        """Calculate 6-axis Euclidean distance in joint space for toolhead pacing."""
        diffs = [
            end_pos[i] - start_pos[i]
            for i in range(min(len(start_pos), len(end_pos), len(self.axes)))
        ]
        dist = math.sqrt(sum([d * d for d in diffs]))
        if dist > 0.000000001:
            return dist
        if len(start_pos) > 6 and len(end_pos) > 6:
            return abs(end_pos[6] - start_pos[6])
        return 0.0

    def set_position(self, newpos, homing_axes=()):
        """Update commanded position registers for all 6 rails."""
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    def _motor_off(self, print_time):
        """Invalidate homed limits when motors are de-energized."""
        self.limits = [(1.0, -1.0)] * len(self.axes)

    def _check_endstops(self, move):
        """Validate destination against joint travel limits."""
        end_pos = move.end_pos
        for i in range(len(self.axes)):
            if move.axes_d[i]:
                pos = end_pos[i]
                l, h = self.limits[i]
                if pos < l or pos > h:
                    if l > h:
                        raise move.move_error("Must home joint J%d first" % (i + 1))
                    raise move.move_error(
                        "Joint J%d travel limit exceeded (%.3f not in [%.3f, %.3f])"
                        % (i + 1, pos, l, h)
                    )

    def check_move(self, move):
        """Check move constraints before queueing into trapq."""
        for i in range(len(self.axes)):
            if move.axes_d[i]:
                pos = move.end_pos[i]
                if pos < self.limits[i][0] or pos > self.limits[i][1]:
                    self._check_endstops(move)
                    break

    def _home_axis(self, homing_state, axis, rail):
        """Perform hardware endstop homing for a single joint rail."""
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None] * (len(self.axes) + 1)
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        """
        Execute sequential homing in PAROL6 physical collision-avoidance order:
        J2 (Shoulder) -> J3 (Elbow) -> J1 (Base) -> J4 (Roll) -> J6 (Wrist Roll) -> J5 (Pitch)
        """
        axes_to_home = homing_state.get_axes()
        parol6_order = [1, 2, 0, 3, 5, 4]
        for axis in parol6_order:
            if axis in axes_to_home and axis < len(self.rails):
                self._home_axis(homing_state, axis, self.rails[axis])

    def get_status(self, eventtime):
        """Expose kinematics status dictionary to Klippy."""
        axes = [a for a, (l, h) in zip(self.axes, self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return Parol6Kinematics(toolhead, config)
