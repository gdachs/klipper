# Code for handling the kinematics of core2xy robots
#
# Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2019       Gerald Dachs   <gda@dachsweb.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper, homing

class Core2XYKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.rails = [ stepper.PrinterRail(config.getsection('stepper_x')),
                       stepper.PrinterRail(config.getsection('stepper_y')),
                       stepper.LookupMultiRail(config.getsection('stepper_z')) ]
        self.rails[0].add_to_endstop(self.rails[1].get_endstops()[0][0])
        self.rails[1].add_to_endstop(self.rails[0].get_endstops()[0][0])
        self.rails[0].setup_itersolve('corexy_stepper_alloc', '+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', '-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', 'z')
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.need_motor_enable = True
        self.limits = [(1.0, -1.0)] * 3
        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        max_xy_halt_velocity = max_halt_velocity * math.sqrt(2.)
        max_xy_accel = max_accel * math.sqrt(2.)
        self.rails[0].set_max_jerk(max_xy_halt_velocity, max_xy_accel)
        self.rails[1].set_max_jerk(max_xy_halt_velocity, max_xy_accel)
        self.rails[2].set_max_jerk(
            min(max_halt_velocity, self.max_z_velocity), self.max_z_accel)
        # Check for dual carriage support
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            dc_axis = dc_config.getchoice('axis', {'x': 'x', 'y': 'y'})
            if dc_axis != 'x':
                raise config.error(
                    "Core2XY kinematic supports only a dual_carriage on the X axis")
            self.dual_carriage_axis = 0
            dc_rail = stepper.LookupMultiRail(dc_config)
            dc_rail.setup_itersolve('corexy_stepper_alloc', 'd')
            dc_rail.set_max_jerk(max_halt_velocity, max_accel)
            self.dual_carriage_rails = [self.rails[0], dc_rail]
            self.printer.lookup_object('gcode').register_command(
                'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
                desc=self.cmd_SET_DUAL_CARRIAGE_help)
        else:
            raise config.error(
                "Core2XY kinematic needs a dual_carriage section for the X axis")
    def get_steppers(self, flags=""):
        if flags == "Z":
            return self.rails[2].get_steppers()
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self):
        pos = [rail.get_commanded_position() for rail in self.rails]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def _home_rail(self, homing_state, axis, rail):
        # Determine movement an home rail
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if axis == self.dual_carriage_axis:
                dc1, dc2 = self.dual_carriage_rails
                altc = self.rails[axis] == dc2
                self._activate_carriage(0)
                self._home_rail(homing_state, axis, dc1)
                self._activate_carriage(1)
                self._home_rail(homing_state, axis, dc2)
                self._activate_carriage(altc)
            else:
                self._home_rail(homing_state, axis, self.rails[axis])
    def motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
        for rail in self.rails:
            rail.motor_enable(print_time, 0)
        self.dual_carriage_rails[1].motor_enable(print_time, 0)
        self.need_motor_enable = True
    def _check_motor_enable(self, print_time, move):
        if move.axes_d[0]:
            self.rails[0].motor_enable(print_time, 1)
            self.dual_carriage_rails[1].motor_enable(print_time, 1)
        if move.axes_d[1]:
            self.rails[1].motor_enable(print_time, 1)
        if move.axes_d[2]:
            self.rails[2].motor_enable(print_time, 1)
        need_motor_enable = False
        for rail in self.rails + self.dual_carriage_rails:
            need_motor_enable |= not rail.is_motor_enabled()
        self.need_motor_enable = need_motor_enable
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise homing.EndstopMoveError(
                        end_pos, "Must home axis first")
                raise homing.EndstopMoveError(end_pos)
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def move(self, print_time, move):
        if self.need_motor_enable:
            self._check_motor_enable(print_time, move)
        axes_d = move.axes_d
        cmove = move.cmove
        rail_x, rail_y, rail_z = self.rails
        if axes_d[0] or axes_d[1]:
            rail_x.step_itersolve(cmove)
            self.dual_carriage_rails[1].step_itersolve(cmove)
            rail_y.step_itersolve(cmove)
        if axes_d[2]:
            rail_z.step_itersolve(cmove)
    def get_status(self):
        return {'homed_axes': "".join([a
                    for a, (l, h) in zip("XYZ", self.limits) if l <= h])
        }
    # Dual carriage support
    def _activate_carriage(self, carriage):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.get_last_move_time()
        dc_rail = self.dual_carriage_rails[carriage]
        dc_axis = self.dual_carriage_axis
        self.rails[dc_axis] = dc_rail
        extruder_pos = toolhead.get_position()[3]
        toolhead.set_position(self.calc_position() + [extruder_pos])
        if self.limits[dc_axis][0] <= self.limits[dc_axis][1]:
            self.limits[dc_axis] = dc_rail.get_range()
        self.dual_carriage_rails[carriage].setup_itersolve('corexy_stepper_alloc', '+')
        self.dual_carriage_rails[1 - carriage].setup_itersolve('corexy_stepper_alloc', 'd')
        self.need_motor_enable = True
    cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"
    def cmd_SET_DUAL_CARRIAGE(self, params):
        gcode = self.printer.lookup_object('gcode')
        carriage = gcode.get_int('CARRIAGE', params, minval=0, maxval=1)
        self._activate_carriage(carriage)
        gcode.reset_last_position()

def load_kinematics(toolhead, config):
    return Core2XYKinematics(toolhead, config)
