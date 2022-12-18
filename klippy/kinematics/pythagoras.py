# Code for handling the kinematics of pythagoras robot
#
# Copyright (C) 2022  Dmitry Lavrov
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper


class PythagorasKinematics:
    def __init__(self, toolhead, config):
        # Setup axis steppers
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abz']
        rail_a = stepper.LookupMultiRail(stepper_configs[0], need_position_minmax = False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=a_endstop)
        rail_z = stepper.LookupMultiRail(stepper_configs[2])
        self.rails = [rail_a, rail_b, rail_z]

        self.kin_params = []

        for i in range(0,2):
            kin_params=(
                stepper_configs[i].getfloat('pulley_x'), 
                stepper_configs[i].getfloat('pulley_y'),
                stepper_configs[i].getfloat('pulley_r'),
                stepper_configs[i].getfloat('tip_r'),
                stepper_configs[i].getfloat('position_max')
                )
            self.kin_params.append(kin_params);
            self.rails[i].setup_itersolve('pythagoras_stepper_alloc',
                *kin_params[:4])

        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
        
        self.steppers=[s for rail in self.rails for s in rail.get_steppers()]

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        
        # self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)

    def get_steppers(self):
        return self.steppers

    def _calc_stepper_from_xy(rail, x, y):
        return rail.calc_position_from_coord( (x,y,0) )        
    
    # TODO: implement
    def calc_position(self, stepper_positions):
        a = stepper_positions[self.rails[0].get_name()]
        b = stepper_positions[self.rails[1].get_name()]
        z_pos = stepper_positions[self.rails[2].get_name()]
        return [0, 280, 
                z_pos]
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
        if 2 in homing_axes:
            self.limit_z = self.rails[1].get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limit_z = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):        
        # TODO: implement homing properly
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        if axis == 0:
            homepos[1] = 0.
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= hi.position_endstop - position_min
        else:
            forcepos[axis] += position_max - hi.position_endstop
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def home(self, homing_state):
        pass
        # Always home XY together
        # TODO: homing
        # homing_axes = homing_state.get_axes()
        # home_xy = 0 in homing_axes or 1 in homing_axes
        # home_z = 2 in homing_axes
        # updated_axes = []
        # if home_xy:
        #     updated_axes = [0, 1]
        # if home_z:
        #     updated_axes.append(2)
        # homing_state.set_axes(updated_axes)
        # # Do actual homing
        # if home_xy:
        #     self._home_axis(homing_state, 0, self.rails[0])
        # if home_z:
        #     self._home_axis(homing_state, 2, self.rails[2])
    def _motor_off(self, print_time):
        self.limit_z = (1.0, -1.0)
        #self.limit_xy2 = -1.
    def check_move(self, move):        
        xpos, ypos = move.end_pos[:2]
        # TODO: check kinematic limits for XY
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        # TODO: check Z endstops
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        #xy_home = "xy" if self.limit_xy2 >= 0. else ""
        # TODO: homing
        xy_home = "xy"
        z_home = "z" if self.limit_z[0] <= self.limit_z[1] else ""
        return {
            'homed_axes': xy_home + z_home,
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return PythagorasKinematics(toolhead, config)
