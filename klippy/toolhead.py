import math, logging, importlib
import mcu, homing, chelper, kinematics

class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed, motion_contr=False):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.cmove = toolhead.cmove
        self.is_kinematic_move = True
        self.axes_d = [end_pos[i] - start_pos[i] for i in range(3)]
        self.move_d = math.sqrt(sum([d * d for d in self.axes_d]))
        if self.move_d < .000000001:
            self.is_kinematic_move = False
            self.move_d = abs(end_pos[3] - start_pos[3])
        self.min_move_t = self.move_d / speed
        self.max_start_v2 = 0.
        self.max_cruise_v2 = speed ** 2
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.set_junction = self._set_junction
        if motion_contr:
            self.motion_contr = motion_contr
            self.set_junction = self._set_junction_motion_contr

    def limit_speed(self, speed, accel):
        speed2 = speed ** 2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel

    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        axes_d = self.axes_d
        prev_axes_d = prev_move.axes_d
        junction_cos_theta = -((axes_d[0] * prev_axes_d[0] +
                                axes_d[1] * prev_axes_d[1] +
                                axes_d[2] * prev_axes_d[2]) /
                               (self.move_d * prev_move.move_d))
        junction_cos_theta = max(junction_cos_theta, -0.999999)
        sin_theta_d2 = math.sqrt(0.5 * (1.0 - junction_cos_theta))
        R = self.toolhead.junction_deviation * sin_theta_d2 / (1. - sin_theta_d2)
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5 * (1.0 + junction_cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
        prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                    * prev_move.accel)
        self.max_start_v2 = min(
            R * self.accel, R * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)

    def _set_junction(self, start_v2, cruise_v2, end_v2):
        inv_delta_v2 = 1. / self.delta_v2
        self.accel_r = (cruise_v2 - start_v2) * inv_delta_v2
        self.decel_r = (cruise_v2 - end_v2) * inv_delta_v2
        self.cruise_r = 1. - self.accel_r - self.decel_r
        self.start_v = math.sqrt(start_v2)
        self.cruise_v = math.sqrt(cruise_v2)
        self.end_v = math.sqrt(end_v2)
        self.accel_t = self.accel_r * self.move_d / ((self.start_v + self.cruise_v) * 0.5)
        self.cruise_t = self.cruise_r * self.move_d / self.cruise_v
        self.decel_t = self.decel_r * self.move_d / ((self.end_v + self.cruise_v) * 0.5)

    def _set_junction_motion_contr(self, start_v2, cruise_v2, end_v2):
        self.start_v = math.sqrt(start_v2)
        self.cruise_v = math.sqrt(cruise_v2)
        self.end_v = math.sqrt(end_v2)
        accel_t, cruise_t, decel_t = self.motion_contr.set_junction(
            self.start_v, self.cruise_v, self.end_v, self.accel, self.move_d)
        self.accel_t = accel_t
        self.cruise_t = cruise_t
        self.decel_t = decel_t

    def move(self):
        next_move_time = self.toolhead.get_next_move_time()
        if self.is_kinematic_move:
            self.toolhead.move_fill(self.cmove, next_move_time,
                                    self.accel_t, self.cruise_t, self.decel_t,
                                    self.start_pos[0], self.start_pos[1], self.start_pos[2],
                                    self.axes_d[0], self.axes_d[1], self.axes_d[2],
                                    self.start_v, self.cruise_v, self.accel)
            self.toolhead.kin.move(next_move_time, self)
        self.toolhead.update_move_time(self.accel_t + self.cruise_t + self.decel_t)

class ToolHead:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.move_queue = MoveQueue()
        self.commanded_pos = [0., 0., 0., 0.]
        self.max_velocity = config.getfloat('max_velocity', above=0.)
        self.max_accel = config.getfloat('max_accel', above=0.)
        self.junction_deviation = 0.
        self._calc_junction_deviation()
        self.print_time = 0.
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        tmc5160 = self.printer.lookup_object("tmc5160", False)
        if tmc5160:
            self.motion_contr = tmc5160
            tmc5160.validate(self.max_accel)
        else:
            self.motion_contr = False
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(ffi_lib.move_alloc(), ffi_lib.free)
        self.move_fill = ffi_lib.move_fill
        kin_name = config.get('kinematics')
        mod = importlib.import_module('kinematics.' + kin_name)
        self.kin = mod.load_kinematics(self, config, self.motion_contr)

    def _calc_junction_deviation(self):
        scv2 = self.max_velocity ** 2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel

    def move(self, newpos, speed):
        speed = min(speed, self.max_velocity)
        move = Move(self, self.commanded_pos, newpos, speed, self.motion_contr)
        if not move.move_d:
            return
        self.kin.check_move(move)
        self.commanded_pos[:] = move.end_pos
        self.move_queue.add_move(move)
