# Interface to Klipper micro-controller code with TMC5160 support
#
# Copyright (C) 2016-2023 Kevin O'Connor
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import sys, os, zlib, logging, math
import serialhdl, pins, chelper, clocksync

class error(Exception):
    pass

class TMC5160:
    REGISTERS = {
        "GCONF": 0x00,
        "IHOLD_IRUN": 0x10,
        "TPOWERDOWN": 0x11,
        "TPWMTHRS": 0x13,
        "TCOOLTHRS": 0x14,
        "THIGH": 0x15,
        "CHOPCONF": 0x6C,
        "COOLCONF": 0x6D,
        "DCCTRL": 0x6E,
        "DRVSTATUS": 0x6F,
        "MSCNT": 0x6A,
        "MSCURACT": 0x6B
    }

    def __init__(self, mcu, spi, config):
        self._mcu = mcu
        self.spi = spi
        self.config = config

        self.microsteps = config.getint("microsteps", default=256)
        self.current_scale = config.getint("current_scale", default=16)
        self.tcoolthrs = config.getint("tcoolthrs", default=0)
        self.thigh = config.getint("thigh", default=0)
        self.rms_current = config.getint("rms_current", default=800)
        self.sense_resistor = config.getfloat("sense_resistor", default=0.11)
        self.spi_speed = config.getint("spi_speed", default=4000000)

        # SPI initialization
        self.spi_init()

        # Initialize TMC5160
        self.init_tmc5160()

    def spi_init(self):
        self.spi.configure(self.spi_speed)

    def set_register(self, reg_name, value):
        reg = self.REGISTERS[reg_name]
        datagram = [(reg >> 16) & 0xFF, (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF]
        self.spi.spi_send(datagram)

    def get_register(self, reg_name):
        reg = self.REGISTERS[reg_name]
        response = self.spi.spi_send_receive([reg, 0x00, 0x00, 0x00, 0x00])
        return (response[1] << 24) | (response[2] << 16) | (response[3] << 8) | response[4]

    def init_tmc5160(self):
        # Basic configuration: microstepping, currents, and thresholds
        self.set_register("GCONF", 0x00000000)  # Standard config
        self.set_register("IHOLD_IRUN", (self.current_scale << 8) | self.rms_current)
        self.set_register("TPOWERDOWN", 0x0000000A)  # Standby time
        self.set_register("TPWMTHRS", self.tcoolthrs)
        self.set_register("TCOOLTHRS", self.tcoolthrs)
        self.set_register("THIGH", self.thigh)

    def set_sixpoint_ramp(self, vstart, v1, v2, amax, dmax):
        """Set SixPoint™ ramp parameters in TMC5160."""
        self.set_register("VSTART", vstart)
        self.set_register("V1", v1)
        self.set_register("V2", v2)
        self.set_register("AMAX", amax)
        self.set_register("DMAX", dmax)

    def get_status(self):
        """Retrieve status information from TMC5160."""
        return self.get_register("DRVSTATUS")


class MCU_stepper:
    def __init__(self, mcu, pin_params, spi, config):
        self._mcu = mcu
        self._spi = spi
        self._config = config
        self._step_pin = pin_params['pin']
        self._invert_step = pin_params['invert']
        self._dir_pin = None
        self._invert_dir = False
        self._mcu_position_offset = 0.0
        self._step_dist = 0.0
        self._min_stop_interval = 0.0
        self._tmc5160 = TMC5160(mcu, spi, config)

        # Initialize callbacks and motion planner
        self._mcu.register_config_callback(self._build_config)
        ffi_main, self._ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(self._ffi_lib.stepcompress_alloc(oid), self._ffi_lib.stepcompress_free)
        self._mcu.register_stepqueue(self._stepqueue)

    def setup_dir_pin(self, pin_params):
        if pin_params['chip'] is not self._mcu:
            raise pins.error("Stepper dir pin must be on same mcu as step pin")
        self._dir_pin = pin_params['pin']
        self._invert_dir = pin_params['invert']

    def setup_step_distance(self, step_dist):
        self._step_dist = step_dist
        self._tmc5160.set_sixpoint_ramp(0, 0, 0, step_dist, step_dist)

    def _build_config(self):
        # Configurations for MCU and stepper motor using TMC5160 features
        self._mcu.add_config_cmd(
            "config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d" % (
                self._oid, self._step_pin, self._dir_pin, self._invert_step
            )
        )
        self._mcu.add_config_cmd("reset_step_clock oid=%d clock=0" % (self._oid,), is_init=True)

    def get_mcu_position(self):
        return self._ffi_lib.itersolve_get_commanded_pos(self._stepper_kinematics)

    def move(self, start_velocity, cruise_velocity, end_velocity, accel, move_distance):
        # Define SixPoint™ ramp motion planning for TMC5160
        self._tmc5160.set_sixpoint_ramp(start_velocity, cruise_velocity, end_velocity, accel, move_distance)
        # Generate steps and execute move
        self._ffi_lib.stepcompress_fill(self._stepqueue, self._mcu.seconds_to_clock(0), self._invert_dir, 0, 0)

    def stop(self):
        # Stop the stepper motor
        self._tmc5160.set_register("AMAX", 0)
        self._tmc5160.set_register("DMAX", 0)
        logging.info("Stepper stopped")

# Main MCU interface with stepper motor control
class MCU:
    def __init__(self, config, clocksync):
        self._printer = config.get_printer()
        self._clocksync = clocksync
        self._reactor = self._printer.get_reactor()
        self._name = config.get_name().replace('mcu ', '')
        self._serialport = config.get('serial', '/dev/ttyS0')
        self._spi_bus = bus.MCU_SPI_from_config(config)
        self._tmc5160_stepper = MCU_stepper(self, {'pin': 'step_pin', 'invert': False}, self._spi_bus, config)

    def reset(self):
        """Reset the MCU and all peripherals"""
        self._tmc5160_stepper.stop()
        self._serial.disconnect()
        logging.info(f"MCU '{self._name}' reset")

    def execute_move(self, start_velocity, cruise_velocity, end_velocity, accel, distance):
        """Command the MCU to move with SixPoint™ ramp"""
        self._tmc5160_stepper.move(start_velocity, cruise_velocity, end_velocity, accel, distance)

    def stop(self):
        """Stop all motors connected to the MCU"""
        self._tmc5160_stepper.stop()
