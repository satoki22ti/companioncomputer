import math
import time
from logging import Logger
from typing import Optional, Tuple
from .state_machine import add_state_machine_to_model, State
from .event_generator import EventGenerator
from .config import (
        WinchingConfiguration as Config,
        EventConfiguration as EventConfig,
        TelemetryLoggingConfiguration as LoggingConfig
        )
from .telemetry_logger import TelemetryLogger
from winch_control.motor_driver.velocity_driver import VelocityDriver
from winch_control.motor_driver.types import State as MotorState, Telemetry
from winch_control.calculators.acceleration_calculator import AccelerationCalculator
from winch_control.commander.commander import Commander, DropBlockReason
from winch_control.calculators.sum import MovingSum


RASPBERRY_LIMIT_SWITCH_PIN = 13  # 13 (board pin) is GPIO 27
RASPBERRY_LIGHT_PIN = 11         # 11 (board pin) is GPIO 17
STATUS_MESSAGE_INTERVAL = 1      # s
FORCE_AVERAGE_NUMBER_OF_SAMPLES = 5


class Control:
    """
    Control model for winch state machine.
    The model is responsible for implementing all actions for the various state machine transitions,
    as well as evaluating the guards and providing a few auxiliary methods.
    """

    try:
        # Make GPIO (for limit switch) optional, so we can run on other hardware than a Raspberry Pi
        from RPi import GPIO  # type: ignore
    except Exception:
        GPIO = None

    def __init__(self, config: Config, event_config: EventConfig, logging_config: Optional[LoggingConfig],
                 driver: VelocityDriver, commander: Commander, logger: Logger) -> None:
        self.config = config
        self.driver = driver
        self.commander = commander
        self.logger = logger
        self.machine = add_state_machine_to_model(self)
        self.event_generator = EventGenerator(event_config, velocity_driver=self.driver, commander=commander,
                                              listener=self, logger=logger)
        self.acceleration_calculator = AccelerationCalculator(a=driver.get_acceleration())

        self.telemetry_logger: Optional[TelemetryLogger] = None
        if logging_config is not None:
            self.telemetry_logger = TelemetryLogger(logging_config)

        # For sending status update to commander
        self.time_last_status_sent = 0.0
        self.force_moving_sum = MovingSum(FORCE_AVERAGE_NUMBER_OF_SAMPLES)
        self.ASR_force = MovingSum(30)
        self.last_position_sent_as_message = 0.0

        # All state/data not covered by state machine.
        # Should be kept as minimal as possible.
        self.home_position = math.nan
        self.drop_length = self.config.drop.length  # May be modified by distance sensor, if enabled

        # Setup GPIO, if available
        if self.GPIO is not None:
            try:
                self.GPIO.setmode(self.GPIO.BOARD)
                self.GPIO.setup(RASPBERRY_LIMIT_SWITCH_PIN, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
                self.GPIO.setup(RASPBERRY_LIGHT_PIN, self.GPIO.OUT)
            except Exception:
                self.GPIO = None
        if self.GPIO is None:
            self.logger.warning('CONTROL: Limit switch (home detecting) is not available')
            self.logger.warning('CONTROL: Light output pin not available')

    def start(self) -> None:
        self.event_generator.start()

    def exit(self) -> None:
        self.event_generator.exit()

    def join(self) -> None:
        self.event_generator.join()

    # Implement AbstractModel from state_machine.py
    def log_state_change(self) -> None:
        # .state attribute will be added by state machine
        self.logger.info(f'CONTROL: Enter {self.state}')  # type: ignore[attr-defined]

    def guard_no_fault(self) -> bool:
        allowed = self.driver.telemetry().state is MotorState.RUNNING
        self.logger.debug(f'CONTROL: guard_no_fault() -> {allowed}')
        return allowed

    def guard_know_home(self) -> bool:
        telemetry = self.driver.telemetry()
        allowed = (telemetry.state == MotorState.RUNNING
                   and math.isfinite(telemetry.position)
                   and math.isfinite(self.home_position))
        self.logger.debug(f'CONTROL: guard_know_home() -> {allowed}')
        return allowed

    def guard_drop_allowed(self) -> bool:
        allowed = self.commander.drop_allowed()
        self.logger.debug(f'CONTROL: guard_drop_allowed() -> {allowed}')
        return allowed

    def _check_if_distance_sensor_allows_drop(self) -> Tuple[float, float, DropBlockReason]:
        """
        Returns a tuple with the modified drop length (set to nan if drop is not allowed),
        the calculated drop length that was blocked (if applicable), and
        the block reason.
        """
        default_drop_length = self.config.drop.length
        if self.config.drop.height_modifier is None:
            return default_drop_length, math.nan, DropBlockReason.NONE
        drone_height_above_terrain = self.commander.height_above_terrain()
        if not math.isfinite(drone_height_above_terrain):
            if self.config.drop.height_modifier.strict:
                return math.nan, math.nan, DropBlockReason.DISTANCE_ERROR
            else:
                return default_drop_length, math.nan, DropBlockReason.DISTANCE_ERROR
        drop_length = drone_height_above_terrain - self.config.drop.height_modifier.distance_to_ground
        if abs(drop_length - self.config.drop.length) > self.config.drop.height_modifier.max_modification:
            drop_block = (DropBlockReason.DISTANCE_HIGH if drop_length - self.config.drop.length > 0
                          else DropBlockReason.DISTANCE_LOW)
            if self.config.drop.height_modifier.strict:
                return math.nan, drop_length, drop_block
            else:
                return default_drop_length, drop_length, drop_block
        return drop_length, math.nan, DropBlockReason.NONE

    def _check_if_requested_drop_length_is_beyond_current_drop(self, drop_length: float) -> bool:
        telemetry = self.driver.telemetry()
        if not math.isfinite(telemetry.position):
            self.logger.error('CONTROL: _check_if_requested_drop_length_is_beyond_current_drop with invalid telemetry')
            return False

        remaining_distance = self.home_position + drop_length - telemetry.position
        return remaining_distance >= 0.0

    def guard_distance_sensor(self) -> bool:
        drop_length, blocked_drop_length, drop_block = self._check_if_distance_sensor_allows_drop()

        if drop_block is DropBlockReason.DISTANCE_ERROR:
            if math.isfinite(drop_length):
                self.logger.warning('CONTROL: No height measurement, use default drop')
                self.commander.send_message('Winch: No height measurement, use default drop')
            else:
                self.logger.warning('CONTROL: Drop rejected because of missing height measurement')
                self.commander.send_alert('Winch: Drop rejected: missing height measurement')
        elif drop_block in (DropBlockReason.DISTANCE_LOW, DropBlockReason.DISTANCE_HIGH):
            if math.isfinite(drop_length):
                self.logger.warning(f'CONTROL: Modified drop ({blocked_drop_length:.1f} m) is out if range, '
                                    'using default length')
                self.commander.send_message(f'Winch: Modified drop ({blocked_drop_length:.1f} m) is out if range')
            else:
                self.logger.warning(f'CONTROL: Modified drop ({blocked_drop_length:.1f} m) '
                                    'is out if range, drop rejected')
                self.commander.send_alert(f'Winch: Modified drop ({blocked_drop_length:.1f} m) is out if range')

        if math.isfinite(drop_length):
            if self._check_if_requested_drop_length_is_beyond_current_drop(drop_length):
                self.drop_length = drop_length
                return True
            else:
                self.logger.info(f'CONTROL: Modified drop ({drop_length:.1f} m) is above current position')
        return False

    def guard_allow_drop_while_retracting(self) -> bool:
        return self.config.drop.allow_while_retracting

    def action_clear_wait_for(self) -> None:
        self.logger.debug('CONTROL: action_clear_wait_for')
        self.event_generator.set_timeout(timeout=math.nan)  # Clears the general timeout
        self.event_generator.clear_wait_for()

    def action_home(self) -> None:
        self.logger.debug('CONTROL: action_home')
        self.commander.send_message('Winch: Homing started')
        self.event_generator.set_timeout(math.nan)  # Clear timeout
        self.driver.setpoint(velocity=-self.config.homing.velocity, force=self.config.homing.force)

        telemetry = self.driver.telemetry()
        if (telemetry.state != MotorState.RUNNING
                or not math.isfinite(telemetry.position)
                or not math.isfinite(self.home_position)):
            # Unknown position, calculate max timeout from wire length, with configurable margins
            timeout = (self.config.wire_length / self.config.homing.velocity
                       * self.config.homing.timeout_modifier.multiplier
                       + self.config.homing.timeout_modifier.offset)
        else:
            # We can calculate how long homing should take, and set timeout, with configurable margins
            # We can ignore acceleration time for homing speed
            timeout = (abs(self.home_position - telemetry.position) / self.config.homing.velocity
                       * self.config.homing.timeout_modifier.multiplier
                       + self.config.homing.timeout_modifier.offset)
        self.logger.info(f'CONTROL: Homing until force is {self.config.homing.force_sense}, '
                         f'setting timeout to {timeout:.2f} seconds')
        self.event_generator.wait_for_force(force=self.config.homing.force_sense, timeout=timeout)

    def action_lock(self) -> None:
        self.logger.debug('CONTROL: action_lock')
        self.commander.send_message('Winch: Locking started')
        self.event_generator.set_timeout(timeout=self.config.homing.lock_time)

    def action_hold_home(self) -> None:
        self.logger.debug('CONTROL: action_home')
        if self.GPIO is not None:
            # Check limit switch (only on raspberry)
            # If the signal is high (pulled up) the limit switch (normally open)
            # is not active (not pulling signal to ground), so the
            # hook is not homed.
            if self.GPIO.input(RASPBERRY_LIMIT_SWITCH_PIN):
                # Switch is closed when not homed, and pulls input down
                self.commander.send_message('Winch: Home switch is not activated')
            else:
                self.commander.send_message('Winch: Homing complete')
        else:
            self.commander.send_message('Winch: Homing complete (without confirmation)')

        telemetry = self.driver.telemetry()
        if telemetry.state == MotorState.RUNNING and math.isfinite(telemetry.position):
            self.home_position = telemetry.position
        else:
            self.home_position = math.nan

        self.driver.setpoint(velocity=-self.config.home.velocity, force=self.config.home.force)

    def action_rehome(self) -> None:
        self.logger.debug('CONTROL: action_rehome')
        self.commander.send_message('Winch: Loading started')
        self.driver.setpoint(self.config.load.velocity, force=self.config.load.force)
        if not math.isfinite(self.home_position):
            # We do not have a valid home position (should not happen, but handle to be safe)
            # Use a short drop, then go to timeout state
            self.logger.error('CONTROL: action_rehome with unknown home position')
            self.event_generator.set_timeout(abs(self.config.load.length / self.config.load.velocity))
        else:
            # Wait for desired position, with timeout
            timeout = (abs(self.config.load.length / self.config.load.velocity)
                       * self.config.load.timeout_modifier.multiplier
                       + self.config.load.timeout_modifier.offset)
            self.event_generator.wait_for_position(position=(self.home_position + self.config.load.length),
                                                   timeout=timeout)

    def action_load(self) -> None:
        self.logger.debug('CONTROL: action_load')
        self.driver.setpoint(velocity=0.0, force=self.config.load.force)
        self.event_generator.set_timeout(timeout=self.config.load.time)

    def action_stop(self) -> None:
        self.logger.debug('CONTROL: action_stop')
        self.commander.send_alert('Winch: Motor stopped')
        self.home_position = math.nan
        self.driver.stop()

    def action_prepare_drop(self) -> None:
        self.logger.debug('CONTROL: action__prepare_drop')
        self.commander.send_message(f'Winch: Dropping started, will drop {self.drop_length:.1f} m')

        # Calculate expected time for drop and set timeout with margins
        # Ignore acceleration for slow length
        time_slow = self.config.drop.length_slow / self.config.drop.velocity_slow
        profile = self.acceleration_calculator.acceleration_profile(
                v0=0.0, vmax=self.config.drop.velocity, d=(self.drop_length - self.config.drop.length_slow))
        if profile is None:
            # Should not happen with valid config, but handle to be safe
            self.logger.error('CONTROL: action_prepare_drop could not get acceleration profile')
            self.event_generator.set_timeout(timeout=0.0)  # Immediately transition to timeout
            return

        expected_time = profile.time_total + time_slow
        timeout = (expected_time * self.config.drop.timeout_modifier.multiplier
                   + self.config.drop.timeout_modifier.offset)
        self.event_generator.set_timeout(timeout=timeout)
        self.logger.info(f'CONTROL: Starting drop ({self.drop_length:.1f} m) with expected duration '
                         f'{expected_time:.1f} seconds, setting timeout to {timeout:.1f} seconds')

    def action_drop(self) -> None:
        self.logger.debug('CONTROL: action_drop')
        telemetry = self.driver.telemetry()
        self.ASR_force.update(telemetry.force) # this is for ASR
        if (telemetry.state != MotorState.RUNNING
                or not math.isfinite(telemetry.position)
                or not math.isfinite(telemetry.velocity)
                or not math.isfinite(self.home_position)):
            # Little chance of this happening, but handle it to be safe
            # Most likely we will handle the problem in transition to fault/stopped/fatal
            self.logger.error('CONTROL: action_drop with invalid telemetry')
            self.event_generator.set_timeout(0.0)  # Immediately transition to timeout
            return

        remaining_distance = self.home_position + self.drop_length - telemetry.position
        if remaining_distance < 0.0:
            # Little chance of this happening, but handle it to be safe
            # Most likely we will handle this in transition to drop_slowing
            self.logger.error('CONTROL: action_drop with no remaining distance')
            self.event_generator.set_timeout(0.0)  # Immediately transition to timeout
            return

        profile = self.acceleration_calculator.acceleration_profile(
                v0=telemetry.velocity, vmax=self.config.drop.velocity, d=remaining_distance)
        if profile is None:
            # This should never happen, but handle it to be safe
            self.logger.error('CONTROL: action_drop could not get acceleration profile')
            self.driver.setpoint(velocity=0.0, force=self.config.drop.force_brake)
            self.event_generator.set_timeout(0.0)  # Immediately transition to timeout
            return

        if telemetry.position - self.home_position < self.config.drop.length_slow:
            # Within slow zone, use velocity slow instead of calculated values
            self.driver.setpoint(velocity=self.config.drop.velocity_slow, force=self.config.drop.force)
        else:
            # Update with new calculated setpoint
            self.driver.setpoint(velocity=profile.velocity_max, force=self.config.drop.force)

        # TODO Make timeout margins configurable? Might not be needed for this...
        timeout = profile.time_start_decelerate + 0.25  # Do not need a big margin, as we will recalculate often
        position = telemetry.position + profile.distance_start_decelerate
        self.event_generator.wait_for_position(position=position, timeout=timeout)

    def action_drop_slowing(self) -> None:
        self.logger.debug('CONTROL: action_drop_slowing')
        telemetry = self.driver.telemetry()
        self.ASR_force.update(telemetry.force) # this is for ASR
        if (telemetry.state != MotorState.RUNNING
                or not math.isfinite(telemetry.position)
                or not math.isfinite(telemetry.velocity)
                or not math.isfinite(self.home_position)):
            # Little chance of this happening, but handle it to be safe
            # Most likely we will handle the problem in transition to fault/stopped/fatal
            self.logger.error('CONTROL: action_drop_slowing with invalid telemetry')
            self.event_generator.set_timeout(0.0)  # Immediately transition to timeout
            return

        # Set a slow speed, so we can move towards target if we decelerate too quick
        self.driver.setpoint(velocity=self.config.drop.velocity_slow, force=self.config.drop.force_brake)

        # Calculate how long it should take to reach target to use for timeout
        # To simplify, use target speed of 0.0 in calculation
        self.acceleration_calculator.calculate(v0=telemetry.velocity, v=0.0)
        timeout = (self.acceleration_calculator.t * self.config.drop.timeout_modifier.multiplier
                   + self.config.drop.timeout_modifier.offset)
        self.logger.info(f'CONTROL: Slowing down drop and continue to position is '
                         f'{self.home_position + self.drop_length:.2f}, with timeout {timeout:.2f}, '
                         f'current position is {telemetry.position:.2f}')
        self.event_generator.wait_for_position(self.home_position + self.drop_length, timeout)

    def action_prepare_retract(self) -> None:
        self.logger.debug('CONTROL: action_prepare_retract')
        self.commander.send_message('Winch: Retract started')
        

        telemetry = self.driver.telemetry()
        self.ASR_force.update(telemetry.force) # this is for ASR
        if (telemetry.state != MotorState.RUNNING
                or not math.isfinite(telemetry.position)
                or not math.isfinite(telemetry.velocity)
                or not math.isfinite(self.home_position)):
            # Little chance of this happening, but handle it to be safe
            # Most likely we will handle the problem in transition to fault/stopped/fatal
            self.logger.error('CONTROL: action_prepare_retract with invalid telemetry')
            self.event_generator.set_timeout(0.0)  # Immediately transition to timeout
            return

        # Note: retraction may start at a velocity other than zero
        # We change sign of current velocity to match calculations
        initial_velocity = -telemetry.velocity

        retraction_length = telemetry.position - self.home_position - self.config.retract.homing_distance
        if retraction_length < 0.0:
            # Can happen that we start within homing distance.
            # We might still go outside homing distance again, if current velocity is high enough.
            # We just use zero retraction_length in these timeout calculations
            retraction_length = 0.0

        # Calculate expected time for retraction and set timeout with margins
        profile = self.acceleration_calculator.acceleration_profile(
                v0=initial_velocity, vmax=self.config.retract.velocity, d=retraction_length)
        if profile is None:
            # Should not happen with valid config, but handle to be safe
            self.logger.error('CONTROL: action_prepare_retract could not get acceleration profile')
            self.event_generator.set_timeout(timeout=0.0)  # Immediately transition to timeout
            return

        expected_time = profile.time_total
        timeout = (expected_time * self.config.retract.timeout_modifier.multiplier
                   + self.config.retract.timeout_modifier.offset)
        self.event_generator.set_timeout(timeout=timeout)
        self.logger.info(f'CONTROL: Starting retraction with expected duration {expected_time:.1f} seconds, '
                         f'setting timeout to {timeout:.1f} seconds')

    def action_retract(self) -> None:
        self.logger.debug('CONTROL: action_retract')
        telemetry = self.driver.telemetry()
        self.ASR_force.update(telemetry.force) # this is for ASR
        if (telemetry.state != MotorState.RUNNING
                or not math.isfinite(telemetry.position)
                or not math.isfinite(telemetry.velocity)
                or not math.isfinite(self.home_position)):
            # Little chance of this happening, but handle it to be safe
            # Most likely we will handle the problem in transition to fault/stopped/fatal
            self.logger.error('CONTROL: action_retract with invalid telemetry')
            self.event_generator.set_timeout(0.0)  # Immediately transition to timeout
            return

        remaining_distance = telemetry.position - (self.home_position + self.config.retract.homing_distance)
        if remaining_distance < 0.0:
            # We are inside homing distance, but we might still go outside if we have a drop velocity.
            # We can just set homing speed and wait for a position closer to home than current position.
            # If we leave homing distance, we will recalculate in later iterations.
            # If we do not leave homing distance, we will eventually reverse direction,
            # reach out wait_for position, and then transition to next state.
            self.driver.setpoint(velocity=-self.config.homing.velocity, force=self.config.drop.force_brake)
            # Just need a small offset to wait_for_position so we are sure to reverse direction before we trigger
            # transition to next state.
            # The offset should not be past home however, so if we are very close to home, just transition at once.
            if telemetry.position - self.home_position < 0.1:
                self.event_generator.wait_for_position(position=telemetry.position + 0.15, timeout=1.0)
            else:
                self.event_generator.wait_for_position(position=telemetry.position - 0.05, timeout=1.0)
            return

        # Need to use negative telemetry.velocity as v0, to get a positive velocity when retracting.
        # Note that self.config.retract.velocity is already positive
        profile = self.acceleration_calculator.acceleration_profile(
                v0=-telemetry.velocity, vmax=self.config.retract.velocity, d=remaining_distance)
        if profile is None:
            # This should never happen, but handle it to be safe
            # Note, we use brake force from drop here, not force from retract
            self.logger.error('CONTROL: action_retract could not get acceleration profile')
            self.driver.setpoint(velocity=0.0, force=self.config.drop.force_brake)
            return

        # Update with new calculated setpoint
        # Set homing velocity as a minimum velocity,
        # this is to prevent that winch to stall with heavy package and low acceleration set

        velocity = -max(profile.velocity_max, self.config.homing.velocity)
        if math.abs(telemetry.force) < math.abs(self.ASR_force.avg()):
            del_vel  = 0.1

        else:
            del_vel = 0.4

        velocity = velocity + del_vel
        self.driver.setpoint(velocity=velocity, force=self.config.retract.force)

        # TODO Make timeout margins configurable? Might not be needed for this...
        timeout = profile.time_start_decelerate + 0.25  # Do not need a big margin, as we will recalculate often
        if profile.distance_start_decelerate <= 0.0:
            # A negative distance_start_decelerate means we still have a drop velocity that we shall
            # decelerate the current drop velocity first, then accelerate to retraction speed, then
            # we need to start deceleration of the retraction speed at a point that is further out
            # than current position. This is no problem, but it has the implication that the position
            # to wait for is already considered reached, and would immediately trigger a transition to the next state.
            # So in this special case, we substitute the position to wait for with a
            # position a bit further in than the current positions.
            # The real position to wait for will be recalculated in later iterations,
            # and after we have moved past it, we can use it to wait for.
            position = telemetry.position - 0.1
        else:
            position = telemetry.position - profile.distance_start_decelerate
        self.event_generator.wait_for_position(position=position, timeout=timeout)

    def action_retract_slowing(self) -> None:
        self.logger.debug('CONTROL: action_retract_slowing')
        telemetry = self.driver.telemetry()
        self.ASR_force.update(telemetry.force) # this is for ASR
        if (telemetry.state != MotorState.RUNNING
                or not math.isfinite(telemetry.position)
                or not math.isfinite(telemetry.velocity)
                or not math.isfinite(self.home_position)):
            # Little chance of this happening, but handle it to be safe
            # Most likely we will handle the problem in transition to fault/stopped/fatal
            self.logger.error('CONTROL: action_retract_slowing with invalid telemetry')
            self.event_generator.set_timeout(0.0)  # Immediately transition to timeout
            return


        # Set a homing speed, so we can move towards home
        if math.abs(telemetry.force) < math.abs(self.ASR_force.avg()):
            del_vel  = 0.1

        else:
            del_vel = 0.4

        velocity = velocity + del_vel
        self.driver.setpoint(velocity=velocity, force=self.config.retract.force)
        #self.driver.setpoint(velocity=self.config.homing.velocity, force=self.config.homing.force) # velocity was self.config.homing.velocity

        # Calculate how long it should take to reach target to use for timeout
        # To simplify, use target speed of 0.0 in calculation
        # Note that we need to reverse sign on v0 to match calculations
        self.acceleration_calculator.calculate(v0=-telemetry.velocity, v=0.0)
        timeout = (self.acceleration_calculator.t * self.config.retract.timeout_modifier.multiplier
                   + self.config.retract.timeout_modifier.offset)
        self.logger.info(f'CONTROL: Slowing down retraction and continue to position is '
                         f'{self.home_position + self.config.retract.homing_distance:.2f}, with timeout {timeout:.2f}, '
                         f'current position is {telemetry.position:.2f}')
        self.event_generator.wait_for_position(position=(self.home_position + self.config.retract.homing_distance),
                                               timeout=timeout)

    def action_hold_drop(self) -> None:
        self.logger.debug('CONTROL: action_hold_drop')
        # Send status message to commander based on what state we just transitioned into
        if self.state is State.DROPPED:  # type: ignore[attr-defined]
            self.commander.send_message('Winch: Dropping complete')
        if self.state is State.TIMEOUT:  # type: ignore[attr-defined]
            self.commander.send_alert('Winch: Operation timed out')
        self.event_generator.clear_wait_for()
        self.event_generator.set_timeout(timeout=math.nan)  # Clear timeout
        telemetry = self.driver.telemetry()
        self.logger.info(f'CONTROL: Setting hold drop at position {telemetry.position:.2f}')
        self.driver.setpoint(velocity=0.0, force=self.config.drop.force_hold)

        # Arm tug detection, if configured, and in DROPPED state
        if (self.state is State.DROPPED  # type: ignore[attr-defined]
                and math.isfinite(self.config.drop.force_tug_detect)):
            self.event_generator.wait_for_force(force=self.config.drop.force_tug_detect, timeout=math.nan)

    def action_freewheel(self) -> None:
        self.logger.debug('CONTROL: action_freewheel')
        self.commander.send_alert('Winch: Motor in emergency mode')
        self.driver.freewheel()

    def action_reset_and_home(self) -> None:
        self.logger.debug('CONTROL: action_reset_and_home')
        self.commander.send_message('Winch: Resetting...')
        self.driver.reset()
        time.sleep(0.2)  # TODO: We need to wait for reset to be done before start homing, add state or other
        self.action_home()

    def action_fault(self) -> None:
        self.logger.debug('CONTROL: action_fault')
        self.logger.warning('CONTROL: Fault detected')
        self.commander.send_alert('Winch: Motor fault detected')

    def action_fatal(self) -> None:
        self.logger.debug('CONTROL: action_fatal')
        self.logger.critical('CONTROL: Entered fatal state, will exit program...')
        self.commander.send_alert('Winch: Fatal, control will exit')
        self.commander.exit()
        self.event_generator.exit()
        self.driver.exit()

    def action_complete_command(self) -> None:
        self.commander.complete_command()

    def action_fail_command(self) -> None:
        self.commander.fail_command()

    # Implement telemetry from abstract Listener in event_generator.py
    def telemetry(self, telemetry: Telemetry) -> None:
        if self.telemetry_logger is not None:
            self.telemetry_logger.log(self.state, telemetry)  # type: ignore[attr-defined]

        self.force_moving_sum.update(telemetry.force)
        if self.time_last_status_sent + STATUS_MESSAGE_INTERVAL < time.monotonic():
            self.time_last_status_sent = time.monotonic()
            if math.isfinite(telemetry.position) and math.isfinite(self.home_position):
                length = telemetry.position - self.home_position
            else:
                length = math.nan

            # Evaluate if anything is currently blocking a drop
            drop_block = DropBlockReason.NONE
            if not (self.state is State.HOME  # type: ignore[attr-defined]
                    or (self.state in [State.RETRACTING, State.RETRACTING_SLOWING]  # type: ignore[attr-defined]
                        and self.config.drop.allow_while_retracting)):
                drop_block = DropBlockReason.NOT_HOMED
            elif not self.commander.drop_allowed():
                drop_block = DropBlockReason.VTOL_MODE
            else:
                drop_length, _, tmp = self._check_if_distance_sensor_allows_drop()
                if not math.isfinite(drop_length):
                    drop_block = tmp

            self.commander.send_status(length=length,
                                       velocity=telemetry.velocity,
                                       force=self.force_moving_sum.avg(),
                                       voltage=math.nan if telemetry.voltage is None else telemetry.voltage,
                                       current=math.nan if telemetry.current is None else telemetry.current,
                                       temperature=math.nan if telemetry.temperature is None else telemetry.temperature,
                                       state=self.state,  # type: ignore[attr-defined]
                                       home_switch=(self.GPIO is None
                                                    or not self.GPIO.input(RASPBERRY_LIMIT_SWITCH_PIN)),
                                       drop_block=drop_block)

            # TODO Remove this feature when QGC has an updated interface.
            # If in emergency (freewheel) state, we will in addition to the winch status message send a log entry
            # with the current length of the wire.
            # This is a temporary feature to allow the pilot to quickly assess if all the wire has
            # left the drone, and it is safe to transition to FW.
            # In the future, this should be replaced with a better QGC winch interface.
            if (self.state is State.FREEWHEEL  # type: ignore[attr-defined]
                    and math.isfinite(length) and abs(length - self.last_position_sent_as_message) > 0.5):
                self.last_position_sent_as_message = length
                # Only send if length have changed at least 0.5 meters
                # (since this is a temporary feature, we will hard code this limit)
                self.commander.send_message(f'Winch length: {length:.1f} m')

    def drone_state_update(self, mr: bool, landed: bool) -> None:
        light_on = mr and not landed
        if self.GPIO is not None:
            self.GPIO.output(RASPBERRY_LIGHT_PIN, light_on)
            self.logger.info('CONTROL: Setting light to ' + ('on' if light_on else 'off')
                             + ' because drone is in ' + ('MR' if mr else 'FW')
                             + ' and ' + ('landed' if landed else 'in the air'))
        else:
            self.logger.info('CONTROL: No GPIO availible, could not set light to ' + ('on' if light_on else 'off')
                             + ' with drone in ' + ('MR' if mr else 'FW')
                             + ' and ' + ('landed' if landed else 'in the air'))

    # Implement rest of Listener from event_generator.py
    # These should all be overridden by the state machine decorator,
    # but we will implement defaults here so that we can
    # 1) Pass type checking
    # 2) Not crash if we alter the state machine and forget to handle an event

    def event_ready(self) -> None:
        pass

    def event_fault(self) -> None:
        pass

    def event_stopped(self) -> None:
        pass

    def event_fatal(self) -> None:
        pass

    def event_exit(self) -> None:
        pass

    def event_position(self) -> None:
        pass

    def event_velocity(self) -> None:
        pass

    def event_force(self) -> None:
        pass

    def event_timeout(self) -> None:
        pass

    def event_retract(self) -> None:
        pass

    def may_event_retract(self) -> bool:
        return False

    def event_drop(self) -> None:
        pass

    def may_event_drop(self) -> bool:
        return False

    def event_emergency(self) -> None:
        pass

    def event_tick(self) -> None:
        pass
