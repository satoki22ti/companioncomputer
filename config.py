from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional
import math

# Hard limits for parameters/configuration
# Events
UPDATE_RATE_MIN = 2.0
UPDATE_RATE_MAX = 10.0
INIT_TIMEOUT_MIN = 0.0
INIT_TIMEOUT_MAX = 60.0
# Winching
WIRE_LENGTH_MIN = 2.0        # m
WIRE_LENGTH_MAX = 100.0      # m
DROP_LENGTH_MIN = 1.0        # m
DROP_LENGTH_MAX = WIRE_LENGTH_MAX
MAX_DROP_MARGIN = 1.0        # m
SLOW_DROP_LENGTH_MIN = 0.0   # m
SLOW_DROP_LENGTH_MAX = 0.5   # m
FAST_VELOCITY_MIN = 0.25     # m/s
FAST_VELOCITY_MAX = 5.00     # m/s
SLOW_VELOCITY_MIN = 0.05     # m/s
SLOW_VELOCITY_MAX = 0.50     # m/s
FORCE_MIN = 0.0              # N
FORCE_MAX = 120.0            # N
FORCE_HOME_MIN = 1.0         # N
FORCE_SENSE_MIN = 1.0        # N
FORCE_TUG_DETECT_MIN = 10.0  # N
LOAD_TIME_MIN = 1.0          # s
LOAD_TIME_MAX = 10.0         # s
LOAD_LENGTH_MIN = 0.05       # m
LOAD_LENGTH_MAX = 1.00       # m
HOMING_DISTANCE_MIN = 0.20   # m
HOMING_DISTANCE_MAX = 1.00   # m
LOCK_TIME_MIN = 0.0          # s
LOCK_TIME_MAX = 10.0         # s
TIMEOUT_MULTIPLIER_MIN = 1.0
TIMEOUT_MULTIPLIER_MAX = 10.0
TIMEOUT_OFFSET_MIN = 0.0
TIMEOUT_OFFSET_MAX = 300.0
# Drop height modifier (based on distance sensor)
DISTANCE_TO_GROUND_MIN = -1.0  # m
DISTANCE_TO_GROUND_MAX = 10.0  # m
MAX_MODIFICATION_MIN = 0.5     # m
MAX_MODIFICATION_MAX = 50.0    # m


@dataclass
class EventConfiguration:
    update_frequency: int
    initialization_timeout: float

    def validate(self, conf_path: str) -> None:
        if self.update_frequency < UPDATE_RATE_MIN or self.update_frequency > UPDATE_RATE_MAX:
            raise ValueError(f'{conf_path}.update_frequency must be in range [{UPDATE_RATE_MIN}, {UPDATE_RATE_MAX}]')
        if self.initialization_timeout < INIT_TIMEOUT_MIN or self.initialization_timeout > INIT_TIMEOUT_MAX:
            raise ValueError(f'{conf_path}.initialization_timeout must be in range '
                             f'[{INIT_TIMEOUT_MIN}, {INIT_TIMEOUT_MAX}]')


@dataclass
class TimeoutModifier:
    multiplier: float
    offset: float

    def validate(self, conf_path: str) -> None:
        if self.multiplier < TIMEOUT_MULTIPLIER_MIN or self.multiplier > TIMEOUT_MULTIPLIER_MAX:
            raise ValueError(f'{conf_path}.multiplier must be in range '
                             f'[{TIMEOUT_MULTIPLIER_MIN}, {TIMEOUT_MULTIPLIER_MAX}]')
        if self.offset < TIMEOUT_OFFSET_MIN or self.offset > TIMEOUT_OFFSET_MAX:
            raise ValueError(f'{conf_path}.offset must be in range [{TIMEOUT_OFFSET_MIN}, {TIMEOUT_OFFSET_MAX}]')


@dataclass
class HeightModifier:
    distance_to_ground: float
    max_modification: float
    strict: bool

    def validate(self, conf_path: str) -> None:
        if self.distance_to_ground < DISTANCE_TO_GROUND_MIN or self.distance_to_ground > DISTANCE_TO_GROUND_MAX:
            raise ValueError(f'{conf_path}.distance_to_ground muse be in range '
                             f'[{DISTANCE_TO_GROUND_MIN}, {DISTANCE_TO_GROUND_MAX}]')
        if self.max_modification < MAX_MODIFICATION_MIN or self.max_modification > MAX_MODIFICATION_MAX:
            raise ValueError(f'{conf_path}.max_modification muse be in range '
                             f'[{MAX_MODIFICATION_MIN}, {MAX_MODIFICATION_MAX}]')


@dataclass
class WinchingHoming:
    velocity: float
    force: float
    force_sense: float
    lock_time: float
    timeout_modifier: TimeoutModifier = field(default=TimeoutModifier(1.5, 5.0))

    def validate(self, conf_path: str) -> None:
        if self.velocity < SLOW_VELOCITY_MIN or self.velocity > SLOW_VELOCITY_MAX:
            raise ValueError(f'{conf_path}.velocity must be in range [{SLOW_VELOCITY_MIN}, {SLOW_VELOCITY_MAX}]')
        if self.force < FORCE_HOME_MIN or self.force > FORCE_MAX:
            raise ValueError(f'{conf_path}.force must be in range [{FORCE_HOME_MIN}, {FORCE_MAX}]')
        if self.force_sense < FORCE_SENSE_MIN or self.force_sense > self.force:
            raise ValueError(f'{conf_path}.force must be in range [{FORCE_SENSE_MIN}, {conf_path}.force]')
        if self.lock_time < LOCK_TIME_MIN or self.lock_time > LOCK_TIME_MAX:
            raise ValueError(f'{conf_path}.lock_time must be in range [{LOCK_TIME_MIN}, {LOCK_TIME_MAX}]')
        self.timeout_modifier.validate('timeout_modifier')


@dataclass
class WinchingHome:
    velocity: float
    force: float

    def validate(self, conf_path: str) -> None:
        if self.velocity < SLOW_VELOCITY_MIN or self.velocity > SLOW_VELOCITY_MAX:
            raise ValueError(f'{conf_path}.velocity must be in range [{SLOW_VELOCITY_MIN}, {SLOW_VELOCITY_MAX}]')
        if self.force < FORCE_HOME_MIN or self.force > FORCE_MAX:
            raise ValueError(f'{conf_path}.force must be in range [{FORCE_HOME_MIN}, {FORCE_MAX}]')


@dataclass
class WinchingLoad:
    time: float
    velocity: float
    force: float
    length: float
    timeout_modifier: TimeoutModifier = field(default=TimeoutModifier(1.5, 2.0))

    def validate(self, conf_path: str) -> None:
        if self.time < LOAD_TIME_MIN or self.time > LOAD_TIME_MAX:
            raise ValueError(f'{conf_path}.time must be in range [{LOAD_TIME_MIN}, {LOAD_TIME_MAX}]')
        if self.velocity < SLOW_VELOCITY_MIN or self.velocity > SLOW_VELOCITY_MAX:
            raise ValueError(f'{conf_path}.velocity must be in range [{SLOW_VELOCITY_MIN}, {SLOW_VELOCITY_MAX}]')
        if self.force < FORCE_MIN or self.force > FORCE_MAX:
            raise ValueError(f'{conf_path}.force must be in range [{FORCE_MIN}, {FORCE_MAX}]')
        if self.length < LOAD_LENGTH_MIN or self.length > LOAD_LENGTH_MAX:
            raise ValueError(f'{conf_path}.length must be in range [{LOAD_LENGTH_MIN}, {LOAD_LENGTH_MAX}]')
        self.timeout_modifier.validate('timeout_modifier')


@dataclass
class WinchingDrop:
    velocity: float
    velocity_slow: float
    force: float
    force_brake: float
    force_hold: float
    allow_while_retracting: bool
    length: float
    length_slow: float = 0.0
    force_tug_detect: float = math.nan
    timeout_modifier: TimeoutModifier = field(default=TimeoutModifier(1.5, 5.0))
    height_modifier: Optional[HeightModifier] = None

    def validate(self, conf_path: str) -> None:
        if self.velocity < FAST_VELOCITY_MIN or self.velocity > FAST_VELOCITY_MAX:
            raise ValueError(f'{conf_path}.velocity must be in range [{FAST_VELOCITY_MIN}, {FAST_VELOCITY_MAX}]')
        if self.velocity_slow < SLOW_VELOCITY_MIN or self.velocity_slow > SLOW_VELOCITY_MAX:
            raise ValueError(f'{conf_path}.velocity_slow must be in range [{SLOW_VELOCITY_MIN}, {SLOW_VELOCITY_MAX}]')
        if self.force < FORCE_MIN or self.force > FORCE_MAX:
            raise ValueError(f'{conf_path}.force must be in range [{FORCE_MIN}, {FORCE_MAX}]')
        if self.force_brake < FORCE_MIN or self.force_brake > FORCE_MAX:
            raise ValueError(f'{conf_path}.force_brake must be in range [{FORCE_MIN}, {FORCE_MAX}]')
        if self.force_hold < FORCE_MIN or self.force_hold > FORCE_MAX:
            raise ValueError(f'{conf_path}.force_hold must be in range [{FORCE_MIN}, {FORCE_MAX}]')
        if math.isfinite(self.force_tug_detect):
            if self.force_tug_detect > self.force_hold:
                raise ValueError(f'{conf_path}.force_tug_detect can not be greater than {conf_path}.force_hold')
            if self.force_tug_detect < FORCE_TUG_DETECT_MIN:
                raise ValueError(f'{conf_path}.force_tug_detect can not be less than {FORCE_TUG_DETECT_MIN}')
        else:
            if not math.isnan(self.force_tug_detect):
                raise ValueError(f'{conf_path}.force_tug_detect must be a finite number or nan')
        if self.length < DROP_LENGTH_MIN or self.length > DROP_LENGTH_MAX:
            raise ValueError(f'{conf_path}.length must be in range [{DROP_LENGTH_MIN}, {DROP_LENGTH_MAX}]')
        if self.length_slow < SLOW_DROP_LENGTH_MIN or self.length_slow > SLOW_DROP_LENGTH_MAX:
            raise ValueError(f'{conf_path}.length_slow must be in range '
                             f'[{SLOW_DROP_LENGTH_MIN}, {SLOW_DROP_LENGTH_MAX}]')
        if self.height_modifier is not None:
            self.height_modifier.validate(f'{conf_path}.height_modifier')
        self.timeout_modifier.validate('timeout_modifier')


@dataclass
class WinchingRetract:
    velocity: float
    force: float
    homing_distance: float
    timeout_modifier: TimeoutModifier = field(default=TimeoutModifier(1.5, 5.0))

    def validate(self, conf_path: str) -> None:
        if self.velocity < FAST_VELOCITY_MIN or self.velocity > FAST_VELOCITY_MAX:
            raise ValueError(f'{conf_path}.velocity must be in range [{FAST_VELOCITY_MIN}, {FAST_VELOCITY_MAX}]')
        if self.force < FORCE_MIN or self.force > FORCE_MAX:
            raise ValueError(f'{conf_path}.force must be in range [{FORCE_MIN}, {FORCE_MAX}]')
        if self.homing_distance < HOMING_DISTANCE_MIN or self.homing_distance > HOMING_DISTANCE_MAX:
            raise ValueError(f'{conf_path}.homing_distance must be in range '
                             f'[{HOMING_DISTANCE_MIN}, {HOMING_DISTANCE_MAX}]')
        self.timeout_modifier.validate('timeout_modifier')


@dataclass
class WinchingConfiguration:
    wire_length: float
    homing: WinchingHoming
    home: WinchingHome
    load: WinchingLoad
    drop: WinchingDrop
    retract: WinchingRetract

    def validate(self, conf_path: str) -> None:
        self.homing.validate(conf_path + 'homing')
        self.home.validate(conf_path + 'home')
        self.load.validate(conf_path + 'load')
        self.drop.validate(conf_path + 'drop')
        self.retract.validate(conf_path + 'retract')
        if self.wire_length < WIRE_LENGTH_MIN or self.wire_length > WIRE_LENGTH_MAX:
            raise ValueError(f'{conf_path}.wire_length must be in range [{WIRE_LENGTH_MIN}, {WIRE_LENGTH_MAX}]')

        # Some additional checks that compares values from different sub components
        if self.wire_length < self.drop.length + MAX_DROP_MARGIN:
            raise ValueError(f'{conf_path}.wire_length must be greater than or equal to {conf_path}.drop.length '
                             f'+ {MAX_DROP_MARGIN}')
        if self.drop.height_modifier is not None:
            if self.wire_length < self.drop.length + self.drop.height_modifier.max_modification + MAX_DROP_MARGIN:
                raise ValueError(f'{conf_path}.wire_length must be greater than or equal to '
                                 f'{conf_path}.drop.length + {conf_path}.drop.height_modifier.max_modification '
                                 f'+ {MAX_DROP_MARGIN}')
            if self.drop.length + self.drop.height_modifier.max_modification > DROP_LENGTH_MAX:
                raise ValueError(f'{conf_path}.drop.length + {conf_path}.drop.height_modifier.max_modification '
                                 f'must not exceed {DROP_LENGTH_MAX}')
            if self.drop.length - self.drop.height_modifier.max_modification < DROP_LENGTH_MIN:
                raise ValueError(f'{conf_path}.drop.length - {conf_path}.drop.height_modifier.max_modification '
                                 f'must not be at least {DROP_LENGTH_MIN}')


@dataclass
class TelemetryLoggingConfiguration:
    path_format: str
    verbose: bool = False

    def validate(self, conf_path: str) -> None:
        try:
            sample_path = datetime.utcnow().strftime(self.path_format)
            if len(sample_path) < 1:
                # TODO  Could add more checks to tests if the result is a valid path, and that we can create file
                raise ValueError(f'{conf_path}.path_format evaluates to an empty string')
        except Exception:
            raise ValueError(f'{conf_path}.path_format has a unknown error')
