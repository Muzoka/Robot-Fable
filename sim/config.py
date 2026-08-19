"""All tunable constants, mirroring the future Arduino firmware config.

Every name here becomes a #define in firmware/competition/config.h.
Units: millimetres, milliseconds, degrees, PWM counts (0..255).
Values marked CAL are replaced by Phase-3 on-robot calibration.
"""

# ------------------------------------------------------------------
# Robot geometry (measured by the team; see docs/HARDWARE_AUDIT.md)
# ------------------------------------------------------------------
ROBOT_WIDTH_MM = 157.0        # outside of tyre to outside of tyre
ROBOT_LENGTH_MM = 140.0
WHEEL_TRACK_MM = 131.0        # tyre centre to tyre centre (157 - 26)
WHEEL_DIAM_MM = 62.0
AXLE_TO_FRONT_MM = 95.0       # caster-front layout; photo estimate, CAL
AXLE_TO_REAR_MM = 45.0        # CAL

# Sensor mounting, relative to axle midpoint (x fwd, y left)
US_F_X_MM, US_F_Y_MM = 95.0, 0.0
US_L_X_MM, US_L_Y_MM = 0.0, 66.0     # face position; from centred readings
US_R_X_MM, US_R_Y_MM = 0.0, -57.5

# ------------------------------------------------------------------
# Track geometry
# ------------------------------------------------------------------
CORRIDOR_MM = 305.0
# Reading each side sensor sees when the robot is centred (team-measured:
# L 8.65 cm, R 9.50 cm -> derived exactly from the face offsets above).
TARGET_L_MM = CORRIDOR_MM / 2 - US_L_Y_MM        # 86.5
TARGET_R_MM = CORRIDOR_MM / 2 + US_R_Y_MM        # 95.0
SIDE_SUM_MM = TARGET_L_MM + TARGET_R_MM          # validity check, ~181.5

# ------------------------------------------------------------------
# Sensor interpretation thresholds (geometry-derived)
# ------------------------------------------------------------------
SIDE_OPEN_MM = 220.0        # bigger reading = opening, not a wall
# A wall is only *trackable* for centering while it reads near the target;
# beyond this the PD would chase a receding wall sideways into junctions.
TRACK_L_MM = TARGET_L_MM + 70.0
TRACK_R_MM = TARGET_R_MM + 70.0
ERR_CLAMP_MM = 40.0         # max believable centering error
FRONT_BLOCKED_MM = 200.0    # wall ahead -> slow down, prepare to decide
FRONT_OPEN_MM = 260.0       # way ahead is clear (hysteresis pair)
FRONT_STOP_MM = 85.0        # stop line: axle ends ~centred in junction
FRONT_TOOCLOSE_MM = 32.0    # emergency: reverse first
US_MIN_RELIABLE_MM = 45.0   # below this HC-SR04 output is garbage
US_MAX_MM = 2000.0
OPEN_DEBOUNCE = 5           # consecutive open-evidence raw samples for an edge
FAR_MISSES = 2              # consecutive no-echo before believing "far"
FRONT_DEBOUNCE = 3          # consecutive blocked samples to enter APPROACH

# ------------------------------------------------------------------
# Speeds (PWM counts)  [CAL: deadband + trim on the real robot]
# ------------------------------------------------------------------
PWM_MAX = 255
PWM_FLOOR = 60              # breakaway/deadband floor, CAL
PWM_CRUISE = 110
PWM_SLOW = 95
PWM_TURN = 120
PWM_BACK = 95
TRIM_R_PWM = 1              # right motor bias, counters rightward drift, CAL
PWM_SLEW = 10               # max PWM change per 20 ms tick

# ------------------------------------------------------------------
# Wall-centering PD
# ------------------------------------------------------------------
KP_WALL = 0.22              # PWM per mm of lateral error
KD_WALL = 0.20              # PWM per (mm/s) of error rate
DERIV_CLAMP_MMS = 600.0
STEER_LIMIT = 80
ERR_DEADBAND_MM = 10.0

# ------------------------------------------------------------------
# Maneuver timing  [CAL: turn times at race charge]
# ------------------------------------------------------------------
LOOP_MS = 20                # 50 Hz control loop; ping order F,L,F,R
TURN90_MS_L = 680
TURN90_MS_R = 680
TURN_TRIM_PULSE_MS = 70     # corrective micro-pivot after a bad verify
TURN_MAX_TRIMS = 2
SETTLE_MS = 250             # stop-dead settle before reading/pivoting
POST_TURN_SETTLE_MS = 150
CREEP_MS = 450              # roll axle to junction centre after side-open
REACQUIRE_MS = 1000
SQUARE_MS = 900             # post-turn D-only wall-parallel squaring
KD_SQUARE = 0.45            # PWM per (mm/s) during SQUARE
APPROACH_TIMEOUT_MS = 3000  # commit: creep at most this long, then decide
APPROACH_ABORT_MM = 350.0   # a VALID reading beyond this ends the approach
APPROACH_FAR_MS = 700
CROSS_MAX_MS = 4000         # hard cap on a straight crossing traversal
CROSS_MIN_MS = 800          # ignore walls-back before this (entry walls)       # sustained far w/ no close history ends it too          # gentle PD re-entry after a turn
BACKUP_MS = 450
STUCK_MS = 1100             # sonar readings frozen this long = stuck
STUCK_FRONT_MM = 400.0      # stuck detection active only when nearing something
FRONT_FAR_MM = 400.0        # front must exceed this for finish detection
DECIDE_REVERSE_MS = 260     # blind-zone escape before pivoting
CHAOS_JUMP_MM = 150.0       # front refresh-to-refresh jump counted as chaos
CHAOS_TRIP = 6              # chaos score that triggers a defensive backup
FINISH_OPEN_TICKS = 35
FINISH_ARM_N = 10           # both-walls side refreshes to arm finish watch      # all-open this many loops = out of the maze
FINISH_DRIVE_MS = 4200      # clear the mouth fully, then stop
RUN_TIMEOUT_MS = 240000

# ------------------------------------------------------------------
# Motor / physics model (sim only; firmware doesn't need these)
# ------------------------------------------------------------------
V_MAX_MMS = 520.0           # wheel surface speed at PWM 255, nominal pack
MOTOR_TAU_S = 0.06          # first-order lag
BRAKE_TAU_S = 0.03          # active brake
ACCEL_LIMIT_MMS2 = 2500.0   # traction limit on smooth painted wood
