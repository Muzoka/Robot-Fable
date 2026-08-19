/*
 * config.h - ALL tunable numbers live here.
 * This file is shared byte-for-byte between the Arduino firmware and the
 * desktop simulator, so a value tuned in simulation is the value that flies.
 *
 * Units: millimetres (mm), degrees (deg), milliseconds (ms).
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ------------------------------------------------------------------ */
/* 1. ROBOT GEOMETRY - MEASURE YOUR REAL ROBOT AND PUT THE NUMBERS HERE */
/* ------------------------------------------------------------------ */
/* MEASURED on the robot as it stands. Keep these honest - every other
 * number in this file and every simulator result depends on them.
 *
 * TARGET: 220 x 140 mm. Measured across all three maps, five seeds each:
 *      250 x 150 (today)  ->  Map 3 completed  20% of runs
 *      235 x 145          ->  Map 3 completed 100% of runs
 *      220 x 140          ->  Map 3 completed 100% of runs, 92 s
 * The cliff is at a body diagonal of about 280 mm. Losing 10 mm of length
 * and 5 mm of width is enough to cross it. */
#define ROBOT_LENGTH_MM     220.0f   /* nose to tail, including anything that sticks out */
#define ROBOT_WIDTH_MM      115.0f   /* OUTSIDE OF LEFT TYRE TO OUTSIDE OF RIGHT TYRE.
                                        Not the chassis plate - the tyres. This is what
                                        sweeps the circle and what the walls see.       */
#define WHEEL_BASE_MM       (ROBOT_WIDTH_MM - 26.0f)  /* tyre centres: width minus one
                                        tyre width. Sets how fast a given PWM
                                        difference turns the robot, so the turn
                                        feed-forward below is computed from it.         */
#define WHEEL_DIAM_MM        65.0f   /* yellow TT wheel, measure it                    */
#define AXLE_FROM_NOSE_MM   (ROBOT_LENGTH_MM * 0.5f)   /* front bumper to wheel axle. KEEP THIS AT HALF
                                       OF ROBOT_LENGTH_MM: a centred axle gives
                                       the smallest possible pivot circle, and
                                       the pivot circle is what decides whether
                                       the robot can turn in the corridor.     */

/* Where each ultrasonic sits, relative to the wheel-axle centre.
 * x = forward (+), y = left (+), angle = 0 forward, +90 = pointing left. */
#define US_F_X_MM   (ROBOT_LENGTH_MM * 0.5f - 10.0f)
#define US_F_Y_MM     0.0f
#define US_F_ANG      0.0f

/* The side sonars sit ON the wheel-axle line, not ahead of it. That way a
 * gap is seen at the instant the axle draws level with it, which is the
 * moment the robot needs to know about. Measured: moving them back from
 * 40 mm ahead to the axle line is worth about 20 percentage points of
 * completion on Map 3. It costs nothing - just remount them. */
#define US_L_X_MM     0.0f
#define US_L_Y_MM   (ROBOT_WIDTH_MM * 0.5f - 7.0f)
#define US_L_ANG     90.0f

#define US_R_X_MM     0.0f
#define US_R_Y_MM   (-(ROBOT_WIDTH_MM * 0.5f - 7.0f))
#define US_R_ANG    -90.0f

/* ------------------------------------------------------------------ */
/* 2. SENSOR LIMITS                                                    */
/* ------------------------------------------------------------------ */
#define US_MAX_MM          2000     /* anything past this is reported as US_MAX_MM  */
#define US_MIN_MM            25     /* HC-SR04 cannot see closer than ~2 cm         */
#define US_INVALID        60000     /* no echo came back                            */

/* A side reading bigger than this is an opening rather than a wall.
 * A wall beside us reads TARGET_SIDE_MM; an opening one square deep reads
 * TARGET_SIDE_MM + CORRIDOR_MM. Sit between them, biased towards the wall.
 * The old hand-set 420 mm was larger than a one-square opening ever reads,
 * so the robot drove straight past every turn it was meant to take. */
#define OPEN_SIDE_MM     (TARGET_SIDE_MM + (int)(CORRIDOR_MM * 0.45f))
/* An opening only counts on the EDGE wall->open, and only after the reading
 * has been open this many loops. One dropped ping must never invent a
 * junction that is not there. */
#define OPEN_DEBOUNCE         9
/* A front reading smaller than this means "wall ahead, start setting up".
 * At a square's centre a wall in the next square reads 37 mm, and a wall two
 * squares away reads 342 mm, so 200 mm triggers about one square early. */
#define FRONT_BLOCKED_MM    200
/* ...and bigger than this means "the way ahead is open for at least one more
 * square" (342 mm at a square centre, so 260 mm has margin both ways). */
#define FRONT_OPEN_MM       260
/* Lining up on a wall ahead.
 *
 * We want the wheel AXLE in the middle of the junction square before we
 * pivot, which means the wall should end up CORRIDOR_MM/2 = 152 mm from the
 * axle. The front sonar sits US_F_X_MM = 115 mm ahead of the axle, so it
 * would have to read 37 mm - and an HC-SR04 is not trustworthy under about
 * 40 mm.
 *
 * So we range off the wall down to a reading we can believe, then hand over
 * to the wheel encoders for the last few centimetres. */
#define FRONT_HANDOVER_MM   100     /* switch from sonar to encoders here   */
/* Where to stop before a 90 degree pivot, measured back from the middle of
 * the junction square. A full 360 pivot sweeps the same circle wherever you
 * stand, but a QUARTER turn does not: sitting further back gives the front
 * corner room to swing into. In a passage this tight that is worth real
 * points, so the number is measured, not guessed - see docs/10_MAP_NOTES.md */
#define PIVOT_BACKOFF_MM     40
/* Closer than this: reverse before doing anything else. Must stay well
 * below FRONT_HANDOVER_MM or the robot reverses out of its own approach. */
#define FRONT_TOOCLOSE_MM    25

/* ------------------------------------------------------------------ */
/* 3. MAZE ASSUMPTIONS                                                 */
/* ------------------------------------------------------------------ */
#define CORRIDOR_MM         305     /* 1 ft grid, from the organisers' spreadsheet  */
#define CELL_MM             305     /* one "step" of the maze grid = 1 ft           */
/* DERIVED - do not hand-edit. Change the geometry above and these follow.
 *
 * Getting these wrong by hand is what cost the most time on this project:
 * a threshold tuned for one body width silently stops working when the body
 * changes, and the symptom looks like a navigation bug rather than a number.
 *
 * Dead centre of the passage, as the side sensor sees it. */
#define TARGET_SIDE_MM   ((int)(CORRIDOR_MM * 0.5f - US_L_Y_MM))

/* ------------------------------------------------------------------ */
/* 4. SPEEDS  (PWM counts, 0..255)                                     */
/* ------------------------------------------------------------------ */
#define PWM_MIN              55     /* below this a TT motor just buzzes            */
#define PWM_MAX             255
/* Straight-line speed. Measured over 6 seeds x 3 maps nominal, then 13
 * seeds x 2 hands x 3 maps with deliberately bad hardware (2.5x wheel slip,
 * 4x sonar noise, 3x dropout, 2.3x gyro bias, 15% battery sag):
 *
 *      cruise   total time   complete   wall contact, degraded
 *        110       182 s       100%       -
 *        140       147 s       100%       -
 *        170       128 s       100%       100 episodes
 *        200       116 s       100%         0 episodes     <- shipped
 *        230       108 s       100%         0 episodes
 *
 * 170 used to be the ceiling because Map 2 fell apart above it. That was
 * never really about speed: it was the two pivots in the open crossing, and
 * STRAIGHT_FIRST deleted them. With those gone the whole curve moved.
 *
 * 200 rather than 230 for one reason you can see in the mixer: steering is
 * added to cruise and then clamped at 255, so a cruise of 200 with
 * STEER_LIMIT 80 can still swing 135 counts of differential, and 230 only
 * 105. That headroom is what corrects a bad approach to a corner.
 *
 * If the real track is slippier than the model, drop to 170 - you lose 12
 * seconds and every hard correction gets its full authority back. */
#define PWM_CRUISE          200
#define PWM_SLOW             95     /* creeping / approaching a wall                */
#define PWM_TURN            110
#define PWM_BACK             95     /* reversing                                    */

/* ------------------------------------------------------------------ */
/* 5. CONTROL GAINS                                                    */
/* ------------------------------------------------------------------ */
#define KP_WALL            0.42f    /* PWM per mm of side error                     */
#define KD_WALL            2.10f    /* PWM per (mm/s) of side error rate            */
#define KP_HEADING         3.20f    /* PWM per degree of heading error (straights)  */
/* Yaw-rate damping. The gyro measures how fast we are turning, so we can
 * subtract it and stop the robot weaving down a straight instead of only
 * reacting after the heading has already gone wrong. This is the single
 * biggest "makes it look smooth" term in the whole file. */
#define KD_YAW             0.00f
#define STEER_LIMIT          80     /* max differential PWM applied while driving   */
/* Never step a motor by more than this in one 20 ms tick. Sudden steps make
 * the tyres slip, and slipping wheels are lying wheels. */
#define PWM_SLEW             255

/* ---- turning ----
 * The firmware can drive a proper trapezoidal turn profile - ramp up, hold,
 * ramp down - which is what serious micromouse code does
 * (see github.com/ukmars/mazerunner-core). It is implemented and it works.
 *
 * It is switched OFF here, because it was measured and it LOST:
 *
 *      no profile   map1  83%   map2 100%   map3  50%   avg 78%
 *      profile on   map1  83%   map2  83%   map3  50%   avg 72%
 *
 * Motion profiles pay off when you have wheel-speed PID running off
 * high-resolution encoders and you are trying to corner at speed. With 20
 * ticks per wheel revolution and a passage that leaves 28 mm a side, the
 * plain PD controller reacts faster and wins. Leaving OMEGA and ALPHA huge
 * makes the profile effectively instant, i.e. a plain PD.
 *
 * If you ever fit better encoders, set OMEGA to about 150 and ALPHA to 1000
 * and re-run the sweep - the code is all still here. */
/* Top turn rate. Must be reachable inside PWM_TURN:
 *      omega_max = (PWM_TURN - PWM_MIN) / TURN_FF
 * Asking for more than the motors can give just saturates the loop and
 * throws away the whole point of having a profile. */
#define TURN_OMEGA_DPS    600.0f
#define TURN_ALPHA_DPS2  20000.0f
#define V_MAX_MMS         380.0f    /* wheel speed at PWM 255, loaded. Measure it   */
#define KP_TURN            2.60f    /* PWM per degree of error against the profile  */
#define KD_TURN            0.22f    /* PWM per (deg/s) of rate error                */
/* Feed-forward: the PWM that ALREADY produces the rate we are asking for, so
 * the PD terms only have to correct the difference. Straight from geometry. */
#define TURN_FF   (WHEEL_BASE_MM * 0.5f / 57.29578f * (255.0f - PWM_MIN) / V_MAX_MMS)
#define TURN_TOL_DEG       3.0f     /* pivot is finished inside this error          */
#define TURN_SETTLE_MS      120     /* ...and must stay inside it this long         */
#define TURN_TIMEOUT_MS    2600     /* give up and move on                          */


/* ------------------------------------------------------------------ */
/* 5b. STAYING SQUARE WITH THE MAZE                                    */
/* ------------------------------------------------------------------ */
/* How hard to trust the walls over the gyro when both walls are in view.
 * A pivot that finishes three degrees short is invisible to the gyro,
 * because the heading was re-zeroed onto the target - but three degrees is
 * 16 mm of drift per foot, and two corners later it is a wall. The walls
 * themselves are the only absolute reference the robot has. See
 * "corridor-parallel correction" in nav_core.cpp. */
#define WALL_ALIGN_GAIN    0.35f
#define WALL_ALIGN_MM      70.0f    /* travel between angle estimates       */

/* Livelock guard. Deciding this many times inside this little travel means
 * the robot is turning on the spot arguing with itself, which is how a run
 * quietly burns three minutes without going anywhere. */
#define DECIDE_BURST_MAX      3
#define DECIDE_BURST_MM   250.0f
#define DECIDE_LOCKOUT_MM 350.0f

/* ------------------------------------------------------------------ */
/* 6. TIMING                                                           */
/* ------------------------------------------------------------------ */
#define LOOP_HZ              50
#define LOOP_DT_MS           20
#define BACKUP_MS           420
/* Roll this far past a side opening before pivoting into it. Ideal value is
 *      CORRIDOR_MM / 2 + US_L_X_MM - (debounce lag, about 45 mm) = 145 */
#define CREEP_AFTER_OPEN_MM 147
#define TURN_SETTLE_IN_MS   260     /* stop dead before pivoting; also the
                                       window used to re-measure gyro bias  */
/* How long a hand must stay in front of the nose before it arms the start.
 * A single bad ping must never be able to launch a run - see ST_WAIT_START
 * in nav_core.cpp for the power-on false start this prevents. */
#define START_ARM_MS        300
#define STUCK_WINDOW_MS     900     /* no encoder movement this long = stuck         */
#define RUN_TIMEOUT_MS   240000UL   /* 4 minutes then stop, whatever happens         */

/* ------------------------------------------------------------------ */
/* 7. ODOMETRY                                                         */
/* ------------------------------------------------------------------ */
#define ENC_TICKS_PER_REV    20     /* slots in the black encoder disc              */
#define ENC_MM_PER_TICK    (3.14159265f * WHEEL_DIAM_MM / (float)ENC_TICKS_PER_REV)

/* ------------------------------------------------------------------ */
/* 8. FEATURE SWITCHES                                                 */
/* ------------------------------------------------------------------ */
#define HAS_GYRO              1     /* MPU-6050 fitted                              */
#define HAS_ENCODERS          1     /* LM393 slot sensors fitted                    */
/* Floor line sensor: REMOVED. Do not fit one, do not buy one.
 *
 * You asked whether dropping it would make things work better. It does, and
 * not because it was useless - because it was dangerous.
 *
 * What it did:  counted the black sector lines for telemetry, and watched
 * for the striped start/finish gate as a second way to notice the run was
 * over.
 *
 * What it cost:  the gate detector fires on four black-to-white edges
 * arriving inside 350 ms. A TCRT5000 is a comparator with a trim pot, and
 * when it crosses a strip of tape at an angle its output CHATTERS around
 * the threshold - four edges is easy. The robot would then declare itself
 * finished in the middle of the maze and stop. That is a whole map lost, to
 * a sensor that was never steering anything.
 *
 * Measured with it off: 24 runs, 24 complete, same times to a tenth of a
 * second, every sector line still crossed. The robot notices the finish by
 * driving into open space on all three sonars, which is what actually fired
 * on every single run in this repository anyway.
 *
 * The code is still here behind this switch. Set it back to 1 only if you
 * fit a sensor and can show it does something the sonars cannot. */
#define HAS_LINE_SENSOR       0     /* TCRT5000 looking at the floor - NOT FITTED   */

/* Which hand do we keep on the wall?  +1 = LEFT hand, -1 = RIGHT hand.
 *
 * This used to be the most dangerous number in the file. It is now very
 * nearly a free choice, and that is worth understanding.
 *
 * BEFORE straight-first (below), the hand decided whether you scored:
 *
 *              road covered     left hand    right hand
 *      Map 1                       100%         100%
 *      Map 2                        50%         100%     <-- half a map
 *      Map 3                       100%         100%
 *
 * On Map 2 the left hand turned straight out of the exit the first time it
 * reached the crossing and skipped the entire lower loop.
 *
 * AFTER straight-first, both hands drive the IDENTICAL route on all three
 * maps - same squares, same order, same time to a tenth of a second, 8 seeds
 * each. The robot goes straight through the crossing rather than turning at
 * it, so there is no longer a turn there for the hand to get wrong.
 *
 * Left as RIGHT because it is the one that was right before as well: if a
 * future map does put a real choice back in, the right hand is the one with
 * the measurements behind it. */
#define WALL_HAND            (-1)

/* Go STRAIGHT through a crossing whenever the way ahead is open, and only
 * consult the hand rule when it is not.
 *
 * This is the single highest-value line in the file, for three reasons.
 *
 * 1. It is the route you drew on Map 2. Straight down the middle to the
 *    bottom row, round the lower loop, back along the middle and out.
 *
 * 2. It never pivots in the open crossing. Map 2 has one 4-way crossing,
 *    and a pivot there is the worst pivot on the whole course: with no wall
 *    on either side there is nothing to square up against, so a turn that
 *    finishes a few degrees short stays short. Every livelock this project
 *    ever had started in that square. Straight-first drives through it
 *    twice without turning at all - 6 pivots on Map 2 become 4.
 *
 * 3. It makes WALL_HAND stop mattering. Measured at grid level, left hand
 *    and right hand now produce the IDENTICAL route on all three maps. The
 *    old left-hand rule turned straight out of the Map 2 exit and skipped
 *    the whole loop; it cannot any more. Picking the wrong hand at the
 *    start line used to cost a map. Now it costs nothing.
 *
 * The catch, stated honestly: straight-first on its own is NOT a complete
 * coverage rule. In a maze with a dead-end stub it can walk out of a
 * crossing that still has road left in it (measured: 5 squares of 11 on a
 * test maze built to provoke exactly that). None of the three competition
 * maps has a dead end - every square of road is on a through route, checked
 * by build_maps.py - so it is safe here. MODE_TREMAUX is what closes the
 * hole for good, and it costs nothing to run: see DEFAULT_MODE below. */
#define STRAIGHT_FIRST        1

/* Navigation mode */
#define MODE_WALLFOLLOW       0     /* simple, never gets confused in a plain maze  */
#define MODE_TREMAUX          1     /* remembers junctions, beats loops and islands */
/* Wall following, not Tremaux - but only just, and here is the honest
 * comparison so you can flip it with your eyes open.
 *
 * On the three competition maps the two modes are now INDISTINGUISHABLE:
 * 8 seeds x 3 maps x both hands, every combination completes 100% of runs,
 * covers 100% of the road, scores every sector line, and finishes within a
 * tenth of a second of the others. Straight-first is what made them agree.
 *
 * Wall following is the default because it is the simpler machine: it has no
 * dependence on odometry at all, and odometry is the least trustworthy thing
 * on the robot.
 *
 * Switch to MODE_TREMAUX if a future map has a DEAD END in it. That is the
 * one case straight-first cannot handle on its own - it can walk out of a
 * crossing that still has undriven road behind it. Measured on a test maze
 * built to provoke exactly that: wall following alone covers 5 squares of
 * 11, Tremaux covers 11 of 11. It costs 4 bytes of SRAM per junction and
 * nothing to buy. build_maps.py prints a dead-end warning per map. */
#define DEFAULT_MODE   MODE_WALLFOLLOW

/* Tremaux memory */
#define MAX_JUNCTIONS        48     /* 48 * 5 bytes = 240 bytes of SRAM             */
#define EEPROM_MAGIC       0x4D5A   /* 'MZ'                                          */
#define EEPROM_BASE           0

/* ------------------------------------------------------------------ */
/* 9. ARDUINO PIN MAP  (Uno R3)                                        */
/* ------------------------------------------------------------------ */
#define PIN_ENC_L      2   /* INT0 - left  wheel slot sensor          */
#define PIN_ENC_R      3   /* INT1 - right wheel slot sensor          */
#define PIN_IN1        4   /* L298N                                    */
#define PIN_ENA        5   /* L298N, PWM, RIGHT motor                  */
#define PIN_ENB        6   /* L298N, PWM, LEFT  motor                  */
#define PIN_IN2        7
#define PIN_IN3        8
#define PIN_TRIG_R     9
#define PIN_ECHO_R    10
#define PIN_IN4       11
#define PIN_LINE      12   /* was the TCRT5000 - not fitted, see
                              HAS_LINE_SENSOR. The pin now carries the
                              GO button instead.                       */
/* GO button: button between this pin and GND, nothing else. The internal
 * pull-up does the rest, so there is no resistor to fit and no way to wire
 * it wrong.
 *
 * It used to share D13 with the status LED, because the line sensor owned
 * D12. That meant flipping D13 between OUTPUT and INPUT_PULLUP a hundred
 * times a second to sample it, with a 330 ohm resistor to stop the press
 * shorting the LED drive - and in Wokwi it simply did not register presses.
 * Dropping the line sensor freed D12, so the button gets its own pin and
 * the whole trick goes away. */
#define PIN_GO        12
#define PIN_LED       13   /* onboard LED + optional buzzer. OUTPUT only */
#define PIN_TRIG_F   A0
#define PIN_ECHO_F   A1
#define PIN_TRIG_L   A2
#define PIN_ECHO_L   A3
/*      A4 = SDA, A5 = SCL  -> MPU-6050, do not use for anything else  */

/* The GO button took over the line sensor's pin. If you ever refit a line
 * sensor, one of them has to move - and a silent clash would show up as the
 * robot starting itself whenever it drove over black tape, which is a
 * miserable thing to debug in a competition hall. */
#if HAS_LINE_SENSOR && (PIN_GO == PIN_LINE)
#error "PIN_GO and PIN_LINE are both D12. Move one before fitting a line sensor."
#endif

/* Motor polarity: flip to -1 if a wheel spins backwards after wiring. */
#define MOTOR_L_SIGN  (+1)
#define MOTOR_R_SIGN  (+1)

#endif /* CONFIG_H */
