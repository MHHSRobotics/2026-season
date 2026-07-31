"""Validation harness for the shoot-while-moving solve in MultiCommands.java.

Mirrors the Java virtual-target solve in Python, then integrates the resulting shot
ballistically to measure where the ball actually lands. Reports miss distance with and
without compensation over a grid of (distance, robot velocity).

Three things this checks:
  1. That the analytic flight-time model is self-consistent with the empirical
     getShooterSpeed() regression - i.e. that the implied exit speed per rad/s comes out
     roughly constant across distance. If it does not, one of the constants is wrong.
  2. How large the heading correction actually gets, which sets how fast you can drive and
     still make the shot.
  3. That the fixed-point solve converges.

No MuJoCo needed: robot.xml sets no air viscosity or density, so ball flight there is pure
gravity and the closed-form integration below reproduces it exactly. The constants are
duplicated from MultiCommands.java (not imported from sim/simulator.py) because the point is
to validate the numbers the robot code actually uses.

Run: python test_shooting_while_moving.py
"""

import math

# --- Constants mirrored from MultiCommands.java ------------------------------
HOOD_ANGLE_DEG = 73.0
EXIT_HEIGHT = 0.4318      # m, shooter exit above the floor
TARGET_HEIGHT = 1.825     # m, hub funnel mouth (field.xml: top of hex funnel)
GRAVITY = 9.80665
MAX_SOLVE_ITERATIONS = 8
SOLVE_TOLERANCE = 0.005
MIN_DIST, MAX_DIST = 1.0, 7.0

HOOD = math.radians(HOOD_ANGLE_DEG)


def shooter_speed(dist):
    """getShooterSpeed(): empirical flywheel setpoint in rad/s."""
    dist = max(MIN_DIST, min(MAX_DIST, dist))
    return 32.64 * dist + 219.9


def flight_time(dist):
    """getFlightTime(): launch speed cancels out for a fixed hood."""
    dist = max(MIN_DIST, min(MAX_DIST, dist))
    rise = EXIT_HEIGHT + dist * math.tan(HOOD) - TARGET_HEIGHT
    if rise <= 0:
        return 0.0
    return math.sqrt(2 * rise / GRAVITY)


def required_exit_speed(dist):
    """Exit speed the shot needs, from the horizontal leg of the ballistics."""
    t = flight_time(dist)
    return dist / (math.cos(HOOD) * t) if t > 0 else float("nan")


def solve(shooter_pos, shooter_vel, hub):
    """The virtual-target fixed point from MultiCommands.solve()."""
    virtual_target = hub
    distance = math.dist(shooter_pos, hub)
    t = 0.0
    iterations = 0
    for i in range(MAX_SOLVE_ITERATIONS):
        iterations = i + 1
        previous = distance
        t = flight_time(distance)
        virtual_target = (hub[0] - shooter_vel[0] * t, hub[1] - shooter_vel[1] * t)
        distance = math.dist(shooter_pos, virtual_target)
        if abs(distance - previous) < SOLVE_TOLERANCE:
            break
    dx = virtual_target[0] - shooter_pos[0]
    dy = virtual_target[1] - shooter_pos[1]
    return {
        "virtual_target": virtual_target,
        "distance": distance,
        "bot_angle": math.atan2(dy, dx),
        "flight_time": t,
        "shooter_speed": shooter_speed(distance),
        "iterations": iterations,
    }


def true_exit_speed(fly_speed):
    """Ground-truth exit speed for a flywheel setpoint.

    Defined by inverting the empirical table: whatever distance the table thinks this
    setpoint is for, the ball must actually reach it. This makes stationary shots land dead
    centre by construction, so any residual in the sweep below is compensation error and
    nothing else.
    """
    d = (fly_speed - 219.9) / 32.64
    return required_exit_speed(max(MIN_DIST, min(MAX_DIST, d)))


def simulate(shooter_pos, shooter_vel, bot_angle, fly_speed, exit_speed=None):
    """Integrate a shot and return where it crosses TARGET_HEIGHT on the way down.

    Pass exit_speed to override the ground-truth shooter, e.g. to model MuJoCo's weaker one.
    """
    v = true_exit_speed(fly_speed) if exit_speed is None else exit_speed
    vh = v * math.cos(HOOD)
    vz = v * math.sin(HOOD)

    # Descending root of exitHeight + vz*t - g*t^2/2 = TARGET_HEIGHT
    disc = vz * vz - 2 * GRAVITY * (TARGET_HEIGHT - EXIT_HEIGHT)
    if disc < 0:
        return None  # never reaches hub height
    t = (vz + math.sqrt(disc)) / GRAVITY

    # Ball velocity is the launch vector plus whatever the robot was carrying it at.
    vx = vh * math.cos(bot_angle) + shooter_vel[0]
    vy = vh * math.sin(bot_angle) + shooter_vel[1]
    return (shooter_pos[0] + vx * t, shooter_pos[1] + vy * t)


def main():
    hub = (4.622, 4.035)  # blue hub center, Field.java

    print("=" * 76)
    print("FLIGHT TIME MODEL  (hood %.1f deg, exit %.3f m, target %.3f m)"
          % (HOOD_ANGLE_DEG, EXIT_HEIGHT, TARGET_HEIGHT))
    print("=" * 76)
    print("  dist    flight     table w    needed v    implied v/w")
    print("   (m)      (s)      (rad/s)      (m/s)      (m/s per rad/s)")
    ks = []
    for d in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]:
        w = shooter_speed(d)
        v = required_exit_speed(d)
        k = v / w
        ks.append(k)
        print("  %4.1f    %5.3f     %7.1f     %6.2f       %.4f" % (d, flight_time(d), w, v, k))

    k_mean = sum(ks) / len(ks)
    spread = (max(ks) - min(ks)) / k_mean
    print("\n  mean implied v/w = %.4f  (spread %.1f%% across 1-7 m)" % (k_mean, 100 * spread))
    print("  Physically v/w is a property of the flywheel and should be constant. Spread here")
    print("  is how far the linear getShooterSpeed() table departs from fixed-hood ballistics.")
    ks_mid = ks[2:]  # 3-7 m
    spread_mid = (max(ks_mid) - min(ks_mid)) / (sum(ks_mid) / len(ks_mid))
    print("  Over 3-7 m the spread is only %.1f%%; the 1-2 m rows are the outliers, so the" % (100 * spread_mid))
    print("  table is a good fit at range and drifts up close. Worth re-fitting under 3 m.")
    print("\n  For reference sim/simulator.py models v/w = FLYWHEEL_RADIUS * SHOOTER_SPEED_EFFICIENCY")
    print("  = 0.05 * 0.376 = 0.0188, which is %.0f%% low vs the real speed table." % (100 * (1 - 0.0188 / k_mean)))

    # --- Miss distance sweep -------------------------------------------------
    print("\n" + "=" * 76)
    print("MISS DISTANCE: compensated vs uncompensated")
    print("=" * 76)
    print("Robot sits due -X of the hub and strafes tangentially (worst case).\n")
    print("  dist   speed    heading corr    virtual shift    miss uncomp    miss comp   iters")
    print("   (m)   (m/s)        (deg)            (m)             (m)           (m)")

    worst_in_range = 0.0
    for d in [2.0, 4.0, 6.0]:
        for speed in [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]:
            pos = (hub[0] - d, hub[1])
            vel = (0.0, speed)  # pure tangential

            sol = solve(pos, vel, hub)
            naive_angle = math.atan2(hub[1] - pos[1], hub[0] - pos[0])
            correction = math.degrees(sol["bot_angle"] - naive_angle)
            shift = math.dist(sol["virtual_target"], hub)

            # Uncompensated: aim straight at the hub, range off the true distance.
            landed_naive = simulate(pos, vel, naive_angle, shooter_speed(math.dist(pos, hub)))
            # Compensated: aim at the virtual target, range off the virtual distance.
            landed_comp = simulate(pos, vel, sol["bot_angle"], sol["shooter_speed"])

            miss_naive = math.dist(landed_naive, hub) if landed_naive else float("nan")
            miss_comp = math.dist(landed_comp, hub) if landed_comp else float("nan")

            # Past 7 m the flywheel setpoint and the flight time both clamp, so the shot is
            # outside what the table can express and the residual is a range limit, not an
            # error in the solve.
            clamped = sol["distance"] > MAX_DIST
            if not clamped:
                worst_in_range = max(worst_in_range, miss_comp)

            print("  %4.1f   %4.1f      %+7.2f        %6.2f          %6.2f        %6.3f     %d  %s"
                  % (d, speed, correction, shift, miss_naive, miss_comp, sol["iterations"],
                     "<- out of table range" if clamped else ""))
        print()

    print("Worst compensated miss within table range: %.4f m" % worst_in_range)
    print("\nNotes:")
    print("  - Within the calibrated range the compensated shot is exact to solver precision,")
    print("    which is the expected result: for a fixed hood the virtual target is not an")
    print("    approximation, it is the closed-form answer.")
    print("  - Rows marked out of table range push the virtual distance past 7 m, where")
    print("    getShooterSpeed and getFlightTime both clamp. That is a calibration limit.")
    print("  - Uncompensated miss is roughly velocity * flight time, which is why it explodes.")
    print("  - Heading correction passes 30 deg around 2 m/s tangential. That is the practical")
    print("    ceiling; Shooter/MaxShootingSpeed defaults to 2.0 m/s for this reason.")
    print("  - Radial motion (driving at or away from the hub) costs almost no heading change,")
    print("    only a range change, so it is far cheaper than strafing.")

    mujoco_check(hub)


# sim/simulator.py: base_speed = flywheel_omega * FLYWHEEL_RADIUS * SHOOTER_SPEED_EFFICIENCY
MUJOCO_EXIT_SPEED_PER_RADS = 0.05 * 0.376


def mujoco_check(hub):
    """Sanity checks against the MuJoCo shooter model.

    Two things here bit us during bring-up:

    1. _launch_ball originally SET the ball's velocity to the launch vector rather than
       adding the chassis velocity, so the ball never inherited the robot's motion. A moving
       shot behaved identically to a stationary one, so a correct lead landed exactly one
       lead-length off target, opposite the direction of travel. Fixed in simulator.py; the
       "lead applied" column below is what the miss used to be.

    2. SHOOTER_SPEED_EFFICIENCY implies a weaker shooter than the robot's own speed table.
       Taken at face value that predicts stationary shots falling metres short - which does
       NOT happen, so the flywheel must be running faster than the code commands. The
       required-omega column is there to check that against a logged flywheel velocity.
    """
    print("\n" + "=" * 76)
    print("MUJOCO CROSS-CHECK")
    print("=" * 76)
    print("Sim launches at %.4f * omega; the speed table implies about 0.0246 * omega.\n"
          % MUJOCO_EXIT_SPEED_PER_RADS)
    print("  dist   flight t   lead at 1 m/s   omega commanded   omega needed to score")
    print("   (m)      (s)          (m)           (rad/s)              (rad/s)")

    for d in [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]:
        t = flight_time(d)
        needed = required_exit_speed(d) / MUJOCO_EXIT_SPEED_PER_RADS
        print("  %4.1f     %5.3f        %5.2f           %6.1f               %6.1f"
              % (d, t, 1.0 * t, shooter_speed(d), needed))

    print("\n  If stationary shots score, the actual flywheel speed is tracking the last")
    print("  column, not the one the code commands. Compare against MuJoCo/Shooter/Flywheel/")
    print("  Velocity. Note SHOOTER_MIN_LAUNCH_SPEED is 450 rad/s while the table never")
    print("  commands above %.1f, so nothing should launch at all if tracking were exact."
          % shooter_speed(MAX_DIST))
    print("\n  Do NOT retune SHOOTER_SPEED_EFFICIENCY to close that gap without measuring the")
    print("  real flight time first - the discrepancy may be in the sim's flywheel control,")
    print("  not its exit-speed model, and those need opposite fixes.")


if __name__ == "__main__":
    main()
