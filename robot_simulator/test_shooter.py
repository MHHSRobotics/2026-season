"""Test script: feeds 5 fuel balls through hopper -> feed -> flywheels and tracks trajectories.

Spins up flywheels to 4800 RPM, places 5 balls on the hopper rollers,
runs hopper + feed + flywheels together, and measures each ball's launch
parameters and full trajectory until landing.
"""

import math
import time
import numpy as np
import mujoco
import mujoco.viewer
from sim import SwerveSimulator, SwerveInputs
from sim.nt_interface import SwerveModuleState, MechanismInputs
from sim.simulator import (
    SHOOTER_EXIT_ANGLE, SHOOTER_SPEED_EFFICIENCY, FLYWHEEL_RADIUS,
    SHOOTER_MIN_LAUNCH_SPEED, SHOOTER_EXIT_POS_LOCAL, _quat_rotate,
)

# Target constants
TARGET_RPM = 4800
TARGET_OMEGA = TARGET_RPM * 2.0 * math.pi / 60.0
TARGET_EXIT_SPEED_FPS = 31.0
TARGET_EXIT_SPEED_MPS = TARGET_EXIT_SPEED_FPS * 0.3048
TARGET_EXIT_ANGLE_DEG = 73.0
TARGET_EXIT_HEIGHT_IN = 17.0
TARGET_EXIT_HEIGHT_M = TARGET_EXIT_HEIGHT_IN * 0.0254

NUM_BALLS = 5
ZERO_MODULE = SwerveModuleState()
TRAJ_SAMPLE_INTERVAL = 0.05  # 50ms

# Valid launch thresholds (reject bad feeds)
MIN_VALID_SPEED_FPS = 15.0
MIN_VALID_ANGLE_DEG = 30.0
MAX_VALID_HEIGHT_IN = 30.0


def regulate_flywheel_voltage(flywheel_vel):
    """Bang-bang + back-EMF hold to regulate flywheel to TARGET_OMEGA."""
    if flywheel_vel < TARGET_OMEGA * 0.95:
        return 12.0
    elif flywheel_vel > TARGET_OMEGA * 1.02:
        return 0.0
    else:
        kv = 12.0 / (5800 * 2 * math.pi / 60)
        return flywheel_vel * kv + 0.5


def main():
    print("=" * 70)
    print("MULTI-BALL SHOOTER TEST (%d balls, full hopper pipeline)" % NUM_BALLS)
    print("=" * 70)

    sim = SwerveSimulator("models/robot.xml")
    sim._rng = np.random.default_rng(seed=42)

    viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
    wall_start = time.time()
    sim_clock = 0.0  # total sim time for real-time pacing

    chassis_adr = sim._chassis_qpos_adr
    chassis_pos = sim.data.qpos[chassis_adr:chassis_adr + 3].copy()
    chassis_quat = sim.data.qpos[chassis_adr + 3:chassis_adr + 7].copy()
    flywheel_dof = sim._mech_jnt_dofs["shooter_flywheel_left_joint"]
    dt = sim.get_timestep()

    print("Robot at (%.3f, %.3f, %.3f)" % (chassis_pos[0], chassis_pos[1], chassis_pos[2]))

    ball_indices = list(range(NUM_BALLS))

    # -- Place balls on hopper rollers ---------------------------------
    # Hopper rollers: X=-0.121 to 0.107 (local), Z=0.056-0.077, Y center=-0.0285
    # Ball radius = 0.075m. Sit on top of roller 4 (highest, Z=0.077, r=0.0255)
    hopper_y = -0.03
    ball_z = 0.077 + 0.0255 + 0.075 + 0.005
    hopper_xs = [-0.10, -0.03, 0.04, 0.11, 0.16]

    print("\nPlacing %d balls on hopper rollers..." % NUM_BALLS)
    for i, ball_idx in enumerate(ball_indices):
        qpos_adr = sim._fuel_qpos_adrs[ball_idx]
        dof_adr = sim._fuel_dof_starts[ball_idx]
        local_pos = np.array([hopper_xs[i], hopper_y, ball_z])
        world_pos = chassis_pos + _quat_rotate(chassis_quat, local_pos)
        sim.data.qpos[qpos_adr:qpos_adr + 3] = world_pos
        sim.data.qpos[qpos_adr + 3:qpos_adr + 7] = [1, 0, 0, 0]
        sim.data.qvel[dof_adr:dof_adr + 6] = 0
        print("  Ball %d: local (%+.3f, %+.3f, %.3f)" %
              (i, local_pos[0], local_pos[1], local_pos[2]))

    mujoco.mj_forward(sim.model, sim.data)

    # Let balls settle
    print("\nSettling (0.5s)...")
    for _ in range(int(0.5 / dt)):
        sim.set_voltages(SwerveInputs(
            fl=ZERO_MODULE, fr=ZERO_MODULE, bl=ZERO_MODULE, br=ZERO_MODULE))
        sim.step(1)
        sim_clock += dt
        viewer.sync()
        elapsed = time.time() - wall_start
        if sim_clock > elapsed:
            time.sleep(sim_clock - elapsed)

    # -- Spin up flywheels ---------------------------------------------
    print("Spinning flywheels to %d RPM..." % TARGET_RPM)
    for _ in range(int(4.0 / dt)):
        fw_v = regulate_flywheel_voltage(abs(sim.data.qvel[flywheel_dof]))
        sim.set_voltages(SwerveInputs(
            fl=ZERO_MODULE, fr=ZERO_MODULE, bl=ZERO_MODULE, br=ZERO_MODULE,
            mechanisms=MechanismInputs(shooter_flywheel_voltage=fw_v)))
        sim.step(1)
        sim_clock += dt
        viewer.sync()
        elapsed = time.time() - wall_start
        if sim_clock > elapsed:
            time.sleep(sim_clock - elapsed)

    fw_vel = abs(sim.data.qvel[flywheel_dof])
    print("Flywheel: %.1f rad/s (%d RPM)" % (fw_vel, fw_vel * 60 / (2 * math.pi)))

    # -- Run hopper + feed + flywheels ---------------------------------
    print("\nRunning HOPPER + FEED + FLYWHEELS...")
    print("      Time       Speed    Angle    Height   FW RPM  Status")

    launch_data = {}
    trajectories = {}  # ball_idx -> [(dt, x, y, z)]
    landed = {}

    sim_time = 0.0
    last_traj_sample = -1.0
    max_run = 20.0

    while sim_time < max_run:
        fw_vel_now = abs(sim.data.qvel[flywheel_dof])
        fw_v = regulate_flywheel_voltage(fw_vel_now)

        sim.set_voltages(SwerveInputs(
            fl=ZERO_MODULE, fr=ZERO_MODULE, bl=ZERO_MODULE, br=ZERO_MODULE,
            mechanisms=MechanismInputs(
                shooter_flywheel_voltage=fw_v,
                shooter_feed_voltage=12.0,
                hopper_roller_voltage=12.0,
            )))
        sim._step_counter = 9
        sim.step(1)
        sim_time += dt
        sim_clock += dt
        viewer.sync()
        elapsed = time.time() - wall_start
        if sim_clock > elapsed:
            time.sleep(sim_clock - elapsed)

        if not viewer.is_running():
            print("\nViewer closed.")
            break

        # Detect launches
        for ball_idx in ball_indices:
            if ball_idx not in launch_data and sim._ball_states[ball_idx] == 1:
                qpos_adr = sim._fuel_qpos_adrs[ball_idx]
                dof_adr = sim._fuel_dof_starts[ball_idx]
                vel = sim.data.qvel[dof_adr:dof_adr + 3].copy()
                pos = sim.data.qpos[qpos_adr:qpos_adr + 3].copy()
                speed = np.linalg.norm(vel)
                horiz = math.sqrt(vel[0]**2 + vel[1]**2)
                angle = math.degrees(math.atan2(vel[2], horiz))
                fw_rpm = fw_vel_now * 60 / (2 * math.pi)
                speed_fps = speed / 0.3048
                height_in = pos[2] / 0.0254

                # Classify launch quality
                valid = (speed_fps >= MIN_VALID_SPEED_FPS and
                         angle >= MIN_VALID_ANGLE_DEG and
                         height_in <= MAX_VALID_HEIGHT_IN)
                status = "OK" if valid else "BAD"

                launch_data[ball_idx] = {
                    'time': sim_time, 'pos': pos.copy(), 'vel': vel.copy(),
                    'speed': speed, 'angle': angle, 'height': pos[2],
                    'fw_rpm': fw_rpm, 'valid': valid,
                }
                trajectories[ball_idx] = [(0.0, pos[0], pos[1], pos[2])]

                i = ball_indices.index(ball_idx)
                print("  #%d  %6.2fs  %6.1f ft/s  %5.1f deg  %5.1f in  %5.0f  %s" %
                      (i, sim_time, speed_fps, angle, height_in, fw_rpm, status))

        # Sample in-flight trajectories (only valid launches, stop at landing)
        if sim_time - last_traj_sample >= TRAJ_SAMPLE_INTERVAL:
            last_traj_sample = sim_time
            for ball_idx in ball_indices:
                if ball_idx in launch_data and launch_data[ball_idx]['valid'] and ball_idx not in landed:
                    qpos_adr = sim._fuel_qpos_adrs[ball_idx]
                    dof_adr = sim._fuel_dof_starts[ball_idx]
                    pos = sim.data.qpos[qpos_adr:qpos_adr + 3].copy()
                    vel = sim.data.qvel[dof_adr:dof_adr + 3].copy()
                    t_rel = sim_time - launch_data[ball_idx]['time']
                    trajectories[ball_idx].append((t_rel, pos[0], pos[1], pos[2]))

                    speed = np.linalg.norm(vel)
                    if pos[2] < 0.15 and speed < 0.5 and t_rel > 0.3:
                        lp = launch_data[ball_idx]['pos']
                        dist = math.sqrt((pos[0] - lp[0])**2 + (pos[1] - lp[1])**2)
                        landed[ball_idx] = {
                            'time': sim_time, 'pos': pos.copy(),
                            'distance': dist, 'flight_time': t_rel,
                        }

        # Early exit
        valid_launched = [b for b in ball_indices if b in launch_data and launch_data[b]['valid']]
        all_valid_landed = all(b in landed for b in valid_launched)
        if len(launch_data) == NUM_BALLS and (all_valid_landed or sim_time > 15.0):
            # Give remaining balls a couple seconds to land
            if all_valid_landed:
                break
        if len(valid_launched) >= NUM_BALLS:
            latest = max(launch_data[b]['time'] for b in valid_launched)
            if sim_time - latest > 5.0:
                break

    # -- Results -------------------------------------------------------
    valid_launches = {k: v for k, v in launch_data.items() if v['valid']}
    bad_launches = {k: v for k, v in launch_data.items() if not v['valid']}
    n_valid = len(valid_launches)

    print("\n" + "=" * 70)
    print("RESULTS: %d/%d launched (%d valid, %d bad)" %
          (len(launch_data), NUM_BALLS, n_valid, len(bad_launches)))
    print("=" * 70)

    if bad_launches:
        print("\nBAD LAUNCHES (ball bounced into feed from wrong position):")
        for ball_idx, ld in sorted(bad_launches.items()):
            i = ball_indices.index(ball_idx)
            print("  #%d: %.1f ft/s @ %.1f deg, %.1f in -- rejected" %
                  (i, ld['speed'] / 0.3048, ld['angle'], ld['height'] / 0.0254))

    if n_valid == 0:
        print("No valid launches!")
        return

    # Valid launches table
    print("\nVALID LAUNCHES:")
    print("Ball   Speed      Angle   Height   FW RPM   Flight    Distance      Peak")
    print("-" * 82)

    speeds = []
    angles = []
    heights = []
    distances = []
    flight_times = []
    peak_heights = []

    for i, ball_idx in enumerate(ball_indices):
        if ball_idx not in valid_launches:
            continue
        ld = valid_launches[ball_idx]
        speeds.append(ld['speed'])
        angles.append(ld['angle'])
        heights.append(ld['height'])

        traj = trajectories.get(ball_idx, [])
        peak_z = max((p[3] for p in traj), default=ld['height'])
        peak_heights.append(peak_z)

        if ball_idx in landed:
            la = landed[ball_idx]
            distances.append(la['distance'])
            flight_times.append(la['flight_time'])
            dist_str = "%.1fm (%.1fft)" % (la['distance'], la['distance'] / 0.3048)
            flight_str = "%.2fs" % la['flight_time']
        else:
            dist_str = "(in flight)"
            flight_str = "---"

        print("  #%d  %5.1f ft/s  %5.1f deg  %4.1f in  %5.0f   %6s  %13s  %.1fm" %
              (i, ld['speed'] / 0.3048, ld['angle'], ld['height'] / 0.0254,
               ld['fw_rpm'], flight_str, dist_str, peak_z))

    # Statistics
    print("\n" + "-" * 70)
    print("STATISTICS (%d valid balls)" % n_valid)
    print("-" * 70)
    print("  Exit speed:    %.1f +/- %.1f ft/s  (target: %.1f)" %
          (np.mean(speeds) / 0.3048, np.std(speeds) / 0.3048, TARGET_EXIT_SPEED_FPS))
    print("  Exit angle:    %.1f +/- %.1f deg    (target: %.1f)" %
          (np.mean(angles), np.std(angles), TARGET_EXIT_ANGLE_DEG))
    print("  Exit height:   %.1f +/- %.1f in     (target: %.1f)" %
          (np.mean(heights) / 0.0254, np.std(heights) / 0.0254, TARGET_EXIT_HEIGHT_IN))
    print("  Peak height:   %.2f +/- %.2f m  (%.1f ft)" %
          (np.mean(peak_heights), np.std(peak_heights), np.mean(peak_heights) / 0.3048))
    if distances:
        print("  Landing dist:  %.2f +/- %.2f m  (%.1f +/- %.1f ft)" %
              (np.mean(distances), np.std(distances),
               np.mean(distances) / 0.3048, np.std(distances) / 0.3048))
    if flight_times:
        print("  Flight time:   %.2f +/- %.2f s" %
              (np.mean(flight_times), np.std(flight_times)))
    fw_rpms = [ld['fw_rpm'] for ld in valid_launches.values()]
    print("  FW RPM range:  %.0f - %.0f  (target: %d)" %
          (min(fw_rpms), max(fw_rpms), TARGET_RPM))

    # Trajectories (valid only, stop at landing)
    print("\n" + "=" * 70)
    print("TRAJECTORIES (50ms samples, valid launches only)")
    print("=" * 70)

    for i, ball_idx in enumerate(ball_indices):
        if ball_idx not in valid_launches:
            continue
        traj = trajectories.get(ball_idx, [])
        if len(traj) < 3:
            continue

        ld = valid_launches[ball_idx]
        lx, ly, lz = ld['pos']
        print("\nBall #%d (%.1f ft/s @ %.1f deg):" %
              (i, ld['speed'] / 0.3048, ld['angle']))
        print("     dt       X       Y       Z      dX      dZ")

        for t_rel, x, y, z in traj:
            print("  %5.2fs  %6.2f  %6.2f  %6.2f  %+5.2f  %+5.2f" %
                  (t_rel, x, y, z, x - lx, z - lz))
            # Stop printing after landing
            if t_rel > 0.3 and z < 0.20:
                print("  ... (landed)")
                break

    # Analytical comparison
    print("\n" + "=" * 70)
    print("ANALYTICAL vs SIMULATED")
    print("=" * 70)

    mean_speed = np.mean(speeds)
    mean_angle_rad = math.radians(np.mean(angles))
    mean_z0 = np.mean(heights)
    vx = mean_speed * math.cos(mean_angle_rad)
    vz = mean_speed * math.sin(mean_angle_rad)
    g = 9.81

    t_peak = vz / g
    z_peak = mean_z0 + vz * t_peak - 0.5 * g * t_peak**2
    disc = vz**2 + 2 * g * mean_z0
    t_land = (vz + math.sqrt(max(0, disc))) / g
    x_land = vx * t_land

    print("  Analytical (no drag):")
    print("    Peak:    %.2fm (%.1fft) at t=%.2fs" % (z_peak, z_peak / 0.3048, t_peak))
    print("    Landing: %.2fm (%.1fft) at t=%.2fs" % (x_land, x_land / 0.3048, t_land))

    if distances and flight_times:
        print("  Simulated (mean of %d balls):" % n_valid)
        print("    Peak:    %.2fm (%.1fft)" %
              (np.mean(peak_heights), np.mean(peak_heights) / 0.3048))
        print("    Landing: %.2fm (%.1fft) at t=%.2fs" %
              (np.mean(distances), np.mean(distances) / 0.3048, np.mean(flight_times)))
        peak_diff = abs(np.mean(peak_heights) - z_peak)
        dist_diff = abs(np.mean(distances) - x_land)
        time_diff = abs(np.mean(flight_times) - t_land)
        print("  Difference:")
        print("    Peak: %.2fm (%.1f%%)  Dist: %.2fm (%.1f%%)  Time: %.2fs (%.1f%%)" %
              (peak_diff, 100 * peak_diff / z_peak if z_peak > 0 else 0,
               dist_diff, 100 * dist_diff / x_land if x_land > 0 else 0,
               time_diff, 100 * time_diff / t_land if t_land > 0 else 0))


if __name__ == "__main__":
    main()
