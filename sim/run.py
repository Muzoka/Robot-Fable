"""Run simulations.

  python3 -m sim.run one map1 [seed] [--plot out.png]   single run + trace
  python3 -m sim.run mc [n]                             Monte Carlo battery
"""
import sys
import numpy as np
import sim.config as C
from sim.track import MAPS
from sim.robot import Robot
from sim.sensors import SonarSuite
from sim.controller import Controller

PHYS_DT = 0.005                      # 200 Hz physics
TICKS_PER_LOOP = int(C.LOOP_MS / 1000 / PHYS_DT)


def run_once(map_name, seed=0, degraded=False, record=False, auto=False):
    rng = np.random.default_rng(seed)
    track = MAPS[map_name]()
    # perturbations. The race plan calibrates turn times / deadband at race
    # voltage and races in a fixed charge window, so the sim models the
    # POST-calibration residuals: nominal = disciplined race day; degraded =
    # sloppy day (forgot to top up, dusty venue, worse placement).
    if not degraded:
        lat = rng.uniform(-15, 15)
        yaw = np.radians(rng.uniform(-2.5, 2.5))
        battery = rng.uniform(0.90, 1.0)
        mism = rng.normal(0.007, 0.004)      # measured ~0.7% right drift
        noise, drop, db_shift = 3.0, 0.02, rng.uniform(-2, 2)
    else:
        lat = rng.uniform(-25, 25)
        yaw = np.radians(rng.uniform(-4, 4))
        battery = rng.uniform(0.82, 1.0)
        mism = rng.normal(0.007, 0.008)
        noise, drop, db_shift = 8.0, 0.06, rng.uniform(-4, 6)

    track.start_xy = track.start_xy + np.array(
        [lat * np.cos(track.start_heading + np.pi / 2),
         lat * np.sin(track.start_heading + np.pi / 2)])
    robot = Robot(track, rng, gain_l=1 + mism / 2, gain_r=1 - mism / 2,
                  battery=battery,
                  db_start=C.PWM_FLOOR + db_shift,
                  db_sustain=C.PWM_FLOOR - 15 + db_shift)
    robot.th += yaw
    sonar = SonarSuite(track, rng, noise_mm=noise, dropout=drop)
    ctl = Controller(track.script, auto=auto)

    path = []
    log = []
    t_ms = 0
    result = 'timeout'
    prev_state = None
    while t_ms < C.RUN_TIMEOUT_MS:
        refreshed = sonar.tick(robot)
        pl, pr, brake = ctl.tick(sonar, refreshed)
        for _ in range(TICKS_PER_LOOP):
            robot.step(pl, pr, brake, PHYS_DT)
        t_ms += C.LOOP_MS
        if record and t_ms % 100 == 0:
            path.append((robot.x, robot.y, robot.th))
        if record and ctl.state != prev_state:
            log.append((t_ms / 1000.0, prev_state, ctl.state,
                        round(robot.x), round(robot.y),
                        round(np.degrees(robot.th)) % 360,
                        round(sonar.dist('F')), round(sonar.dist('L')),
                        round(sonar.dist('R')), len(ctl.script)))
            prev_state = ctl.state
        if track.in_finish(robot.x, robot.y):
            result = 'success'
            break
        if ctl.done:
            result = 'stopped-short'
            break
    return {
        'result': result, 'time_s': t_ms / 1000.0,
        'contacts': robot.contacts, 'backups': ctl.backups,
        'anomalies': ctl.anomaly, 'path': path, 'track': track,
        'script_left': len(ctl.script), 'log': log,
    }


def plot_run(r, fname):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tr = r['track']
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(~tr.free, origin='lower', cmap='gray_r', alpha=0.9,
              extent=[tr.x0, tr.x1, tr.y0, tr.y1])
    if r['path']:
        p = np.array([(x, y) for x, y, _ in r['path']])
        ax.plot(p[:, 0], p[:, 1], 'r-', lw=1.5)
        ax.plot(p[0, 0], p[0, 1], 'go', ms=8)
        ax.plot(p[-1, 0], p[-1, 1], 'bs', ms=8)
    fx0, fy0, fx1, fy1 = tr.finish_rect
    ax.add_patch(plt.Rectangle((fx0, fy0), fx1 - fx0, fy1 - fy0,
                               fill=False, ec='lime', lw=2))
    ax.set_title(f"{tr.name}: {r['result']} {r['time_s']:.1f}s "
                 f"contacts={r['contacts']}")
    ax.set_aspect('equal')
    fig.savefig(fname, dpi=110, bbox_inches='tight')
    plt.close(fig)


def monte_carlo(n=25, auto=False):
    tag = 'AUTO (no script)' if auto else 'scripted'
    print(f"-- {tag} --")
    print(f"{'map':10s} {'mode':9s} {'done':>5s} {'contact-free':>12s} "
          f"{'avg contacts':>12s} {'avg time':>9s}")
    all_ok = True
    for mname in ('map1', 'map2', 'map3'):
        for degraded in (False, True):
            res = [run_once(mname, seed=s, degraded=degraded, auto=auto)
                   for s in range(n)]
            ok = sum(r['result'] == 'success' for r in res)
            cf = sum(r['result'] == 'success' and r['contacts'] == 0
                     for r in res)
            ac = np.mean([r['contacts'] for r in res])
            at = np.mean([r['time_s'] for r in res if r['result'] == 'success']
                         or [0])
            mode = 'degraded' if degraded else 'nominal'
            print(f"{mname:10s} {mode:9s} {ok:3d}/{n:<3d} {cf:8d}/{n:<3d} "
                  f"{ac:12.2f} {at:8.1f}s")
            if ok < n:
                all_ok = False
                bad = [(s, r) for s, r in enumerate(res)
                       if r['result'] != 'success']
                for s, r in bad[:4]:
                    print(f"    seed {s}: {r['result']} t={r['time_s']:.0f}s "
                          f"contacts={r['contacts']} backups={r['backups']} "
                          f"script_left={r['script_left']} {r['anomalies'][:3]}")
    return all_ok


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'mc'
    if cmd == 'one':
        m = sys.argv[2]
        seed = int(sys.argv[3]) if len(sys.argv) > 3 else 0
        r = run_once(m, seed, degraded='--deg' in sys.argv, record=True,
                     auto='--auto' in sys.argv)
        print(r['result'], f"{r['time_s']:.1f}s", 'contacts', r['contacts'],
              'backups', r['backups'], 'script_left', r['script_left'],
              r['anomalies'][:5])
        if '--log' in sys.argv:
            for row in r['log']:
                print('  t=%6.1f %-9s->%-9s xy=(%5d,%5d) th=%3d '
                      'F/L/R=%4d/%4d/%4d script=%d' % row)
        if '--plot' in sys.argv:
            out = sys.argv[sys.argv.index('--plot') + 1]
            plot_run(r, out)
            print('wrote', out)
    else:
        args = [a for a in sys.argv[2:] if not a.startswith('--')]
        n = int(args[0]) if args else 25
        monte_carlo(n, auto='--auto' in sys.argv)
