"""PNG plots of the PR #591 SITL runs (blend OFF vs ON) and of the yaw P/I mixing fix."""

import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt   # noqa: E402

RUN_DIR, OUT_DIR = sys.argv[1], sys.argv[2]
os.makedirs(OUT_DIR, exist_ok=True)

# reference palette, light mode
SURFACE, INK, INK2, MUTED, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984', '#e6e5e1'
ON, OFF = '#2a78d6', '#eb6834'
RPM_A, RPM_B = 2100, 1700
SWITCH, BACK = 26.0, 36.0
T0, T1 = 23.0, 42.0

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'font.family': 'DejaVu Sans', 'font.size': 10, 'text.color': INK,
    'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'axes.titlecolor': INK,
    'axes.titlesize': 11, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left',
    'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'xtick.color': INK2, 'ytick.color': INK2, 'xtick.major.size': 0, 'ytick.major.size': 0,
    'lines.linewidth': 2.0, 'lines.solid_capstyle': 'round', 'legend.frameon': False,
})


def load(name):
    with open(os.path.join(RUN_DIR, name)) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def window(rows):
    return [r for r in rows if T0 <= r['t'] <= T1]


sim = {b: window(load('run_%s.csv' % b)) for b in ('off', 'on')}
fw = {b: window(load('run_%s_fw.csv' % b)) for b in ('off', 'on')}


def col(rows, key, f=lambda v, r: v):
    return [r['t'] for r in rows], [f(r[key], r) for r in rows]


def mark_switches(ax, label=False):
    for t, txt in ((SWITCH, 'switch to profile 2\n(%d rpm)' % RPM_B), (BACK, 'back to profile 1\n(%d rpm)' % RPM_A)):
        ax.axvline(t, color=MUTED, lw=1, ls=(0, (3, 3)), zorder=0)
        if label:
            ax.text(t + 0.15, 1.0, txt, transform=ax.get_xaxis_transform(), va='top', ha='left',
                    color=INK2, fontsize=8.5)


def end_label(ax, x, y, text, color, dy=0):
    ax.annotate(text, (x[-1], y[-1]), xytext=(6, dy), textcoords='offset points',
                color=INK, fontsize=9, va='center', ha='left')
    ax.plot([x[-1]], [y[-1]], 'o', ms=4, color=color, mec=SURFACE, mew=1.5)


def series(ax, b, x, y, **kw):
    color = ON if b == 'on' else OFF
    ax.plot(x, y, color=color, label='blend %s' % b.upper(), zorder=3 if b == 'on' else 2, **kw)


def finish(fig, name, note):
    fig.text(0.01, 0.006, note, color=MUTED, fontsize=8, ha='left', va='bottom', linespacing=1.5)
    fig.savefig(os.path.join(OUT_DIR, name), dpi=160)
    plt.close(fig)
    print('wrote', name)


NOTE = ('rotorflight-sim (Blade 360 model, ELECTRIC governor), SITL built from PR #591 merged into feature/sitl-build.\n'
        'Profile 1: %d rpm, yaw P 80. Profile 2: %d rpm, yaw P, I, D x(%d/%d)^2 (P 122). '
        'One switch changes PID and rate profile together.' % (RPM_A, RPM_B, RPM_A, RPM_B))

# ---- 1: PID gains on the downshift and upshift ----------------------------------------
fig, axes = plt.subplots(3, 1, figsize=(10, 9.2), sharex=True,
                         gridspec_kw={'height_ratios': [1, 1, 1.25], 'hspace': 0.42})
fig.suptitle('In-flight PID profile switch: gains follow the headspeed', x=0.01, ha='left',
             fontsize=14, fontweight='bold', color=INK)
fig.text(0.01, 0.935, 'Same flight twice, blend OFF and ON. Values read live from the firmware (debug mode PROFILE_BLEND).',
         color=INK2, fontsize=10)

ax = axes[0]
for b in ('off', 'on'):
    series(ax, b, *col(fw[b], 'headspeed'))
x, y = col(fw['on'], 'target')
ax.plot(x, y, color=INK2, lw=1.2, ls=(0, (5, 3)), label='governor target', zorder=1)
ax.set_title('Headspeed')
ax.set_ylabel('rpm')
ax.set_ylim(1450, 2350)
mark_switches(ax, label=True)
ax.legend(loc='center right', fontsize=9)

ax = axes[1]
for b in ('off', 'on'):
    x, y = col(fw[b], 'yaw_p')
    series(ax, b, x, y)
ax.set_title('Yaw P gain in use')
ax.set_ylabel('yaw P')
ax.set_ylim(60, 140)
mark_switches(ax)
ax.annotate('steps at once', (SWITCH, 122), xytext=(SWITCH - 2.6, 131), color=INK2, fontsize=9,
            arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
ax.annotate('follows the headspeed', (28.0, 103), xytext=(28.9, 82), color=INK2, fontsize=9,
            arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))

ax = axes[2]
# yaw authority goes with rpm^2: the loop gain the tail sees, relative to profile 1 at its headspeed
loop = lambda v, r: r['yaw_p'] / 80.0 * (r['headspeed'] / RPM_A) ** 2
peak = {}
for b in ('off', 'on'):
    x, y = col(fw[b], 'yaw_p', loop)
    series(ax, b, x, y)
    peak[b] = (max(y), min(y))
ax.axhline(1.0, color=INK2, lw=1, zorder=1)
ax.text(T0 + 0.1, 1.02, 'as tuned', color=INK2, fontsize=8.5, va='bottom')
ax.set_title('Yaw loop gain: yaw P x (headspeed / %d)^2, relative to the tune' % RPM_A)
ax.set_ylabel('x tuned loop gain')
ax.set_xlabel('time (s)')
ax.set_ylim(0.5, 1.7)
mark_switches(ax)
ax.annotate('+%.0f%% while the rotor slows' % ((peak['off'][0] - 1) * 100), (SWITCH + 0.05, peak['off'][0]),
            xytext=(SWITCH + 1.2, 1.6), color=INK, fontsize=9,
            arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
ax.annotate('%.0f%% while the rotor speeds up' % ((peak['off'][1] - 1) * 100), (BACK + 0.05, peak['off'][1]),
            xytext=(BACK + 1.6, 0.58), color=INK, fontsize=9,
            arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
on_dev = max(abs(loop(r['yaw_p'], r) - 1) for r in fw['on'] if r['ratio'] < 1000 and r['t'] >= SWITCH)
ax.text(SWITCH + 3.4, 1.12, 'blend ON: within %.1f%% while blending' % (on_dev * 100), color=INK, fontsize=9)
low = min(fw['on'], key=lambda r: r['headspeed'])
ax.annotate('governor undershoots to %.0f rpm\nafter the blend (both runs)' % low['headspeed'],
            (low['t'], loop(low['yaw_p'], low)), xytext=(low['t'] + 0.6, 0.6), color=INK2, fontsize=8.5,
            arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
ax.legend(loc='upper left', fontsize=9)
finish(fig, '1_pid_gain_blend.png', NOTE)

# ---- 2: collective -------------------------------------------------------------------------
NOTE2 = ('rotorflight-sim (Blade 360 model, ELECTRIC governor), SITL built from PR #591 merged into feature/sitl-build.\n'
         'Collective stick held at hover throughout. Rate profile 2: collective_rc_rate x(%d/%d)^2 = 153 '
         '(a bit much for this model at %d rpm, so both runs climb slowly there).\n'
         'The small steps in the blended pitch are the SITL\'s 1 us servo pulse resolution.' % (RPM_A, RPM_B, RPM_B))
fig, axes = plt.subplots(3, 1, figsize=(10, 9.2), sharex=True,
                         gridspec_kw={'height_ratios': [1.1, 1, 1], 'hspace': 0.42})
fig.suptitle('In-flight rate profile switch: collective pitch', x=0.01, ha='left',
             fontsize=14, fontweight='bold', color=INK)
fig.text(0.01, 0.935, 'Same flight twice, blend OFF and ON, same stick. Collective is an angle, so a new curve moves the blades.',
         color=INK2, fontsize=10)

ax = axes[0]
for b in ('off', 'on'):
    series(ax, b, *col(sim[b], 'mix_coll'))
ax.set_title('Collective pitch command (mixer)')
ax.set_ylabel('collective (mixer units)')
mark_switches(ax, label=True)
ax.legend(loc='lower left', fontsize=9)

ax = axes[1]
for b in ('off', 'on'):
    series(ax, b, *col(sim[b], 'climb_ms'))
ax.axhline(0, color=INK2, lw=1, zorder=1)
ax.set_title('Climb rate')
ax.set_ylabel('m/s')
mark_switches(ax)
for t0, t1, pick, dy in ((SWITCH, SWITCH + 3, max, 10), (BACK, BACK + 4, min, -16)):
    pk = {b: pick((r for r in sim[b] if t0 <= r['t'] <= t1), key=lambda r: r['climb_ms']) for b in ('off', 'on')}
    ax.annotate('OFF %+.1f m/s, ON %+.1f m/s' % (pk['off']['climb_ms'], pk['on']['climb_ms']),
                (pk['off']['t'], pk['off']['climb_ms']), xytext=(14, dy), textcoords='offset points',
                color=INK, fontsize=9, arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))

ax = axes[2]
for b in ('off', 'on'):
    series(ax, b, *col(sim[b], 'alt_m'))
ax.set_title('Height')
ax.set_ylabel('m')
ax.set_xlabel('time (s)')
mark_switches(ax)
finish(fig, '2_collective_blend.png', NOTE2)

# ---- 3: yaw P/I mixing fix (analytic, the formulas in pidBlendMix) -----------------------
fig, axes = plt.subplots(1, 2, figsize=(10, 4.6), sharey=True, gridspec_kw={'wspace': 0.08})
fig.suptitle('Yaw P during a blend, before and after the stop-gain fix', x=0.01, ha='left',
             fontsize=14, fontweight='bold', color=INK)
fig.text(0.01, 0.87, 'Effective yaw P = Kp x master gain x stop gain. Two profiles where it is the same at both ends.',
         color=INK2, fontsize=10)
fig.texts[-1].set_text('Effective yaw P = Kp x master gain x stop gain, relative to the old profile. '
                       'Gray lines: the two profiles\' values.')
cases = [('A: same yaw P in both profiles\nKp x master 1.0 -> 2.0, stop gain 1.0 -> 0.5', (1.0, 1.0), (2.0, 0.5)),
         ('B: #586-like 1.8x gain step\nKp x master 1.0 -> 1.8, stop gain 1.0 -> 0.6', (1.0, 1.0), (1.8, 0.6))]
r = [i / 100 for i in range(101)]
for ax, (title, (ps, ss), (pt, st)) in zip(axes, cases):
    base = ps * ss
    old = [(ps + (pt - ps) * x) * (ss + (st - ss) * x) / base for x in r]
    new = []
    for x in r:
        pm = ps + (pt - ps) * x
        s = (ps * ss + (pt * st - ps * ss) * x) / pm
        new.append(pm * s / base)
    ax.plot(r, old, color=OFF, label='before the fix')
    ax.plot(r, new, color=ON, label='after the fix (5455fe0)')
    hi = max(1.0, pt * st / base)
    for v in {1.0, pt * st / base}:
        ax.axhline(v, color=INK2, lw=1, zorder=1)
    i = max(range(101), key=lambda k: old[k])
    ax.annotate('%+.1f%% past both profiles' % ((old[i] / hi - 1) * 100), (r[i], old[i]), xytext=(0, 8),
                textcoords='offset points', ha='center', color=INK, fontsize=9)
    ax.set_title(title, fontsize=9.5, fontweight='normal', color=INK)
    ax.set_xlabel('blend progress')
    ax.set_xlim(0, 1)
axes[0].set_ylabel('effective yaw P (relative)')
axes[0].set_ylim(0.95, 1.2)
axes[1].legend(loc='lower right', fontsize=9)
fig.subplots_adjust(top=0.74, bottom=0.18)
finish(fig, '3_yaw_stop_gain_fix.png',
       'Analytic, from the mixing formulas in pidBlendMix(). The same holds for yaw I with its own stop gains.')
