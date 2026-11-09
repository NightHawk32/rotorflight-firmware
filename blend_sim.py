"""
Profile-switch scenario for PR #591 on top of rotorflight-sim's heli_bridge.

Flies the bridge's lockstep loop with a custom pilot that switches PID and rate profile
on AUX3 in a governed hover (continuous adjustments: AUX3 low = profile 1, high =
profile 2). The bridge itself is not modified: its rescue pilot slot is replaced by
this pilot, and the rescue CLI lines it would add are dropped.

    python blend_sim.py <SITL elf> <sim repo> --out run.csv --blend on|off [options]
"""

import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('elf')
    ap.add_argument('simrepo')
    ap.add_argument('--out', required=True)
    ap.add_argument('--blend', choices=('on', 'off'), required=True)
    ap.add_argument('--rpm-a', type=int, default=2100)
    ap.add_argument('--rpm-b', type=int, default=1700)
    ap.add_argument('--yaw-p', type=int, default=80, help='profile 1 yaw P; I, D, F scale with it')
    ap.add_argument('--no-switch', action='store_true', help='hover on profile 1 only (gain sweep)')
    ap.add_argument('--switch-at', type=float, default=26.0)
    ap.add_argument('--back-at', type=float, default=36.0)
    ap.add_argument('--seconds', type=float, default=52.0)
    ap.add_argument('--servos', default='ideal')
    ap.add_argument('--hover-trim', type=float, default=6.0, help='us added to the hover stick to hold height')
    a = ap.parse_args()

    sys.path.insert(0, os.path.join(a.simrepo, 'heli'))
    import heli_bridge as hb

    k = (a.rpm_a / a.rpm_b) ** 2       # gain and pitch a profile tuned for rpm_b needs

    class SwitchPilot:
        """Angle-mode governed hover, collective stick held at hover; AUX3 switches profile."""

        def __init__(self, hover):
            self.hover = hover
            self.t_inverted = self.t_rescue = self.t_end = None    # bridge prints these

        def rc(self, t, heli):
            arm = 1900 if 6.0 <= t < a.seconds - 2.0 else 1000
            thr = 2000 if 8.0 <= t < a.seconds - 2.0 else 1000
            coll = 1500 - 150
            if 18.0 <= t < 20.0:
                coll = 1500 + self.hover + 60                   # lift off to a few metres
            elif 20.0 <= t < a.seconds - 10.0:
                coll = 1500 + self.hover + a.hover_trim          # hover stick, held
            elif t >= a.seconds - 10.0:
                coll = 1500 + self.hover - 25                    # descend and land
            yaw = 1500
            for t0 in (a.switch_at + 1.0, a.back_at + 1.0):     # same small yaw pulse in every run
                if t0 <= t < t0 + 0.3:
                    yaw = 1500 + 60
            aux3 = 1000
            if not a.no_switch and a.switch_at <= t < a.back_at:
                aux3 = 2000
            return thr, coll, arm, 1900, 1500, 1500, yaw, aux3

    hb.RescuePilot = SwitchPilot

    blend = 'ON' if a.blend == 'on' else 'OFF'
    yp = a.yaw_p
    ycfg = lambda s: ['set yaw_p_gain = %d' % round(yp * s), 'set yaw_i_gain = %d' % round(120 * yp / 80 * s),
                      'set yaw_d_gain = %d' % round(10 * yp / 80 * s)]
    extra = [
        'set gov_profile_blend = %s' % blend,
        'set gov_profile_blend_collective = %s' % blend,
        'set gov_profile_blend_time = 30',
        # profile 1: tuned for rpm_a; profile 2: for rpm_b (gains and pitch x (a/b)^2)
        'profile 0', 'set gov_headspeed = %d' % a.rpm_a, *ycfg(1.0),
        'profile 1', 'set gov_headspeed = %d' % a.rpm_b, *ycfg(k),
        'profile 0',
        'rateprofile 0', 'set collective_rc_rate = 100',
        'rateprofile 1', 'set collective_rc_rate = %d' % round(100 * k),
        'rateprofile 0',
        # AUX3 (adjustment channel 2) selects both profiles: 1000 us -> 1, 2000 us -> 2
        'adjfunc 0 2 255 900 2100 2 1000 2000 900 900 0 1 2',
        'adjfunc 1 1 255 900 2100 2 1000 2000 900 900 0 1 2',
    ]

    real_cli = hb.cli

    def cli(port, commands, timeout=3.0):
        keep = [c for c in commands if not (c.startswith('aux 2 53') or c.startswith('set rescue_'))]
        if keep and keep[-1] == 'save':
            keep = keep[:-1] + extra + ['save']
        out = real_cli(port, keep, timeout)     # sets hb.cli.rejected (this wrapper) itself
        with open(a.out + '.cli.txt', 'w') as f:
            f.write(out)
        return out

    hb.cli = cli

    # Firmware debug values (debug_mode PROFILE_BLEND) every RC frame: an MSP_DEBUG request
    # rides along with each MSP_SET_RAW_RC (sent every 20 ms of sim time from t = 0), and the
    # replies are tapped off the MSP socket. The bridge ignores replies it does not know.
    MSP_DEBUG = 254
    real_frame = hb.msp_frame
    sent = []

    def msp_frame(cmd, payload=b''):
        f = real_frame(cmd, payload)
        if cmd == hb.MSP_SET_RAW_RC:
            sent.append(len(sent) * 0.02)
            f += real_frame(MSP_DEBUG)
        return f

    hb.msp_frame = msp_frame
    rx = []
    real_connect = hb.socket.create_connection

    class Tap:
        def __init__(self, s):
            self._s = s

        def recv(self, n):
            d = self._s.recv(n)
            rx.append(d)
            return d

        def __getattr__(self, name):
            return getattr(self._s, name)

    def create_connection(addr, *args, **kw):
        s = real_connect(addr, *args, **kw)
        if addr[1] == 5761:
            del rx[:]                   # keep the last connection: the flight's MSP link
            return Tap(s)
        return s

    hb.socket.create_connection = create_connection
    extra.insert(0, 'set debug_mode = PROFILE_BLEND')
    sys.argv = ['heli_bridge.py', a.elf, '--rescue', '--governor', str(a.rpm_a),
                '--seconds', str(a.seconds), '--servos', a.servos, '--log', a.out]
    try:
        hb.main()
    finally:
        buf = b''.join(rx)
        vals, i = [], 0
        while True:
            i = buf.find(b'$M>', i)
            if i < 0 or len(buf) < i + 5:
                break
            size, cmd = buf[i + 3], buf[i + 4]
            if cmd == MSP_DEBUG and size == 32 and len(buf) >= i + 5 + size:
                vals.append(hb.struct.unpack_from('<8i', buf, i + 5))
            i += 1
        with open(a.out.replace('.csv', '') + '_fw.csv', 'w') as f:
            f.write('t,ratio,mode,headspeed,target,yaw_p,yaw_f,pitch_p,roll_p\n')
            for t, v in zip(sent, vals):
                f.write('%.2f,%s\n' % (t, ','.join(str(x) for x in v)))
        print('firmware debug: %d of %d frames' % (len(vals), len(sent)))


if __name__ == '__main__':
    main()
