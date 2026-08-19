"""Steering controller for the goal-lane descent.  Pure python, no ROS.

Input : the lane pair-centre offset from /line_offset ([-1, 1], + = the pair
        centre is to the robot's right, NaN/None = lanes lost).
Output: (v, w) in NORMALIZED units -- v forward duty [-1, 1], w turn duty
        [-1, 1] (+ = turn right).  drive_node mixes them into wheel PWM.
        No encoders exist, so real m/s is meaningless here; duties are
        calibrated on the physical robot (field_spec TODO-FIELD family).

Behaviour:
  * PD on the offset, with the derivative smoothed over a couple frames.
  * Forward speed backs off as |offset| grows (don't rush a bad line).
  * Lost lanes: keep the last steer for HOLD_FRAMES at reduced speed (the
    tape often drops out for a few frames under glare), then stop forward
    motion and rotate toward the side the pair was last seen -- the pair is
    wide (0.57 of image width near-field), so a small rotation refinds it.
"""

KP = 0.9
KD = 0.35
V_CRUISE = 0.55         # forward duty on a clean line
V_MIN = 0.25            # forward duty at worst in-band offset
HOLD_FRAMES = 6         # keep-last-steer grace after losing the lanes
SEARCH_W = 0.35         # rotation duty while searching
D_SMOOTH = 0.6          # EMA factor for the derivative term


class Steering:
    def __init__(self):
        self.prev = None
        self.d_ema = 0.0
        self.lost = 0
        self.last_w = 0.0
        self.last_seen_side = 0.0   # sign of the last valid offset

    def step(self, offset):
        """offset: float in [-1, 1], or None when lanes are lost.
        Returns (v, w)."""
        if offset is None:
            self.lost += 1
            if self.lost <= HOLD_FRAMES:
                return V_MIN, self.last_w
            # rotate toward where the pair went; if it vanished dead-centre
            # (e.g. we overran it), search in the last steering direction
            side = self.last_seen_side or (1.0 if self.last_w >= 0 else -1.0)
            return 0.0, SEARCH_W * side

        if self.lost:                       # reacquired: reset derivative
            self.prev, self.d_ema = None, 0.0
        self.lost = 0
        if offset != 0.0:
            self.last_seen_side = 1.0 if offset > 0 else -1.0

        d = 0.0 if self.prev is None else offset - self.prev
        self.d_ema = D_SMOOTH * self.d_ema + (1 - D_SMOOTH) * d
        self.prev = offset

        w = max(-1.0, min(1.0, KP * offset + KD * self.d_ema))
        v = V_CRUISE - (V_CRUISE - V_MIN) * min(1.0, abs(offset))
        self.last_w = w
        return v, w
