#!/usr/bin/env python3
"""Logic tests for the nav layer -- plain python, no ROS, no numpy.

    python3 nav/test_logic.py
"""

import sys

from steering import Steering, HOLD_FRAMES, V_MIN
from goal_sides import GoalSides, frame_verdict, MIN_VOTES
from mission_logic import Mission

FAIL = 0


def check(name, cond):
    global FAIL
    print(("ok   " if cond else "FAIL ") + name)
    if not cond:
        FAIL += 1


def det(cls, conf, cx, h=0.3, cy=0.5, w=0.1):
    return dict(cls=cls, conf=conf, cx=cx, cy=cy, w=w, h=h)


# --- steering ----------------------------------------------------------------
s = Steering()
v, w = s.step(0.0)
check("steering: centred line -> straight, cruise", w == 0.0 and v > V_MIN)
v, w = s.step(0.5)
check("steering: pair right -> steer right, slower", w > 0.2 and v < 0.55)
for _ in range(HOLD_FRAMES):
    v, w = s.step(None)
check("steering: brief loss -> hold last steer", w > 0 and v == V_MIN)
v, w = s.step(None)
check("steering: long loss -> stop and search toward last side",
      v == 0.0 and w > 0)
v, w = s.step(-0.2)
check("steering: reacquire -> drive again", v > 0 and w < 0)

# --- goal_sides: frame filter --------------------------------------------------
check("verdict: clean H|R pair", frame_verdict(
    [det("H", 0.8, 0.3), det("R", 0.8, 0.7)]) == "HR")
check("verdict: mirrored pair", frame_verdict(
    [det("R", 0.8, 0.3), det("H", 0.8, 0.7)]) == "RH")
check("verdict: dual-class phantom deduped", frame_verdict(
    [det("H", 0.8, 0.3), det("R", 0.6, 0.31), det("R", 0.9, 0.7)]) == "HR")
check("verdict: far-range boxes abstain", frame_verdict(
    [det("H", 0.8, 0.3, h=0.1), det("R", 0.8, 0.7, h=0.1)]) is None)
check("verdict: RR abstains (H->R flip guard)", frame_verdict(
    [det("R", 0.8, 0.3), det("R", 0.8, 0.7)]) is None)
check("verdict: edge sliver dropped -> single box abstains", frame_verdict(
    [det("R", 0.9, 0.01), det("H", 0.8, 0.5)]) is None)

# --- goal_sides: voting ---------------------------------------------------------
g = GoalSides()
hr = [det("H", 0.8, 0.3), det("R", 0.8, 0.7)]
rr = [det("R", 0.8, 0.3), det("R", 0.8, 0.7)]
out = None
for i in range(MIN_VOTES - 1):
    out = g.update(hr)
check("vote: not latched before MIN_VOTES", out is None)
g.update(rr)                     # abstaining frame must not reset progress
out = g.update(hr)
check("vote: latched at MIN_VOTES, abstains ignored", out == "HR")
check("vote: stays latched against contradiction",
      g.update([det("R", 0.9, 0.3), det("H", 0.9, 0.7)]) == "HR")

g2 = GoalSides()
for i in range(20):
    g2.update(hr if i % 2 == 0 else
              [det("R", 0.8, 0.3), det("H", 0.8, 0.7)])
check("vote: 50/50 conflict never latches", g2.latched is None)

# --- mission -------------------------------------------------------------------
T = [0.0]
clk = lambda: T[0]
m = Mission(n_balls=2, cfg_side="blue", cfg_goal_sides="HR",
            unknown_side="right", clock=clk)
st = Steering()

cmds = m.step(None, st)
check("mission: SENSE_SIDE holds still",
      m.state == "SENSE_SIDE" and ("drive", (0.0, 0.0)) in cmds)
m.on_team_side("blue")
m.step(None, st)
check("mission: side sensed -> DESCEND", m.state == "DESCEND")
cmds = m.step(0.1, st)
check("mission: DESCEND drives", cmds[0][0] == "drive" and cmds[0][1][0] > 0)

m.on_goal_sides("RH")            # latched on the way down
m.on_arrived()
check("mission: arrived -> WAIT_GOALS", m.state == "WAIT_GOALS")
cmds = m.step(None, st)
check("mission: goals known -> SORT + first feed",
      m.state == "SORT" and ("feed", None) in cmds)

cmds = m.on_ball_color("orange")
check("mission: orange with RH -> flap right, feed next",
      ("flap", "right") in cmds and ("feed", None) in cmds)
cmds = m.on_ball_color("silver")
check("mission: silver with RH -> flap left, last ball -> DONE",
      ("flap", "left") in cmds and m.state == "DONE")

# timeout paths
m2 = Mission(n_balls=1, cfg_side="red", cfg_goal_sides="HR", clock=clk)
T[0] += 6.0
m2.step(None, st)
check("mission: side timeout -> configured side, DESCEND",
      m2.state == "DESCEND" and m2.team_side == "red")
m2.on_arrived()
T[0] += 9.0
cmds = m2.step(None, st)
check("mission: goals timeout -> fallback HR, SORT",
      m2.state == "SORT" and m2.goal_sides == "HR")
T[0] += 7.0                       # ball never classified
cmds = m2.step(None, st)
check("mission: jam watchdog routes unknown to unknown_side and finishes",
      ("flap", "right") in cmds and m2.state == "DONE")

print(f"\n{FAIL} failure(s)")
sys.exit(1 if FAIL else 0)
