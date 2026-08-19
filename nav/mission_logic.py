"""Match state machine.  Pure python, no ROS -- the wrapper feeds it events
and executes the commands it returns.

States (the happy path, top to bottom):

  SENSE_SIDE   wait for /team_side (latched one-shot).  Timeout -> use the
               pre-configured side (it is announced >= 5 min before the
               match [RB p8-f]; sensing is a config-error guard, not the
               source of truth).
  DESCEND      steer the lane-pair centre down the slope (steering.py).
               /goal_sides latches en route -- the goals are in view the
               whole way down.
  WAIT_GOALS   arrived but goal identity not latched yet: hold still and
               keep watching.  Timeout -> pre-configured fallback (a coin
               flip is still 50%; a stall is 0%).
  SORT         for each ball: point the flap at the ball's goal, then feed
               exactly one ball.  orange -> the HUMAN figure's goal,
               silver -> the ROBOT figure's goal [RB p6].  A ball the chute
               camera can't classify goes to the LOW-RISK side (param):
               a wrong ball in a goal breaks that goal's all-correct DOUBLE,
               so default routes unknowns to the goal we stuff least.
  DONE         stop everything.  (Return-to-Start-A tie-break run is out of
               scope until odometry-by-timing is calibrated -- the slope
               makes blind reversing risky.)

Commands returned by step()/on_*: list of (verb, arg) tuples:
  ("drive", (v, w))   normalized duties for drive_node
  ("flap", "left"|"right")
  ("feed", None)      release exactly one ball
  ("done", None)
"""

import time

SIDE_TIMEOUT_S = 5.0        # SENSE_SIDE patience
GOALS_TIMEOUT_S = 8.0       # WAIT_GOALS patience
FLAP_SETTLE_S = 0.4         # servo travel time before feeding
FEED_PERIOD_S = 1.6         # min spacing between feeds (ball must clear)


class Mission:
    def __init__(self, n_balls, cfg_side, cfg_goal_sides, unknown_side="right",
                 clock=time.monotonic):
        self.n_balls = n_balls            # preload 1+1 + cyber advantage
        self.cfg_side = cfg_side          # "red"|"blue", from the 5-min draw
        self.cfg_goal_sides = cfg_goal_sides   # "HR"|"RH" fallback
        self.unknown_side = unknown_side
        self.clock = clock

        self.state = "SENSE_SIDE"
        self.t_state = clock()
        self.team_side = None
        self.goal_sides = None
        self.balls_fed = 0
        self.t_fed = 0.0
        self.awaiting_ball = False

    # --- event inputs ------------------------------------------------------
    def on_team_side(self, side):
        self.team_side = side

    def on_goal_sides(self, verdict):
        self.goal_sides = verdict

    def on_arrived(self):
        if self.state == "DESCEND":
            self._go("WAIT_GOALS")

    def on_ball_color(self, color):
        """Chute camera event for the ball we just fed."""
        if self.state != "SORT" or not self.awaiting_ball:
            return []
        self.awaiting_ball = False
        return self._route_and_feed_next(color)

    # --- per-frame tick ----------------------------------------------------
    def step(self, line_offset, steering):
        """line_offset: float or None; steering: steering.Steering.
        Returns the command list for this tick."""
        now = self.clock()
        dt = now - self.t_state

        if self.state == "SENSE_SIDE":
            if self.team_side or dt > SIDE_TIMEOUT_S:
                if not self.team_side:
                    self.team_side = self.cfg_side
                elif self.team_side != self.cfg_side:
                    # sensed != configured: trust the SENSED tape, we may be
                    # set up for the wrong half
                    pass
                self._go("DESCEND")
            return [("drive", (0.0, 0.0))]

        if self.state == "DESCEND":
            return [("drive", steering.step(line_offset))]

        if self.state == "WAIT_GOALS":
            if self.goal_sides is None and dt > GOALS_TIMEOUT_S:
                self.goal_sides = self.cfg_goal_sides
            if self.goal_sides is not None:
                self._go("SORT")
                return [("drive", (0.0, 0.0))] + self._feed_first()
            return [("drive", (0.0, 0.0))]

        if self.state == "SORT":
            # watchdog: ball fed but never classified (jam or camera miss)
            if self.awaiting_ball and now - self.t_fed > 4 * FEED_PERIOD_S:
                self.awaiting_ball = False
                return self._route_and_feed_next(None)
            return []

        return [("done", None)]

    # --- internals ----------------------------------------------------------
    def _go(self, state):
        self.state = state
        self.t_state = self.clock()

    def _flap_side(self, color):
        """left/right for this ball.  goal_sides 'HR' = human goal on the
        LEFT; orange balls belong to the human figure's goal."""
        if color == "orange":
            return "left" if self.goal_sides == "HR" else "right"
        if color == "silver":
            return "right" if self.goal_sides == "HR" else "left"
        return self.unknown_side

    def _feed_first(self):
        if self.n_balls <= 0:
            self._go("DONE")
            return [("done", None)]
        self.balls_fed = 1
        self.awaiting_ball = True
        self.t_fed = self.clock()
        return [("feed", None)]

    def _route_and_feed_next(self, color):
        cmds = [("flap", self._flap_side(color))]
        if self.balls_fed >= self.n_balls:
            self._go("DONE")
            return cmds + [("done", None)]
        self.balls_fed += 1
        self.awaiting_ball = True
        self.t_fed = self.clock()
        cmds.append(("feed", None))
        return cmds
