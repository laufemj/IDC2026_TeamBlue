"""Dual-lane centreline tracker for the goal approach.  Pure numpy.

The two goals of a pair each have a black tape lane leading into their bore.
The robot drives the CENTRE between them, so that goal 1 ends up on its left
and goal 2 on its right, and the servo flap sorts balls left/right from
there.  So the tracking target is not a line but the midpoint of two lines.

Method, per frame (lower band of the IMX500 preview stream):
  1. binarize "tape-dark" pixels: the tape is near-black on lime -- in the
     venue frames tape luminance is ~0.28 of the mat's, so a relative
     threshold against the band's bright quantile is lighting-independent,
  2. per scanline row, find dark runs of plausible lane width,
  3. cluster runs into lanes, pick the two strongest lanes,
  4. centre = mean of their centres; with one lane visible, offset it by
     half the known lane spacing (sign from which side it is);
  5. smooth over rows, output normalized lateral offset in [-1, 1].

Everything is a ratio of image width, so it works at any preview resolution.
"""

import numpy as np

import field_spec as F

LANE_W_MIN = 0.015          # min dark-run width, fraction of image width
LANE_W_MAX = 0.16           # max (a lane very close is wide)
# Expected lane centre-to-centre distance as a fraction of image width, AT
# THE ROWS WHERE CLUSTERING HAPPENS (bottom quarter of the 0.45 band, i.e.
# image rows 0.89-1.0, ground 0.29-0.35 m ahead).  Derived from the field
# geometry instead of the old eyeballed 0.42 -- which was the mid-band value
# and made the single-lane fallback offset ~35% short.  Depends on
# field_spec CAM_HEIGHT / CAM_TILT_DEG (both TODO-FIELD): one frame of the
# real mount looking at the real lanes verifies it.
PAIR_SPACING = F.lane_pair_image_fraction(0.945)   # ~0.57 at current spec
DARK_RATIO = 0.55           # run is "dark" if below this x band brightness


def track(band_rgb):
    """band_rgb: uint8 (H, W, 3) lower-image band.

    Returns (offset, n_lanes, centres):
       offset   [-1, 1] lateral position of the pair centre (+ = right), or
                None if no lane seen
       n_lanes  0, 1 or 2
       centres  lane centres as fractions of width (diagnostics)
    """
    rgb = band_rgb.astype(np.float32) / 255.0
    g = rgb.mean(axis=2)
    H, W = g.shape

    # --- sanity gate: is this even the field? -----------------------------
    # A steering value computed off tables and chair legs is worse than no
    # value.  The approach band must be mostly lime mat; if it is not, the
    # robot is misaligned or something blocks the view -- report lost.
    mx = rgb.max(axis=2)
    mn = rgb.min(axis=2)
    s = np.where(mx > 0, (mx - mn) / np.maximum(mx, 1e-6), 0)
    greenish = (rgb[..., 1] > rgb[..., 0]) & (rgb[..., 1] > rgb[..., 2]) \
        & (s > 0.25)
    if greenish.mean() < 0.35:
        return None, 0, []

    bright = np.quantile(g, 0.8, axis=1, keepdims=True)   # per-row field level
    dark = g < bright * DARK_RATIO

    # --- cluster only the NEAREST rows ------------------------------------
    # Perspective converges the lanes toward the top of the band; clustering
    # the whole band smears a slanted lane across a wide x range and merges
    # neighbouring lanes.  The bottom quarter is (a) nearly convergence-free
    # and (b) the ground the robot is about to drive on -- the right control
    # point anyway.
    r0 = int(H * 0.75)
    centres_rows = []
    for r in range(r0, H):
        d = np.diff(np.concatenate(([0], dark[r].view(np.int8), [0])))
        starts = np.nonzero(d == 1)[0]
        ends = np.nonzero(d == -1)[0]
        for st, en in zip(starts, ends):
            if LANE_W_MIN * W <= en - st <= LANE_W_MAX * W:
                centres_rows.append(0.5 * (st + en) / W)

    rows_used = H - r0
    if len(centres_rows) < max(3, 0.25 * rows_used):
        return None, 0, []
    order = np.sort(np.array(centres_rows))
    gaps = np.diff(order)
    cuts = np.nonzero(gaps > 0.08)[0]        # locally distinct lanes
    groups = [gr for gr in np.split(order, cuts + 1)
              if len(gr) >= 0.25 * rows_used]
    if not groups:
        return None, 0, []
    lanes = sorted((float(np.median(gr)) for gr in groups))

    if len(lanes) >= 2:
        # the pair bracketing the image centre if one exists, else the two
        # most central lanes
        best = None
        for a, b in zip(lanes[:-1], lanes[1:]):
            mid = 0.5 * (a + b)
            score = (0 if a <= 0.5 <= b else 1, abs(mid - 0.5))
            if best is None or score < best[0]:
                best = (score, a, b)
        _, c1, c2 = best
        centre, n, centres = 0.5 * (c1 + c2), 2, [c1, c2]
    else:
        c1 = lanes[0]
        centre = c1 + (PAIR_SPACING / 2 if c1 < 0.5 else -PAIR_SPACING / 2)
        n, centres = 1, [c1]
    return float(np.clip(2.0 * (centre - 0.5), -1, 1)), n, centres
