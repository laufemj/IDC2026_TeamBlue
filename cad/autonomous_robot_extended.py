# Extended-arms pose of the IDC2026 autonomous robot (post-start collecting
# configuration). Same model as autonomous_robot.py with ARM_EXT = 110 mm.

import autonomous_robot as m

m.ARM_EXT = 110.0
m.FLAP_ANG = 0.0


def gen_step():
    return m.gen_step()
