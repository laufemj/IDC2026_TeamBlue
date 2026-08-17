# IDC2026 autonomous ball-sorter robot — rev B CAD
#
# Datum: floor point midway between the two wheel contact patches.
# +X forward (ball exit / AI-camera boresight), +Y left, +Z up. Units mm.
#
# Rev B: the FIELD is sloped, so the robot carries no internal ramp. Balls
# stay on the ground the whole way: ground-level funnel arms at the rear
# guide them into a ground-level corridor that runs under a raised deck
# (z=70) to the front. Gate and steering flap hang down through the deck;
# the color camera looks down at the queued ball through a deck window.
#
# Rule constraints (Contest Rule Book v1.01):
#   - autonomous robot must fit a 500 mm cube at start  -> ARM_EXT = 0 pose
#   - kit parts: Pi 5, Camera Module 3, AI camera RPI-SC1174, 3x MG996R,
#     Bosch motors, 2x MDD3A, 2 wheels (o173 x 14.5), MDF boards
#
# Extension system (from sample STL concept): one MG996R drives a central
# pinion meshing two opposed transverse racks above the rear deck; each rack
# carries one ground-level funnel arm through a drop carrier outside the deck
# edge. Gear teeth and rack guides omitted at rev B (pitch-line envelopes).

import math
from build123d import *
from cadpy.assembly import AssemblyHelper

# ---------------------------------------------------------------- parameters
BALL_D = 60.0
CH_W = BALL_D + 6.0          # corridor inner width
PT = 5.5                     # MDF plate thickness

WHEEL_D, WHEEL_W, TRACK = 173.0, 14.5, 242.0   # kit wheel, user-measured
AXLE_Z = WHEEL_D / 2

MOT_D = 42.0                 # Bosch motor body, measured from Bosch_motor_v8.stl
MOT_SHAFT_Z = 15.5           # shaft axis height above lug plate underside

DECK_Z = 70.0                # deck underside; ball headroom = DECK_Z - 4
DECK_TOP = DECK_Z + PT
CH_REAR_X, CH_FRONT_X = -200.0, 165.0
CH_HALF_W = 110.0

WALL_Z0 = 4.0                # ground clearance under corridor/funnel walls
THROAT_X = -113.0            # funnel -> corridor transition
FUN_Y_OUT = 115.0            # funnel wall plan outer y at rear edge
WALL_YC = CH_W / 2 + PT / 2  # corridor wall centerline

ARM_EXT = 0.0                # arm extension stroke; 0 = stowed (start-legal)
ARM_L, ARM_H = 140.0, 60.0
ARM_TOP_Z = 66.0             # arm plate z 6..66 — sticks to the ground
PINION_D, RACK_L = 30.0, 240.0
PINION_X = -166.0
BRIDGE_Z = 95.0              # underside of extension bridge plate
RACK_Z = BRIDGE_Z - 8.0      # rack / pinion mid-plane
ATT_Y = 114.0                # carrier attach station on both racks

GATE_X = -20.0
GATE_PIV_Z = 90.0
FLAP_X = 35.0                # steering flap pivot
FLAP_ANG = 20.0              # steering pose (deg from +X, about Z)

MAST_X0, MAST_X1 = 80.0, 95.0
MAST_YC, MAST_W = 60.0, 15.0
MAST_TOP = 330.0
CAM1_Z, CAM1_PITCH = 300.0, 20.0   # AI camera lens height / down-pitch

SHROUD_X0, SHROUD_X1 = -75.0, -25.0
SHROUD_TOP = 170.0
CM3_POS = (-50.0, 0.0, 155.0)

BAT_L, BAT_W, BAT_H = 138.0, 46.0, 38.0    # Zee LiPo 3S 5200


def bosch_motor():
    """Bosch gearmotor from Bosch_motor_v8.stl measurements.
    Local frame: 9x9 through-shaft along Y at origin (protrudes 10 mm per
    side); lug plate underside at z=-15.5; body cylinder along +X."""
    head = Pos(3.5, 0, 5) * Box(45, 41, 41)
    body = Pos(63.5, 0, 16.5) * Rot(Y=90) * Cylinder(21, 75)
    lug = Pos(-2.5, 0, -13) * Box(43, 43, 5)
    for hx, hy in ((13.7, 16.0), (-18.4, -16.0)):
        lug -= Pos(hx, hy, -13) * Cylinder(2.1, 8)
    shaft = Box(9, 61, 9)
    return head + body + lug + shaft


def mg996r():
    """MG996R envelope, local origin = output shaft axis at flange top, shaft +Z"""
    sh_off = 10.0
    body = Pos(sh_off - 40.7 / 2, 0, -15.5) * Box(40.7, 19.7, 31.0)
    flange = Pos(sh_off - 40.7 / 2, 0, 1.4) * Box(54.5, 19.7, 2.8)
    cap = Pos(0, 0, 5.3) * Cylinder(6.5, 5.0)
    shaft = Pos(0, 0, 9.8) * Cylinder(3.0, 4.0)
    return body + flange + cap + shaft


def place(shape, cx, cy, cz):
    bb = shape.bounding_box()
    c = bb.center()
    return Pos(cx - c.X, cy - c.Y, cz - c.Z) * shape


def gen_step():
    asm = AssemblyHelper("autonomous_robot")

    # ---------------------------------------------------------------- deck
    deck = Pos((CH_FRONT_X + CH_REAR_X) / 2, 0, DECK_Z + PT / 2) * Box(
        CH_FRONT_X - CH_REAR_X, 2 * CH_HALF_W, PT)
    # openings for the motor bodies poking up through the deck
    for sy in (1, -1):
        deck -= Pos(-38.5, sy * 88, DECK_Z + PT / 2) * Box(131, 46, 30)
    # gate flap slot
    deck -= Pos(-17, 0, DECK_Z + PT / 2) * Box(16, 68, 30)
    # color-camera window over the queued ball
    deck -= Pos(-50, 0, DECK_Z + PT / 2) * Box(40, 50, 30)
    # steering-flap shaft hole
    deck -= Pos(FLAP_X, 0, DECK_Z + PT / 2) * Cylinder(4, 30)
    # rear corner cuts so the arm carriers can drop past the deck edge
    for sy in (1, -1):
        deck -= Pos(-170, sy * 110, DECK_Z + PT / 2) * Box(70, 10.2, 30)
    asm.add(deck, "deck_plate", color=Color(0.82, 0.71, 0.55))

    # support legs (deck -> ground, 1 mm float; robot rests on the wheels)
    for sx, sy, tag in ((-80, 55, "rear_left"), (-80, -55, "rear_right"),
                        (145, 55, "front_left"), (145, -55, "front_right")):
        leg = Pos(sx, sy, (DECK_Z + 1) / 2 + 0.5) * Box(30, 30, DECK_Z - 1)
        asm.add(leg, f"deck_leg_{tag}", color=Color(0.6, 0.6, 0.62))

    # ----------------------------------------------------------- drivetrain
    # wheels couple directly onto the Bosch 9x9 through-shaft; motor lug
    # plates bolt to the deck underside (1 mm shim), bodies pass the cutouts
    mot_cy = TRACK / 2 - 30.5
    for sy, tag in ((1, "left"), (-1, "right")):
        wheel = Pos(0, sy * TRACK / 2, AXLE_Z) * Rot(X=90) * Cylinder(
            WHEEL_D / 2, WHEEL_W)
        asm.add(wheel, f"wheel_{tag}", color=Color(0.15, 0.15, 0.15))
        mot = Pos(0, sy * mot_cy, AXLE_Z) * Rot(Z=180) * bosch_motor()
        asm.add(mot, f"bosch_motor_{tag}", color=Color(0.75, 0.2, 0.2))

    # -------------------------------------------- ground-level ball guides
    for sy, tag in ((1, "left"), (-1, "right")):
        w = Pos((CH_FRONT_X + THROAT_X) / 2, sy * WALL_YC,
                (WALL_Z0 + DECK_Z) / 2) * Box(
            CH_FRONT_X - THROAT_X, PT, DECK_Z - WALL_Z0)
        asm.add(w, f"corridor_wall_{tag}", color=Color(0.82, 0.71, 0.55))

    fl = math.hypot(THROAT_X - CH_REAR_X, FUN_Y_OUT - WALL_YC)
    fang = math.degrees(math.atan2(WALL_YC - FUN_Y_OUT, THROAT_X - CH_REAR_X))
    for sy, tag in ((1, "left"), (-1, "right")):
        mid = ((CH_REAR_X + THROAT_X) / 2, sy * (FUN_Y_OUT + WALL_YC) / 2)
        w = Pos(mid[0], mid[1], (WALL_Z0 + DECK_Z) / 2) * Rot(Z=sy * fang) \
            * Box(fl, PT, DECK_Z - WALL_Z0)
        # clearance notch at the rear-outer top corner for the arm carrier
        w -= Pos(-180, sy * 114, 68) * Box(46, 18, 10)
        asm.add(w, f"funnel_wall_{tag}", color=Color(0.82, 0.71, 0.55))

    # ------------------------------------------- arm extension (one servo)
    bridge = Pos(-175, 0, BRIDGE_Z + PT / 2) * Box(55, 2 * FUN_Y_OUT, PT)
    asm.add(bridge, "extension_bridge", color=Color(0.6, 0.6, 0.62))
    for sy in (1, -1):
        h = BRIDGE_Z - DECK_TOP
        leg = Pos(-175, sy * 95, DECK_TOP + h / 2) * Box(12, 12, h)
        asm.add(leg, f"bridge_leg_{'left' if sy > 0 else 'right'}",
                color=Color(0.6, 0.6, 0.62))
    servo_ext = Pos(PINION_X, 0, BRIDGE_Z + PT) * Rot(Z=90) * Rot(X=180) * mg996r()
    asm.add(servo_ext, "servo_arm_extension", color=Color(0.2, 0.3, 0.8))
    pinion = Pos(PINION_X, 0, RACK_Z) * Cylinder(PINION_D / 2, 10)
    asm.add(pinion, "extension_pinion", color=Color(0.9, 0.75, 0.2))

    # wall plan geometry shared by racks and arms
    dx = (THROAT_X - CH_REAR_X) / fl          # wall direction, rear -> throat
    dy = (WALL_YC - FUN_Y_OUT) / fl
    nx, ny = -dy, dx                          # outboard normal (left side)
    arm_off = 18.0
    fe = (THROAT_X + nx * arm_off, WALL_YC + ny * arm_off)   # arm front end

    for sx, sy, tag in ((1, 1, "left"), (-1, -1, "right")):
        rx = PINION_X + sx * (PINION_D / 2 + 6)
        rack = Pos(rx, sy * ARM_EXT, RACK_Z) * Box(12, RACK_L, 10)
        asm.add(rack, f"rack_{tag}", color=Color(0.9, 0.75, 0.2))
        carrier = Pos(rx, sy * (ATT_Y + ARM_EXT), (RACK_Z + 5 + ARM_TOP_Z) / 2) \
            * Box(12, 12, RACK_Z + 5 - ARM_TOP_Z)
        asm.add(carrier, f"arm_carrier_{tag}", color=Color(0.6, 0.6, 0.62))
        t = (ATT_Y - fe[1]) / -dy             # arm x at the carrier station
        ax = fe[0] - dx * t
        link_l = abs(rx - ax) + 14
        link = Pos((rx + ax) / 2, sy * (ATT_Y + ARM_EXT), ARM_TOP_Z + 3) * Box(
            link_l, 12, 10)
        asm.add(link, f"arm_link_{tag}", color=Color(0.6, 0.6, 0.62))
        mid = (fe[0] - dx * ARM_L / 2, sy * (fe[1] - dy * ARM_L / 2 + ARM_EXT))
        arm = Pos(mid[0], mid[1], ARM_TOP_Z - ARM_H / 2) * Rot(Z=sy * fang) \
            * Box(ARM_L, PT, ARM_H)
        asm.add(arm, f"extension_arm_{tag}", color=Color(0.3, 0.65, 0.9))

    # ------------------------------------------------------- metering gate
    bkt = Pos(GATE_X, -(WALL_YC + 12), DECK_TOP + PT / 2) * Box(55, 40, PT)
    asm.add(bkt, "gate_servo_bracket", color=Color(0.6, 0.6, 0.62))
    servo_gate = Pos(GATE_X, -(CH_W / 2 + 6), GATE_PIV_Z) * Rot(X=-90) * mg996r()
    asm.add(servo_gate, "servo_gate_xz", color=Color(0.2, 0.3, 0.8))
    # closed pose: hangs down through the deck slot to ~12 mm off the floor
    gate = Pos(GATE_X, 0, GATE_PIV_Z) * Rot(Y=85) * Pos(39, 0, 0) * Box(
        78, 54, 4)
    asm.add(gate, "gate_flap", color=Color(0.9, 0.4, 0.1))

    # --------------------------------------------- color window (camera 2)
    sh_cx = (SHROUD_X0 + SHROUD_X1) / 2
    shroud = Pos(sh_cx, 0, (DECK_TOP + SHROUD_TOP) / 2) * Box(
        SHROUD_X1 - SHROUD_X0, 80, SHROUD_TOP - DECK_TOP)
    shroud -= Pos(sh_cx, 0, (DECK_TOP + SHROUD_TOP) / 2 - PT) * Box(
        SHROUD_X1 - SHROUD_X0 - 2 * PT, 80 - 2 * PT, SHROUD_TOP - DECK_TOP)
    # notch for the gate servo + bracket passing the shroud right wall
    shroud -= Pos(-36, -37, 88) * Box(46, 24, 34)
    asm.add(shroud, "color_shroud", color=Color(0.35, 0.35, 0.38))
    cm3 = import_step("parts/raspberry_pi_camera_module_3_standard.step")
    ball_cx = GATE_X - 34.0              # queued ball center resting at gate
    aim = math.degrees(math.atan2(ball_cx - CM3_POS[0],
                                  CM3_POS[2] - BALL_D / 2))
    cm3 = Rot(Y=180 - aim) * place(cm3, 0, 0, 0)
    asm.add(Pos(*CM3_POS) * cm3, "camera2_cm3_color", color=Color(0.1, 0.5, 0.1))

    # ------------------------------------------------------ steering flap
    servo_flap = Pos(FLAP_X, 0, DECK_TOP + 14) * Rot(X=180) * mg996r()
    asm.add(servo_flap, "servo_flap_xy", color=Color(0.2, 0.3, 0.8))
    rod = Pos(FLAP_X, 0, (ARM_TOP_Z + DECK_TOP + 14) / 2) * Cylinder(
        3, DECK_TOP + 14 - ARM_TOP_Z)
    asm.add(rod, "flap_drive_rod", color=Color(0.6, 0.6, 0.62))
    flap = Pos(FLAP_X, 0, 37) * Rot(Z=FLAP_ANG) * Pos(35, 0, 0) * Box(70, 4, 58)
    asm.add(flap, "steering_flap", color=Color(0.9, 0.4, 0.1))

    # ------------------------------------------------- mast + camera 1 + Pi
    for sy, tag in ((1, "left"), (-1, "right")):
        leg = Pos((MAST_X0 + MAST_X1) / 2, sy * MAST_YC,
                  (MAST_TOP + DECK_TOP) / 2) * Box(
            MAST_W, MAST_W, MAST_TOP - DECK_TOP)
        asm.add(leg, f"mast_leg_{tag}", color=Color(0.6, 0.6, 0.62))
        b0, b1 = (84.0, 170.0), (30.0, DECK_TOP)
        br_l = math.hypot(b0[0] - b1[0], b0[1] - b1[1])
        br_a = math.degrees(math.atan2(b0[1] - b1[1], b0[0] - b1[0]))
        brace = Pos((b0[0] + b1[0]) / 2, sy * MAST_YC,
                    (b0[1] + b1[1]) / 2) * Rot(Y=-br_a) * Box(br_l, 12, 12)
        asm.add(brace, f"mast_brace_{tag}", color=Color(0.6, 0.6, 0.62))
    cross = Pos((MAST_X0 + MAST_X1) / 2, 0, MAST_TOP - 7.5) * Box(
        MAST_W, 2 * MAST_YC + MAST_W, MAST_W)
    asm.add(cross, "mast_crossbar", color=Color(0.6, 0.6, 0.62))
    cam_plate = Pos(MAST_X1 + 3, 0, CAM1_Z) * Rot(Y=CAM1_PITCH) * Box(PT, 40, 45)
    asm.add(cam_plate, "camera1_plate", color=Color(0.6, 0.6, 0.62))
    ai_body = Rot(Y=CAM1_PITCH) * (Box(12, 25, 24) + Pos(8, 0, 0) * Rot(Y=90)
                                   * Cylinder(5.5, 6))
    asm.add(Pos(MAST_X1 + 12, 0, CAM1_Z) * ai_body, "camera1_ai_rpi_sc1174",
            color=Color(0.1, 0.5, 0.1))

    pi_plate = Pos(MAST_X1 + 2.75, 0, 190) * Box(PT, 2 * MAST_YC - MAST_W, 100)
    asm.add(pi_plate, "pi_mount_plate", color=Color(0.6, 0.6, 0.62))
    pi = import_step("parts/raspberry_pi_5.step")
    pi = Rot(Z=90) * place(pi, 0, 0, 0)   # native board plane is XZ -> YZ
    asm.add(place(pi, MAST_X1 + 17.5, 0, 190), "raspberry_pi_5",
            color=Color(0.0, 0.45, 0.2))

    # ------------------------------------------------------- electronics
    # battery + power bank hang in strap bays under the deck (low CG),
    # between the corridor wall and the wheels
    bat = Pos(14, 62, DECK_Z - BAT_H / 2) * Box(BAT_L, BAT_W, BAT_H)
    asm.add(bat, "battery_zee_lipo_3s", color=Color(0.2, 0.2, 0.25))
    pb = Pos(14, -65, DECK_Z - 12.5) * Box(100, 50, 25)
    asm.add(pb, "anker_power_bank", color=Color(0.25, 0.25, 0.3))
    for sy, tag in ((1, "left"), (-1, "right")):
        drv = Pos(137.5, sy * 70.5, DECK_TOP + 11) * Box(45, 75, 22)
        asm.add(drv, f"mdd3a_driver_{tag}", color=Color(0.55, 0.1, 0.1))

    return asm.build()
