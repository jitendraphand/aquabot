"""
AquaBot — single source of truth for every dimension, mass and rating.

Units: millimetres, grams, seconds, degrees (unless a name says otherwise).
Body frame: origin on the plane of the four hip-abduction (J1) axes, centred
between them; +X forward, +Y left, +Z up.

Everything downstream (CAD, kinematics, simulation, URDF, firmware config,
BOM and QA) is generated from these values. Change a number here and run
`python -m scripts.build_all` to regenerate the whole project.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import radians

VERSION = "1.0.0"
PROJECT = "AquaBot"

# --------------------------------------------------------------------------
# Manufacturing
# --------------------------------------------------------------------------
PRINT_BED = (220.0, 220.0, 250.0)  # Ender-3 class; every printed part must fit
MIN_WALL = 1.6                     # 4 perimeters @ 0.4 mm nozzle
PRINT_SHELL = 1.6                  # shell thickness used for mass estimate
PRINT_INFILL = 0.30                # gyroid infill fraction for mass estimate

MATERIALS = {
    # name: density g/cm3, colour (rgba 0..1), note
    "PETG": dict(density=1.27, rgba=(0.93, 0.93, 0.90, 1.0), note="PETG, 4 walls, 30 % gyroid"),
    "PETG-G": dict(density=1.27, rgba=(0.20, 0.62, 0.36, 1.0), note="PETG (green), 4 walls, 30 % gyroid"),
    "PETG-B": dict(density=1.27, rgba=(0.12, 0.45, 0.80, 1.0), note="PETG (blue), 100 % walls for water tightness"),
    "TPU95A": dict(density=1.21, rgba=(0.10, 0.10, 0.10, 1.0), note="TPU 95A, 3 walls, 25 % infill"),
}

# --------------------------------------------------------------------------
# Vendor servo envelopes (datasheet values, verify against your batch)
# Servo local frame: origin = shaft axis ∩ case top face, +Z = shaft (out),
# +X = towards the long side of the case (case spans X ∈ [-shaft_off, L-shaft_off]).
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class ServoSpec:
    name: str
    case_l: float
    case_w: float
    case_h: float
    shaft_off: float        # shaft axis distance from the near end of the case
    ear_span: float         # tip-to-tip length of the mounting flange
    ear_t: float            # flange thickness
    ear_top: float          # distance from case top face down to flange top face
    hole_pitch_l: float     # mounting hole pitch along length
    hole_pitch_w: float     # mounting hole pitch across width
    hole_d: float           # flange hole diameter
    boss_d: float           # output boss diameter
    boss_h: float           # output boss height above case top
    horn_d: float           # disc horn diameter
    horn_t: float           # disc horn thickness
    horn_top: float         # horn top face height above case top
    horn_hub_d: float
    horn_pcd: float         # horn bolt circle
    horn_hole_d: float
    horn_screws: str        # fastener used horn -> printed part
    mass: float             # g (servo only)
    horn_mass: float        # g
    stall_torque: float     # N·m at 6.0 V
    speed: float            # rad/s no-load at 6.0 V
    stall_current: float    # A at 6.0 V
    idle_current: float     # A
    range_deg: float
    price_usd: float
    mount_screw: str

    @property
    def case_center_x(self) -> float:
        return self.case_l / 2.0 - self.shaft_off

    @property
    def ear_z0(self) -> float:  # flange bottom face (towards case bottom)
        return -self.ear_top - self.ear_t

    @property
    def ear_z1(self) -> float:  # flange top face
        return -self.ear_top


DS3218 = ServoSpec(
    name="DS3218MG 270°", case_l=40.0, case_w=20.0, case_h=40.5, shaft_off=10.0,
    ear_span=54.5, ear_t=2.5, ear_top=11.0, hole_pitch_l=49.5, hole_pitch_w=10.0,
    hole_d=4.2, boss_d=13.0, boss_h=1.5, horn_d=25.0, horn_t=2.5, horn_top=7.5,
    horn_hub_d=9.0, horn_pcd=18.0, horn_hole_d=3.2, horn_screws="M3x8 SHCS",
    mass=60.0, horn_mass=6.0, stall_torque=1.96, speed=7.0, stall_current=2.5,
    idle_current=0.008, range_deg=270.0, price_usd=14.0, mount_screw="M3x10 SHCS",
)

MG90S = ServoSpec(
    name="MG90S", case_l=22.8, case_w=12.2, case_h=22.7, shaft_off=6.0,
    ear_span=32.2, ear_t=2.5, ear_top=4.5, hole_pitch_l=27.8, hole_pitch_w=0.0,
    hole_d=2.2, boss_d=11.8, boss_h=4.0, horn_d=20.0, horn_t=2.0, horn_top=8.0,
    horn_hub_d=7.0, horn_pcd=12.0, horn_hole_d=2.2, horn_screws="M2x6 self-tapping",
    mass=13.4, horn_mass=1.5, stall_torque=0.216, speed=10.5, stall_current=0.7,
    idle_current=0.006, range_deg=180.0, price_usd=4.0, mount_screw="M2x8 self-tapping",
)

# Fork-joint layout shared by every DS3218 pivot (J2, J3, arm shoulder, arm elbow).
# Joint frame: origin on the pivot axis in the link mid-plane, +Y along the
# servo shaft. Child link = fork: outer tine on the horn, inner tine on an
# idler bearing carried by the parent.
TINE_T = 6.0          # tine thickness
TINE_GAP = 1.5        # clearance servo case bottom -> inner tine
IDLER_T = 6.0         # parent idler arm thickness
IDLER_GAP = 1.0       # inner tine -> idler arm
TINE_R = 15.0         # tine boss radius
TINE_HALF_W = 12.0    # tine plate half width
WEB_R0 = 28.0         # fork crossbar inner radius from pivot
WEB_R1 = 36.0         # fork crossbar outer radius from pivot


def fork_layout(s: ServoSpec = DS3218) -> dict:
    """Axial (Y) positions of a fork joint, mid-plane at Y = 0."""
    mid = (-s.case_h + s.horn_top - TINE_GAP) / 2.0     # in servo axial coords
    case_top = -mid                                      # servo case top in joint Y
    lay = dict(
        case_top=case_top,
        case_bot=case_top - s.case_h,
        horn_top=case_top + s.horn_top,
        ear0=case_top + s.ear_z0,
        ear1=case_top + s.ear_z1,
    )
    lay["outer_tine"] = (lay["horn_top"], lay["horn_top"] + TINE_T)
    lay["inner_tine"] = (lay["case_bot"] - TINE_GAP - TINE_T, lay["case_bot"] - TINE_GAP)
    lay["idler"] = (lay["inner_tine"][0] - IDLER_GAP - IDLER_T, lay["inner_tine"][0] - IDLER_GAP)
    lay["ear_plate"] = (lay["ear0"] - 5.0, lay["ear0"])   # plate under the flange
    return lay


FORK = fork_layout(DS3218)

# Idler pivot hardware (every fork joint)
IDLER_BEARING = dict(name="625-2RS", d=5.0, D=16.0, B=5.0, mass=5.0, price_usd=0.6)
IDLER_BOLT = "M5x20 low-head SHCS (DIN 7984)"
IDLER_NUT = "M5 nyloc nut + M5 washer"

# --------------------------------------------------------------------------
# Body
# --------------------------------------------------------------------------
TUB_X = 90.0            # tub spans x ∈ [-TUB_X, TUB_X]; ends closed by bulkheads
BULKHEAD_T = 5.0        # bulkheads at |x| ∈ [TUB_X, TUB_X + BULKHEAD_T]
BODY_W = 46.0           # half width (outer)
WALL_T = 3.0
TUB_Z0 = -50.0          # tub bottom
FLOOR_T = 4.0
TUB_Z1 = 36.0           # tub rim
DECK_T = 4.0            # deck z ∈ [TUB_Z1, TUB_Z1 + DECK_T]
DECK_Z1 = TUB_Z1 + DECK_T
BULK_Z0, BULK_Z1 = TUB_Z0, DECK_Z1
BOSS_D = 8.0            # insert bosses
INSERT_M3 = dict(d_hole=4.0, depth=5.0)   # heat-set insert pilot

# Hip abduction (J1) servos: shaft along ±X out of the bulkheads, case
# vertical with its bulk downward.
Y_J1 = 28.0
J1_CASE_TOP_X = TUB_X + BULKHEAD_T + (DS3218.ear_top + DS3218.ear_t)  # flange seats on bulkhead
J1_HORN_X = J1_CASE_TOP_X + DS3218.horn_top                             # 116.0

# Shoulder bracket / J2 placement (relative to J1 joint origin at horn face)
BRACKET_T = 5.0
J2_DX = BRACKET_T + 1.0 + TINE_R          # J2 axis ahead of the J1 horn face
J2_DZ = -45.0                              # J2 axis below the J1 axis (QA iter 2: was -32, femur hit tub floor)
Y_LEG = 58.0                               # leg mid-plane |y|
J2_DY = Y_LEG - Y_J1                       # lateral offset J1 axis -> leg plane

# Leg segment lengths
L_FEMUR = 90.0
L_TIBIA = 100.0
FOOT_R = 13.0

# Derived hip point (J2 axis in body frame, front-left)
X_HIP = J1_HORN_X + J2_DX                  # 137.0

# --------------------------------------------------------------------------
# Water system
# --------------------------------------------------------------------------
TANK = dict(x0=-62.0, x1=62.0, y=42.0, z0=-46.0, z1=0.0, wall=2.4, floor=2.4)
LID_T = 2.4   # QA iter 8: was 3.0 (mass budget)
TANK_TARGET_ML = 400.0
FILL_NECK = dict(x=-50.0, y=28.0, id=20.0, od=26.0, top=DECK_Z1 + 4.0)
DIP_PORT = dict(x=-15.0, y=-30.0)
PUMP = dict(flow_ml_min=100.0, voltage=6.0, current=0.35, mass=62.0, price_usd=9.0,
            name="6 V DC mini peristaltic pump, 2x3 mm tube head")
NOZZLE_ORIFICE = 1.5      # mm
TUBE = dict(id=2.0, od=4.0)   # silicone tube (matches peristaltic head)

# --------------------------------------------------------------------------
# Arm (yaw + shoulder + elbow + wrist pitch) and gripper
# --------------------------------------------------------------------------
ARM_YAW_XY = (55.0, 0.0)
ARM_SHOULDER_Z = 128.0     # shoulder axis height in body frame (QA iter 7: +8 mm so the arm can reach down to -27°)
L_ARM1 = 120.0             # shoulder -> elbow
L_ARM2 = 115.0             # elbow -> wrist
L_TCP = 75.0               # wrist -> gripper TCP (between finger pads)
NOZZLE_OFF = (58.0, 20.0)  # nozzle tip in wrist frame: (along tool, towards 'up' when tool horizontal)
GRIP_STROKE = 20.0         # per jaw
GRIP_PINION_TEETH = 20
GRIP_MODULE = 1.0
PAYLOAD_G = 100.0
YAW_BEARING = dict(name="61806-2RS", d=30.0, D=42.0, B=7.0, mass=18.0, price_usd=4.0)

# --------------------------------------------------------------------------
# Joint limits (deg). Mechanical limits are verified by the QA sweep.
# Kinematic conventions (see docs/kinematics.md):
#   hip_abd: +rot about +X;  hip_pitch, knee: +rot about +Y (foot moves -X).
#   arm_yaw: +rot about +Z;  arm_shoulder/elbow/wrist: +rot = tool pitches up.
# --------------------------------------------------------------------------
LIMITS = {
    "hip_abd": (-25.0, 25.0),
    "hip_pitch_front": (-75.0, 60.0),     # QA sweep: femur meets tub floor at ~65°
    "hip_pitch_rear": (-60.0, 75.0),
    "knee_front": (-125.0, -10.0),        # QA sweep: tibia web meets femur housing at ~-132°
    "knee_rear": (10.0, 125.0),           # rear knees point forward (ANYmal-style 'X' layout)
    "arm_yaw": (-120.0, 120.0),
    "arm_shoulder": (-27.0, 125.0),       # QA sweep: upper-arm crossbar meets turntable at ~-31°
    "arm_elbow": (-125.0, 15.0),          # QA sweep: forearm web meets upper-arm housing at ~-130°
    "arm_wrist": (-100.0, 100.0),
    "grip": (0.0, GRIP_STROKE),
}

# --------------------------------------------------------------------------
# Stance & gait defaults (tuned by QA loop, see qa/qa_history.json)
# --------------------------------------------------------------------------
STAND_HEIGHT = 225.0          # body origin above ground (QA iter 6: >=8° joint-limit margin over crawl envelope)
CROUCH_HEIGHT = 150.0         # pick-up crouch (front knee ~ -122°, limit -125°)
GAIT = dict(period=0.50, duty=0.5, step_height=30.0, stride=50.0, stance_dx=5.0,
            foot_in=0.0, stance_dy=16.0)
# foot_in : feet inboard of J2 (between hip and knee) balances hip/knee torque for a trot (25 mm),
#           but the crawl needs hip-pitch margin while lifting, so QA iter 6 set it to 0.
# stance_dx: whole stance shifted forward towards the CoG (arm is forward of centre)

# Crawl gait (default locomotion). Values may be overridden by the QA control
# loop, which writes aquabot/tuned.json (see aquabot/qa/run.py).
CRAWL = dict(period=2.4, swing=0.108, step_height=28.0, margin=20.0, v_cmd=0.035,
             turn_period=3.0)   # QA iter 10: turning in place rocks diagonally & tips for 1.9–2.35 s cycles
WATER_STANCE = dict(height=195.0, roll_deg=-4.0, pitch_deg=-50.0, drop=55.0)
PICK_STANCE = dict(height=150.0, roll_deg=-10.0, shift_y=20.0, pitch_deg=-75.0,
                   foot_dx_front=20.0, foot_dx_rear=-30.0)   # QA iter 8: feet re-stepped before crouching


def _load_tuned():
    import json, os
    path = os.path.join(os.path.dirname(__file__), "tuned.json")
    if os.path.exists(path):
        with open(path) as fh:
            t = json.load(fh)
        CRAWL.update(t.get("CRAWL", {}))


_load_tuned()

# --------------------------------------------------------------------------
# Electronics & power
# --------------------------------------------------------------------------
BATTERY = dict(name="2S 7.4 V 2200 mAh LiPo (XT60)", cells=2, mah=2200.0, v_nom=7.4,
               dims=(106.0, 34.0, 19.0), mass=118.0, price_usd=16.0,
               x0=-18.0, z0=4.5)
UBEC = dict(name="6.0 V 10 A UBEC", dims=(47.0, 23.5, 12.0), mass=28.0, price_usd=9.0,
            eff=0.90)
BUCK5 = dict(name="Mini-360 buck 5 V 2 A", dims=(17.0, 11.0, 4.0), mass=2.0, price_usd=1.0)
ESP32 = dict(name="ESP32-DevKitC V4 (WROOM-32E)", dims=(55.0, 28.0, 13.0), mass=10.0,
             price_usd=8.0, current=0.16)
PCA9685 = dict(name="PCA9685 16-ch PWM driver", dims=(62.0, 25.4, 15.0), mass=9.0,
               price_usd=4.0, current=0.01)
IMU = dict(name="MPU-6050 6-axis IMU (GY-521)", dims=(21.0, 16.0, 3.0), mass=2.0, price_usd=2.0)
TOF = dict(name="VL53L0X ToF range sensor", dims=(25.0, 13.0, 3.0), mass=1.0, price_usd=4.0)
MOSFET = dict(name="AO3400 logic-level MOSFET module (pump)", dims=(20.0, 15.0, 5.0), mass=2.0,
              price_usd=1.5)

SERVO_EFFICIENCY = 0.45       # electrical -> mechanical (small coreless/brushed gear servos)
ELECTRONICS_W = 1.6           # ESP32 + logic, W

# --------------------------------------------------------------------------
# Requirements the QA loop verifies (traceable IDs in docs/requirements.md)
# --------------------------------------------------------------------------
REQ = dict(
    mass_max_g=3200.0,
    tank_ml_min=400.0,
    torque_sf_static_min=2.0,     # stall / static demand
    sim_peak_torque_frac=0.90,    # of stall, 50 ms moving-average (impact spikes reported separately)
    sim_p99_torque_frac=0.75,     # of stall, 99th percentile instantaneous
    stability_margin_min=25.0,    # mm, CoG to support polygon edge (4-leg stance)
    walk_speed_min=0.02,          # m/s, crawl gait with full tank (1.2 m/min)
    tilt_max_deg=12.0,
    runtime_min=20.0,             # min
    water_delivery_min=0.85,      # fraction of emitted water landing in target pot
    ik_err_max=1e-6,              # mm
    fw_parity_max=1e-3,           # deg
    grasp_lift_min=30.0,          # mm
    place_err_max=25.0,           # mm
    interference_max_mm3=0.5,
    ground_clearance_min=100.0,   # mm, belly at stand height
)


def sign_e(leg: str) -> int:
    return 1 if leg[0] == "F" else -1


def sign_s(leg: str) -> int:
    return 1 if leg[1] == "L" else -1


LEGS = ("FL", "FR", "RL", "RR")


def hip_pitch_limits(leg: str):
    return LIMITS["hip_pitch_front"] if leg[0] == "F" else LIMITS["hip_pitch_rear"]


def knee_limits(leg: str):
    return LIMITS["knee_front"] if leg[0] == "F" else LIMITS["knee_rear"]


def knee_sign(leg: str) -> int:
    """-1: knee points backwards (front legs), +1: knee points forwards (rear legs)."""
    return -1 if leg[0] == "F" else 1


def as_dict() -> dict:
    """Flat snapshot used for hashing / firmware config generation."""
    out = {}
    for k, v in globals().items():
        if k.isupper() and isinstance(v, (int, float, str, tuple, dict)):
            out[k] = v
    return out
