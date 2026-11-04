"""Registry of every part (printed + purchased), built once per process."""
from __future__ import annotations

from functools import lru_cache
from typing import Dict

from .. import params as P
from . import parts_body as PB, parts_leg as PL, parts_arm as PA, vendor as V
from .geom import Part

PRINTED_BUILDERS = {
    "chassis_tub": PB.chassis_tub,
    "bulkhead_front": lambda: PB.bulkhead(True),
    "bulkhead_rear": lambda: PB.bulkhead(False),
    "top_deck": PB.top_deck,
    "tank_body": PB.tank_body,
    "tank_lid": PB.tank_lid,
    "fill_cap": PB.fill_cap,
    "electronics_tray": PB.electronics_tray,
    "electronics_cover": PB.electronics_cover,
    "arm_pedestal": PB.arm_pedestal,
    "arm_bearing_plate": PB.arm_bearing_plate,
    "arm_turntable": PA.arm_turntable,
    "upper_arm": PA.upper_arm,
    "forearm": PA.forearm,
    "gripper_base": PA.gripper_base,
    "gripper_jaw": PA.gripper_jaw,
    "gripper_pinion": PA.pinion,
}

VENDOR_BUILDERS = {
    "servo_ds3218": lambda: V.servo(P.DS3218, "servo_ds3218"),
    "servo_mg90s": lambda: V.servo(P.MG90S, "servo_mg90s"),
    "horn_ds3218": lambda: V.horn(P.DS3218, "horn_ds3218"),
    "horn_mg90s": lambda: V.horn(P.MG90S, "horn_mg90s"),
    "bearing_625": lambda: V.bearing(P.IDLER_BEARING, "bearing_625"),
    "bearing_61806": lambda: V.bearing(P.YAW_BEARING, "bearing_61806"),
    "battery_2s_2200": V.battery,
    "pump_peristaltic": V.pump,
    "ubec_6v_10a": V.ubec,
    "buck_5v": lambda: V.board("buck_5v", P.BUCK5, V.PCB_B, comp_h=2.4),
    "imu_mpu6050": lambda: V.board("imu_mpu6050", P.IMU, V.PCB_B, comp_h=1.4),
    "tof_vl53l0x": V.tof,
    "rocker_switch": V.rocker_switch,
    "xt60_panel_mount": V.xt60_panel,
    "pca9685": lambda: V.board("pca9685", P.PCA9685, V.PCB_B),
    "esp32_devkitc": lambda: V.board("esp32_devkitc", P.ESP32, (0.08, 0.08, 0.10, 1.0)),
    "mosfet_module": lambda: V.board("mosfet_module", P.MOSFET, V.PCB_G),
    "rod_3x74": lambda: V.steel_rod(74.0, 3.0, "rod_3x74"),
    "nozzle_brass": V.nozzle,
}


@lru_cache(maxsize=1)
def all_parts() -> Dict[str, Part]:
    parts: Dict[str, Part] = {}
    for k, f in PRINTED_BUILDERS.items():
        parts[k] = f()
    parts.update(PL.build_leg_parts())
    for k, f in VENDOR_BUILDERS.items():
        p = f()
        p.name = k
        parts[k] = p
    for k, p in parts.items():
        if p.rgba is None:
            p.rgba = P.MATERIALS.get(p.material, {}).get("rgba", (0.7, 0.7, 0.7, 1.0))
    return parts


def printed_parts() -> Dict[str, Part]:
    return {k: p for k, p in all_parts().items() if not p.vendor}


def vendor_parts() -> Dict[str, Part]:
    return {k: p for k, p in all_parts().items() if p.vendor}
