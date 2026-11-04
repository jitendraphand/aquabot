from pathlib import Path

from aquabot import bom, robot as R
from aquabot.cad.catalog import all_parts
from aquabot import firmware_config

ROOT = Path(__file__).resolve().parents[1]


def test_every_instance_has_a_part():
    parts = all_parts()
    for l in R.build_tree().values():
        for it in l.instances:
            assert it.part in parts


def test_parts_are_single_valid_solids():
    for k, p in all_parts().items():
        vals = p.shape.vals()
        assert len(vals) == 1 and vals[0].isValid(), k


def test_bom_quantities_match_tree():
    rows = {r["item"]: r["qty"] for r in bom.build_rows()}
    for k, n in bom.instance_counts().items():
        assert rows[k] == n


def test_firmware_config_is_current():
    assert (ROOT / "firmware" / "aquabot" / "config.h").read_text() == firmware_config.generate()
