from pathlib import Path
import yaml

# -------------------------------------------------
# Script-Verzeichnis als Root
# -------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
GOALS_YAML = SCRIPT_DIR / "goals.yaml"


# -------------------------------------------------
# YAML laden
# -------------------------------------------------
with GOALS_YAML.open("r") as f:
    _raw = yaml.safe_load(f)


# -------------------------------------------------
# In internes GOALS-Format überführen
# -------------------------------------------------
GOALS = {}

for name, data in _raw.items():
    GOALS[name] = {
        "frame_id": data["header"].get("frame_id", "map"),
        "position": {
            "x": data["point"]["x"],
            "y": data["point"]["y"],
            "z": data["point"]["z"],
        },
        # feste Orientierung (Nav2-kompatibel)
        "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 1.0,
        }
    }
