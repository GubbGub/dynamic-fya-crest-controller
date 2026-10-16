"""
parse_ssm.py  --  turn SUMO's SSM logs into one clean table of left-turner vs oncoming conflicts

Usage:  python parse_ssm.py
Output: data_outputs/conflicts.csv   one row per (left-turner, oncoming car) pair per run

Columns: mode, seed, left_turner, oncoming, oncoming_type, TTC, PET, oncoming_speed, in_crossing, free_flow

in_crossing = the closest approach happened while the left-turner was inside the junction.
free_flow   = in_crossing AND the oncoming car was moving at least FREE_FLOW_MIN_SPEED at that moment.
Why: at the start of every green, the oncoming queue is still pulling away from the stop line at 2-5 m/s
and the left-turner slips across in front of the first car. SUMO scores that as TTC ~ 1.5 s every cycle.
That is a queue-discharge squeeze, not a car clearing the blind crest, so the hypotheses use free_flow only.
"""
import glob
import os
import re
import xml.etree.ElementTree as ET

import pandas as pd

OUT = "data_outputs"
FREE_FLOW_MIN_SPEED = 8.0     # m/s  (about 18 mph)


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None          # SUMO writes "NA" when a measure never happened


def parse_one(path, mode, seed):
    tags = pd.read_csv(os.path.join(OUT, f"tags_{mode}_{seed}.csv"), dtype={"Vehicle_ID": str}).set_index("Vehicle_ID")
    left = tags["Is_Left_Turner"].to_dict()
    cross = pd.read_csv(os.path.join(OUT, f"crossings_{mode}_{seed}.csv"), dtype={"Vehicle_ID": str}).set_index("Vehicle_ID")
    win = {v: (r.Enter_Junction_s, r.Exit_Junction_s) for v, r in cross.iterrows()}
    vtype = tags["Vehicle_Type"].to_dict()
    rows = []
    for c in ET.parse(path).getroot().iter("conflict"):
        ego, foe = c.get("ego"), c.get("foe")
        # SUMO logs every pair twice (once from each car). Keep the record where ego = the oncoming car,
        # so the speed in it is the oncoming car's speed.
        if left.get(ego, 1) != 0 or left.get(foe, 0) != 1:
            continue
        ttc, pet = c.find("minTTC"), c.find("PET")
        speed = num(ttc.get("speed")) if ttc is not None else None
        t_min = num(ttc.get("time")) if ttc is not None else None
        # the closest approach has to happen while the left-turner is actually crossing
        w0, w1 = win.get(foe, (None, None))
        in_window = int(t_min is not None and w0 is not None and w0 - 1.0 <= t_min <= w1 + 1.0)
        rows.append({
            "mode": mode, "seed": seed, "left_turner": foe, "oncoming": ego,
            "oncoming_type": vtype.get(ego),
            "TTC": num(ttc.get("value")) if ttc is not None else None,
            "PET": num(pet.get("value")) if pet is not None else None,
            "oncoming_speed": speed,
            "in_crossing": in_window,
            "free_flow": int(in_window and speed is not None and speed >= FREE_FLOW_MIN_SPEED),
        })
    return rows


def main():
    rows = []
    for p in sorted(glob.glob(os.path.join(OUT, "ssm_*_*.xml"))):
        m = re.match(r"ssm_(baseline|dynamic|protected)_(\d+)\.xml", os.path.basename(p))
        if m and os.path.exists(os.path.join(OUT, f"tags_{m.group(1)}_{m.group(2)}.csv")):
            rows += parse_one(p, m.group(1), int(m.group(2)))     # runs that crashed have no tags file -> skipped
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(OUT, "conflicts.csv"), index=False)
    print(f"{len(df)} left-turner / oncoming pairs ({int(df['free_flow'].sum()) if len(df) else 0} free-flow) "
          f"-> {OUT}/conflicts.csv")


if __name__ == "__main__":
    main()