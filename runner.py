"""
runner.py  --  one simulation run (baseline FYA or sensor-actuated controller)

Usage:
    python runner.py --mode baseline --seed 1
    python runner.py --mode dynamic  --seed 1
    python runner.py --mode protected --seed 1      (reference: protected-only left turns, no permissive left)
    python runner.py --mode baseline --seed 1 --blind off     (old behaviour: no hidden crest, for comparison)

Each run writes:
    data_outputs/vehicles_<mode>_<seed>.csv   one row per vehicle (first detection at the crest loop)
    data_outputs/ssm_<mode>_<seed>.xml        SUMO's own TTC / PET conflict log (gitignored: *.xml)
    data_outputs/summary_<mode>_<seed>.csv    left-turners, how many were exposed to hidden cars, collisions, delay
    data_outputs/crossings_<mode>_<seed>.csv  when each left-turner was inside the junction
"""
import argparse
import csv
import math
import os
import sys

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    sys.exit("CRITICAL ERROR: Please configure your SUMO_HOME environment pointer.")

import traci

# ----------------------------- settings -----------------------------
TLS_ID = "center"
LEFT_FROM = "S2Center"   # the approach whose left-turners cross the oncoming crest traffic
MAX_TIME = 4000          # s  hard stop (SUMO ignores <end> when TraCI is stepping)
SENSOR_IDS = ["upstream_sensor_lane_0", "upstream_sensor_lane_1"]
TAU = 6.0            # s  override threshold. Tuned on seeds 1-3 (loop 100 m from stop line); the original 4.5 fired for almost nobody
HOLD_AFTER_ARRIVAL = 2.0   # s  arrow stays red until the triggering car is this long past the stop line
LEFT_TURN_SPEED = 2.24     # m/s  left-turners cross the junction slowly: 5 mph (from the notes)
MIN_SPEED = 0.1      # m/s  ignore cars that are effectively stopped

# ---- blind-crest visibility model --------------------------------------------------------
# SUMO has no line of sight, so we add one. AASHTO crest-curve sight distance (when S < L):
#       S = ( sqrt(2*h1) + sqrt(2*h2) ) * sqrt(100*L / A)
# For this one hill, sqrt(100*L/A) is a constant, so how far a driver can see scales with
# ( sqrt(2*h1) + sqrt(2*h2) ). We anchor it to the distance measured at the real site for a sedan.
SEDAN_VISIBILITY_M = 77.0     # m   how far a sedan driver could see over the crest (from the notes)
H2 = 0.60                     # m   height of the oncoming object (headlights/grille), AASHTO
EYE_HEIGHT = {                # m   h1, driver eye height by vehicle type
    "sedan": 1.08,
    "left_sedan": 1.08,
    "truck": 1.80,
    "speeding_truck": 1.80,
    "school_bus": 2.40,
}
OPPOSING_EDGES = ["N2Crest", "Crest2Center"]   # the approach that is hidden behind the crest
SPEEDMODE_NORMAL = 31         # TraCI speed mode: all safety checks on
SPEEDMODE_IGNORE_ROW = 23     # same, but ignores right of way (the driver "doesn't see" a foe)
# --------------------------------------------------------------------------------------------


def edge_of(lane_id):
    return lane_id.rsplit("_", 1)[0]


def heading(edge_id, at_end):
    """Direction (dx, dy) of an edge's first or last lane segment."""
    shape = traci.lane.getShape(edge_id + "_0")
    (x1, y1), (x2, y2) = (shape[-2], shape[-1]) if at_end else (shape[0], shape[1])
    return x2 - x1, y2 - y1


def is_left_turn(from_edge, to_edge):
    """True if going from_edge -> to_edge is a left turn (net-independent, uses geometry)."""
    ax, ay = heading(from_edge, at_end=True)
    bx, by = heading(to_edge, at_end=False)
    angle = math.degrees(math.atan2(ax * by - ay * bx, ax * bx + ay * by))
    return 30 < angle < 150          # counter-clockwise = left in SUMO's coordinates


def left_turn_signal_indices(tls_id):
    """Signal-state character positions that control left turns FROM the LEFT_FROM approach."""
    idx = []
    for i, links in enumerate(traci.trafficlight.getControlledLinks(tls_id)):
        for in_lane, out_lane, _via in links:
            if edge_of(in_lane) == LEFT_FROM and is_left_turn(edge_of(in_lane), edge_of(out_lane)):
                idx.append(i)
                break
    return idx


def left_turn_edge_pairs(tls_id):
    """(from_edge, to_edge) pairs that are left turns at this signal."""
    pairs = set()
    for links in traci.trafficlight.getControlledLinks(tls_id):
        for in_lane, out_lane, _via in links:
            a, b = edge_of(in_lane), edge_of(out_lane)
            if a == LEFT_FROM and is_left_turn(a, b):
                pairs.add((a, b))
    return pairs


def visible_distance(vtype):
    """How far upstream (m from the oncoming car's stop line) this driver can see an oncoming car."""
    k = math.sqrt(2 * EYE_HEIGHT.get(vtype, EYE_HEIGHT["sedan"])) + math.sqrt(2 * H2)
    k_sedan = math.sqrt(2 * EYE_HEIGHT["sedan"]) + math.sqrt(2 * H2)
    return SEDAN_VISIBILITY_M * k / k_sedan


def oncoming_distances():
    """Distance to the stop line for every car currently on the hidden approach."""
    out = []
    for edge in OPPOSING_EDGES:
        for veh in traci.edge.getLastStepVehicleIDs(edge):
            for tls_id, _idx, dist, _state in traci.vehicle.getNextTLS(veh):
                if tls_id == TLS_ID:
                    out.append(dist)
                    break
    return out


def apply_blindness(left_set, vtypes, modes, exposed):
    """
    Every step, for each left-turner that is approaching OR crossing the junction:
      - if it can SEE an oncoming car (inside its visible distance) -> normal SUMO behaviour (it yields)
      - if it sees none -> it ignores right of way, like a real driver who doesn't know a car is coming
    SUMO makes the actual go/yield decision at an internal point INSIDE the junction, so blindness has to
    stay on until the car has left the junction. It is switched back to normal once it is on the exit road.
    """
    live = set(traci.vehicle.getIDList())
    active = set()
    for veh in left_set:
        if veh in live:
            road = traci.vehicle.getRoadID(veh)
            if road == LEFT_FROM or road.startswith(":"):
                active.add(veh)
    dists = oncoming_distances() if active else []
    for veh in active:
        vis = visible_distance(vtypes[veh])
        sees_car = any(d <= vis for d in dists)
        want = SPEEDMODE_NORMAL if sees_car else SPEEDMODE_IGNORE_ROW
        if modes.get(veh) != want:
            traci.vehicle.setSpeedMode(veh, want)
            modes[veh] = want
        if want == SPEEDMODE_IGNORE_ROW and any(d > vis for d in dists):
            exposed.add(veh)                         # released while a hidden car was on the approach
    for veh in [v for v in modes if v not in active]:
        if veh in live:
            traci.vehicle.setSpeedMode(veh, SPEEDMODE_NORMAL)
        del modes[veh]


def slow_left_turns(left_set, orig_max, capped):
    """Left-turners approach the stop line and cross the junction slowly (5 mph), as in the notes."""
    on_approach = [v for v in traci.edge.getLastStepVehicleIDs(LEFT_FROM) if v in left_set]
    for veh in on_approach:
        nxt = traci.vehicle.getNextTLS(veh)
        if nxt and nxt[0][2] <= 25.0 and veh not in capped:
            orig_max[veh] = traci.vehicle.getMaxSpeed(veh)
            traci.vehicle.setMaxSpeed(veh, LEFT_TURN_SPEED)
            capped.add(veh)
    for veh in [v for v in capped if v in left_set]:
        road = traci.vehicle.getRoadID(veh) if veh in traci.vehicle.getIDList() else None
        if road is not None and road != LEFT_FROM and not road.startswith(":"):
            traci.vehicle.setMaxSpeed(veh, orig_max[veh])      # through the junction: back to normal
            capped.discard(veh)


def install_protected_program(tls_id):
    """
    Reference condition: protected-only left turns (no permissive left at all).
    Same 90 s cycle as the baseline. North-south time is 45 s, split as:
        11 s  south approach gets a green ARROW for its left turn (north approach is red)
         3 s  left arrow ends in yellow, south through traffic still green
        26 s  north and south through/right traffic green, both left turns RED
         5 s  yellow
    East-west phases are copied unchanged from the baseline program.
    """
    base = traci.trafficlight.getAllProgramLogics(tls_id)[0]
    ew_green, ew_yellow = base.phases[2], base.phases[3]
    info = []
    for links in traci.trafficlight.getControlledLinks(tls_id):
        a, b = edge_of(links[0][0]), edge_of(links[0][1])
        if is_left_turn(a, b):
            kind = "left"
        elif a.split("2")[0] == b.split("2")[-1]:
            kind = "uturn"
        else:
            kind = "other"
        info.append((a, kind))
    S, N = LEFT_FROM, OPPOSING_EDGES[-1]

    def st(rule):
        return "".join(rule(e, k) for e, k in info)

    A = st(lambda e, k: "G" if e == S else "r")
    A2 = st(lambda e, k: ("y" if k in ("left", "uturn") else "G") if e == S else "r")
    B = st(lambda e, k: "G" if e in (S, N) and k == "other" else "r")
    By = st(lambda e, k: "y" if e in (S, N) and k == "other" else "r")
    phases = [traci.trafficlight.Phase(11, A), traci.trafficlight.Phase(3, A2),
              traci.trafficlight.Phase(26, B), traci.trafficlight.Phase(5, By),
              traci.trafficlight.Phase(ew_green.duration, ew_green.state),
              traci.trafficlight.Phase(ew_yellow.duration, ew_yellow.state)]
    logic = traci.trafficlight.Logic("protected", 0, 0, phases=phases)
    traci.trafficlight.setProgramLogic(tls_id, logic)
    traci.trafficlight.setProgram(tls_id, "protected")
    traci.trafficlight.setPhase(tls_id, 0)


def force_left_turns_red(tls_id, left_idx):
    state = list(traci.trafficlight.getRedYellowGreenState(tls_id))
    for i in left_idx:
        state[i] = "r"
    traci.trafficlight.setRedYellowGreenState(tls_id, "".join(state))


def oncoming_timeloss(path):
    """Mean time loss (s) of every vehicle that is NOT a left-turner, from SUMO's trip log."""
    import xml.etree.ElementTree as ET
    vals = [float(t.get("timeLoss")) for t in ET.parse(path).getroot().iter("tripinfo")
            if not t.get("id").startswith("left_")]
    return sum(vals) / len(vals) if vals else float("nan")


def run(mode, seed, out_dir, blind=True, tau=TAU):
    os.makedirs(out_dir, exist_ok=True)
    vehicles_csv = os.path.join(out_dir, f"vehicles_{mode}_{seed}.csv")
    ssm_xml = os.path.join(out_dir, f"ssm_{mode}_{seed}.xml")
    tripinfo_xml = os.path.join(out_dir, f"tripinfo_{mode}_{seed}.xml")

    sumo = os.path.join(os.environ["SUMO_HOME"], "bin", "sumo")
    traci.start([
        sumo, "-c", "simulation.sumocfg", "--no-warnings", "--no-step-log", "--seed", str(seed),
        # SUMO's built-in surrogate safety measures: logs TTC and PET for every conflict
        "--device.ssm.probability", "1",
        "--device.ssm.measures", "TTC PET",
        "--device.ssm.thresholds", "6.0 3.0",   # log anything under 6.0 s TTC / 3.0 s PET
        "--device.ssm.file", ssm_xml,
        # count real crashes too, including inside the junction, and let cars keep moving afterwards
        "--collision.check-junctions", "true",
        "--collision.action", "warn",
        "--tripinfo-output", tripinfo_xml,
    ])
    if mode == "protected":
        install_protected_program(TLS_ID)

    # distance from each loop to the stop line, read from the network (not hard-coded)
    dist_to_stopline = {}
    for s in SENSOR_IDS:
        lane = traci.inductionloop.getLaneID(s)
        dist_to_stopline[s] = traci.lane.getLength(lane) - traci.inductionloop.getPosition(s)

    left_idx = left_turn_signal_indices(TLS_ID)
    left_pairs = left_turn_edge_pairs(TLS_ID)
    print(f"left-turn signal indices at {TLS_ID}: {left_idx}   (should not be empty)")
    print(f"loop distance to the end of its own edge (m): { {k: round(v, 1) for k, v in dist_to_stopline.items()} }  (TTI uses the true stop-line distance)")
    tags = []                       # one row per vehicle: is it a left-turner?
    left_set, vtypes, modes, exposed = set(), {}, {}, set()
    collisions = set()
    orig_max, capped = {}, set()
    enter_t, cross_time, timeloss = {}, {}, {}
    red_seconds = 0.0
    original_program = traci.trafficlight.getProgram(TLS_ID)

    seen = set()
    rows = []
    red_until = -1.0
    overriding = False

    while traci.simulation.getMinExpectedNumber() > 0 and traci.simulation.getTime() < MAX_TIME:
        traci.simulationStep()
        now = traci.simulation.getTime()

        for veh_id in traci.simulation.getDepartedIDList():
            route = traci.vehicle.getRoute(veh_id)
            is_left = int(any((a, b) in left_pairs for a, b in zip(route, route[1:])))
            vtypes[veh_id] = traci.vehicle.getTypeID(veh_id)
            if is_left:
                left_set.add(veh_id)
            tags.append([veh_id, vtypes[veh_id], is_left])

        slow_left_turns(left_set, orig_max, capped)
        if blind:
            apply_blindness(left_set, vtypes, modes, exposed)
        live = set(traci.vehicle.getIDList())
        for veh in left_set:
            if veh in live:
                timeloss[veh] = traci.vehicle.getTimeLoss(veh)
                road = traci.vehicle.getRoadID(veh)
                if road.startswith(":") and veh not in enter_t:
                    enter_t[veh] = now
                elif veh in enter_t and veh not in cross_time and not road.startswith(":"):
                    cross_time[veh] = now - enter_t[veh]

        for c in traci.simulation.getCollisions():
            collisions.add(frozenset((c.collider, c.victim)))

        for sensor in SENSOR_IDS:
            for veh_id in traci.inductionloop.getLastStepVehicleIDs(sensor):
                if veh_id in seen:
                    continue                        # log each vehicle once, not every timestep
                seen.add(veh_id)

                speed = traci.vehicle.getSpeed(veh_id)
                if speed <= MIN_SPEED:
                    continue
                nxt = traci.vehicle.getNextTLS(veh_id)
                dist = nxt[0][2] if nxt else dist_to_stopline[sensor]     # true distance to the stop line
                tti = dist / speed
                triggered = int(mode == "dynamic" and tti <= tau)

                rows.append([round(now, 1), veh_id, traci.vehicle.getTypeID(veh_id),
                             round(speed * 2.23694, 1), round(tti, 2), triggered])

                if triggered:
                    red_until = max(red_until, now + tti + HOLD_AFTER_ARRIVAL)

        # dynamic controller: hold the left-turn arrow red until the hold time runs out
        if mode == "dynamic":
            if now < red_until:
                force_left_turns_red(TLS_ID, left_idx)
                overriding = True
                red_seconds += 0.1
            elif overriding:
                traci.trafficlight.setProgram(TLS_ID, original_program)   # hand control back
                overriding = False

    traci.close()

    with open(vehicles_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Time", "Vehicle_ID", "Vehicle_Type", "Speed_MPH", "TTI", "Override_Triggered"])
        w.writerows(rows)
    with open(os.path.join(out_dir, f"tags_{mode}_{seed}.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Vehicle_ID", "Vehicle_Type", "Is_Left_Turner"])
        w.writerows(tags)
    with open(os.path.join(out_dir, f"crossings_{mode}_{seed}.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Vehicle_ID", "Enter_Junction_s", "Exit_Junction_s"])
        for veh, t0 in enter_t.items():
            if veh in cross_time:
                w.writerow([veh, round(t0, 1), round(t0 + cross_time[veh], 1)])
    with open(os.path.join(out_dir, f"summary_{mode}_{seed}.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["blind", "left_turners", "exposed_to_hidden_car", "collisions",
                    "mean_crossing_s", "mean_left_timeloss_s", "red_seconds", "overrides",
                    "mean_oncoming_timeloss_s"])
        mc = sum(cross_time.values()) / len(cross_time) if cross_time else float("nan")
        ml = sum(timeloss.values()) / len(timeloss) if timeloss else float("nan")
        w.writerow([int(blind), len(left_set), len(exposed), len(collisions),
                    round(mc, 2), round(ml, 2), round(red_seconds, 1), sum(r[5] for r in rows),
                    round(oncoming_timeloss(tripinfo_xml), 2)])
    print(f"[{mode} seed={seed}] left-turners: {len(left_set)}, exposed to a hidden car: {len(exposed)}, "
          f"collisions: {len(collisions)}")
    print(f"[{mode} seed={seed}] {len(rows)} vehicles logged -> {vehicles_csv}")
    print(f"[{mode} seed={seed}] SSM conflicts -> {ssm_xml}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["baseline", "dynamic", "protected"], required=True)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="data_outputs")
    ap.add_argument("--tau", type=float, default=TAU, help="override threshold in seconds (dynamic mode)")
    ap.add_argument("--blind", choices=["on", "off"], default="on",
                    help="on = left-turners cannot see cars hidden behind the crest")
    a = ap.parse_args()
    try:
        run(a.mode, a.seed, a.out, blind=(a.blind == "on"), tau=a.tau)
    except traci.exceptions.FatalTraCIError as e:
        # SUMO itself died mid-run (e.g. an SSM geometry problem). Skip this seed instead of killing a batch.
        print(f"[{a.mode} seed={a.seed}] FAILED: {e}  -> skipped, check sumo's error output", file=sys.stderr)
        sys.exit(2)