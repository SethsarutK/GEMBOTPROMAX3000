"""à¸•à¸£à¸§à¸ˆ sim à¸‹à¹‰à¸³à¸«à¸¥à¸²à¸¢ seed à¹à¸¥à¹‰à¸§à¸£à¸²à¸¢à¸‡à¸²à¸™à¸„à¸§à¸²à¸¡à¸œà¸´à¸”à¸›à¸à¸•à¸´à¸•à¹ˆà¸­à¸£à¸­à¸š"""
import re, sys, collections
import sim_fast

def analyze(seed, speed, ideal=False):
    sim_fast.RealWorld.SPEED_CM_S = speed
    st, lines = sim_fast.run(seed=seed, ideal=ideal, verbose=False)
    tot = collections.Counter(); prev_t = 0.0; last_t = 0.0
    idle_gaps = []
    for l in lines:
        m = re.match(r"\[\s*([\d.]+)s\] (\w+) -> (\w+)", l)
        if m:
            t, a = float(m.group(1)), m.group(2)
            tot[a] += t - prev_t
            if t - prev_t > 20 and a not in ("GO_APPROACH", "GO_ZONE_AP"):
                idle_gaps.append((a, round(t - prev_t)))
            prev_t = t
    issues = []
    if st["delivered"] < 4: issues.append(f"delivered à¸•à¹ˆà¸³ {st['delivered']}")
    if st["timeout"]: issues.append(f"timeout {st['timeout']}")
    if st["giveup"] >= 3: issues.append(f"give up {st['giveup']}")
    if st["wrong_colour_grabs"] >= 3: issues.append(f"à¸«à¸™à¸µà¸šà¸œà¸´à¸”à¸ªà¸µ {st['wrong_colour_grabs']}")
    if st["verify TP/FP/TN/FN"][3] >= 2: issues.append(f"VERIFY FN {st['verify TP/FP/TN/FN'][3]}")
    if st["verify TP/FP/TN/FN"][1] >= 3: issues.append(f"VERIFY FP {st['verify TP/FP/TN/FN'][1]}")
    if st["runover"] >= 3: issues.append(f"à¸—à¸±à¸šà¸«à¸´à¸™ {st['runover']}")
    if idle_gaps: issues.append(f"à¸„à¹‰à¸²à¸‡à¹ƒà¸™ state {idle_gaps}")
    lost = sum(1 for l in lines if re.search(r"zone (\d+)->(\d+)", l) and int(re.search(r"zone (\d+)->(\d+)", l).group(2)) < int(re.search(r"zone (\d+)->(\d+)", l).group(1)))
    if lost: issues.append(f"à¸«à¸´à¸™à¸«à¸¥à¸¸à¸”à¸­à¸­à¸à¸ˆà¸²à¸à¸§à¸‡ {lost} à¸„à¸£à¸±à¹‰à¸‡")
    nopick = sum(1 for l in lines if "no pickable" in l)
    skips = sum(1 for l in lines if "contested" in l)
    if skips >= 8: issues.append(f"contested skip {skips}")
    return st, issues

if __name__ == "__main__":
    for label, speed, ideal in (("motor normal", 0.6, False), ("motor slow 40%", 0.36, False)):
        print(f"===== {label} =====")
        tot = 0; bad = 0; hits = []; exit_by = {}; score = 0
        for seed in range(3, 15):
            st, issues = analyze(seed, speed, ideal)
            tot += st["delivered"]
            flag = "  <-- " + "; ".join(issues) if issues else ""
            if issues: bad += 1
            for k, v in st["zone_exit_by"].items():
                exit_by[k] = exit_by.get(k, 0) + v
            score += st["score"]
            print(f"seed {seed:2d}: delivered {st['delivered']:2d} SCORE {st['score']:2d}  picked {st['picked']:2d}  still {st['still']:2d}  verify {st['verify TP/FP/TN/FN']}  wrong {st['wrong_colour_grabs']}"
                  f"  | hit body {st['hit_body']} seat {st['hit_seat']} jaw {st['hit_jaw']} zone_exit {st['zone_exits']}{flag}")
            hits.append(st)
        hb = sum(s['hit_body'][0] for s in hits); hj = sum(s['hit_jaw'][0] for s in hits); ze = sum(s['zone_exits'] for s in hits)
        print(f"TOTAL delivered {tot}  SCORE {score} / 12 runs (avg {score/12:.1f})   runs with issues: {bad}   | body-hit gems {hb}  jaw-hit gems {hj}  zone exits {ze}")
        print("zone exits by state/part:", dict(sorted(exit_by.items(), key=lambda kv: -kv[1])))
