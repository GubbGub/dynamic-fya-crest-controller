"""
run_batch.py  --  N baseline + N dynamic runs, then parse the SSM logs

Usage:
    python run_batch.py 3                      quick test, seeds 1-3
    python run_batch.py 50 --start 101         the real experiment: seeds 101-150, baseline + dynamic
    python run_batch.py 50 --start 101 --tau 6.0
Tune tau / loop distance on seeds 1-10, then run the final experiment on fresh seeds (101+).
"""
import argparse
import subprocess
import sys

ap = argparse.ArgumentParser()
ap.add_argument("n", type=int, nargs="?", default=50)
ap.add_argument("--start", type=int, default=1)
ap.add_argument("--tau", type=float, default=None)
a = ap.parse_args()

failed = []
for seed in range(a.start, a.start + a.n):
    for mode in ("baseline", "dynamic"):
        cmd = [sys.executable, "runner.py", "--mode", mode, "--seed", str(seed)]
        if a.tau is not None and mode == "dynamic":
            cmd += ["--tau", str(a.tau)]
        if subprocess.run(cmd).returncode != 0:
            failed.append((mode, seed))

subprocess.run([sys.executable, "parse_ssm.py"], check=True)
print(f"\nDone. {2 * a.n - len(failed)}/{2 * a.n} runs succeeded. Failed: {failed or 'none'}")