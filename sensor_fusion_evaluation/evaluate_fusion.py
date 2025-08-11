#!/usr/bin/env python3

import os
import glob
import shutil
import subprocess
import sys
import tempfile

def run_cmd(cmd, cwd=None):
    """Run a shell command (printing it) and raise on error."""
    print(f"\n>>> Running: {' '.join(cmd)}")
    subprocess.run(cmd, check=True, cwd=cwd)

def extract_and_rename(bag_path: str, topic: str, out_name: str):
    """
    Extract one trajectory topic from a ROS2 bag into TUM format,
    using evo_traj bag2 <bag> <topic> --save_as_tum in a temp folder,
    then move the single .tum file back to cwd with name out_name.
    """
    with tempfile.TemporaryDirectory() as td:
        # 1) Run evo_traj bag2 <bag> <topic> --save_as_tum in td
        cmd = ["evo_traj", "bag2", bag_path, topic, "--save_as_tum"]
        run_cmd(cmd, cwd=td)

        # 2) Find the generated .tum file
        tum_files = glob.glob(os.path.join(td, "*.tum"))
        if len(tum_files) == 0:
            raise RuntimeError(f"No .tum file generated for topic {topic}")
        if len(tum_files) > 1:
            print("Warning: multiple .tum files found; picking the first one:")
            print("\n".join(tum_files))
        src = tum_files[0]

        # 3) Move/rename it back to the caller's cwd
        shutil.move(src, out_name)
        print(f">>> {topic} → {out_name}")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 evaluate_fusion.py <ros2_bag2_path>")
        sys.exit(1)
    bag_path = sys.argv[1]

    fused_tum = "fused.tum"
    raw_tum   = "raw.tum"
    gt_tum    = "gt.tum"

    # 1) Extract filtered odometry and raw odometry
    try:
        extract_and_rename(bag_path, "/odometry/filtered", fused_tum)
    except Exception as e:
        print(f"ERROR extracting /odometry/filtered: {e}")
        sys.exit(1)

    try:
        extract_and_rename(bag_path, "/odom_cov", raw_tum)
    except Exception as e:
        print(f"ERROR extracting /odom_cov: {e}")
        sys.exit(1)

    

    # 2) Try to extract ground truth (may be missing or unsupported)
    has_gt = False
    try:
        extract_and_rename(bag_path, "/ground_truth", gt_tum)
        has_gt = True
    except subprocess.CalledProcessError:
        # evo_traj threw an error (e.g. unsupported message type)
        print("\n>>> '/ground_truth' unsupported or missing; skipping ATE & RPE.")
    except RuntimeError as e:
        # our extractor logic failed to find a .tum
        print(f"\n>>> Skipping GT: {e}")

    # 3) If we have GT, run APE & RPE
    if has_gt:
        run_cmd([
            "evo_ape", "tum", gt_tum, fused_tum,
            "-p", "-a", "--save_plot", "ate_fused.png"
        ])
        run_cmd([
            "evo_ape", "tum", gt_tum, raw_tum,
            "-p", "-a", "--save_plot", "ate_raw.png"
        ])
        run_cmd([
            "evo_rpe", "tum", gt_tum, fused_tum,
            "-p", "-a", "--delta", "1", "--delta_unit", "m",
            "--save_plot", "rpe_fused.png"
        ])
        run_cmd([
            "evo_rpe", "tum", gt_tum, raw_tum,
            "-p", "-a", "--delta", "1", "--delta_unit", "m",
            "--save_plot", "rpe_raw.png"
        ])

    # 4) Finally, compare raw vs fused if both exist
    if os.path.exists(raw_tum) and os.path.exists(fused_tum):
        run_cmd([
            "evo_traj", "tum", raw_tum, fused_tum,
            "-p", "--plot_mode", "xyz",
            "--save_plot", "compare_raw_fused.png"
        ])
    else:
        print("\n>>> Cannot compare raw vs fused: one of the files is missing.")

    print("\nEvaluation complete.")

if __name__ == "__main__":
    main()
