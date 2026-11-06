#!/usr/bin/env python3
"""Fetch FOR-instance plot LAS files from Zenodo and convert them to PCD for
scenario_registration.

Streams only the requested members out of the monolithic FORinstance_dataset.zip
(1.6 GB) using HTTP range requests via remotezip — no full-zip download.

Produces two clouds in a shared local frame (a common UTM offset is subtracted):

  - <outdir>/<prefix>_ref.pcd : union of --ref-plots (the reference map, PCref)
  - <outdir>/<prefix>_uav.pcd : single --uav-plot (the UAV survey patch, PCUAV)

The default selection is the NIBIO boreal-forest site (Norway, UTM 32N, Riegl
VUX UAV laser scanning): plots 17/18/19 form a contiguous ~30x76 m strip and
plot_19 lies inside it, giving a real PCUAV-inside-PCref setup.

Dependencies (not needed at runtime, only for this script):

    pip install remotezip laspy numpy

Example:

    python3 scripts/fetch_benchmark_data.py \
        --ref-plots 17 18 19 --uav-plot 19 --voxel 0.05
"""

import argparse
import io
import sys

import numpy as np

ZENODO_URL = (
    "https://zenodo.org/api/records/8287792/"
    "files/FORinstance_dataset.zip/content"
)
COLLECTION = "NIBIO"


def read_las_xyz(buf: bytes) -> np.ndarray:
    import laspy

    las = laspy.read(io.BytesIO(buf))
    return np.column_stack([las.x, las.y, las.z])


def voxel_thin(xyz: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0:
        return xyz
    keys = np.floor(xyz / voxel)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return xyz[np.sort(idx)]


def write_pcd_binary(path: str, xyz: np.ndarray) -> None:
    n = xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(xyz.astype("<f4").tobytes())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref-plots", type=int, nargs="+", default=[17, 18, 19],
                    help="FOR-instance NIBIO plot numbers merged into the reference cloud")
    ap.add_argument("--uav-plots", type=int, nargs="+", default=[19],
                    help="NIBIO plot numbers merged into the UAV observation cloud")
    ap.add_argument("--voxel", type=float, default=0.05,
                    help="voxel-grid edge (m) for thinning; 0 keeps all points")
    ap.add_argument("--outdir", default="data/benchmarks")
    ap.add_argument("--prefix", default="nibio")
    args = ap.parse_args()

    import remotezip

    print(f"Opening {ZENODO_URL}")
    rz = remotezip.RemoteZip(ZENODO_URL)
    wanted = {n: f"{COLLECTION}/plot_{n}_annotated.las"
              for n in set(args.ref_plots) | set(args.uav_plots)}
    members = {i.filename: i for i in rz.infolist()}
    for n, m in wanted.items():
        if m not in members:
            sys.exit(f"member not found in archive: {m}")
        print(f"  {m}: {members[m].file_size/1e6:.0f} MB "
              f"({members[m].compress_size/1e6:.0f} MB transfer)")

    clouds = {}
    for n, m in sorted(wanted.items()):
        print(f"Streaming {m} ...", flush=True)
        with rz.open(m) as f:
            buf = f.read()
        xyz = read_las_xyz(buf)
        thinned = voxel_thin(xyz, args.voxel)
        print(f"  {xyz.shape[0]} pts -> {thinned.shape[0]} after {args.voxel} m thinning")
        clouds[n] = thinned

    ref = np.vstack([clouds[n] for n in args.ref_plots])
    uav = np.vstack([clouds[n] for n in args.uav_plots])
    offset = ref.min(axis=0)
    ref -= offset
    uav -= offset
    print(f"local offset (subtract to restore UTM 32N + ellipsoidal height): {offset}")

    import os
    os.makedirs(args.outdir, exist_ok=True)
    ref_path = f"{args.outdir}/{args.prefix}_ref.pcd"
    uav_path = f"{args.outdir}/{args.prefix}_uav.pcd"
    write_pcd_binary(ref_path, ref)
    write_pcd_binary(uav_path, uav)
    print(f"wrote {ref_path}: {ref.shape[0]} pts, "
          f"bbox {ref.min(0).round(1)} .. {ref.max(0).round(1)}")
    print(f"wrote {uav_path}: {uav.shape[0]} pts, "
          f"bbox {uav.min(0).round(1)} .. {uav.max(0).round(1)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
