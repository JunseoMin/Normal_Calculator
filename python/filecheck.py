#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import csv
import argparse
from typing import List, Tuple, Set

RE_GRAY_REL = re.compile(r'^gray/([0-9]{10,})\.png$')
RE_DEPTH_REL = re.compile(r'^depth/([0-9]{10,})\.pcd$')

def parse_time_sync(txt_path: str) -> List[Tuple[int, int, str, str]]:
    """Return list of (gray_ts, depth_ts, gray_rel, depth_rel)."""
    pairs = []
    with open(txt_path, 'r', encoding='utf-8', errors='ignore') as f:
        for lineno, line in enumerate(f, start=1):
            s = line.strip()
            if not s:
                continue
            toks = s.split()
            if len(toks) != 4:
                raise ValueError(f"[L{lineno}] Expected 4 cols, got {len(toks)}: {s}")
            gts_s, grel, dts_s, drel = toks
            try:
                gts = int(gts_s)
                dts = int(dts_s)
            except Exception:
                raise ValueError(f"[L{lineno}] Non-integer ts: gray_ts={gts_s}, depth_ts={dts_s}")

            mg = RE_GRAY_REL.match(grel)
            md = RE_DEPTH_REL.match(drel)
            if mg and int(mg.group(1)) != gts:
                raise ValueError(f"[L{lineno}] gray rel ts != gray_ts: {grel} vs {gts}")
            if md and int(md.group(1)) != dts:
                raise ValueError(f"[L{lineno}] depth rel ts != depth_ts: {drel} vs {dts}")
            pairs.append((gts, dts, grel, drel))
    return pairs

def scan_dir_for_timestamps(img_dir: str, pcd_dir: str) -> Tuple[Set[int], Set[int]]:
    
    """Scan directories and collect timestamps from filenames <ts>.png / <ts>.pcd."""

    def collect(dirpath: str, suffix: str) -> Set[int]:
        if not os.path.isdir(dirpath):
            return set()
        out = set()
        for name in os.listdir(dirpath):
            if not name.endswith(suffix):
                continue
            stem = name[:-len(suffix)]
            if stem.isdigit():
                out.add(int(stem))
        return out
    img_ts = collect(img_dir, ".png")
    pcd_ts = collect(pcd_dir, ".pcd")
    return img_ts, pcd_ts

def write_csv(path: str, rows: List[dict], header: List[str]):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        for r in rows:
            w.writerow(r)

def main():
    ap = argparse.ArgumentParser(description="Sync link checker (txt vs filesystem)")
    ap.add_argument("--txt", default="./data/time_sync.txt")
    ap.add_argument("--imgdir", default="./data/images/")
    ap.add_argument("--pcddir", default="./data/depthcloud/")
    ap.add_argument("--report-prefix", default="sync_check")
    args = ap.parse_args()

    # Parse document
    pairs = parse_time_sync(args.txt)

    # Expected sets (from document)
    expected_img_ts = {gts for gts, _, _, _ in pairs}
    expected_pcd_ts = {dts for _, dts, _, _ in pairs}

    # Detect duplicates (not 1:1 match)
    dup_gray = len(expected_img_ts) != len([g for g,_,_,_ in pairs])
    dup_depth = len(expected_pcd_ts) != len([d for _,d,_,_ in pairs])

    # Scan actual directories
    actual_img_ts, actual_pcd_ts = scan_dir_for_timestamps(args.imgdir, args.pcddir)

    # Compute missing/orphan sets
    missing_imgs = sorted(list(expected_img_ts - actual_img_ts))
    missing_pcds = sorted(list(expected_pcd_ts - actual_pcd_ts))
    orphan_imgs  = sorted(list(actual_img_ts - expected_img_ts))
    orphan_pcds  = sorted(list(actual_pcd_ts - expected_pcd_ts))

    # summary
    print("==== Sync Link Check (TXT vs FS) ====")
    print(f"txt            : {args.txt}")
    print(f"images dir     : {args.imgdir}")
    print(f"depth dir      : {args.pcddir}")
    print(f"#lines in txt  : {len(pairs)}")
    print(f"expected images: {len(expected_img_ts)}  | actual: {len(actual_img_ts)}  | missing: {len(missing_imgs)}  | orphans: {len(orphan_imgs)}")
    print(f"expected pcds  : {len(expected_pcd_ts)}  | actual: {len(actual_pcd_ts)}  | missing: {len(missing_pcds)}  | orphans: {len(orphan_pcds)}")
    if dup_gray:
        print("WARNING: duplicate gray_ts found in txt (not 1:1).")
    if dup_depth:
        print("WARNING: duplicate depth_ts found in txt (not 1:1).")

    # CSV report
    rows_missing = [{"type":"missing_image","ts":ts,"path":os.path.join(args.imgdir, f"{ts}.png")} for ts in missing_imgs] + \
                   [{"type":"missing_pcd","ts":ts,"path":os.path.join(args.pcddir, f"{ts}.pcd")} for ts in missing_pcds]
    rows_orphan  = [{"type":"orphan_image","ts":ts,"path":os.path.join(args.imgdir, f"{ts}.png")} for ts in orphan_imgs] + \
                   [{"type":"orphan_pcd","ts":ts,"path":os.path.join(args.pcddir, f"{ts}.pcd")} for ts in orphan_pcds]
    write_csv(f"{args.report_prefix}_missing.csv", rows_missing, ["type","ts","path"])
    write_csv(f"{args.report_prefix}_orphans.csv", rows_orphan, ["type","ts","path"])

    # Per-line link check (verify file existence)
    line_checks = []
    for idx, (gts, dts, _, _) in enumerate(pairs, start=1):
        gpath = os.path.join(args.imgdir, f"{gts}.png")
        dpath = os.path.join(args.pcddir, f"{dts}.pcd")
        g_ok = os.path.exists(gpath)
        d_ok = os.path.exists(dpath)
        ok = g_ok and d_ok
        line_checks.append({
            "line": idx,
            "gray_ts": gts,
            "depth_ts": dts,
            "img_exists": int(g_ok),
            "pcd_exists": int(d_ok),
            "linked_ok": int(ok),
            "img_path": gpath,
            "pcd_path": dpath
        })
    write_csv(f"{args.report_prefix}_line_checks.csv", line_checks,
              ["line","gray_ts","depth_ts","img_exists","pcd_exists","linked_ok","img_path","pcd_path"])

    # Hint / Conclusion
    print("\nTip:")
    print("- If there is one or more 'orphans', it means there are extra files in the folders that are not listed in the txt file.")
    print("- For the case you mentioned (one more PCD than images), it will appear as orphan_pcds=1 in the summary above.")
    if orphan_pcds:
        print(f"  -> extra PCD timestamps: {orphan_pcds[:10]}{' ...' if len(orphan_pcds)>10 else ''}")

if __name__ == "__main__":
    main()
