"""每 5 分鐘回報用：批次進度、上次回報以來的新錯誤、新完成的趟、GPU。"""
import datetime as dt, json, pathlib, re, subprocess, sys
sys.path.insert(0, "/home/aa/IsaacLab/sim_ws/scripts")
WS = pathlib.Path("/home/aa/IsaacLab/sim_ws")
STATE = pathlib.Path(__file__).with_name("progress_state.json")
from run_layout import run_dirs
from make_readme import parse_nav_table

st = json.loads(STATE.read_text()) if STATE.exists() else {"batch_lines": 0, "abl_lines": 0, "seen": []}
now = dt.datetime.now()
alive = subprocess.run(["pgrep", "-f", "overnight\\.s[h]"], capture_output=True).returncode == 0
ov = max((WS / "reports").glob("overnight*.log"), key=lambda q: q.stat().st_mtime).read_text(errors="replace").splitlines()
stage = next((l for l in reversed(ov) if "═══" in l), "?")
finished = any("夜間排程全部完成" in l for l in ov[-40:])

roots = [WS / "recordings"] + sorted(p for p in (WS / "recordings_abl").glob("對照*") if p.is_dir())
done = []
for r in roots:
    for d in run_dirs(r):
        done.append((r.name, d))
n_main = sum(1 for r, _ in done if r == "recordings")
n_abl = len(done) - n_main

# 所有 batch.log（主批次 + 三組對照各一份）裡的新行
def new_lines(path, key):
    if not path.exists():
        return []
    ls = path.read_text(errors="replace").splitlines()
    k = st.get(key, 0)
    if k > len(ls):
        k = 0
    st[key] = len(ls)
    return ls[k:]
logs = {"主": WS / "recordings/batch.log"}
for p in sorted((WS / "recordings_abl").glob("對照*/batch.log")):
    logs[p.parent.name[:3]] = p
fresh = []
for tag, p in logs.items():
    fresh += [(tag, l) for l in new_lines(p, f"lines_{tag}")]
errs = [f"[{t}] {l}" for t, l in fresh if re.search(r"⚠|失敗|FAIL|Traceback|中斷|error", l, re.I)]
cur = next((l for t, l in reversed(fresh) if "════" in l), None)
last = fresh[-1][1] if fresh else "（5 分鐘內沒有新行）"

from scene_counts import counts_from_log
def _counts(d):
    f = d / "isaac_nav.log"
    return counts_from_log(f.read_text(errors="replace")) if f.exists() else None

# 新完成的趟
newly = []
for root, d in done:
    key = f"{root}/{d.name}"
    if key in st["seen"]:
        continue
    st["seen"].append(key)
    r = json.loads((d / "run.json").read_text())
    legs = parse_nav_table((d / "nav.log").read_text(errors="replace")) if (d / "nav.log").exists() else []
    col = r.get("collisions") or {}
    det = col.get("detector", {})
    newly.append({"組": root, "趟": r["tag"],
                  "抵達": f"{sum(l['result'] == 'OK' for l in legs)}/{len(legs)}",
                  "各段秒數": [round(l["seconds"]) for l in legs],
                  "擦撞": col.get("episodes"), "擦撞類別": col.get("by_category"),
                  "偵測器ok": det.get("overlap_ok"), "對照": det.get("positive_controls"),
                  "靜動": _counts(d)})
    # dynamic 必須完全沒有靜態物 —— 這是這次重錄的主因之一
    c = _counts(d)
    if r.get("scenario") == "dynamic" and c and c[0] != 0:
        errs.append(f"[{root}] {r['tag']} 是 dynamic 卻有 靜{c[0]}（站立人物沒被停用？）")
    if legs and sum(l["result"] == "OK" for l in legs) < len(legs):
        errs.append(f"[{root}] {r['tag']} 只抵達 {sum(l['result'] == 'OK' for l in legs)}/{len(legs)}")
    if col and not det.get("overlap_ok"):
        errs.append(f"[{root}] {r['tag']} 擦撞偵測器失效：{det}")

# bag 收尾：跑完的趟（第二遍已結束）bag 應該已有 metadata.yaml。
# 當下還沒有的記起來，下次再看；過了一整趟還沒有才算真的壞掉。
pending = st.setdefault("bag_pending", {})
bag_bad = []
for root, d in done:
    key = f"{root}/{d.name}"
    b = next((x for x in (d / "bag").iterdir() if x.is_dir()), None) if (d / "bag").exists() else None
    ok = bool(b and (b / "metadata.yaml").exists())
    if ok:
        pending.pop(key, None)
    else:
        pending[key] = pending.get(key, 0) + 1
        if pending[key] >= 3:              # 連續 3 次回報（約 15 分鐘）都沒收好
            bag_bad.append(d.name)

# 目前那一趟的 Isaac 有沒有當掉
crash = []
act = max((p for p in WS.glob("recordings*/**/isaac_nav.log") if "/_" not in str(p)),
          key=lambda p: p.stat().st_mtime, default=None)
if act and (now.timestamp() - act.stat().st_mtime) < 900:
    for l in act.read_text(errors="replace").splitlines()[-400:]:
        if re.search(r"Traceback|terminate called|⚠", l):
            crash.append(l.strip()[:160])

gpu = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total",
                      "--format=csv,noheader"], capture_output=True, text=True).stdout.strip()
apps = subprocess.run("nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader | "
                      "while read -r p m; do ps -p ${p%,} -o user=,cmd= 2>/dev/null | cut -c1-60 | "
                      "sed \"s/^/$m  /\"; done | grep -v gnome-remote", shell=True,
                      capture_output=True, text=True).stdout.strip().splitlines()

# 預估：用本批次（23:02 之後）已完成趟的平均時間
t0 = dt.datetime(2026, 9, 23, 23, 2, 5)
fin = [d for _, d in done if dt.datetime.fromtimestamp((d / "run.json").stat().st_mtime) > t0]
eta = ""
if fin:
    per = (now - t0).total_seconds() / len(fin)
    rest = 72 - n_main
    eta = f"平均 {per/60:.1f} 分/趟，剩 {rest} 趟 → 約 {(now + dt.timedelta(seconds=per*rest)).strftime('%m-%d %H:%M')} 完成"

STATE.write_text(json.dumps(st, ensure_ascii=False))
print(json.dumps({
    "時間": now.strftime("%H:%M"), "排程存活": alive, "全部完成": finished, "階段": stage,
    "主批次": f"{n_main}/72", "預估": eta,
    "目前": cur, "最新一行": last,
    "新完成": newly, "新錯誤": errs, "Isaac 當機跡象": crash[-5:],
    "bag 等待收尾": sorted(k.split("/")[-1] for k in pending),
    "bag 確定壞掉": bag_bad,
    "GPU": gpu, "別人的程序": apps,
}, ensure_ascii=False, indent=1))
