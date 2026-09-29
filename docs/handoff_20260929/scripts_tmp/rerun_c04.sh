#!/bin/bash
# 重錄 Character_04 在場的 24 趟（身體偏離根節點 5.3 m，6f68ed8 已修）
cd /home/aa/IsaacLab/sim_ws
say(){ echo "[$(date '+%F %T')] $*"; }
A="$PWD/recordings/_舊版與作廢錄影_保留不刪/4_作廢_有bug或失敗_勿引用/5_Character04身體偏移5m_鬼影_0924"
mkdir -p "$A"
TAGS=$(python3 - <<'PY'
import json,glob,os
for f in sorted(glob.glob('recordings/模型*/*/scene.json')):
    if any(c['name']=='Character_04' for c in json.load(open(f))['characters']):
        print(os.path.basename(os.path.dirname(f)))
PY
)
N=$(echo "$TAGS" | wc -l); say "要重錄 $N 趟"
for t in $TAGS; do m=${t%%_*}; mv "recordings/模型$m/$t" "$A/"; done
i=0
for t in $TAGS; do
  i=$((i+1)); say "($i/$N) $t"
  ONLY="$t" bash scripts/record_batch.sh
done
source ./setup_sim_env.sh >/dev/null 2>&1
say "收尾分析"
python3 scripts/make_sync.py recordings 2>&1 | grep -vE "^\[(WARN|INFO)\]" | tail -1
python3 scripts/localization_error.py recordings 2>&1 | grep -vE "^\[(WARN|INFO)\]" > reports/localization_error.txt; tail -3 reports/localization_error.txt
python3 scripts/check_recordings.py recordings > reports/recordings_health.txt 2>&1; tail -1 reports/recordings_health.txt
tmp=$(mktemp); python3 scripts/make_readme.py recordings > "$tmp" && [ -s "$tmp" ] && mv "$tmp" recordings/README.md && chmod 644 recordings/README.md && say "README $(wc -l < recordings/README.md) 行"
python3 scripts/make_browse_tree.py >/dev/null 2>&1 && say "中文索引已重建"
say "重錄全部完成"
