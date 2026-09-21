"""啟用 omni.anim.people，讓 People 角色真的走路（執行期設定）。

配方沿用專案既有可行版本 `/home/aa/Ros/charge_rl/setup_pedestrians_complete.py`。
兩個容易漏掉、漏了就「角色站著不動」的步驟：

1. **要寫 GoTo 命令檔**。omni.anim.people 的行為腳本是讀設定指到的命令檔，
   沒有命令就只會播 idle。
2. **要 Stop → Play 一次**。行為腳本在 play 的當下初始化；直接 play 而沒有
   先 stop，腳本不會掛上去。實測漏掉這步時骨骼只動 0.28 mm、根節點位移 0。

⚠ navmesh 一律關閉：3F 走廊的門與隔牆把 NavMesh 切成不連通區域，開著會讓
跨區域路徑規劃失敗（專案既有結論）。
"""

from __future__ import annotations

import glob

#: IRA 的預設命令檔位置（版本號會變，用 glob 找）。
_CMD_GLOB = ("/home/aa/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/"
             "isaacsim/extscache/isaacsim.replicator.agent.core-*/config/default_command.txt")


def find_command_file() -> str | None:
    hits = sorted(glob.glob(_CMD_GLOB))
    return hits[-1] if hits else None


def write_goto_commands(path: str, walks) -> int:
    """寫入 GoTo 命令。``walks`` 為 ``(角色名, [(x, y), ...])``。

    Returns: 寫入的命令列數。
    """
    lines = []
    for name, pts in walks:
        for x, y in pts:
            lines.append(f"{name} GoTo {x:.2f} {y:.2f} 0 _")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return len(lines)


def setup(stage, character_root: str = "/World/Characters", walks=None):
    """套用動畫圖、行為腳本與命令檔。回傳 ``(角色數, 命令列數, 訊息)``。"""
    import carb
    from isaacsim.replicator.agent.core.settings import (AssetPaths,
                                                         BehaviorScriptPaths,
                                                         PrimPaths)
    from isaacsim.replicator.agent.core.stage_util import (CharacterUtil,
                                                           populate_anim_graph)

    bp = stage.GetPrimAtPath(PrimPaths.biped_prim_path())
    if not (bp and bp.IsValid()):
        return (0, 0, "找不到 Biped_Setup")
    populate_anim_graph()
    graph = CharacterUtil.get_anim_graph_from_character(bp)
    if not graph:
        return (0, 0, "取不到 animation graph")

    chars = CharacterUtil.get_characters_in_stage()
    CharacterUtil.setup_animation_graph_to_character(chars, graph)
    CharacterUtil.setup_python_scripts_to_character(
        chars, BehaviorScriptPaths.behavior_script_path())

    n_cmd = 0
    cmd = find_command_file()
    if cmd and walks:
        n_cmd = write_goto_commands(cmd, walks)

    s = carb.settings.get_settings()
    s.set("/persistent/exts/omni.anim.people/character_prim_path", character_root)
    s.set("/exts/omni.anim.people/command_settings/command_file_path", cmd or "")
    s.set("/exts/omni.anim.people/navigation_settings/navmesh_enabled", False)
    s.set("/exts/omni.anim.people/command_settings/number_of_loop", "inf")
    return (len(chars), n_cmd, f"{len(chars)} 角色 / {n_cmd} 條 GoTo")


def restart_for_behavior_scripts(sim, app, settle_frames: int = 10) -> None:
    """Stop → Play，讓行為腳本初始化。漏掉這步角色就站著不動。"""
    sim.stop()
    for _ in range(settle_frames):
        app.update()
    sim.play()
    for _ in range(settle_frames):
        app.update()
