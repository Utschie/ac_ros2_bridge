# 语义规则文件（semantic_annotation 中的 json）

`ac_simulator_suite\semantic_annotation\` 目录下的 json 文件是**按赛道生效的
"物体 → 类别 / 剔除"规则真值**：标注与场景编辑工具（Annotator）的保存产物，
也可以手工编辑。游戏画面、语义相机模态与雷达点云都由它驱动。

## 文件命名与回退链

- `<赛道>__<布局>.json` —— 布局级（最优先），如 `ks_nordschleife__endurance.json`；
- `<赛道>.json` —— 赛道级（全部布局共用）；
- `default.json` —— 通用回退（随包附带）。

进入赛道时自动加载；链上命中的第一个文件**整文件生效**。保存 ini 约
1–2 秒后热重载（游戏开着时），无需重启。

## 文件结构速览

```json
{
  "meta": { "track": "ks_barcelona", "config": "layout_gp",
            "note": "人读备注（加载器不消费，标注工具往返保留）" },
  "classes": {
    "TRACK":  { "exact":  ["Object061"],
                "prefix": ["LOFT"],
                "names":  ["ASPH", "LINE"],
                "regex":  ["^TRM-NRM-\\d+$"],
                "note":   "…" },
    "GRASS":  { "names": ["TERRAIN"], "prefix": ["GR_"] }
  },
  "path":  [ { "contains": "PEOPLE.FBX", "class": "PERSON" } ],
  "tree":  [ { "ancestor": "BLURRED", "class": "PERSON" } ],
  "discard": { "exact": ["OBJECT061"], "names": ["SMOKE"],
               "regex": ["RIM_BLUR_.*"] },
  "lodGroups": [ { "key": "^(.+?)_(?:HI|LOW)_(\\d+)$" } ]
}
```

## 段落说明

| 段 | 作用 |
|---|---|
| `meta` / 各段 `note` | 人读备注；标注工具保存时原样保留 |
| `classes.<类名>.exact` | 节点全名相等（最优先）——标注工具 Save 写的就是这段 |
| `classes.<类名>.prefix` / `names` | 节点名前缀 / 名字子串 |
| `path` / `tree` | 按完整路径子串 / 按祖先节点名命中整棵子树 |
| `regex` | 正则兜底段（`std::regex`，不区分大小写） |
| `discard` | 剔除段（与类别无关）：命中的物体从游戏画面、相机、雷达和碰撞中全部消失——工具里右键 Discard 写的就是这段 |
| `lodGroups` | LOD 档联动分组（标注工具用；游戏侧忽略） |

匹配优先级：`exact` > `prefix` > `path` > `names` > `tree` > `regex`；
全部未命中 → 车辆子树兜底 → UNKNOWN。

## 手工编辑注意

- 类名必须在 `configs\classes.json` 词表内（未知类名告警跳过）；
- 正则里的 `\d` 要写成 `\\d`（JSON 转义一层、正则再吃一层）；
- 写错的段告警跳过、不阻塞整个文件加载；
- 与标注工具可混用：工具只增删 `exact`/`discard` 条目，你的手编段落与
  `note` 不会被覆盖。

标注操作方法见 [annotator.md](annotator.md)；渲染开关见
[configs/semantic_classes.md](configs/semantic_classes.md)。
