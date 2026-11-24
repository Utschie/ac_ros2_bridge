# semantic_classes.ini — 语义渲染开关

语义渲染的全局开关。"节点名 → 类别"的按赛道规则在
`ac_simulator_suite\semantic_annotation\` 的 json 文件里（回退链：
`<track>__<layout>.json → <track>.json → default.json`），进入赛道时自动
加载；标注方法见 [annotator.md](../annotator.md)；规则文件格式见 [annotation-files.md](../annotation-files.md)。

| 键 | 出厂值 | 说明 |
|---|---|---|
| `UNKNOWN_POLICY` | `white` | 未分类像素的显示：`white`（白色，便于发现缺口）或 `discard`。 |
| `[visual_exclusion] ENABLED` | `1` | 被标注工具 Discard 的物体是否从游戏画面与相机中隐藏。 |
| `[collision_exclusion] ENABLED` | `1` | 被 Discard 的物体是否同时从游戏碰撞中移除（改动后重进赛道生效）。 |
