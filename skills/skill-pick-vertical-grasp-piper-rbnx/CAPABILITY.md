---
description: 纯臂抓取技能：分拣（pick，抓起→放盒）、演示（demo，两点往返）、抓起来并保持握持（grab，目标不在桌面时返回 "not detected"）、把已握持的物体放进盒子（put_in_box）、就地松手（put_down）。
---

# Pick skill（纯臂）

这个技能用竖直向下的夹爪抓取桌面物体。它是 `robonix.skill.pick.vertical_grasp_piper`，
与 S10 版（`skill-pick-vertical-grasp-rbnx`）共用同一套模块结构，差别只有两处部署事实：
相机固定在**顶部**（不是臂上），`grab` 成功后停在**悬停位**（不是回安全位）。

注册的能力：`driver`（生命周期）、`pick`（分拣）、`demo`（两点往返演示）、
`grab`（抓起并保持）、`put_in_box`（放盒）、`put_down`（就地松手）。

## `pick`

用户说"把它放进盒子 / 分拣 / put it in the box"时调用（也可直接说"抓起X"来表达
一次完整分拣）。传用户的原话作为 `object_name`（名字、颜色、形状、任何语言）：

```json
{"object_name": "红色物体"}
```

流程：抓起（与 grab 同一段）→ 搬进盒子 → 释放 → 空手回安全位。**每次只分拣一个**；
目标不在桌面时报 "object not detected"，原样转述给用户。要"抓起来递给我/一直拿着"
请用 `grab`。

## `demo`

用户说"演示一下 / 来回搬几次 / demo"时调用。传 `object_name`；可选
`demo_rounds`（>0 跑固定轮数，0/省略则跑到时间预算用完）：

```json
{"object_name": "胡萝卜", "demo_rounds": 3}
```

每轮重新检测（要知道物体此刻在哪），然后把它送到**离它更远的那个演示点**
（`demo_pos_a` / `demo_pos_b`），于是物体在两点之间往返，动作间隔约 3 秒。

## `grab`

Call when the user wants the object picked up and HELD — "grab it / pick it up /
hand it to me / hold it", described by name, color or shape in ANY language
(西红柿 / 红色物体 / the red one / 蓝色方块 / toy carrot / potato...).
Pass the user's description as `object_name`:

```json
{"object_name": "红色物体"}
```

The arm grasps the matched object and keeps holding it, **stopping at the hover
pose above the grasp point** (gripper closed, no lateral motion). If the target
is NOT on the table, the tool reports "object not detected" — relay that to the
user as-is (e.g. "没有蓝色方块") and grasp nothing.

Do NOT infer what is on the table from this manual: only the call result says
what is actually there.

The object stays in the gripper until the user asks for it to be released, so
follow up with `put_in_box` (put it in the box) or `put_down` (let go).

## `put_in_box`

Call after a successful `grab` when the user wants the held object put away:
"把东西放到盒子里 / 放进盒子里 / 收起来 / put it in the box". Carries the
held object over the box, releases it there, and returns home empty-handed.
Request: `{}`.

## `put_down`

Call when the user wants the held object released WHERE IT IS — "put it down /
release it / let go / 放下 / 松手". Opens the gripper in place with zero arm
motion (reads the live arm joint feedback and republishes it unchanged, only the
fingers widen). Request: `{}`.
