# Yimu hand, second generation ("shou17"), vendored

Supplied by the vendor on 2026-09-21 as two SolidWorks "export as URDF" drops:

| directory | hand | vendor's name |
|---|---|---|
| `shou17-you-0921/` | **right** | 右 (*you*) |
| `shou17-zuo0921/` | **left** | 左 (*zuo*) |

This is a different hand from `third_party/yimu/` ("shou20260727-22"), not a
re-export of it: every one of the 44 STLs differs, and the geometry comparison
is in `models/hands/yimu_v2_hand_notes.md` §1. It shares the vendor, the
22-link / 21-joint topology and the `<finger>-<link>` naming, which is why the
port could be structural rather than from scratch — and also why it is easy to
mistake one generation's constants for the other's.

## What is here

| | |
|---|---|
| `shou17-{you,zuo}-0921/urdf/*.urdf` + `*.csv` | Vendor URDF and SolidWorks mass-properties report, **byte-identical to what was supplied**. |
| `DEMO2手指耦合角度关系表20260922.xlsx` | The four fingers' PIP→DIP linkage, 119 rows, plus the hand's DOF count and total mass in its header. Supplied 2026-09-22. |
| `微型关节模组_CB-1010-PW-2规格书.pdf` | Datasheet for the joint module every actuated joint uses. Supplied 2026-09-22. |
| `shou17-{you,zuo}-0921/meshes/` | Vendor STL, byte-identical, 22 per side. Mesh names collide between the sides (both have `1-1.STL`); only ever one side is loaded per model. |
| `make_pristine.py` | Generates the two files below from the two URDFs. Read its docstring before touching either. |
| `yimu_v2_hand.xml`, `yimu_v2_hand_left.xml` | **Generated. Do not hand-edit** — re-run the generator. The pristine, hand-only MJCFs `models/hands/yimu_v2_spec.py` loads. |

Also present on disk but **gitignored** (see `.gitignore`): the ROS scaffolding
the exporter writes around the model — `CMakeLists.txt`, `package.xml`,
`launch/`, `config/`, empty `textures/`, and a 2.9 MB `export.log` per side.
Nothing in this repo reads any of it, and `third_party/yimu/` was vendored
without it too. Kept on disk rather than deleted so the drop stays as
delivered; pruning it is a separate call.

Everything this project adds on top of the generated pair — the wrist joints,
the forearm DOFs, the forearm visual, per-link collision primitives, pad sites,
contact parameters, the ready-pose keyframe — is built programmatically in
`models/hands/yimu_v2_spec.py` and touches no file in this directory.

The forearm/wrist visual meshes are NOT copied here: the shared arm
(`models/hands/arm.py`) points straight at
`third_party/shadow_hand/assets/{forearm_1,wrist}.obj`, one copy of a borrowed
cosmetic asset for all five hands.

## 1. Vendor caveats the loader acts on

1. **The URDFs carry no actuation spec** — `effort="0" velocity="0"` on all 21
   joints, which MuJoCo reads as *unlimited*, not as zero torque (checked on
   the compiled model: `jnt_actfrclimited` is False for all 21). **Superseded
   2026-09-22** by the CB-1010-PW-2 datasheet; see §5. Until that arrived the
   hand ran with no torque clamp at all, which is worse than v1's ±1 N·m CAD
   placeholder.
2. **No joint damping, frictionloss or armature**, and no actuators. The
   generator adds shadow's own default joint class values (damping 0.05,
   frictionloss 0.01) and one `<position kp="2">` per joint;
   `yimu_v2_spec._FINGER_JOINT_ARMATURE` adds the armature, which is the third
   value of that same shadow class and the one three previous ports each
   dropped.
3. **The collision geometry is the visual STL**, used directly, which
   self-penetrates by centimetres and jitters. The pristine files therefore
   disable contacts; `yimu_v2_spec._add_collision_geoms` fits real primitives
   and clears the flag. Loading either pristine file alone is for *looking* at
   the hand.
4. **The left hand's joint signs are not in mirror convention** — six of them.
   See §3.
5. **Four links' declared masses disagree between the sides** while their
   geometry does not. See §4.
6. **The link masses are shell masses**, and the whole hand comes to 92.1 g
   against the vendor's own 720 g. See §5.
7. **The four fingers' DIP are underactuated.** See §5.

## 2. Is the vendor's left really a mirror of its right?

Yes, to a few millimetres — measured, because the whole port assumes it. The
two are different CAD files with different tessellations (57 053 vs 57 218 palm
vertices), so the comparison is nearest-neighbour distance from the
Y-reflected right hand to the left, per link, on the compiled models:

| link | p50 | p95 | max |
|---|---|---|---|
| palm (`base_link`) | 0.02 mm | 0.79 mm | 5.70 mm |
| thumb `1-1` … `1-5` | 0.00 – 1.19 mm | 0.64 – 4.47 mm | 2.69 – 6.07 mm |
| fingers `2-*` … `5-*` | 0.02 – 0.10 mm | 0.30 – 1.07 mm | 1.47 – 5.55 mm |

Median p95 across all 22 links is 0.91 mm; the worst is the thumb's `1-4` at
4.47 mm. The `*_fingertip` sites, which is what the reward actually reads,
mirror to **0.05 mm on the four fingers and 1.58 mm on the thumb**.

So this is a genuine mirror pair and both sides are used as shipped. Contrast
`third_party/yimu/`, where the vendor supplied only a right hand and
`make_left.py` generates the left as an exact Y-plane reflection.

## 3. The six left-hand joint axes the generator negates

`1-1`, `1-2`, `2-1`, `3-1`, `4-1`, `5-1`.

The repo's mirror relies on an invariant the vendor does not provide: **the
same joint vector must mean the mirrored pose on both hands**, because
`rl/hsd/mirror.py` presents the left hand to the policy as a right hand and
passes the hand joints through untouched. When that was wrong for yimu and
sharpa it cost the entire left hand — note-onset F1 exactly 0.000 on all 15
PIG_DEV songs.

The criterion is exact. A reflection reverses rotational handedness,
`M_y R(a, θ) M_y = R(M_y a, −θ)`, so for one qpos to mean mirrored motion the
left hand's **world** joint axis must be `−M_y(a_right)`. On the two compiled
vendor models at qpos = 0 (`data.xaxis`), all 21 axis pairs came out exactly
parallel or exactly antiparallel to `M_y a_right` — no ambiguity, no residual —
and those six are the wrong one.

Cross-checked against motion rather than algebra, driving each joint to 60 % of
its range one at a time and comparing the distal link's displacement against
its mirror: the flip cuts the mismatch from 154.9 mm to 1.3 mm on `1-2` and
from 42.7 mm to 0.0 mm on `2-1`…`5-1`, while the 15 unflipped joints already
sit at 0.0–2.9 mm. `1-1` moves the thumb tip only 8.6 mm over its whole range,
inside the assets' own 2–6 mm mirror tolerance, so the algebra is what decides
it there.

After the flip, **all 21 joint ranges are identical between the two sides**
(`1-2`'s `(0, +1.6581)` becomes `(−1.6581, 0)`, matching the right; the other
five were already symmetric).

## 4. The mass disagreements, and what is done about them

Four links declare different masses on the two sides while their meshes are the
same shape. Volumes below are MuJoCo's own integration of each mesh:

| link | right | left | mass ratio | volume ratio |
|---|---|---|---|---|
| `base_link` | 46.894 g | 32.089 g | 0.684 | 0.977 |
| `1-2` | 1.790 g | 2.142 g | 1.197 | 1.016 |
| `1-4` | 2.293 g | 3.696 g | 1.612 | 1.004 |
| `1-5` | 1.759 g | 1.774 g | 1.009 | 1.007 |

The other 18 links' masses are byte-identical between the sides. A mass that
contradicts its own geometry is a CAD bookkeeping difference — a different
component set assigned in SolidWorks — not a physical one.

* **The palm is corrected.** `yimu_v2_spec._BASE_LINK_INERTIAL` carries the
  RIGHT hand's CAD inertial and `_mirror_inertial` reflects it onto the left.
  The palm's inertial has to be restored explicitly anyway (the URDF root has
  no owning `<body>`, so `MjSpec.attach()` re-parents its geom and drops the
  inertial; without the restoration MuJoCo auto-integrates **1.1486 kg**
  against the real 46.9 g), so this costs nothing extra. Both palm frames are
  exact mirrors, so the transform is exact.
* **The three thumb links are left alone**, and recorded rather than silently
  fixed: their body frames are not plain mirrors of the right's, so "the
  right's value mirrored" would need a frame transform this repo would have to
  invent. The residual is 1.4 g at worst and 13 g over the whole hand, against
  the 3.794 kg `forearm_dofs.CARRIAGE_LOAD_KG` the forearm is ballasted to —
  0.04 % and 0.34 % of what the carriage holds.

## Regenerating

    cd pv/models/hands/third_party/yimu_v2
    python make_pristine.py            # rewrites both pristine MJCFs
    python make_pristine.py --check    # CI-style: fails if they are stale

The assembled-spec cache (`models/hands/spec_cache.py`) lists both pristine
XMLs, `yimu_v2_spec.py` and `make_pristine.py` as sources, so it invalidates
itself; there is no separate cache step to remember.


## 5. What the 2026-09-22 documents changed

Two files arrived after the port shipped, and between them they settle three
things the CAD could not.

### 5.1 Seventeen motors, twenty-one joints

The coupling table's header says **共17个自由度** — 17 actuated DOF — and the
table itself is why: each finger's **DIP carries no motor** and follows its own
PIP through a mechanical linkage. 5 thumb joints + AA/MCP/PIP on four fingers
= 17.

The relationship is strongly saturating, `dDIP/dPIP` running 0.95 at full
extension down to ~0 past 115°, so it is not a ratio:

| PIP | 0° | 15° | 30° | 60° | 90° | 118° |
|---|---|---|---|---|---|---|
| DIP | 0.00° | 13.64° | 25.75° | 45.40° | 58.90° | 63.22° |

`yimu_v2_spec` models it as MuJoCo's quartic joint equality and deletes the
four DIP actuators. `nu` drops by 4; the joints stay, so the fingertips still
move, collide and carry their pad sites. Two measurements decided the details:

* **A tendon would not do.** Fitting the table linearly — which is all a fixed
  tendon can express, and what yimu v1's `pip_dip_coupled` uses — is 13.84° out
  at worst. The quartic is 0.12°.
* **MuJoCo's default `solimp` would not do either.** An equality constraint is
  SOFT, and on the defaults this one holds only to **9.90°** — a linkage with a
  finger joint's worth of slop, which would have shipped looking like a
  mechanism. At `solref=(0.01, 1)`, `solimp=(0.9999, 0.9999, 1e-6)` it holds to
  **0.011°**, and still does at 4 solver iterations.

Two knock-on facts, both measured rather than assumed:

* The **ready pose had to be re-swept**. At PIP 55° the linkage puts the DIP at
  42.5°, so the 2026-09-21 entry (25, 55, 20) was no longer a reachable pose.
  (30, 35) replaces it — `models/hands/ready_pose.py` carries the sweep.
* The **hsd reset had to learn about dependent joints**. Its
  `reset_joint_noise` perturbs every joint column independently, so it wrote a
  posture up to **7.20°** (median 4.60°) off the curve at every episode start.
  `rl/hsd/commands._project_dependent_joints` now puts equality-driven joints
  back on their own curve after the noise — read off the compiled model's
  equality table, so it is hand-agnostic and exactly a no-op on the four hands
  with none.

### 5.2 The motor: CB-1010-PW-2

An integrated motor + gearbox + driver + encoder module, 10 × 10 × 33.5 mm,
**20 g**, gear ratio **568**, from 深圳星际光年 / Stella Robot.

| | |
|---|---|
| rated torque | **0.16 N·m** |
| stall torque | 0.45 N·m |
| rated speed | 33 rpm = **3.456 rad/s** |
| no-load speed | 56 rpm = 5.864 rad/s |
| rated / max power | 4.6 W / 7.2 W at 12 V |
| position loop | 200 Hz, 0.01° resolution |

`build_yimu_v2_hand` writes the **rated** torque onto all 17 hand actuators'
`forcerange` by default (`motor_torque_nm=None` restores the old unlimited
behaviour; the stall figure is there as a constant). The wrist keeps the shared
arm's own ±5/±10 N·m — those are not CB-1010 modules.

Measured consequences:

* The hand still sounds a key: 101.7% of key travel at rated, against 102.3%
  unlimited. Peak key velocity 0.576 against 0.631, so the limit costs ~9% of
  press velocity.
* It **binds**: the pressing finger's MCP and PIP both saturate at 0.16 N·m
  during that press, while every other joint stays under 4% of rating.

Against v1, whose vendor figures are 0.08 N·m and 3.3661 rad/s: the new module
is **the same joint speed at twice the torque**. Note the comparison hazard —
v1's MJCF still carries its ±1 N·m CAD placeholder, 12.5× its own real spec, so
a v1-vs-v2 run now differs in actuation as well as geometry unless it passes
`motor_torque_nm=None`.

**Speed is recorded but NOT enforced.** `models/hands/joint_speed_limits.py`
can cap it by deriving damping, and is used by the keyboard task — but no
piano-task hand enforces a speed limit, so turning it on here alone would make
this the only hand in the comparison with one. The constant is in
`yimu_v2_constants` and `press_capability.speed_limit` now returns it, so the
probe reports are real rather than upper bounds; putting it in the model is a
separate, deliberate decision.

### 5.3 Seven hundred and twenty grams

The header also says **单手总重约720g**, against the URDF link masses' own
**92.1 g** — the hand was 7.8× light. Reverse-engineering the CAD's implied
density gives 155–1001 kg/m³ with no consistent material, i.e. shell masses at
a default density with the 17 modules, gearboxes, wiring and fasteners absent
from the tree entirely. (v1 has the same disease in a tidier form: its 22 links
all imply 968–1009 kg/m³, SolidWorks' default ≈ water.)

`_apply_link_mass_model` puts the total back: 20 g at each of the 17 actuated
links, and the CAD shells scaled uniformly (×4.13) to make up the rest. The one
assumption is the placement — every actuated link is large enough to contain a
33.5 mm module, but no drawing here shows it — and it is worth little: the
alternative of scaling every link uniformly was built and compared, and the
distribution matters far less than the 7.8× total did.

The forearm is ballast, so the carriage still totals `CARRIAGE_LOAD_KG` and
only the **distribution** moves. That is the part that was wrong: what the
wrist holds went from 192 g to 820 g.
