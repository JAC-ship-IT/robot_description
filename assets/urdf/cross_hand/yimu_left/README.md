# yimu_v2 (left hand) -- self-contained MuJoCo model

Open it directly:

    python -m mujoco.viewer --mjcf=yimu_v2_left.xml

Everything in this directory is relative; move or rename the folder freely, but
keep `yimu_v2_left.xml` and `meshes/` together.

## What this is

The Yimu second-generation hand ("shou17"), as modelled for the primavista
piano work. It is NOT the vendor's raw export: the vendor URDF does not load in
MuJoCo at all (ROS `package://` mesh paths), and carries neither the hand's real
mass, nor its motor limits, nor its finger linkage. All of those are baked in
here.

- **22 bodies, 21 qpos, 17 actuators, 4 equality constraints, 44 geoms, 720.0 g.**
- **17 actuated DOF for 21 finger joints.** Each finger's DIP carries no motor;
  it follows that finger's PIP through a mechanical linkage, modelled as a
  quartic `<equality><joint>`. Drive `*-3_act` and the DIP follows -- there is
  deliberately no `*-4_act`.
- **Torque limits** are CB-1010-PW-2's rated 0.16 N*m on every hand
  actuator. The module's stall torque is 0.45 N*m.
- **Speed is NOT limited.** The module is rated 33 rpm (3.456 rad/s) but MuJoCo
  has no joint speed cap; the limit is recorded, not enforced. Any speed this
  model reaches is a property of its torque and damping, not a datasheet claim.
- **Mass** is the vendor's stated 720 g for the hand, distributed as 20 g per
  joint module at the link it drives plus uniformly-scaled CAD shells. The
  vendor's own URDF link masses sum to 92.1 g and omit the motors entirely.
- Collision geometry is PCA-fitted primitives per link, with the five
  fingertips colliding with their full-precision STL. Original STL geoms are kept as visual-only (group 2);
  colliders are group 3.
- Keyframe `open_hand` is the pose the piano task resets to.

The root body is the palm, welded to the world -- pass --free_base for a floating hand.

## What is NOT here

The wrist joints, the forearm and the synthetic carriage DOFs the piano task
adds around this hand -- they position it over a keyboard and are not part of
the hand.

## Provenance and caveats

See `PROVENANCE.md`, copied in beside this file. It records, among other things,
which vendor numbers were corrected and why, and that the left hand's joint
axes needed six sign flips to be in mirror convention with the right.
