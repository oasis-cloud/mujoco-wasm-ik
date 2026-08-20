import loadMujoco from "../dist/mujoco.js";

const xml = `
<mujoco>
  <worldbody>
    <body name="obstacle" pos="0.5 0 0.1">
      <geom name="obstacle" type="box" size="0.05 0.05 0.05"/>
    </body>
    <body name="link" pos="0 0 0.2">
      <joint name="hinge" type="hinge" axis="0 1 0" limited="true" range="-1.57 1.57"/>
      <geom name="link_geom" type="capsule" size="0.02" fromto="0 0 0 0.25 0 0"/>
      <site name="ee" pos="0.25 0 0" size="0.01"/>
    </body>
  </worldbody>
</mujoco>
`;

const eqXml = `
<mujoco>
  <worldbody>
    <body name="link" pos="0 0 0.2">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom size="0.02"/>
    </body>
  </worldbody>
  <equality>
    <joint joint1="hinge" polycoef="0 1 0 0 0"/>
  </equality>
</mujoco>
`;

const mujoco = await loadMujoco();
const model = mujoco.MjModel.from_xml_string(xml);
const data = new mujoco.MjData(model);
const ik = new mujoco.IkConfiguration(model, data);

console.log("nq", ik.nq(), "nv", ik.nv());
const start = ik.getFramePose("ee", "site");
console.log("start pose", start);
console.log("com", ik.getCom());

const ee = new mujoco.FrameTask("ee", "site", 1.0, 0.0, 1.0, 0.0);
ee.setTargetPosQuat(
  [start.pos[0], start.pos[1], start.pos[2] - 0.04],
  start.quat,
);

const posture = new mujoco.PostureTask(model, 0.01, 1.0, 0.0);
posture.setTargetFromConfiguration(ik);

const limit = new mujoco.ConfigurationLimit(model);
const dt = 0.02;

for (let i = 0; i < 80; i++) {
  const vel = mujoco.solveIK(ik, [ee], [limit], dt, 1e-12);
  if (i < 3) {
    console.log("step", i, "vel", vel, "q", ik.q());
  }
  ik.integrateInplace(vel, dt);
}

const end = ik.getFramePose("ee", "site");
console.log("end pose", end);
console.log("q", ik.q());

if (!(end.pos[2] < start.pos[2] - 0.005) || Math.abs(ik.q()[0]) < 1e-4) {
  throw new Error(
    `IK did not move toward -Z target (z ${start.pos[2]} -> ${end.pos[2]})`,
  );
}

const rel = new mujoco.RelativeFrameTask(
  "ee",
  "site",
  "link",
  "body",
  1.0,
  0.0,
  1.0,
  0.0,
);
rel.setTargetFromConfiguration(ik);

const com = new mujoco.ComTask(0.001, 1.0, 0.0);
com.setTargetFromConfiguration(ik);

const damping = new mujoco.DampingTask(model, 0.001);
const velLimit = new mujoco.VelocityLimit(model, { hinge: 2.0 });
const collision = new mujoco.CollisionAvoidanceLimit(model, [
  ["link_geom", "obstacle"],
]);

const p2Vel = mujoco.solveIK(
  ik,
  [ee, posture, rel, com, damping],
  [limit, velLimit, collision],
  dt,
  1e-12,
);
if (!Array.isArray(p2Vel) || p2Vel.length !== ik.nv()) {
  throw new Error(`P2 solveIK returned unexpected velocity: ${p2Vel}`);
}
ik.integrateInplace(p2Vel, dt);
console.log("P2 mixed solve vel", p2Vel);

const eqModel = mujoco.MjModel.from_xml_string(eqXml);
const eqData = new mujoco.MjData(eqModel);
const eqIk = new mujoco.IkConfiguration(eqModel, eqData);
const eqTask = new mujoco.EqualityConstraintTask(eqModel, 1.0, [0], 1.0, 0.0);
const eqVel = mujoco.solveIK(eqIk, [eqTask], [], dt, 1e-12);
if (!Array.isArray(eqVel) || eqVel.length !== eqIk.nv()) {
  throw new Error(`EqualityConstraintTask solveIK failed: ${eqVel}`);
}
console.log("equality solve vel", eqVel);

try {
  new mujoco.EqualityConstraintTask(model, 1.0);
  throw new Error("EqualityConstraintTask should reject models with no equalities");
} catch (e) {
  const msg = String(e);
  if (!msg.includes("no equality")) {
    throw e;
  }
}

rel.delete();
com.delete();
damping.delete();
velLimit.delete();
collision.delete();
eqTask.delete();
eqIk.delete();
eqData.delete();
eqModel.delete();
ik.delete();
ee.delete();
posture.delete();
limit.delete();
data.delete();
model.delete();
console.log("P2 smoke OK");

const lookXml = `
<mujoco>
  <worldbody>
    <body name="link" pos="0 0 0.2">
      <joint name="hinge" type="hinge" axis="0 1 0" limited="true" range="-1.57 1.57"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.25 0 0"/>
      <site name="ee" pos="0.25 0 0" size="0.01"/>
    </body>
  </worldbody>
</mujoco>
`;

const lookModel = mujoco.MjModel.from_xml_string(lookXml);
const lookData = new mujoco.MjData(lookModel);
const lookIk = new mujoco.IkConfiguration(lookModel, lookData);
const look = new mujoco.LookAtTask("ee", "site", [1, 0, 0], 1.0, 1.0, 0.0);
look.setTargetFromConfiguration(lookIk);
const align = new mujoco.AxisAlignTask("ee", "site", [1, 0, 0], 1.0, 1.0, 0.0);
align.setTargetFromConfiguration(lookIk);
const ke = new mujoco.KineticEnergyRegularizationTask(1e-4);
ke.setDt(dt);
const lookLimit = new mujoco.ConfigurationLimit(lookModel);
const p3Vel = mujoco.solveIK(lookIk, [look, align, ke], [lookLimit], dt, 1e-12);
if (!Array.isArray(p3Vel) || p3Vel.length !== lookIk.nv()) {
  throw new Error(`P3 look/align/ke solveIK failed: ${p3Vel}`);
}
console.log("P3 look/align/ke vel", p3Vel);

const freezeEe = new mujoco.FrameTask("ee", "site", 1.0, 0.0, 1.0, 0.0);
const lookStart = lookIk.getFramePose("ee", "site");
freezeEe.setTargetPosQuat(
  [lookStart.pos[0], lookStart.pos[1], lookStart.pos[2] - 0.04],
  lookStart.quat,
);
const freeze = new mujoco.DofFreezingTask(lookModel, [0]);
const frozenVel = mujoco.solveIK(
  lookIk,
  [freezeEe],
  [lookLimit],
  dt,
  1e-12,
  [freeze],
);
if (Math.abs(frozenVel[0]) > 1e-6) {
  throw new Error(`DofFreezingTask constraint should hold hinge still, got ${frozenVel}`);
}
console.log("P3 freeze constraint vel", frozenVel);

const freeXml = `
<mujoco>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <freejoint name="root"/>
      <geom type="sphere" size="0.05"/>
    </body>
  </worldbody>
</mujoco>
`;
const freeModel = mujoco.MjModel.from_xml_string(freeXml);
const freeData = new mujoco.MjData(freeModel);
const freeIk = new mujoco.IkConfiguration(freeModel, freeData);
const freeLimit = new mujoco.FreeJointVelocityLimit(freeModel, 1.0, 2.0, "root");
const freeDamp = new mujoco.DampingTask(freeModel, 1.0);
const freeVel = mujoco.solveIK(freeIk, [freeDamp], [freeLimit], dt, 1e-12);
if (!Array.isArray(freeVel) || freeVel.length !== 6) {
  throw new Error(`FreeJointVelocityLimit solveIK failed: ${freeVel}`);
}
console.log("P3 free-joint vel", freeVel);

try {
  new mujoco.FreeJointVelocityLimit(lookModel, 1.0, 2.0);
  throw new Error("FreeJointVelocityLimit should reject models with no free joint");
} catch (e) {
  const msg = String(e);
  if (!msg.includes("no free joint")) {
    throw e;
  }
}

look.delete();
align.delete();
ke.delete();
lookLimit.delete();
freezeEe.delete();
freeze.delete();
lookIk.delete();
lookData.delete();
lookModel.delete();
freeLimit.delete();
freeDamp.delete();
freeIk.delete();
freeData.delete();
freeModel.delete();
console.log("P3 smoke OK");
