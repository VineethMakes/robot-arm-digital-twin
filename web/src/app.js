import * as THREE from "https://unpkg.com/three@0.165.0/build/three.module.js";

const canvas = document.querySelector("#scene");
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.fog = new THREE.Fog(0x101314, 1.8, 4.8);

const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 10);
camera.position.set(1.25, 0.82, 1.35);
camera.lookAt(0.18, 0.22, 0);

scene.add(new THREE.HemisphereLight(0xf8efe3, 0x27302f, 2.2));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.4);
keyLight.position.set(1.5, 2.2, 1.1);
scene.add(keyLight);

const floor = new THREE.Mesh(
  new THREE.CircleGeometry(0.82, 96),
  new THREE.MeshStandardMaterial({ color: 0x242a28, roughness: 0.82 }),
);
floor.rotation.x = -Math.PI / 2;
scene.add(floor);

const grid = new THREE.GridHelper(1.6, 16, 0x52766d, 0x2f3a37);
grid.position.y = 0.002;
scene.add(grid);

const armMaterial = new THREE.MeshStandardMaterial({
  color: 0xec5f48,
  metalness: 0.22,
  roughness: 0.48,
});
const jointMaterial = new THREE.MeshStandardMaterial({
  color: 0xf3f0e8,
  metalness: 0.3,
  roughness: 0.35,
});
const targetMaterial = new THREE.MeshStandardMaterial({
  color: 0x96d6c5,
  emissive: 0x264f46,
  roughness: 0.35,
});

const target = new THREE.Mesh(new THREE.SphereGeometry(0.025, 32, 16), targetMaterial);
scene.add(target);

const links = [];
const joints = [];
for (let i = 0; i < 3; i += 1) {
  const link = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 1, 18), armMaterial);
  link.rotation.z = Math.PI / 2;
  links.push(link);
  scene.add(link);

  const joint = new THREE.Mesh(new THREE.SphereGeometry(0.035, 32, 16), jointMaterial);
  joints.push(joint);
  scene.add(joint);
}
const endJoint = new THREE.Mesh(new THREE.SphereGeometry(0.028, 32, 16), jointMaterial);
scene.add(endJoint);

const controls = {
  x: document.querySelector("#target-x"),
  y: document.querySelector("#target-y"),
  z: document.querySelector("#target-z"),
};
const output = {
  mode: document.querySelector("#solver-mode"),
  reachability: document.querySelector("#reachability"),
  error: document.querySelector("#error"),
  baseYaw: document.querySelector("#base-yaw"),
  shoulder: document.querySelector("#shoulder"),
  elbow: document.querySelector("#elbow"),
  wrist: document.querySelector("#wrist"),
};

let solveArm = fallbackSolveArm;

try {
  const wasm = await import("../pkg/arm_kinematics.js");
  await wasm.default();
  solveArm = (x, y, z) => JSON.parse(wasm.solve_arm(x, y, z));
  output.mode.textContent = "Rust WASM solver";
} catch {
  output.mode.textContent = "JavaScript fallback solver";
}

function currentTarget() {
  return {
    x: Number.parseFloat(controls.x.value),
    y: Number.parseFloat(controls.y.value),
    z: Number.parseFloat(controls.z.value),
  };
}

function update() {
  const point = currentTarget();
  const solution = solveArm(point.x, point.y, point.z);
  renderArm(solution.pose.points);
  target.position.set(point.x, point.y, point.z);

  const jointsOut = solution.pose.joints;
  output.reachability.textContent = solution.reachable ? "Reachable target" : "Best effort target";
  output.error.textContent = `${solution.error_m.toFixed(3)} m error`;
  output.baseYaw.textContent = `${Math.round(jointsOut.base_yaw_deg)} deg`;
  output.shoulder.textContent = `${Math.round(jointsOut.shoulder_deg)} deg`;
  output.elbow.textContent = `${Math.round(jointsOut.elbow_deg)} deg`;
  output.wrist.textContent = `${Math.round(jointsOut.wrist_deg)} deg`;
}

function renderArm(points) {
  const vectors = points.map((point) => new THREE.Vector3(point.x, point.y, point.z));

  for (let i = 0; i < links.length; i += 1) {
    const start = vectors[i];
    const end = vectors[i + 1];
    const mid = start.clone().lerp(end, 0.5);
    const direction = end.clone().sub(start);
    links[i].position.copy(mid);
    links[i].scale.set(1, direction.length(), 1);
    links[i].quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
    joints[i].position.copy(start);
  }
  endJoint.position.copy(vectors[vectors.length - 1]);
}

function resize() {
  const rect = canvas.parentElement.getBoundingClientRect();
  camera.aspect = rect.width / rect.height;
  camera.updateProjectionMatrix();
  renderer.setSize(rect.width, rect.height, false);
}

function animate() {
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

Object.values(controls).forEach((control) => control.addEventListener("input", update));
window.addEventListener("resize", resize);

resize();
update();
animate();

function fallbackSolveArm(x, y, z) {
  const baseHeight = 0.16;
  const lengths = [0.28, 0.24, 0.12];
  const yaw = Math.atan2(z, x);
  const target2d = [Math.hypot(x, z), y - baseHeight];
  const angles = [20, 50, -20];

  for (let step = 0; step < 180; step += 1) {
    const end = planarEnd(lengths, angles);
    const error = [target2d[0] - end[0], target2d[1] - end[1]];
    if (Math.hypot(error[0], error[1]) < 0.002) break;
    for (let joint = 2; joint >= 0; joint -= 1) {
      const derivative = planarDerivative(lengths, angles, joint);
      const gradient = error[0] * derivative[0] + error[1] * derivative[1];
      angles[joint] = clamp(angles[joint] + gradient * 18, -135, 135);
    }
  }

  const points = [{ x: 0, y: 0, z: 0 }];
  let radius = 0;
  let height = baseHeight;
  let angle = 0;
  for (let i = 0; i < lengths.length; i += 1) {
    angle += (angles[i] * Math.PI) / 180;
    radius += lengths[i] * Math.cos(angle);
    height += lengths[i] * Math.sin(angle);
    points.push({ x: radius * Math.cos(yaw), y: height, z: radius * Math.sin(yaw) });
  }

  const end = points[points.length - 1];
  const error = Math.hypot(end.x - x, end.y - y, end.z - z);
  return {
    pose: {
      joints: {
        base_yaw_deg: (yaw * 180) / Math.PI,
        shoulder_deg: angles[0],
        elbow_deg: angles[1],
        wrist_deg: angles[2],
      },
      points,
      end_effector: end,
    },
    target: { x, y, z },
    error_m: error,
    iterations: 180,
    reachable: error < 0.04,
  };
}

function planarEnd(lengths, angles) {
  let angle = 0;
  const end = [0, 0];
  for (let i = 0; i < lengths.length; i += 1) {
    angle += (angles[i] * Math.PI) / 180;
    end[0] += lengths[i] * Math.cos(angle);
    end[1] += lengths[i] * Math.sin(angle);
  }
  return end;
}

function planarDerivative(lengths, angles, joint) {
  let angle = 0;
  const derivative = [0, 0];
  for (let i = 0; i < lengths.length; i += 1) {
    angle += (angles[i] * Math.PI) / 180;
    if (i >= joint) {
      derivative[0] += -lengths[i] * Math.sin(angle) * (Math.PI / 180);
      derivative[1] += lengths[i] * Math.cos(angle) * (Math.PI / 180);
    }
  }
  return derivative;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}
