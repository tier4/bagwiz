// Browser-side viewer for `bagwiz slam run --vis`. Loads the locally served
// map.ply, colors each point by height, and renders it with orbit controls.
// Linted by ESLint (eslint.config.mjs) and formatted by Prettier.

import * as THREE from "three";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101014);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100000);
// The SLAM map is Z-up (sensor/world frame); tell OrbitControls so up stays up.
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const statusEl = document.getElementById("status");

function setStatus(text) {
  if (statusEl) {
    statusEl.textContent = text;
  }
}

// Color points by height (z) with an HSL ramp (low = blue, high = red). Gives a
// legible map without relying on per-point intensity.
function colorByHeight(geometry) {
  const positions = geometry.getAttribute("position");
  const count = positions.count;
  const colors = new Float32Array(count * 3);

  let minZ = Infinity;
  let maxZ = -Infinity;
  for (let i = 0; i < count; i += 1) {
    const z = positions.getZ(i);
    if (z < minZ) {
      minZ = z;
    }
    if (z > maxZ) {
      maxZ = z;
    }
  }
  const span = maxZ - minZ || 1;

  const color = new THREE.Color();
  for (let i = 0; i < count; i += 1) {
    const t = (positions.getZ(i) - minZ) / span;
    color.setHSL((1 - t) * 0.7, 1.0, 0.5);
    colors[i * 3] = color.r;
    colors[i * 3 + 1] = color.g;
    colors[i * 3 + 2] = color.b;
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
}

// Point the camera at the cloud's bounding sphere and size the clip planes to it.
function frameCamera(geometry) {
  geometry.computeBoundingSphere();
  const sphere = geometry.boundingSphere;
  if (!sphere) {
    return;
  }
  const { center, radius } = sphere;
  controls.target.copy(center);
  camera.position.set(center.x + radius * 1.5, center.y - radius * 1.5, center.z + radius * 1.0);
  camera.near = Math.max(radius / 1000, 0.01);
  camera.far = radius * 100;
  camera.updateProjectionMatrix();
  controls.update();
}

const loader = new PLYLoader();
setStatus("Loading map.ply…");
loader.load(
  "map.ply",
  (geometry) => {
    colorByHeight(geometry);
    const material = new THREE.PointsMaterial({
      size: 0.05,
      vertexColors: true,
      sizeAttenuation: true,
    });
    scene.add(new THREE.Points(geometry, material));
    frameCamera(geometry);
    const count = geometry.getAttribute("position").count;
    setStatus(`${count.toLocaleString()} points`);
  },
  (event) => {
    if (event.lengthComputable) {
      const pct = Math.round((event.loaded / event.total) * 100);
      setStatus(`Loading map.ply… ${pct}%`);
    }
  },
  (error) => {
    setStatus(`Failed to load map.ply: ${error}`);
  },
);

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate() {
  window.requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();
