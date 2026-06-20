// Browser-side viewer for `bagwiz slam run --vis`. Loads the locally served
// map.ply and renders it with configurable controls: which scalar drives the
// color (x/y/z/intensity), its value range (auto or manual), the colormap, a
// subsample rate, point size, a 3D/2D-top view toggle, and double-click-to-anchor
// recentering. Linted by ESLint (eslint.config.mjs) and formatted by Prettier.

import * as THREE from "three";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { COLORMAP_NAMES, DEFAULT_COLORMAP, sampleColormap } from "./map_colormaps.js";

const byId = (id) => document.getElementById(id);
const statusEl = byId("status");

function setStatus(text) {
  if (statusEl) {
    statusEl.textContent = text;
  }
}

// ---------------------------------------------------------------------------
// Scene, renderer, camera, controls
// ---------------------------------------------------------------------------
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101014);

// The SLAM map is Z-up (sensor/world frame); tell the camera so up stays up.
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100000);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

// ---------------------------------------------------------------------------
// Viewer state
// ---------------------------------------------------------------------------
const state = {
  geometry: null,
  material: null,
  points: null,
  positions: null, // Float32Array, length count*3
  intensity: null, // Float32Array, length count, or null when the map has none
  colors: null, // Float32Array, length count*3 (the 'color' attribute)
  count: 0,
  boundingSphere: null,
  scalar: "z", // x | y | z | intensity
  colormap: DEFAULT_COLORMAP,
  autoRange: true,
  rangeMin: 0,
  rangeMax: 1,
  subsample: 1.0, // 1.0 = draw every point
  viewMode: "3d", // 3d | 2d
};

// ---------------------------------------------------------------------------
// Scalars
// ---------------------------------------------------------------------------
function scalarAt(i) {
  switch (state.scalar) {
    case "x":
      return state.positions[i * 3];
    case "y":
      return state.positions[i * 3 + 1];
    case "intensity":
      return state.intensity ? state.intensity[i] : 0;
    case "z":
    default:
      return state.positions[i * 3 + 2];
  }
}

// Min/max of the active scalar over all points (not just the drawn subset).
function scalarExtent() {
  let lo = Infinity;
  let hi = -Infinity;
  for (let i = 0; i < state.count; i += 1) {
    const v = scalarAt(i);
    if (v < lo) {
      lo = v;
    }
    if (v > hi) {
      hi = v;
    }
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
    return [0, 1];
  }
  return [lo, hi];
}

// ---------------------------------------------------------------------------
// Coloring
// ---------------------------------------------------------------------------
function recolor() {
  const lo = state.rangeMin;
  const span = state.rangeMax - state.rangeMin || 1;
  const colors = state.colors;
  for (let i = 0; i < state.count; i += 1) {
    sampleColormap(state.colormap, (scalarAt(i) - lo) / span, colors, i * 3);
  }
  state.geometry.getAttribute("color").needsUpdate = true;
  drawColorbar();
  updateStatus();
}

// ---------------------------------------------------------------------------
// Subsampling: a deterministic shuffled index lets a draw-range prefix act as a
// uniform random subset, and keeps that subset stable as the rate is dragged
// (raising the rate only adds points). Map points are spatially ordered, so a
// plain stride/prefix would alias; the shuffle avoids that.
// ---------------------------------------------------------------------------
function shuffledIndex(n) {
  const idx = new Uint32Array(n);
  for (let i = 0; i < n; i += 1) {
    idx[i] = i;
  }
  let seed = 0x9e3779b9; // fixed seed -> identical subset across reloads
  const rand = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 4294967296;
  };
  for (let i = n - 1; i > 0; i -= 1) {
    const j = Math.floor(rand() * (i + 1));
    const t = idx[i];
    idx[i] = idx[j];
    idx[j] = t;
  }
  return idx;
}

function applySubsample() {
  const shown = Math.max(1, Math.round(state.subsample * state.count));
  state.geometry.setDrawRange(0, shown);
  updateStatus();
}

// ---------------------------------------------------------------------------
// Camera modes
// ---------------------------------------------------------------------------
function defaultPointSize(radius) {
  return Math.min(Math.max(radius * 0.001, 0.02), 0.3);
}

// Frame the whole cloud from the current view mode's canonical pose.
function frameToSphere(sphere) {
  const { center, radius } = sphere;
  controls.target.copy(center);
  placeCamera(center, radius * 2.2);
  camera.near = Math.max(radius / 1000, 0.01);
  camera.far = radius * 100;
  camera.updateProjectionMatrix();
  controls.update();
}

// Position the camera around `target` at `dist`, honoring the active view mode.
function placeCamera(target, dist) {
  if (state.viewMode === "2d") {
    // Bird's-eye: straight above, looking down -Z, north (y) up, no rotation.
    camera.up.set(0, 1, 0);
    camera.position.set(target.x, target.y, target.z + dist);
    controls.enableRotate = false;
  } else {
    camera.up.set(0, 0, 1);
    camera.position.set(target.x + dist * 0.7, target.y - dist * 0.7, target.z + dist * 0.5);
    controls.enableRotate = true;
  }
}

function setViewMode(mode) {
  state.viewMode = mode;
  const target = controls.target.clone();
  const radius = state.boundingSphere ? state.boundingSphere.radius : 10;
  const dist = camera.position.distanceTo(target) || radius * 2.2;
  placeCamera(target, dist);
  camera.updateProjectionMatrix();
  controls.update();
}

// ---------------------------------------------------------------------------
// Anchor: double-click a point to recenter the orbit/pan target on it.
// ---------------------------------------------------------------------------
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();

function onDoubleClick(event) {
  if (!state.points || !state.boundingSphere) {
    return;
  }
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  // World-space radius around the pick ray; scaled to the cloud so it stays a
  // few pixels on screen at the default framing.
  raycaster.params.Points.threshold = state.boundingSphere.radius * 0.02;
  const hits = raycaster.intersectObject(state.points, false);
  if (hits.length === 0) {
    setStatus("No point under cursor — double-click directly on the cloud.");
    return;
  }
  const p = hits[0].point;
  // Recenter without changing the viewing direction: move the camera by the same
  // delta as the target so the framing is preserved.
  camera.position.add(p.clone().sub(controls.target));
  controls.target.copy(p);
  controls.update();
  setStatus(`Anchored at (${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.z.toFixed(2)})`);
}

renderer.domElement.addEventListener("dblclick", onDoubleClick);

// ---------------------------------------------------------------------------
// Colorbar legend (reflects scalar + range + colormap)
// ---------------------------------------------------------------------------
function drawColorbar() {
  const canvas = byId("colorbar");
  if (!canvas) {
    return;
  }
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const rgb = new Float32Array(3);
  for (let x = 0; x < w; x += 1) {
    sampleColormap(state.colormap, x / (w - 1), rgb, 0);
    ctx.fillStyle = `rgb(${rgb[0] * 255},${rgb[1] * 255},${rgb[2] * 255})`;
    ctx.fillRect(x, 0, 1, h);
  }
  byId("cbMin").textContent = state.rangeMin.toFixed(2);
  byId("cbMax").textContent = state.rangeMax.toFixed(2);
}

// ---------------------------------------------------------------------------
// Status line
// ---------------------------------------------------------------------------
function updateStatus() {
  const shown = Math.max(1, Math.round(state.subsample * state.count));
  setStatus(
    `${state.count.toLocaleString()} pts · showing ${shown.toLocaleString()} · ` +
      `${state.scalar} · ${state.colormap}`,
  );
}

// ---------------------------------------------------------------------------
// Controls wiring
// ---------------------------------------------------------------------------
function syncAutoRange() {
  const [lo, hi] = scalarExtent();
  state.rangeMin = lo;
  state.rangeMax = hi;
  byId("rangeMin").value = lo.toFixed(3);
  byId("rangeMax").value = hi.toFixed(3);
}

function setManualRangeEnabled(disabled) {
  byId("rangeMin").disabled = disabled;
  byId("rangeMax").disabled = disabled;
}

function buildUI() {
  const scalarSel = byId("scalar");
  if (state.intensity) {
    const opt = document.createElement("option");
    opt.value = "intensity";
    opt.textContent = "intensity";
    scalarSel.appendChild(opt);
  }
  scalarSel.value = state.scalar;
  scalarSel.addEventListener("change", () => {
    state.scalar = scalarSel.value;
    if (state.autoRange) {
      syncAutoRange();
    }
    recolor();
  });

  const colormapSel = byId("colormap");
  for (const name of COLORMAP_NAMES) {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    colormapSel.appendChild(opt);
  }
  colormapSel.value = state.colormap;
  colormapSel.addEventListener("change", () => {
    state.colormap = colormapSel.value;
    recolor();
  });

  const autoBox = byId("autoRange");
  autoBox.checked = state.autoRange;
  autoBox.addEventListener("change", () => {
    state.autoRange = autoBox.checked;
    setManualRangeEnabled(autoBox.checked);
    if (autoBox.checked) {
      syncAutoRange();
      recolor();
    }
  });

  const applyManual = () => {
    const lo = parseFloat(byId("rangeMin").value);
    const hi = parseFloat(byId("rangeMax").value);
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
      return;
    }
    state.rangeMin = lo;
    state.rangeMax = hi;
    recolor();
  };
  byId("rangeMin").addEventListener("change", applyManual);
  byId("rangeMax").addEventListener("change", applyManual);
  byId("resetRange").addEventListener("click", () => {
    autoBox.checked = true;
    state.autoRange = true;
    setManualRangeEnabled(true);
    syncAutoRange();
    recolor();
  });

  const subSlider = byId("subsample");
  const subVal = byId("subsampleVal");
  subSlider.value = String(state.subsample);
  subVal.textContent = state.subsample.toFixed(2);
  subSlider.addEventListener("input", () => {
    state.subsample = parseFloat(subSlider.value);
    subVal.textContent = state.subsample.toFixed(2);
    applySubsample();
  });

  const sizeSlider = byId("pointSize");
  const sizeVal = byId("pointSizeVal");
  sizeSlider.value = String(state.material.size);
  sizeVal.textContent = state.material.size.toFixed(3);
  sizeSlider.addEventListener("input", () => {
    state.material.size = parseFloat(sizeSlider.value);
    sizeVal.textContent = state.material.size.toFixed(3);
  });

  const viewBtn = byId("viewMode");
  viewBtn.addEventListener("click", () => {
    setViewMode(state.viewMode === "3d" ? "2d" : "3d");
    viewBtn.textContent = state.viewMode === "3d" ? "View: 3D free" : "View: 2D top";
  });

  byId("resetView").addEventListener("click", () => {
    if (state.boundingSphere) {
      frameToSphere(state.boundingSphere);
    }
  });
}

// ---------------------------------------------------------------------------
// Load
// ---------------------------------------------------------------------------
function onLoad(geometry) {
  state.geometry = geometry;
  const position = geometry.getAttribute("position");
  state.positions = position.array;
  state.count = position.count;
  const intensityAttr = geometry.getAttribute("intensity");
  state.intensity = intensityAttr ? intensityAttr.array : null;

  state.colors = new Float32Array(state.count * 3);
  geometry.setAttribute("color", new THREE.BufferAttribute(state.colors, 3));
  geometry.setIndex(new THREE.BufferAttribute(shuffledIndex(state.count), 1));
  geometry.computeBoundingSphere();
  state.boundingSphere = geometry.boundingSphere;

  state.material = new THREE.PointsMaterial({
    size: defaultPointSize(state.boundingSphere.radius),
    vertexColors: true,
    sizeAttenuation: true,
  });
  state.points = new THREE.Points(geometry, state.material);
  scene.add(state.points);

  syncAutoRange();
  recolor();
  applySubsample();
  buildUI();
  frameToSphere(state.boundingSphere);
  updateStatus();
}

const loader = new PLYLoader();
// Pull the optional per-point intensity (present for LiDAR maps) into its own
// attribute so it can be selected as a coloring scalar.
loader.setCustomPropertyNameMapping({ intensity: ["intensity"] });
setStatus("Loading map.ply…");
loader.load(
  "map.ply",
  onLoad,
  (event) => {
    if (event.lengthComputable) {
      setStatus(`Loading map.ply… ${Math.round((event.loaded / event.total) * 100)}%`);
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
