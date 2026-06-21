// Browser-side viewer for `bagwiz slam run --vis`. Loads the locally served
// map.pcd and renders it with configurable controls: which scalar drives the
// color (x/y/z/intensity), its value range (auto or manual), the colormap, a
// subsample rate, point size, a 3D/2D-top view toggle, and double-click-to-anchor
// recentering. TypeScript source; compiled to map_viewer.js at build time and
// embedded into bagwiz (see CMakeLists.txt). three.js is resolved from a CDN at
// runtime via the import map in map_viewer.html.

import * as THREE from "three";
import { PCDLoader } from "three/addons/loaders/PCDLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { COLORMAP_NAMES, DEFAULT_COLORMAP, sampleColormap } from "./map_colormaps.js";

// Look up a required element by id; a missing id is a programmer error because
// the markup in map_viewer.html is fixed and this module runs after it parses.
function el<T extends HTMLElement>(id: string): T {
  const node = document.getElementById(id);
  if (!node) {
    throw new Error(`map viewer: missing element #${id}`);
  }
  return node as T;
}

const statusEl = el<HTMLElement>("status");

function setStatus(text: string): void {
  statusEl.textContent = text;
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
// On-demand rendering
// ---------------------------------------------------------------------------
// The scene only repaints when something changes — the camera moved, a control
// was touched, or the window resized. This keeps the GPU idle while the view is
// still, which matters because the glass panels' backdrop-filter would otherwise
// recomposite on every frame. OrbitControls' damping is honored by re-scheduling
// while `update()` reports the camera is still easing toward its target.
let frameQueued = false;

function renderFrame(): void {
  frameQueued = false;
  const stillEasing = controls.update();
  renderer.render(scene, camera);
  if (stillEasing) {
    requestFrame();
  }
}

function requestFrame(): void {
  if (!frameQueued) {
    frameQueued = true;
    window.requestAnimationFrame(renderFrame);
  }
}

// Pointer drags and programmatic camera moves both dispatch 'change'.
controls.addEventListener("change", requestFrame);

// ---------------------------------------------------------------------------
// Viewer state
// ---------------------------------------------------------------------------
type ViewMode = "3d" | "2d";

interface ViewerState {
  geometry: THREE.BufferGeometry | null;
  material: THREE.PointsMaterial | null;
  points: THREE.Points | null;
  colorAttr: THREE.BufferAttribute | null;
  positions: ArrayLike<number> | null; // length count*3
  intensity: ArrayLike<number> | null; // length count, or null when absent
  colors: Float32Array | null; // length count*3 (the 'color' attribute data)
  count: number;
  boundingSphere: THREE.Sphere | null;
  scalar: string; // x | y | z | intensity
  colormap: string;
  autoRange: boolean;
  rangeMin: number;
  rangeMax: number;
  subsample: number; // 1.0 = draw every point
  viewMode: ViewMode;
}

const state: ViewerState = {
  geometry: null,
  material: null,
  points: null,
  colorAttr: null,
  positions: null,
  intensity: null,
  colors: null,
  count: 0,
  boundingSphere: null,
  scalar: "z",
  colormap: DEFAULT_COLORMAP,
  autoRange: true,
  rangeMin: 0,
  rangeMax: 1,
  subsample: 1.0,
  viewMode: "3d",
};

// ---------------------------------------------------------------------------
// Scalars
// ---------------------------------------------------------------------------
function scalarAt(i: number): number {
  const positions = state.positions;
  if (!positions) {
    return 0;
  }
  switch (state.scalar) {
    case "x":
      return positions[i * 3];
    case "y":
      return positions[i * 3 + 1];
    case "intensity":
      return state.intensity ? state.intensity[i] : 0;
    case "z":
    default:
      return positions[i * 3 + 2];
  }
}

// Min/max of the active scalar over all points (not just the drawn subset).
function scalarExtent(): [number, number] {
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
function recolor(): void {
  if (!state.colors || !state.colorAttr) {
    return;
  }
  const lo = state.rangeMin;
  const span = state.rangeMax - state.rangeMin || 1;
  const colors = state.colors;
  for (let i = 0; i < state.count; i += 1) {
    sampleColormap(state.colormap, (scalarAt(i) - lo) / span, colors, i * 3);
  }
  state.colorAttr.needsUpdate = true;
  drawColorbar();
  updateStatus();
  requestFrame();
}

// ---------------------------------------------------------------------------
// Subsampling: a deterministic shuffled index lets a draw-range prefix act as a
// uniform random subset, and keeps that subset stable as the rate is dragged
// (raising the rate only adds points). Map points are spatially ordered, so a
// plain stride/prefix would alias; the shuffle avoids that.
// ---------------------------------------------------------------------------
function shuffledIndex(n: number): Uint32Array {
  const idx = new Uint32Array(n);
  for (let i = 0; i < n; i += 1) {
    idx[i] = i;
  }
  let seed = 0x9e3779b9; // fixed seed -> identical subset across reloads
  const rand = (): number => {
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

function applySubsample(): void {
  if (!state.geometry) {
    return;
  }
  const shown = Math.max(1, Math.round(state.subsample * state.count));
  state.geometry.setDrawRange(0, shown);
  updateStatus();
  requestFrame();
}

// ---------------------------------------------------------------------------
// Camera modes
// ---------------------------------------------------------------------------
function defaultPointSize(radius: number): number {
  return Math.min(Math.max(radius * 0.001, 0.02), 0.3);
}

// Position the camera around `target` at `dist`, honoring the active view mode.
function placeCamera(target: THREE.Vector3, dist: number): void {
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

// Frame the whole cloud from the current view mode's canonical pose.
function frameToSphere(sphere: THREE.Sphere): void {
  const { center, radius } = sphere;
  controls.target.copy(center);
  placeCamera(center, radius * 2.2);
  camera.near = Math.max(radius / 1000, 0.01);
  camera.far = radius * 100;
  camera.updateProjectionMatrix();
  controls.update();
}

function setViewMode(mode: ViewMode): void {
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

function onDoubleClick(event: MouseEvent): void {
  if (!state.points || !state.boundingSphere) {
    return;
  }
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  // World-space radius around the pick ray; scaled to the cloud so it stays a
  // few pixels on screen at the default framing.
  raycaster.params.Points = { threshold: state.boundingSphere.radius * 0.02 };
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
function drawColorbar(): void {
  const canvas = el<HTMLCanvasElement>("colorbar");
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    return;
  }
  const w = canvas.width;
  const h = canvas.height;
  const rgb = new Float32Array(3);
  for (let x = 0; x < w; x += 1) {
    sampleColormap(state.colormap, x / (w - 1), rgb, 0);
    ctx.fillStyle = `rgb(${rgb[0] * 255},${rgb[1] * 255},${rgb[2] * 255})`;
    ctx.fillRect(x, 0, 1, h);
  }
  el<HTMLElement>("cbMin").textContent = state.rangeMin.toFixed(2);
  el<HTMLElement>("cbMax").textContent = state.rangeMax.toFixed(2);
}

// ---------------------------------------------------------------------------
// Status line
// ---------------------------------------------------------------------------
function updateStatus(): void {
  const shown = Math.max(1, Math.round(state.subsample * state.count));
  el<HTMLElement>("ptCount").textContent = `${state.count.toLocaleString()} pts`;
  setStatus(
    `${shown.toLocaleString()} / ${state.count.toLocaleString()} shown · ` +
      `${state.scalar} · ${state.colormap}`,
  );
}

// ---------------------------------------------------------------------------
// Controls wiring
// ---------------------------------------------------------------------------
function syncAutoRange(): void {
  const [lo, hi] = scalarExtent();
  state.rangeMin = lo;
  state.rangeMax = hi;
  el<HTMLInputElement>("rangeMin").value = lo.toFixed(3);
  el<HTMLInputElement>("rangeMax").value = hi.toFixed(3);
}

function setManualRangeEnabled(disabled: boolean): void {
  el<HTMLInputElement>("rangeMin").disabled = disabled;
  el<HTMLInputElement>("rangeMax").disabled = disabled;
}

// Paint a range slider's filled portion up to its current value. WebKit reads
// the --fill custom property in the track gradient; Firefox fills natively via
// ::-moz-range-progress, so this is a no-op there beyond setting the property.
function setSliderFill(slider: HTMLInputElement): void {
  const min = parseFloat(slider.min);
  const max = parseFloat(slider.max);
  const value = parseFloat(slider.value);
  const pct = max > min ? ((value - min) / (max - min)) * 100 : 0;
  slider.style.setProperty("--fill", `${pct}%`);
}

function buildUI(): void {
  const scalarSel = el<HTMLSelectElement>("scalar");
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

  const colormapSel = el<HTMLSelectElement>("colormap");
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

  const autoBox = el<HTMLInputElement>("autoRange");
  autoBox.checked = state.autoRange;
  autoBox.addEventListener("change", () => {
    state.autoRange = autoBox.checked;
    setManualRangeEnabled(autoBox.checked);
    if (autoBox.checked) {
      syncAutoRange();
      recolor();
    }
  });

  const minInput = el<HTMLInputElement>("rangeMin");
  const maxInput = el<HTMLInputElement>("rangeMax");
  const applyManual = (): void => {
    const lo = parseFloat(minInput.value);
    const hi = parseFloat(maxInput.value);
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
      return;
    }
    state.rangeMin = lo;
    state.rangeMax = hi;
    recolor();
  };
  minInput.addEventListener("change", applyManual);
  maxInput.addEventListener("change", applyManual);
  el<HTMLButtonElement>("resetRange").addEventListener("click", () => {
    autoBox.checked = true;
    state.autoRange = true;
    setManualRangeEnabled(true);
    syncAutoRange();
    recolor();
  });

  const subSlider = el<HTMLInputElement>("subsample");
  const subVal = el<HTMLElement>("subsampleVal");
  subSlider.value = String(state.subsample);
  subVal.textContent = state.subsample.toFixed(2);
  setSliderFill(subSlider);
  subSlider.addEventListener("input", () => {
    state.subsample = parseFloat(subSlider.value);
    subVal.textContent = state.subsample.toFixed(2);
    setSliderFill(subSlider);
    applySubsample();
  });

  const sizeSlider = el<HTMLInputElement>("pointSize");
  const sizeVal = el<HTMLElement>("pointSizeVal");
  const material = state.material;
  if (material) {
    sizeSlider.value = String(material.size);
    sizeVal.textContent = material.size.toFixed(3);
    setSliderFill(sizeSlider);
    sizeSlider.addEventListener("input", () => {
      material.size = parseFloat(sizeSlider.value);
      sizeVal.textContent = material.size.toFixed(3);
      setSliderFill(sizeSlider);
      requestFrame();
    });
  }

  const viewSeg = el<HTMLElement>("viewSeg");
  const viewButtons = Array.from(viewSeg.querySelectorAll<HTMLButtonElement>("button[data-mode]"));
  const syncViewSeg = (): void => {
    for (const button of viewButtons) {
      const on = button.dataset.mode === state.viewMode;
      button.classList.toggle("active", on);
      button.setAttribute("aria-pressed", on ? "true" : "false");
    }
  };
  for (const button of viewButtons) {
    button.addEventListener("click", () => {
      setViewMode(button.dataset.mode === "2d" ? "2d" : "3d");
      syncViewSeg();
    });
  }
  syncViewSeg();

  el<HTMLButtonElement>("resetView").addEventListener("click", () => {
    if (state.boundingSphere) {
      frameToSphere(state.boundingSphere);
    }
  });
}

// ---------------------------------------------------------------------------
// Load
// ---------------------------------------------------------------------------
function onLoad(points: THREE.Points): void {
  // PCDLoader resolves to a THREE.Points (geometry plus a default material we
  // discard); take its geometry and build our own material/Points below.
  const geometry = points.geometry;
  state.geometry = geometry;
  const position = geometry.getAttribute("position");
  state.positions = position.array;
  state.count = position.count;
  const intensityAttr = geometry.getAttribute("intensity");
  state.intensity = intensityAttr ? intensityAttr.array : null;

  state.colors = new Float32Array(state.count * 3);
  state.colorAttr = new THREE.BufferAttribute(state.colors, 3);
  geometry.setAttribute("color", state.colorAttr);
  geometry.setIndex(new THREE.BufferAttribute(shuffledIndex(state.count), 1));
  geometry.computeBoundingSphere();
  state.boundingSphere = geometry.boundingSphere;

  const radius = state.boundingSphere ? state.boundingSphere.radius : 10;
  state.material = new THREE.PointsMaterial({
    size: defaultPointSize(radius),
    vertexColors: true,
    sizeAttenuation: true,
  });
  state.points = new THREE.Points(geometry, state.material);
  scene.add(state.points);

  syncAutoRange();
  recolor();
  applySubsample();
  buildUI();
  if (state.boundingSphere) {
    frameToSphere(state.boundingSphere);
  }
  updateStatus();
}

const loader = new PCDLoader();
// PCDLoader natively exposes the optional per-point `intensity` field (present
// for LiDAR maps) as its own geometry attribute, so it can be selected as a
// coloring scalar with no extra configuration.
setStatus("Loading map.pcd…");
loader.load(
  "map.pcd",
  onLoad,
  (event: ProgressEvent) => {
    if (event.lengthComputable) {
      setStatus(`Loading map.pcd… ${Math.round((event.loaded / event.total) * 100)}%`);
    }
  },
  (error: unknown) => {
    setStatus(`Failed to load map.pcd: ${String(error)}`);
  },
);

window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  requestFrame();
});

// Kick off the first paint; every later frame is driven on demand (see above).
requestFrame();
