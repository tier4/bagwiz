// Browser-side viewer for `bagwiz slam run --viewer`. Loads the locally served
// map.pcd and renders it with configurable controls: which scalar drives the
// color (x/y/z/intensity), its value range (auto or manual), the colormap, a
// subsample rate, point size, a 3D / 2D view toggle (2D is a top-down bird's-eye
// view, switchable between orthographic and perspective projection), and
// double-click-to-anchor
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

// Toolbar segmented controls (present in the fixed markup). Looked up once so
// view/projection state can be mirrored onto them from anywhere.
const projSeg = el<HTMLElement>("projSeg");
const viewButtons = Array.from(
  el<HTMLElement>("viewSeg").querySelectorAll<HTMLButtonElement>("button[data-mode]"),
);
const projButtons = Array.from(projSeg.querySelectorAll<HTMLButtonElement>("button[data-proj]"));

// ---------------------------------------------------------------------------
// Scene, renderer, camera, controls
// ---------------------------------------------------------------------------
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101014);

// The SLAM map is Z-up (sensor/world frame); tell every camera so up stays up.
// PERSP_FOV is the vertical field of view shared by the 3D view and 2D's
// perspective option, kept in one place so the framing math stays consistent.
const PERSP_FOV = 60;
const perspCamera = new THREE.PerspectiveCamera(
  PERSP_FOV,
  window.innerWidth / window.innerHeight,
  0.1,
  100000,
);
perspCamera.up.set(0, 0, 1);

// Backs 2D mode's default parallel projection. The frustum is sized to the cloud
// before the first 2D frame; these placeholder extents are overwritten then.
const orthoCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 100000);
orthoCamera.up.set(0, 0, 1);

// Whichever projection is currently active. `applyView` swaps it; every
// render / raycast / resize path reads this single reference.
let camera: THREE.PerspectiveCamera | THREE.OrthographicCamera = perspCamera;

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
type Projection2D = "ortho" | "persp";

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
  projection2d: Projection2D; // 2D projection: parallel (ortho) or perspective
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
  projection2d: "ortho",
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
// Cameras, view modes, and framing
// ---------------------------------------------------------------------------
const DEG2RAD = Math.PI / 180;

function aspect(): number {
  return window.innerWidth / window.innerHeight;
}

function defaultPointSize(radius: number): number {
  return Math.min(Math.max(radius * 0.001, 0.02), 0.3);
}

// 2D bird's-eye pose. `TOP_OFFSET` is the unit direction from the target to the
// camera (straight up, looking down -Z); `TOP_UP` keeps north (+y) upright.
const TOP_OFFSET = new THREE.Vector3(0, 0, 1);
const TOP_UP = new THREE.Vector3(0, 1, 0);

// Set the ortho frustum left/right/top/bottom from a half-height, fitting the
// viewport aspect. Leaves zoom and near/far untouched.
function setOrthoExtent(halfHeight: number): void {
  const a = aspect();
  orthoCamera.top = halfHeight;
  orthoCamera.bottom = -halfHeight;
  orthoCamera.right = halfHeight * a;
  orthoCamera.left = -halfHeight * a;
  orthoCamera.updateProjectionMatrix();
}

// Half-height that fits a sphere of `radius` in the viewport's smaller dimension.
function fitHalfHeight(radius: number): number {
  const a = aspect();
  const r = radius * 1.1; // margin so the cloud is not flush to the edges
  return a >= 1 ? r : r / a;
}

// Perspective distance at which a sphere of `radius` fills the view (both axes).
function fitDistance(radius: number): number {
  const t = Math.tan((PERSP_FOV * DEG2RAD) / 2);
  return (radius * 1.1) / (t * Math.min(1, aspect()));
}

function setNearFar(cam: THREE.PerspectiveCamera | THREE.OrthographicCamera, radius: number): void {
  cam.near = Math.max(radius / 1000, 0.01);
  cam.far = radius * 100;
}

// The active camera follows state: 3D and 2D-perspective share the perspective
// camera (different poses); 2D-orthographic uses the ortho camera.
function activeCamera(): THREE.PerspectiveCamera | THREE.OrthographicCamera {
  return state.viewMode === "2d" && state.projection2d === "ortho" ? orthoCamera : perspCamera;
}

// Sync `controls`, the active camera reference, and the toolbar to `state`.
// Camera poses are set by frameView / setProjection2d; this only swaps which
// camera is live and refreshes the UI, so it is safe to call after any change.
function applyView(): void {
  camera = activeCamera();
  controls.object = camera;
  controls.enableRotate = true;
  controls.update();
  syncToolbar();
  requestFrame();
}

// Frame the whole cloud from the current mode/plane/projection's canonical pose.
function frameView(): void {
  const sphere = state.boundingSphere;
  if (!sphere) {
    return;
  }
  const { center, radius } = sphere;
  controls.target.copy(center);

  if (state.viewMode === "3d") {
    setNearFar(perspCamera, radius);
    perspCamera.up.set(0, 0, 1);
    const dist = radius * 2.2;
    perspCamera.position.set(center.x + dist * 0.7, center.y - dist * 0.7, center.z + dist * 0.5);
    perspCamera.updateProjectionMatrix();
  } else {
    if (state.projection2d === "ortho") {
      setNearFar(orthoCamera, radius);
      orthoCamera.up.copy(TOP_UP);
      orthoCamera.position.copy(center).addScaledVector(TOP_OFFSET, radius * 2);
      orthoCamera.zoom = 1;
      setOrthoExtent(fitHalfHeight(radius));
    } else {
      setNearFar(perspCamera, radius);
      perspCamera.up.copy(TOP_UP);
      perspCamera.position.copy(center).addScaledVector(TOP_OFFSET, fitDistance(radius));
      perspCamera.updateProjectionMatrix();
    }
  }
  controls.update();
  requestFrame();
}

// Switch the view mode (3D <-> 2D) and reframe from its canonical pose.
function setViewMode(mode: ViewMode): void {
  if (state.viewMode === mode) {
    return;
  }
  state.viewMode = mode;
  frameView();
  applyView();
}

// Toggle 2D between parallel (ortho) and perspective, preserving the current
// viewing direction, target, and on-screen scale so only the projection changes.
function setProjection2d(projection: Projection2D): void {
  if (state.viewMode !== "2d" || state.projection2d === projection) {
    return;
  }
  const radius = state.boundingSphere ? state.boundingSphere.radius : 10;
  const from = activeCamera();
  const target = controls.target;
  const dir = from.position.clone().sub(target);
  const dist = dir.length() || radius * 2;
  dir.normalize();

  state.projection2d = projection;
  if (projection === "ortho") {
    // Match the ortho half-height to perspective's visible half-height at the
    // target distance, so the cloud keeps its on-screen size.
    setNearFar(orthoCamera, radius);
    orthoCamera.up.copy(from.up);
    orthoCamera.position.copy(from.position);
    orthoCamera.zoom = 1;
    setOrthoExtent(dist * Math.tan((PERSP_FOV * DEG2RAD) / 2));
  } else {
    // Place the perspective camera at the distance whose visible half-height
    // equals the current ortho half-height (top / zoom).
    setNearFar(perspCamera, radius);
    perspCamera.up.copy(from.up);
    const halfHeight = orthoCamera.top / orthoCamera.zoom;
    const d = halfHeight / Math.tan((PERSP_FOV * DEG2RAD) / 2);
    perspCamera.position.copy(target).addScaledVector(dir, d);
    perspCamera.updateProjectionMatrix();
  }
  applyView();
}

// Mirror one segmented control's buttons against the active value.
function syncSeg(buttons: HTMLButtonElement[], isActive: (b: HTMLButtonElement) => boolean): void {
  for (const button of buttons) {
    const on = isActive(button);
    button.classList.toggle("active", on);
    button.setAttribute("aria-pressed", on ? "true" : "false");
  }
}

// Reflect view/projection state onto the toolbar, showing the 2D-only
// projection toggle only while in 2D.
function syncToolbar(): void {
  const is2d = state.viewMode === "2d";
  syncSeg(viewButtons, (b) => b.dataset.mode === state.viewMode);
  syncSeg(projButtons, (b) => b.dataset.proj === state.projection2d);
  projSeg.hidden = !is2d;
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

  for (const button of viewButtons) {
    button.addEventListener("click", () => {
      setViewMode(button.dataset.mode === "2d" ? "2d" : "3d");
    });
  }
  for (const button of projButtons) {
    button.addEventListener("click", () => {
      setProjection2d(button.dataset.proj === "persp" ? "persp" : "ortho");
    });
  }
  syncToolbar();

  el<HTMLButtonElement>("resetView").addEventListener("click", () => {
    frameView();
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
  frameView();
  applyView();
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
  perspCamera.aspect = aspect();
  perspCamera.updateProjectionMatrix();
  // Re-fit the ortho frustum's width to the new aspect, preserving zoom/height.
  setOrthoExtent(orthoCamera.top);
  renderer.setSize(window.innerWidth, window.innerHeight);
  requestFrame();
});

// Kick off the first paint; every later frame is driven on demand (see above).
requestFrame();
