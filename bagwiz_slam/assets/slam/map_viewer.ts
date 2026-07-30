// Browser-side viewer for `bagwiz map slam --viewer`. Loads the locally served
// map.pcd and renders it with configurable controls: which scalar drives the
// color (x/y/z/intensity), its value range (auto or manual), the colormap,
// point size, point opacity, a 3D / 2D view toggle (2D is a top-down bird's-eye
// view, switchable between orthographic and perspective projection, and
// left-drag rotatable about the vertical Z axis while the top-down tilt stays
// locked), and double-click-to-anchor
// recentering. A corner orientation gizmo (three.js ViewHelper) shows the global
// X/Y/Z axes and snaps the view on click, and a scale bar reports the on-screen
// distance scale. When a sibling traj.tum exists next to map.pcd, an optional
// (default off) trajectory overlay is also offered — each pose's X/Y/Z axes
// joined by a backbone line; see the Trajectory section below. TypeScript
// source; compiled to map_viewer.js at build time and
// embedded into bagwiz (see CMakeLists.txt). three.js is resolved from a CDN at
// runtime via the import map in map_viewer.html.

import * as THREE from "three";
import { PCDLoader } from "three/addons/loaders/PCDLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { LineSegments2 } from "three/addons/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/addons/lines/LineSegmentsGeometry.js";
import { COLORMAP_NAMES, DEFAULT_COLORMAP, sampleColormap } from "./map_colormaps.js";
import { createOrientationGizmo, createScaleBar } from "./map_viewer_overlay.js";

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
// Matches --bg in map_viewer.html so the canvas has no seam. Also used as the
// manual clear color, since the gizmo overlay forces autoClear off (see below).
const BG_COLOR = 0x101014;
const scene = new THREE.Scene();
scene.background = new THREE.Color(BG_COLOR);

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
// The corner gizmo overlay renders a second pass, so the scene is cleared
// manually once per frame (see renderFrame) instead of by each render() call.
renderer.autoClear = false;
renderer.setClearColor(BG_COLOR, 1);
document.body.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = false;

// Bottom-corner overlay widgets (see map_viewer_overlay.ts): the CloudCompare-
// style orientation gizmo that tracks the camera and snaps the view on click,
// and the scale bar. `clock` paces the gizmo's click-to-snap animation.
const clock = new THREE.Clock();
const gizmo = createOrientationGizmo(camera, renderer.domElement);
const scaleBar = createScaleBar(el<HTMLElement>("scaleBarLine"), el<HTMLElement>("scaleBarLabel"));

// ---------------------------------------------------------------------------
// On-demand rendering
// ---------------------------------------------------------------------------
// The scene only repaints when something changes — the camera moved, a control
// was touched, or the window resized. This keeps the GPU idle while the view is
// still, which matters because the glass panels' backdrop-filter would otherwise
// recomposite on every frame.
let frameQueued = false;

function renderFrame(): void {
  frameQueued = false;
  let gizmoAnimating = false;
  if (gizmo.animating()) {
    gizmoAnimating = gizmo.update(clock.getDelta());
  }
  const stillEasing = controls.update();
  renderer.clear();
  renderer.render(scene, camera);
  gizmo.render(renderer); // overlays the orientation gizmo in the bottom-right
  scaleBar.update(worldPerPixel());
  if (stillEasing || gizmoAnimating) {
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
  rgb: Float32Array | null; // length count*3, the PCD rgb field (0..1), null when absent
  count: number;
  boundingSphere: THREE.Sphere | null;
  scalar: string; // x | y | z | intensity | rgb
  colormap: string;
  autoRange: boolean;
  rangeMin: number;
  rangeMax: number;
  viewMode: ViewMode;
  projection2d: Projection2D; // 2D projection: parallel (ortho) or perspective
  heading2d: number; // 2D bird's-eye yaw about the vertical (Z) axis, radians
  trajPoses: TrajPoses | null; // parsed traj.tum poses (position + orientation)
  trajGroup: THREE.Group | null; // backbone + pose-axis triads + end markers
  trajMaterials: LineMaterial[]; // every fat-line material, for .resolution on resize
  showTrajectory: boolean; // mirrors #trajToggle; default off
}

const state: ViewerState = {
  geometry: null,
  material: null,
  points: null,
  colorAttr: null,
  positions: null,
  intensity: null,
  colors: null,
  rgb: null,
  count: 0,
  boundingSphere: null,
  scalar: "z",
  colormap: DEFAULT_COLORMAP,
  autoRange: true,
  rangeMin: 0,
  rangeMax: 1,
  viewMode: "3d",
  projection2d: "ortho",
  heading2d: 0,
  trajPoses: null,
  trajGroup: null,
  trajMaterials: [],
  showTrajectory: false,
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
  if (state.scalar === "rgb" && state.rgb) {
    // True camera colors from the PCD's rgb field: copied verbatim, no
    // colormap/range mapping involved.
    state.colors.set(state.rgb);
    state.colorAttr.needsUpdate = true;
    drawColorbar();
    updateStatus();
    requestFrame();
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
// Cameras, view modes, and framing
// ---------------------------------------------------------------------------
const DEG2RAD = Math.PI / 180;

function aspect(): number {
  return window.innerWidth / window.innerHeight;
}

function defaultPointSize(radius: number): number {
  return Math.min(Math.max(radius * 0.0005, 0.02), 0.3);
}

// 2D bird's-eye pose. `TOP_OFFSET` is the unit direction from the target to the
// camera (straight up, looking down -Z).
const TOP_OFFSET = new THREE.Vector3(0, 0, 1);

// Screen-up for the 2D bird's-eye view: north (+y) at heading 0, rolled about
// the vertical (Z) axis by the current heading. Rolling the camera up-vector
// (rather than orbiting) spins the top-down map around Z while keeping it
// perfectly flat — OrbitControls' orbit stays disabled in 2D because it would
// tilt the view.
function top2dUp(): THREE.Vector3 {
  const h = state.heading2d;
  return new THREE.Vector3(-Math.sin(h), Math.cos(h), 0);
}

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
  gizmo.rebind(camera); // rebind the gizmo to the now-active camera
  controls.enableRotate = state.viewMode !== "2d";
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
      orthoCamera.up.copy(top2dUp());
      orthoCamera.position.copy(center).addScaledVector(TOP_OFFSET, radius * 2);
      orthoCamera.zoom = 1;
      setOrthoExtent(fitHalfHeight(radius));
    } else {
      setNearFar(perspCamera, radius);
      perspCamera.up.copy(top2dUp());
      perspCamera.position.copy(center).addScaledVector(TOP_OFFSET, fitDistance(radius));
      perspCamera.updateProjectionMatrix();
    }
  }
  controls.update();
  requestFrame();
}

// Re-roll the active 2D camera to the current heading without changing its
// position or target, so the top-down map spins about the vertical (Z) axis.
// No-op outside 2D.
function apply2dHeading(): void {
  if (state.viewMode !== "2d") {
    return;
  }
  activeCamera().up.copy(top2dUp());
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

// World-space pick radius that corresponds to a few screen pixels at the
// current camera distance. This keeps double-click anchoring precise even when
// zoomed in close to the ground.
function pickThreshold(): number {
  const distance = camera.position.distanceTo(controls.target);
  const worldPerPixel = (2 * distance * Math.tan((PERSP_FOV * DEG2RAD) / 2)) / window.innerHeight;
  const minThreshold = state.boundingSphere ? state.boundingSphere.radius * 0.001 : 0.05;
  return Math.max(worldPerPixel * 4, minThreshold);
}

function onDoubleClick(event: MouseEvent): void {
  if (!state.points || !state.boundingSphere) {
    return;
  }
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  // Use an adaptive, pixel-scale threshold so the anchor lands on the point
  // actually under the cursor rather than a neighbor up to ~1 m away.
  raycaster.params.Points = { threshold: pickThreshold() };
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
// 2D heading: left-drag spins the bird's-eye view about the vertical (Z) axis.
// OrbitControls' orbit is disabled in 2D (it would tilt the top-down lock), so
// the left button is free here; we roll the camera up-vector instead. Panning
// (right-drag) and zoom (wheel) stay owned by OrbitControls.
// ---------------------------------------------------------------------------
let headingDrag: { pointerId: number; lastAngle: number } | null = null;

// Angle of the pointer about the viewport centre (screen y-down, so it grows
// clockwise). Dragging around the centre acts like a turntable.
function pointerAngleAboutCenter(event: PointerEvent): number {
  const rect = renderer.domElement.getBoundingClientRect();
  return Math.atan2(
    event.clientY - (rect.top + rect.height / 2),
    event.clientX - (rect.left + rect.width / 2),
  );
}

renderer.domElement.addEventListener("pointerdown", (event: PointerEvent) => {
  if (state.viewMode !== "2d" || event.button !== 0) {
    return;
  }
  headingDrag = { pointerId: event.pointerId, lastAngle: pointerAngleAboutCenter(event) };
  renderer.domElement.setPointerCapture(event.pointerId);
});

renderer.domElement.addEventListener("pointermove", (event: PointerEvent) => {
  if (!headingDrag || event.pointerId !== headingDrag.pointerId) {
    return;
  }
  const angle = pointerAngleAboutCenter(event);
  let delta = angle - headingDrag.lastAngle;
  // Shortest arc so crossing the ±180° seam does not snap the view around.
  if (delta > Math.PI) {
    delta -= 2 * Math.PI;
  } else if (delta < -Math.PI) {
    delta += 2 * Math.PI;
  }
  headingDrag.lastAngle = angle;
  // Turn the map the same way the cursor sweeps, so a grabbed feature follows
  // the pointer.
  state.heading2d += delta;
  apply2dHeading();
});

function endHeadingDrag(event: PointerEvent): void {
  if (!headingDrag || event.pointerId !== headingDrag.pointerId) {
    return;
  }
  renderer.domElement.releasePointerCapture(headingDrag.pointerId);
  headingDrag = null;
}
renderer.domElement.addEventListener("pointerup", endHeadingDrag);
renderer.domElement.addEventListener("pointercancel", endHeadingDrag);

// Click an axis on the corner gizmo to snap the camera to that view, orbiting
// around the current target. Restricted to 3D mode: in 2D the bird's-eye
// constraint owns the camera orientation, so a snap there would break it.
// handleClick returns false for clicks outside the gizmo, leaving orbit/pan
// drags that merely end in the corner unaffected.
renderer.domElement.addEventListener("pointerup", (event: PointerEvent) => {
  if (state.viewMode !== "3d") {
    return;
  }
  if (gizmo.handleClick(event, controls.target)) {
    clock.getDelta(); // drop idle time so the first animation step is small
    requestFrame();
  }
});

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
  if (state.scalar === "rgb") {
    // True-color mode has no scalar-to-color mapping to legend.
    ctx.clearRect(0, 0, w, h);
    el<HTMLElement>("cbMin").textContent = "";
    el<HTMLElement>("cbMax").textContent = "";
    return;
  }
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
  el<HTMLElement>("ptCount").textContent = `${state.count.toLocaleString()} pts`;
  const coloring = state.scalar === "rgb" ? "rgb" : `${state.scalar} · ${state.colormap}`;
  setStatus(`${state.count.toLocaleString()} pts · ${coloring}`);
}

// ---------------------------------------------------------------------------
// Scale bar feed: world metres per CSS pixel at the orbit-target depth (exact
// for orthographic; measured at the target distance for perspective, like the
// pick scale). The scale bar widget snaps this to a round on-screen length.
// ---------------------------------------------------------------------------
function worldPerPixel(): number {
  const heightPx = window.innerHeight;
  if (camera instanceof THREE.OrthographicCamera) {
    return (camera.top - camera.bottom) / camera.zoom / heightPx;
  }
  const dist = camera.position.distanceTo(controls.target);
  return (2 * dist * Math.tan((PERSP_FOV * DEG2RAD) / 2)) / heightPx;
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

// Colormap and range controls only apply to scalar coloring; in rgb
// (true-color) mode they are inert, so gray them out.
function updateScalarControls(): void {
  const isRgb = state.scalar === "rgb";
  el<HTMLSelectElement>("colormap").disabled = isRgb;
  el<HTMLInputElement>("autoRange").disabled = isRgb;
  setManualRangeEnabled(isRgb || state.autoRange);
}

function buildUI(): void {
  const scalarSel = el<HTMLSelectElement>("scalar");
  if (state.intensity) {
    const opt = document.createElement("option");
    opt.value = "intensity";
    opt.textContent = "intensity";
    scalarSel.appendChild(opt);
  }
  if (state.rgb) {
    const opt = document.createElement("option");
    opt.value = "rgb";
    opt.textContent = "rgb";
    scalarSel.appendChild(opt);
  }
  scalarSel.value = state.scalar;
  updateScalarControls();
  scalarSel.addEventListener("change", () => {
    state.scalar = scalarSel.value;
    if (state.autoRange && state.scalar !== "rgb") {
      syncAutoRange();
    }
    updateScalarControls();
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
  const material = state.material;
  if (material) {
    const sizeSlider = el<HTMLInputElement>("pointSize");
    const sizeVal = el<HTMLElement>("pointSizeVal");
    sizeSlider.value = String(material.size);
    sizeVal.textContent = material.size.toFixed(3);
    setSliderFill(sizeSlider);
    sizeSlider.addEventListener("input", () => {
      material.size = parseFloat(sizeSlider.value);
      sizeVal.textContent = material.size.toFixed(3);
      setSliderFill(sizeSlider);
      requestFrame();
    });

    // The material is created with transparent:true (see onLoad), so changing
    // opacity alone takes effect without a shader recompile (no needsUpdate).
    const opacitySlider = el<HTMLInputElement>("pointOpacity");
    const opacityVal = el<HTMLElement>("pointOpacityVal");
    opacitySlider.value = String(material.opacity);
    opacityVal.textContent = material.opacity.toFixed(2);
    setSliderFill(opacitySlider);
    opacitySlider.addEventListener("input", () => {
      material.opacity = parseFloat(opacitySlider.value);
      opacityVal.textContent = material.opacity.toFixed(2);
      setSliderFill(opacitySlider);
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
    state.heading2d = 0; // back to the canonical north-up bird's-eye
    frameView();
  });
}

// ---------------------------------------------------------------------------
// Trajectory (traj.tum), toggled from the inspector; off by default. Fetched
// eagerly alongside map.pcd, but only added to the scene once the toggle is
// switched on, so an unused trajectory costs nothing in the render graph.
// ---------------------------------------------------------------------------
// Each pose is drawn as an X/Y/Z coordinate triad built from its orientation
// quaternion, colored to match the corner orientation gizmo (three.js
// ViewHelper's defaults): X red, Y green, Z blue. Triads sit at a subset of the
// actual poses, spaced by arc length, and their origins are joined by a neutral
// "backbone" line so the path stays continuous between triads. The vehicle's
// forward axis is X, so the red axis doubles as a direction-of-travel cue.
const TRAJ_AXIS_X_COLOR = new THREE.Color(0xff4466); // gizmo X
const TRAJ_AXIS_Y_COLOR = new THREE.Color(0x88ff44); // gizmo Y
const TRAJ_AXIS_Z_COLOR = new THREE.Color(0x4488ff); // gizmo Z
const TRAJ_AXIS_LENGTH = 1.5; // metres, fixed; no UI slider
const TRAJ_AXIS_POSE_STRIDE = 10; // one axis triad every N poses
const TRAJ_AXIS_DIRS: Array<[THREE.Vector3, THREE.Color]> = [
  [new THREE.Vector3(1, 0, 0), TRAJ_AXIS_X_COLOR],
  [new THREE.Vector3(0, 1, 0), TRAJ_AXIS_Y_COLOR],
  [new THREE.Vector3(0, 0, 1), TRAJ_AXIS_Z_COLOR],
];
const TRAJ_AXIS_WIDTH_PX = 3.5;
const TRAJ_ARROWHEAD_HEIGHT_RATIO = 0.22; // arrowhead length as a fraction of axis length
const TRAJ_ARROWHEAD_RADIUS_RATIO = 0.35; // arrowhead base radius as a fraction of its height

// A neutral backbone (not the teal/blue accent) stays distinct from the blue Z
// axis and the point cloud. Both the backbone and the axes get a dark outline
// so they read over any colormap — jet's low end (the default; see
// map_colormaps.ts) is itself blue.
const TRAJ_BACKBONE_COLOR = 0xd5d9e2;
const TRAJ_OUTLINE_COLOR = 0x08090c;
const TRAJ_OUTLINE_EXTRA_PX = 2; // an outline is this many px wider than its line

// Start (oldest) / end (newest) pose markers — a hollow teal ring and a filled
// blue node — so the open path's two ends are told apart at a glance.
const TRAJ_START_COLOR = 0x3fd2c7; // --a1
const TRAJ_END_COLOR = 0x4ea1ff; // --a2

// Reused scratch objects for axis construction (avoid per-vertex allocation).
const trajQuat = new THREE.Quaternion();
const trajOrigin = new THREE.Vector3();
const trajAxisEnd = new THREE.Vector3();
const trajArrowDir = new THREE.Vector3();
const trajArrowUp = new THREE.Vector3(0, 1, 0);

// Parsed traj.tum: pose origins plus the orientation quaternions that rotate the
// unit axes into world-frame triads. Positions are already in the map's world
// frame (the same SLAM run wrote both map.pcd and traj.tum), so no extra
// transform is needed.
interface TrajPoses {
  positions: Float32Array; // length count*3, pose origins in the map world frame
  quaternions: Float32Array; // length count*4 (x, y, z, w)
  count: number;
}

// Parse TUM trajectory lines ("timestamp tx ty tz qx qy qz qw") into pose
// origins plus orientation quaternions. Blank lines and '#' comments are
// skipped, mirroring bagwiz's core::read_tum; a line that isn't exactly 8
// numeric fields (or has a non-finite value) is skipped rather than aborting
// the whole load.
function parseTumPoses(text: string): TrajPoses {
  const positions: number[] = [];
  const quaternions: number[] = [];
  for (const rawLine of text.split("\n")) {
    const line = rawLine.trim();
    if (line.length === 0 || line.startsWith("#")) {
      continue;
    }
    const fields = line.split(/\s+/);
    if (fields.length !== 8) {
      continue;
    }
    const tx = parseFloat(fields[1]);
    const ty = parseFloat(fields[2]);
    const tz = parseFloat(fields[3]);
    const qx = parseFloat(fields[4]);
    const qy = parseFloat(fields[5]);
    const qz = parseFloat(fields[6]);
    const qw = parseFloat(fields[7]);
    if (![tx, ty, tz, qx, qy, qz, qw].every(Number.isFinite)) {
      continue;
    }
    positions.push(tx, ty, tz);
    quaternions.push(qx, qy, qz, qw);
  }
  return {
    positions: new Float32Array(positions),
    quaternions: new Float32Array(quaternions),
    count: positions.length / 3,
  };
}

// Pick the poses that get an axis triad: every `stride` poses, plus the last
// pose so both ends of an open path always carry a frame. The returned indices
// are always real poses from traj.tum -- axes are never placed at interpolated
// points -- and the backbone still runs through every pose regardless of this
// subset.
function selectAxisFrames(poses: TrajPoses, stride: number): number[] {
  const frames: number[] = [];
  if (poses.count === 0) {
    return frames;
  }
  const step = Math.max(1, Math.floor(stride));
  for (let i = 0; i < poses.count; i += step) {
    frames.push(i);
  }
  if (frames[frames.length - 1] !== poses.count - 1) {
    frames.push(poses.count - 1);
  }
  return frames;
}

// Build the trajectory group: a neutral backbone (simple THREE.Line) through
// every pose origin, an X/Y/Z axis triad (LineSegments2 plus a cone arrowhead
// on the positive end) at each selected pose, and start / end origin markers.
// Axis lines use three/addons LineSegments2 for real pixel-width control and a
// dark outline; the backbone is intentionally a plain 1 px line so it does not
// read as a screen-facing ribbon or cylinder. Every LineMaterial is returned so
// the caller can track it for .resolution updates on resize. The frame count is
// returned for the inspector's hint.
function buildTrajectory(
  poses: TrajPoses,
  axisLength: number,
  axisPoseStride: number,
): { group: THREE.Group; materials: LineMaterial[]; frameCount: number } {
  const group = new THREE.Group();
  const materials: LineMaterial[] = [];

  // Backbone through all pose origins, drawn as a simple 1 px line so it does
  // not look like a screen-facing fat line or a cylinder.
  const backboneGeom = new THREE.BufferGeometry();
  backboneGeom.setAttribute("position", new THREE.BufferAttribute(poses.positions, 3));
  const backboneMat = new THREE.LineBasicMaterial({ color: TRAJ_BACKBONE_COLOR });
  const backbone = new THREE.Line(backboneGeom, backboneMat);
  backbone.renderOrder = 1;
  group.add(backbone);

  // Pose-axis triads at fixed pose-count intervals.
  const frames = selectAxisFrames(poses, axisPoseStride);
  const axisPositions: number[] = [];
  const axisColors: number[] = [];
  const arrowHeight = axisLength * TRAJ_ARROWHEAD_HEIGHT_RATIO;
  const arrowRadius = arrowHeight * TRAJ_ARROWHEAD_RADIUS_RATIO;
  const arrowGeometry = new THREE.ConeGeometry(arrowRadius, arrowHeight, 12);
  const arrowMaterials = new Map<number, THREE.MeshBasicMaterial>();
  for (const i of frames) {
    trajOrigin.set(poses.positions[i * 3], poses.positions[i * 3 + 1], poses.positions[i * 3 + 2]);
    trajQuat.set(
      poses.quaternions[i * 4],
      poses.quaternions[i * 4 + 1],
      poses.quaternions[i * 4 + 2],
      poses.quaternions[i * 4 + 3],
    );
    for (const [dir, color] of TRAJ_AXIS_DIRS) {
      trajAxisEnd.copy(dir).multiplyScalar(axisLength).applyQuaternion(trajQuat).add(trajOrigin);
      axisPositions.push(
        trajOrigin.x,
        trajOrigin.y,
        trajOrigin.z,
        trajAxisEnd.x,
        trajAxisEnd.y,
        trajAxisEnd.z,
      );
      axisColors.push(color.r, color.g, color.b, color.r, color.g, color.b);

      // Smooth cone arrowhead on the positive end of the axis.
      trajArrowDir.copy(dir).applyQuaternion(trajQuat).normalize();
      let arrowMat = arrowMaterials.get(color.getHex());
      if (!arrowMat) {
        arrowMat = new THREE.MeshBasicMaterial({ color });
        arrowMaterials.set(color.getHex(), arrowMat);
      }
      const arrow = new THREE.Mesh(arrowGeometry, arrowMat);
      arrow.position.copy(trajAxisEnd).addScaledVector(trajArrowDir, -arrowHeight * 0.5);
      arrow.quaternion.setFromUnitVectors(trajArrowUp, trajArrowDir);
      arrow.renderOrder = 4;
      group.add(arrow);
    }
  }
  const axisGeom = new LineSegmentsGeometry();
  axisGeom.setPositions(axisPositions);
  axisGeom.setColors(axisColors);
  const axisOutlineMat = new LineMaterial({
    color: TRAJ_OUTLINE_COLOR,
    linewidth: TRAJ_AXIS_WIDTH_PX + TRAJ_OUTLINE_EXTRA_PX,
  });
  const axisMat = new LineMaterial({ linewidth: TRAJ_AXIS_WIDTH_PX, vertexColors: true });
  const axisOutline = new LineSegments2(axisGeom, axisOutlineMat);
  axisOutline.renderOrder = 2;
  const axes = new LineSegments2(axisGeom, axisMat);
  axes.renderOrder = 3;
  group.add(axisOutline, axes);
  materials.push(axisOutlineMat, axisMat);

  // Start (teal ring) and end (blue node) origin markers, sized off the axes.
  const markerRadius = Math.max(0.12, axisLength * 0.18);
  const startRing = new THREE.Mesh(
    new THREE.TorusGeometry(markerRadius * 1.5, markerRadius * 0.32, 8, 24),
    new THREE.MeshBasicMaterial({ color: TRAJ_START_COLOR }),
  );
  startRing.position.set(poses.positions[0], poses.positions[1], poses.positions[2]);
  startRing.renderOrder = 4;
  const last = poses.count - 1;
  const endNode = new THREE.Mesh(
    new THREE.SphereGeometry(markerRadius * 1.2, 16, 12),
    new THREE.MeshBasicMaterial({ color: TRAJ_END_COLOR }),
  );
  endNode.position.set(
    poses.positions[last * 3],
    poses.positions[last * 3 + 1],
    poses.positions[last * 3 + 2],
  );
  endNode.renderOrder = 5;
  group.add(startRing, endNode);

  for (const material of materials) {
    material.resolution.set(window.innerWidth, window.innerHeight);
  }
  return { group, materials, frameCount: frames.length };
}

// Free the GPU resources of a trajectory group before dropping it, so repeated
// rebuilds don't leak geometries or materials.
function disposeTrajectoryGroup(group: THREE.Group): void {
  group.traverse((obj) => {
    const holder = obj as THREE.Object3D & {
      geometry?: THREE.BufferGeometry;
      material?: THREE.Material | THREE.Material[];
    };
    holder.geometry?.dispose();
    if (Array.isArray(holder.material)) {
      holder.material.forEach((m) => m.dispose());
    } else {
      holder.material?.dispose();
    }
  });
}

// (Re)build the trajectory group from the current poses and fixed axis
// settings, swapping it into the scene when the overlay is shown.
function rebuildTrajectory(): void {
  if (!state.trajPoses) {
    return;
  }
  if (state.trajGroup) {
    scene.remove(state.trajGroup);
    disposeTrajectoryGroup(state.trajGroup);
  }
  const built = buildTrajectory(state.trajPoses, TRAJ_AXIS_LENGTH, TRAJ_AXIS_POSE_STRIDE);
  state.trajGroup = built.group;
  state.trajMaterials = built.materials;
  if (state.showTrajectory) {
    scene.add(state.trajGroup);
  }
  el<HTMLElement>("trajHint").textContent =
    `${state.trajPoses.count.toLocaleString()} poses · ` +
    `${built.frameCount.toLocaleString()} axis frames · ` +
    `${TRAJ_AXIS_POSE_STRIDE} poses per axis`;
  requestFrame();
}

// Reveal the inspector's Trajectory group (hidden by default in the markup) and
// wire its toggle, once traj.tum has been fetched and parsed into at least one
// pose. Left untouched -- so the panel stays hidden -- when traj.tum is absent
// (404) or empty.
function revealTrajectoryUi(): void {
  el<HTMLElement>("trajGrp").hidden = false;

  el<HTMLInputElement>("trajToggle").addEventListener("change", (event) => {
    state.showTrajectory = (event.target as HTMLInputElement).checked;
    if (!state.trajGroup) {
      return;
    }
    if (state.showTrajectory) {
      scene.add(state.trajGroup);
    } else {
      scene.remove(state.trajGroup);
    }
    requestFrame();
  });
}

// Fetch and parse the sibling traj.tum served alongside map.pcd (see
// register_map_viewer_routes in map_viewer.cpp). A 404 means no trajectory was
// written next to this map -- not an error -- so the Trajectory panel simply
// stays hidden.
function loadTrajectory(): void {
  fetch("traj.tum")
    .then((res) => (res.ok ? res.text() : null))
    .then((text) => {
      if (text === null) {
        return;
      }
      const poses = parseTumPoses(text);
      if (poses.count === 0) {
        return;
      }
      state.trajPoses = poses;
      revealTrajectoryUi();
      // Build the group now (parsing + geometry are the costly part); it only
      // joins the scene once the toggle is switched on.
      rebuildTrajectory();
    })
    .catch(() => {
      // No trajectory to offer; leave the panel hidden.
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

  // PCDLoader exposes an rgb field (map slam --color) as a normalized 'color'
  // attribute; keep a copy before installing our own colormap buffer under the
  // same attribute name, and default to showing the true colors.
  const rgbAttr = geometry.getAttribute("color");
  state.rgb = rgbAttr ? Float32Array.from(rgbAttr.array) : null;
  if (state.rgb) {
    state.scalar = "rgb";
  }

  state.colors = new Float32Array(state.count * 3);
  state.colorAttr = new THREE.BufferAttribute(state.colors, 3);
  geometry.setAttribute("color", state.colorAttr);
  geometry.computeBoundingSphere();
  state.boundingSphere = geometry.boundingSphere;

  const radius = state.boundingSphere ? state.boundingSphere.radius : 10;
  // transparent:true (default opacity 1.0 = fully opaque) lets the Point opacity
  // slider fade the whole cloud; at 1.0 it renders identically to an opaque cloud.
  state.material = new THREE.PointsMaterial({
    size: defaultPointSize(radius),
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 1.0,
  });
  state.points = new THREE.Points(geometry, state.material);
  scene.add(state.points);

  syncAutoRange();
  recolor();
  buildUI();
  frameView();
  applyView();
  updateStatus();
}

loadTrajectory();

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
  // LineMaterial (Line2) sizes its screen-space width from this uniform, so it
  // must track the viewport like the cameras above.
  for (const material of state.trajMaterials) {
    material.resolution.set(window.innerWidth, window.innerHeight);
  }
  requestFrame();
});

// Kick off the first paint; every later frame is driven on demand (see above).
requestFrame();
