// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Bottom-corner overlay widgets for the map viewer, split out of map_viewer.ts to
// keep that core module focused: a CloudCompare-style orientation gizmo (a
// screen-fixed X/Y/Z triad that tracks the camera and snaps the view on click)
// and a scale bar (the world distance a fixed on-screen span represents).
// Compiled to map_viewer_overlay.js at build time and embedded/served alongside
// map_viewer.js (see CMakeLists.txt); three.js resolves from a CDN at runtime.

import type * as THREE from "three";
import { ViewHelper } from "three/addons/helpers/ViewHelper.js";

// ---------------------------------------------------------------------------
// Orientation gizmo
// ---------------------------------------------------------------------------
// three.js's ViewHelper binds to a single camera at construction and only reads
// that camera, so the gizmo must be rebuilt when the viewer swaps cameras (2D
// orthographic uses a different camera object than 3D / 2D-perspective).
export interface OrientationGizmo {
  // Draw the gizmo overlay in the bottom-right corner (call after the scene,
  // with renderer.autoClear off so it does not wipe the scene).
  render(renderer: THREE.WebGLRenderer): void;
  // Advance the click-to-snap animation by `deltaSeconds`; returns true while
  // the animation is still running.
  update(deltaSeconds: number): boolean;
  animating(): boolean;
  // Rebind to a newly active camera (no-op if unchanged).
  rebind(camera: THREE.Camera): void;
  // Snap toward a clicked axis, orbiting around `center`; returns true if the
  // click landed on the gizmo (and an animation was started).
  handleClick(event: PointerEvent, center: THREE.Vector3): boolean;
}

export function createOrientationGizmo(
  camera: THREE.Camera,
  domElement: HTMLElement,
): OrientationGizmo {
  // Build a helper bound to `cam` with the positive-axis spheres labelled X/Y/Z.
  // ViewHelper draws no labels by default, so they are (re)applied on every build
  // (including rebind, which constructs a fresh ViewHelper).
  const build = (cam: THREE.Camera): ViewHelper => {
    const h = new ViewHelper(cam, domElement);
    h.setLabels("X", "Y", "Z");
    return h;
  };
  let helper = build(camera);
  let bound = camera;
  return {
    render: (renderer) => {
      helper.render(renderer);
    },
    update: (deltaSeconds) => {
      helper.update(deltaSeconds);
      return helper.animating;
    },
    animating: () => helper.animating,
    rebind: (next) => {
      if (bound === next) {
        return;
      }
      helper.dispose();
      helper = build(next);
      bound = next;
    },
    handleClick: (event, center) => {
      helper.center.copy(center);
      return helper.handleClick(event);
    },
  };
}

// ---------------------------------------------------------------------------
// Scale bar
// ---------------------------------------------------------------------------
const SCALE_BAR_TARGET_PX = 140; // bar grows toward this width, then snaps down

// Largest 1 / 2 / 5 x 10^n value not exceeding `raw` (raw is always > 0 here).
function niceLength(raw: number): number {
  const base = 10 ** Math.floor(Math.log10(raw));
  const f = raw / base;
  const step = f >= 5 ? 5 : f >= 2 ? 2 : 1;
  return step * base;
}

// Format a metre length with a unit, switching to km / cm at the extremes.
function formatLength(m: number): string {
  if (m >= 1000) {
    return `${(m / 1000).toLocaleString()} km`;
  }
  if (m >= 1) {
    return `${m.toLocaleString()} m`;
  }
  return `${Math.round(m * 100).toLocaleString()} cm`;
}

export interface ScaleBar {
  // Update from the current world-metres-per-CSS-pixel; ignores non-finite or
  // non-positive input (pre-load / degenerate pose), leaving the last reading.
  update(worldPerPixel: number): void;
}

export function createScaleBar(line: HTMLElement, label: HTMLElement): ScaleBar {
  return {
    update: (worldPerPixel) => {
      if (!Number.isFinite(worldPerPixel) || worldPerPixel <= 0) {
        return;
      }
      const meters = niceLength(SCALE_BAR_TARGET_PX * worldPerPixel);
      line.style.width = `${Math.round(meters / worldPerPixel)}px`;
      label.textContent = formatLength(meters);
    },
  };
}
