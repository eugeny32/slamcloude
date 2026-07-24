/**
 * Streaming COPC point cloud viewer (three.js + copc.js).
 *
 * COPC is a LAZ 1.4 with an embedded octree: the `copc` library reads the
 * header/hierarchy and individual nodes via HTTP Range requests, so the file
 * is never downloaded whole. Nodes are loaded coarse-to-fine (breadth-first
 * by octree depth) until POINT_BUDGET is reached — a whole-scene LOD suited
 * for an MVP; view-dependent refinement can replace the ordering later.
 */

import { Copc, Hierarchy } from "copc";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

const POINT_BUDGET = 3_000_000;

export type ColorMode = "rgb" | "height" | "intensity";

export interface ViewerStats {
  loadedPoints: number;
  loadedNodes: number;
  totalNodes: number;
  done: boolean;
  hasRgb: boolean;
  hasIntensity: boolean;
}

// Per-node raw attributes kept so the color buffer can be recomputed on the
// fly when the user switches display mode, without re-fetching the octree.
interface NodeAttrs {
  zWorld: Float32Array;
  intensity: Float32Array | null;
  rgb: Float32Array | null; // normalized 0..1, length n*3
}

export class CopcViewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera: THREE.PerspectiveCamera;
  private controls: OrbitControls;
  private disposed = false;
  private animationId = 0;
  private pointObjects: THREE.Points[] = [];
  private colorMode: ColorMode = "rgb";
  private minZ = 0;
  private maxZ = 1;
  private minI = Infinity;
  private maxI = -Infinity;
  private hasAnyRgb = false;
  private hasAnyIntensity = false;

  constructor(
    private container: HTMLElement,
    private url: string,
    private onStats: (s: ViewerStats) => void,
  ) {
    this.renderer = new THREE.WebGLRenderer({ antialias: false });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.scene.background = new THREE.Color(0x10131a);
    container.appendChild(this.renderer.domElement);

    this.camera = new THREE.PerspectiveCamera(
      60,
      container.clientWidth / container.clientHeight,
      0.01,
      1e7,
    );
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    window.addEventListener("resize", this.handleResize);
  }

  async load(): Promise<void> {
    const copc = await Copc.create(this.url);
    const [minX, minY, minZ, maxX, maxY, maxZ] = copc.info.cube;
    // Offset all coordinates by the cube center: UTM-scale values (~1e6)
    // destroy float32 vertex precision otherwise.
    const offset: [number, number, number] = [
      (minX + maxX) / 2,
      (minY + maxY) / 2,
      (minZ + maxZ) / 2,
    ];
    const size = Math.max(maxX - minX, maxY - minY, maxZ - minZ);

    this.camera.position.set(size * 0.7, -size * 0.7, size * 0.5);
    this.camera.up.set(0, 0, 1); // Z is height in point clouds
    this.controls.target.set(0, 0, 0);
    this.controls.update();
    this.startRenderLoop();

    const { nodes } = await Copc.loadHierarchyPage(this.url, copc.info.rootHierarchyPage);
    const entries = Object.entries(nodes)
      .flatMap(([key, node]) => (node ? [{ key, node, depth: Number(key.split("-")[0]) }] : []))
      .sort((a, b) => a.depth - b.depth);

    let loadedPoints = 0;
    let loadedNodes = 0;
    const pointSize = copc.info.spacing / 2 ** entries[entries.length - 1].depth;

    for (const { node } of entries) {
      if (this.disposed) return;
      if (loadedPoints + node.pointCount > POINT_BUDGET) break;
      await this.addNode(copc, node, offset, minZ, maxZ, pointSize);
      loadedPoints += node.pointCount;
      loadedNodes += 1;
      this.emitStats(loadedPoints, loadedNodes, entries.length, false);
    }
    // Default to RGB when present, else the height ramp; intensity is opt-in.
    if (!this.hasAnyRgb && this.colorMode === "rgb") this.setColorMode("height");
    else this.setColorMode(this.colorMode);
    this.emitStats(loadedPoints, loadedNodes, entries.length, true);
  }

  private emitStats(
    loadedPoints: number,
    loadedNodes: number,
    totalNodes: number,
    done: boolean,
  ): void {
    this.onStats({
      loadedPoints,
      loadedNodes,
      totalNodes,
      done,
      hasRgb: this.hasAnyRgb,
      hasIntensity: this.hasAnyIntensity,
    });
  }

  private async addNode(
    copc: Copc,
    node: Hierarchy.Node,
    offset: [number, number, number],
    minZ: number,
    maxZ: number,
    pointSize: number,
  ): Promise<void> {
    this.minZ = minZ;
    this.maxZ = maxZ;
    const view = await Copc.loadPointDataView(this.url, copc, node);
    const n = view.pointCount;
    const getX = view.getter("X");
    const getY = view.getter("Y");
    const getZ = view.getter("Z");
    const hasRgb = ["Red", "Green", "Blue"].every((d) => d in view.dimensions);
    const getR = hasRgb ? view.getter("Red") : null;
    const getG = hasRgb ? view.getter("Green") : null;
    const getB = hasRgb ? view.getter("Blue") : null;
    const hasIntensity = "Intensity" in view.dimensions;
    const getI = hasIntensity ? view.getter("Intensity") : null;

    const positions = new Float32Array(n * 3);
    const zWorld = new Float32Array(n);
    let rgb: Float32Array | null = null;
    let colorSum = 0;
    if (getR && getG && getB) {
      rgb = new Float32Array(n * 3);
    }
    let intensity: Float32Array | null = null;
    if (getI) {
      intensity = new Float32Array(n);
    }
    for (let i = 0; i < n; i++) {
      const z = getZ(i);
      positions[i * 3] = getX(i) - offset[0];
      positions[i * 3 + 1] = getY(i) - offset[1];
      positions[i * 3 + 2] = z - offset[2];
      zWorld[i] = z;
      if (rgb && getR && getG && getB) {
        const r = getR(i);
        const g = getG(i);
        const b = getB(i);
        colorSum += r + g + b;
        rgb[i * 3] = r / 65535; // LAS RGB is 16-bit.
        rgb[i * 3 + 1] = g / 65535;
        rgb[i * 3 + 2] = b / 65535;
      }
      if (intensity && getI) {
        const iv = getI(i);
        intensity[i] = iv;
        if (iv < this.minI) this.minI = iv;
        if (iv > this.maxI) this.maxI = iv;
      }
    }
    // A cloud with an all-zero RGB channel is effectively uncolorized.
    if (rgb && colorSum > 0) this.hasAnyRgb = true;
    if (intensity) this.hasAnyIntensity = true;

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(n * 3), 3));
    const material = new THREE.PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: true,
    });
    const points = new THREE.Points(geometry, material);
    points.userData.attrs = { zWorld, intensity, rgb } as NodeAttrs;
    this.pointObjects.push(points);
    this.scene.add(points);
    this.applyNodeColor(points);
  }

  /** Recompute one node's color buffer for the current display mode. */
  private applyNodeColor(points: THREE.Points): void {
    const attrs = points.userData.attrs as NodeAttrs | undefined;
    if (!attrs) return;
    const colorAttr = points.geometry.getAttribute("color") as THREE.BufferAttribute;
    const colors = colorAttr.array as Float32Array;
    const n = attrs.zWorld.length;
    const c = new THREE.Color();

    // Fall back to a height ramp when the requested channel is unavailable.
    let mode = this.colorMode;
    if (mode === "rgb" && !attrs.rgb) mode = "height";
    if (mode === "intensity" && !attrs.intensity) mode = "height";

    if (mode === "rgb" && attrs.rgb) {
      colors.set(attrs.rgb);
    } else if (mode === "intensity" && attrs.intensity) {
      const range = Math.max(this.maxI - this.minI, 1e-6);
      for (let i = 0; i < n; i++) {
        const t = (attrs.intensity[i] - this.minI) / range;
        colors[i * 3] = t;
        colors[i * 3 + 1] = t;
        colors[i * 3 + 2] = t; // grayscale intensity ramp
      }
    } else {
      const zRange = Math.max(this.maxZ - this.minZ, 1e-6);
      for (let i = 0; i < n; i++) {
        const t = (attrs.zWorld[i] - this.minZ) / zRange;
        c.setHSL(0.66 * (1 - t), 0.95, 0.55); // blue(low) -> red(high)
        colors[i * 3] = c.r;
        colors[i * 3 + 1] = c.g;
        colors[i * 3 + 2] = c.b;
      }
    }
    colorAttr.needsUpdate = true;
  }

  /** Switch display mode (rgb / height / intensity) and recolor all loaded nodes. */
  setColorMode(mode: ColorMode): void {
    this.colorMode = mode;
    for (const points of this.pointObjects) this.applyNodeColor(points);
  }

  private startRenderLoop(): void {
    const tick = () => {
      if (this.disposed) return;
      this.animationId = requestAnimationFrame(tick);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    tick();
  }

  private handleResize = (): void => {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  };

  dispose(): void {
    this.disposed = true;
    cancelAnimationFrame(this.animationId);
    window.removeEventListener("resize", this.handleResize);
    for (const p of this.pointObjects) {
      p.geometry.dispose();
      (p.material as THREE.Material).dispose();
    }
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
