
import * as THREE from "../../three/build/three.module.min.js";
import { Voxel } from "../../voxels/voxel.js";
import { voxelExists, isClearAbove } from "../core/path_planner.js";
import { getPossibleActions } from "../core/actions.js";
import { scene } from "../../scene.js";


if (typeof window !== "undefined") {
  window._enableBenchmark = true;

  // Helper to reset the benchmark structure
  window.resetBenchmark = function resetBenchmark() {
    window._benchmark = {
      general: {
        totalSteps: 0,
        totalMovements: 0,
        voxelCount: 0,
        movementsSinceLastVoxel: 0
      },
      computation: {
        loopIterations: 0,
        loopSinceLastPlacement: 0
      },
      lastVoxelTime: performance.now(),
      simulationStartTime: performance.now(),
      completed: false,
      endTime: null,
      derived: {}
    };
    window._benchmark.perVoxelLog = [];
  };

  if (!window._benchmark) window.resetBenchmark();
}

const r = n => Math.round(n * 1000) / 1000;

// REF FOR REAL ROBOT DO NOT MODIFY
const current = {
    forward: new THREE.Vector3(1, 0, 0),
    normal: new THREE.Vector3(0, 0, 1)
};

export class BuildingManager {
    constructor(robot, sequence, currentSource, scene) {
      this.robot = robot;
      // Sorted voxelmap assigned to this source (will be annotated with distance data)
      this.sequence = sequence;

      this.source = currentSource;     // Create a class for Source with: 

      // DEBUG: verify source meta coming from Simulation.js (elevator/stairs)
      try {
        console.log("[BuildingManager] Init source meta:", {
          supportType: this.source?.meta?.supportType,
          meta: this.source?.meta,
          pos: this.source?.position,
          dir: this.source?.direction
        });
      } catch (e) { /* no-op */ }

      this.scene = scene;

      // --- Source meta for elevator support tracking ---
      this._ensureSourceMeta();

      // enable step by step simulation
      this.stepModeEnabled = false;
      this.stepFlag = false;
      // Optional callback: called after a voxel is built
      this.onVoxelBuilt = undefined;

      // Track number of voxels placed since last pickup
      this.voxelPlacedSinceLastSource = null;
      this.voxelCarriedCount = null;
      this.maxVoxelsPerPickup = 3;
    }

    _finalizeBenchmark() {
      if (!window._enableBenchmark || !window._benchmark) return;
      const b = window._benchmark;
      b.completed = true;
      b.endTime = performance.now();
      const totalTimeMs = b.endTime - b.simulationStartTime;
      const voxels = Math.max(1, b.general.voxelCount);
      b.derived = {
        totalTimeMs: Math.round(totalTimeMs),
        avgMovementsPerVoxel: b.general.totalMovements / voxels,
        avgStepsPerVoxel: b.general.totalSteps / voxels,
        voxelsPerMinute: (b.general.voxelCount / (totalTimeMs / 60000)).toFixed(2)
      };
    }
    // --- Elevator support helpers ---
    _ensureSourceMeta() {
      if (!this.source) return;
      const src = this.source;
      src.meta = src.meta || {};
      // Direction used to evaluate which supports belong to this source
      const dir = (src.direction || src.dir || new THREE.Vector3(1, 0, 0)).clone();
      dir.setZ(0); // ensure purely planar
      if (dir.length() === 0) dir.set(1, 0, 0);
      src.meta.supportDir = dir.normalize();
      // Base XY on which the support column is keyed
      if (!src.meta.baseXY) src.meta.baseXY = { x: src.position.x, y: src.position.y };
      // Count how many required support blocks (2x2 stacked + 4x2 stacked) are placed for the current layer
      if (typeof src.meta.supportsPlacedThisLayer !== "number") src.meta.supportsPlacedThisLayer = 0;
      // Vertical step to raise the source by (one full layer = 1.0)
      if (typeof src.meta.stepZ !== "number") src.meta.stepZ = 1.0;
      // Tolerance for geometric checks
      if (typeof src.meta.eps !== "number") src.meta.eps = 1e-3;
      // DEBUG: post-init meta snapshot
      try {
        console.log("[BuildingManager] _ensureSourceMeta:", {
          supportType: this.source?.meta?.supportType,
          baseXY: this.source?.meta?.baseXY,
          supportDir: this.source?.meta?.supportDir
        });
      } catch (e) { /* no-op */ }
    }

    _debugElevatorMeta(tag) {
      const src = this.source; if (!src || !src.meta) return;
      try {
        console.log(`[Elevator][${tag}] baseXY=`, src.meta.baseXY, 'dir=', src.meta.supportDir, 'z=', src.position?.z, 'count=', src.meta.supportsPlacedThisLayer);
      } catch(e) {}
    }

    // Visualize voxel centers as spheres (debug helper)
    _markVoxelCenters(voxel, color = 0xff0000, r = 0.08) {
      try {
        if (!voxel || !voxel.centers || !Array.isArray(voxel.centers)) return;
        const geom = new THREE.SphereGeometry(r, 16, 16);
        const mat = new THREE.MeshBasicMaterial({ color });
        voxel.centers.forEach(cstr => {
          if (typeof cstr !== "string") return;
          const [x, y, z] = cstr.split(",").map(Number);
          const m = new THREE.Mesh(geom, mat.clone());
          m.position.set(x, y, z);
          // tag so we could remove later if needed
          m.userData.debugMarker = true;
          scene.add(m);
        });
      } catch (e) {
        console.warn("Failed to mark voxel centers:", e);
      }
    }

    _isOnRayFromBaseXY(x, y) {
      const src = this.source; if (!src) return false;
      const { x: bx, y: by } = src.meta.baseXY;
      const d = src.meta.supportDir; // normalized 2D
      const vx = x - bx, vy = y - by;
      // Perpendicular distance to ray direction (2D cross product magnitude)
      const perp = Math.abs(vx * d.y - vy * d.x);
      if (perp > 0.01) return false;
      // Must be in front of base (non-negative projection)
      const proj = vx * d.x + vy * d.y;
      return proj >= -0.05; // small slack
    }

    _isStacked4x2ForSource(voxel) {
      // type 6 in your code: stacked 4x2 (two XY centers at z and the same two at z+0.5)
      if (voxel.type !== 6) return false;
      const src = this.source; if (!src) return false;
      const eps = src.meta.eps;
      // derive unique (x,y,zLow) for base layer of this stacked voxel
      const centers = (voxel.centers || []).map(c => c.split(',').map(Number));
      if (centers.length < 4) return false;
      const zLow = Math.min(...centers.map(c => c[2]));
      const baseXY = centers.filter(c => Math.abs(c[2] - zLow) < 0.5 * (eps + 1));
      // Expect at least two distinct XY points on the lower slab
      if (baseXY.length < 2) return false;
      // Project onto the support ray from baseXY reference
      const d = src.meta.supportDir; // normalized 2D
      const bx = src.meta.baseXY.x, by = src.meta.baseXY.y;
      const ts = baseXY.map(([x, y]) => ((x - bx) * d.x + (y - by) * d.y));
      ts.sort((a, b) => a - b);
      // Accept either (0,1) if the 4x2 starts at the source, or (1,2) if it starts one step in front
      const is01 = (Math.abs(ts[0] - 0.0) < 0.12) && (Math.abs(ts[1] - 1.0) < 0.12);
      const is12 = (Math.abs(ts[0] - 1.0) < 0.12) && (Math.abs(ts[1] - 2.0) < 0.12);
      return is01 || is12;
    }

    _isStacked2x2ForSource(voxel) {
      // type 9 in your code: stacked 2x2 (one XY at z and the same at z+0.5)
      // Some scenes may encode a simple 2x2 as type 0; allow that as well.
      if (voxel.type !== 9 && voxel.type !== 0) return false;
      const src = this.source; if (!src) return false;
      const eps = src.meta.eps;
      const centers = (voxel.centers || []).map(c => c.split(',').map(Number));
      if (centers.length < 2) return false;
      const zLow = Math.min(...centers.map(c => c[2]));
      const baseXY = centers.filter(c => Math.abs(c[2] - zLow) < 0.5 * (eps + 1));
      if (baseXY.length < 1) return false;
      const d = src.meta.supportDir; // normalized 2D
      const bx = src.meta.baseXY.x, by = src.meta.baseXY.y;
      const t = ((baseXY[0][0] - bx) * d.x + (baseXY[0][1] - by) * d.y);
      // Accept at the source (t≈0), one step out (t≈1), or two steps out (t≈2)
      return (Math.abs(t - 0.0) < 0.12) || (Math.abs(t - 1.0) < 0.12) || (Math.abs(t - 2.0) < 0.12);
    }

    // Elevator support recognition: accepts stacked 4x2 at t≈0 & t≈1 (directly in front of source) or t≈1 & t≈2 (one further out),
    // and stacked 2x2 at t≈0, t≈1, or t≈2 along source.meta.supportDir from source.meta.baseXY.
    _maybeAdvanceSourceAfterSupport(voxel) {
      const src = this.source; if (!src) return;
      // ✅ Only run elevator lift logic for elevator-type supports
      if (!src.meta || src.meta.supportType !== "elevator") return;
      this._ensureSourceMeta();
      this._debugElevatorMeta('before');
      let matched = false;
      if (this._isStacked4x2ForSource(voxel)) matched = true;
      else if (this._isStacked2x2ForSource(voxel)) matched = true;
      if (!matched) return;
            console.log("AAAAAAAAAAAAA[Elevator] Checking voxel for elevator support:gewgjkhjuriuheilu", voxel);

      src.meta.supportsPlacedThisLayer = (src.meta.supportsPlacedThisLayer || 0) + 1;
      // When both elevator support blocks for the current layer are placed, lift the source one layer
      if (src.meta.supportsPlacedThisLayer >= 1) {
        const oldZ = src.position.z;
        const newZ = oldZ + 1;
        src.position.z = newZ;
        // Try dedicated updater if present
        if (typeof src.updatePosition === "function") {
          src.updatePosition(src.position.x, src.position.y, newZ);
        } else if (src.group && src.group.position) { // most Source instances have a group
          src.group.position.set(src.position.x, src.position.y, newZ);
        } else if (src.mesh && src.mesh.position) {   // fallback if a single mesh is used
          src.mesh.position.set(src.position.x, src.position.y, newZ);
        }
        // Also update any cached pose used elsewhere (optional safety)
        if (src.Pos) {
          src.Pos.z = newZ;
        }
        // Reset per-layer counter
        src.meta.supportsPlacedThisLayer = 0;
        try {
          console.log("[Elevator] Source lifted from z=", oldZ, "to z=", newZ);
        } catch(e) {}
        this._debugElevatorMeta('after');
      }
    }
    // Modular helper for step mode
    async awaitStepFlagIfNeeded() {
      if (!this.stepModeEnabled) return;
      await new Promise(resolve => {
        const checkFlag = () => {
          if (this.stepFlag) {
            this.stepFlag = false;
            resolve();
          } else {
            requestAnimationFrame(checkFlag);
          }
        };
        checkFlag();
      });
    }

    async start() {
      // Log step-by-step mode status at start
      this.stepFlag = false;
      // DEBUG: confirm at start
      console.log("[BuildingManager] start() with source supportType =", this.source?.meta?.supportType);
      if (window._enableBenchmark && typeof window.resetBenchmark === 'function') {
        window.resetBenchmark();
        window._benchmark.simulationStartTime = performance.now();
        window._benchmark.lastVoxelTime = performance.now();
      }
      let validPlacements;

      while (true) {
        if (window._enableBenchmark && window._benchmark) {
          window._benchmark.computation.loopIterations++;
        }
        // If sequence is empty, return to source and finish
        if (this.sequence.length === 0) {
          console.log(" All voxels placed. Returning to source...");

          const { frontLeg, backLeg } = this.source.getPlacementCoordinates(0);  // or use any default type
          const returnPath = await this.robot.planPathToLegPositions({
            frontLegGoal: frontLeg,
            backLegGoal: backLeg,
            normalGoal: new THREE.Vector3(0, 0, 1)
          });

          if (returnPath.success) {
            for (const step of returnPath.path) {
              await this.awaitStepFlagIfNeeded.call(this);
              await this.robot.executeAction(step.action);
              this.sendWebsocketStep(step.action);
            }
          } else { console.warn("⚠️ Final return to source failed."); }

          this.voxelPlacedSinceLastSource = 0;
          console.log("📦 Movement history size:",  this.robot.movementHistory.length);
          this._finalizeBenchmark();
          return;
        }
        // Go to source if needed
        if (this.voxelPlacedSinceLastSource === null || this.voxelPlacedSinceLastSource >= this.maxVoxelsPerPickup) {
          this.voxelPlacedSinceLastSource = 0;
          const firstVoxel = this.sequence[0];
          const { frontLeg, backLeg } = this.source.getPlacementCoordinates(firstVoxel.type);

          const returnPath = await this.robot.planPathToLegPositions({
            frontLegGoal: frontLeg,
            backLegGoal: backLeg,
            normalGoal: new THREE.Vector3(0, 0, 1),
          });

          if (returnPath.success) {
            for (const step of returnPath.path) {
              await this.awaitStepFlagIfNeeded.call(this);
              await this.robot.executeAction(step.action);
              this.sendWebsocketStep(step.action);

              if (window._enableBenchmark) {
                window._benchmark.general.totalSteps++;
                window._benchmark.general.totalMovements++;
                window._benchmark.general.movementsSinceLastVoxel++;
              }
            }
          } else {
            console.warn("Failed to reach source pickup position.");
            return;
          }

          // Do actual pickup
          if (firstVoxel.type === 0 || firstVoxel.type === 3 || firstVoxel.type === 4 || firstVoxel.type === 5) {
            await this.robot.executeAction("pickUpLeft");
            this.sendWebsocketStep("pickUp");
          } else if (firstVoxel.type === 1 || firstVoxel.type === 2 || firstVoxel.type === 6 || firstVoxel.type === 9) {
            await this.robot.executeAction("pickUpRight");
            this.sendWebsocketStep("pickUp");
          }
          this.voxelCarriedCount = this.maxVoxelsPerPickup;
          this.voxelPlacedSinceLastSource = 0;
          continue;
        }
        const voxel = this.sequence.shift();

        // --- Elevator support pre-check: ensure at least 2 voxels loaded before placing the first support above source ---
        try {
          const src = this.source;
          if (src && src.meta && src.meta.supportType === "elevator") {
            // Build three source XY positions we want to avoid/recognize
            const srcDir = (src.direction || new THREE.Vector3(1,0,0)).clone();
            srcDir.setZ(0);
            if (srcDir.length() === 0) srcDir.set(1,0,0);
            const sourcePos1 = src.position.clone();
            const sourcePos2 = src.position.clone().add(srcDir.clone());
            const sourcePos3 = src.position.clone().add(srcDir.clone().multiplyScalar(2));

            const r = n => Math.round(n * 1000) / 1000;
            const toVec = (s) => { const [x,y,z] = s.split(',').map(Number); return new THREE.Vector3(r(x), r(y), r(z)); };
            const eqXY = (a,b,tol=1e-3) => Math.abs(a.x-b.x) < tol && Math.abs(a.y-b.y) < tol;

            const c1 = (voxel.centers && voxel.centers[0]) ? toVec(voxel.centers[0]) : null;
            const c2 = (voxel.centers && voxel.centers[1]) ? toVec(voxel.centers[1]) : null;
            const overlaps =
              (c1 && (eqXY(c1, sourcePos1) || eqXY(c1, sourcePos2) || eqXY(c1, sourcePos3))) ||
              (c2 && (eqXY(c2, sourcePos1) || eqXY(c2, sourcePos2) || eqXY(c2, sourcePos3)));

            // If this looks like the first support voxel over the source and we don't have at least 2 in hand, reload first.
            const needReload = (this.voxelPlacedSinceLastSource === null) || (this.voxelPlacedSinceLastSource > 1);
            if (overlaps && needReload) {
              console.log("[Elevator] Pre-check: ensuring >=2 voxels carried before first support. Returning to source for pickup.");
              // Put voxel back so we place it after pickup
              this.sequence.unshift(voxel);

              const { frontLeg, backLeg } = this.source.getPlacementCoordinates(voxel.type);
              const returnPath = await this.robot.planPathToLegPositions({
                frontLegGoal: frontLeg,
                backLegGoal: backLeg,
                normalGoal: new THREE.Vector3(0, 0, 1),
              });
              if (returnPath.success) {
                for (const step of returnPath.path) {
                  await this.awaitStepFlagIfNeeded.call(this);
                  await this.robot.executeAction(step.action);
                  this.sendWebsocketStep(step.action);
                  if (window._enableBenchmark) {
                    window._benchmark.general.totalSteps++;
                    window._benchmark.general.totalMovements++;
                    window._benchmark.general.movementsSinceLastVoxel++;
                  }
                }
              } else {
                console.warn("[Elevator] Could not return to source for pre-pickup.");
              }

              // Perform pickup (fills to max per current sim convention)
              if (voxel.type === 0 || voxel.type === 3 || voxel.type === 4 || voxel.type === 5) {
                await this.robot.executeAction("pickUpLeft");
                this.sendWebsocketStep("pickUp");
              } else if (voxel.type === 1 || voxel.type === 2 || voxel.type === 6 || voxel.type === 9) {
                await this.robot.executeAction("pickUpRight");
                this.sendWebsocketStep("pickUp");
              }
              this.voxelCarriedCount = this.maxVoxelsPerPickup;
              this.voxelPlacedSinceLastSource = 0;
              // Try placing again on next loop iteration
              continue;
            }
          }
        } catch (e) { console.warn("[Elevator] Pre-check error:", e); }

        // ===================== FIND CORRECT PLACING POS  =========================
        let target, normal, supportPos;
        if (voxel.type === 0 || voxel.type === 9) {
          const key = voxel.centers[0];
          const [x, y, z] = key.split(',').map(Number);
          target = new THREE.Vector3(r(x), r(y), r(z));
          if( target.z === this.source.position.z){ 
            if (voxel.type === 9) target.z += 1
            else target.z += 0.5
          }
          validPlacements = this.findValidPlacementPosition(target);
        } else if (voxel.type === 1 || voxel.type === 4 || voxel.type === 5 || voxel.type === 6) {
          const [key1, key2] = voxel.centers;
          const [x1, y1, z1] = key1.split(',').map(Number);
          const [x2, y2, z2] = key2.split(',').map(Number);
          const pos1 = new THREE.Vector3(r(x1), r(y1), r(z1));
          const pos2 = new THREE.Vector3(r(x2), r(y2), r(z2));
          // console.log('Finding placement for voxel type',voxel.type,'at',pos1,'and',pos2)
          if( z1 <= this.source.position.z){ 
            if (voxel.type === 6){
              pos1.z += 1;
              pos2.z += 1;
            } else {
              pos1.z += 0.5
              pos2.z  += 0.5
            }
          }
          // console.log("Finding placement for voxel type",voxel.type,"at",pos1,"and",pos2)
          validPlacements = this.findValidPlacementPosition(pos1, pos2);
          // console.log("Valid placements found:", validPlacements);
        } else if (voxel.type === 2 || voxel.type === 3) {
          const [key1, key2] = voxel.centers;
          const [x1, y1, z1] = key1.split(',').map(Number);
          const [x2, y2, z2] = key2.split(',').map(Number);
          const pos1 = new THREE.Vector3(r(x1), r(y1), r(z1));
          const pos2 = new THREE.Vector3(r(x2), r(y2), r(z2));
          const top = pos1.z > pos2.z ? pos1 : pos2;
          const topTarget = new THREE.Vector3(top.x, top.y, top.z);
          validPlacements = this.findValidPlacementPosition(topTarget);
        }

        // Special-case: if we shift away from the source (elevator supports),
        // keep the raw planned path without alignment correction.
        let preferRawPath = false;
        // Build three source XY positions to avoid
        let sourcePos1 = this.source.position.clone();
        let sourcePos2 = this.source.position.clone().add(this.source.direction.clone());
        let sourcePos3 = this.source.position.clone().add(this.source.direction.clone().multiplyScalar(2));

        // console.log("Source positions to avoid:", sourcePos1, sourcePos2, sourcePos3);
        // console.log("Robot direction:", this.robot.direction, this.robot.target.position, this.robot.origin.position);
        // console.log("Voxel to place:", voxel, "at centers:", voxel.centers);
        // console.log("source direction", this.source.direction);

        // Helpers: parse center "x,y,z" -> Vector3, and XY equality with tolerance
        const toVec = (s) => {
          const [x, y, z] = s.split(',').map(Number);
          return new THREE.Vector3(r(x), r(y), r(z));
        };
        const eqXY = (a, b, tol = 1e-3) =>
          Math.abs(a.x - b.x) < tol && Math.abs(a.y - b.y) < tol;

        // Safely parse up to two centers
        const c1 = voxel.centers && voxel.centers[0] ? toVec(voxel.centers[0]) : null;
        const c2 = voxel.centers && voxel.centers[1] ? toVec(voxel.centers[1]) : null;

       
        // Check overlap against source line (source, source+dir, source+2*dir)
        const overlaps =
          (c1 && (eqXY(c1, sourcePos1) || eqXY(c1, sourcePos2) || eqXY(c1, sourcePos3))) ||
          (c2 && (eqXY(c2, sourcePos1) || eqXY(c2, sourcePos2) || eqXY(c2, sourcePos3)));

        if (overlaps && Array.isArray(validPlacements) && validPlacements.length) {
          // Which one overlaps? (guard each side)
          const overlap1 = (c1 && eqXY(c1, sourcePos1)) || (c2 && eqXY(c2, sourcePos1));
          const overlap2 = (c1 && eqXY(c1, sourcePos2)) || (c2 && eqXY(c2, sourcePos2));
          const overlap3 = (c1 && eqXY(c1, sourcePos3)) || (c2 && eqXY(c2, sourcePos3));

          let originshift = null;
          if (overlap1) originshift = sourcePos1;
          else if (overlap2) originshift = sourcePos2;
          else if (overlap3) originshift = sourcePos3;

          if (originshift) {
            // Snap source direction to a cardinal unit step
            const srcDir = this.source.direction.clone();
            const dirUnit = (Math.abs(srcDir.x) >= Math.abs(srcDir.y))
              ? new THREE.Vector3(Math.sign(srcDir.x) || 1, 0, 0)
              : new THREE.Vector3(0, Math.sign(srcDir.y) || 1, 0);

            const shifted = originshift.clone().sub(dirUnit.clone().multiplyScalar(2));
            // Elevator/source-overlap case: skip alignment correction later
            preferRawPath = true;

            // console.log("Voxel is directly above source, adjusting valid placements away from source");
            for (const entry of validPlacements) {
              entry.pos.x = r(shifted.x);
              entry.pos.y = r(shifted.y);
              // z unchanged by design
            }
          }
        }
        // console.log("valid placement",validPlacements)
        let success = false

        // SORT valid placement upper layer first 
        validPlacements.sort((a,b) => b.pos.z - a.pos.z);
        // console.log("Valid placements (sorted):", validPlacements);
        for (const { pos, target: t, normal: n } of validPlacements) {
          supportPos = pos;     // Front leg target position (robot's stance point for placing the voxel)
          target = t;           // One of the voxel's center positions (used for placement gesture and avoidance)
          normal = n;           // Surface normal of the placement (usually up or wall direction)

          // STEP 1: Check if this placement is reachable from current robot position
          // console.log("1. FIRST PLACEMENT PATH: for voxel:", voxel, "at target:", target, "with support:", supportPos);
          const pathFromRobot = await this.robot.planPathToLegPositions({
            frontLegGoal: supportPos,
            normalGoal: normal,
            voxelToAvoid: target.clone(),
          });
          let finalPath = null;

          if (pathFromRobot.success && pathFromRobot.path.length > 0) {
            // console.log('Path form robot was sucessfull', pathFromRobot);

            // If we just shifted away from the source (elevator case),
            // keep the original path without alignment correction/trimming.
            if (preferRawPath) {
              finalPath = pathFromRobot;
            } else {
              // Correct position of robot to match placing movements
              // sort only centers[0] and centers[1] by distance to front leg, without affecting extra centers
              let lastIdx = pathFromRobot.path.length;
              const frontLegXY = pathFromRobot.path[lastIdx - 1].frontLeg.clone().setZ(0);

              let voxelOrientation, projectedVoxelTarget;
              if (voxel.type === 0 || voxel.type === 9){
                const [x1, y1, z1] = voxel.centers[0].split(',').map(Number);
                projectedVoxelTarget = new THREE.Vector3(x1, y1, 0);
                voxelOrientation = new THREE.Vector3(
                    x1 - frontLegXY.x,
                    y1 - frontLegXY.y,
                    0
                );
              } else {
                let [x1_old, y1_old, z1_old] = voxel.centers[0].split(',').map(Number);
                let [x2_old, y2_old, z2_old] = voxel.centers[1].split(',').map(Number);

                const dist2 = frontLegXY.distanceTo(new THREE.Vector3(x2_old, y2_old, z2_old));
                const dist1 = frontLegXY.distanceTo(new THREE.Vector3(x1_old, y1_old, z1_old));

                if (dist2 < dist1) {
                  [voxel.centers[0], voxel.centers[1]] = [voxel.centers[1], voxel.centers[0]];
                }
                const [x1, y1, z1] = voxel.centers[0].split(',').map(Number);
                const [x2, y2, z2] = voxel.centers[1].split(',').map(Number);

                const dx = Math.round(10 * (x2 - x1)) / 10;
                const dy = Math.round(10 * (y2 - y1)) / 10;
                voxelOrientation = new THREE.Vector3(dx, dy, 0);
                projectedVoxelTarget = new THREE.Vector3(x1, y1, 0);
              }
              // If already aligned: no correction needed
              const expectedFront = frontLegXY.clone().add(voxelOrientation);
            
              if (expectedFront.distanceTo(projectedVoxelTarget) < 0.1) {
                  // console.log('NEED TO compute correction')
                  // Adjust correction goal if last movement was switchLeg
                  const lastStep = pathFromRobot.path[pathFromRobot.path.length - 1];
                  let correctionStartFront = lastStep.frontLeg.clone();
                  let correctionStartBack = lastStep.backLeg.clone();
                  let frontGoal = pathFromRobot.path[lastIdx-1].backLeg.clone();
                  // Attempt to compute correction step
                  let normalup = new THREE.Vector3(0,0,1);
                  const correctionPath = await this.robot.planPathToLegPositions({
                    frontLegGoal: frontGoal,
                    backLegGoal: null,
                    normalGoal: normalup.clone(),
                    voxelToAvoid: undefined,
                    requireParity: true,
                    start: {
                      frontLeg: correctionStartFront,
                      backLeg: correctionStartBack,
                      normal: normalup,
                      forward: new THREE.Vector3().subVectors(
                        correctionStartFront.clone().setZ(0),
                        correctionStartBack.clone().setZ(0)
                      )
                    },
                  });

                if (correctionPath.success) {
                  finalPath = {
                    success: true,
                    path: pathFromRobot.path.concat(correctionPath.path)
                  };
                  // console.log('Applied correction path for alignment',finalPath);
                  // --- REVISED PATH TRIMMING LOGIC ---
                  const MAX_TRIM_LENGTH = 100;
                  const pathLen = finalPath.path.length;
                  if (pathLen > 2) {
                    const lastStep = finalPath.path[pathLen - 1];
                    const finalFront = lastStep.frontLeg;
                    const finalBack = lastStep.backLeg;
                    const finalDir = lastStep.forward;

                    // Check if the robot's current position is already at the final pose
                    const currentFront = this.robot.target.position.clone();
                    const currentBack = this.robot.origin.position.clone();
                    const currentDir = new THREE.Vector3().subVectors(currentFront.clone().setZ(0), currentBack.clone().setZ(0)).normalize();

                    let trimmed = false;
                    if (
                      currentFront.equals(finalFront) &&
                      currentBack.equals(finalBack) &&
                      currentDir.equals(finalDir)
                    ) {
                      finalPath = {
                        success: true,
                        path: []
                      };
                      trimmed = true;
                    }

                    // Existing trimming logic for other repeated positions
                    if (!trimmed) {
                      for (let i = pathLen - 2; i >= 0; i--) {
                        const step = finalPath.path[i];
                        if (
                          step.frontLeg.equals(finalFront) &&
                          step.backLeg.equals(finalBack) &&
                          step.forward.equals(finalDir)
                        ) {
                          finalPath = {
                            success: true,
                            path: finalPath.path.slice(0, i + 1)
                          };
                          trimmed = true;
                          break;
                        }
                      }
                    }

                    if (!trimmed && pathLen > MAX_TRIM_LENGTH) {
                        // 🛡️ Ensure we never trim initial switchLeg
                      let sliced = finalPath.path.slice(-MAX_TRIM_LENGTH);
                      if (finalPath.path[0].action === "switchLeg" && sliced[0].action !== "switchLeg") {
                        sliced.unshift(finalPath.path[0]);  // keep the first switchLeg manually
                      }
                      finalPath = {
                        success: true,
                        path: sliced
                      };
                    }
                  }
                } else { finalPath = pathFromRobot;}
              } else { finalPath = pathFromRobot;}
            }
          } else {finalPath = pathFromRobot;}
          if (finalPath.success) {

            // console.log('Executing path',finalPath)
            // Execute all steps normally
            for (const step of finalPath.path) {
              await this.awaitStepFlagIfNeeded.call(this);
              await this.robot.executeAction(step.action);
              this.sendWebsocketStep(step.action);

              if (window._enableBenchmark) {
                window._benchmark.general.totalSteps++;
                window._benchmark.general.totalMovements++;
                window._benchmark.general.movementsSinceLastVoxel++;
              }
            }
            // Wait for user step if in step mode
            await this.awaitStepFlagIfNeeded.call(this);

            // STEP 3: Determine placement configuration and execute gesture
            const robotFront = this.robot.target.position.clone();
            const robotBack = this.robot.origin.position.clone();


            // Place the voxel visually in the scene
            const [vx, vy, vz] = voxel.centers[0].split(',').map(Number);

            const closestCenterVec = new THREE.Vector3(vx, vy, vz);

            let voxelDirection,furthestCenterVec
            // Determine gesture name from configuration
            if (voxel.type === 0 || voxel.type === 9) {
              const robDir = robotFront.clone().setZ(0).sub(robotBack.clone().setZ(0));
              const robotFrontatPlacement0 = robotFront.clone().setZ(0).add(robDir);
              let frontToVoxelDir = new THREE.Vector3(vx - robotFrontatPlacement0.x, vy - robotFrontatPlacement0.y, 0);
              voxelDirection = frontToVoxelDir.clone().normalize();
              furthestCenterVec = closestCenterVec.clone().add(voxelDirection);
            } else {
              const [ux, uy, uz] = voxel.centers[1].split(',').map(Number);
              furthestCenterVec = new THREE.Vector3(ux, uy, uz);
              voxelDirection = new THREE.Vector3(ux - vx, uy - vy, uz - vz).normalize();
            }
            
            // Execute gesture movement
            let gestureName = this.determinePlacementConfiguration(robotFront, robotBack, closestCenterVec, furthestCenterVec);
            await this.robot.executeAction(gestureName);

            // STEP 4: Determine which voxel to place and rotation of end effector
            const robotFrontatPlacement = this.robot.target.position.clone().setZ(0);
            const robotBackatPlacement = this.robot.origin.position.clone().setZ(0);
            const robotDirection = new THREE.Vector3().subVectors(robotFrontatPlacement, robotBackatPlacement);

            const { angle, slot} = this.findRotationNeeded(robotDirection, voxelDirection);
            // CHECK
            await this.robot.rotateBackLegEndEffector(-angle);
            await new Promise(resolve => setTimeout(resolve, 150));
            await this.robot.updateArrowStatus(slot, true);
            await new Promise(resolve => setTimeout(resolve, 150));
            await this.robot.rotateBackLegEndEffector(0);

            this.sendWebsocketStep(gestureName, slot);
            new Voxel(vx, vy, vz, this.scene, voxel.type, false, voxelDirection);

            // REMOVE VOXEL FROM BUILT LIST
            try {
              if (typeof this.onVoxelBuilt === "function") {
                this.onVoxelBuilt(voxel); // 'voxel' is the original sequence entry we just placed
              }
            } catch (e) {
              console.warn("onVoxelBuilt callback failed:", e);
            }
            // Check if this placed voxel belongs to this source's elevator support; if yes, count & maybe lift source
            this._maybeAdvanceSourceAfterSupport(voxel);
            this.voxelPlacedSinceLastSource++;
            this.voxelCarriedCount--;
            // console.log("=======================================")
            // === Execute reverse of placing gesture ===
            if (typeof gestureName === 'string') {
              const reverseGesture = gestureName.replace("placing_", "placing_r_");
              await this.robot.executeAction(reverseGesture);
              this.sendWebsocketStep(reverseGesture, slot);

            } else {
              console.warn("Invalid gesture name, cannot compute reverse gesture:", gestureName);
            }

            success = true;
            break;
          }
        }
        if (!success) {
          console.warn("Path to support voxel failed for all placements:", voxel);
          // Visual debug: drop red spheres at all centers of the voxel we failed to place
          this._markVoxelCenters(voxel, 0xff0000, 0.08);
          return;
        }
        // perf logging for this voxel placement
        if (window._enableBenchmark) {
          const g = window._benchmark.general;
          const c = window._benchmark.computation;
          const now = performance.now();
          window._benchmark.perVoxelLog.push({
            voxelIndex: g.voxelCount,
            movementsSinceLastPlacement: g.movementsSinceLastVoxel,
            totalSteps: g.totalSteps,
            loopSinceLastPlacement: c.loopIterations - c.loopSinceLastPlacement,
            loopTotal: c.loopIterations,
            durationSinceLastVoxel: Math.round(now - window._benchmark.lastVoxelTime),
            totalSimulationTime: Math.round(now - window._benchmark.simulationStartTime)
          });
          window._benchmark.lastVoxelTime = now;
          g.voxelCount++;
          g.movementsSinceLastVoxel = 0;
          c.loopSinceLastPlacement = c.loopIterations;
        }
      }
    }

    findRotationNeeded(robotDir, voxelDir) {
        const cross = robotDir.x * voxelDir.y - robotDir.y * voxelDir.x; // angle from robot to voxel  Counter Clockwise positive
        const dot = robotDir.dot(voxelDir);
        const angleRobotVoxel = Math.atan2(cross, dot);

        let angle, slot;
        const slots = this.robot.voxelSlots; // shorter reference
        if (angleRobotVoxel > 0) {
            if (!slots.right.isEmpty) { angle = angleRobotVoxel - Math.PI / 2; slot = "right"; }
            else if (!slots.backward.isEmpty) { angle = angleRobotVoxel; slot = "backward"; }
            else if (!slots.left.isEmpty) { angle = angleRobotVoxel + Math.PI / 2; slot = "left"; }
        } else if (angleRobotVoxel < 0) {
            if (!slots.left.isEmpty) { angle = angleRobotVoxel + Math.PI / 2; slot = "left"; }
            else if (!slots.backward.isEmpty) { angle = angleRobotVoxel  ; slot = "backward"; }
            else if (!slots.right.isEmpty) { angle = angleRobotVoxel - Math.PI / 2; slot = "right"; }
        } else {
            if (!slots.backward.isEmpty) { angle = angleRobotVoxel; slot = "backward"; }
            else if (!slots.right.isEmpty) { angle = angleRobotVoxel - Math.PI / 2 ; slot = "right"; }
            else if (!slots.left.isEmpty) { angle = angleRobotVoxel + Math.PI / 2; slot = "left"; }
        }
        return { angle, slot};
    }
    findValidPlacementPosition(targetVoxelCenterOne, targetVoxelCenterTwo = null) {
      // Rules for robot position when placing a voxel:
      //  - Back leg should at most be 
      //      - 2 voxels horizontally from same height up to 1.5 unit (3 voxels),
      //      - 1 voxel side way and same in height
      // 
      // If not on same wall  (next step not yet okay ?)
      //      - Only 0.5 away in x OR y and exactly + 1.5 above      
      //
      // As for now we build layer by layer, we suppose that there is no placing at lower height
      //
      // Input: centers of the next built voxel
      // Output: Best target for front Leg 
      //
      // Compute all valid placement for the centers then pick the one clothest to the source
      // console.log("Finding valid placement positions for voxel centers:", targetVoxelCenterOne, targetVoxelCenterTwo);
      const center1 = new THREE.Vector3(r(targetVoxelCenterOne.x), r(targetVoxelCenterOne.y), r(targetVoxelCenterOne.z));
      const center2 = targetVoxelCenterTwo
        ? new THREE.Vector3(r(targetVoxelCenterTwo.x), r(targetVoxelCenterTwo.y), r(targetVoxelCenterTwo.z))
        : null;
  
      const horizontalSteps = [-1, 0, 1];
      const verticalSteps = [-1.5, -1, -0.5, 0, 0.5, 1];
      const offsets = [
        new THREE.Vector3(0, 0, 0), 
        new THREE.Vector3(0.5, 0, 0), new THREE.Vector3(-0.5, 0, 0),
        new THREE.Vector3(0, 0.5, 0), new THREE.Vector3(0, -0.5, 0),
        new THREE.Vector3(0.5, 0.5, 0), new THREE.Vector3(-0.5, 0.5, 0),
        new THREE.Vector3(0.5, -0.5, 0), new THREE.Vector3(-0.5, -0.5, 0),
      ];
  
      const allValidPositions = [];
  
      const checkCandidates = (centerA, centerB) => {
        for (const dx of horizontalSteps) {
          for (const dy of horizontalSteps) {
            if (Math.abs(dx) + Math.abs(dy) > 1) continue;
            for (const dz of verticalSteps) {
              if (dx === 0 && dy === 0 && dz === 0) continue;
  
              const candidate = centerA.clone().add(new THREE.Vector3(dx, dy, dz));
  
              let legOverlaps = false;
              if (centerB) {
                for (const offset of offsets) {
                  if (candidate.equals(centerB.clone().add(offset))) {
                    legOverlaps = true;
                    break;
                  }
                }
              }
              if (!legOverlaps && this.canStandAt(candidate)) {
                const dist = candidate.distanceTo(this.source.position);
                allValidPositions.push({ pos: candidate.clone(),
                                        target: centerA.clone(),
                                        normal: new THREE.Vector3(0,0,1),
                                        distance: dist });
              }
            }
          }
        }
      };
  
      const checkWallPlacements = (center) => {
        if(center.z < 2.5) return;        // Defined for one layer of voxel above ground before structure 
        const wallNormals = [
          new THREE.Vector3(1, 0, 0),
          new THREE.Vector3(-1, 0, 0),
          new THREE.Vector3(0, 1, 0),
          new THREE.Vector3(0, -1, 0),
        ];
      
        for (const normal of wallNormals) {
          // Step back along the wall normal by one voxel
          const candidate = center.clone().add(normal.clone().multiplyScalar(0.5));
      
          // Set the z one unit *below* the voxel to simulate "standing on the wall"
          candidate.z -= 0.5;
      
          if (this.canStandAt(candidate, normal,true)) {
            const dist = candidate.distanceTo(this.source.position);
            allValidPositions.push({
              pos: candidate.clone(),
              target: center.clone(),
              normal: normal,
              distance: dist
            });
          }
        }
      };
  
      checkCandidates(center1, center2);
      checkWallPlacements(center1);
      if (center2) checkCandidates(center2, center1);
  
      if (center2) checkWallPlacements(center2);
  
      for (const option of allValidPositions) {
        const adjustment = option.normal.clone().multiplyScalar(0.5);
        option.pos.sub(adjustment); // ensure legs are below the robot body
      }
  
      // console.log("All valid positions:", allValidPositions.sort((a, b) => a.distance - b.distance));
      return allValidPositions.sort((a, b) => a.distance - b.distance);
    }
  
    canStandAt(pos, normal = new THREE.Vector3(0, 0, 1),vertical = false) { // VERIF QUE CA VA PARTOUT 
  
      const exists = voxelExists(pos, normal, vertical);
      const clear = isClearAbove(pos, normal);
      return exists && clear;
    }

  // Determine robot/voxel placement configuration and return gesture name
  determinePlacementConfiguration(robotFront, robotBack, closestCenterVec, furthestCenterVec = undefined) {
    // console.log(robotFront, robotBack, closestCenterVec)
    const centerClose = closestCenterVec.clone().setZ(0);

    if( closestCenterVec.z === this.source.position.z){ closestCenterVec.z += 0.5}
    // get placing height
    const z = Math.round((closestCenterVec.z - robotBack.z ) * 10) / 10;
    const zStr = z.toString().replace('.', '_');

    // Parse voxel centers and determine close/far and direction
    // if (furthestCenterVec === undefined){return `placing_2l_${zStr}`;}
    const centerFar = furthestCenterVec.setZ(0);
    const voxelDir = centerFar.clone().sub(centerClose).normalize();

    const robotF = robotFront.clone().setZ(0);  
    const robotB = robotBack.clone().setZ(0);  
    const robotDir = robotF.clone().sub(robotB).normalize();

    const up = new THREE.Vector3(0, 0, 1);
    const cross = new THREE.Vector3().crossVectors(robotDir, voxelDir);
    const dotZ = cross.dot(up);

    // console.log("Robot direction",robotDir , "\n voxel direction:", voxelDir, "\nz:", z, "\ndotZ:", dotZ);
    // 1.  dotZ = 0 placing 1_1 R/L + placing 3 C + placing 4 R/L
    if (Math.abs(dotZ) === 0) {
      const distFrontToC1 = Math.abs(centerClose.x - robotFront.x) + Math.abs(centerClose.y - robotFront.y);
      const voxelToCenter1 = centerClose.clone().sub(robotFront.clone()).normalize();
      const cross1 = new THREE.Vector3().crossVectors(robotDir, voxelToCenter1);
      const dotZ1 = cross1.dot(up);
      if (distFrontToC1 === 1) {
        if (dotZ1 < 0) return `placing_4l_${zStr}`;
        else if (dotZ1 > 0) return `placing_4r_${zStr}`;
      }
      else if (distFrontToC1 > 1) {
        if (dotZ1 < 0) return `placing_1l_1_${zStr}`;
        else if (dotZ1 > 0) return `placing_1r_1_${zStr}`;
        else return `placing_3c_${zStr}`
      }
    }
    else if (dotZ < 0.01) {
      // 2. dotZ < 0 placing 1_2 R + placing 2 r + placing 3 r
      // const distFrontToC1 = centerClose.distanceTo(robotFront.clone());
      const distFrontToC1 = Math.sqrt((centerClose.x - robotFront.x) ** 2 + (centerClose.y - robotFront.y) ** 2);
    
      if (distFrontToC1 === 1) return `placing_2r_${zStr}`;                   // 
      if (distFrontToC1 === 2) return `placing_3r_${zStr}`;                   // kijo
      else return `placing_1r_2_${zStr}`;                                     //
    }
    else if (dotZ > 0.01) {
      // 3. dotZ > 0 placing 1_2 L + placing 2 l + placing 3 l
      // const distFrontToC1 = centerClose.distanceTo(robotFront.clone());
      const distFrontToC1 = Math.sqrt((centerClose.x - robotFront.x) ** 2 + (centerClose.y - robotFront.y) ** 2);
      if (distFrontToC1 === 1) return `placing_2l_${zStr}`;                   // 
      if (distFrontToC1 === 2) return `placing_3l_${zStr}`;                   // kkok
      else return `placing_1l_2_${zStr}`;                                     //
    }
    console.log("no gesture found")
  }

    // Sends a single step/action to the websocket for the specified robot
  sendWebsocketStep(actionName,lastPlacementSlot=undefined ) {
    if (!this.lainConnected()) return;

    let robot = this.robot;
    if (!window.robotSwitchCount) window.robotSwitchCount = 0;

    if (actionName === "switchLeg") {
      window.robotSwitchCount++;
    } else if (actionName.startsWith("placing_r")) {
      const slot = lastPlacementSlot;
      const mvtMessage = [
        "PlaceMvtReverse",
        slot,
        actionName
      ].join(":");
      // console.log("PlaceMvt Reverse message to robot:", mvtMessage);
      window.lainSocket.send(mvtMessage);
    } else if (actionName.startsWith("placing_")) {
      const slot = lastPlacementSlot;
      const mvtMessage = [
        "PlaceMvt",
        slot,
        actionName
      ].join(":");
      // console.log("PlaceMvt message to robot:", mvtMessage);
      window.lainSocket.send(mvtMessage);
    }else if (actionName.startsWith("pickUp")) {
      const mvtMessage = [
        "PickMvt",
        actionName
      ].join(":");
      // console.log("PickMvt message to robot:", mvtMessage);
      window.lainSocket.send(mvtMessage);
    } else {
      const originPos = robot.origin.position;
      const targetPos = robot.target.position;
      const relativeFront = targetPos.clone().sub(originPos);
      const [match] = getPossibleActions(current, 1, () => false, actionName);
      const frontLegDelta = match?.moveFront || new THREE.Vector3(0, 0, 0);
      const backLegDelta  = match?.moveBack  || new THREE.Vector3(0, 0, 0);

      const useSwitch = window.robotSwitchCount % 2 === 1;
      const tagged = useSwitch ? `switch_${actionName}` : actionName;
      const carriedVoxel = this.voxelCarriedCount;
      
      const mvtMessage = [
        "WebMvt",
        carriedVoxel,
        useSwitch ? 1 : 0,
        relativeFront.x, relativeFront.y, relativeFront.z,
        frontLegDelta.x, frontLegDelta.y, frontLegDelta.z,
        backLegDelta.x, backLegDelta.y, backLegDelta.z,
        tagged
      ].join(":");
      // console.log("🔄 WebMvt message to robot:", mvtMessage);
      window.lainSocket.send(mvtMessage);
    }
  }

  lainConnected() {
      return window.lainSocket && window.lainSocket.readyState === WebSocket.OPEN;
  }
}


