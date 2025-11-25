import * as THREE from "../three/build/three.module.min.js";
const r = n => Math.round(n * 1000) / 1000;

export class BuildingSequencePlanner {
  constructor(currentSource, voxelizedStructure, robots = 1,supportStruc=[] ) {
    this.source = currentSource;
    this.structure = voxelizedStructure;
    this.robots = robots;
    this.robotSequences = {};
    this.sequence = [];
    this.flipFlag = true;         // Alternate between the two sources if exactly equidistant
    this.supportBuildList = supportStruc;

    this._initialize();

  }
  // ========================== SORTING ALGO ==========================

  _initialize() {
    // Normalize inputs
    const voxels = this.structure instanceof Set ? Array.from(this.structure) : (Array.isArray(this.structure) ? this.structure : []);
    const sources = Array.isArray(this.source) ? this.source : [ this.source ];

    // Prepare buckets per source
    const structureBuckets = sources.map(() => []);
    const supportBuckets   = sources.map(() => []);

    // Assign structure voxels to nearest source
    for (const voxel of voxels) {
      const sIdx = this._getClosestSourceIndex(voxel);
      structureBuckets[sIdx].push(voxel);
    }

    // Assign supports (if any) to nearest source
    if (Array.isArray(this.supportBuildList) && this.supportBuildList.length > 0) {
      for (const voxel of this.supportBuildList) {
        const sIdx = this._getClosestSourceIndex(voxel);
        supportBuckets[sIdx].push(voxel);
      }
    }

    // Helper: group voxels by discrete Z
    const groupByLayer = (arr) => {
      const map = new Map();
      for (const v of arr) {
        if (!v || !Array.isArray(v.centers) || v.centers.length === 0) continue;
        const pos = this._parseKey(v.centers[0]);
        const z = pos.z; // already discretized in 0.5 steps upstream
        const list = map.get(z) || [];
        list.push(v);
        map.set(z, list);
      }
      return map;
    };

    // Helper: dedupe by centers signature
    const dedupe = (arr) => {
      const seen = new Set();
      const out = [];
      for (const v of arr) {
        const key = (v.centers || []).slice().sort().join("|");
        if (key && !seen.has(key)) {
          seen.add(key);
          out.push(v);
        }
      }
      return out;
    };

    // For each source, build per-layer sequence: supports-first, then regular
    sources.forEach((src, i) => {
      const structByZ  = groupByLayer(structureBuckets[i]);
      const supportByZ = groupByLayer(supportBuckets[i]);

      // All layer keys that this source will handle
      const allZ = Array.from(new Set([...structByZ.keys(), ...supportByZ.keys()])).sort((a,b) => a - b);

      const perSourceSequence = [];

      for (const z of allZ) {
        const supports = supportByZ.get(z) || [];
        const regulars = structByZ.get(z) || [];

        // Order supports: furthest-from-source first (always)
        const supportsSorted = this._sortLayerByDistanceToSourceDesc(supports, src);

        // Order regulars: use existing heuristic (closest-first on same Z, etc.)
        const regularsSorted = this._sortLayerByDistanceToSource(regulars, src);

        perSourceSequence.push(...supportsSorted, ...regularsSorted);
      }

      // Final per-source sequence with duplicates removed (in case a support also exists in regulars)
      this.robotSequences[`robot${i+1}`] = dedupe(perSourceSequence);
    });

    // Debug
    // console.log("Robot sequences:", JSON.parse(JSON.stringify(this.robotSequences)));
  }
  _sortLayerByDistanceToSourceDesc(layer, source) {
    if (!Array.isArray(layer) || layer.length === 0) return [];
    const origin = new THREE.Vector2(source.position.x, source.position.y);
    // Use the max XY distance among the first two centers (works for stacked gadgets too)
    const withKey = layer.map(v => {
      const pts = (v.centers || []).slice(0, 2).map(c => this._parseKey(c));
      const d = pts.length
        ? Math.max(...pts.map(p => new THREE.Vector2(p.x, p.y).distanceTo(origin)))
        : 0;
      return { v, d };
    });
    withKey.sort((a, b) => b.d - a.d); // furthest first
    return withKey.map(e => e.v);
  }

  _sortLayerByDistanceToSource(layer, source) {
    const origin = new THREE.Vector2(source.position.x, source.position.y);
    const sourceZ = source.position.z;
    const seen = new Set();

    const sorted = layer.slice().sort((a, b) => {
      const aVecs = a.centers.slice(0, 2).map(c => this._parseKey(c));
      const bVecs = b.centers.slice(0, 2).map(c => this._parseKey(c));

      const aSameLevel = aVecs.some(v => v.z === sourceZ);
      const bSameLevel = bVecs.some(v => v.z === sourceZ);

      const aMaxDist = Math.max(...aVecs.map(v => new THREE.Vector2(v.x, v.y).distanceTo(origin)));
      const bMaxDist = Math.max(...bVecs.map(v => new THREE.Vector2(v.x, v.y).distanceTo(origin)));

      if (aSameLevel && bSameLevel) {
        return aMaxDist - bMaxDist; // Closest to furthest on same Z-level
      } else if (aSameLevel) {
        return -1; // Prioritize same-level voxels
      } else if (bSameLevel) {
        return 1;
      } else {
        return aMaxDist - bMaxDist; // Closest to furthest for other levels
      }
    });

    return sorted.filter(voxel => {
      const key = voxel.centers.slice().sort().join("|");
      if (seen.has(key)) return false;
      seen.add(key);
      return true;
    });
  }

  _getClosestSourceIndex(voxel) {
    const centerVectors = voxel.centers.map(c => this._parseKey(c));

    if (!Array.isArray(this.source)) return 0;

    let minDistance = Infinity;
    let closestSources = [];

    for (let s = 0; s < this.source.length; s++) {
      const src = this.source[s];
      const srcPos = new THREE.Vector3(src.position.x, src.position.y, src.position.z); // 3D in case z matters

      const distances = centerVectors.map(c => new THREE.Vector3(c.x, c.y, c.z).distanceTo(srcPos));
      const minDistForSource = Math.min(...distances);

      if (minDistForSource < minDistance - 1e-6) {
        minDistance = minDistForSource;
        closestSources = [s];
      } else if (Math.abs(minDistForSource - minDistance) < 1e-6) {
        closestSources.push(s);
      }
    }

    if (closestSources.length === 1) {
      return closestSources[0];
    } else {
      console.warn(`Voxel at ${voxel.centers} is equidistant to multiple sources. Alternating assignment.`);
      const index = this.flipFlag ? closestSources[0] : closestSources[1];
      this.flipFlag = !this.flipFlag;
      return index;
    }
  }

  _parseKey(key) {
    const [x, y, z] = key.split(',').map(Number);
    return new THREE.Vector3(r(x), r(y), r(z));
  }
}


  // _sortLayerByDistanceXY(layer, source) {
  //   const result = [];

  //   const origin = new THREE.Vector2(source.position.x, source.position.y);
  //   const seen = new Set();

  //   // Sort layer from furthest to closest in |x|, then |y|
  //   layer.sort((a, b) => {
  //     const aVecs = a.centers.slice(0, 2).map(c => this._parseKey(c));
  //     const bVecs = b.centers.slice(0, 2).map(c => this._parseKey(c));

  //     const aMaxX = Math.min(...aVecs.map(p => Math.abs(p.x)));
  //     const bMaxX = Math.min(...bVecs.map(p => Math.abs(p.x)));
  //     if (bMaxX !== aMaxX) return bMaxX - aMaxX;

  //     const aMaxY = Math.max(...aVecs.map(p => Math.abs(p.y)));
  //     const bMaxY = Math.max(...bVecs.map(p => Math.abs(p.y)));
  //     return bMaxY - aMaxY;
  //   });

  //   for (const voxel of layer) {
  //     const key = voxel.centers.slice().sort().join("|");
  //     if (!seen.has(key)) {
  //       seen.add(key);
  //       result.push(voxel);
  //     }
  //   }

  //   return result;
  // }


// OTHER SORTING F.

  // _getDistanceToSource(voxel) {
  //   const center = voxel.centers[0];
  //   const position = this._parseKey(center);
  //   const xyPos = new THREE.Vector2(position.x, position.y);
  
  //   if (Array.isArray(this.source)) {
  //     let minDist = Infinity;
  //     this.source.forEach(src => {
  //       const dist = xyPos.distanceTo(new THREE.Vector2(src.position.x, src.position.y));
  //       if (dist < minDist) {
  //         minDist = dist;
  //       }
  //     });
  //     return minDist;
  //   } else {
  //     const xySource = new THREE.Vector2(this.source.position.x, this.source.position.y);
  //     return xyPos.distanceTo(xySource);
  //   }
  // }

  // _sortLayersForSource(layers, source) {
  //   const result = [];
  //   const sortedLayerKeys = Object.keys(layers).map(Number).sort((a, b) => a - b);

  //   const orientation = source.direction || new THREE.Vector3(0, 1, 0);
  //   const unit = new THREE.Vector2(orientation.x, orientation.y).normalize(); // primary axis
  //   const perp = new THREE.Vector2(-unit.y, unit.x).normalize();              // secondary axis
  //   const origin = new THREE.Vector2(source.position.x, source.position.y);

  //   for (const z of sortedLayerKeys) {
  //     const layer = layers[z];

  //     if (z === 1 || z === 1.5 || z === 2) {
  //       // Special case: build line by line, starting with closest to source along perpendicular direction
  //       layer.sort((a, b) => {
  //         const aPos = this._parseKey(a.centers[0]);
  //         const bPos = this._parseKey(b.centers[0]);

  //         const relA = new THREE.Vector2(aPos.x, aPos.y).sub(origin);
  //         const relB = new THREE.Vector2(bPos.x, bPos.y).sub(origin);

  //         const aLine = relA.dot(unit);   // layer coordinate
  //         const bLine = relB.dot(unit);
  //         if (aLine !== bLine) return bLine - aLine;

  //         const aDist = relA.dot(perp);   // distance from source in strip
  //         const bDist = relB.dot(perp);
  //         return Math.abs(aDist) - Math.abs(bDist);
  //       });

  //       // Ensure centers are consistently ordered for type 1 voxels
  //       for (const voxel of layer) {
  //         if (voxel.type === 1) {
  //           const [c1, c2] = voxel.centers.map(this._parseKey);
  //           const dir = new THREE.Vector2(c2.x - c1.x, c2.y - c1.y);
  //           if (dir.dot(unit) < 0) {
  //             [voxel.centers[0], voxel.centers[1]] = [voxel.centers[1], voxel.centers[0]];
  //           }
  //         }
  //       }

  //       result.push(...layer);
  //       continue;
  //     }

      // Then: sort by perpendicular axis (left to right across the strip)
      // const aPerp = relA.dot(perp);
      // const bPerp = relB.dot(perp);
      // Sort from center 
      // const aPerp = Math.abs(relA.dot(perp));
      // const bPerp = Math.abs(relB.dot(perp));
      // Compute strip center relative to source
      // let totalPerp = 0;
      // for (const voxel of layer) {
      //   const p = this._parseKey(voxel.centers[0]);
      //   const rel = new THREE.Vector2(p.x, p.y).sub(origin);
      //   totalPerp += rel.dot(perp);
      // }
      // const avgPerp = totalPerp / layer.length;
      // const scanDirection = avgPerp < 0 ? -1 : 1;

      // // Sort within the layer: first across rows (parallel), then forward (perpendicular)
      // layer.sort((a, b) => {
      //   const aPos = this._parseKey(a.centers[0]);
      //   const bPos = this._parseKey(b.centers[0]);

      //   const relA = new THREE.Vector2(aPos.x, aPos.y).sub(origin);
      //   const relB = new THREE.Vector2(bPos.x, bPos.y).sub(origin);

      //   // First: sort by parallel axis (farthest to closest)
      //   const aPar = relA.dot(unit);
      //   const bPar = relB.dot(unit);
      //   if (aPar !== bPar) return bPar - aPar;

      //   // Then: sort across the strip starting from the side closest to the source
      //   const aPerp = relA.dot(perp);
      //   const bPerp = relB.dot(perp);
      //   return scanDirection * (aPerp - bPerp);
      // });

      // Ensure centers are consistently ordered for type 1, 4, 5, 6 voxels (aligned with source)
  //     for (const voxel of layer) {
  //       if ([1, 4, 5, 6].includes(voxel.type) && voxel.centers.length >= 2) {
  //         const centerVecs = voxel.centers.map(c => this._parseKey(c));
  //         const origin2D = new THREE.Vector2(source.position.x, source.position.y);
  //         centerVecs.sort((a, b) =>
  //           new THREE.Vector2(a.x, a.y).distanceTo(origin2D) -
  //           new THREE.Vector2(b.x, b.y).distanceTo(origin2D)
  //         );
  //         voxel.centers = centerVecs.map(vec => `${vec.x},${vec.y},${vec.z}`);
  //       }
  //     }

  //     result.push(...layer);
  //   }

  //   return result;
  // }