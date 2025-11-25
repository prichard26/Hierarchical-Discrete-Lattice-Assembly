// IF NOT OPTI TRY TO REMOVE 1 "layer" of gadgets
// TRY TO do greedy from 4 corener and take best, do that for each subgroup
// IF GROUP TO BIG TO OPTIMIZE DIVIDE INTO SUBGROUPS and 

// TRY TO CREATE A CONNECTIVE optimizer 
// MERGING TO REDO 
// WHEN STACKED IT CRASHES WHEN GOING TO SIMU 


// TRY NOT SORTING THE CENTERS WHEN DOING GREEDY SEARCH

import * as THREE from './three/build/three.module.min.js';
import { scene } from './scene.js';

// ============================= Global Constants ============================== //
const rotationMap = [
  ([dx, dy]) => [dx, dy],    // 0° rotation
  ([dx, dy]) => [-dy, dx],   // 90°
  ([dx, dy]) => [-dx, -dy],  // 180°
  ([dx, dy]) => [dy, -dx]    // 270°
];

const voxelSize = window.voxelSize || 65; // Default voxel size, can be overridden by global variable

// Remember the last corner chosen by greedy so Voxelizer can coordinate blocks
let __lastGreedyCornerChosen = null;

function layerXYKeySet(layer) {
  const round = n => Math.round(n * 1000) / 1000;
  const key2D = (x, y) => `${round(x)},${round(y)}`;
  const s = new Set();
  for (const pt of layer) s.add(key2D(pt.x, pt.y));
  return s;
}

function cloneEntryWithZOffset(entry, dz) {
  // Deep-clone an entry (gadget or single) and shift all z by dz
  const shiftPt = (p) => ({ x: p.x, y: p.y, z: p.z + dz });
  if (entry.type === "gadget") {
    return {
      name: entry.name,
      type: "gadget",
      singles: entry.singles.map(shiftPt),
      centers: (entry.centers || []).map(shiftPt),
      origin: entry.origin ? shiftPt(entry.origin) : undefined
    };
  } else {
    const singles = (entry.singles || []).map(shiftPt);
    return {
      name: null,
      type: "single",
      singles,
      centers: []
    };
  }
}

// ============================= Main Voxelizer Function ============================== //
export function Voxelizer(voxelCenters) {
    const startTime = performance.now();
    const voxelMap = []; // Final voxel map to be returned
    const round = n => Math.round(n * 1000) / 1000;

    const layers = divideIntoLayers(voxelCenters);
    let layerCount = 0;

    let previousBlockCorner = null; // corner index used in the previous 2-layer block
    let currentBlockCorner = null;  // corner index to use for this 2-layer block

    for (let i = 0; i < layers.length; i++) {
      console.log(`Processing layer ${i}/${layers.length}`);
        // Determine if we are at the start of a 2-layer block
        const inBlockIndex = i % 2; // 0 or 1 within the block
        if (inBlockIndex === 0) {
          currentBlockCorner = null; // reset; will be set by first group selection below
        }
        // Bypass identical XY only on odd layers (i = 1, 3, 5, ...):
        // clone previous layer's placements and just shift z.
        if (i > 0 && (i % 2 === 1)) {
          const prev = layers[i - 1];
          const curr = layers[i];
          const prevSet = layerXYKeySet(prev);
          const currSet = layerXYKeySet(curr);
          if (prevSet.size === currSet.size && [...currSet].every(k => prevSet.has(k))) {
            const prevZ = prev[0]?.z;
            const currZ = curr[0]?.z;
            const dz = currZ - prevZ;
            const round = n => Math.round(n * 1000) / 1000;
            // Entries belonging to previous layer: match by origin.z when present, else by first single's z
            const prevLayerEntries = voxelMap.filter(e => {
              const zRef = (e.origin && e.origin.z != null) ? e.origin.z : (e.singles && e.singles[0] ? e.singles[0].z : undefined);
              return zRef != null && round(zRef) === round(prevZ);
            });
            const clones = prevLayerEntries.map(e => cloneEntryWithZOffset(e, dz));
            voxelMap.push(...clones);
            // Record block corner and advance layerCount even when bypassing
            if (currentBlockCorner !== null) {
              previousBlockCorner = currentBlockCorner;
            }
            layerCount += 1;
            // console.log(`Finished (bypass) layer i=${i}, layerCount(after)=${layerCount}, prevBlockCorner=${previousBlockCorner}`);
            // Do not increment layerCount here; it is incremented once at the end of the loop
            continue; // skip computing this layer; it's identical in XY to the previous
          }
        }
        const groups = divideIntoConnectedGroups(layers[i]);
        for (let j = 0; j < groups.length; j++) {
            if (layerCount > 0){
              const { matches: overhangmatches, remainingGroup } = findOverhangsPattern(layers[i-1], groups[j]);
              voxelMap.push(...overhangmatches);
              // Find all gadgets from previous layer for patternsBelow (same XY/Z below)
              let patternsBelow = [];
              {
                const prevZ = layers[i - 1][0]?.z;
                patternsBelow = voxelMap.filter(v =>
                  v && v.type === "gadget" && v.origin && round(v.origin.z) === round(prevZ)
                );
              }
              // Enforce corner: for the first group of a new 2-layer block, forbid the previous block corner;
              // once a corner is chosen, force it for the rest of this block
              const forced = (currentBlockCorner !== null) ? currentBlockCorner : null;
              const forbidden = (currentBlockCorner === null) ? previousBlockCorner : null;
              const matches = iterativePatternSearch(remainingGroup, layerCount, patternsBelow, forced, forbidden);
              if (currentBlockCorner === null && __lastGreedyCornerChosen !== null) {
                currentBlockCorner = __lastGreedyCornerChosen;
              }

              voxelMap.push(...matches);
            } else {
              // const matches = optimizeWithILP(groups[j]);
              // const matches = globalPatternSearchGreedy(groups[j], 0);
              const forced = (currentBlockCorner !== null) ? currentBlockCorner : null;
              const forbidden = (currentBlockCorner === null) ? previousBlockCorner : null;
              const matches = iterativePatternSearch(groups[j], layerCount, [], forced, forbidden);
              if (currentBlockCorner === null && __lastGreedyCornerChosen !== null) {
                currentBlockCorner = __lastGreedyCornerChosen;
              }

              voxelMap.push(...matches);
            }
        }

        // When finishing the second layer in a 2-layer block, record the corner used for this block
        if (i % 2 === 1 && currentBlockCorner !== null) {
          previousBlockCorner = currentBlockCorner;
        }
        layerCount += 1;
    }

    // Only consider odd-numbered layers (1, 3, 5, ...)
    for (let i = 1; i < layers.length; i += 2) {
      const currentLayerZ = layers[i][0]?.z;
      const previousLayerZ = layers[i - 1][0]?.z;
      if ((currentLayerZ - previousLayerZ) === voxelSize) {
        mergeStackedSingles(voxelMap, previousLayerZ, currentLayerZ);
      }
    }
    // Print voxel stats at the end
    printVoxelStats(voxelMap);
    const endTime = performance.now();
    console.log(`Voxelization time: ${(endTime - startTime).toFixed(2)} ms`);
    return { voxelMap };
}

function iterativePatternSearch(group, layerCount = 0, bottomGroup = [], forcedCorner = null, forbiddenCorner = null) {

  // 1) Initial greedy pass (returns { voxelMap, singles })
  // let { voxelMap: greedyMatches, singles } =  greedyFromCorners(group, layerCount, forcedCorner, forbiddenCorner);
  let { voxelMap: greedyMatches, singles } = globalPatternSearchGreedy(group, layerCount, true);
  let previousSingles = singles.length + 1; // force at least one loop
  const MAX_ILP_GROUP = 100;

  // 2) Iterate: free weakly connected gadgets, then ILP-fill connected single groups
  while (singles.length < previousSingles) {
    previousSingles = singles.length;

    // Free all non-connected voxels or neighbors of singles
    const { iterateSingle, updatedMatches } = freeUnconnectedVoxels(singles, greedyMatches);
    greedyMatches = updatedMatches;
    // drawSingles(iterateSingle);
    // Partition singles into connected groups
    const singleGroups = divideIntoConnectedGroups(iterateSingle);

    // ILP per group (with recursive splitting for large groups)
    const ilpMatches = solveGroupsWithILP(singleGroups, bottomGroup, MAX_ILP_GROUP, layerCount);
    greedyMatches.push(...ilpMatches);

    // Recompute singles from the updated map
    singles = greedyMatches
      .filter(v => v.type === "single")
      .map(v => v.singles[0]);
  }

  // 3) Final merge pass (2x2 → 2x4, plus sandwich variants)
  greedyMatches = mergeAdjacentpattern(greedyMatches);
  return greedyMatches;
}

// ============================= Layer and Group Division ============================== //
function divideIntoLayers(voxels) {
  const layerMap = {};
  for (const pt of voxels) {
    const z = pt.z;
    if (!layerMap[z]) layerMap[z] = [];
    layerMap[z].push(pt);
  }

  const sortedZ = Object.keys(layerMap).map(Number).sort((a, b) => a - b);
  const layers = [];
  for (const z of sortedZ) {
    layers.push(layerMap[z]);
  }

  return layers;
}

function divideIntoConnectedGroups(voxels) {
  // Use simple 2D flood-fill based on x, y distance
  // Voxels within diagonal reach (under voxelSize^2) are considered connected
  const key = (x, y) => `${x},${y}`;
  const voxelMap = new Map();

  // Build a 2D lookup map for this layer
  for (const v of voxels) {
    voxelMap.set(key(v.x, v.y), v);
  }

  const visited = new Set();
  const groups = [];

  for (const v of voxels) {
    const k = key(v.x, v.y);
    if (visited.has(k)) continue;

    const queue = [v];
    const group = [];
    visited.add(k);

    while (queue.length) {
      const current = queue.shift();
      group.push(current);

      for (const candidateKey of voxelMap.keys()) {
        if (visited.has(candidateKey)) continue;

        const [nx, ny] = candidateKey.split(',').map(Number);
        const dx = nx - current.x;
        const dy = ny - current.y;
        const distSq = dx * dx + dy * dy;

        if (distSq <= voxelSize * voxelSize + 1e-3) {
          visited.add(candidateKey);
          queue.push(voxelMap.get(candidateKey));
        }
      }
    }
    groups.push(group);
  }
  return groups;
}

function findOverhangsPattern(previousLayer, currentGroup) {
  const key = (x, y) => `${x},${y}`;
  const round = (n) => Math.round(n * 1000) / 1000;

  // Map to store overhang voxel positions
  const overhangMap = new Map();
  const overhangKeys = new Set();

  // Create a quick lookup for voxel positions in the previous layer
  const prevLookup = new Set(previousLayer.map(p => key(round(p.x), round(p.y))));

  // Identify overhangs (voxels with no support underneath)
  for (const pt of currentGroup) {
    const k = key(round(pt.x), round(pt.y));
    if (!prevLookup.has(k)) {
      overhangMap.set(k, pt);
      overhangKeys.add(k);
    }
  }

  const result = [];
  const consumedCenters = new Set();

  // Lookup map of all voxel positions in this group
  const groupLookup = new Map(currentGroup.map(pt => [key(round(pt.x), round(pt.y)), pt]));

  // Define available pattern types and their footprint (relative anchor points)
  const offsets = {
    "2x4": [[0, 0], [voxelSize, 0], [2 * voxelSize, 0], [3 * voxelSize, 0],
            [0, voxelSize], [voxelSize, voxelSize], [2 * voxelSize, voxelSize], [3 * voxelSize, voxelSize]],
    "2x3": [[0, 0], [voxelSize, 0], [2 * voxelSize, 0],
            [0, voxelSize], [voxelSize, voxelSize], [2 * voxelSize, voxelSize]],
    "2x2": [[0, 0], [voxelSize, 0], [0, voxelSize], [voxelSize, voxelSize]],
  };

  // For each pattern, try all 4 possible anchor points (bottom-left, bottom-right, top-right, top-left)
  for (const [name, pattern] of Object.entries(offsets)) {
    const anchorIndices = pattern.map((_, i) => i); // try all pattern points as anchors
    for (const anchorIdx of anchorIndices) {
      const [anchorDx, anchorDy] = pattern[anchorIdx];

      for (const overhangKey of overhangKeys) {
        const centerPt = overhangMap.get(overhangKey);

        for (let rot = 0; rot < 4; rot++) {
          const rotationFn = rotationMap[rot];
          const rotatedPattern = pattern.map(offset => rotationFn(offset));
          const [rotAnchorDx, rotAnchorDy] = rotationFn([anchorDx, anchorDy]);

          const occupied = [];
          let overhangCount = 0;
          let isValid = true;

          for (const [dx, dy] of rotatedPattern) {
            const x = round(centerPt.x + (dx - rotAnchorDx));
            const y = round(centerPt.y + (dy - rotAnchorDy));
            const posKey = key(x, y);

            if (consumedCenters.has(posKey) || !groupLookup.has(posKey)) {
              isValid = false;
              break;
            }
            occupied.push(groupLookup.get(posKey));
            if (overhangKeys.has(posKey)) overhangCount++;
          }

          if (!isValid) continue;

          const nonOverhangCount = pattern.length - overhangCount;
          const nonOverhangPts = occupied.filter(v => !overhangKeys.has(key(round(v.x), round(v.y))));
          const allSameY = nonOverhangPts.length === 4 && nonOverhangPts.every(pt2 => pt2.y === nonOverhangPts[0].y);

          const withinPatternLimits =
            (name === "2x4" &&
              nonOverhangCount === 4 &&
              overhangCount === 4 &&
              (() => {
                const overPts = occupied.filter(v => overhangKeys.has(key(round(v.x), round(v.y))));
                const allSameY = overPts.every(pt => pt.y === overPts[0].y);
                const allSameX = overPts.every(pt => pt.x === overPts[0].x);
                return !(allSameY || allSameX);
              })()
            ) ||
            (name === "2x3" && nonOverhangCount === 4 && overhangCount === 2) ||
            (name === "2x2" && nonOverhangCount >= 2 && overhangCount >= 2);

          if (withinPatternLimits && !allSameY) {
            result.push({
              name,
              type: "gadget",
              singles: occupied,
              centers: [...occupied],
              origin: centerPt
            });
            occupied.forEach(v => consumedCenters.add(key(round(v.x), round(v.y))));
          }
        }
      }
    }
  }
  // Search pattern among remaining overhangs
  const remainingGroup = currentGroup.filter(pt => !consumedCenters.has(key(round(pt.x), round(pt.y))));
  const overhangOnly = remainingGroup.filter(pt => !prevLookup.has(key(round(pt.x), round(pt.y))));
  // const extraMatches = globalPatternSearchGreedy(overhangOnly);
  // const extraMatches = optimizeWithILP(overhangOnly);
  const extraMatches = iterativePatternSearch(overhangOnly);
  
  // Visualize physical links between overhang-connected voxels
  const voxelKey = (voxel) => `${Math.round(voxel.origin.x)},${Math.round(voxel.origin.y)},${Math.round(voxel.origin.z)}`;
  const added = new Set();
  const pairs = [];

  const semiOverhangVoxels = result.filter(v => v.type === "gadget");
  const overhangVoxels  = extraMatches.filter(v => v.type === "gadget");

  const allVoxels = [...overhangVoxels, ...semiOverhangVoxels];

  // Loop over voxel pairs only once
  for (let i = 0; i < allVoxels.length; i++) {
    const a = allVoxels[i];
    const aIsSemi = semiOverhangVoxels.includes(a);

    for (let j = i + 1; j < allVoxels.length; j++) {
      const b = allVoxels[j];
      if (a === b) continue;

      const bIsSemi = semiOverhangVoxels.includes(b);
      // Skip if both are semi-overhangs
      if (aIsSemi && bIsSemi) continue;

      const keyA = voxelKey(a);
      const keyB = voxelKey(b);
      const pairKey = keyA < keyB ? `${keyA}|${keyB}` : `${keyB}|${keyA}`;
      if (added.has(pairKey)) continue;

      for (const centerA of a.singles) {
        for (const centerB of b.singles) {
          const dx = Math.abs(centerA.x - centerB.x);
          const dy = Math.abs(centerA.y - centerB.y);
          const dz = Math.abs(centerA.z - centerB.z);

          if (dz === 0 && ((dx === voxelSize && dy === 0) || (dy === voxelSize && dx === 0))) {
            pairs.push([centerA, centerB]);
            added.add(pairKey);
            break;
          }
        }
        if (added.has(pairKey)) break;
      }
    }
  }

  if (pairs.length > 0) {
    // drawBasisBetweenVoxels(pairs, allVoxels);
  }

  return {
    matches: result.concat(extraMatches),
    remainingGroup: remainingGroup.filter(pt => !extraMatches.flatMap(r => r.singles).includes(pt))
  };
}

// ============================= Global Pattern Search ============================== //

// Corner-based sorting: 0=top-left, 1=top-right, 2=bottom-right, 3=bottom-left
function sortByCorner(points, cornerIndex = 0) {
  const cmp = [
    (a, b) => a.y - b.y || a.x - b.x,   // top-left
    (a, b) => a.y - b.y || b.x - a.x,   // top-right
    (a, b) => b.y - a.y || b.x - a.x,   // bottom-right
    (a, b) => b.y - a.y || a.x - b.x,   // bottom-left
  ][cornerIndex % 4];
  return points.slice().sort(cmp);
}

function greedyFromCorners(group, layerCount = 0, forcedCorner = null, forbiddenCorner = null) {
  let best = null;
  let bestCorner = null;
  for (let corner = 0; corner < 4; corner++) {
    if (forcedCorner !== null && corner !== forcedCorner) continue;
    if (forbiddenCorner !== null && corner === forbiddenCorner) continue;
    const res = globalPatternSearchGreedy(group, layerCount, true, corner, /*drawCorner*/ false);
    if (!best || res.singles.length < best.singles.length) {
      best = res;
      bestCorner = corner;
    }
  }
  // Fallback if filters excluded all corners: allow any corner
  if (!best) {
    for (let corner = 0; corner < 4; corner++) {
      const res = globalPatternSearchGreedy(group, layerCount, true, corner, /*drawCorner*/ false);
      if (!best || res.singles.length < best.singles.length) {
        best = res;
        bestCorner = corner;
      }
    }
  }
  __lastGreedyCornerChosen = bestCorner;
  // After choosing the best corner, mark it once
  const sortedForBest = sortByCorner(group, bestCorner ?? 0);
  if (sortedForBest.length > 0) {
    addOriginMarker(sortedForBest[0]);
  }
  return best; // shape: { voxelMap, singles }
}

// ===== Greedy helpers (modularized) ===== //
function getCornerDir(cornerIndex = 0) {
  // 0=top-left (+x, +y), 1=top-right (-x, +y), 2=bottom-right (-x, -y), 3=bottom-left (+x, -y)
  return [
    { dx:  1, dy:  1 },
    { dx: -1, dy:  1 },
    { dx: -1, dy: -1 },
    { dx:  1, dy: -1 },
  ][cornerIndex % 4];
}

function pickAnchorForDir(rotatedOffsets, dir) {
  // Choose anchor opposite to growth direction so gadget expands inward from corner
  const scored = rotatedOffsets.map(([dx, dy], i) => ({ i, sdx: dx * dir.dx, sdy: dy * dir.dy }));
  const minSdx = Math.min(...scored.map(o => o.sdx));
  const cand = scored.filter(o => o.sdx === minSdx);
  const minSdy = Math.min(...cand.map(o => o.sdy));
  const idx = cand.find(o => o.sdy === minSdy).i;
  return rotatedOffsets[idx]; // [ax, ay]
}

function preplaceFromCorner(centerMap, group, cornerIndex, layerCount, patternOffsets, rotationMap, voxelSize) {
  // === Pre-place two 2x3 gadgets every two layers: on two consecutive layers, then skip two ===
  const preplaced = [];
  const L = Math.floor(layerCount);
  const twoOnTwoOff = (L % 4 === 2 || L % 4 === 3); // place on two consecutive layers, then skip two
  if (!twoOnTwoOff) return preplaced;

  const sortedGroup = sortByCorner(group, cornerIndex);
  if (sortedGroup.length === 0) return preplaced;

  const basePt = sortedGroup[0];
  const dir = getCornerDir(cornerIndex);
  const key2D = (x, y) => `${Math.round(x * 1000) / 1000},${Math.round(y * 1000) / 1000}`;

  // First gadget: 2x3 horizontal (rotation 0)
  {
    const offsets = patternOffsets["2x3"];
    const rotationIdx = 0;
    const rotated = offsets.map(off => rotationMap[rotationIdx](off));
    const [ax, ay] = pickAnchorForDir(rotated, dir);
    const coords = rotated.map(([dx, dy]) => key2D(basePt.x + (dx - ax) * voxelSize, basePt.y + (dy - ay) * voxelSize));
    if (coords.every(k => centerMap.has(k))) {
      const pts = coords.map(k => centerMap.get(k));
      preplaced.push({ name: "2x3", type: "gadget", singles: pts, centers: pts, origin: basePt });
      coords.forEach(k => centerMap.delete(k));
    }
  }

  // Second gadget: 2x3 vertical (rotation 1), two voxels inward along Y
  {
    const offsets = patternOffsets["2x3"];
    const rotationIdx = 1;
    const rotated = offsets.map(off => rotationMap[rotationIdx](off));
    const [ax, ay] = pickAnchorForDir(rotated, dir);
    const origin2 = { x: basePt.x, y: basePt.y + dir.dy * voxelSize * 2, z: basePt.z };
    const coords = rotated.map(([dx, dy]) => key2D(origin2.x + (dx - ax) * voxelSize, origin2.y + (dy - ay) * voxelSize));
    if (coords.every(k => centerMap.has(k))) {
      const pts = coords.map(k => centerMap.get(k));
      preplaced.push({ name: "2x3", type: "gadget", singles: pts, centers: pts, origin: origin2 });
      coords.forEach(k => centerMap.delete(k));
    }
  }

  // Bottom-row fill from corner: try 2x4, then 2x3, then 2x2 (horizontal)
  if (centerMap.size > 0) {
    const xStart = basePt.x;
    const yStart = basePt.y;
    const xs = Array.from(centerMap.values()).map(p => p.x);
    if (xs.length) {
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const meshWidth = maxX - minX;
      const steps = Math.floor(meshWidth / voxelSize);
      for (let i = 0; i < steps; i++) {
        for (const pattern of ["2x4", "2x3", "2x2"]) {
          const ro = patternOffsets[pattern].map(offset => rotationMap[0](offset));
          const minDx = Math.min(...ro.map(p => p[0]));
          const minDy = Math.min(...ro.map(p => p[1]));
          const adjusted = ro.map(([dx, dy]) => [dx - minDx, dy - minDy]);
          const origin = { x: xStart + dir.dx * i * voxelSize, y: yStart, z: basePt.z };
          const coords = adjusted.map(([dx, dy]) => key2D(origin.x + dx * voxelSize, origin.y + dy * voxelSize));
          if (coords.every(k => centerMap.has(k))) {
            const pts = coords.map(k => centerMap.get(k));
            preplaced.push({ name: pattern, type: "gadget", singles: pts, centers: pts, origin });
            coords.forEach(k => centerMap.delete(k));
            break;
          }
        }
      }
    }
  }

  // Leftmost-column fill from corner: try 2x4, then 2x3, then 2x2 (vertical)
  if (centerMap.size > 0) {
    const columnOrigin = { x: basePt.x, y: basePt.y, z: basePt.z };
    const ys = Array.from(centerMap.values()).map(p => p.y);
    if (ys.length) {
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const meshHeight = maxY - minY;
      const maxStepsY = Math.floor(meshHeight / voxelSize);
      let j = 0;
      while (j < maxStepsY) {
        let placed = false;
        for (const pattern of ["2x4", "2x3", "2x2"]) {
          const ro = patternOffsets[pattern].map(offset => rotationMap[1](offset));
          const minDx = Math.min(...ro.map(p => p[0]));
          const minDy = Math.min(...ro.map(p => p[1]));
          const adjusted = ro.map(([dx, dy]) => [dx - minDx, dy - minDy]);
          const origin = { x: columnOrigin.x, y: columnOrigin.y + dir.dy * j * voxelSize, z: columnOrigin.z };
          const coords = adjusted.map(([dx, dy]) => key2D(origin.x + dx * voxelSize, origin.y + dy * voxelSize));
          if (coords.every(k => centerMap.has(k))) {
            const pts = coords.map(k => centerMap.get(k));
            preplaced.push({ name: pattern, type: "gadget", singles: pts, centers: pts, origin });
            coords.forEach(k => centerMap.delete(k));
            const height = new Set(ro.map(p => p[1])).size;
            j += height;
            placed = true;
            break;
          }
        }
        if (!placed) j++;
      }
    }
  }

  return preplaced;
}

function splitUntilBelowMax(subgroup, maxSize) {
  const out = [];
  const stack = [subgroup];
  while (stack.length) {
    const chunk = stack.pop();
    if (!chunk || !chunk.length) continue;
    if (chunk.length > maxSize) {
      const parts = splitLargeGroupIntoTwo(chunk);
      if (parts && parts.length) {
        for (const p of parts) if (p && p.length) stack.push(p);
      } else {
        // Fallback to guarantee progress
        const mid = Math.floor(chunk.length / 2);
        stack.push(chunk.slice(0, mid), chunk.slice(mid));
      }
    } else {
      out.push(chunk);
    }
  }
  return out;
}

function solveGroupsWithILP(singleGroups, bottomGroup, maxIlpGroup, layerCount = 0) {
  const matches = [];
  for (const subgroup of singleGroups) {
    const chunks = splitUntilBelowMax(subgroup, maxIlpGroup);
    for (const chunk of chunks) {
      const ilpMatches = optimizeWithILP(chunk, bottomGroup, layerCount);
      matches.push(...ilpMatches);
    }
  }
  return matches;
}

function buildGreedyCandidates(centerMap, remainingPts, patternOffsets, rotationMap, rotationOrder, patternCost, voxelSize) {
  const candidates = [];
  const patternOrder = ["2x4", "2x3", "2x2"];
  for (const patternName of patternOrder) {
    const offsets = patternOffsets[patternName];
    const cost = patternCost[patternName];
    for (let idx = 0; idx < 4; idx++) {
      const rotationIdx = rotationOrder[idx];
      for (const pt of remainingPts) {
        const ro = offsets.map(offset => rotationMap[rotationIdx](offset));
        const minDx = Math.min(...ro.map(p => p[0]));
        const minDy = Math.min(...ro.map(p => p[1]));
        const adjusted = ro.map(([dx, dy]) => [dx - minDx, dy - minDy]);
        const coords = adjusted.map(([dx, dy]) => `${pt.x + dx * voxelSize},${pt.y + dy * voxelSize}`);
        if (coords.every(k => centerMap.has(k))) {
          candidates.push({
            name: patternName,
            cost,
            occupied: new Set(coords),
            singles: coords.map(k => centerMap.get(k)),
            origin: pt,
            rotation: rotationIdx
          });
        }
      }
    }
  }
  return candidates;
}

function selectNonOverlapping(candidates) {
  const used = new Set();
  const chosen = [];
  for (const cand of candidates) {
    if ([...cand.occupied].some(k => used.has(k))) continue;
    chosen.push(cand);
    cand.occupied.forEach(k => used.add(k));
  }
  return { chosen, used };
}

function emitGreedyVoxelMap(chosen, remainingPts, usedKeys) {
  const voxelMap = [];
  chosen.forEach(p => {
    voxelMap.push({ name: p.name, type: "gadget", singles: p.singles, centers: [...p.singles], origin: p.origin });
  });
  for (const pt of remainingPts) {
    const k = `${Math.round(pt.x * 1000) / 1000},${Math.round(pt.y * 1000) / 1000}`;
    if (!usedKeys.has(k)) {
      voxelMap.push({ name: null, type: "single", singles: [pt], centers: [] });
    }
  }
  return voxelMap;
}

function globalPatternSearchGreedy(group, layerCount = 0, returnSingles = false, cornerIndex = 0, drawCorner = false) {
  const round = (n) => Math.round(n * 1000) / 1000;
  const key = (x, y) => `${round(x)},${round(y)}`;

  const patternOffsets = {
    "2x4": [[0, 0], [1, 0], [2, 0], [3, 0], [0, 1], [1, 1], [2, 1], [3, 1]],
    "2x3": [[0, 0], [1, 0], [2, 0], [0, 1], [1, 1], [2, 1]],
    "2x2": [[0, 0], [1, 0], [0, 1], [1, 1]],
  };
  const patternCost = { "2x4": 1, "2x3": 2, "2x2": 3 };

  // Build a map of voxel positions to ensure consistent keys
  const centerMap = new Map(group.map(pt => [key(pt.x, pt.y), pt]));
  const sorted = sortByCorner(group, cornerIndex);

  const rotationMapLocal = [
    ([dx, dy]) => [dx, dy],
    ([dx, dy]) => [-dy, dx],
    ([dx, dy]) => [-dx, -dy],
    ([dx, dy]) => [dy, -dx]
  ];
  // Swap orientation every 2 layers
  const rotationOrder = Math.floor(layerCount) % 2 === 0 ? [0, 1, 2, 3] : [2, 3, 0, 1];

  // Preplace from selected corner with 2-on / 2-off cadence; mutates centerMap
  const preplaced = preplaceFromCorner(centerMap, group, cornerIndex, layerCount, patternOffsets, rotationMapLocal, voxelSize);

  // Now run greedy search using the updated centerMap (with preplaced gadgets' points removed)
  const remainingPts = Array.from(centerMap.values());
  const sortedRemaining = sortByCorner(remainingPts, cornerIndex);
  const candidates = buildGreedyCandidates(centerMap, sortedRemaining, patternOffsets, rotationMapLocal, rotationOrder, patternCost, voxelSize);
  const { chosen, used } = selectNonOverlapping(candidates);
  const voxelMap = emitGreedyVoxelMap(chosen, remainingPts, used);

  // Add origin marker for this group, if requested
  if (drawCorner && sorted.length > 0) {
    addOriginMarker(sorted[0]);
  }

  if (returnSingles) {
    const singles = remainingPts.filter(pt => !used.has(`${round(pt.x)},${round(pt.y)}`));
    return { voxelMap: [...preplaced, ...voxelMap], singles };
  }
  return [...preplaced, ...voxelMap];
}



// ============================= Helper: Merge Adjacent 2x2 Gadgets to 2x4 ============================== //
function mergeAdjacentpattern(gadgets) {
  const round = n => Math.round(n * 1000) / 1000;
  const mergedIndices = new Set();
  const mergedGadgets = [];
  // Strict 4-neighbor adjacency (no diagonals, no overlap)
  const isGridNeighbor2by2 = (a, b) =>
    a.z === b.z &&
    ((Math.abs(a.x - b.x) === voxelSize && a.y === b.y) ||
     (Math.abs(a.y - b.y) === voxelSize && a.x === b.x));
  for (let i = 0; i < gadgets.length; i++) {
    const g1 = gadgets[i];
    if (g1.type !== "gadget" || g1.name !== "2x2" || mergedIndices.has(i)) continue;

    for (let j = i + 1; j < gadgets.length; j++) {
      const g2 = gadgets[j];
      if (g2.type !== "gadget" || g2.name !== "2x2" || mergedIndices.has(j)) continue;

      const allPts = [...g1.singles, ...g2.singles];
      const zs = new Set(allPts.map(p => p.z));
      if (zs.size !== 1) continue;

      const areNeighboring = g1.singles.some(pt1 =>
        g2.singles.some(pt2 => isGridNeighbor2by2(pt1, pt2))
      );

      if (areNeighboring) {
        // Ensure 8 distinct voxels forming a 2x4 footprint (no diagonal-only contact)
        const uniq = Array.from(new Map(allPts.map(p => [`${p.x},${p.y},${p.z}`, p])).values());
        const xs = new Set(uniq.map(p => p.x));
        const ys = new Set(uniq.map(p => p.y));
        const is2x4 = uniq.length === 8 && ((xs.size === 4 && ys.size === 2) || (xs.size === 2 && ys.size === 4));
        if (!is2x4) {
          continue;
        }
        // Orientation-aware sort for merged points
        const sorted = sortGadgetPoints(uniq);
        const mergedGadget = {
          name: "2x4",
          type: "gadget",
          singles: sorted,
          centers: sorted,
          origin: sorted[0]
        };
        mergedGadgets.push(mergedGadget);
        mergedIndices.add(i);
        mergedIndices.add(j);
        break;
      }
    }
  }

  // Merge 2x3 + 2x2 + 2x3 into two 2x4 gadgets (sandwich pattern)
  const originKey = pt => `${round(pt.x)},${round(pt.y)},${round(pt.z)}`;
  // Determine footprint orientation of a 2x3 gadget: "H" if 3 wide x 2 tall, "V" if 2 wide x 3 tall
  function gadget2x3Orientation(g) {
    const xs = new Set(g.singles.map(p => p.x));
    const ys = new Set(g.singles.map(p => p.y));
    if (xs.size === 3 && ys.size === 2) return "H";
    if (xs.size === 2 && ys.size === 3) return "V";
    return null; // unexpected
  }
  // Strict 4-neighbor adjacency (shared edge)
  const isGridNeighbor = (a, b) =>
    a.z === b.z &&
    ((Math.abs(a.x - b.x) === voxelSize && a.y === b.y) ||
     (Math.abs(a.y - b.y) === voxelSize && a.x === b.x));

  for (let i = 0; i < gadgets.length; i++) {
    const g1 = gadgets[i];
    if (g1.name !== "2x3" || g1.type !== "gadget" || mergedIndices.has(i)) continue;

    for (let j = 0; j < gadgets.length; j++) {
      const g2 = gadgets[j];
      if (j === i || g2.name !== "2x2" || g2.type !== "gadget" || mergedIndices.has(j)) continue;
      // Check that g1(2x3) and g2(2x2) share exactly one full edge (2 touching voxels)
      const neighSetG1 = new Map();
      g1.singles.forEach(a => g2.singles.forEach(b => { if (isGridNeighbor(a, b)) neighSetG1.set(`${b.x},${b.y},${b.z}`, b); }));
      const neighborsPattern1 = Array.from(neighSetG1.values());
      if (neighborsPattern1.length !== 2) continue;

      // Determine orientation of g1 and require the third gadget g3 to have the SAME orientation
      const ori1 = gadget2x3Orientation(g1);
      if (!ori1) continue;

      // Axis alignment between g1 and g2 must be consistent with ori1 (i.e., g2 lies along the long axis side)
      const axisXAligned = (ori1 === "H") && g1.singles.every(p => p.y === g2.singles[0].y || p.y === g2.singles[0].y + voxelSize || p.y === g2.singles[0].y - voxelSize);
      const axisYAligned = (ori1 === "V") && g1.singles.every(p => p.x === g2.singles[0].x || p.x === g2.singles[0].x + voxelSize || p.x === g2.singles[0].x - voxelSize);
      if (!(axisXAligned || axisYAligned)) continue;

      // Find a g3 (2x3) on the opposite side of g2, same orientation as g1, edge-adjacent to g2 by exactly two voxels
      const g3 = gadgets.find((g, idx) => {
        if (g === g1 || g === g2) return false;
        if (g.name !== "2x3" || g.type !== "gadget" || mergedIndices.has(idx)) return false;
        if (g.origin.z !== g2.origin.z) return false;
        // Must have same orientation as g1
        const ori3 = gadget2x3Orientation(g);
        if (ori3 !== ori1) return false;
        // Must share exactly 2 voxels with g2 along an edge
        const neighSetG3 = new Map();
        g2.singles.forEach(a => g.singles.forEach(b => { if (isGridNeighbor(a, b)) neighSetG3.set(`${a.x},${a.y},${a.z}`, a); }));
        const neighborsPattern2 = Array.from(neighSetG3.values());
        if (neighborsPattern2.length !== 2) return false;
        // Ensure g1 and g3 are on opposite sides of g2 (use bounding boxes)
        const minX1 = Math.min(...g1.singles.map(p => p.x)), maxX1 = Math.max(...g1.singles.map(p => p.x));
        const minY1 = Math.min(...g1.singles.map(p => p.y)), maxY1 = Math.max(...g1.singles.map(p => p.y));
        const minX3 = Math.min(...g.singles.map(p => p.x)), maxX3 = Math.max(...g.singles.map(p => p.x));
        const minY3 = Math.min(...g.singles.map(p => p.y)), maxY3 = Math.max(...g.singles.map(p => p.y));
        const minX2 = Math.min(...g2.singles.map(p => p.x)), maxX2 = Math.max(...g2.singles.map(p => p.x));
        const minY2 = Math.min(...g2.singles.map(p => p.y)), maxY2 = Math.max(...g2.singles.map(p => p.y));
        const separatedAlongX = (maxX1 <= minX2 - voxelSize && maxX2 <= minX3 - voxelSize) || (maxX3 <= minX2 - voxelSize && maxX2 <= minX1 - voxelSize);
        const separatedAlongY = (maxY1 <= minY2 - voxelSize && maxY2 <= minY3 - voxelSize) || (maxY3 <= minY2 - voxelSize && maxY2 <= minY1 - voxelSize);
        if (!(separatedAlongX || separatedAlongY)) return false;
        // Stash for use outside finder
        g.__neighborsFromG2 = neighborsPattern2;
        return true;
      });
      if (!g3) continue;
      const neighborsPattern2 = g3.__neighborsFromG2 || [];
      const g3Index = gadgets.indexOf(g3);

      // First 2x4 gadget (g1 + two neighbors from g2)
      const allPts1 = [...g1.singles, ...neighborsPattern1];
      const uniq1 = Array.from(new Map(allPts1.map(p => [`${p.x},${p.y},${p.z}`, p])).values());
      const xs1 = new Set(uniq1.map(p => p.x));
      const ys1 = new Set(uniq1.map(p => p.y));
      const valid2x4_1 = uniq1.length === 8 && ((xs1.size === 4 && ys1.size === 2) || (xs1.size === 2 && ys1.size === 4));
      if (!valid2x4_1) continue;
      const sorted1 = sortGadgetPoints(uniq1);
      const mergedGadget1 = { name: "2x4", type: "gadget", singles: sorted1, centers: sorted1, origin: sorted1[0] };
      mergedGadgets.push(mergedGadget1);

      // Second 2x4 gadget (g3 + two neighbors from g2)
      const allPts2 = [...g3.singles, ...neighborsPattern2];
      const uniq2 = Array.from(new Map(allPts2.map(p => [`${p.x},${p.y},${p.z}`, p])).values());
      const xs2 = new Set(uniq2.map(p => p.x));
      const ys2 = new Set(uniq2.map(p => p.y));
      const valid2x4_2 = uniq2.length === 8 && ((xs2.size === 4 && ys2.size === 2) || (xs2.size === 2 && ys2.size === 4));
      if (!valid2x4_2) continue;
      const sorted2 = sortGadgetPoints(uniq2);
      const mergedGadget2 = { name: "2x4", type: "gadget", singles: sorted2, centers: sorted2, origin: sorted2[0] };
      mergedGadgets.push(mergedGadget2);

      mergedIndices.add(i);
      mergedIndices.add(j);
      mergedIndices.add(g3Index);
      break;
    }
  }
  const filtered = gadgets.filter((_, idx) => !mergedIndices.has(idx));
  // After creating mergedGadgets, sort them as 2x4 gadgets are typically sorted (by origin.y, then origin.x)
  mergedGadgets.sort((a, b) => a.origin.y - b.origin.y || a.origin.x - b.origin.x);
  return [...filtered, ...mergedGadgets];
}

// Split a large connected group into two parts along the dominant axis, then re-check connectivity
function splitLargeGroupIntoTwo(group) {
  if (!group || group.length === 0) return [];
  const xs = group.map(p => p.x);
  const ys = group.map(p => p.y);
  const rangeX = Math.max(...xs) - Math.min(...xs);
  const rangeY = Math.max(...ys) - Math.min(...ys);
  const axis = rangeX >= rangeY ? 'x' : 'y';

  // Sort by the chosen axis and split at the median
  const sorted = group.slice().sort((a, b) => (a[axis] - b[axis]));
  const mid = Math.floor(sorted.length / 2);
  const threshold = (sorted[Math.max(0, mid - 1)][axis] + sorted[mid][axis]) / 2;
  const left = sorted.filter(p => p[axis] <= threshold);
  const right = sorted.filter(p => p[axis] > threshold);

  // Re-run connectivity on each half to ensure we return connected subgroups
  const groupsLeft = left.length ? divideIntoConnectedGroups(left) : [];
  const groupsRight = right.length ? divideIntoConnectedGroups(right) : [];
  return [...groupsLeft, ...groupsRight];
}


function freeUnconnectedVoxels(singles, matches) {
  const round = n => Math.round(n * 1000) / 1000;
  const key = pt => `${round(pt.x)},${round(pt.y)},${round(pt.z)}`;

  const singleMap = new Map(singles.map(pt => [key(pt), pt]));
  const iterateSet = new Set();
  const iterateSingle = [];
  const keepMatches = new Set(matches);

  // Build a fast lookup map for all gadget voxels
  const gadgetMap = new Map();
  for (const match of matches) {
    if (match.type === "gadget") {
      for (const voxel of match.singles) {
        gadgetMap.set(key(voxel), match);
      }
    }
  }

  for (const single of singles) {
    const neighbors = [];
    for (const dx of [-voxelSize, 0, voxelSize]) {
      for (const dy of [-voxelSize, 0, voxelSize]) {
        if (Math.abs(dx) + Math.abs(dy) !== voxelSize) continue;
        const nx = single.x + dx;
        const ny = single.y + dy;
        const nz = single.z;
        const neighborKey = key({ x: nx, y: ny, z: nz });
        if (singleMap.has(neighborKey) || gadgetMap.has(neighborKey)) {
          neighbors.push({ key: neighborKey, isGadget: gadgetMap.has(neighborKey) });
        }
      }
    }

    const nb = neighbors.length;
    const nbGadget = neighbors.filter(n => n.isGadget).length;
    const nbSingle = nb - nbGadget;

    if (nb <= 1) {
      // Delete isolated single
      continue;
    } else if (nb === 2) {
      if (nbGadget === 2) {
        // Connected to 2 gadgets, keep it
        continue;
      } else {
        // Remove all gadgets connected to these neighbors
        for (const n of neighbors) {
          if (n.isGadget) {
            const match = gadgetMap.get(n.key);
            if (keepMatches.has(match)) {
              keepMatches.delete(match);
              for (const voxel of match.singles) {
                const voxelKey = key(voxel);
                if (!iterateSet.has(voxelKey)) {
                  iterateSet.add(voxelKey);
                  iterateSingle.push(voxel);
                }
              }
            }
          }
        }
      }
    } else if (nb >= 3) {
      // Remove all gadgets around
      for (const n of neighbors) {
        if (n.isGadget) {
          const match = gadgetMap.get(n.key);
          if (keepMatches.has(match)) {
            keepMatches.delete(match);
            for (const voxel of match.singles) {
              const voxelKey = key(voxel);
              if (!iterateSet.has(voxelKey)) {
                iterateSet.add(voxelKey);
                iterateSingle.push(voxel);
              }
            }
          }
        }
      }
    }
    // Add the single itself to the new single list
    const singleKey = key(single);
    if (!iterateSet.has(singleKey)) {
      iterateSet.add(singleKey);
      iterateSingle.push(single);
    }
  }

  // Post-filter: Remove any single voxel from updatedMatches if its key is in iterateSet
  const cleanedMatches = [...keepMatches].filter(match =>
    !(match.type === "single" && iterateSet.has(key(match.singles[0])))
  );

  return {
    iterateSet,
    iterateSingle,
    updatedMatches: cleanedMatches
  };
}

// =============================  Helper ============================== //
function printVoxelStats(voxelMap) {
  const totalVoxels = voxelMap.length;
  const singles = voxelMap.filter(v => v.type === "single").length;
  const gadgets = voxelMap.filter(v => v.type === "gadget").length;
  const totalCenters = voxelMap.reduce((sum, v) => sum + (v.centers?.length || 0), 0);

  console.log("=== Voxelization Stats ===");
  console.log("Total voxel entries:", totalVoxels);
  console.log("Number of gadgets:", gadgets);
  console.log("Number of singles:", singles);
  console.log("Total voxel centers:", totalCenters);
  console.log("Fraction of singles:", (singles / totalCenters).toFixed(3));
}

//  Gadget Point Sorting

function sortGadgetPoints(points) {
  const cx = points.map(p => p.x);
  const cy = points.map(p => p.y);
  const rangeX = Math.max(...cx) - Math.min(...cx);
  const rangeY = Math.max(...cy) - Math.min(...cy);

  return points.slice().sort((a, b) => {
    if (rangeX >= rangeY) {
      return a.y - b.y || a.x - b.x;  // Horizontal orientation
    } else {
      return a.x - b.x || a.y - b.y;  // Vertical orientation
    }
  });
}
// ============================= ILP Pattern Optimization ============================== //
// Note: solver, model, and tableau are expected to be available globally (via script tags).

function optimizeWithILP(possiblePatterns, patternsBelow = [], layerCount = 0) {
  // Guard clause for empty or falsy input
  if (!possiblePatterns || possiblePatterns.length === 0) {
    console.warn("No patterns provided to ILP.");
    return [];
  }

  if (typeof solver === 'undefined' || !solver.Solve) {
    console.error("solver.Solve is not available. Make sure to include tableau.js, model.js, and solver.js via script tags.");
    return [];
  }

  // Step 1: Generate all valid pattern candidates (2x2, 2x3, 2x4 in 4 rotations)
  const patterns = generatePatternCandidates(possiblePatterns);
  // Step 2: Build ILP model from pattern candidates
  const L = Math.floor(layerCount);
  const model = (L % 2 === 0)
    ? buildEvenLayerILPModel(patterns, patternsBelow)
    : buildOddLayerILPModel(patterns, patternsBelow);
 
  console.log('calling solver.Solve with model:', (L % 2 === 0) ? 'Even Layer' : 'Odd Layer');

  const result = solver.Solve(model);
  if (!result.feasible) {
    console.warn("No feasible solution found.");
    return [];
  }

  // Select patterns from the solution
  const selectedPatterns = [];
  patterns.forEach((pattern, i) => {
    const varName = `p${i}`;
    if (result[varName] === 1) {
      selectedPatterns.push(pattern);
    }
  });

  // console.log("Selected patterns by ILP:", selectedPatterns);
  const voxelMap = [];

  selectedPatterns.forEach(pattern => {
    voxelMap.push({
      name: pattern.name,
      type: "gadget",
      singles: pattern.singles,
      centers: [...pattern.singles],
      origin: pattern.origin
    });
  });

  // Add unmatched voxels as singles
  const usedKeys = new Set(selectedPatterns.flatMap(p => p.singles.map(c => `${c.x},${c.y},${c.z}`)));

  for (const pt of possiblePatterns) {
    const ptKey = `${pt.x},${pt.y},${pt.z}`;
    if (!usedKeys.has(ptKey)) {
      voxelMap.push({
        name: null,
        type: "single",
        singles: [pt],
        centers: []
      });
    }
  }

  return voxelMap;
}
// ============================= Pattern Candidate Generation and ILP Model Builder ============================== //

function generatePatternCandidates(group) {
  const round = n => Math.round(n * 1000) / 1000;
  const key = (x, y) => `${round(x)},${round(y)}`;
  const voxelMap = new Map(group.map(pt => [key(pt.x, pt.y), pt]));
  const patterns = [];

  // Define patterns as base units (always with dx, dy in increasing order, e.g., along x, then y)
  // Orientation is determined by rotation, not by pattern name.
  const patternDefs = {
    "2x4": [
      [0, 0], [1, 0], [2, 0], [3, 0],
      [0, 1], [1, 1], [2, 1], [3, 1]
    ],
    "2x3": [
      [0, 0], [1, 0], [2, 0],
      [0, 1], [1, 1], [2, 1]
    ],
    "2x2": [
      [0, 0], [1, 0],
      [0, 1], [1, 1]
    ],
  };

  const rotationMap = [
    ([dx, dy]) => [dx, dy],
    ([dx, dy]) => [-dy, dx],
    ([dx, dy]) => [-dx, -dy],
    ([dx, dy]) => [dy, -dx],
  ];

  for (const pt of group) {
    for (const [name, offsets] of Object.entries(patternDefs)) {
      for (let rot = 0; rot < 4; rot++) {
        const rotate = rotationMap[rot];
        const rotated = offsets.map(([dx, dy]) => rotate([dx, dy]));
        const minDx = Math.min(...rotated.map(([dx, dy]) => dx));
        const minDy = Math.min(...rotated.map(([dx, dy]) => dy));
        const adjusted = rotated.map(([dx, dy]) => [dx - minDx, dy - minDy]);

        // Calculate the coordinates for the pattern at this rotation and anchor
        const coords = adjusted.map(([dx, dy]) => key(pt.x + dx * voxelSize, pt.y + dy * voxelSize));
        const voxelPts = coords.map(k => voxelMap.get(k));

        if (coords.every(k => voxelMap.has(k))) {
          // Sort the points consistently after rotation:
          // For horizontal patterns (rot % 2 == 0), sort by y then x.
          // For vertical patterns (rot % 2 == 1), sort by x then y.
          const sortedPts = voxelPts.slice().sort((a, b) => {
            const aPos = [a.x, a.y];
            const bPos = [b.x, b.y];
            if (rot % 2 === 0) {
              return aPos[1] - bPos[1] || aPos[0] - bPos[0];
            } else {
              return aPos[0] - bPos[0] || aPos[1] - bPos[1];
            }
          });
          patterns.push({
            name,
            origin: pt,
            centers: sortedPts,
            singles: sortedPts
          });
        }
      }
    }
  }

  return patterns;
}
// ============================= ILP Models for Even and Odd Layers ============================== //

function buildEvenLayerILPModel(patterns, patternsBelow = []) {
  const model = {
    optimize: "cost",
    opType: "min",
    constraints: {},
    variables: {},
    ints: {},
  };

  const weightCoverage = 10.0;
  const weightTypedVoxel = 0.1;
  const weightNeighbor = 1.0;  // tune this

  // Helper: compute vertical neighbors
  function computeVerticalNeighbors(pattern, allPatternsBelow) {
    const neighbors = new Set();

    pattern.centers.forEach(upper => {
      allPatternsBelow.forEach((lowerPattern, j) => {
        lowerPattern.centers.forEach(lower => {
          if (
            upper.x === lower.x &&
            upper.y === lower.y &&
            upper.z === lower.z + voxelSize
          ) {
            neighbors.add(j);
          }
        });
      });
    });

    return neighbors.size;
  }

  patterns.forEach((pattern, i) => {
    const varName = `p${i}`;
    const size = pattern.centers.length;
    const neighborCount = computeVerticalNeighbors(pattern, patternsBelow);
    const cost = (-weightCoverage * size)
               + (weightTypedVoxel)
               - (weightNeighbor * neighborCount);
    model.variables[varName] = { cost };
    model.ints[varName] = 1;
    // Standard center coverage constraints
    pattern.centers.forEach(center => {
      const k = `${center.x},${center.y},${center.z}`;
      model.variables[varName][k] = 1;
      model.constraints[k] = { max: 1 };
    });
  });

  return model;
}

function buildOddLayerILPModel(patterns, patternsBelow = []) {
  const model = {
    optimize: "cost",
    opType: "min",
    constraints: {},
    variables: {},
    ints: {},
  };

  const weightCoverage = 10.0;
  const weightTypedVoxel = 0.1;
  const weightNeighbor = 1.0;  // tune this

  // Helper: compute vertical neighbors
  function computeVerticalNeighbors(pattern, allPatternsBelow) {
    const neighbors = new Set();

    pattern.centers.forEach(upper => {
      allPatternsBelow.forEach((lowerPattern, j) => {
        lowerPattern.centers.forEach(lower => {
          if (
            upper.x === lower.x &&
            upper.y === lower.y &&
            upper.z === lower.z + voxelSize
          ) {
            neighbors.add(j);
          }
        });
      });
    });

    return neighbors.size;
  }

  patterns.forEach((pattern, i) => {
    const varName = `p${i}`;
    const size = pattern.centers.length;
    const neighborCount = computeVerticalNeighbors(pattern, patternsBelow);
    // For odd layers: penalize vertical neighbors (minimize how many gadgets this pattern stands on)
    const cost = (-weightCoverage * size)
               + (weightTypedVoxel)
               + (weightNeighbor * neighborCount);
    model.variables[varName] = { cost };
    model.ints[varName] = 1;
    // Standard center coverage constraints
    pattern.centers.forEach(center => {
      const k = `${center.x},${center.y},${center.z}`;
      model.variables[varName][k] = 1;
      model.constraints[k] = { max: 1 };
    });
  });

  return model;
}
// have 2 models 1 for odd layzer and 1 for even layer 

// the one for even layer need to minimize the number of single why minimize the number of typed voxels and optimize the number of link per voxel
// the one for odd layer need to minimize the number of single and optimize the number of typed voxels an minimize the number of link per voxel (try to place same block as layer bellow ) to later merge them  



// ============================= Merge Stacked Singles Helper ============================== //
function mergeStackedSingles(voxelMap, lowerZ, upperZ) {
  const round = n => Math.round(n * 1000) / 1000;
  const key3D = (pt) => `${round(pt.x)},${round(pt.y)},${round(pt.z)}`;
  const key2D = (pt) => `${round(pt.x)},${round(pt.y)}`;

  const merged = [];
  const toRemove = new Set();

  const upperGadgets = voxelMap.filter(v =>
    v.type === "gadget" && v.singles.length > 0 && round(v.origin.z) === round(upperZ)
  );

  const lowerGadgetsMap = new Map();
  voxelMap.forEach(v => {
    if (
      v.type === "gadget" &&
      v.singles.length > 0 &&
      round(v.origin.z) === round(lowerZ)
    ) {
      const keys = v.singles.map(pt => key2D(pt)).sort().join('|');
      lowerGadgetsMap.set(keys, v);
    }
  });

  for (const upper of upperGadgets) {
    const keys = upper.singles.map(pt => key2D(pt)).sort().join('|');
    const lower = lowerGadgetsMap.get(keys);
    if (lower && lower.name === upper.name) {
      // console.log('Merging',lower, upper)
      merged.push({
        name: `${upper.name}x2`,
        type: "gadget",
        singles: [...lower.singles, ...upper.singles],
        centers: [...lower.centers, ...upper.centers],
        origin: lower.origin
      });
      // console.log('InTO',merged[merged.length - 1]);
      toRemove.add(lower);
      toRemove.add(upper);
    }
  }

  // Filter out merged gadgets
  const newVoxelMap = voxelMap.filter(v => !toRemove.has(v));
  voxelMap.length = 0;
  voxelMap.push(...newVoxelMap, ...merged);
}



// ============================== Draw Helpers ============================== //

function addOriginMarker(pt) {
  const sphere = new THREE.Mesh(
    new THREE.SphereGeometry(10, 16, 16),
    new THREE.MeshStandardMaterial({ color: 0xff69b4 })
  );
  sphere.position.set(pt.x, pt.y, pt.z);
  scene.add(sphere);
}

function drawBasisBetweenVoxels(voxelPairs, allVoxels) {
  for (const [a, b] of voxelPairs) {
    const voxelA = allVoxels.find(v => v?.singles?.some(c => c.x === a.x && c.y === a.y && c.z === a.z));
    const voxelB = allVoxels.find(v => v?.singles?.some(c => c.x === b.x && c.y === b.y && c.z === b.z));

    // console.log("Voxel A:", voxelA);
    // console.log("Voxel B:", voxelB);

    const direction = new THREE.Vector3().subVectors(b, a).normalize();
    const perp = new THREE.Vector3(-direction.y, direction.x, 0).normalize();

    const checkOffsets = [1, 2];

    function countNeighbors(refPt, voxel) {
      let forward = 0;
      let backward = 0;

      for (const mult of checkOffsets) {
        const forwardPt = new THREE.Vector3().addVectors(refPt, perp.clone().multiplyScalar(voxelSize * mult));
        const backwardPt = new THREE.Vector3().addVectors(refPt, perp.clone().multiplyScalar(-voxelSize * mult));

        const hasForward = voxel?.singles?.some(c =>
          (c.x !== refPt.x || c.y !== refPt.y || c.z !== refPt.z) &&
          Math.round(c.x) === Math.round(forwardPt.x) &&
          Math.round(c.y) === Math.round(forwardPt.y) &&
          Math.round(c.z) === Math.round(forwardPt.z)
        );

        const hasBackward = voxel?.singles?.some(c =>
          (c.x !== refPt.x || c.y !== refPt.y || c.z !== refPt.z) &&
          Math.round(c.x) === Math.round(backwardPt.x) &&
          Math.round(c.y) === Math.round(backwardPt.y) &&
          Math.round(c.z) === Math.round(backwardPt.z)
        );

        if (hasForward) forward++;
        if (hasBackward) backward++;
      }

      // console.log(`Voxel at (${refPt.x}, ${refPt.y}, ${refPt.z}) - Forward: ${forward}, Backward: ${backward}`);
      return { forward, backward };
    }

    const aCounts = countNeighbors(a, voxelA);
    const bCounts = countNeighbors(b, voxelB);

    let totalForward = Math.min(aCounts.forward, bCounts.forward);
    let totalBackward = Math.min(aCounts.backward, bCounts.backward);

    let offsetDirection = 1;
    if (totalBackward > totalForward) {
      // console.log("Offsetting backward");
      offsetDirection = -1;
    }

    const offsetAmount = (totalForward + totalBackward <= 1) ? 0.5 * voxelSize : 1.5 * voxelSize;
    const offsetVector = perp.clone().multiplyScalar(offsetAmount * offsetDirection);

    drawLinkBetweenVoxels(a, b, offsetVector);
  }
}

function drawLinkBetweenVoxels(a, b, offset = new THREE.Vector3()) {
  const material = new THREE.MeshStandardMaterial({ color: 0xff0000 });
  const rectLength = voxelSize * 0.8;
  const rectHeight = 5;
  const rectWidth = 20;

  const geometry = new THREE.BoxGeometry(rectLength, rectWidth, rectHeight);
  const mid = new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5).add(offset);
  const mesh = new THREE.Mesh(geometry, material);

  mesh.position.set(mid.x, mid.y, mid.z - voxelSize * 0.5);

  const dx = Math.abs(a.x - b.x);
  const dy = Math.abs(a.y - b.y);

  if (dy > dx) {
    mesh.rotateZ(Math.PI / 2);
  }

  scene.add(mesh);
}

// =Visualize iterateSingle as green spheres
function drawSingles(singles) {
  singles.forEach(pt => {
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(10, 12, 12),
      new THREE.MeshStandardMaterial({ color: 0x00ff00 }) // green
    );
    sphere.position.set(pt.x, pt.y, pt.z);
    scene.add(sphere);
  });
}