import * as THREE from './three/build/three.module.min.js';

export function extractVoxelCenters(geometry, voxelSize = 0.65) {
    // Get all single voxel composing a mesh
    let iterations = 0;
    let rayIterations = 0;
    const startTime = performance.now();

    const triangleCount = geometry.index ? geometry.index.count / 3 : geometry.attributes.position.count / 3;

    const bbox = geometry.boundingBox;
    const min = bbox.min;
    const max = bbox.max;
    const step = voxelSize;
    const result = [];
     
    const round = (n) => Math.round(n * 1000) / 1000;
    for (let x = min.x; x <= max.x; x += step) {
        for (let y = min.y; y <= max.y; y += step) {
            for (let z = min.z; z <= max.z; z += step) {
                iterations++;
                const pt = new THREE.Vector3(
                    round(x + step / 2),
                    round(y + step / 2),
                    round(z + step / 2)
                );
                const directions = [
                    new THREE.Vector3(1, 0, 0),
                    new THREE.Vector3(0, 1, 0),
                    new THREE.Vector3(0, 0, 1)
                ];
                const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ side: THREE.DoubleSide }));
                let insideVotes = 0;

                for (const direction of directions) {
                    rayIterations++;
                    const raycaster = new THREE.Raycaster();
                    raycaster.set(pt, direction.clone().normalize());
                    const intersects = raycaster.intersectObject(mesh, true);

                    // Filter out duplicate hits (same point)
                    const uniquePoints = [];
                    for (const inter of intersects) {
                        if (!uniquePoints.some(p => p.equals(inter.point))) {
                            uniquePoints.push(inter.point);
                        }
                    }

                    if (uniquePoints.length % 2 === 1) {
                        insideVotes++;
                    }
                }

                if (insideVotes >= 2) {
                    result.push(pt.clone());
                }
            }
        }
    }
    const endTime = performance.now();
    console.log(`Voxelization completed in ${iterations} iterations`);
    console.log(`Time elapsed: ${(endTime - startTime).toFixed(2)} ms`);
    console.log(`Total raycasting iterations: ${rayIterations}`);
    console.log(`Triangle count per ray: ${triangleCount}`);
    console.log(`Estimated total triangle-ray intersection checks: ${triangleCount * rayIterations}`);
    return result;
}


const pattern_2x2 = [[0,0,0],[1,0,0],[0,1,0],[1,1,0]];

const pattern_2x3 = [[0,0,0],[1,0,0],[0,1,0],[1,1,0],[0,2,0],[1,2,0]];

const pattern_2x4 = [[0,0,0],[1,0,0],[2,0,0],[3,0,0],[0,1,0],[1,1,0],[2,1,0],[3,1,0]];

const pattern_2x2_stacked = [[0,0,0],[1,0,0],[0,1,0],[1,1,0],
                             [0,0,1],[1,0,1],[0,1,1],[1,1,1]];

const pattern_2x3_stacked = [[0,0,0],[1,0,0],[0,1,0],[1,1,0],[0,2,0],[1,2,0],
                             [0,0,1],[1,0,1],[0,1,1],[1,1,1],[0,2,1],[1,2,1]];     

const pattern_2x4_stacked = [
  [0,0,0],[0,1,0],[1,0,0],[1,1,0],[2,0,0],[2,1,0],[3,0,0],[3,1,0],
  [0,0,1],[0,1,1],[1,0,1],[1,1,1],[2,0,1],[2,1,1],[3,0,1],[3,1,1],
];


export function findPatternsInOrder(points, voxelSize = 0.65, patternList = []) {
  const key = (x, y, z) => `${Math.round(x * 1000) / 1000},${Math.round(y * 1000) / 1000},${Math.round(z * 1000) / 1000}`;
  const round = n => Math.round(n * 1000) / 1000;

  const voxelSet = new Set(points.map(pt => key(pt.x, pt.y, pt.z)));
  const checked = new Set();
  const voxelMap = [];

const patternMap = {
  "2x2": pattern_2x2,
  "2x3": pattern_2x3,
  "2x4": pattern_2x4,
  "2x2x2": pattern_2x2_stacked,
  "2x3x2": pattern_2x3_stacked,
  "2x4x2": pattern_2x4_stacked
};

const patternCost = {
  "2x4x2": 0.8,
  "2x4": 1.0,
  "2x3x2": 1.2,
  "2x3": 2,
  "2x2x2": 1.5,
  "2x2": 3
};

  const voxels = Array.from(voxelSet).map(k => k.split(',').map(Number));
  const allCandidates = [];

  for (const patternName of patternList) {
    const offsets = patternMap[patternName];
    const cost = patternCost[patternName] || 10;

    // Create rotated version of the offsets (90° clockwise around Z axis)
    const rotatedOffsets = offsets.map(([dx, dy, dz]) => [-dy, dx, dz]);

    for (const base of voxels) {
      const k0 = key(base[0], base[1], base[2]);
      if (checked.has(k0)) continue;

      const occupied = offsets.map(([dx, dy, dz]) => key(
        base[0] + dx * voxelSize,
        base[1] + dy * voxelSize,
        base[2] + dz * voxelSize
      ));

      if (occupied.every(k => voxelSet.has(k) && !checked.has(k))) {
        allCandidates.push({
          name: patternName,
          base,
          offsets,
          occupied,
          efficiency: cost / occupied.length
        });
      }

      // Also try a rotated version of the pattern (90° clockwise around Z axis)
      const occupiedRot = rotatedOffsets.map(([dx, dy, dz]) => key(
        base[0] + dx * voxelSize,
        base[1] + dy * voxelSize,
        base[2] + dz * voxelSize
      ));

      if (occupiedRot.every(k => voxelSet.has(k) && !checked.has(k))) {
        allCandidates.push({
          name: patternName,
          base,
          offsets: rotatedOffsets,
          occupied: occupiedRot,
          efficiency: cost / occupiedRot.length
        });
      }
    }
  }

  // Sort candidates by cost efficiency
  allCandidates.sort((a, b) => a.efficiency - b.efficiency);

  for (const cand of allCandidates) {
    if (cand.occupied.every(k => !checked.has(k))) {
      const pts = cand.offsets.map(([dx, dy, dz]) =>
        new THREE.Vector3(
          round(cand.base[0] + dx * voxelSize),
          round(cand.base[1] + dy * voxelSize),
          round(cand.base[2] + dz * voxelSize)
        )
      );
      for (const k of cand.occupied) checked.add(k);
      voxelMap.push({ name: cand.name, singles: pts, type: "gadget", centers: [] });
    }
  }

  // Add single voxels for unused points
  for (const v of voxels) {
    const k = key(...v);
    if (!checked.has(k)) {
      voxelMap.push({
        name: null,
        singles: [new THREE.Vector3(...v)],
        type: "single",
        centers: []
      });
    }
  }

  return { voxelMap };
}

export function findBestPatternCover(points, voxelSize = 0.65, patternList = [], passes = 10) {
  const key = (x, y, z) => `${Math.round(x * 1000) / 1000},${Math.round(y * 1000) / 1000},${Math.round(z * 1000) / 1000}`;

  function shuffle(arr) {
    for (let i = arr.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [arr[i], arr[j]] = [arr[j], arr[i]];
    }
  }

  let bestMap = null;
  let bestCost = Infinity;

  for (let i = 0; i < passes; i++) {
    const shuffledPatterns = [...patternList];
    shuffle(shuffledPatterns);
    const { voxelMap } = findPatternsInOrder(points, voxelSize, shuffledPatterns);

    let cost = 0;
    for (const block of voxelMap) {
      if (block.name) {
        cost += (patternList.includes(block.name) ? block.singles.length : 10);
      } else {
        cost += 10; // cost for single voxels
      }
    }

    if (cost < bestCost) {
      bestCost = cost;
      bestMap = voxelMap;
    }
  }

  return { voxelMap: bestMap };
}


export function generateConstructibleVoxelMap(singleCenters, voxelSize = 0.65, allowedPatterns = ["2x4", "2x4x2", "2x3", "2x3x2", "2x2", "2x2x2"]) {
  // Helper functions
  const round = n => Math.round(n * 1000) / 1000;
  const key = (x, y, z) => `${round(x)},${round(y)},${round(z)}`;

  // Group points by layer (z)
  const byLayer = {};
  for (const pt of singleCenters) {
    const z = round(pt.z);
    if (!byLayer[z]) byLayer[z] = [];
    byLayer[z].push(pt);
  }

  // Sort layers by z ascending
  const layers = Object.keys(byLayer).map(Number).sort((a, b) => a - b);
  const overallChecked = new Set();
  const voxelMap = [];

  // For each layer
  for (const z of layers) {
    let layerPts = byLayer[z].filter(pt => !overallChecked.has(key(pt.x, pt.y, pt.z)));
    if (layerPts.length === 0) continue;
    // Find patterns in this layer
    const { voxelMap: layerMap } = findBestPatternCover(layerPts, voxelSize, allowedPatterns);
    // Mark used
    for (const v of layerMap) {
      for (const pt of v.singles) {
        overallChecked.add(key(pt.x, pt.y, pt.z));
      }
    }

    // Add to global voxelMap
    voxelMap.push(...layerMap);
    // Compute usage ratio
    const used = new Set();
    for (const v of layerMap) {
      for (const pt of v.singles) used.add(key(pt.x, pt.y, pt.z));
    }
    const total = layerPts.length;
    const usedCount = used.size;
    // If not all used, remove neighbors of unused centers
    if (usedCount < total) {
      // Find unused
      const unused = layerPts.filter(pt => !used.has(key(pt.x, pt.y, pt.z)));
      // Remove their neighbors from all subsequent layers (by marking as checked)
      const neighborOffsets = [
        [0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]
      ];
      // for (const pt of unused) {
      //   for (const [dx, dy, dz] of neighborOffsets) {
      //     const nx = round(pt.x + dx * voxelSize);
      //     const ny = round(pt.y + dy * voxelSize);
      //     const nz = round(pt.z + dz * voxelSize);
      //     overallChecked.add(key(nx, ny, nz));
      //   }
      // }
    }
  }
  return { voxelMap };
}


export function findOnly2x2(points, voxelSize = 0.65) {
  return findPatternsInOrder(points, voxelSize, ["2x2"]);
}

export function findOnly2x3(points, voxelSize = 0.65) {
  return findPatternsInOrder(points, voxelSize, ["2x3"]);
}

export function findOnly2x4(points, voxelSize = 0.65) {
  return findPatternsInOrder(points, voxelSize, ["2x4"]);
}


export function findAllPatterns(points, voxelSize = 0.65) {
  const start = performance.now();
  const result = findPatternsInOrder(points, voxelSize, [
    "2x4x2", "2x4", "2x3x2", "2x3", "2x2x2", "2x2"
  ]);
  const end = performance.now();
  console.log(`findAllPatterns took ${(end - start).toFixed(2)} ms`);
  return result;
}
