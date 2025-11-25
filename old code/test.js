const voxelSize = 65;

/**
 * Main Voxelizer function supporting modular voxel processing steps.
 * Currently implements Testuz et al. greedy merging approach.
 * Future integration of Luo et al. optimization method is planned.
 */
export function Voxelizer(voxelCenters, useOptimization = false) {
    // Step 1: Generate voxel layers from input points
    const layers = generateVoxelGrid(voxelCenters);

    // Step 2: Group connected voxels within each layer
    const connectedLayers = layers.map(layer => groupConnectedVoxels(layer));

    // Step 3: Apply pattern merging to each connected group (greedy approach or optimization)
    const mergedVoxelGroups = connectedLayers.map(groups =>
        groups.map(group => applyPatternMerging(group))
    );

    // Step 5: Output final voxel map formatting gadgets and singles
    const voxelMap = outputVoxelMap(mergedVoxelGroups);

    // Debug logs
    console.log('Layers:', layers);
    console.log('Connected Layers:', connectedLayers);
    console.log('Merged Voxel Groups:', mergedVoxelGroups);
    console.log('Voxel Map:', voxelMap);

    return { voxelMap };
}

/**
 * Step 1: Generate voxel layers by grouping voxel centers by their z-coordinate.
 */
function generateVoxelGrid(voxels) {
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

/**
 * Step 2: Group connected voxels within a layer using 2D flood-fill based on proximity.
 */
function groupConnectedVoxels(voxels) {
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

/**
 * Step 3: Apply greedy pattern merging (Testuz et al.) to a connected voxel group.
 * Renamed from globalPatternSearch for modularity.
 */
function applyPatternMerging(group) {
    // Pattern definitions and associated cost
    const patternOffsets = {
        "2x4": [[0, 0], [1, 0], [2, 0], [3, 0], [0, 1], [1, 1], [2, 1], [3, 1]],
        "2x3": [[0, 0], [1, 0], [2, 0], [0, 1], [1, 1], [2, 1]],
        "2x2": [[0, 0], [1, 0], [0, 1], [1, 1]],
    };

    const patternCost = { "2x2": 3, "2x3": 2, "2x4": 1 };

    const compareByOriginThenEfficiency = (a, b) => {
        if (a.origin.y !== b.origin.y) return a.origin.y - b.origin.y;
        if (a.origin.x !== b.origin.x) return a.origin.x - b.origin.x;
        return a.efficiency - b.efficiency;
    };

    const key = (x, y) => `${x},${y}`;

    // Build a map of voxel positions for quick lookup
    const centerMap = new Map();
    group.forEach(pt => {
        centerMap.set(key(pt.x, pt.y), pt);
    });

    // Sort voxel positions from top-left to bottom-right (by y ascending, then x ascending)
    const sortedGroup = group.slice().sort((a, b) => {
        if (a.y !== b.y) return a.y - b.y;
        return a.x - b.x;
    });

    // Try placing each pattern type at every voxel in the sorted group
    const candidates = [];
    for (const patternName in patternOffsets) {
        const offsets = patternOffsets[patternName];
        const cost = patternCost[patternName];
        for (const pt of sortedGroup) {
            const coords = offsets.map(([dx, dy]) => {
                const x = pt.x + dx * voxelSize;
                const y = pt.y + dy * voxelSize;
                return key(x, y);
            });

            // If all required positions for this pattern exist in the group, store as candidate
            if (coords.every(k => centerMap.has(k))) {
                candidates.push({
                    name: patternName,
                    cost,
                    occupied: new Set(coords),
                    singles: coords.map(k => centerMap.get(k)),
                    efficiency: cost / coords.length,
                    origin: pt
                });
            }
        }
    }

    // Sort candidates to prioritize top-left patterns and larger (lower cost) patterns
    candidates.sort(compareByOriginThenEfficiency);

    const usedKeys = new Set();
    const selectedPatterns = [];

    for (const cand of candidates) {
        const overlaps = [...cand.occupied].some(k => usedKeys.has(k));
        // Greedily select non-overlapping pattern matches
        if (!overlaps) {
            selectedPatterns.push(cand);
            cand.occupied.forEach(k => usedKeys.add(k));
        }
    }

    // Compose final voxel map for this group
    const voxelMap = [];

    // Add selected patterns as gadgets
    selectedPatterns.forEach(p => {
        voxelMap.push({
            name: p.name,
            type: "gadget",
            singles: p.singles,
            centers: [...p.singles]
        });
    });

    // Add unmatched voxels as singles
    for (const pt of group) {
        const snappedKey = key(pt.x, pt.y);
        if (!usedKeys.has(snappedKey)) {
            voxelMap.push({
                name: null,
                type: "single",
                singles: [pt],
                centers: []
            });
        }
    }

    console.log("Patterns used:", selectedPatterns.length);

    return voxelMap;
}

/**
 * Step 5: Format and flatten the final voxel map output.
 */
function outputVoxelMap(stableVoxelGroups) {
    // Flatten nested arrays: stableVoxelGroups[layer][group][voxels]
    const voxelMap = [];
    for (const layerGroups of stableVoxelGroups) {
        for (const groupVoxels of layerGroups) {
            voxelMap.push(...groupVoxels);
        }
    }
    return voxelMap;
}