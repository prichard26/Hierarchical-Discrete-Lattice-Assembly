// ============+ OLD CODE



export function findPattern1(points, voxelSize = 0.65) {
    // search for specific patterns/ combination of single voxels 

    // Function to generate a unique key for a voxel coordinate (truncate like Rhino int())
    const key = (x, y, z) => `${Math.floor(x*100)/100},${Math.floor(y*100)/100},${Math.floor(z*100)/100}`;

    // Preprocess: convert all points to grid and store as set
    const checked = new Set();
    const voxelMap = [];

    // Use grid-aligned rounding for set membership
    const voxelSet = new Set(points.map(pt => {
        const x = pt.x;
        const y = pt.y;
        const z =  pt.z;
        return key(x, y, z);
    }));
    console.log(voxelSet)
    // Pattern existence check
    function patternExists(base, offsets) {
        return offsets.every(([dx, dy, dz]) => {
            const k = key(base[0] + dx, base[1] + dy, base[2] + dz);
            return voxelSet.has(k) && !checked.has(k);
        });
    }

    // Mark pattern voxels as used
    function markUsed(base, offsets) {
        for (const [dx, dy, dz] of offsets) {
            checked.add(key(base[0] + dx, base[1] + dy, base[2] + dz));
        }
    }

    // Make THREE.Vector3 points for a pattern, rounding to 3 decimals for precision consistency
    function makePts(base, offsets) {
        return offsets.map(([dx, dy, dz]) =>
            new THREE.Vector3(
                base[0] + dx,
                base[1] + dy,
                base[2] + dz
            )
        );
    }

    // Search for a gadget pattern
    function searchGadget(offsets, reverseZ = false, patternName = null) {
        // Get all voxel coordinates as arrays
        const voxels = Array.from(voxelSet).map(k => k.split(',').map(Number));
        voxels.sort((a, b) =>
            reverseZ ? b[2] - a[2] || b[1] - a[1] || b[0] - a[0]
                     : a[2] - b[2] || a[1] - b[1] || a[0] - b[0]
        );
        for (const v of voxels) {
            if (checked.has(key(...v))) continue;
            if (patternExists(v, offsets)) {
                const pts = makePts(v, offsets);
                voxelMap.push({ name: patternName, singles: pts, type: "gadget", centers: [] });
                markUsed(v, offsets);
            }
        }
    }

    // Define patterns (offsets in each direction)
    const vs = voxelSize;
    const block2x2 = [
        [0, 0, 0], [vs, 0, 0],
        [0, vs, 0], [vs, vs, 0]
    ];

    const zigzag_xy_rev = [
        ...block2x2,
        [-vs, -vs, vs], [0, -vs, vs],
        [-vs, 0, vs], [0, 0, vs]
    ];
    const zigzag_xy_fwd = [
        ...block2x2,
        [vs, vs, vs], [2 * vs, vs, vs],
        [vs, 2 * vs, vs], [2 * vs, 2 * vs, vs]
    ];
    const zigzag_x_rev = [
        ...block2x2,
        [-vs, 0, vs], [0, 0, vs],
        [-vs, vs, vs], [0, vs, vs]
    ];
    const zigzag_x_fwd = [
        ...block2x2,
        [vs, 0, vs], [2 * vs, 0, vs],
        [vs, vs, vs], [2 * vs, vs, vs]
    ];
    const zigzag_y_rev = [
        ...block2x2,
        [0, -vs, vs], [vs, -vs, vs],
        [0, 0, vs], [vs, 0, vs]
    ];
    const zigzag_y_fwd = [
        ...block2x2,
        [0, vs, vs], [vs, vs, vs],
        [0, 2 * vs, vs], [vs, 2 * vs, vs]
    ];

    // Search for gadgets
    for (const [offsets, name] of [[zigzag_xy_rev, "zigzag_xy_rev"],[zigzag_xy_fwd, "zigzag_xy_fwd"]]) {
        searchGadget(offsets, false, name);
        searchGadget(offsets, true, name);
    }
    for (const [offsets, name] of [[zigzag_x_fwd, "zigzag_x_fwd"], [zigzag_x_rev, "zigzag_x_rev"], [zigzag_y_fwd, "zigzag_y_fwd"], [zigzag_y_rev, "zigzag_y_rev"]]) {
        searchGadget(offsets, false, name);
        searchGadget(offsets, true, name);
    }

    // Now search for 2x2 "modules"
    const flatVoxels = Array.from(voxelSet).map(k => k.split(',').map(Number));
    flatVoxels.sort((a, b) => a[2] - b[2] || a[1] - b[1] || a[0] - b[0]);

    for (const v of flatVoxels) {
        const k = key(...v);
        if (checked.has(k)) continue;
        if (patternExists(v, block2x2)) {
            const pts = makePts(v, block2x2);
            voxelMap.push({ name: "2x2", singles: [...pts, pts[0]], type: "module", centers: [] });
            markUsed(v, block2x2);
        } else {
            // singles handled later
        }
    }

    // Try to group remaining singles into modules (2x2 in XY)
    const remaining = flatVoxels.filter(v => !checked.has(key(...v)));
    const zLayers = new Set(remaining.map(v => v[2]));

    for (const z of zLayers) {
        const layerPts = remaining.filter(v => v[2] === z).map(v => [v[0], v[1]]);
        const layerSet = new Set(layerPts.map(([x, y]) => `${x},${y}`));
        for (const pt of layerPts) {
            const base_x = pt[0];
            const base_y = pt[1];
            // Offsets for a 2x2 in XY
            const offsets = [[0, 0], [vs, 0], [0, vs], [vs, vs]];
            const candidate = offsets.map(([dx, dy]) => [base_x + dx, base_y + dy, z]);
            const hasUsed = candidate.some(([x, y, z]) => checked.has(key(x, y, z)));
            const present = candidate.filter(([x, y, z]) => voxelSet.has(key(x, y, z)));
            if (!hasUsed && present.length >= 3) {
                const pts = candidate.map(([x, y, z]) => new THREE.Vector3(x, y, z));
                voxelMap.push({ name: "2x2", singles: [...pts, pts[0]], type: "module", centers: [] });
                present.forEach(([x, y, z]) => checked.add(key(x, y, z)));
            } else {
                const [x, y] = pt;
                if (!checked.has(key(x, y, z))) {
                    voxelMap.push({ name: null, singles: [new THREE.Vector3(x, y, z)], type: "single", centers: [] });
                    checked.add(key(x, y, z));
                }
            }
        }
    }
    return { voxelMap };
}

export function findPattern2(points, voxelSize = 0.65) {
  const key = (x, y, z) => `${x},${y},${z}`;
  const voxelSet = new Set();
  const checked = new Set();
  const voxelMap = [];

  for (const pt of points) {
    const x = Math.round(pt.x *1000/ voxelSize)/1000;
    const y = Math.round(pt.y *1000/ voxelSize)/1000;
    const z = Math.round(pt.z *1000/ voxelSize)/1000;
    voxelSet.add(`${x},${y},${z}`);
  }

  function patternExists(base, offsets) {
    return offsets.every(([dx, dy, dz]) =>
      voxelSet.has(key(base[0] + dx, base[1] + dy, base[2] + dz)) &&
      !checked.has(key(base[0] + dx, base[1] + dy, base[2] + dz))
    );
  }

  function markUsed(base, offsets) {
    for (const [dx, dy, dz] of offsets) {
      checked.add(key(base[0] + dx, base[1] + dy, base[2] + dz));
    }
  }

  function makePts(base, offsets) {
    return offsets.map(([dx, dy, dz]) =>
      new THREE.Vector3(
        (base[0] + dx) * voxelSize,
        (base[1] + dy) * voxelSize,
        (base[2] + dz) * voxelSize
      )
    );
  }

  function rotateOffsets(offsets, rot) {
    return offsets.map(([dx, dy, dz]) => {
      switch (rot) {
        case 0: return [dx, dy, dz];
        case 1: return [-dy, dx, dz];
        case 2: return [-dx, -dy, dz];
        case 3: return [dy, -dx, dz];
      }
    });
  }

  function searchPattern(offsets, name) {
    const voxels = Array.from(voxelSet).map(k => k.split(',').map(Number));
    for (const rot of [0, 1, 2, 3]) {
      const rotated = rotateOffsets(offsets, rot);
      for (const v of voxels) {
        if (checked.has(key(...v))) continue;
        if (patternExists(v, rotated)) {
          const pts = makePts(v, rotated);
          voxelMap.push({ name, singles: pts, type: "gadget", centers: [] });
          markUsed(v, rotated);
        }
      }
    }
  }
  

  // searchPattern(pattern_2x2_flat, "2x2");
  // searchPattern(pattern_2x3_flat, "2x3");
  searchPattern(pattern_2x4_2x4, "2x4x2");
  searchPattern(pattern_2x2_flat, "2x2");
  // searchPattern(pattern_2x4_zigzag, "2x4_zigzag");
  // searchPattern(pattern_2x4_2x4, "2x4+2x4");
  // searchPattern(pattern_cat3, "cat3");

  const allVoxels = Array.from(voxelSet).map(k => k.split(',').map(Number));
  for (const v of allVoxels) {
    const k = key(...v);
    if (!checked.has(k)) {
      voxelMap.push({
        name: null,
        singles: [new THREE.Vector3(v[0] * voxelSize, v[1] * voxelSize, v[2] * voxelSize)],
        type: "single",
        centers: []
      });
    }
  }

  return { voxelMap };
}

export function findPattern3(points, voxelSize = 0.65) {
  const key = (x, y, z) => `${Math.floor(x * 100) / 100},${Math.floor(y * 100) / 100},${Math.floor(z * 100) / 100}`;
  const checked = new Set();
  const voxelMap = [];

  const round = (n) => Math.round(n * 1000) / 1000;

  const voxelSet = new Set(points.map(pt => key(pt.x, pt.y, pt.z)));
  const voxels = Array.from(voxelSet).map(k => k.split(',').map(Number));

  function findOrigins(voxels) {
    let minX = Infinity, minY = Infinity, maxX = -Infinity;
    for (const [x, y, z] of voxels) {
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
    }
    return {
      leftBack: [minX, minY],
      rightBack: [maxX, minY]
    };
  }

  const offsetsXY = [[0, 0], [voxelSize, 0], [0, voxelSize], [voxelSize, voxelSize]];
  const offsetsYX = [[0, 0], [0, voxelSize], [-voxelSize, 0], [-voxelSize, voxelSize]];

  const zLayers = new Set(voxels.map(v => v[2]));
  for (const z of zLayers) {
    const layerPts = voxels.filter(v => v[2] === z);
    const { leftBack, rightBack } = findOrigins(layerPts);
    const layerSet = new Set(layerPts.map(([x, y]) => `${x},${y}`));

    for (const [x, y] of layerPts.map(([x, y]) => [x, y])) {
      const isLeft = x === leftBack[0] && y === leftBack[1];
      const isRight = x === rightBack[0] && y === rightBack[1];
      const offsets = isRight ? offsetsYX : offsetsXY;

      const candidate = offsets.map(([dx, dy]) => [x + dx, y + dy, z]);
      const hasUsed = candidate.some(([cx, cy, cz]) => checked.has(key(cx, cy, cz)));
      const present = candidate.filter(([cx, cy, cz]) => voxelSet.has(key(cx, cy, cz)));

      if (!hasUsed && present.length >= 3) {
        const pts = candidate.map(([cx, cy, cz]) => new THREE.Vector3(round(cx), round(cy), round(cz)));
        voxelMap.push({ name: "2x2", singles: [...pts, pts[0]], type: "module", centers: [] });
        present.forEach(([cx, cy, cz]) => checked.add(key(cx, cy, cz)));
      } else {
        const k = key(x, y, z);
        if (!checked.has(k)) {
          voxelMap.push({ name: null, singles: [new THREE.Vector3(round(x), round(y), round(z))], type: "single", centers: [] });
          checked.add(k);
        }
      }
    }
  }

  return { voxelMap };
}