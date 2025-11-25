function performance(geometry, singles, voxels){
    // Get algorithm performances
    // Input: 
    //    - geometry: stl mesh 
    //    - singles: centers points
    //    - voxels: voxel( 
    //                     single: [all single centers used]     
    //                     type : voxel Type
    //                     centers: converted centers that matches rest of the pipeline          
    //                      )
    //  Output:
    //    - connectivity percentage,
    //    - nb of center used / total nb of center  (fidelity)
    //    - nb of voxel used,
    //    - nb of voxel used / volume of mesh
    //    - nb of voxel placed that would fall 

    const voxelSize = 65; // Adjust if needed
    const voxelVolume = Math.pow(voxelSize, 3);

    const usedKeys = new Set();
    let connected = 0;
    let unsupported = 0;

    // 1. Collect all used voxel positions
    for (const v of voxels) {
        for (const pt of v.singles) {
            const k = `${pt.x},${pt.y},${pt.z}`;
            usedKeys.add(k);
        }
    }

    // 2. Build neighbor offset table (6-neighborhood)
    const neighbors = [
        [voxelSize, 0, 0], [-voxelSize, 0, 0],
        [0, voxelSize, 0], [0, -voxelSize, 0],
        [0, 0, voxelSize], [0, 0, -voxelSize]
    ];

    // 3. Check connectivity and unsupported voxels
    for (const k of usedKeys) {
        const [x, y, z] = k.split(',').map(Number);
        let hasNeighbor = false;
        let hasSupportBelow = false;

        for (const [dx, dy, dz] of neighbors) {
            const neighborKey = `${x + dx},${y + dy},${z + dz}`;
            if (usedKeys.has(neighborKey)) {
                hasNeighbor = true;
                if (dz < 0) hasSupportBelow = true;
            }
        }

        if (hasNeighbor) connected++;
        if (!hasSupportBelow) unsupported++;
    }

    // 4. Fidelity
    const fidelity = usedKeys.size / singles.length;

    // 5. Volume
    const usedVolume = usedKeys.size * voxelVolume;
    geometry.computeBoundingBox();
    const bbox = geometry.boundingBox;
    const meshVolume = (bbox.max.x - bbox.min.x) * (bbox.max.y - bbox.min.y) * (bbox.max.z - bbox.min.z);

    const coverageRatio = usedVolume / meshVolume;

    // Final result
    return {
        connectivityPercent: (connected / usedKeys.size) * 100,
        fidelity: fidelity,
        voxelCount: usedKeys.size,
        volumeCoverage: coverageRatio,
        unsupportedVoxels: unsupported
    };
}