import * as THREE from './three/build/three.module.min.js';


export function computeVoxelizationStats(geometry, voxelCenters, voxelMap = []) {
  const voxelVolume = Math.pow(65, 3);

  geometry.computeBoundingBox();
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  console.log("▶ Calculating mesh volume...");
  const meshVolume = computeMeshVolume(geometry);
  const meshVolumeM3 = meshVolume / 1e9;

  // const typedVoxels = voxelMap.filter(v => v.name && v.name !== "type0");
  // console.log("▶ Typed voxel map:", typedVoxels);
  const typedVoxelCount = voxelMap.filter(v => v.type === 'gadget').length;
  // const totalTypedCenters = typedVoxels.reduce((sum, v) => sum + (v.centers?.length || 0), 0);
  // console.log("▶ Total typed voxel centers:", totalTypedCenters);
  // const typedVoxelizedVolume = totalTypedCenters * voxelVolume;
  const gadgetVoxels = voxelMap.filter(v => v.type === 'gadget');
  const totalGadgetSingles = gadgetVoxels.reduce((sum, v) => sum + (v.singles?.length || 0), 0);
  const typedVoxelizedVolume = totalGadgetSingles * voxelVolume;
  const typedVoxelizedVolumeM3 = typedVoxelizedVolume / 1e9;

  console.log("▶ All voxel centers length:", voxelCenters.length);
  const optimalVoxelizedVolumeM3 = (voxelCenters.length * voxelVolume) / 1e9;

  const precision = meshVolumeM3 > 0 ? typedVoxelizedVolumeM3 / meshVolumeM3   : 0;

  console.log("▶ Final stats:", {
    meshVolumeM3,
    typedVoxelizedVolumeM3,
    optimalVoxelizedVolumeM3,
    precision
  });

  return {
    meshVolume: Number(meshVolumeM3.toFixed(8)),
    voxelVolume,
    voxelCount: voxelCenters.length,
    typedVoxelCount,
    voxelizedVolume: Number(typedVoxelizedVolumeM3.toFixed(8)),
    optimalVoxelizedVolume: Number(optimalVoxelizedVolumeM3.toFixed(8)),
    precision: Number(precision.toFixed(8)),
    optimalprecision: Number((optimalVoxelizedVolumeM3 / meshVolumeM3).toFixed(8))
  };
}

function computeMeshVolume(geometry) {
  const bufferGeometry = geometry.toNonIndexed();  // Ensure non-indexed geometry
  const posAttr = bufferGeometry.getAttribute('position');
  let volume = 0;

  for (let i = 0; i < posAttr.count; i += 3) {
    const p1 = new THREE.Vector3().fromBufferAttribute(posAttr, i);
    const p2 = new THREE.Vector3().fromBufferAttribute(posAttr, i + 1);
    const p3 = new THREE.Vector3().fromBufferAttribute(posAttr, i + 2);
    volume += signedVolumeOfTriangle(p1, p2, p3);
  }

  return Math.abs(volume);
}

function signedVolumeOfTriangle(p1, p2, p3) {
  return p1.dot(p2.clone().cross(p3)) / 6.0;
}
