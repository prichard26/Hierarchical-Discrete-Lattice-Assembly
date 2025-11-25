import * as THREE from './three/build/three.module.min.js';

const nameToType = {
  "2x2": 0,
  "2x3": 1,
  "2by2Offset": 2,
  "2by2FullOffset": 3,
  "2x4": 4,
  "2by3by2": 5,
  "2by4by2": 6,
  "2by3_offset_left": 7,
  "2by3_offset_right": 8,
  "2by2by2": 9,
};

const nameAliases = [
  { pattern: /^2x2$/i, type: 0 },
  { pattern: /^2x3$/i, type: 1 },
  { pattern: /^2x4$/i, type: 4 },
  { pattern: /^2x2x2$/i, type: 9 },
  { pattern: /^2x3x2$/i, type: 5 },
  { pattern: /^2x4x2$/i, type: 6 },
];


function getVoxelType(name) {
  if (!name) return null;

  // Direct match
  if (nameToType[name] !== undefined) return nameToType[name];

  // Try aliases
  for (const alias of nameAliases) {
    if (alias.pattern.test(name)) return alias.type;
  }

  // Default fallback
  return 0;
}
export function translate(voxelMap) {

  const allPoints = [];

  // First pass to collect all points as Vector3
  const blocks = voxelMap
    .filter(entry => entry.name !== null)
    .map(entry => {

      const type = getVoxelType(entry.name);
      // Compute centers per pattern
      const centers = [];
      // console.log(`Processing entry: ${entry.name} (type: ${type})`);
      if (entry.name === "2x2" || entry.name === "2x2x2") {
         if (entry.singles.length === 8) {
           // stacked 2x2x2 (top layer: first 4, bottom layer: last 4)
           const top = [entry.singles[0], entry.singles[1], entry.singles[2], entry.singles[3]];
           const bottom = [entry.singles[4], entry.singles[5], entry.singles[6], entry.singles[7]];
           const pattern = [
             [0, 0], [1, 0],
             [0, 1], [1, 1]
           ];
           // For 2x2 blocks, only one block per layer (the whole 4 points)
           for (const block of [top, bottom]) {
             const validBlock = block.filter(v => v);
             const avg = validBlock.reduce(
               (sum, vec) => sum.add(vec instanceof THREE.Vector3 ? vec.clone() : new THREE.Vector3(vec.x, vec.y, vec.z)),
               new THREE.Vector3(0, 0, 0)
             );
             avg.divideScalar(validBlock.length);
             centers.push(avg);
           }
         } else {
           // Single center from 4 points
           const validSingles = entry.singles.filter(v => v);
           const avg = validSingles.reduce(
             (sum, vec) => sum.add(vec instanceof THREE.Vector3 ? vec.clone() : new THREE.Vector3(vec.x, vec.y, vec.z)),
             new THREE.Vector3(0, 0, 0)
           );
           avg.divideScalar(validSingles.length);
           centers.push(avg);
         }

      } else if (entry.name === "2x3" || entry.name === "2x3x2") {
        if (entry.singles.length > 6) {
          // stacked 2x3x2
          const top = entry.singles.slice(0, 6);
          const bottom = entry.singles.slice(6, 12);
          const pattern = [
            [0, 0], [1, 0], [2, 0],
            [0, 1], [1, 1], [2, 1]
          ];
          // The 2x3 block can be split into two 2x2 blocks per layer:
          // top blocks: [0,1,3,4], [1,2,4,5]
          // bottom blocks: [0,1,3,4], [1,2,4,5]
          const topBlocks = [
            [top[0], top[1], top[3], top[4]],
            [top[1], top[2], top[4], top[5]]
          ];

          const bottomBlocks = [
            [bottom[0], bottom[1], bottom[3], bottom[4]],
            [bottom[1], bottom[2], bottom[4], bottom[5]]
          ];
          for (const block of [...topBlocks, ...bottomBlocks]) {
            const validBlock = block.filter(v => v);
            const avg = validBlock.reduce(
              (sum, vec) => sum.add(vec instanceof THREE.Vector3 ? vec.clone() : new THREE.Vector3(vec.x, vec.y, vec.z)),
              new THREE.Vector3(0, 0, 0)
            );
            avg.divideScalar(validBlock.length);
            centers.push(avg);
          }
        } else {
          // Two 2x2 blocks overlapping vertically
          const block1 = [entry.singles[0], entry.singles[3], entry.singles[1], entry.singles[4]];
          const block2 = [entry.singles[1], entry.singles[4], entry.singles[2], entry.singles[5]];
          for (const block of [block1, block2]) {
            const validBlock = block.filter(v => v);
            const avg = validBlock.reduce(
              (sum, vec) => sum.add(vec instanceof THREE.Vector3 ? vec.clone() : new THREE.Vector3(vec.x, vec.y, vec.z)),
              new THREE.Vector3(0, 0, 0)
            );
            avg.divideScalar(validBlock.length);
            centers.push(avg);
          }
        }

      } else if (entry.name === "2x4" || entry.name === "2x4x2") {
        if (entry.singles.length > 8) {
          // stacked 2x4x2
          const top = entry.singles.slice(0, 8);
          const bottom = entry.singles.slice(8, 16);
          const pattern = [
            [0, 0], [1, 0], [2, 0], [3, 0],
            [0, 1], [1, 1], [2, 1], [3, 1]
          ];
          // Split into four 2x2 blocks per layer:
          // blocks indices: [0,1,4,5], [2,3,6,7]
          // but since it's 2x4, split into two 2x2 blocks per layer:
          const topBlocks = [
            [top[0], top[1], top[4], top[5]],
            [top[2], top[3], top[6], top[7]]
          ];
          const bottomBlocks = [
            [bottom[0], bottom[1], bottom[4], bottom[5]],
            [bottom[2], bottom[3], bottom[6], bottom[7]]
          ];
          for (const block of [...topBlocks, ...bottomBlocks]) {
            const validBlock = block.filter(v => v);
            const avg = validBlock.reduce(
              (sum, vec) => sum.add(vec instanceof THREE.Vector3 ? vec.clone() : new THREE.Vector3(vec.x, vec.y, vec.z)),
              new THREE.Vector3(0, 0, 0)
            );
            avg.divideScalar(validBlock.length);
            centers.push(avg);
          }
        } else {
          // Split into two 2x2 blocks, average each to get 2 centers
          const block1 = [entry.singles[0], entry.singles[1], entry.singles[4], entry.singles[5]];
          const block2 = [entry.singles[2], entry.singles[3], entry.singles[6], entry.singles[7]];
          for (const block of [block1, block2]) {
            const validBlock = block.filter(v => v);
            const avg = validBlock.reduce(
              (sum, vec) => sum.add(vec instanceof THREE.Vector3 ? vec.clone() : new THREE.Vector3(vec.x, vec.y, vec.z)),
              new THREE.Vector3(0, 0, 0)
            );
            avg.divideScalar(validBlock.length);
            centers.push(avg);
          }
        }
      } else {
        // Fallback: average all
        const validSingles = entry.singles.filter(v => v);
        const avg = validSingles.reduce(
          (sum, vec) => sum.add(vec instanceof THREE.Vector3 ? vec.clone() : new THREE.Vector3(vec.x, vec.y, vec.z)),
          new THREE.Vector3(0, 0, 0)
        );
        avg.divideScalar(validSingles.length);
        centers.push(avg);
      }

      centers.forEach(c => allPoints.push(c));

      return { type, centers };
    });

  // Find global min x and y
  const minX = Math.min(...allPoints.map(pt => pt.x));
  const minY = Math.min(...allPoints.map(pt => pt.y));
  const minZ = Math.min(...allPoints.map(pt => pt.z));
  // Final pass to normalize and convert to string format
  return blocks.map(block => {
    const remappedCenters = block.centers.map(vec => {
      const remapped = new THREE.Vector3(
        (vec.x - minX) / (window.voxelSize * 2),
        (vec.y - minY) / (window.voxelSize * 2),
        (vec.z - minZ) / (window.voxelSize * 2)
      );
      return `${remapped.x},${remapped.y},${remapped.z}`;
    });

    return {
      type: block.type,
      centers: remappedCenters
    };
  });
}