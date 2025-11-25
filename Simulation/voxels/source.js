import * as THREE from "../three/build/three.module.min.js";
import { Voxel} from "./voxel.js";

export class Source {
    constructor(x, y, z, direction = new THREE.Vector3(0, 1, 0), scene, preview = true) {
        this.scene = scene;
        this.position = new THREE.Vector3(x, y, z);
        this.direction = direction.clone().normalize();
        this.preview = preview;
        this.createdVoxels = [];
        this.group = new THREE.Group();
        this.scene.add(this.group);
        this.group.position.copy(this.position);

        const perp = new THREE.Vector3().crossVectors(this.direction, new THREE.Vector3(0, 0, 1)).normalize();
        const perpNeg = perp.clone().negate();

        // Store voxel positions by type
        this.sourceVoxels = {};
 
        // Base line of 5 voxels under the source center
        for (let i = 0; i <= 2; i++) {
            const basePos = this.position.clone().add(this.direction.clone().multiplyScalar(i));
            this.addVoxel(basePos.x, basePos.y, basePos.z, undefined, undefined, 0x111111, false);
        }

        const zOffset = 0.5;
        const sourceZ = this.position.z + zOffset;

        // top-left
        const pos1 = this.position.clone().add(this.direction.clone().multiplyScalar(2)).add(perp.clone());
        this.addVoxel(pos1.x, pos1.y, sourceZ, 4, perp.clone(), 0xffa1ff, true);
       
        // top-right
        const pos2 = this.position.clone().add(this.direction.clone().multiplyScalar(2)).add(perp.clone().negate());
        this.addVoxel(pos2.x, pos2.y, sourceZ, 4, perpNeg.clone(), 0xffa1ff, true);

        // Bottom
        const pos3 = this.position.clone().add(this.direction.clone().multiplyScalar(3));
        this.addVoxel(pos3.x, pos3.y, sourceZ, 4, this.direction.clone(), 0xffa1ff, true);

        this.sourceVoxels[0] = this.position.clone().multiplyScalar(2);
        this.sourceVoxels[1] = this.position.clone().multiplyScalar(2);
        this.sourceVoxels[2] = this.position.clone().multiplyScalar(2);
        this.sourceVoxels[3] = this.position.clone().multiplyScalar(2);
        this.sourceVoxels[4] = this.position.clone().multiplyScalar(2);
    }

    addVoxel(x, y, z, type, direction, color, sourceVoxel) {
        new Voxel(x, y, z, this.scene, type, this.preview, direction, color,undefined, sourceVoxel, (voxel) => {
            // console.log("Creating voxel with type:", type, "preview:", this.preview);
            if (voxel.mesh) {
                voxel.mesh.userData.source = this;
                voxel.mesh.position.sub(this.position); // shift to local space
                this.group.add(voxel.mesh);
                this.createdVoxels.push(voxel);
            }
        });
    }

    delete() {
        if (this.createdVoxels) {
            this.createdVoxels.forEach(voxel => {
                // Remove mesh from scene
                if (voxel.mesh?.parent) voxel.mesh.parent.remove(voxel.mesh);
                this.scene.remove(voxel.mesh);

                // Remove hitbox from scene
                if (voxel.hitbox?.parent) voxel.hitbox.parent.remove(voxel.hitbox);
                this.scene.remove(voxel.hitbox);
            });

            // Remove line from VoxelMap (take the coordinate of the 7 voxel from this.position to direction)
            if(!this.preview)
            for (let i = 0; i <= 6; i++) {
                const basePos = this.position.clone().add(this.direction.clone().multiplyScalar(i));
                const key = `${basePos.x},${basePos.y},${basePos.z}`;
                if (window.voxelMap) {
                    const mapArray = Array.from(window.voxelMap);
                    for (const voxel of mapArray) {
                        if (voxel.centers?.includes(key)) {
                            window.voxelMap.delete(voxel);
                        }
                    }
                }
            }
        }
        if (this.group?.parent) {
            this.scene.remove(this.group);
        }

        this.createdVoxels = [];
    }
    getPlacementCoordinates(type) {
        let frontLeg, backLeg;
        frontLeg = this.position.clone().add(this.direction.clone().multiplyScalar(1));
        backLeg  = this.position.clone().add(this.direction.clone().multiplyScalar(2));
        // if (type === 0 || type === 1|| type === 5 || type === 6 || type === 9 || type === 4) {
        //     frontLeg = this.position.clone().add(this.direction.clone().multiplyScalar(1));
        //     backLeg  = this.position.clone().add(this.direction.clone().multiplyScalar(2));
        // } else if (type === 2 || type === 3) {
        //     frontLeg = this.position.clone().add(this.direction.clone().multiplyScalar(3));
        //     backLeg  = this.position.clone().add(this.direction.clone().multiplyScalar(4));
        // } else if (type === 4 || type === 7) {
        //     frontLeg = this.position.clone().add(this.direction.clone().multiplyScalar(5));
        //     backLeg  = this.position.clone().add(this.direction.clone().multiplyScalar(6));
        // }
        return {
            frontLeg,
            backLeg
        };
    }
    rotate() {
        // console.log("Rotating source");
        const dx = this.direction.x;
        const dy = this.direction.y;

        this.direction.set(-dy, dx, 0); // Rotate counter-clockwise

        // Rotate the group visually
        if (this.group) {
            this.group.rotation.z = this.getRotationAngle();
        }
        // console.log(`Voxel rotated! New direction: (${this.direction.x}, ${this.direction.y}, ${this.direction.z})`);
    }
    getRotationAngle(){
        let angle = 0;
        if (this.direction.x === 0 && this.direction.y === 1) angle = 0;
        else if (this.direction.x === -1 && this.direction.y === 0) angle = Math.PI/2;
        else if (this.direction.x === 0 && this.direction.y === -1) angle = Math.PI ;
        else if (this.direction.x === 1 && this.direction.y === 0) angle = -Math.PI/2;

        return angle;
    }
}