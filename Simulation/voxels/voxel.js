import * as THREE from "../three/build/three.module.min.js";
import { STLLoader } from "../three/examples/jsm/loaders/STLLoader.js";

const VOXEL_SIZE = 1300;

export class Voxel {
    constructor(x, y, z, scene, type = 0, isPreview = false, direction = new THREE.Vector3(0,1,0), colorInput = null, toBuild = false, sourceVoxel=false, onReady = null) {

        this.scene = scene;
        this.toBuild = toBuild;
        let path;
        if (type === 0) path = "./voxels/voxelCAD/stl/2by2.stl";
        if (type === 1) path = "./voxels/voxelCAD/stl/2by3.stl";
        if (type === 2) path = "./voxels/voxelCAD/stl/2by2Offset.stl";
        if (type === 3) path = "./voxels/voxelCAD/stl/2by2FullOffset.stl";
        if (type === 4) path = "./voxels/voxelCAD/stl/2by4.stl";
        if (type === 5) path = "./voxels/voxelCAD/stl/2by3by2.stl";
        if (type === 6) path = "./voxels/voxelCAD/stl/2by4by2.stl";
        if (type === 7) path = "./voxels/voxelCAD/stl/2by3_offset_left.stl";
        if (type === 8) path = "./voxels/voxelCAD/stl/2by3_offset_right.stl";
        if (type === 9) path = "./voxels/voxelCAD/stl/2by2by2.stl";

        this.path = path;

        this.position = new THREE.Vector3(x, y, z);
        this.direction = direction ;
        this.isPreview = isPreview;
        this.type = type;

        this.sourceVoxel = sourceVoxel;
        
        this.onReady = onReady;
        
        if (!window.voxelMap) window.voxelMap = new Map();

        this.loader = new STLLoader();
        this.loadVoxel(colorInput);
    }

    loadVoxel(colorInput) {
        this.loader.load(
            this.path, 
            (geometry) => this.onVoxelLoaded(geometry,colorInput), 
            undefined, 
            (error) => console.error("Error loading voxel:", error)
        );
    }
    
    onVoxelLoaded(geometry,colorInput) {
        const actualColor = this.isPreview ? (colorInput ?? 0x00ff00) : (colorInput ?? 0x555555);
        let material = new THREE.MeshStandardMaterial({ 
            color: actualColor,
            metalness: 0.8, 
            roughness: 0.6, 
            transparent: this.isPreview, 
            opacity: this.isPreview ? 0.2 : 1.0 
        });

        this.mesh = new THREE.Mesh(geometry, material);
        let bbox = new THREE.Box3().setFromObject(this.mesh);
        let size = new THREE.Vector3();
        bbox.getSize(size);

        const scaleFactor = new THREE.Vector3(1 / VOXEL_SIZE, 1 / VOXEL_SIZE, 1 / VOXEL_SIZE);
        this.mesh.scale.set(scaleFactor.x, scaleFactor.y, scaleFactor.z);

        bbox = new THREE.Box3().setFromObject(this.mesh);
        let bboxCenter = new THREE.Vector3();
        bbox.getCenter(bboxCenter);

        if (this.type === 0) {          //  2by2
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0, 0, 0.125));
        } else if (this.type === 1) {   //  2by3
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0, 0.25, 0.125));
        } else if (this.type === 2) {   //  single offset
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0, 0.25, -0.125));
        } else if (this.type === 3) {   //  offest diag
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0.25, 0.250, -0.125));
        } else if (this.type === 4) {   //  4by2
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0, 0.5, 0.125));
        } else if (this.type === 5) {   //  3by2by2
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0, 0.25, 0.375));
        } else if (this.type === 6) {   //  4by2by2
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0, 0.5, 0.375));
        } else if (this.type === 7) {    //  3by2 offset right
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(-0.25, 0.5, -0.125));
        } else if (this.type === 8) {    //  3by2 offset left
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0.25, 0.5, -0.125));
        } else if (this.type === 9) {   //  single offset
            this.mesh.position.copy(this.position).sub(bboxCenter).add(new THREE.Vector3(0, 0, 0.375));
        }
        this.mesh.castShadow = true;
        this.mesh.receiveShadow = true;

        if ((!this.isPreview || this.toBuild) && !this.sourceVoxel) { // should allow to have hitbox on everything but preview
            this.createHitbox();
        }

        this.scene.add(this.mesh);
        if (typeof this.onReady === "function") {
            this.onReady(this);
        }
        let angle = this.getRotationAngle();
        
        this.mesh.rotation.z = angle;

        if (!this.isPreview && !this.sourceVoxel) {
            if (!window.voxelMap || !(window.voxelMap instanceof Set)) {
                window.voxelMap = new Set();
            }
            if (!window.voxelCenters || !(window.voxelCenters instanceof Set)) {
                window.voxelCenters = new Set();
            }
            if (this.type === 0) {
                const center = `${this.position.x},${this.position.y},${this.position.z}`;
                const voxelData = { type: 0, centers: [center] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center);
            } else if (this.type === 1) { 
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 0.5 * this.direction.x},${this.position.y + 0.5 * this.direction.y},${this.position.z}`;
                const voxelData = { type: 1, centers: [center1, center2] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
            } else if (this.type === 2) {
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 0.5 * this.direction.x},${this.position.y + 0.5 * this.direction.y},${this.position.z-0.5}`;
                const voxelData = { type: 2, centers: [center1, center2] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
            }else if (this.type === 3) {
                let vert = new THREE.Vector3(0, 0, 1);
                let norm = new THREE.Vector3().copy(this.direction).cross(vert);
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 0.5 * (this.direction.x + norm.x)},${this.position.y + 0.5 * (this.direction.y + norm.y)},${this.position.z-0.5}`;
                const voxelData = { type: 3, centers: [center1, center2] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
            } else if ( this.type === 4 ){
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 1 * this.direction.x},${this.position.y + 1 * this.direction.y},${this.position.z}`;
                const voxelData = { type: 4, centers: [center1, center2] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
            } else if (this.type === 5) { 
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 0.5 * this.direction.x},${this.position.y + 0.5 * this.direction.y},${this.position.z}`;
                const [x1, y1, z1] = center1.split(',').map(Number);
                const [x2, y2, z2] = center2.split(',').map(Number);

                const center3 = `${x1},${y1},${z1 + 0.5}`;
                const center4 = `${x2},${y2},${z2 + 0.5}`;
                const voxelData = { type: 5, centers: [center1, center2, center3, center4] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
                window.voxelCenters.add(center3);
                window.voxelCenters.add(center4);
            }  else if ( this.type === 6 ){
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 1 * this.direction.x},${this.position.y + 1 * this.direction.y},${this.position.z}`;
                const [x1, y1, z1] = center1.split(',').map(Number);
                const [x2, y2, z2] = center2.split(',').map(Number);

                const center3 = `${x1},${y1},${z1 + 0.5}`;
                const center4 = `${x2},${y2},${z2 + 0.5}`;
                const voxelData = { type: 6, centers: [center1, center2, center3, center4] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
                window.voxelCenters.add(center3);
                window.voxelCenters.add(center4);

            }else if (this.type === 7) {
                let vert = new THREE.Vector3(0, 0, 1);
                let norm = new THREE.Vector3().copy(this.direction).cross(vert);
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 0.5 * this.direction.x},${this.position.y + 0.5 * this.direction.y},${this.position.z}`;
                const center3 = `${this.position.x + 1 * this.direction.x - 0.5 * norm.x},${this.position.y + 1 * this.direction.y - 0.5 * norm.y},${this.position.z - 0.5}`;
                const voxelData = { type: 6, centers: [center1, center2, center3] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
                window.voxelCenters.add(center3);
            } else if (this.type === 8) {
                let vert = new THREE.Vector3(0, 0, 1);
                let norm = new THREE.Vector3().copy(this.direction).cross(vert);
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x + 0.5 * this.direction.x},${this.position.y + 0.5 * this.direction.y},${this.position.z}`;
                const center3 = `${this.position.x + 1 * this.direction.x + 0.5 * norm.x},${this.position.y + 1 * this.direction.y + 0.5 * norm.y},${this.position.z - 0.5}`;
                const voxelData = { type: 6, centers: [center1, center2, center3] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
                window.voxelCenters.add(center3);
            } else if (this.type === 9) {
                const center1 = `${this.position.x},${this.position.y},${this.position.z}`;
                const center2 = `${this.position.x},${this.position.y},${this.position.z+0.5}`;
                const voxelData = { type: 9, centers: [center1, center2] };
                window.voxelMap.add(voxelData);
                window.voxelCenters.add(center1);
                window.voxelCenters.add(center2);
            }
        }
    }

    createHitbox() {
        const boxMaterial = new THREE.MeshBasicMaterial({
            color: 0xff0000,
            transparent: true,
            opacity: 0.0,
            depthWrite: false,  
        });      
        const boxSize = new THREE.Vector3(1, 1, 0.5);
        const geometry = new THREE.BoxGeometry(boxSize.x, boxSize.y, boxSize.z);
        const angle = this.getRotationAngle();

        const createAndPlaceHitbox = (positionVec) => {
            const box = new THREE.Mesh(geometry, boxMaterial);
            box.position.copy(positionVec);
            box.rotation.z = angle;
            box.name = "voxel_hitbox";
            this.scene.add(box);
            return box;
        };

        if (this.type === 0) {
            this.hitbox = createAndPlaceHitbox(this.mesh.position.clone());
        } else if (this.type === 1) {
            const center1 = this.position.clone();
            const center2 = center1.clone().add(new THREE.Vector3(
                0.5 * this.direction.x,
                0.5 * this.direction.y,
                0
            ));
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2)
            ];
        } else if (this.type === 2) {
            const center1 = this.position.clone();
            const center2 = center1.clone().add(new THREE.Vector3(
                0.5 * this.direction.x,
                0.5 * this.direction.y,
                -0.5
            ));
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2)
            ];
        } else if (this.type === 3) {
            let vert = new THREE.Vector3(0, 0, 1);
            let norm = new THREE.Vector3().copy(this.direction).cross(vert);
            const center1 = this.position.clone();
            const center2 = center1.clone().add(new THREE.Vector3(
                0.5 * (this.direction.x + norm.x),
                0.5 * (this.direction.y + norm.y),
                -0.5
            ));
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2)
            ];
        } else if (this.type === 4) {
            const center1 = this.position.clone();
            const center2 = center1.clone().add(new THREE.Vector3(
                1 * this.direction.x,
                1 * this.direction.y,
                0
            ));
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2)
            ];
        } else if (this.type === 5) {
            const center1 = this.position.clone();
            const center2 = center1.clone().add(new THREE.Vector3(
                0.5 * this.direction.x,
                0.5 * this.direction.y,
                0
            ));
            const center3 = center1.clone().add(new THREE.Vector3(
                0 ,
                0,
                0.5 
            ));
            const center4 = center2.clone().add(new THREE.Vector3(
                0 ,
                0,
                0.5
            ));
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2),
                createAndPlaceHitbox(center3),
                createAndPlaceHitbox(center4)
            ];
        } else if (this.type === 6) {
            const center1 = this.position.clone();
            const center2 = center1.clone().add(new THREE.Vector3(
                1 * this.direction.x,
                1 * this.direction.y,
                0
            ));
            const center3 = center1.clone().add(new THREE.Vector3(
                0 ,
                0,
                0.5 
            ));
            const center4 = center2.clone().add(new THREE.Vector3(
                0 ,
                0,
                0.5 
            ));
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2),
                createAndPlaceHitbox(center3),
                createAndPlaceHitbox(center4)
            ];
        } else if (this.type === 7) {
            // 2by3_offset_left: 3 hitboxes, one below (to left), two above (aligned with direction)
            let vert = new THREE.Vector3(0, 0, 1);
            let norm = new THREE.Vector3().copy(this.direction).cross(vert);
            // hitbox 1: at this.position
            const center1 = this.position.clone();
            // hitbox 2: above, in the direction
            const center2 = this.position.clone().add(new THREE.Vector3(
                0.5 * this.direction.x,
                0.5 * this.direction.y,
                0
            ));
            // hitbox 3: below, offset to the right (positive norm), z -0.5
            let center3 = this.position.clone().add(new THREE.Vector3(
                1 * this.direction.x - 0.5 * norm.x,
                1 * this.direction.y - 0.5 * norm.y,
                -0.5
            ));
           
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2),
                createAndPlaceHitbox(center3)
            ];
        } else if (this.type === 8) {
            // 2by3_offset_right: same as type 6, but below voxel is to the right (positive norm)
            let vert = new THREE.Vector3(0, 0, 1);
            let norm = new THREE.Vector3().copy(this.direction).cross(vert);
            // hitbox 1: at this.position
            const center1 = this.position.clone();
            // hitbox 2: above, in the direction
            const center2 = this.position.clone().add(new THREE.Vector3(
                0.5 * this.direction.x,
                0.5 * this.direction.y,
                0
            ));
            // hitbox 3: below, offset to the right (positive norm), z -0.5
            let center3 = this.position.clone().add(new THREE.Vector3(
                1 * this.direction.x + 0.5 * norm.x,
                1 * this.direction.y + 0.5 * norm.y,
                -0.5
            ));

            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2),
                createAndPlaceHitbox(center3)
            ];
        } else if (this.type === 9) {
            const center1 = this.position.clone();
            const center2 = center1.clone().add(new THREE.Vector3(
                0,
                0,
                0.5
            ));
            this.hitbox = [
                createAndPlaceHitbox(center1),
                createAndPlaceHitbox(center2)
            ];
        }
        
        else {
            this.hitbox = null;
        }
    }

    rotate() {
        if (!this.mesh) return;
    
        const dx = this.direction.x;
        const dy = this.direction.y;
    
        this.direction.set(-dy, dx, 0); // Rotate counter-clockwise
    
        let angle = this.getRotationAngle();
        this.mesh.rotation.z = angle;
    
        console.log(`Voxel rotated! New direction: (${this.direction.x}, ${this.direction.y}, ${this.direction.z})`);
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

    