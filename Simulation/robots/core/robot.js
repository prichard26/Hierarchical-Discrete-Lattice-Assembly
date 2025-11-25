import * as THREE from "../../three/build/three.module.min.js";
 
import {goForward, forwardHalf, switchLeg, turnRight, turnLeft, climbUp, climbDown, planTransitionConcave, 
        planTransitionConvex, rotateMovingLeg, calculateMovementVector, displayTrajectory, 
        clearTrajectory, moveLegBezier, interpolateMovement, pickUpLeft,pickUpRight,
        climbTopStair,climbTopStairHalf,climbBottomStair,climbBottomStairHalf,
        columnDown,columnUp,
        F101B101,F10m1B10m1,Fm101Bm101,Fm10m1Bm10m1,
        Fm10m1Bm10m1Half,Fm101Bm101Half,F10m1B10m1Half,F101B101Half,
        F0p5L1B1, F0p5R1B1, F1L1B0p5, F1R1B0p5,
        placing_1l_1_1,
        placing_1l_1_0_5, placing_1l_1_0,
        placing_1r_1_1, placing_1r_1_0_5, placing_1r_1_0,
        placing_1l_2_1, placing_1l_2_0_5, placing_1l_2_0,
        placing_1r_2_1, placing_1r_2_0_5, placing_1r_2_0,
        placing_2l_1, placing_2l_0_5, placing_2l_0,
        placing_2r_1, placing_2r_0_5, placing_2r_0,
        placing_3l_1, placing_3l_0_5, placing_3l_0,
        placing_3r_1, placing_3r_0_5, placing_3r_0,
        placing_3c_1, placing_3c_0_5, placing_3c_0,
        placing_4l_1, placing_4l_0_5, placing_4l_0,
        placing_4r_1, placing_4r_0_5, placing_4r_0,
        placing_r_1r_1_1, placing_r_1r_1_0_5, placing_r_1r_1_0,
        placing_r_1l_1_1, placing_r_1l_1_0_5, placing_r_1l_1_0,
        placing_r_1l_2_1, placing_r_1l_2_0_5, placing_r_1l_2_0,
        placing_r_1r_2_1, placing_r_1r_2_0_5, placing_r_1r_2_0,
        placing_r_2l_1, placing_r_2l_0_5, placing_r_2l_0,
        placing_r_2r_1, placing_r_2r_0_5, placing_r_2r_0,
        placing_r_3l_1, placing_r_3l_0_5, placing_r_3l_0,
        placing_r_3r_1, placing_r_3r_0_5, placing_r_3r_0,
        placing_r_3c_1, placing_r_3c_0_5, placing_r_3c_0,
        placing_r_4l_1, placing_r_4l_0_5, placing_r_4l_0,
        placing_r_4r_1, placing_r_4r_0_5, placing_r_4r_0,
        placing_1l_1_2,placing_1r_1_2,placing_1l_2_2,placing_1r_2_2,placing_2l_2,placing_2r_2,
        placing_3l_2,placing_3r_2,placing_3c_2,placing_4l_2,placing_4r_2,
        placing_r_1l_1_2,placing_r_1r_1_2,placing_r_1l_2_2,placing_r_1r_2_2,placing_r_2l_2,
        placing_r_2r_2,placing_r_3l_2,placing_r_3r_2,placing_r_3c_2,placing_r_4l_2,placing_r_4r_2,
        rotateMovingLegWithZOffset,

        turnRightClimbUp, turnRightClimbDown, turnLeftClimbUp, turnLeftClimbDown,
        turnRightClimbUpHalf, turnRightClimbDownHalf, turnLeftClimbUpHalf, turnLeftClimbDownHalf,
        turnRightClimbUpBack, turnRightClimbDownBack, turnLeftClimbUpBack, turnLeftClimbDownBack,
        turnRightClimbUpHalfBack, turnRightClimbDownHalfBack, turnLeftClimbUpHalfBack, turnLeftClimbDownHalfBack,
        sideStepFlatLeftHalf,sideStepFlatRightHalf,sideStepFlatLeft,sideStepFlatRight,
        climbDownStairFlatOneHalf,climbDownFlatStairOneHalf,climbUpStairFlatOneHalf,climbUpFlatStairOneHalf,forwardTwo,forwardOneHalf,
        climbDownStairFullHalf,climbDownStairHalfFull,climbBottomStairFullHalf,climbBottomStairHalfFull,
        climbUpStairFullHalf,climbUpStairHalfFull,climbTopStairFullHalf,climbTopStairHalfFull,
        climbUpFlatStair,climbUpFlatStairHalf,climbUpStairStair,climbUpStairStairHalf,climbUpStairFlat,climbUpStairFlatHalf,
        climbDownFlatStair,climbDownFlatStairHalf,climbDownStairStair,climbDownStairStairHalf,climbDownStairFlat,climbDownStairFlatHalf,
        sideHalfDownLeft, sideHalfUpLeft, climbDownHalf, climbUpHalf, sideHalfUpRight, sideHalfDownRight,
        sideStepUpRight, sideStepDownRight, sideStepUpLeft, sideStepDownLeft } from "./robot_movement.js";

import {planPathToLegPositions } from './path_planner.js';

const NO_LIGHT_MODE = true;
let sharedJointGeometry = null;
let sharedJointMaterial = null;


const robotWorker = new Worker(new URL("./robot_worker.js", import.meta.url), { type: "module" });
const robotWorkerQueue = new Map();

robotWorker.onmessage = function (e) {
  const { id, result } = e.data;
  if (robotWorkerQueue.has(id)) {
    robotWorkerQueue.get(id)(result);
    robotWorkerQueue.delete(id);
  }
};

const round = n => Math.round(n * 1000) / 1000;
// const DEFAULT_GEO = [[0,0,1.25], [0,0,2], [0,0,2], [0,0,1.25], [0,0,0]]

const DEFAULT_GEO = [[0,0,1.675], [0,0,1.62], [0,0,1.62], [0,0,1.675], [0,0,0]]
// const DEFAULT_GEO = [[0,0,1.25], [0,0,2], [0,0,2], [0,0,1.25], [0,0,0]]

const DEFAULT_LIMITS = [
  [-2 * Math.PI, 2 * Math.PI],
  [2 * Math.PI, 2 * Math.PI],
  [2 * Math.PI, 2 * Math.PI],
  [2 * Math.PI, 2 * Math.PI],
  [2 * Math.PI, 2 * Math.PI]
];

export class THREERobot{
    constructor(origin, target, normal, scene, initialGeometry = DEFAULT_GEO , limits = DEFAULT_LIMITS) {
        this.scene = scene;
        this.angles = [0, 0, 0, 0, 0]; 
        this.joints = [];                           // Contains the angle of each joint
        this.robotBones = [];                       // Contains the Three.js element of the link in the simulation
        this.initialGeometry = initialGeometry;     
        this.limits = limits;                       
        this.leg1 = initialGeometry[1][2];          
        this.leg2 = initialGeometry[2][2];          
        this.offset = initialGeometry[3][2];        // Both first qnd last link
        this.fixed_leg = 0;                         // Can either be 0 or 4 
        this.transitionType = 'None';               // Used for plan transition of the robot
        this.lastTransitionType = 'None';           // Keep track of last transition that is not None
        this.lastPlacementVector = null;            // Keep track of last placement vector to be able to have revert movement
        
        this.actionQueue = [];                      //  Action queue to store movement actions
        this.isExecuting = false;                   //  Track if an action is being executed
        this.movementHistory = [];                  //  Total List of Movement 
        
        this.origin = {                             // Position and normal vector of the target (fixed leg)
            position: origin.clone(), 
            normal: normal.clone().normalize() 
        };

        this.target = {                             // Position and normal vector of the target (moving leg)
            position: target.clone(), 
            normal: normal.clone().normalize() 
        };

        // Direction from origin to target
        this.direction = new THREE.Vector3().subVectors(this.target.position, this.origin.position);

        this.colors = [0x6a0088, 0xffffff, 0xffffff, 0xffffff, 0xffffff, 0xffffff];

        // Arrow slot status and colors for direction arrows
        // All arrows start as empty (black) and isEmpty: true, only track initialColor
        // Direction vectors are fixed and independent of parity
        this.voxelSlots = {
            backward: { mesh: null, isEmpty: true, initialColor: 0x008800, vector: new THREE.Vector3(-1, 0, 0) },
            left:     { mesh: null, isEmpty: true, initialColor: 0x880000, vector: new THREE.Vector3(0, 1, 0) },
            right:    { mesh: null, isEmpty: true, initialColor: 0x000088, vector: new THREE.Vector3(0, -1, 0) }
        };

        this.robotGroup = new THREE.Group();        // Three.js group containing the robot
        this.robotGroup.position.copy(origin);

        this.buildRobot(initialGeometry, limits);   // Build the robot and add it to the group
        this.scene.add(this.robotGroup);            // Add it to the scene
        this.robotGroup.updateMatrixWorld(true);    // Forces a world matrix update

        this.addDirectionArrow();

        let currentEndEffector = new THREE.Vector3();
        this.robotBones[4].getWorldPosition(currentEndEffector);
        this.direction = currentEndEffector.clone().sub(this.origin.position).normalize();

        // Ensure `this.direction` is always valid  
        if (Math.abs(this.direction.dot(normal) - 1) < 1e-6) {  
            if (this.origin.normal.x > 0.9) this.direction.set(0, 0, -1);
            else if (this.origin.normal.x < -0.9) this.direction.set(0, 0, 1);
            else if (this.origin.normal.y > 0.9) this.direction.set(1, 0, 0);
            else if (this.origin.normal.y < -0.9) this.direction.set(1, 0, 0);
            else if (this.origin.normal.z > 0.9) this.direction.set(1, 0, 0);
            else if (this.origin.normal.z < -0.9) this.direction.set(-1, 0, 0);
        }
        this.updateAnglesFromTarget();              // Find and apply the angle needed to reach target

        this.target.position.copy(this.computeEndEffectorPosition());
    }
    addDirectionArrow() {
        const baseJoint = this.fixed_leg === 0 ? this.robotBones[0] : this.robotBones[4];
        if (!baseJoint) return;

        const arrowLength = 1.5;
        const arrowRadius = 0.05;
        const arrowGeometry = new THREE.CylinderGeometry(arrowRadius, arrowRadius, arrowLength, 8);


        Object.entries(this.voxelSlots).forEach(([name, data]) => {
            // All arrows start as black (empty)
            const material = new THREE.MeshBasicMaterial({ color: 0x000000 });
            const arrow = new THREE.Mesh(arrowGeometry, material);
            arrow.name = `arrow_${name}`;

            const axis = new THREE.Vector3(0, 1, 0);
            const quaternion = new THREE.Quaternion().setFromUnitVectors(axis, data.vector.clone().normalize());
            arrow.setRotationFromQuaternion(quaternion);
            arrow.position.copy(data.vector.clone().multiplyScalar(arrowLength / 2));

            baseJoint.add(arrow);
            this.voxelSlots[name].mesh = arrow;
        });
    }

    // initializeSlotColors() {
    //     Object.entries(this.voxelSlots).forEach(([dir, data]) => {
    //         if (data.mesh) {
    //             data.mesh.material.color.setHex(data.initialColor);
    //         }
    //     });
    // }

    /**
     * Mark the slot as "full" (colored), not empty.
     * Now isEmpty: false means slot is full (colored), true means empty (black).
     */
    updateArrowStatus(direction, isNowEmpty = false) {
        Object.entries(this.voxelSlots).forEach(([dir, data]) => {
            if (!data.mesh) return;
            if (data.mesh.name === `arrow_${direction}`) {
                data.isEmpty = isNowEmpty;
                const color = isNowEmpty ? 0x000000 : data.initialColor;
                data.mesh.material.color.setHex(color);
            }
        });
    }
    /**
     * Mark slot as empty (black color).
     */
    markSlotEmpty(direction) {
        const slot = this.voxelSlots[direction];
        if (slot && slot.mesh) {
            slot.isEmpty = true;
            slot.mesh.material.color.setHex(0x000000); // black = empty
        }
    }
    /**
     * Reset all arrows to empty (black) and mark as empty (isEmpty: true).
     */
    resetArrowStatus() {
        Object.entries(this.voxelSlots).forEach(([name, slot]) => {
            if (slot.mesh) {
                slot.mesh.material.color.setHex(0x000000);
                slot.isEmpty = true;
            }
        });
    }
    buildRobot(initialGeometry, limits) {
        let parentObject = this.robotGroup;
        let fixedTargetPosition = this.target.position.clone(); 

        let defaultNormal = new THREE.Vector3(0, 0, 1); 
        let rotationQuaternion = new THREE.Quaternion();
        if (!this.origin.normal.equals(defaultNormal)) {
            rotationQuaternion.setFromUnitVectors(defaultNormal, this.origin.normal);
        }
        this.robotGroup.quaternion.copy(rotationQuaternion);
    
        let x = 0, y = 0, z = 0;
        for (let i = 0; i < initialGeometry.length; i++) {
            let link = initialGeometry[i];
    
            let linkGeo = this.createJointBone(
                x, y, z,
                link[0], link[1], link[2], i
            );
            x = link[0];
            y = link[1];
            z = link[2];
    
            parentObject.add(linkGeo);
            parentObject = linkGeo;
            this.robotBones.push(linkGeo);
        }
        this.robotGroup.updateMatrixWorld(true);
        this.target.position.copy(fixedTargetPosition);
    }

    createJointBone(x, y, z, w, h, d, jointNumber) {
        // Thickening factor to avoid rendering issues
        const thicken = 0.5;
        const w_thickened = Math.abs(w) + thicken;
        const h_thickened = Math.abs(h) + thicken;
        const d_thickened = Math.abs(d) + thicken;

        // Create link
        const fallbackColor = 0xffffff;
        const color = this.colors[jointNumber] ?? fallbackColor;
        const material = NO_LIGHT_MODE
            ? new THREE.MeshBasicMaterial({ color })
            : new THREE.MeshLambertMaterial({ color });
        const geometry = new THREE.BoxGeometry(w_thickened, h_thickened, d_thickened);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(w / 2, h / 2, d / 2);

        const group = new THREE.Object3D();
        group.position.set(x, y, z);
        group.add(mesh);

        // Create joint using shared geometry and material
        if (!sharedJointGeometry) {
            sharedJointGeometry = new THREE.CylinderGeometry(0.4, 0.4, 0.8, 16, 16);
        }
        if (!sharedJointMaterial) {
            sharedJointMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 });
        }

        const jointMesh = new THREE.Mesh(sharedJointGeometry, sharedJointMaterial);
        if (jointNumber === 0 || jointNumber === 4) {
            jointMesh.rotation.x = Math.PI / 2;
        }
        const joint = new THREE.Group();
        joint.name = `robot_joint`;
        joint.add(jointMesh);
        this.joints.push(joint);

        group.add(joint);
        return group;
    }

    updateGeometry(newGeo) {
        let currentAngles = [...this.angles];

        this.scene.remove(this.robotGroup);
        this.robotBones = [];
        this.joints = [];
    
        // Create a new robot group
        this.robotGroup = new THREE.Group();
        this.buildRobot(newGeo);
        this.scene.add(this.robotGroup);
    
        // Restore previous angles
        for (let i = 0; i < currentAngles.length; i++) {
            this.setAngle(i, currentAngles[i]);
        }
    }
    
    setAngles(angles1) {
        this.angles = angles1;
        this.robotBones[0].rotation.z = this.angles[0];
        this.robotBones[1].rotation.y = this.angles[1];
        this.robotBones[2].rotation.y = this.angles[2];
        this.robotBones[3].rotation.y = this.angles[3];
        this.robotBones[4].rotation.z = this.angles[4];
    }
    
    setAngle(index, angle) {
        this.angles[index] = angle;
        this.setAngles(this.angles);
    }
    
    setToTargetIK(px, py, pz, nx,ny,nz, transitionType = null) {
        this.target.position.set(round(px), round(py), round(pz));
        this.target.normal.set(nx,ny,nz);
        this.transitionType = transitionType;
        this.updateAnglesFromTarget();
    }
    
    updateAnglesFromTarget() {
        let normal = this.origin.normal.clone().normalize();
    
        // Project the Target onto the Plane Defined by `origin.normal`
        let diff = this.target.position.clone().sub(this.origin.position);
        let distanceToPlane = diff.dot(normal);
        let projectedTarget = this.target.position.clone().sub(normal.clone().multiplyScalar(distanceToPlane));
        projectedTarget.set(round(projectedTarget.x), round(projectedTarget.y), round(projectedTarget.z));

        // Compute Rotation Angle θ0 (Yaw Rotation)
        let crossProduct = new THREE.Vector3().crossVectors(this.direction, projectedTarget.clone().sub(this.origin.position).normalize()).dot(normal);
        let dotProduct = this.direction.dot(projectedTarget.clone().sub(this.origin.position).normalize());
        let theta0 = Math.atan2(crossProduct, dotProduct);

        // Compute Rotation Angle θ0 (Yaw Rotation)
        let aAxis = projectedTarget.clone().sub(this.origin.position).normalize();
        let bAxis = normal.clone();
    
        this.setAngle(0, theta0);

        // Convert Target Position to Local Coordinates (A-B Plane)
        let targetPoint = new THREE.Vector2(
            round(this.target.position.clone().sub(this.origin.position).dot(aAxis)),
            round(this.target.position.clone().sub(this.origin.position).dot(bAxis))
        );
        
        // Compute End-Effector Angle (Corrected)
        let normalA = this.origin.normal.clone().normalize();
        let normalB = this.target.normal.clone().normalize();
        let crossAB = new THREE.Vector3().crossVectors(normalA, normalB);
        let angleEndEffector = Math.acos(normalA.dot(normalB));

        if(this.transitionType == 'Concave' || this.transitionType == "ConcaveSwap" ){angleEndEffector *= -1;}
        
        // Solve IK for the 3R Leg
        let angles = this.ik3R(targetPoint.y, targetPoint.x, this.offset, this.leg1, this.leg2, this.offset, angleEndEffector + Math.PI);
    
        // Apply the Computed Angles
        this.setAngle(1, angles.theta1);
        this.setAngle(2, angles.theta2);
        this.setAngle(3, angles.theta3);
        if (this.transitionType != 'None')this.lastTransitionType = this.transitionType;
        this.transitionType = 'None';

        this.robotGroup.updateMatrixWorld(true);
    }

    ik3R(x, y, L0, L1, L2, L3, psi) {
        let x2, y2;

        if(this.transitionType == 'Convex'){
            x2 = x - L3 *  Math.cos(-psi) - L0;
            y2 = y + L3 * Math.sin(-psi) ;
        }
        else if(this.transitionType == 'ConvexSwap'){
            x2 = x - L3 * Math.cos(psi) - L0;
            y2 = y + L3 * Math.sin(-psi);
        }        
        else if(this.transitionType == "ConcaveSwap"){
            x2 = x - L3 * Math.sin(psi);
            y2 = y - L3 * Math.cos(-psi) - L0;
        }
        else{
            x2 = x - L3 * Math.sin(psi);
            y2 = y - L3 * Math.cos(psi) - L0; 
        }
        let dSquared = x2 ** 2 + y2 ** 2;
        let cosTheta2 = (dSquared - L1 ** 2 - L2 ** 2) / (2 * L1 * L2);
        
        cosTheta2 = Math.min(1, Math.max(-1, cosTheta2));
    
        let theta2 = Math.acos(cosTheta2);
        let theta1 = Math.atan2(y2, x2) - Math.atan2(L2 * Math.sin(theta2), L1 + L2 * Math.cos(theta2));    
        let theta3 = psi - (theta1 + theta2);
        return { theta1, theta2, theta3 };
    }

    swapFixedLeg() {
        let newOrigin = this.target.position.clone();
        let newOriginNormal = this.target.normal.clone();
        let newTarget = this.origin.position.clone();
        let newTargetNormal = this.origin.normal.clone();

        let savedState = {
            angles: [...this.angles], 
            actionQueue: [...this.actionQueue], 
            isExecuting: this.isExecuting,
            lastTransitionType : this.lastTransitionType,
            initialGeometry: this.initialGeometry,
        };

        // Remove robot group and resources
        if (this.robotGroup) {
            while (this.robotGroup.children.length > 0) {
                let child = this.robotGroup.children.pop();
                if (child.geometry) child.geometry.dispose();
                if (child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach(mat => mat.dispose());
                    } else {
                        child.material.dispose();
                    }
                }
                this.robotGroup.remove(child);
            }
            this.scene.remove(this.robotGroup); 
        }
        this.origin = { position: newOrigin.clone(), normal: newOriginNormal.clone() };
        this.target = { position: newTarget.clone(), normal: newTargetNormal.clone() };
        this.fixed_leg = this.fixed_leg === 0 ? 4 : 0;

        // Save isEmpty status of arrows before rebuild
        const savedIsEmpty = {};
        Object.entries(this.voxelSlots).forEach(([name, slot]) => {
            savedIsEmpty[name] = slot.isEmpty;
        });

        // Rebuild robot structure
        this.robotGroup = new THREE.Group();
        this.robotGroup.position.copy(this.origin.position);
        this.joints = []; 
        this.robotBones = [];
        this.transitionType = 'None';

        // Restore State Variables
        this.angles = savedState.angles;
        this.actionQueue = savedState.actionQueue;
        this.isExecuting = savedState.isExecuting;

        if(this.lastTransitionType == "Concave"){this.transitionType = "ConcaveSwap"}
        if(this.lastTransitionType == "Convex"){this.transitionType = "ConvexSwap"}
        this.initialGeometry = savedState.initialGeometry

        this.buildRobot(this.initialGeometry);
        this.scene.add(this.robotGroup);

        let currentEndEffector = new THREE.Vector3();
        this.robotBones[4].getWorldPosition(currentEndEffector);
        this.direction = currentEndEffector.clone().sub(this.origin.position).normalize();

        let defaultNormal = new THREE.Vector3(0, 0, 1); 
        let rotationQuaternion = new THREE.Quaternion();
        if (!this.origin.normal.equals(defaultNormal)) {
            rotationQuaternion.setFromUnitVectors(defaultNormal, this.origin.normal);
        }
        this.robotGroup.quaternion.copy(rotationQuaternion);

        if (Math.abs(this.direction.dot(this.origin.normal) - 1) < 1e-6) {  
            if (this.origin.normal.x > 0.9) this.direction.set(0, 0, -1);
            else if (this.origin.normal.x < -0.9) this.direction.set(0, 0, 1);
            else if (this.origin.normal.y > 0.9) this.direction.set(1, 0, 0);
            else if (this.origin.normal.y < -0.9) this.direction.set(1, 0, 0);
            else if (this.origin.normal.z > 0.9) this.direction.set(1, 0, 0);
            else if (this.origin.normal.z < -0.9) this.direction.set(-1, 0, 0);
        }

        this.target.position.copy(newTarget.clone());
        this.target.normal.copy(newTargetNormal.clone()); 

        // Remove old arrows if any
        if (this.robotBones[0] && Array.isArray(this.robotBones[0].children)) {
            this.robotBones[0].children = this.robotBones[0].children.filter(child => !child.name?.startsWith("arrow_"));
        }

        // Set left/right/backward direction based on fixed leg parity
        const parity = this.fixed_leg === 0 ? 0 : 1;
        if (parity === 0) {
            this.voxelSlots.left.vector = new THREE.Vector3(0, 1, 0);
            this.voxelSlots.right.vector = new THREE.Vector3(0, -1, 0);
        } else {
            this.voxelSlots.left.vector = new THREE.Vector3(0, -1, 0);
            this.voxelSlots.right.vector = new THREE.Vector3(0, 1, 0);
        }
        this.voxelSlots.backward.vector = new THREE.Vector3(-1, 0, 0);

        this.addDirectionArrow();

        // Restore isEmpty status for arrows. Set color to black if empty, else initial color.
        Object.entries(this.voxelSlots).forEach(([name, slot]) => {
            slot.isEmpty = savedIsEmpty[name] !== undefined ? savedIsEmpty[name] : true;
            if (slot.mesh) {
                if (slot.isEmpty) {
                    slot.mesh.material.color.setHex(0x000000);
                } else if (slot.initialColor !== undefined) {
                    slot.mesh.material.color.setHex(slot.initialColor);
                }
            }
        });

        this.updateAnglesFromTarget();
    }

    computeEndEffectorPosition() {
        const lastBone = this.robotBones[this.robotBones.length - 1];
        lastBone.updateMatrixWorld(true);
        let pos = lastBone.getWorldPosition(new THREE.Vector3());
        return new THREE.Vector3(round(pos.x), round(pos.y), round(pos.z));
    }

    enqueueAction(action) {
        this.actionQueue.push(action);
        if (!this.isExecuting) {
            this.processNextAction();
        }
    }

    async processNextAction() {
        if (this.actionQueue.length === 0) {
            this.isExecuting = false;
            return;
        }
        this.isExecuting = true;
        
        let action = this.actionQueue.shift();
        await this.executeAction(action);
        this.processNextAction();
    }

    disposeRobotResources() {
        this.robotBones.forEach(bone => {
            if (bone.children && bone.children.length > 0) {
                bone.children.forEach(child => {
                    if (child.geometry) child.geometry.dispose();
                    if (child.material) {
                        if (Array.isArray(child.material)) {
                            child.material.forEach(m => m.dispose());
                        } else {
                            child.material.dispose();
                        }
                    }
                });
            }
            if (bone.geometry) bone.geometry.dispose();
            if (bone.material) {
                if (Array.isArray(bone.material)) {
                    bone.material.forEach(m => m.dispose());
                } else {
                    bone.material.dispose();
                }
            }
        });
        this.robotBones = [];
    }

    rotate() {
        // console.log("Rotating Robot");
        const dx = this.direction.x;
        const dy = this.direction.y;

        this.direction.set(-dy, dx, 0); // Rotate counter-clockwise

        // console.log(`Robot rotated! New direction: (${this.direction.x}, ${this.direction.y}, ${this.direction.z})`);
    }
    getRotationAngle(){
        let angle = 0;
        if (this.direction.x === 0 && this.direction.y === 1) angle = 0;
        else if (this.direction.x === -1 && this.direction.y === 0) angle = Math.PI/2;
        else if (this.direction.x === 0 && this.direction.y === -1) angle = Math.PI ;
        else if (this.direction.x === 1 && this.direction.y === 0) angle = -Math.PI/2;

        return angle;
    }
    delete() {
        // Remove joints
        if (Array.isArray(this.joints)) {
            this.joints.forEach(joint => {
                if (joint.parent) this.scene.remove(joint);
            });
        }

        // Remove robot bones
        if (Array.isArray(this.robotBones)) {
            this.robotBones.forEach(bone => {
                if (bone.parent) this.scene.remove(bone);
                if (bone.geometry) bone.geometry.dispose();
                if (bone.material) {
                    if (Array.isArray(bone.material)) {
                        bone.material.forEach(m => m.dispose());
                    } else {
                        bone.material.dispose();
                    }
                }
            });
        }

        // Remove entire robot group
        if (this.robotGroup && this.robotGroup.parent) {
            this.scene.remove(this.robotGroup);
        }

        this.joints = [];
        this.robotBones = [];
    }
    async executeAction(action) {
 
        return new Promise((resolve) => {
            // console.log(`Executing: ${action}`);
            // if (!VISUALIZATION) {
            this.movementHistory.push({
                action: action
            });
            // }
            // Normalize action name to remove invalid negative suffixes like "_-1"
            if (typeof action === "string") {
                action = action.replace(/_(-1)(?=[^0-9]|$)/g, '_0');
            } else {
                console.warn("Undefined or invalid action received:", action);
                resolve();
                return;
            }
            // console.log(`Executing action: ${action}`);
            if (action === "goForward") this.goForward(resolve);
            else if (action === "forwardHalf") this.forwardHalf(resolve);

            else if (action === "switchLeg") this.switchLeg(resolve);
            else if (action === "turnRight") this.turnRight(resolve);
            else if (action === "turnLeft") this.turnLeft(resolve);
            else if (action === "climbUp") this.climbUp(resolve);
            else if (action === "climbDown") this.climbDown(resolve);

            else if (action ==="sideStepDownRight") this.sideStepDownRight(resolve);
            else if (action ==="sideStepUpRight") this.sideStepUpRight(resolve);
            else if (action ==="sideStepDownLeft") this.sideStepDownLeft(resolve);
            else if (action ==="sideStepUpLeft") this.sideStepUpLeft(resolve);

            else if (action ==="climbDownHalf") this.climbDownHalf(resolve);
            else if (action ==="climbUpHalf") this.climbUpHalf(resolve);
            else if (action ==="sideHalfDownRight") this.sideHalfDownRight(resolve);
            else if (action ==="sideHalfUpRight") this.sideHalfUpRight(resolve);
            else if (action ==="sideHalfDownLeft") this.sideHalfDownLeft(resolve);
            else if (action ==="sideHalfUpLeft") this.sideHalfUpLeft(resolve);

            else if (action === "planTransitionConcave") this.planTransitionConcave(resolve);
            else if (action === "planTransitionConvex") this.planTransitionConvex(resolve);

            else if (action ==="climbUpFlatStair") this.climbUpFlatStair(resolve);
            else if (action ==="climbUpFlatStairHalf") this.climbUpFlatStairHalf(resolve);
            else if (action ==="climbUpStairStair") this.climbUpStairStair(resolve);
            else if (action ==="climbUpStairStairHalf") this.climbUpStairStairHalf(resolve);
            else if (action ==="climbUpStairFlat") this.climbUpStairFlat(resolve);
            else if (action ==="climbUpStairFlatHalf") this.climbUpStairFlatHalf(resolve);

            else if (action ==="climbTopStair") this.climbTopStair(resolve);
            else if (action ==="climbTopStairHalf") this.climbTopStairHalf(resolve);
            else if (action ==="climbBottomStair") this.climbBottomStair(resolve);
            else if (action ==="climbBottomStairHalf") this.climbBottomStairHalf(resolve);

            else if (action ==="climbDownFlatStair") this.climbDownFlatStair(resolve);
            else if (action ==="climbDownFlatStairHalf") this.climbDownFlatStairHalf(resolve);
            else if (action ==="climbDownStairStair") this.climbDownStairStair(resolve);
            else if (action ==="climbDownStairStairHalf") this.climbDownStairStairHalf(resolve);
            else if (action ==="climbDownStairFlat") this.climbDownStairFlat(resolve);
            else if (action ==="climbDownStairFlatHalf") this.climbDownStairFlatHalf(resolve);

            else if (action ==="climbDownStairFullHalf") this.climbDownStairFullHalf(resolve);
            else if (action ==="climbDownStairHalfFull") this.climbDownStairHalfFull(resolve);
            else if (action ==="climbBottomStairFullHalf") this.climbBottomStairFullHalf(resolve);
            else if (action ==="climbBottomStairHalfFull") this.climbBottomStairHalfFull(resolve);
            else if (action ==="climbUpStairFullHalf") this.climbUpStairFullHalf(resolve);
            else if (action ==="climbUpStairHalfFull") this.climbUpStairHalfFull(resolve);
            else if (action ==="climbTopStairFullHalf") this.climbTopStairFullHalf(resolve);
            else if (action ==="climbTopStairHalfFull") this.climbTopStairHalfFull(resolve);
            
            else if (action === "climbDownStairFlatOneHalf") this.climbDownStairFlatOneHalf(resolve);
            else if (action === "climbDownFlatStairOneHalf") this.climbDownFlatStairOneHalf(resolve);
            else if (action === "climbUpStairFlatOneHalf") this.climbUpStairFlatOneHalf(resolve);
            else if (action === "climbUpFlatStairOneHalf") this.climbUpFlatStairOneHalf(resolve);
            else if (action === "forwardOneHalf") this.forwardOneHalf(resolve);
            else if (action === "forwardTwo") this.forwardTwo(resolve);

            // COLUMNS
            else if(action === "columnUp") this.columnUp(resolve);
            else if(action === "columnDown") this.columnDown(resolve);

            // Picking up action
            else if(action === "pickUpRight") {
                this.pickUpRight(resolve);
                this.updateArrowStatus("left"); 
                this.updateArrowStatus("right");
                this.updateArrowStatus("backward");
            }
            else if(action === "pickUpLeft")  {
                this.pickUpLeft(resolve);
                this.updateArrowStatus("left"); 
                this.updateArrowStatus("right");
                this.updateArrowStatus("backward");
            }

            // Placement Gestures
            else if (action === "placing_1l_1_1") this.placing_1l_1_1(resolve);
            else if (action === "placing_1l_1_0_5") this.placing_1l_1_0_5(resolve);
            else if (action === "placing_1l_1_0") this.placing_1l_1_0(resolve);
            else if (action === "placing_1r_1_1") this.placing_1r_1_1(resolve);
            else if (action === "placing_1r_1_0_5") this.placing_1r_1_0_5(resolve);
            else if (action === "placing_1r_1_0") this.placing_1r_1_0(resolve);
            else if (action === "placing_1l_2_1") this.placing_1l_2_1(resolve);
            else if (action === "placing_1l_2_0_5") this.placing_1l_2_0_5(resolve);
            else if (action === "placing_1l_2_0") this.placing_1l_2_0(resolve);
            else if (action === "placing_1r_2_1") this.placing_1r_2_1(resolve);
            else if (action === "placing_1r_2_0_5") this.placing_1r_2_0_5(resolve);
            else if (action === "placing_1r_2_0") this.placing_1r_2_0(resolve);
            else if (action === "placing_2l_1") this.placing_2l_1(resolve);
            else if (action === "placing_2l_0_5") this.placing_2l_0_5(resolve);
            else if (action === "placing_2l_0") this.placing_2l_0(resolve);
            else if (action === "placing_2r_1") this.placing_2r_1(resolve);
            else if (action === "placing_2r_0_5") this.placing_2r_0_5(resolve);
            else if (action === "placing_2r_0") this.placing_2r_0(resolve);
            else if (action === "placing_3l_1") this.placing_3l_1(resolve);
            else if (action === "placing_3l_0_5") this.placing_3l_0_5(resolve);
            else if (action === "placing_3l_0") this.placing_3l_0(resolve);
            else if (action === "placing_3r_1") this.placing_3r_1(resolve);
            else if (action === "placing_3r_0_5") this.placing_3r_0_5(resolve);
            else if (action === "placing_3r_0") this.placing_3r_0(resolve);
            else if (action === "placing_3c_1") this.placing_3c_1(resolve);
            else if (action === "placing_3c_0_5") this.placing_3c_0_5(resolve);
            else if (action === "placing_3c_0") this.placing_3c_0(resolve);
            else if (action === "placing_4l_1") this.placing_4l_1(resolve);
            else if (action === "placing_4l_0_5") this.placing_4l_0_5(resolve);
            else if (action === "placing_4l_0") this.placing_4l_0(resolve);
            else if (action === "placing_4r_1") this.placing_4r_1(resolve);
            else if (action === "placing_4r_0_5") this.placing_4r_0_5(resolve);
            else if (action === "placing_4r_0") this.placing_4r_0(resolve);
            else if (action === "placing_r_1r_1_1") this.placing_r_1r_1_1(resolve);
            else if (action === "placing_r_1r_1_0_5") this.placing_r_1r_1_0_5(resolve);
            else if (action === "placing_r_1r_1_0") this.placing_r_1r_1_0(resolve);
            else if (action === "placing_r_1l_1_1") this.placing_r_1l_1_1(resolve);
            else if (action === "placing_r_1l_1_0_5") this.placing_r_1l_1_0_5(resolve);
            else if (action === "placing_r_1l_1_0") this.placing_r_1l_1_0(resolve);
            else if (action === "placing_r_1l_2_1") this.placing_r_1l_2_1(resolve);
            else if (action === "placing_r_1l_2_0_5") this.placing_r_1l_2_0_5(resolve);
            else if (action === "placing_r_1l_2_0") this.placing_r_1l_2_0(resolve);
            else if (action === "placing_r_1r_2_1") this.placing_r_1r_2_1(resolve);
            else if (action === "placing_r_1r_2_0_5") this.placing_r_1r_2_0_5(resolve);
            else if (action === "placing_r_1r_2_0") this.placing_r_1r_2_0(resolve);
            else if (action === "placing_r_2l_1") this.placing_r_2l_1(resolve);
            else if (action === "placing_r_2l_0_5") this.placing_r_2l_0_5(resolve);
            else if (action === "placing_r_2l_0") this.placing_r_2l_0(resolve);
            else if (action === "placing_r_2r_1") this.placing_r_2r_1(resolve);
            else if (action === "placing_r_2r_0_5") this.placing_r_2r_0_5(resolve);
            else if (action === "placing_r_2r_0") this.placing_r_2r_0(resolve);
            else if (action === "placing_r_3l_1") this.placing_r_3l_1(resolve);
            else if (action === "placing_r_3l_0_5") this.placing_r_3l_0_5(resolve);
            else if (action === "placing_r_3l_0") this.placing_r_3l_0(resolve);
            else if (action === "placing_r_3r_1") this.placing_r_3r_1(resolve);
            else if (action === "placing_r_3r_0_5") this.placing_r_3r_0_5(resolve);
            else if (action === "placing_r_3r_0") this.placing_r_3r_0(resolve);
            else if (action === "placing_r_3c_1") this.placing_r_3c_1(resolve);
            else if (action === "placing_r_3c_0_5") this.placing_r_3c_0_5(resolve);
            else if (action === "placing_r_3c_0") this.placing_r_3c_0(resolve);
            else if (action === "placing_r_4l_1") this.placing_r_4l_1(resolve);
            else if (action === "placing_r_4l_0_5") this.placing_r_4l_0_5(resolve);
            else if (action === "placing_r_4l_0") this.placing_r_4l_0(resolve);
            else if (action === "placing_r_4r_1") this.placing_r_4r_1(resolve);
            else if (action === "placing_r_4r_0_5") this.placing_r_4r_0_5(resolve);
            else if (action === "placing_r_4r_0") this.placing_r_4r_0(resolve);
            // === Placement 2.0 Gestures ===
            else if (action === "placing_1l_1_2") this.placing_1l_1_2(resolve);
            else if (action === "placing_1r_1_2") this.placing_1r_1_2(resolve);
            else if (action === "placing_1l_2_2") this.placing_1l_2_2(resolve);
            else if (action === "placing_1r_2_2") this.placing_1r_2_2(resolve);
            else if (action === "placing_2l_2") this.placing_2l_2(resolve);
            else if (action === "placing_2r_2") this.placing_2r_2(resolve);
            else if (action === "placing_3l_2") this.placing_3l_2(resolve);
            else if (action === "placing_3r_2") this.placing_3r_2(resolve);
            else if (action === "placing_3c_2") this.placing_3c_2(resolve);
            else if (action === "placing_4l_2") this.placing_4l_2(resolve);
            else if (action === "placing_4r_2") this.placing_4r_2(resolve);
            else if (action === "placing_r_1l_1_2") this.placing_r_1l_1_2(resolve);
            else if (action === "placing_r_1r_1_2") this.placing_r_1r_1_2(resolve);
            else if (action === "placing_r_1l_2_2") this.placing_r_1l_2_2(resolve);
            else if (action === "placing_r_1r_2_2") this.placing_r_1r_2_2(resolve);
            else if (action === "placing_r_2l_2") this.placing_r_2l_2(resolve);
            else if (action === "placing_r_2r_2") this.placing_r_2r_2(resolve);
            else if (action === "placing_r_3l_2") this.placing_r_3l_2(resolve);
            else if (action === "placing_r_3r_2") this.placing_r_3r_2(resolve);
            else if (action === "placing_r_3c_2") this.placing_r_3c_2(resolve);
            else if (action === "placing_r_4l_2") this.placing_r_4l_2(resolve);
            else if (action === "placing_r_4r_2") this.placing_r_4r_2(resolve);
            // Side Step Actions
            else if (action === "sideStepFlatLeftHalf") this.sideStepFlatLeftHalf(resolve);
            else if (action === "sideStepFlatRightHalf") this.sideStepFlatRightHalf(resolve);
            else if (action === "sideStepFlatLeft") this.sideStepFlatLeft(resolve);
            else if (action === "sideStepFlatRight") this.sideStepFlatRight(resolve);
            else if (action === "turnRightClimbUp") this.turnRightClimbUp(resolve);
            else if (action === "turnRightClimbDown") this.turnRightClimbDown(resolve);
            else if (action === "turnLeftClimbUp") this.turnLeftClimbUp(resolve);
            else if (action === "turnLeftClimbDown") this.turnLeftClimbDown(resolve);
            else if (action === "turnRightClimbUpHalf") this.turnRightClimbUpHalf(resolve);
            else if (action === "turnRightClimbDownHalf") this.turnRightClimbDownHalf(resolve);
            else if (action === "turnLeftClimbUpHalf") this.turnLeftClimbUpHalf(resolve);
            else if (action === "turnLeftClimbDownHalf") this.turnLeftClimbDownHalf(resolve);

            else if (action === "turnRightClimbUpBack") this.turnRightClimbUpBack(resolve);
            else if (action === "turnRightClimbDownBack") this.turnRightClimbDownBack(resolve);
            else if (action === "turnLeftClimbUpBack") this.turnLeftClimbUpBack(resolve);
            else if (action === "turnLeftClimbDownBack") this.turnLeftClimbDownBack(resolve);
            else if (action === "turnRightClimbUpHalfBack") this.turnRightClimbUpHalfBack(resolve);
            else if (action === "turnRightClimbDownHalfBack") this.turnRightClimbDownHalfBack(resolve);
            else if (action === "turnLeftClimbUpHalfBack") this.turnLeftClimbUpHalfBack(resolve);
            else if (action === "turnLeftClimbDownHalfBack") this.turnLeftClimbDownHalfBack(resolve);
            else if (action === 'F101B101') this.F101B101(resolve);
            else if (action === 'F10m1B10m1') this.F10m1B10m1(resolve);
            else if (action === 'Fm101Bm101') this.Fm101Bm101(resolve);
            else if (action === 'Fm10m1Bm10m1') this.Fm10m1Bm10m1(resolve);
            else if (action === 'F101B101Half') this.F101B101Half(resolve);
            else if (action === 'F10m1B10m1Half') this.F10m1B10m1Half(resolve);     
            else if (action === 'Fm101Bm101Half') this.Fm101Bm101Half(resolve);
            else if (action === 'Fm10m1Bm10m1Half') this.Fm10m1Bm10m1Half(resolve);
            else if (action === 'F0p5L1B1') this.F0p5L1B1(resolve);
            else if (action === 'F0p5R1B1') this.F0p5R1B1(resolve);
            else if (action === 'F1L1B0p5') this.F1L1B0p5(resolve);
            else if (action === 'F1R1B0p5') this.F1R1B0p5(resolve);
            else if (action === 'rotateMovingLegWithZOffset') this.rotateMovingLegWithZOffset(resolve);
            // else if (action === 'turnLeftClimbDownBackInvert') this.turnLeftClimbDownBackInvert(resolve);
            // else if (action === 'turnRightClimbDownBackInvert') this.turnRightClimbDownBackInvert(resolve);
            // else if (action === 'turnRightClimbDownHalfBackInvert') this.turnRightClimbDownHalfBackInvert(resolve); 
            // else if (action === 'turnLeftClimbDownHalfBackInvert') this.turnLeftClimbDownHalfBackInvert(resolve);

            else {
                console.warn("Action unknown", action);
                resolve(); 
            }
        });
    }
    /**
     * Animates the rotation of the last joint (back leg end effector) around the Z-axis to the target angle smoothly.
     * @param {number} targetAngle - The angle (in radians) to set for the last joint's Z rotation.
     * @param {number} duration - Duration of the animation in milliseconds (default: 300ms).
     * @returns {Promise<void>} Resolves when the rotation is complete.
     */
    rotateBackLegEndEffector(targetAngle, duration = 300) {
        return new Promise(resolve => {
            if (!this.robotBones[4]) return resolve();
            const joint = this.robotBones[4];
            const startAngle = joint.rotation.z;
            const startTime = performance.now();

            const animate = (now) => {
                const elapsed = now - startTime;
                const t = Math.min(elapsed / duration, 1);
                joint.rotation.z = THREE.MathUtils.lerp(startAngle, targetAngle, t);
                this.angles[4] = joint.rotation.z;
                if (t < 1) {
                    requestAnimationFrame(animate);
                } else {
                    resolve();
                }
            };

            requestAnimationFrame(animate);
        });
    }
}
THREERobot.prototype.rotateMovingLegWithZOffset = rotateMovingLegWithZOffset
THREERobot.prototype.turnRightClimbUpBack = turnRightClimbUpBack;
THREERobot.prototype.turnRightClimbDownBack = turnRightClimbDownBack;
THREERobot.prototype.turnLeftClimbUpBack = turnLeftClimbUpBack;
THREERobot.prototype.turnLeftClimbDownBack = turnLeftClimbDownBack;
THREERobot.prototype.turnRightClimbUpHalfBack = turnRightClimbUpHalfBack;
THREERobot.prototype.turnRightClimbDownHalfBack = turnRightClimbDownHalfBack;
THREERobot.prototype.turnLeftClimbUpHalfBack = turnLeftClimbUpHalfBack;
THREERobot.prototype.turnLeftClimbDownHalfBack = turnLeftClimbDownHalfBack;

// === Placement 2.0 Movement Prototypes ===
THREERobot.prototype.placing_1l_1_2 = placing_1l_1_2;
THREERobot.prototype.placing_1r_1_2 = placing_1r_1_2;
THREERobot.prototype.placing_1l_2_2 = placing_1l_2_2;
THREERobot.prototype.placing_1r_2_2 = placing_1r_2_2;
THREERobot.prototype.placing_2l_2 = placing_2l_2;
THREERobot.prototype.placing_2r_2 = placing_2r_2;
THREERobot.prototype.placing_3l_2 = placing_3l_2;
THREERobot.prototype.placing_3r_2 = placing_3r_2;
THREERobot.prototype.placing_3c_2 = placing_3c_2;
THREERobot.prototype.placing_4l_2 = placing_4l_2;
THREERobot.prototype.placing_4r_2 = placing_4r_2;
THREERobot.prototype.placing_r_1l_1_2 = placing_r_1l_1_2;
THREERobot.prototype.placing_r_1r_1_2 = placing_r_1r_1_2;
THREERobot.prototype.placing_r_1l_2_2 = placing_r_1l_2_2;
THREERobot.prototype.placing_r_1r_2_2 = placing_r_1r_2_2;
THREERobot.prototype.placing_r_2l_2 = placing_r_2l_2;
THREERobot.prototype.placing_r_2r_2 = placing_r_2r_2;
THREERobot.prototype.placing_r_3l_2 = placing_r_3l_2;
THREERobot.prototype.placing_r_3r_2 = placing_r_3r_2;
THREERobot.prototype.placing_r_3c_2 = placing_r_3c_2;
THREERobot.prototype.placing_r_4l_2 = placing_r_4l_2;
THREERobot.prototype.placing_r_4r_2 = placing_r_4r_2;
// THREERobot.prototype.turnLeftClimbDownBackInvert = turnLeftClimbDownBackInvert;
// THREERobot.prototype.turnRightClimbDownBackInvert = turnRightClimbDownBackInvert;
// THREERobot.prototype.turnRightClimbDownHalfBackInvert = turnRightClimbDownHalfBackInvert;
// THREERobot.prototype.turnLeftClimbDownHalfBackInvert = turnLeftClimbDownHalfBackInvert;

THREERobot.prototype.F101B101 = F101B101;
THREERobot.prototype.F10m1B10m1 = F10m1B10m1;
THREERobot.prototype.Fm101Bm101 = Fm101Bm101;
THREERobot.prototype.Fm10m1Bm10m1 = Fm10m1Bm10m1;

THREERobot.prototype.F101B101Half = F101B101Half;
THREERobot.prototype.F10m1B10m1Half = F10m1B10m1Half;
THREERobot.prototype.Fm101Bm101Half = Fm101Bm101Half;
THREERobot.prototype.Fm10m1Bm10m1Half = Fm10m1Bm10m1Half;

THREERobot.prototype.columnUp = columnUp;
THREERobot.prototype.columnDown = columnDown;

THREERobot.prototype.turnRightClimbUp = turnRightClimbUp;
THREERobot.prototype.turnRightClimbDown = turnRightClimbDown;
THREERobot.prototype.turnLeftClimbUp = turnLeftClimbUp;
THREERobot.prototype.turnLeftClimbDown = turnLeftClimbDown;
THREERobot.prototype.turnRightClimbUpHalf = turnRightClimbUpHalf;
THREERobot.prototype.turnRightClimbDownHalf = turnRightClimbDownHalf;
THREERobot.prototype.turnLeftClimbUpHalf = turnLeftClimbUpHalf;
THREERobot.prototype.turnLeftClimbDownHalf = turnLeftClimbDownHalf;

THREERobot.prototype.sideStepFlatLeftHalf=sideStepFlatLeftHalf;
THREERobot.prototype.sideStepFlatRightHalf=sideStepFlatRightHalf;
THREERobot.prototype.sideStepFlatLeft=sideStepFlatLeft;
THREERobot.prototype.sideStepFlatRight=sideStepFlatRight;

THREERobot.prototype.climbDownStairFlatOneHalf=climbDownStairFlatOneHalf;
THREERobot.prototype.climbDownFlatStairOneHalf=climbDownFlatStairOneHalf;
THREERobot.prototype.climbUpStairFlatOneHalf=climbUpStairFlatOneHalf;
THREERobot.prototype.climbUpFlatStairOneHalf=climbUpFlatStairOneHalf;
THREERobot.prototype.forwardTwo=forwardTwo;
THREERobot.prototype.forwardOneHalf=forwardOneHalf;

THREERobot.prototype.climbTopStair=climbTopStair;
THREERobot.prototype.climbTopStairHalf=climbTopStairHalf;
THREERobot.prototype.climbBottomStair=climbBottomStair;
THREERobot.prototype.climbBottomStairHalf=climbBottomStairHalf;

THREERobot.prototype.climbUpFlatStair=climbUpFlatStair;
THREERobot.prototype.climbUpFlatStairHalf=climbUpFlatStairHalf;
THREERobot.prototype.climbUpStairStair=climbUpStairStair;
THREERobot.prototype.climbUpStairStairHalf=climbUpStairStairHalf;
THREERobot.prototype.climbUpStairFlat=climbUpStairFlat;
THREERobot.prototype.climbUpStairFlatHalf=climbUpStairFlatHalf;

THREERobot.prototype.climbDownFlatStair=climbDownFlatStair;
THREERobot.prototype.climbDownFlatStairHalf=climbDownFlatStairHalf;
THREERobot.prototype.climbDownStairStair=climbDownStairStair;
THREERobot.prototype.climbDownStairStairHalf=climbDownStairStairHalf;
THREERobot.prototype.climbDownStairFlat=climbDownStairFlat;
THREERobot.prototype.climbDownStairFlatHalf=climbDownStairFlatHalf;

THREERobot.prototype.climbDownStairFullHalf=climbDownStairFullHalf;
THREERobot.prototype.climbDownStairHalfFull=climbDownStairHalfFull;
THREERobot.prototype.climbBottomStairFullHalf=climbBottomStairFullHalf;
THREERobot.prototype.climbBottomStairHalfFull=climbBottomStairHalfFull;

THREERobot.prototype.climbUpStairFullHalf=climbUpStairFullHalf;
THREERobot.prototype.climbUpStairHalfFull=climbUpStairHalfFull;
THREERobot.prototype.climbTopStairFullHalf=climbTopStairFullHalf;
THREERobot.prototype.climbTopStairHalfFull=climbTopStairHalfFull;

THREERobot.prototype.pickUpLeft=pickUpLeft;
THREERobot.prototype.pickUpRight=pickUpRight;


// === Placement Movement Prototypes ===
THREERobot.prototype.placing_1l_1_1 = placing_1l_1_1;
THREERobot.prototype.placing_1l_1_0_5 = placing_1l_1_0_5;
THREERobot.prototype.placing_1l_1_0 = placing_1l_1_0;

THREERobot.prototype.placing_1r_1_1 = placing_1r_1_1;
THREERobot.prototype.placing_1r_1_0_5 = placing_1r_1_0_5;
THREERobot.prototype.placing_1r_1_0 = placing_1r_1_0;

THREERobot.prototype.placing_1l_2_1 = placing_1l_2_1;
THREERobot.prototype.placing_1l_2_0_5 = placing_1l_2_0_5;
THREERobot.prototype.placing_1l_2_0 = placing_1l_2_0;

THREERobot.prototype.placing_1r_2_1 = placing_1r_2_1;
THREERobot.prototype.placing_1r_2_0_5 = placing_1r_2_0_5;
THREERobot.prototype.placing_1r_2_0 = placing_1r_2_0;

THREERobot.prototype.placing_2l_1 = placing_2l_1;
THREERobot.prototype.placing_2l_0_5 = placing_2l_0_5;
THREERobot.prototype.placing_2l_0 = placing_2l_0;

THREERobot.prototype.placing_2r_1 = placing_2r_1;
THREERobot.prototype.placing_2r_0_5 = placing_2r_0_5;
THREERobot.prototype.placing_2r_0 = placing_2r_0;

THREERobot.prototype.placing_3l_1 = placing_3l_1;
THREERobot.prototype.placing_3l_0_5 = placing_3l_0_5;
THREERobot.prototype.placing_3l_0 = placing_3l_0;

THREERobot.prototype.placing_3r_1 = placing_3r_1;
THREERobot.prototype.placing_3r_0_5 = placing_3r_0_5;
THREERobot.prototype.placing_3r_0 = placing_3r_0;

THREERobot.prototype.placing_3c_1 = placing_3c_1;
THREERobot.prototype.placing_3c_0_5 = placing_3c_0_5;
THREERobot.prototype.placing_3c_0 = placing_3c_0;

THREERobot.prototype.placing_4l_1 = placing_4l_1;
THREERobot.prototype.placing_4l_0_5 = placing_4l_0_5;
THREERobot.prototype.placing_4l_0 = placing_4l_0;

THREERobot.prototype.placing_4r_1 = placing_4r_1;
THREERobot.prototype.placing_4r_0_5 = placing_4r_0_5;
THREERobot.prototype.placing_4r_0 = placing_4r_0;

// === Inverse Placement Movement Prototypes ===
THREERobot.prototype.placing_r_1r_1_1 = placing_r_1r_1_1;
THREERobot.prototype.placing_r_1r_1_0_5 = placing_r_1r_1_0_5;
THREERobot.prototype.placing_r_1r_1_0 = placing_r_1r_1_0;

THREERobot.prototype.placing_r_1l_1_1 = placing_r_1l_1_1;
THREERobot.prototype.placing_r_1l_1_0_5 = placing_r_1l_1_0_5;
THREERobot.prototype.placing_r_1l_1_0 = placing_r_1l_1_0;

THREERobot.prototype.placing_r_1l_2_1 = placing_r_1l_2_1;
THREERobot.prototype.placing_r_1l_2_0_5 = placing_r_1l_2_0_5;
THREERobot.prototype.placing_r_1l_2_0 = placing_r_1l_2_0;

THREERobot.prototype.placing_r_1r_2_1 = placing_r_1r_2_1;
THREERobot.prototype.placing_r_1r_2_0_5 = placing_r_1r_2_0_5;
THREERobot.prototype.placing_r_1r_2_0 = placing_r_1r_2_0;

THREERobot.prototype.placing_r_2l_1 = placing_r_2l_1;
THREERobot.prototype.placing_r_2l_0_5 = placing_r_2l_0_5;
THREERobot.prototype.placing_r_2l_0 = placing_r_2l_0;

THREERobot.prototype.placing_r_2r_1 = placing_r_2r_1;
THREERobot.prototype.placing_r_2r_0_5 = placing_r_2r_0_5;
THREERobot.prototype.placing_r_2r_0 = placing_r_2r_0;

THREERobot.prototype.placing_r_3l_1 = placing_r_3l_1;
THREERobot.prototype.placing_r_3l_0_5 = placing_r_3l_0_5;
THREERobot.prototype.placing_r_3l_0 = placing_r_3l_0;

THREERobot.prototype.placing_r_3r_1 = placing_r_3r_1;
THREERobot.prototype.placing_r_3r_0_5 = placing_r_3r_0_5;
THREERobot.prototype.placing_r_3r_0 = placing_r_3r_0;

THREERobot.prototype.placing_r_3c_1 = placing_r_3c_1;
THREERobot.prototype.placing_r_3c_0_5 = placing_r_3c_0_5;
THREERobot.prototype.placing_r_3c_0 = placing_r_3c_0;

THREERobot.prototype.placing_r_4l_1 = placing_r_4l_1;
THREERobot.prototype.placing_r_4l_0_5 = placing_r_4l_0_5;
THREERobot.prototype.placing_r_4l_0 = placing_r_4l_0;

THREERobot.prototype.placing_r_4r_1 = placing_r_4r_1;
THREERobot.prototype.placing_r_4r_0_5 = placing_r_4r_0_5;
THREERobot.prototype.placing_r_4r_0 = placing_r_4r_0;


// Attach movement functions dynamically
THREERobot.prototype.goForward = goForward;
THREERobot.prototype.forwardHalf = forwardHalf;

THREERobot.prototype.switchLeg = switchLeg;
THREERobot.prototype.turnRight = turnRight;
THREERobot.prototype.turnLeft = turnLeft;
THREERobot.prototype.planTransitionConcave = planTransitionConcave;
THREERobot.prototype.planTransitionConvex = planTransitionConvex;
THREERobot.prototype.climbDown = climbDown;
THREERobot.prototype.climbUp = climbUp;

THREERobot.prototype.sideHalfDownRight = sideHalfDownRight;
THREERobot.prototype.sideHalfUpRight = sideHalfUpRight;
THREERobot.prototype.sideHalfUpLeft = sideHalfUpLeft;
THREERobot.prototype.climbDownHalf = climbDownHalf;
THREERobot.prototype.climbUpHalf=climbUpHalf;
THREERobot.prototype.sideHalfDownLeft=sideHalfDownLeft;

THREERobot.prototype.calculateMovementVector = calculateMovementVector;
// THREERobot.prototype.displayTrajectory = displayTrajectory;
// THREERobot.prototype.clearTrajectory = clearTrajectory;
THREERobot.prototype.rotateMovingLeg = rotateMovingLeg;
THREERobot.prototype.moveLegBezier = moveLegBezier;
THREERobot.prototype.sideStepDownRight = sideStepDownRight;
THREERobot.prototype.sideStepUpRight = sideStepUpRight;
THREERobot.prototype.sideStepDownLeft = sideStepDownLeft;
THREERobot.prototype.sideStepUpLeft = sideStepUpLeft;
THREERobot.prototype.interpolateMovement = interpolateMovement;
// THREERobot.prototype.planPathToCoordinate = function(goalPos, goalNormal, voxelToAvoid = undefined) {
//     return planPathToCoordinate.call(this, goalPos, goalNormal, voxelToAvoid); // Bind function to robot instance
// };
THREERobot.prototype.planPathToLegPositions = function({ frontLegGoal, backLegGoal = null, normalGoal, voxelToAvoid = undefined, requireParity = true, start = null }) {
    return planPathToLegPositions.call(this, { frontLegGoal, backLegGoal, normalGoal, voxelToAvoid, requireParity, start });
};
THREERobot.prototype.F0p5L1B1 = F0p5L1B1;
THREERobot.prototype.F0p5R1B1 = F0p5R1B1;
THREERobot.prototype.F1L1B0p5 = F1L1B0p5;
THREERobot.prototype.F1R1B0p5 = F1R1B0p5;
// The planTransition function will handle the transition between two adjacent surfaces by adjusting the robot’s legs to match the new surface orientation. There are two cases:
// 	1.	Concave Transition:
// 	•	The robot transitions from a surface to another at a 90-degree inward angle.
// 	•	It moves the moving leg first, performing a 90-degree rotation around the vector perpendicular to the two surface normals.
// 	•	The leg is placed two step sizes away on the new surface.
// 	•	The fixed leg is then moved one step size to follow.
// 	2.	Convex Transition:
// 	•	The robot transitions from a surface to another at a 90-degree outward angle.
// 	•	It moves in the direction of the normal to the current surface.
// 	•	The movement is performed around the intersection vector of the current and target surface normals.
// 	•	The leg follows a parabolic path, lifting the robot over the edge smoothly.
// 	•	The fixed leg follows after the moving leg reaches the new surface.

