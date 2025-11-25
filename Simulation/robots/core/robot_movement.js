import * as THREE from "../../three/build/three.module.min.js";

// Do I really have a switch leg in reality ? maybe not ...

const STEP_SIZE = 1.0;              // Global step size should match voxel size
// const INTERMEDIARY_STEPS = 1;      // number of interpolated point in each movment (reduce for faster simulation)
// const DELAY = 1;                   // delay between two movement 
const tmpVec1 = new THREE.Vector3();
const tmpVec2 = new THREE.Vector3();
const tmpVec3 = new THREE.Vector3();
const tmpVec4 = new THREE.Vector3();
const tmpVec5 = new THREE.Vector3();

export let INTERMEDIARY_STEPS = 1;
export let DELAY = 0;

export function setSpeedTuning(steps, delay) {
  INTERMEDIARY_STEPS = steps;
  DELAY = delay;
}

// ==================== ESSENTIAL MOVEMENT ====================

export function switchLeg(resolve) {
    this.swapFixedLeg();
    if (resolve) resolve();
}
// ==================== COLUMN MVT =========================

export function columnUp(resolve, step = 1) {
    // 1) Compute start & fixed positions
    this.swapFixedLeg();

    const fixedPos  = this.origin.position.clone();
    const movingPos = this.target.position.clone();

    // 2) Compute the rotated foot position around the vertical normal
    const axis  = this.origin.normal.clone().normalize();
    const angle = (Math.PI / 2) * step;                         // 90° * step
    const q     = new THREE.Quaternion().setFromAxisAngle(axis, angle);
    const rel   = movingPos.clone().sub(fixedPos).applyQuaternion(q);
  
    // 3) Add the upward climb in the same move
    const lift  = axis.clone().multiplyScalar(0.5 * STEP_SIZE); // half‑voxel up
    const endPos = fixedPos.clone().add(rel).add(lift)
                       .multiplyScalar(10).round().divideScalar(10);
  
    // 4) Perform one Bézier move: rotating+lifting in one go
    this.moveLegBezier(
      movingPos,         // start
      endPos,            // end (rotated + lifted)
      this.target.normal,// start normal
      axis,              // end normal (same surface)
      () => {            // on complete
        this.swapFixedLeg();

        if (resolve) resolve();
      }
    );
  }
  export function columnDown(resolve, step = 1) {

    // 1) Get the fixed and moving leg start positions
    const fixedPos  = this.origin.position.clone();
    const movingPos = this.target.position.clone();

    // 2) Rotation axis is the surface normal (vertical)
    const axis  = this.origin.normal.clone().normalize();

    // 3) We want to rotate 90° *left*, i.e. -π/2 radians
    const angle = - (Math.PI / 2) * step;
    const q     = new THREE.Quaternion().setFromAxisAngle(axis, angle);

    // 4) Compute the new relative vector after rotation
    const rel = movingPos.clone().sub(fixedPos).applyQuaternion(q);

    // 5) Compute the “down” lift of half a voxel
    const down = axis.clone().multiplyScalar(-0.5 * STEP_SIZE);

    // 6) Final target position for the moving leg
    const endPos = fixedPos.clone()
                           .add(rel)
                           .add(down)
                           // round to grid
                           .multiplyScalar(10)
                           .round()
                           .divideScalar(10);

    // 7) Execute one Bézier move: rotate+descend
    this.moveLegBezier(
      movingPos,           // start
      endPos,              // end (rotated left + down 0.5)
      this.target.normal,  // start normal
      axis,                // end normal (still same surface)
      () => {
        // done!

        if (resolve) resolve();
      }
    );
}
export function goForward(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let firstLegEndPosition = startMovingLeg.clone().add(movementVector.clone().multiplyScalar(step*STEP_SIZE))
                                            .multiplyScalar(10).round().divideScalar(10);
    let secondLegEndPosition = startMovingLeg2.clone().add(movementVector.clone().multiplyScalar(step*STEP_SIZE))
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function turnRight(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function turnLeft(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal,movementVector).normalize();
    
    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .multiplyScalar(10).round().divideScalar(10);
    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbUp(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(2 * STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(1.5*step * STEP_SIZE);                                // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).add(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbDown(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(2 * STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                                // Move down 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).sub(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function planTransitionConvex(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(0.5 * STEP_SIZE);        // Move forward
    let step2 = currentNormal.clone().multiplyScalar(1.5 * STEP_SIZE);          // Move up onto new surface
    
    let rotationAxis = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let newNormal = currentNormal.clone().applyAxisAngle(rotationAxis, -Math.PI / 2).normalize();
 
    let firstLegPosition = startMovingLeg.clone().add(step1).sub(step2);        // First leg moves onto the wall

    this.interpolateMovement(startMovingLeg, firstLegPosition, currentNormal, newNormal, 'Convex',() => {
        this.swapFixedLeg();                                                    // Swap fixed leg after first move
        let step3 = movementVector.clone().multiplyScalar(1.5 * STEP_SIZE);  
        let step4 = currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE);  

        let secondLegPosition = startMovingLeg2.clone().add(step3).sub(step4);  // Second leg moves onto the wall

        this.interpolateMovement(startMovingLeg2, secondLegPosition, currentNormal, newNormal,'ConvexSwap', () => {
            this.swapFixedLeg();                                                // Swap fixed leg after final move
            if (resolve) resolve();
        });
    });
}

export function planTransitionConcave(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(0.5 * STEP_SIZE);  
    let step2 = currentNormal.clone().multiplyScalar(1.5 * STEP_SIZE);  
    
    let rotationAxis = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let newNormal = currentNormal.clone().applyAxisAngle(rotationAxis, Math.PI / 2).normalize();

    let firstLegPosition = startMovingLeg.clone().add(step1).add(step2);        // First leg moves onto the wall

     this.interpolateMovement(startMovingLeg, firstLegPosition, currentNormal, newNormal, 'Concave',() => {
        this.swapFixedLeg();                                                    // Swap fixed leg after first move
        let step3 = movementVector.clone().multiplyScalar(1.5 * STEP_SIZE); 
        let step4 = currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE);  

        let secondLegPosition = startMovingLeg2.clone().add(step3).add(step4);  // Second leg moves onto the wall

        this.interpolateMovement(startMovingLeg2, secondLegPosition, currentNormal, newNormal,'ConcaveSwap', () => {
            this.swapFixedLeg();                                                // switch leg again
            if (resolve) resolve();
        });
    });
}

// ========================= TURN + CLIMB ========================


export function F101B101(resolve, step = 1) { // right
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .add(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .add(upStep)
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function F10m1B10m1(resolve, step = 1) { // left
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .sub(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .sub(upStep)
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function Fm101Bm101(resolve, step = 1) { // right
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .add(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);
    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .add(upStep)
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function Fm10m1Bm10m1(resolve, step = 1) { // left
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .sub(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);
    let secondLegEndPosition  =  startMovingLeg2.clone()
                                                .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                                .sub(upStep)
                                                .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function Fm10m1Bm10m1Half(resolve) {
    this.Fm10m1Bm10m1(resolve,0.5);
}
export function Fm101Bm101Half(resolve) {
    this.Fm101Bm101(resolve,0.5);
}
export function F10m1B10m1Half(resolve) {
    this.F10m1B10m1(resolve,0.5);
}
export function F101B101Half(resolve) {
    this.F101B101(resolve,0.5);
}

// ==== Four new movement functions ====
export function F0p5L1B1(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    const rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    
    const frontLegOffset = rotationVector.clone().multiplyScalar(STEP_SIZE).add(currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE));
    const backLegOffset = movementVector.clone().multiplyScalar(STEP_SIZE).add(currentNormal.clone().multiplyScalar(STEP_SIZE));

    const firstLegEndPosition = startMovingLeg.clone().add(frontLegOffset).multiplyScalar(10).round().divideScalar(10);
    const secondLegEndPosition = startMovingLeg2.clone().add(backLegOffset).multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function F0p5R1B1(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    const rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    
    const frontLegOffset = rotationVector.clone().multiplyScalar(STEP_SIZE).add(currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE));
    const backLegOffset = movementVector.clone().multiplyScalar(STEP_SIZE).add(currentNormal.clone().multiplyScalar(STEP_SIZE));

    const firstLegEndPosition = startMovingLeg.clone().add(frontLegOffset).multiplyScalar(10).round().divideScalar(10);
    const secondLegEndPosition = startMovingLeg2.clone().add(backLegOffset).multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function F1L1B0p5(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    const rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    
    const frontLegOffset = rotationVector.clone().multiplyScalar(STEP_SIZE).sub(currentNormal.clone().multiplyScalar(STEP_SIZE));
    const backLegOffset = movementVector.clone().multiplyScalar(STEP_SIZE).sub(currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE));

    const firstLegEndPosition = startMovingLeg.clone().add(frontLegOffset).multiplyScalar(10).round().divideScalar(10);
    const secondLegEndPosition = startMovingLeg2.clone().add(backLegOffset).multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function F1R1B0p5(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    const rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    
    const frontLegOffset = rotationVector.clone().multiplyScalar(STEP_SIZE).sub(currentNormal.clone().multiplyScalar(STEP_SIZE));
    const backLegOffset = movementVector.clone().multiplyScalar(STEP_SIZE).sub(currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE));

    const firstLegEndPosition = startMovingLeg.clone().add(frontLegOffset).multiplyScalar(10).round().divideScalar(10);
    const secondLegEndPosition = startMovingLeg2.clone().add(backLegOffset).multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function turnRightClimbUp(resolve,step = 1) { 
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .add(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function turnRightClimbDown(resolve,step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .sub(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function turnLeftClimbUp(resolve,step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .add(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);
    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function turnLeftClimbDown(resolve,step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .sub(upStep)
                                            .multiplyScalar(10).round().divideScalar(10);
    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .multiplyScalar(10).round().divideScalar(10);

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function turnRightClimbUpHalf(resolve) {
    this.turnRightClimbUp(resolve,0.5);
}

export function turnRightClimbDownHalf(resolve) {
    this.turnRightClimbDown(resolve,0.5);
}

export function turnLeftClimbUpHalf(resolve) {
    this.turnLeftClimbUp(resolve,0.5);
}

export function turnLeftClimbDownHalf(resolve) {
    this.turnLeftClimbDown(resolve,0.5);
}

// ========================= TURN + CLIMB BACK LEG FIRST ========================

export function turnRightClimbUpBack(resolve,step = 1) {    // reverse motion of turnRightClimbUp
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .add(upStep)
                                              .multiplyScalar(10).round().divideScalar(10);                                            

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function turnRightClimbDownBack(resolve,step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .sub(upStep)
                                              .multiplyScalar(10).round().divideScalar(10);                                            

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function turnLeftClimbUpBack(resolve,step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .add(upStep)
                                              .multiplyScalar(10).round().divideScalar(10);                                            

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function turnLeftClimbDownBack(resolve,step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
    let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

    let firstLegEndPosition = startMovingLeg.clone()
                                            .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
                                            .multiplyScalar(10).round().divideScalar(10);

    let secondLegEndPosition = startMovingLeg2.clone()
                                              .add(movementVector.clone().multiplyScalar(STEP_SIZE))
                                              .sub(upStep)
                                              .multiplyScalar(10).round().divideScalar(10);                                            

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function turnRightClimbUpHalfBack(resolve) {
    this.turnRightClimbUpBack(resolve,0.5);
}

export function turnRightClimbDownHalfBack(resolve) {
    this.turnRightClimbDownBack(resolve,0.5);
}

export function turnLeftClimbUpHalfBack(resolve) {
    this.turnLeftClimbUpBack(resolve,0.5);
}

export function turnLeftClimbDownHalfBack(resolve) {
    this.turnLeftClimbDownBack(resolve,0.5);
}
// ========================= NEW MOVEMENT ========================

// export function turnLeftClimbDownBackInvert(resolve,step = 1) {
//     const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
//     let rotationVector = new THREE.Vector3().crossVectors(currentNormal, movementVector).normalize();
//     let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

//     let firstLegEndPosition = startMovingLeg.clone()
//                                             .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
//                                             .sub(upStep)
//                                             .multiplyScalar(10).round().divideScalar(10);

//     let secondLegEndPosition = startMovingLeg2.clone()
//                                               .add(movementVector.clone().multiplyScalar(STEP_SIZE))
//                                               .sub(0.5*upStep)
//                                               .multiplyScalar(10).round().divideScalar(10);                                            

//     performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
// }

// export function turnRightClimbDownBackInvert(resolve,step = 1) {
//     const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
//     let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
//     let upStep = currentNormal.clone().multiplyScalar(step*STEP_SIZE);

//     let firstLegEndPosition = startMovingLeg.clone()
//                                             .add(rotationVector.clone().multiplyScalar(STEP_SIZE))
//                                             .sub(upStep)
//                                             .multiplyScalar(10).round().divideScalar(10);

//     let secondLegEndPosition = startMovingLeg2.clone()
//                                               .add(movementVector.clone().multiplyScalar(STEP_SIZE))
//                                               .sub(0.5*upStep)
//                                               .multiplyScalar(10).round().divideScalar(10);                                            

//     performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
// }

// export function turnRightClimbDownHalfBackInvert(resolve) {
//     this.turnRightClimbDownBackInvert(resolve,0.5);
// }
// export function turnLeftClimbDownHalfBackInvert(resolve) {
//     this.turnLeftClimbDownBackInvert(resolve,0.5);
// }
//+========================
export function climbUpFlatStair(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbUpFlatStairHalf(resolve) {
    this.climbUpFlatStair(resolve,0.5);
}

export function climbUpStairStair(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).add(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbUpStairStairHalf(resolve) {
    this.climbUpStairStair(resolve,0.5);
}

export function climbUpStairFlat(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).add(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbUpStairFlatHalf(resolve) {
    this.climbUpStairFlat(resolve,0.5);
}
//+++++
export function climbDownFlatStair(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbDownFlatStairHalf(resolve) {
    this.climbDownFlatStair(resolve,0.5);
}

export function climbDownStairStair(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).sub(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbDownStairStairHalf(resolve) {
    this.climbDownStairStair(resolve,0.5);
}

export function climbDownStairFlat(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).sub(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function climbDownStairFlatHalf(resolve) {
    this.climbDownStairFlat(resolve,0.5);
}
//++++
export function climbTopStair(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).add(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbTopStairHalf(resolve) {
    this.climbTopStair(resolve,0.5);
}
export function climbBottomStair(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                           // Move forward
    let step2 = currentNormal.clone().multiplyScalar(step * STEP_SIZE);                            // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).sub(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbBottomStairHalf(resolve) {
    this.climbBottomStair(resolve,0.5);
}
// +++
export function climbTopStairHalfFull(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).add(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbTopStairFullHalf(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).add(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbUpStairHalfFull(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).add(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbUpStairFullHalf(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).add(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
//+++
export function climbBottomStairFullHalf(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).sub(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbBottomStairHalfFull(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).sub(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbDownStairFullHalf(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).sub(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}
export function climbDownStairHalfFull(resolve) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);

    let step1 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step2 = currentNormal.clone().multiplyScalar(STEP_SIZE);                             
    let step3 = movementVector.clone().multiplyScalar(STEP_SIZE);                            
    let step4 = currentNormal.clone().multiplyScalar(0.5*STEP_SIZE);                             

    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step3).sub(step4)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves flat
    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}




// ==================== NON ESSENTIAL MOVEMENT ====================

export function sideStepUpRight(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();

    let step1 = rotationVector.clone().multiplyScalar(STEP_SIZE);                               // Move side way
    let step2 = currentNormal.clone().multiplyScalar(step*STEP_SIZE);                                // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).add(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function sideStepDownRight(resolve,step = 1 ) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();

    let step1 = rotationVector.clone().multiplyScalar(STEP_SIZE);                               // Move side way
    let step2 = currentNormal.clone().multiplyScalar(step*STEP_SIZE);                                // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).sub(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function sideStepUpLeft(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal,movementVector).normalize();

    let step1 = rotationVector.clone().multiplyScalar(STEP_SIZE);                               // Move side way
    let step2 = currentNormal.clone().multiplyScalar(step*STEP_SIZE);                                // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).add(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).add(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function sideStepDownLeft(resolve,step=1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal,movementVector).normalize();

    let step1 = rotationVector.clone().multiplyScalar(STEP_SIZE);                               // Move side way
    let step2 = currentNormal.clone().multiplyScalar(step*STEP_SIZE);                                // Move up 
    let firstLegEndPosition = startMovingLeg.clone().add(step1).sub(step2)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1).sub(step2)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function sideStepFlatRight(resolve,step = 1 ) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();

    let step1 = rotationVector.clone().multiplyScalar(step*STEP_SIZE);                               // Move side way
    let firstLegEndPosition = startMovingLeg.clone().add(step1)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function sideStepFlatLeft(resolve, step = 1) {
    const { startMovingLeg, startMovingLeg2, movementVector, currentNormal } = initMovement.call(this);
    let rotationVector = new THREE.Vector3().crossVectors(currentNormal,movementVector).normalize();

    let step1 = rotationVector.clone().multiplyScalar(step*STEP_SIZE);                               // Move side way
    let firstLegEndPosition = startMovingLeg.clone().add(step1)
                                            .multiplyScalar(10).round().divideScalar(10);       // First leg moves first
    let secondLegEndPosition = startMovingLeg2.clone().add(step1)
                                              .multiplyScalar(10).round().divideScalar(10);     // Then second leg moves 

    performLegMovement.call(this, startMovingLeg, firstLegEndPosition, startMovingLeg2, secondLegEndPosition, currentNormal, resolve);
}

export function sideStepFlatRightHalf(resolve) {
    this.sideStepFlatRight(resolve, 0.5);
}

export function sideStepFlatLeftHalf(resolve) {
    this.sideStepFlatLeft(resolve, 0.5);
}

// ======================== HALF MVT ========================

export function forwardHalf(resolve) {
    this.goForward(resolve, 0.5);
}
export function climbUpHalf(resolve) {
    this.climbUp(resolve,0.5);
}
export function climbDownHalf(resolve) {
    this.climbDown(resolve,0.5);
}
export function sideHalfUpRight(resolve) {
    this.sideStepUpRight(resolve, 0.5);
}
export function sideHalfDownRight(resolve) {
    this.sideStepDownRight(resolve, 0.5);
}
export function sideHalfUpLeft(resolve) {
    this.sideStepUpLeft(resolve, 0.5);
}
export function sideHalfDownLeft(resolve) {
    this.sideStepDownLeft(resolve, 0.5);
}

// ======================== PLUS THAN ONE TIME SCALED MVT ========================

export function forwardOneHalf(resolve) {
    this.goForward(resolve, 1.5);
}
export function forwardTwo(resolve) {
    this.goForward(resolve, 2);
}
export function climbUpFlatStairOneHalf(resolve) {
    this.climbUpFlatStair(resolve,1.5);
}
export function climbUpStairFlatOneHalf(resolve) {
    this.climbUpStairFlat(resolve,1.5);
}
export function climbDownFlatStairOneHalf(resolve) {
    this.climbDownFlatStair(resolve,1.5);
}
export function climbDownStairFlatOneHalf(resolve) {
    this.climbDownStairFlat(resolve,1.5);
}

// ==================== Pick Up Movement ====================

export function pickUpLeft(resolve){
    const { startMovingLeg, movementVector, currentNormal } = initMovement.call(this);
    const rotationVector = new THREE.Vector3().crossVectors(movementVector, currentNormal).normalize();
 
    const upStep = currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE);
    const leftStep = rotationVector.clone().negate().multiplyScalar(STEP_SIZE);
 
    const peakPosition = startMovingLeg.clone().add(upStep).add(leftStep).multiplyScalar(10).round().divideScalar(10);
 
    this.moveLegBezier(startMovingLeg, peakPosition, currentNormal, currentNormal, () => {
        this.moveLegBezier(peakPosition, startMovingLeg, currentNormal, currentNormal, () => {
            if (resolve) resolve();
        });
    });
}

export function pickUpRight(resolve){
    const { startMovingLeg2, currentNormal } = initMovement.call(this);

    const upStep = currentNormal.clone().multiplyScalar(0.5 * STEP_SIZE);
    const peakPosition = startMovingLeg2.clone().add(upStep).multiplyScalar(10).round().divideScalar(10);
    this.swapFixedLeg();

    this.moveLegBezier(startMovingLeg2, peakPosition, currentNormal, currentNormal, () => {
        this.moveLegBezier(peakPosition, startMovingLeg2, currentNormal, currentNormal, () => {
            this.swapFixedLeg();
            if (resolve) resolve();
        });
    });
}

// ==================== Placing Movement ====================

// === Auto‑generated voxel placement movements ===

export function placing_1l_1_1(resolve, step = 1) {
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI / 2, () => {
         resolve();
    }, step);
}
export function placing_1l_1_0_5(resolve) {this.placing_1l_1_1(resolve, 0.5);}
export function placing_1l_1_0(resolve) {this.placing_1l_1_1(resolve, 0);}
export function placing_1l_1_2(resolve) {this.placing_1l_1_1(resolve, 2);}

export function placing_1r_1_1(resolve, step = 1) {    
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI / 2, () => {
         resolve();
    }, step);
}
export function placing_1r_1_0_5(resolve) {this.placing_1r_1_1(resolve, 0.5);}
export function placing_1r_1_0(resolve) {this.placing_1r_1_1(resolve, 0);}
export function placing_1r_1_2(resolve) {this.placing_1r_1_1(resolve,2);}

export function placing_1l_2_1(resolve, step = 1) {    
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI, () => {
         resolve();
    }, step);
}
export function placing_1l_2_0_5(resolve) {this.placing_1l_2_1(resolve, 0.5);}
export function placing_1l_2_0(resolve) {this.placing_1l_2_1(resolve, 0);}
export function placing_1l_2_2(resolve) {this.placing_1l_2_1(resolve, 2);}

export function placing_1r_2_1(resolve, step = 1) {
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI, () => {
         resolve();
    }, step);
}
export function placing_1r_2_0_5(resolve) {this.placing_1r_2_1(resolve, 0.5);}
export function placing_1r_2_0(resolve) {this.placing_1r_2_1(resolve, 0);}
export function placing_1r_2_2(resolve) {this.placing_1r_2_1(resolve, 2);}

export function placing_2l_1(resolve, step = 1) {
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), 3/4*Math.PI, () => {
         resolve();
    }, step);
}
export function placing_2l_0_5(resolve) {this.placing_2l_1(resolve, 0.5);}
export function placing_2l_0(resolve) {this.placing_2l_1(resolve, 0);}
export function placing_2l_2(resolve) {this.placing_2l_1(resolve, 2);}

export function placing_2r_1(resolve, step = 1) {
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -3/4*Math.PI, () => {
         resolve();
    }, step);
}
export function placing_2r_0_5(resolve) {this.placing_2r_1(resolve, 0.5);}
export function placing_2r_0(resolve) {this.placing_2r_1(resolve, 0);}
export function placing_2r_2(resolve) {this.placing_2r_1(resolve, 2);}

export function placing_3l_1(resolve, step = 1) {    
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI / 2, () => {
         resolve();
    }, step);
}
export function placing_3l_0_5(resolve) {this.placing_3l_1(resolve, 0.5);}
export function placing_3l_0(resolve) {this.placing_3l_1(resolve, 0);}
export function placing_3l_2(resolve) {this.placing_3l_1(resolve, 2);}

export function placing_3r_1(resolve, step = 1) {
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI / 2, () => {
         resolve();
    }, step);
}
export function placing_3r_0_5(resolve) {this.placing_3r_1(resolve, 0.5);}
export function placing_3r_0(resolve) {this.placing_3r_1(resolve, 0);}
export function placing_3r_2(resolve) {this.placing_3r_1(resolve, 2);}

export function placing_3c_1(resolve, step = 1) {
        this.swapFixedLeg();
        this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI, () => {
            resolve();
        }, step);
}
export function placing_3c_0_5(resolve) {this.placing_3c_1(resolve, 0.5);}
export function placing_3c_0(resolve) {this.placing_3c_1(resolve, 0);}
export function placing_3c_2(resolve) {this.placing_3c_1(resolve, 2);}

export function placing_4l_1(resolve, step = 1) {    
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI / 4, () => {
         resolve();
    }, step);
}
export function placing_4l_0_5(resolve) {this.placing_4l_1(resolve, 0.5)}
export function placing_4l_0(resolve) {this.placing_4l_1(resolve, 0)}
export function placing_4l_2(resolve) {this.placing_4l_1(resolve, 2)}
export function placing_4r_1(resolve, step = 1) {
    this.swapFixedLeg();
    this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI / 4, () => {
         resolve();
    }, step);
}
export function placing_4r_0_5(resolve) {this.placing_4r_1(resolve, 0.5);}
export function placing_4r_0(resolve) {this.placing_4r_1(resolve, 0);}
export function placing_4r_2(resolve) {this.placing_4r_1(resolve, 2);}


// INVERSE MVT ==========================================================


export function placing_r_1l_1_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI / 2, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_1l_1_0_5(resolve) {this.placing_r_1l_1_1(resolve, 0.5);}
export function placing_r_1l_1_0(resolve) {this.placing_r_1l_1_1(resolve, 0);}
export function placing_r_1l_1_2(resolve) {this.placing_r_1l_1_1(resolve, 2);}

export function placing_r_1r_1_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI / 2, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_1r_1_0_5(resolve) {this.placing_r_1r_1_1(resolve, 0.5);}
export function placing_r_1r_1_0(resolve) {this.placing_r_1r_1_1(resolve, 0);}
export function placing_r_1r_1_2(resolve) {this.placing_r_1r_1_1(resolve, 2);}


export function placing_r_1l_2_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_1l_2_0_5(resolve) {this.placing_r_1l_2_1(resolve, 0.5);}
export function placing_r_1l_2_0(resolve) {this.placing_r_1l_2_1(resolve, 0);}
export function placing_r_1l_2_2(resolve) {this.placing_r_1l_2_1(resolve, 2);}

export function placing_r_1r_2_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_1r_2_0_5(resolve) {this.placing_r_1r_2_1(resolve, 0.5);}
export function placing_r_1r_2_0(resolve) {this.placing_r_1r_2_1(resolve, 0);}
export function placing_r_1r_2_2(resolve) {this.placing_r_1r_2_1(resolve, 2);}

export function placing_r_2l_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -3/4*Math.PI, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_2l_0_5(resolve) {this.placing_r_2l_1(resolve, 0.5);}
export function placing_r_2l_0(resolve) {this.placing_r_2l_1(resolve, 0);}
export function placing_r_2l_2(resolve) {this.placing_r_2l_1(resolve, 2);}

export function placing_r_2r_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), 3/4*Math.PI, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_2r_0_5(resolve) {this.placing_r_2r_1(resolve, 0.5);}
export function placing_r_2r_0(resolve) {this.placing_r_2r_1(resolve, 0);}
export function placing_r_2r_2(resolve) {this.placing_r_2r_1(resolve, 2);}

export function placing_r_3l_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI / 2, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_3l_0_5(resolve) {this.placing_r_3l_1(resolve, 0.5);}
export function placing_r_3l_0(resolve) {this.placing_r_3l_1(resolve, 0);}
export function placing_r_3l_2(resolve) {this.placing_r_3l_1(resolve, 2);}

export function placing_r_3r_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI / 2, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_3r_0_5(resolve) {this.placing_r_3r_1(resolve, 0.5);}
export function placing_r_3r_0(resolve) {this.placing_r_3r_1(resolve, 0);}
export function placing_r_3r_2(resolve) {this.placing_r_3r_1(resolve, 2);}

export function placing_r_3c_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_3c_0_5(resolve) {this.placing_r_3c_1(resolve, 0.5);}
export function placing_r_3c_0(resolve) {this.placing_r_3c_1(resolve, 0);}
export function placing_r_3c_2(resolve) {this.placing_r_3c_1(resolve, 2);}

export function placing_r_4l_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), -Math.PI / 4, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_4l_0_5(resolve) {this.placing_r_4l_1(resolve, 0.5)}
export function placing_r_4l_0(resolve) {this.placing_r_4l_1(resolve, 0)}
export function placing_r_4l_2(resolve) {this.placing_r_4l_1(resolve, 2)}

export function placing_r_4r_1(resolve, step = 1) {
     this.rotateMovingLegWithZOffset(this.origin.normal.clone(), Math.PI / 4, () => {
        this.swapFixedLeg();
        resolve();
    }, -step);
}
export function placing_r_4r_0_5(resolve) {this.placing_r_4r_1(resolve, 0.5);}
export function placing_r_4r_0(resolve) {this.placing_r_4r_1(resolve, 0);}
export function placing_r_4r_2(resolve) {this.placing_r_4r_1(resolve, 2);}

// ======================== HELPER FUNCTION ========================

function initMovement() {
    // Initialize common variables for a movement
    const startMovingLeg = this.target.position.clone();
    const startMovingLeg2 = this.origin.position.clone();
    const movementVector = this.calculateMovementVector();
    const currentNormal = this.origin.normal.clone();
    return { startMovingLeg, startMovingLeg2, movementVector, currentNormal };
}

function performLegMovement(firstLegStart, firstLegEnd, secondLegStart, secondLegEnd, newNormal, resolve) {
    this.moveLegBezier(firstLegStart, firstLegEnd, this.target.normal, newNormal, () => {
        this.swapFixedLeg();
        this.moveLegBezier(secondLegStart, secondLegEnd, this.origin.normal, newNormal, () => {
            this.swapFixedLeg();
            if (resolve) resolve();
        });
    });
}

export function calculateMovementVector() {
    let rawVector = new THREE.Vector3().subVectors(this.target.position, this.origin.position);
    let surfaceNormal = this.origin.normal.clone().normalize();

    // Project movement vector onto the plane defined by the surface normal
    let dot = rawVector.dot(surfaceNormal);
    let projection = surfaceNormal.multiplyScalar(dot);
    let movementVector = rawVector.sub(projection).normalize();

    return movementVector;
}

// ======================== INTERPOLATING FUNCTION ========================


export function moveLegBezier(startPos, endPos, startNormal, endNormal, resolve = () => {}) {
    let stepIndex = 0;
    let timeoutID;

    // Define Bezier Control Points for a Smooth Arc
    let control1 = startPos.clone().lerp(endPos, 0.33).add(startNormal.clone().multiplyScalar(STEP_SIZE * 0.5));
    let control2 = startPos.clone().lerp(endPos, 0.66).add(endNormal.clone().multiplyScalar(STEP_SIZE * 0.5));

    const step = () => {
        if (stepIndex <= INTERMEDIARY_STEPS) {

            let t = (1 - Math.cos((stepIndex / INTERMEDIARY_STEPS) * Math.PI)) / 2; 

            let interpolatedPos = cubicBezier(startPos, control1, control2, endPos, t);
            let interpolatedNormal = startNormal.clone().lerp(endNormal, t).normalize();
            this.setToTargetIK(interpolatedPos.x, interpolatedPos.y, interpolatedPos.z, 
                             interpolatedNormal.x, interpolatedNormal.y, interpolatedNormal.z);

            // this.displayTrajectory();
            stepIndex++;

            clearTimeout(timeoutID);
            timeoutID = setTimeout(step, DELAY);
        } else { // ensure final pos is reached 
            this.setToTargetIK(endPos.x, endPos.y, endPos.z, endNormal.x, endNormal.y, endNormal.z);
            // this.displayTrajectory()

            if (resolve) resolve();
        }
    };
    step();
}

export function interpolateMovement(startPos, endPos, startNormal, endNormal, transitionType, callback) {
    let stepIndex = 0;

    const step = () => {
        if (stepIndex <= INTERMEDIARY_STEPS) {

            let t = stepIndex / INTERMEDIARY_STEPS;
            let interpolatedPos = startPos.clone().lerp(endPos, t);
            let interpolatedNormal = startNormal.clone().lerp(endNormal, t).normalize();

            this.setToTargetIK(interpolatedPos.x, interpolatedPos.y, interpolatedPos.z, 
                             interpolatedNormal.x, interpolatedNormal.y, interpolatedNormal.z, transitionType);
            
            // this.displayTrajectory()

            stepIndex++;
            setTimeout(step, DELAY);
        } else {
            if (callback) callback();
        }
    };
    step();
}
function cubicBezier(p0, p1, p2, p3, t) {
    const pA = tmpVec1.copy(p0).lerp(p1, t);
    const pB = tmpVec2.copy(p1).lerp(p2, t);
    const pC = tmpVec3.copy(p2).lerp(p3, t);
    const pD = tmpVec4.copy(pA).lerp(pB, t);
    const pE = tmpVec5.copy(pB).lerp(pC, t);
    return tmpVec1.copy(pD).lerp(pE, t);
}

// ===================== VISUALIZATION FUNCTION =====================

export function displayTrajectory() {
    if (!this.showTrajectory) return;     
    let markerGeometry = new THREE.SphereGeometry(0.1, 10, 10);
    let markerMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });

    let marker = new THREE.Mesh(markerGeometry, markerMaterial);
    marker.position.copy(this.target.position);
    this.scene.add(marker);
    this.trajectoryPoints.push(marker);  
}

export function clearTrajectory() {
    this.trajectoryPoints.forEach(point => this.scene.remove(point));
    this.trajectoryPoints = [];
}


// ================= TEST MOVEMENT ( NOT NECESSARY ) ==================

export function halfturn(resolve) {
    let rotationAxis = this.origin.normal.clone();

    this.rotateMovingLeg(rotationAxis, Math.PI, resolve);
}
export function rotateMovingLeg(rotationAxis, angleOffset, resolve = () => {}, zOffset = 0) {
    // +Math.PI turn left
    // -Math.PI turn right 
    let startMovingLeg = this.target.position.clone();
    let fixedLeg = this.origin.position.clone();

    let relativeStart = startMovingLeg.clone().sub(fixedLeg);
    let angleStep = angleOffset / INTERMEDIARY_STEPS;

    let stepIndex = 0;

    const step = () => {
        if (stepIndex <= INTERMEDIARY_STEPS) {
            let angle = stepIndex * angleStep;
            let quaternion = new THREE.Quaternion().setFromAxisAngle(rotationAxis, angle);
            let rotatedVector = relativeStart.clone().applyQuaternion(quaternion);
            let interpolatedMovingLeg = fixedLeg.clone().add(rotatedVector);

            // Smooth elevation arc
            let lift = Math.sin(Math.PI * (stepIndex / INTERMEDIARY_STEPS)) * (STEP_SIZE / 3);
            interpolatedMovingLeg.add(rotationAxis.clone().normalize().multiplyScalar(lift));

            if (stepIndex === INTERMEDIARY_STEPS) {
                interpolatedMovingLeg.z += zOffset;
            }

            this.setToTargetIK(
                interpolatedMovingLeg.x, interpolatedMovingLeg.y, interpolatedMovingLeg.z,
                this.target.normal.x, this.target.normal.y, this.target.normal.z
            );

            stepIndex++;
            setTimeout(step, DELAY);
        } else {
            if (resolve) resolve();
        }
    };
    step();
}
// Like rotateMovingLeg, but smoothly interpolates zOffset throughout the rotation
export function rotateMovingLegWithZOffset(rotationAxis, angleOffset, resolve = () => {}, zOffset = 0) {
    let startMovingLeg = this.target.position.clone();
    let fixedLeg = this.origin.position.clone();

    let relativeStart = startMovingLeg.clone().sub(fixedLeg);
    let angleStep = angleOffset / INTERMEDIARY_STEPS;

    let stepIndex = 0;

    const step = () => {
        if (stepIndex <= INTERMEDIARY_STEPS) {
            let angle = stepIndex * angleStep;
            let quaternion = new THREE.Quaternion().setFromAxisAngle(rotationAxis, angle);
            let rotatedVector = relativeStart.clone().applyQuaternion(quaternion);
            let interpolatedMovingLeg = fixedLeg.clone().add(rotatedVector);

            // Smooth elevation arc with Z offset distributed over all steps
            let zLift = zOffset * (stepIndex / INTERMEDIARY_STEPS);
            interpolatedMovingLeg.z += zLift;

            this.setToTargetIK(
                interpolatedMovingLeg.x, interpolatedMovingLeg.y, interpolatedMovingLeg.z,
                this.target.normal.x, this.target.normal.y, this.target.normal.z
            );

            stepIndex++;
            setTimeout(step, DELAY);
        } else {
            if (resolve) resolve();
        }
    };
    step();
}