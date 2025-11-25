import * as THREE from "../../three/build/three.module.min.js";

export function getPossibleActions(current, STEP_SIZE, voxelExists, filterAction = null) {
    let actions = [
        {
            action: "switchLeg",
            moveFront: new THREE.Vector3(0,0,0), // switchleg is a simulation specific movement and each serie of movement end with an even nb thus we just update direction
            moveBack: new THREE.Vector3(0,0,0),
            newDir: current.forward.clone().negate(),
            newNorm: current.normal.clone()
        },
        {
            action: "goForward",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "forwardHalf",
            moveFront: current.forward.clone().multiplyScalar(0.5 * STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(0.5 * STEP_SIZE),
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "turnRight",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE), 
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),  
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeft",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE), 
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),  
            newNorm: current.normal.clone()
        },
        {
            action: "climbUpFlatStair",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbUpFlatStairHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbUpStairStair",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbUpStairStairHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbUpStairFlat",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbUpStairFlatHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbDownFlatStair",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),  
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbDownFlatStairHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbDownStairStair",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbDownStairStairHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbDownStairFlat",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbDownStairFlatHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbTopStair",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },           
        {
            action: "climbTopStairHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },            
        {
            action: "climbBottomStair",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },            
        {
            action: "climbBottomStairHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },

        //+++
        {
            action: "climbTopStairHalfFull",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },           
        {
            action: "climbTopStairFullHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },            
        {
            action: "climbUpStairHalfFull",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },           
        {
            action: "climbUpStairFullHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },            
        {
            action: "climbBottomStairHalfFull",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },            
        {
            action: "climbBottomStairFullHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {
            action: "climbDownStairHalfFull",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },            
        {
            action: "climbDownStairFullHalf",
            moveFront: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5*STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)), 
            newDir: current.forward.clone(),
            newNorm: current.normal.clone(),
        },
        {   
            action: "turnRightClimbUp",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE).add(current.normal.clone()),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnRightClimbDown",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().sub(current.normal.clone()).multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbUp",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().add(current.normal.clone()).multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbDown",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().sub(current.normal.clone()).multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnRightClimbUpHalf",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().add((current.normal.clone()).multiplyScalar(0.5 * STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnRightClimbDownHalf",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().sub((current.normal.clone()).multiplyScalar(0.5 * STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbUpHalf",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().add((current.normal.clone()).multiplyScalar(0.5 * STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbDownHalf",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().sub((current.normal.clone()).multiplyScalar(0.5 * STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        // side step flat
        {
            action: "turnRightClimbUpBack",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone()),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnRightClimbDownBack",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone()),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbUpBack",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone()),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbDownBack",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone()),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnRightClimbUpHalfBack",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnRightClimbDownHalfBack",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbUpHalfBack",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "turnLeftClimbDownHalfBack",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "sideStepFlatRight",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE),
            moveBack: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE),
            newDir: current.forward.clone(),  
            newNorm: current.normal.clone()
        },
        {
            action: "sideStepFlatLeft",
            moveFront: new THREE.Vector3().crossVectors(current.normal,current.forward).normalize().multiplyScalar(STEP_SIZE),
            moveBack: new THREE.Vector3().crossVectors(current.normal,current.forward).normalize().multiplyScalar(STEP_SIZE),
            newDir: current.forward.clone(),  
            newNorm: current.normal.clone()
        },
        {
            action: "sideStepFlatRightHalf",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(0.5*STEP_SIZE),
            moveBack: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(0.5*STEP_SIZE),
            newDir: current.forward.clone(),  
            newNorm: current.normal.clone()
        },
        {
            action: "sideStepFlatLeftHalf",
            moveFront: new THREE.Vector3().crossVectors(current.normal,current.forward).normalize().multiplyScalar(0.5*STEP_SIZE),
            moveBack: new THREE.Vector3().crossVectors(current.normal,current.forward).normalize().multiplyScalar(0.5*STEP_SIZE),
            newDir: current.forward.clone(),  
            newNorm: current.normal.clone()
        },
        // stairs for the following namings let's suppose robot is facing y direction 
        // and F represent relative mvt of front leg and B represent relative mvt of back leg
        {   // front leg Up and Right back leg forward and Up
            action: "F101B101",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE).add(current.normal.clone()),
            moveBack: current.forward.clone().add(current.normal.clone()).multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "F10m1B10m1",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().sub(current.normal.clone()).multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().sub(current.normal.clone()).multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "Fm101Bm101",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().add(current.normal.clone()).multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().add(current.normal.clone()).multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "Fm10m1Bm10m1",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().sub(current.normal.clone()).multiplyScalar(STEP_SIZE),
            moveBack: current.forward.clone().sub(current.normal.clone()).multiplyScalar(STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {   // front leg Up and Right back leg forward and Up
            action: "F101B101Half",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(0.5*STEP_SIZE).add(current.normal.clone()),
            moveBack: current.forward.clone().add(current.normal.clone()).multiplyScalar(0.5*STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "F10m1B10m1Half",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().sub(current.normal.clone()).multiplyScalar(0.5*STEP_SIZE),
            moveBack: current.forward.clone().sub(current.normal.clone()).multiplyScalar(0.5*STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "Fm101Bm101Half",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().add(current.normal.clone()).multiplyScalar(0.5*STEP_SIZE),
            moveBack: current.forward.clone().add(current.normal.clone()).multiplyScalar(0.5*STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "Fm10m1Bm10m1Half",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().sub(current.normal.clone()).multiplyScalar(0.5*STEP_SIZE),
            moveBack: current.forward.clone().sub(current.normal.clone()).multiplyScalar(0.5*STEP_SIZE),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },

        {
            action: "F0p5L1B1",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "F0p5R1B1",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).add(current.normal.clone().multiplyScalar(STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "F1L1B0p5",
            moveFront: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.normal, current.forward).normalize(),
            newNorm: current.normal.clone()
        },
        {
            action: "F1R1B0p5",
            moveFront: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(STEP_SIZE)),
            moveBack: current.forward.clone().multiplyScalar(STEP_SIZE).sub(current.normal.clone().multiplyScalar(0.5 * STEP_SIZE)),
            newDir: new THREE.Vector3().crossVectors(current.forward, current.normal).normalize(),
            newNorm: current.normal.clone()
        },
    ];
    if (filterAction) {
        return actions.filter(a => a.action === filterAction);
    }
    return actions;
}

