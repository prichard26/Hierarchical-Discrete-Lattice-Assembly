import * as THREE from "../../three/build/three.module.min.js";
import { getPossibleActions } from './actions.js';


const LOG = false;
const WARN = false;

let GLOBAL = 0;
let LAST_SOURCE_SWITCH = 0;

const round = n => Math.round(n * 1000) / 1000;

const keyCache = new Map();
const toKeyCached = (v) => {
    const k = `${round(v.x)},${round(v.y)},${round(v.z)}`;
    if (!keyCache.has(k)) keyCache.set(k, k);
    return keyCache.get(k);
};

// Removed rebuildVoxelKeySet function as window.voxelCenters is maintained live.

class PriorityQueue { // may be optimized 
    constructor() {
        this.items = [];
    }

    enqueue(element, priority) {
        const node = { element, priority };
        let added = false;
        for (let i = 0; i < this.items.length; i++) {
            if (this.items[i].priority > node.priority) {
                this.items.splice(i, 0, node);
                added = true;
                break;
            }
        }
        if (!added) this.items.push(node);
    }
    dequeue() {
        return this.items.shift()?.element;
    }
    get length() {
        return this.items.length;
    }
}
export async function planPathToLegPositions({
  frontLegGoal,
  backLegGoal = null,
  normalGoal,
  voxelToAvoid = undefined,
  requireParity = true,
  start = null, 
}) {
    // Use BacklegGoal to constraint an orientation
    // Use VoxelToAvoid to avoid having backleg on the voxel you're ƒgonna place
    const STEP_SIZE = 1.0;
    const HEURISTIC_WEIGHT = 1.0; // Set to 1.0 for strict A*, <1.0 for more exploration

    const pathLogMap = new Map(); // key => path[]

    // Adjust to voxel center
    if (!start) {
        frontLegGoal = frontLegGoal.clone().add(normalGoal.clone().multiplyScalar(0.5));
        if (backLegGoal) backLegGoal = backLegGoal.clone().add(normalGoal.clone().multiplyScalar(0.5));
    }
    // Check goal validity
    if (!voxelExists(frontLegGoal, normalGoal)) return {failed:console.log("⚠️voxel does not Exist at",frontLegGoal,normalGoal), success: false, path: [] };
    if (backLegGoal && !voxelExists(backLegGoal, normalGoal)) return {failed:console.log("⚠️voxelExists2"), success: false, path: [] };
    if (!isClearAbove(frontLegGoal, normalGoal)) return {failed:console.log("⚠️clearAbove"), success: false, path: [] };

    const serialize = (front, back, normal, forward, parity) =>
        ` ${round(front.x)},${round(front.y)},${round(front.z)}
        |${round(back.x)},${round(back.y)},${round(back.z)}
        |${round(normal.x)},${round(normal.y)},${round(normal.z)}
        |${round(forward.x)},${round(forward.y)},${round(forward.z)}|${parity}`;

    const heuristic = (pos) => {
        return Math.abs(pos.x - frontLegGoal.x) +
               Math.abs(pos.y - frontLegGoal.y) + 
               Math.abs(pos.z - frontLegGoal.z);
    };

    const openSet = new PriorityQueue();
    const initialFront = start?.frontLeg?.clone() ?? this.target.position.clone();
    const initialBack = start?.backLeg?.clone() ?? this.origin.position.clone();
    const initialNormal = start?.normal?.clone() ?? this.target.normal.clone();
    const initialForward = start?.forward?.clone() ?? this.calculateMovementVector();
    // if(start)console.warn("Using start position:", initialFront, initialBack, initialNormal, initialForward);
    openSet.enqueue({
        frontLeg: initialFront,
        backLeg: initialBack,
        normal: initialNormal,
        forward: initialForward,
        path: [],
        key: null,
        legSwitchCount: 0
    }, heuristic(initialFront));

    const gScoreMap = new Map();
    const fScoreMap = new Map();
    const visited = new Set();
    const enableStats = typeof window !== "undefined" && window._enableBenchmark;

    openSet.items[0].element.key = serialize(openSet.items[0].element.frontLeg, openSet.items[0].element.backLeg, openSet.items[0].element.normal, openSet.items[0].element.forward, 0);
    gScoreMap.set(openSet.items[0].element.key, 0);
    fScoreMap.set(openSet.items[0].element.key, heuristic(openSet.items[0].element.frontLeg));

    const applyParityWrapper = (move, paritySwitched) => {
        return {
            moveFront: paritySwitched ? move.moveBack.clone() : move.moveFront.clone(),
            moveBack: paritySwitched ? move.moveFront.clone() : move.moveBack.clone(),
            newDir: move.newDir.clone(),
            newNorm: move.newNorm.clone(),
            action: move.action,
            condition: move.condition
        };
    };

    while (openSet.length > 0) {
        if (enableStats) {
            window._benchmark.computation.loopIterations++;
            window._benchmark.computation.maxOpenSetSize = Math.max(
                window._benchmark.computation.maxOpenSetSize,
                openSet.length
            );
        }
        const current = openSet.dequeue();
        if (LOG) console.log(
          `Exploring node - frontLeg: ${current.frontLeg.x},${current.frontLeg.y},${current.frontLeg.z} | backLeg: ${current.backLeg.x},${current.backLeg.y},${current.backLeg.z}`
        );
        if (WARN && current?.path.length) {
            const lastAction = current.path[current.path.length - 1];
            console.log(" A* exploring via:", lastAction.action);
            const pathSummary = current.path.map(p => p.action).join(" → ");
            console.log(" Current path trace:", pathSummary);
        }
        if (!current || visited.has(current.key)) continue;
        visited.add(current.key);
        if (WARN) pathLogMap.set(current.key, current.path);

        if (visited.size > 10000) {
            console.warn("Pathfinding aborted: visited limit exceeded.");
            return { success: false, path: [], visitedNodes: visited.size };
        }

        const totalSwitchCount = GLOBAL + current.path.filter(p => p.action === "switchLeg").length;    // total nb of switch during the research 
        const paritySwitched = (totalSwitchCount - LAST_SOURCE_SWITCH) % 2 === 1;

        const physicalFrontLegPos = paritySwitched ? current.backLeg : current.frontLeg;
        const physicalBackLegPos  = paritySwitched ? current.frontLeg : current.backLeg;

        // Integrate isFootOnTarget logic into backOK, but allow for continuing path exploration if invalid
        const frontOK = physicalFrontLegPos.distanceTo(frontLegGoal) < 0.01;
        const backTooCloseToAvoid = voxelToAvoid && physicalBackLegPos.distanceTo(voxelToAvoid) < 0.01;
        // console.warn("voxelToAvoid", voxelToAvoid, "physicalBackLegPos", physicalBackLegPos, "distance", physicalBackLegPos.distanceTo(voxelToAvoid));
        const backOK = (!backLegGoal || physicalBackLegPos.distanceTo(backLegGoal) < 0.01)
                    && !backTooCloseToAvoid;
        const normalOK = current.normal.angleTo(normalGoal) < 0.1;

        if (frontOK && normalOK && backOK) {
            // console.warn("physicalFrontLegPos", physicalFrontLegPos, "physicalBackLegPos", physicalBackLegPos);
            //  console.warn("voxelToAvoid",voxelToAvoid)
            if (backTooCloseToAvoid) {
                if (LOG) console.warn("⚠️ Final state has back leg on voxelToAvoid — not accepting, but continuing path exploration.");
                continue;
            }

            const pathSwitchCount = current.path.filter(p => p.action === "switchLeg").length;
            const globalSwitchAfter = GLOBAL + pathSwitchCount;
            const parityBefore = LAST_SOURCE_SWITCH % 2;
            const parityAfter = globalSwitchAfter % 2;

            const parityMatches = parityBefore === parityAfter;

            if (requireParity && !parityMatches) {
                if (LOG) console.warn("Reached spatial goal but with wrong physical leg parity. Continuing to search for correct parity...");
            } else {
                GLOBAL = globalSwitchAfter;
                LAST_SOURCE_SWITCH = GLOBAL;
                const path = current.path;
                return { success: true, path, visitedNodes: visited.size };
            }
        }

        const actions = getPossibleActions(current, STEP_SIZE, voxelExists);
        for (const move of actions) {
            // if (LOG) console.log("🔍 Checking action:", move.action);
            const interpretedMove = applyParityWrapper(move, paritySwitched);
            const newFront = current.frontLeg.clone().add(interpretedMove.moveFront);
            const newBack = current.backLeg.clone().add(interpretedMove.moveBack);
            // if (interpretedMove.action === "columnDown") {
            //   console.log(
            //     `columnUp result: frontLeg ${newFront.x},${newFront.y},${newFront.z} | backLeg ${newBack.x},${newBack.y},${newBack.z}`
            //   );
            // }
            const newNormal = interpretedMove.newNorm;
            const newForward = interpretedMove.newDir;
            let newSwitchCount = current.legSwitchCount;

            if (move.action === "switchLeg") {newSwitchCount++;}

            if (
                !voxelExists(newFront, newNormal)     ||
                !voxelExists(newBack, newNormal)      ||
                !isClearAbove(newFront, newNormal)    ||
                !isClearAbove(newBack, newNormal)
            ) continue;

            let cost = 1;

            // Penalize change in direction
            if (!current.forward.equals(interpretedMove.newDir)) {
                cost += 0;
            }

            const newKey = serialize(newFront, newBack, newNormal, newForward, newSwitchCount % 2);
            const tentativeG = gScoreMap.get(current.key) + cost;
            const estimatedF = tentativeG + HEURISTIC_WEIGHT * heuristic(newFront);
            
            if (!gScoreMap.has(newKey) || tentativeG < gScoreMap.get(newKey)) {
                gScoreMap.set(newKey, tentativeG);
                fScoreMap.set(newKey, estimatedF);
            
                if (LOG) console.log("Adding valid action to openSet:", move.action);
                openSet.enqueue({
                    frontLeg: newFront,
                    backLeg: newBack,
                    normal: newNormal,
                    forward: newForward,
                    path: [...current.path, {
                        frontLeg: newFront.clone(),
                        backLeg: newBack.clone(),
                        normal: newNormal.clone(),
                        forward: newForward,
                        action: move.action
                    }],
                    key: newKey,
                    legSwitchCount: newSwitchCount
                }, estimatedF);
            }
        }
    }
    console.log("Pathfinding failed. Final visited size:", visited.size);
    if (WARN) {
        console.log("Explored action paths:");
        for (const [key, path] of pathLogMap.entries()) {
            if (path.length) {
                const pathStr = path.map(p => p.action).join(" → ");
                console.log(` • Path [${key.split("|")[4]}]: ${pathStr}`);
            }
        }
    }
    return { success: false, path: [], visitedNodes: visited.size };
}

export function voxelExists(position, normal, checkAbove = false) { // add if true
    const STEP = 0.5;
    if (window._enableBenchmark) window._benchmark.computation.voxelExistsCalls++;

    const adjusted = position.clone().sub(normal.clone().multiplyScalar(STEP));
    const adjustedKey = toKeyCached(adjusted);
    const normalKey = toKeyCached(normal);
    const exists = key => window.voxelCenters?.has(key);

    const checkDoubleSupport = (offsetVec) => {
        const key1 = toKeyCached(adjusted.clone().add(offsetVec));
        const key2 = toKeyCached(adjusted.clone().sub(offsetVec));
        return exists(key1) && exists(key2);
    };

    const hasSufficientSupport = () => {
        const offsets = [
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0.5, 0, 0),
            new THREE.Vector3(-0.5, 0, 0),
            new THREE.Vector3(0, 0.5, 0),
            new THREE.Vector3(0, -0.5, 0),
            new THREE.Vector3(0.5, 0.5, 0),
            new THREE.Vector3(-0.5, 0.5, 0),
            new THREE.Vector3(0.5, -0.5, 0),
            new THREE.Vector3(-0.5, -0.5, 0),
        ];
        let supportCount = 0;
        for (const offset of offsets) {
            const key = toKeyCached(adjusted.clone().add(offset));
            if (exists(key)) supportCount++;
        }
        return supportCount >= 4;
    };
   
    const getOrthogonalVectors = (normal) => {
        const up = Math.abs(normal.z) > 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 0, 1);
        const tangent1 = new THREE.Vector3().crossVectors(normal, up).normalize();
        const tangent2 = new THREE.Vector3().crossVectors(normal, tangent1).normalize();
        return [tangent1, tangent2];
    };

    if (normalKey !== "0,0,1" && normalKey !== "0,0,-1") {
        const [t1, t2] = getOrthogonalVectors(normal);
        const wallOffsets = [
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0.5, 0, 0),
            new THREE.Vector3(-0.5, 0, 0),
            new THREE.Vector3(0, 0.5, 0),
            new THREE.Vector3(0, -0.5, 0),
            new THREE.Vector3(0.5, 0.5, 0),
            new THREE.Vector3(-0.5, 0.5, 0),
            new THREE.Vector3(0.5, -0.5, 0),
            new THREE.Vector3(-0.5, -0.5, 0),
        ];

        // NEW: direct voxel test
        if (exists(toKeyCached(adjusted))) {
            // if (LOG) console.log("Standing exactly on center voxel:", adjusted.toArray());
            return true;
        }

        let supportCount = 0;
        for (const offset of wallOffsets) {
            const key = toKeyCached(adjusted.clone().add(offset));
            if (exists(key)) supportCount++;
        }
        return supportCount >= 3;
    }
    if(checkAbove){
        return exists(adjustedKey);
    } else {
        return exists(adjustedKey)
            || checkDoubleSupport(new THREE.Vector3(0.5, 0, 0))
            || checkDoubleSupport(new THREE.Vector3(0, 0.5, 0))
            || hasSufficientSupport();
    }
}

export function isClearAbove(position, normal) {
    const STEP_SIZE = 1.0;
    const normalKey = toKeyCached(normal);
    
    const getOrthogonalVectors = (normal) => {
        const up = Math.abs(normal.z) > 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 0, 1);
        const tangent1 = new THREE.Vector3().crossVectors(normal, up).normalize();
        const tangent2 = new THREE.Vector3().crossVectors(normal, tangent1).normalize();
        return [tangent1, tangent2];
    };

    const [tangent1, tangent2] = getOrthogonalVectors(normal);
    const offsets = [
        new THREE.Vector3(0, 0, 0),
        tangent1.clone().multiplyScalar(0.5),
        tangent1.clone().multiplyScalar(-0.5),
        tangent2.clone().multiplyScalar(0.5),
        tangent2.clone().multiplyScalar(-0.5),
        tangent1.clone().add(tangent2).multiplyScalar(0.5),
        tangent1.clone().sub(tangent2).multiplyScalar(0.5),
        tangent2.clone().sub(tangent1).multiplyScalar(0.5)
    ];

    for (let i = 0.5; i <= 2; i += 0.5) {
        const spaceAbove = position.clone().add(normal.clone().multiplyScalar(i * STEP_SIZE));
        for (const offset of offsets) {
            const checkPos = spaceAbove.clone().add(offset);
            const key = toKeyCached(checkPos);
            if (voxelExists(checkPos, normal, true)) {
                // if(LOG)console.log("position blocked ",position)
                // if(LOG)console.log(`⛔ Block above at: ${key},caused by offset: ${offset.x}, ${offset.y}, ${offset.z}`);
                return false;
            }
        }
    }
    return true;
}

