// ==================== GLOBAL UTILITY: WebSocket Connection Check ====================
// Returns true if the lainSocket WebSocket connection is open and ready.
window.lainConnected = () => {
    return window.lainSocket && window.lainSocket.readyState === WebSocket.OPEN;
};
import * as THREE from "./three/build/three.module.min.js";
import { GUI } from "./three/examples/jsm/libs/lil-gui.module.min.js";
import { Source} from "./voxels/source.js";
import { THREERobot } from "./robots/core/robot.js";
import { Voxel } from "./voxels/voxel.js";
import { BuildingSequencePlanner } from "./planners/building_sequence.js";
import { BuildingManager } from "./robots/manager/robot_manager2.js";
import { setSpeedTuning } from "./robots/core/robot_movement.js";   
import { getPossibleActions } from "./robots/core/actions.js";

// TO DO 
// remove hitbox when deleting structure
// when adding a structure add voxel in the right type not as now ... 



// ==================== DATA MANAGEMENT OVERVIEW ====================
//
// VOXELS:
//   - Placed voxels are stored globally in:       window.voxelMap (Set)
//   - To-be-built voxels are stored in:           this.voxelTool.buildList (Array)
//   - Placed voxel mesh pos are tracked in:       this.voxelTool.placed (Array of {x,y,z,type})
//   - All occupied voxel centers:                 this.voxelTool.voxelCenters (Set<string>)
//
// ROBOTS:
//   - All robots currently in scene:              this.robotTool.placed (Array of THREERobot)
//   - Initial spawn positions:                    this.robotTool.initialPos (Array of {origin, target})
//   - Preview robot instance:                     this.robotTool.preview
//
// SOURCES:
//   - All placed sources:                         this.sourceTool.placed (Array of Source)
//   - Source position tracking (legacy):          this.sourceTool.Pos (Array of {Pos, Dir})
//   - Preview source instance:                    this.sourceTool.preview


// ==================== CONSTRUCTOR SETUP ====================

// REF FOR REAL ROBOT DO NOT MODIFY
const current = {
    forward: new THREE.Vector3(1, 0, 0),
    normal: new THREE.Vector3(0, 0, 1)
};
const stepSize = 1.0;
// =====================================


export class BuildingSoftwareGUI {
    constructor(scene, camera, scene_x_min, scene_x_max, scene_y_min, scene_y_max) {
        // console.log("--- BuildingSoftwareGUI initialized ---", scene, camera, scene_x_min, scene_x_max, scene_y_min, scene_y_max);
        this.scene = scene;
        this.camera = camera;
        this.scene_x_min = scene_x_min;
        this.scene_x_max = scene_x_max;
        this.scene_y_min = scene_y_min;
        this.scene_y_max = scene_y_max;

        // --- General ---
        this.activeTool = "Visualization"; // can be: "voxel", "robot", "source", "camera"
        this.rotationEnabled = false;
        this.snappedMousePosition = null;
        this.cameraRotationPaused = true;
        this.realRobotMode = false;
        // --- Tool: Voxel ---
        this.voxelTool = {
            currentType: 0,
            lastType: 0,
            preview: null,
            placed: [], // Track placed voxels to be able to undo them 
            voxelCenters: new Set(),
            buildList: [],
            supportBuildList: [],
            builderMode: false,
            flipped: false,
            rainbow: false,
            rainbowHue: 0,
            rainbowStep: 40,
            color: "#555555",
            lockedVoxelData: null, // For preview voxel movement/placement
            // For structure preview
            structurePreview: [],
            structureOffset: { x: 0, y: 0, z: 0 }
        };
        // --- Deletion Mode (global flag) ---
        this.deletionMode = false;

        // --- Tool: Robot ---
        this.robotTool = {
            preview: null,
            initialPos: [], // TO IMPLEMENT
            placed: [],
            rotation: 0,
            speed: 4.0 // Default robot speed
        };

        // --- Tool: Source ---
        this.sourceTool = {
            preview: null,
            Pos: [],        
            placed: [],
            rotation: 0
        };

        // Camera rotation loop state
        this._cameraLoopRunning = false;
        // --- Visualization: PointGoal mode ---
        this.pointGoalMode = false;
        this.lastHighlightedMesh = null;
        this.lastOriginalColor = null;
        this.selectedRobotLabel = "robot0";
        this.robotLabelMap = {};
        this.robotSwitchCount = {};
        this.robotSelectorController = null;
        
        // --- Visualization: Step by step mode ---
        this.stepByStepMode = false;
        this.stepByStepManager = undefined;
        this.stepByStepRobot = undefined;
        // Handler for the Step button in step-by-step mode
        this.stepButtonPressed = () => {
            if (this.stepByStepManager) {
            this.stepByStepManager.stepFlag = true;
            }
        };


        this.gui = new GUI();
        this.initGUI();
        // this.createBaseLayer();
        this.trackMouseIntersections();
        this.initMouseControls();
    }

    // ==================== GUI SETUP ====================
    initGUI() {
        // Add Real Robot Mode toggle at the root level of the GUI
        this.gui.add(this, "realRobotMode").name("🔁 Real Robot Mode").onChange((enabled) => {
            if (enabled) {
                this.initRealRobotOverlay();
            } else {
                this.clearRealRobotOverlay();
                if (window.lainSocket) window.lainSocket.close();
            }
        });

        // Tool switching with folder toggling (refactored with controller) - move Active Tool dropdown here
        const activeToolController = this.gui.add(this, "activeTool", {
            Voxel: "voxel",
            Robot: "robot",
            Source: "source",
            Visualization: "Visualization"
        }).name("Active Tool").onChange((value) => {
            this.voxelFolder.hide();
            this.robotFolder.hide();
            this.sourceFolder.hide();
            this.visualFolder.hide();
            this.pointGoalMode = false;

            if (this.lastHighlightedMesh && this.lastOriginalColor) {
                this.lastHighlightedMesh.material.color.copy(this.lastOriginalColor);
                this.lastHighlightedMesh = null;
                this.lastOriginalColor = null;
            }

            if (value === "Visualization") {
                this.visualFolder.show();
                this.rotationEnabled = true;
                this.cameraRotationPaused = true;
                this._cameraLoopRunning = false; // allow restart
                if (this.visualToggleController) {
                    this.visualToggleController.name("▶️ Start Rotation");
                }
            } else {
                this.rotationEnabled = false;
                if (value === "voxel") this.voxelFolder.show();
                else if (value === "robot") this.robotFolder.show();
                else if (value === "source") this.sourceFolder.show();
            }
        });

        // Force trigger update at initialization (defer to next tick to avoid folder undefined issues)
        setTimeout(() => {
            activeToolController.setValue("Visualization");
        }, 0);

        // Tool-specific folders (must be created before setValue is called)
        this.voxelFolder = this.gui.addFolder("Voxel Settings");
        this.robotFolder = this.gui.addFolder("Robot Settings");
        setSpeedTuning(10 * this.robotTool.speed); // Ensure robot speed is initialized
        this.sourceFolder = this.gui.addFolder("Source Settings");
        this.visualFolder = this.gui.addFolder("Active Tool Viz");
        // --- Carried Voxel Count for Visualization ---
        this.carriedVoxelCount = 0;

        this.visualFolder.hide(); 

        // --- Voxel tool controls in voxelFolder ---
        this.voxelFolder.add(this.voxelTool, "currentType", {
            "2x2": 0,
            "2x3": 1,
            "2x2 Offset": 2,
            "2x2 Full Offset": 3,
            "4x2": 4,
            "Stacked 2x3": 5,
            "Stacked 4x2": 6,
            "2x3 Offset Left": 7,
            "2x3 Offset Right": 8,
            "2x2x2": 9,
        }).name("Voxel Type").onChange(() => {
            this.clearPreviewVoxel();
            if (this.snappedMousePosition) {
                const { x, y, z } = this.snappedMousePosition;
                this.updatePreviewVoxel(x, y, z);            
            }
        });
        this.voxelFolder.addColor(this.voxelTool, "color").name("Voxel Color");
        this.rainbowStepController = this.voxelFolder.add(this.voxelTool, "rainbowStep", 1, 60, 1).name("Hue Step").hide();
        this.voxelFolder.add(this.voxelTool, "rainbow").name("Rainbow Colors").onChange((value) => {
            value ? this.rainbowStepController.show() : this.rainbowStepController.hide();
        });
        this.voxelFolder.add(this.voxelTool, "builderMode").name("Builder Mode");
        this.voxelFolder.add(this, "loadVoxelStructure").name("📦 Load Structure");

        // Add Start Building button to robotFolder
        // Add Speed slider to robotFolder
        this.visualFolder.add(this.robotTool, "speed", 0.1, 10.0, 0.1).name("Speed").onChange((value) => {
            setSpeedTuning(10 * value);
        }).setValue(this.robotTool.speed); // Ensure correct sync on init
        this.visualFolder.add(this, "carriedVoxelCount", 0, 3, 1).name("🧱 Carried Voxels");
        this.robotSelectorController = this.visualFolder.add(this, "selectedRobotLabel", {}).name("Controlled Robot");
        // Start with only voxelFolder shown
        this.voxelFolder.show();
        this.robotFolder.hide();
        this.sourceFolder.hide();

        // Always-visible controls
        this.gui.add(this, "deletionMode").name("Delete Mode");
        this.gui.add(this, "startRobotBuilding").name("🤖 Start Building");

        this.gui.add(this, "exportSetUp").name("💌 Export Set Up");
        this.gui.add(this, "loadSetup").name("📥 Load Setup");
        this.gui.add(this, "printVoxelMap").name("🖨️ Print VoxelMap");
        this.gui.add(this, "downloadBenchmarkJSON").name("📊 Export Benchmark JSON");
        this.gui.add({
          resetBench: () => {
            if (typeof window.resetBenchmark === 'function') window.resetBenchmark();
            console.log("Benchmark reset.");
            alert("Benchmark counters reset.");
          }
        }, "resetBench").name("🔄 Reset Benchmark");
    // (moved downloadBenchmarkJSON to class method)


        // --- Visualization folder for active tool ---
        this.visualFolder.add({ assignVoxelColors: () => this.colorVoxelsBySource() }, "assignVoxelColors").name("🎨 Color Voxels by Source");
        this.visualToggleController = this.visualFolder.add(this, "toggleCameraRotation").name("⏸ Pause Rotation");
        this.visualFolder.add(this, "stepByStepMode").name("🚶 Step-by-Step Mode");     
        this.visualFolder.add(this, "stepButtonPressed").name("▶️ Step");
        this.visualFolder.add(this, "activatePointGoalMode").name("🎯 PointGoal");
    }
    // ==================== BENCHMARK EXPORT ====================
    downloadBenchmarkJSON() {
        try {
            if (!window._enableBenchmark || !window._benchmark) {
                alert("No benchmark data available yet.");
                return;
            }
            const payload = window._benchmark;
            const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
            const a = document.createElement("a");
            a.href = URL.createObjectURL(blob);
            const ts = new Date().toISOString().replace(/[:.]/g, "-");
            a.download = `benchmark-${ts}.json`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
        } catch (e) {
            console.error("Failed to export benchmark JSON:", e);
            alert("Failed to export benchmark JSON. Check console for details.");
        }
    }

    // ==================== REAL ROBOT MODE OVERLAY ====================
    initRealRobotOverlay() {
        const box = document.createElement("div");
        box.id = "robot-status-overlay";
        box.style.position = "fixed";
        box.style.top = "10px";
        box.style.left = "10px";
        box.style.background = "rgba(0,0,0,0.7)";
        box.style.color = "#fff";
        box.style.padding = "12px";
        box.style.zIndex = "9999";
        box.style.fontFamily = "monospace";
        box.innerHTML = `
          <div><strong>Robot Connection</strong></div>
          <button id="connect-robot-button" style="margin-top:5px;">Try Connection</button>
          <div id="connection-status" style="margin-top:5px;">Status: Not connected</div>
          <div><label for="robot-target">Robot:</label> 
            <select id="robot-target">
              <option value="robot0">robot0</option>
              <option value="robot1">robot1</option>
            </select>
          </div>
        `;
        document.body.appendChild(box);

        document.getElementById("robot-target").addEventListener("change", (e) => {
          this.selectedRobotLabel = e.target.value;
        });

        document.getElementById("connect-robot-button").addEventListener("click", () => {
          this.connectToRealRobot();
        });
    }

    clearRealRobotOverlay() {
        const box = document.getElementById("robot-status-overlay");
        if (box) box.remove();
    }

    connectToRealRobot() {
        try {
            window.lainSocket = new WebSocket("ws://localhost:8765");
            window.lainSocket.onopen = () => {
                const el = document.getElementById("connection-status");
                if (el) el.textContent = "Status: ✅ Connected";
            };
            window.lainSocket.onerror = () => {
                const el = document.getElementById("connection-status");
                if (el) el.textContent = "Status: ❌ Error connecting";
            };
            window.lainSocket.onclose = () => {
                const el = document.getElementById("connection-status");
                if (el) el.textContent = "Status: ❌ Connection failed";
            };
        } catch (e) {
            console.error("WebSocket init error", e);
        }
    }
    // ==================== POINTGOAL FUNCTIONALITY ====================
    activatePointGoalMode() {
        if (this.robotTool.placed.length < 1) {
            alert("At least one robot must be placed to use PointGoal mode.");
            return;
        }
        this.pointGoalMode = true;
        // If step-by-step mode is toggled, clear any previous manager
        if (this.stepByStepMode) {
            this.stepByStepManager = undefined; // Clear any previous manager
        }
    }

    handlePointGoalClick(x, y, z) {
        if (!this.lastHighlightedMesh) {
            console.warn("No highlighted voxel mesh to target.");
            return;
        }

        const robotIndex = parseInt(this.selectedRobotLabel.replace("robot", ""));
        const robot = this.robotTool.placed[robotIndex];
        if (!robot) {
            console.warn("⚠️ No robot selected or available.");
            return;
        }
        // Ensure robot.label exists for WebSocket routing
        if (!robot.label) {
            robot.label = `robot${robotIndex}`;
        }

        const { x: gx, y: gy, z: gz } = this.lastHighlightedMesh.position;
        const target = new THREE.Vector3(gx, gy, gz);  // place just above the voxel center
        const normal = new THREE.Vector3(0, 0, 1);

        // console.log("PointGoal target:", target);

        robot.planPathToLegPositions({
            frontLegGoal: target,
            backLegGoal: null,
            normalGoal: normal,
            voxelToAvoid: undefined,
            requireParity: false

        }).then(({ success, path }) => {
            if (!success || !Array.isArray(path)) {
                console.warn("⚠️ No valid path found to:", target);
                return;
            }

            if (this.stepByStepMode) {
                const manager = {
                    stepFlag: false
                };
                this.stepByStepManager = manager;

                const executePointGoalSteps = async () => {
                    for (const step of path) {
                        await new Promise(resolve => {
                            const checkFlag = () => {
                                if (manager.stepFlag) {
                                    manager.stepFlag = false;
                                    resolve();
                                } else {
                                    requestAnimationFrame(checkFlag);
                                }  
                            };
                            checkFlag();
                        });
                        // Send high-level command to Python WebSocket before each action
                        if (window.lainConnected()) {
                            let actionName = step.action;
                            if (actionName === "switchLeg") {
                                if (!this.robotSwitchCount[robot.label] && this.robotSwitchCount[robot.label] !== 0) {
                                    this.robotSwitchCount[robot.label] = 0;
                                }
                                this.robotSwitchCount[robot.label]++;
                            } else {
                                // --- NEW FLAT MESSAGE FORMAT with robot ID as second field ---
                                const originPos = robot.origin.position;
                                const targetPos = robot.target.position;
                                const relativeFront = targetPos.clone().sub(originPos);
                                const [match] = getPossibleActions(current, stepSize, () => false, actionName);
                                const frontLegDelta = match?.moveFront || new THREE.Vector3(0, 0, 0);
                                const backLegDelta  = match?.moveBack  || new THREE.Vector3(0, 0, 0);
                                const useSwitch = this.robotSwitchCount[robot.label] % 2 === 1;
                                const tagged = useSwitch ? `switch_${actionName}` : actionName;
                                const carriedVoxel = this.carriedVoxelCount;
                                const mvtMessage = [
                                  "WebMvt",
                                  carriedVoxel,
                                  useSwitch ? 1 : 0,
                                  relativeFront.x, relativeFront.y, relativeFront.z,
                                  frontLegDelta.x, frontLegDelta.y, frontLegDelta.z,
                                  backLegDelta.x, backLegDelta.y, backLegDelta.z,
                                  tagged
                                ].join(":");
                                // console.log("🔄 WebMvt message to robot:", mvtMessage);
                                window.lainSocket.send(mvtMessage);
                            }
                        }
                        await robot.executeAction(step.action);
                        // console.log("Executing action:", step.action);
                    }
                };
                executePointGoalSteps();
            } 
        });

        this.pointGoalMode = false;
        // Reset last highlighted voxel color and state when goal is selected
        if (this.lastHighlightedMesh && this.lastOriginalColor) {
            this.lastHighlightedMesh.material.color.copy(this.lastOriginalColor);
            this.lastHighlightedMesh = null;
            this.lastOriginalColor = null;
        }
    }

    
    // ==================== BASE LAYER ====================
    createBaseLayer() {
        for (let x = this.scene_x_min; x <= this.scene_x_max - 1; x += 1) {
            for (let y = this.scene_y_min; y <= this.scene_y_max - 1; y += 1) {
                new Voxel(x + 0.5, y + 0.5, 0, this.scene, 0, false, undefined, 0xffffff);
            }
        }
    }
    // ==================== MOUSE CONTROLS ====================

    trackMouseIntersections() {
        const raycaster = new THREE.Raycaster();
        const mouse = new THREE.Vector2();

        window.addEventListener("mousemove", (event) => {
            // --- PointGoal mode highlighting ---
            if (this.pointGoalMode) {
                const raycaster = new THREE.Raycaster();
                const mouse = new THREE.Vector2();
                mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                raycaster.setFromCamera(mouse, this.camera);

                const intersects = raycaster.intersectObjects(
                    this.scene.children.filter(obj =>
                        obj.isMesh &&
                        obj.name !== "voxel_hitbox" &&
                        obj.name !== "floor" &&
                        obj.material?.color
                    )
                );

                if (intersects.length > 0) {
                    const targetMesh = intersects[0].object;
                    if (this.lastHighlightedMesh && this.lastHighlightedMesh !== targetMesh) {
                        this.lastHighlightedMesh.material.color.copy(this.lastOriginalColor);
                    }

                    if (targetMesh !== this.lastHighlightedMesh) {
                        this.lastOriginalColor = targetMesh.material.color.clone();
                        targetMesh.material.color.set(0xff0000);
                        this.lastHighlightedMesh = targetMesh;
                    }
                } else if (this.lastHighlightedMesh) {
                    this.lastHighlightedMesh.material.color.copy(this.lastOriginalColor);
                    this.lastHighlightedMesh = null;
                }

                return;
            }
            // --- End PointGoal mode highlighting ---

            if (this.deletionMode) {
                this.clearPreviewVoxel();
                mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                raycaster.setFromCamera(mouse, this.camera);
                // Only highlight deletable objects for the currently active tool
                if (this.activeTool === "voxel") {
                    const intersects = raycaster.intersectObjects(this.scene.children.filter(obj =>
                        obj.name === "voxel_hitbox"
                    ));
                    if (intersects.length > 0) {
                        const target = intersects[0].object;
                        if (this.lastHighlightedVoxel && this.lastHighlightedVoxel !== target) {
                            this.lastHighlightedVoxel.material.opacity = 0;
                            this.lastHighlightedVoxel.material.transparent = true;
                            this.lastHighlightedVoxel.material.visible = false;
                            this.lastHighlightedVoxel.material.color.set(0x00ff00);
                        }
                        if (!target.material.visible) {
                            target.material.visible = true;
                        }
                        target.material.transparent = true;
                        target.material.opacity = 0.2;
                        target.material.color.set(0xff0000);
                        this.lastHighlightedVoxel = target;
                    } else if (this.lastHighlightedVoxel) {
                        this.lastHighlightedVoxel.material.opacity = 0;
                        this.lastHighlightedVoxel.material.transparent = false;
                        this.lastHighlightedVoxel.material.color.set(0x00ff00);
                        this.lastHighlightedVoxel.material.visible = false;
                        this.lastHighlightedVoxel = null;
                    }
                } else if (this.lastHighlightedVoxel) {
                    // Remove highlight if not on voxel tool
                    this.lastHighlightedVoxel.material.opacity = 0;
                    this.lastHighlightedVoxel.material.transparent = false;
                    this.lastHighlightedVoxel.material.color.set(0x00ff00);
                    this.lastHighlightedVoxel.material.visible = false;
                    this.lastHighlightedVoxel = null;
                }
                return;
            }
            mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
            mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

            // --- UPDATED INTERSECTION LOGIC ---
            let intersection = null;
            let intersectZ = 0;

            raycaster.setFromCamera(mouse, this.camera);

            // First try voxel hitboxes
            const voxelHits = raycaster.intersectObjects(
                this.scene.children.filter(obj => obj.name === "voxel_hitbox"),
                true
            );

            // Then try the floor
            const floor = this.scene.getObjectByName("floor");
            const floorHits = floor ? raycaster.intersectObject(floor, true) : [];

            if (voxelHits.length > 0 && (!floorHits.length || voxelHits[0].distance < floorHits[0].distance)) {
                intersection = voxelHits[0].point;
                intersectZ = voxelHits[0].object.position.z + 0.5;
            } else if (floorHits.length > 0) {
                intersection = floorHits[0].point;
                intersectZ = 0;
            } else {
                this.clearPreviewVoxel();
                this.clearPreviewRobot();
                this.clearPreviewSource();
                return;
            }
            // --- NEW INTERSECTION LOGIC END ---

            let snappedZ = Math.floor(intersectZ / 0.5) * 0.5;

            // Here we should get the center of the square the mouse is in 
            let squareX = Math.floor(intersection.x / 0.25);
            let squareY = Math.floor(intersection.y / 0.25);

            if(squareX%2 === 0){squareX += 1;}
            if(squareY%2 === 0){squareY += 1;}

            squareX *= 0.25;
            squareY *= 0.25;

            let voxelDirection = this.voxelTool.preview ? this.voxelTool.preview.direction : new THREE.Vector3(0, 1, 0);
            let normalDirection = new THREE.Vector3(-voxelDirection.y, voxelDirection.x, 0); // Counterclockwise normal
            let offsetDirection = this.voxelTool.flipped
                ? voxelDirection.clone().sub(normalDirection)
                : voxelDirection.clone().add(normalDirection);
            let offset = offsetDirection.multiplyScalar(0.25);

            // Final Position for Placement
            let finalX = squareX + offset.x;
            let finalY = squareY + offset.y;
            let finalZ = snappedZ;

            this.snappedMousePosition = { x: finalX, y: finalY, z: finalZ };
            if (this.activeTool === "voxel") this.updatePreviewVoxel(finalX, finalY, finalZ);
            else if (this.activeTool === "robot") this.updateRobotPreview(finalX, finalY, finalZ);
            else if (this.activeTool === "source") this.updateSourcePreview(finalX, finalY, finalZ);        
        });
    }

    initMouseControls() {
        let isDragging = false;
    
        window.addEventListener("mousedown", () => {
            isDragging = false; // Reset drag state
        });
        window.addEventListener("mousemove", () => {
            isDragging = true; // If mouse moves, it's a drag action
        });
        window.addEventListener("mouseup", (event) => {
            if (!isDragging) {
                const isGuiClick = event.composedPath().some(el =>
                    el instanceof HTMLElement &&
                    (el.closest(".dg") || el.closest(".voxel-option") || el.closest("#info-modal") || ["BUTTON", "INPUT", "LABEL", "SELECT"].includes(el.tagName))
                );
                if (!isGuiClick) {
                    // PointGoal click: handle before other tool checks
                    if (this.pointGoalMode && this.snappedMousePosition) {
                        const { x, y, z } = this.snappedMousePosition;
                        this.handlePointGoalClick(x, y, z);
                        return;
                    }
                    if (this.deletionMode) {
                        // Only allow deletion for the currently active tool
                        if (this.activeTool === "voxel") {
                            // 🔍 Perform raycasting to find intersected voxel
                            const raycaster = new THREE.Raycaster();
                            const mouse = new THREE.Vector2();
                            mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                            mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                            raycaster.setFromCamera(mouse, this.camera);

                            const intersects = raycaster.intersectObjects(this.scene.children.filter(obj =>
                                obj.name === "voxel_hitbox"
                            ));

                            if (intersects.length > 0) {
                                const target = intersects[0].object;
                                const { x, y, z } = target.position;

                                let voxelToDelete = Array.from(window.voxelMap).find(v =>
                                    v.centers.some(c => {
                                        const [cx, cy, cz] = c.split(',').map(Number);
                                        return Math.abs(cx - x) < 0.01 &&
                                               Math.abs(cy - y) < 0.01 &&
                                               Math.abs(cz - z) < 0.01;
                                    })
                                );

                                if (!voxelToDelete) {
                                    voxelToDelete = Array.from(this.voxelTool.buildList).find(v =>
                                        v.centers.some(c => {
                                            const [cx, cy, cz] = c.split(',').map(Number);
                                            return Math.abs(cx - x) < 0.01 &&
                                                   Math.abs(cy - y) < 0.01 &&
                                                   Math.abs(cz - z) < 0.01;
                                        })
                                    );
                                }
                                if (voxelToDelete) {
                                    const centersToRemove = new Set(voxelToDelete.centers);

                                    const hitboxesToRemove = this.scene.children.filter(obj =>
                                        obj.name === "voxel_hitbox" &&
                                        centersToRemove.has(`${obj.position.x},${obj.position.y},${obj.position.z}`)
                                    );
                                    hitboxesToRemove.forEach(obj => this.scene.remove(obj));

                                    const meshToRemove = this.scene.children.filter(obj =>
                                        obj.isMesh &&
                                        centersToRemove.has(`${obj.position.x},${obj.position.y},${obj.position.z}`)
                                    );
                                    meshToRemove.forEach(obj => this.scene.remove(obj));

                                    // delete voxel from everywhere.
                                    voxelToDelete.centers.forEach(c => {
                                        this.voxelTool.voxelCenters.delete(c);
                                        if (window.voxelCenters && typeof window.voxelCenters.delete === "function") {
                                            window.voxelCenters.delete(c);
                                        }
                                    });
                                    this.voxelTool.buildList = this.voxelTool.buildList.filter(v => v !== voxelToDelete);
                                    window.voxelMap.delete(voxelToDelete);
                                }
                                // Return here to avoid robot deletion if voxel was deleted
                                return;
                            }
                        } else if (this.activeTool === "robot") {
                            // ---- Improved robot deletion logic: check all children of robotGroup ----
                            const robotRaycaster = new THREE.Raycaster();
                            const robotMouse = new THREE.Vector2();
                            robotMouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                            robotMouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                            robotRaycaster.setFromCamera(robotMouse, this.camera);

                            const robotIntersects = robotRaycaster.intersectObjects(
                                this.robotTool.placed.flatMap(robot => robot.robotGroup ? robot.robotGroup.children : []),
                                true
                            );

                            if (robotIntersects.length > 0) {
                                const target = robotIntersects[0].object;
                                const robotToDelete = this.robotTool.placed.find(robot =>
                                    (robot.robotGroup && (
                                        robot.robotGroup.children.includes(target) ||
                                        robot.robotGroup.children.includes(target.parent)
                                    ))
                                );
                                if (robotToDelete && typeof robotToDelete.delete === "function") {
                                    robotToDelete.delete();
                                    this.robotTool.placed = this.robotTool.placed.filter(r => r !== robotToDelete);
                                }
                            }
                        } else if (this.activeTool === "source") {
                            // Updated source deletion logic: click any part of a source to delete the whole source
                            const raycaster = new THREE.Raycaster();
                            const mouse = new THREE.Vector2();
                            mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
                            mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
                            raycaster.setFromCamera(mouse, this.camera);

                            const intersects = raycaster.intersectObjects(this.scene.children.filter(obj =>
                                obj.isMesh && typeof obj.userData?.source?.delete === "function"
                            ));

                            if (intersects.length > 0) {
                                const matchedSource = intersects[0].object.userData.source;
                                matchedSource.delete();
                                this.sourceTool.placed = this.sourceTool.placed.filter(s => s !== matchedSource);
                            }
                        }
                    } else {
                        if (this.activeTool === "voxel") this.placeVoxelAtMousePosition();
                        else if (this.activeTool === "robot") this.placeRobotAtMousePosition();
                        else if (this.activeTool === "source") this.placeSourceAtMousePosition();
                    }
                }
            }
        });

        window.addEventListener("keydown", (event) => {
            // --- Structure preview movement/placement with arrow keys and Enter ---
            if (this.voxelTool.structurePreview && this.voxelTool.structurePreview.length > 0) {
                const step = 0.5;
                let moved = false;
                if (event.key === "ArrowUp") { this.voxelTool.structureOffset.y += step; moved = true; }
                else if (event.key === "ArrowDown") { this.voxelTool.structureOffset.y -= step; moved = true; }
                else if (event.key === "ArrowLeft") { this.voxelTool.structureOffset.x -= step; moved = true; }
                else if (event.key === "ArrowRight") { this.voxelTool.structureOffset.x += step; moved = true; }
                else if (event.key === "Enter") {
                    // Remove preview
                    this.voxelTool.structurePreview.forEach(({ voxel }) => {
                        if (voxel.mesh) this.scene.remove(voxel.mesh);
                    });

                    const jsondata = Array.from(this.voxelTool.structurePreview).map(d => ({
                        type: d.voxel.type,
                        centers: d.voxel.centers
                    }));

                    this.voxelTool.structurePreview = [];
                    this.promptStructureLoadMode().then(mode => {
                        if (mode === "cancel") {
                            this.voxelTool.structurePreview.forEach(({ voxel }) => {
                                if (voxel.mesh) this.scene.remove(voxel.mesh);
                            });
                            this.voxelTool.structurePreview = [];
                            this.voxelTool.structureOffset = { x: 0, y: 0, z: 0 };
                            return;
                        }
                        const asBuildList = (mode === "buildList");
                        jsondata.forEach(entry => {
                            const [x, y, z] = entry.centers[0].split(',').map(Number);
                            const offset = this.voxelTool.structureOffset;
                            let direction = new THREE.Vector3(0, 1, 0);
                            if (entry.centers.length > 1) {
                                const [x2, y2] = entry.centers[1].split(',').map(Number);
                                direction = new THREE.Vector3(x2 - x, y2 - y, 0).normalize();
                            }
                            const finalX = x + offset.x;
                            const finalY = y + offset.y;
                            const finalZ = z + offset.z;

                            const voxel = new Voxel(finalX, finalY, finalZ, this.scene, entry.type, asBuildList, direction, undefined, asBuildList);
                            voxel.centers = entry.centers.map(c => {
                                const [cx, cy, cz] = c.split(',').map(Number);
                                return `${cx + offset.x},${cy + offset.y},${cz + offset.z}`;
                            });

                        if (asBuildList) {
                            this.voxelTool.buildList.push(voxel);
                            // Track centers for overlap checking (when structure is loaded as build-list)
                            voxel.centers.forEach(c => {
                                this.voxelTool.voxelCenters.add(c);
                                if (window.voxelCenters && typeof window.voxelCenters.add === "function") {
                                    window.voxelCenters.add(c);
                                }
                            });
                        }
                        });

                        this.voxelTool.structurePreview = [];
                        this.voxelTool.structureOffset = { x: 0, y: 0, z: 0 };
                    });
                    return;
                }
                if (moved) {
                    const { x: dx, y: dy, z: dz } = this.voxelTool.structureOffset;
                    this.voxelTool.structurePreview.forEach(({ voxel, original }) => {
                        voxel.mesh.position.set(original.x + dx, original.y + dy, original.z + dz);
                    });
                    return;
                }
            }
            // --- End structure preview movement/placement ---
            // --- Preview voxel movement/placement with arrow keys and Enter ---
            if (this.voxelTool.lockedVoxelData) {
                const step = 0.5;
                let { x, y, z } = this.voxelTool.lockedVoxelData;
                let moved = false;
                if (event.key === "ArrowUp") {
                    y += step; moved = true;
                } else if (event.key === "ArrowDown") {
                    y -= step; moved = true;
                } else if (event.key === "ArrowLeft") {
                    x -= step; moved = true;
                } else if (event.key === "ArrowRight") {
                    x += step; moved = true;
                } else if (event.key === "Enter") {
                    const d = this.voxelTool.lockedVoxelData;
                    const voxel = new Voxel(x, y, z, this.scene, d.type, true, d.direction, undefined, true);
                    voxel.toBuild = true;
                    voxel.centers = d.centers;
                    this.voxelTool.buildList.push(voxel);
                    this.clearPreviewVoxel();
                    this.voxelTool.lockedVoxelData = null;
                    // Remove yellow previews after placement
                    this.scene.children = this.scene.children.filter(obj => {
                    if (obj.userData?.structurePreview) {
                        this.scene.remove(obj);
                        return false;
                    }
                    return true;
                    });
                    this.voxelTool.structurePreview = [];
                    return;
                }
                if (moved) {
                    this.voxelTool.lockedVoxelData.x = x;
                    this.voxelTool.lockedVoxelData.y = y;
                    this.updatePreviewVoxel(x, y, z);
                    return;
                }
            }
            // --- End preview voxel movement/placement ---
            if (event.key.toLowerCase() === "r") {
                if (this.activeTool === "voxel" && this.voxelTool.preview?.rotate) {
                    this.voxelTool.preview.rotate();
                } else if (this.activeTool === "robot" && this.robotTool.preview?.rotate) {
                    this.robotTool.preview.rotate();
                } else if (this.activeTool === "source" && this.sourceTool.preview?.rotate) {
                    this.sourceTool.preview.rotate();
                }
            }
            // Only switch voxel types with spacebar if not in Visualization mode
            if (event.key === " " && this.activeTool !== "Visualization") {
                const tempType = this.voxelTool.currentType;
                this.voxelTool.currentType = this.voxelTool.lastType;
                this.voxelTool.lastType = tempType;
                this.clearPreviewVoxel(); // Refresh preview with new type
                if (this.snappedMousePosition) {
                    const { x, y, z } = this.snappedMousePosition;
                    this.updatePreviewVoxel(x, y, z);
                }
            }
            // Step-by-step handler (centralized for both build and PointGoal)
            if ( event.key === " " && this.stepByStepMode && this.stepByStepManager) {
                this.stepByStepManager.stepFlag = true;
            }
            if (event.key === "b") {
                this.voxelTool.flipped = !this.voxelTool.flipped;
            }
            if (event.key === "p") {
                if (this.activeTool === "voxel") {
                    this.undoLastVoxel();
                } else if (this.activeTool === "robot") {
                    this.undoLastRobot();
                } else if (this.activeTool === "source") {
                    this.undoLastSource();
                }
            }
        });
    }

    // ==================== VOXEL TOOL ====================
    // --- Preview, placement, undo, etc. ---
    updatePreviewVoxel(x, y, z) {
        // If lockedVoxelData exists, set type/direction accordingly
        if (this.voxelTool.lockedVoxelData) {
            this.voxelTool.currentType = this.voxelTool.lockedVoxelData.type;
            if (this.voxelTool.lockedVoxelData.direction) {
                // Only set if direction property exists
                this.voxelTool.preview && (this.voxelTool.preview.direction = this.voxelTool.lockedVoxelData.direction.clone());
            }
        }
        // If a preview voxel already exists, move it
        if (this.voxelTool.preview) {
            if (this.voxelTool.preview.mesh) {
                this.voxelTool.preview.mesh.position.set(x, y, z);
            }
            return;
        }
        // Use default values for direction if previewVoxel does not exist
        let direction = this.voxelTool.preview ? this.voxelTool.preview.direction : new THREE.Vector3(0, 1, 0);
        if (this.voxelTool.lockedVoxelData && this.voxelTool.lockedVoxelData.direction) {
            direction = this.voxelTool.lockedVoxelData.direction.clone();
        }
        this.voxelTool.preview = new Voxel(x, y, z, this.scene, this.voxelTool.currentType, true, direction);
        // Wait for the STL model to load before modifying position
        this.voxelTool.preview.loader.load(this.voxelTool.preview.path, (geometry) => {
            if (this.voxelTool.preview && this.voxelTool.preview.mesh) {
                this.voxelTool.preview.mesh.position.set(x, y, z);
            }
        });
    }
    clearPreviewVoxel() {
        if (this.voxelTool.preview) {
            this.scene.remove(this.voxelTool.preview.mesh);
            this.voxelTool.preview = null;
        }
    }

    // ==================== VOXEL PLACEMENT ====================

    placeVoxelAtMousePosition() {
        if (!this.snappedMousePosition || !this.voxelTool.preview) return;

        const { x, y, z } = this.snappedMousePosition;
        let voxelData, centerPositions = [];

        let color = this.voxelTool.color;
        if (this.voxelTool.rainbow) {
            const hsl = `hsl(${this.voxelTool.rainbowHue}, 100%, 50%)`;
            const tempColor = new THREE.Color(hsl);
            color = `#${tempColor.getHexString()}`;
            this.voxelTool.rainbowHue = (this.voxelTool.rainbowHue + this.voxelTool.rainbowStep) % 360;
        }

        const type = this.voxelTool.currentType;
        const previewVoxel = this.voxelTool.preview;
        if (type === 0) { // **2x2 voxel (1 center)**
            let center = `${x},${y},${z}`;
            voxelData = { type: 0, centers: [center] };
            centerPositions.push(center);
        } else if (type === 1) {
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 0.5 * previewVoxel.direction.x},${y + 0.5 * previewVoxel.direction.y},${z}`;
            voxelData = { type: 1, centers: [center1, center2] };
            centerPositions.push(center1, center2);
        } else if (type === 2) {
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 0.5 * previewVoxel.direction.x},${y + 0.5 * previewVoxel.direction.y},${z - 0.5}`;
            voxelData = { type: 2, centers: [center1, center2] };
            centerPositions.push(center1, center2);
        } else if (type === 3) {
            let vert = new THREE.Vector3(0, 0, 1);
            let norm = new THREE.Vector3().copy(previewVoxel.direction).cross(vert);
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 0.5 * (previewVoxel.direction.x + norm.x)},${y + 0.5 * (previewVoxel.direction.y + norm.y)},${z - 0.5}`;
            voxelData = { type: 3, centers: [center1, center2] };
            centerPositions.push(center1, center2);
        } else if (type === 4) {
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 1 * previewVoxel.direction.x},${y + 1 * previewVoxel.direction.y},${z}`;
            voxelData = { type: 4, centers: [center1, center2] };
            centerPositions.push(center1, center2);
        } else if (type === 5) {
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 0.5 * previewVoxel.direction.x},${y + 0.5 * previewVoxel.direction.y},${z}`;
            let center3 = `${x},${y},${z + 0.5}`;
            let center4 = `${x + 0.5 * previewVoxel.direction.x},${y + 0.5 * previewVoxel.direction.y},${z + 0.5}`;
            voxelData = { type: 5, centers: [center1, center2, center3, center4] };
            centerPositions.push(center1, center2, center3, center4);
        } else if (type === 6) {
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 1 * previewVoxel.direction.x},${y + 1 * previewVoxel.direction.y},${z}`;
            let center3 = `${x},${y},${z + 0.5}`;
            let center4 = `${x + 1 * previewVoxel.direction.x},${y + 1 * previewVoxel.direction.y},${z + 0.5}`;
            voxelData = { type: 6, centers: [center1, center2, center3, center4] };
            centerPositions.push(center1, center2, center3, center4);
        } else if (type === 7) {
            // 2by3_offset_left
            let vert = new THREE.Vector3(0, 0, 1);
            let norm = new THREE.Vector3().copy(previewVoxel.direction).cross(vert);
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 0.5 * previewVoxel.direction.x},${y + 0.5 * previewVoxel.direction.y},${z}`;
            let center3 = `${x + 1 * previewVoxel.direction.x - 0.5 * norm.x},${y + 1 * previewVoxel.direction.y - 0.5 * norm.y},${z - 0.5}`;
            voxelData = { type: 7, centers: [center1, center2, center3] };
            centerPositions.push(center1, center2, center3);
        } else if (type === 8) {
            // 2by3_offset_right
            let vert = new THREE.Vector3(0, 0, 1);
            let norm = new THREE.Vector3().copy(previewVoxel.direction).cross(vert);
            let center1 = `${x},${y},${z}`;
            let center2 = `${x + 0.5 * previewVoxel.direction.x},${y + 0.5 * previewVoxel.direction.y},${z}`;
            let center3 = `${x + 1 * previewVoxel.direction.x + 0.5 * norm.x},${y + 1 * previewVoxel.direction.y + 0.5 * norm.y},${z - 0.5}`;
            voxelData = { type: 8, centers: [center1, center2, center3] };
            centerPositions.push(center1, center2, center3);
        } else if (type === 9) {
            let vert = new THREE.Vector3(0, 0, 1);
            let norm = new THREE.Vector3().copy(previewVoxel.direction).cross(vert);
            let center1 = `${x},${y},${z}`;
            let center2 = `${x},${y},${z+0.5}`;
            voxelData = { type: 9, centers: [center1, center2] };
            centerPositions.push(center1, center2);
        }
        for (let center of centerPositions) {
            const [cx, cy, cz] = center.split(',').map(Number);
            const nearbyOffsets = [
                [0, 0], [0.5, 0], [-0.5, 0], [0, 0.5], [0, -0.5],
                [0.5, 0.5], [-0.5, 0.5], [0.5, -0.5], [-0.5, -0.5]
            ];
            for (let [dx, dy] of nearbyOffsets) {
                const offsetCenter = `${cx + dx},${cy + dy},${cz}`;
                if (this.voxelTool.voxelCenters.has(offsetCenter)) {
                    return;
                }
            }
            if (this.voxelTool.voxelCenters.has(center)) {
                return;
            }
        }

        // Place the voxel and PASS ROTATION (no direct mesh access here!)
        if (this.voxelTool.builderMode === true) {
            const buildVoxel = new Voxel(x, y, z, this.scene, type, true, previewVoxel.direction, color, true);
            
            buildVoxel.toBuild = true;
            buildVoxel.centers = centerPositions;

            // Remove this voxel from buildList to prevent duplicates
            this.voxelTool.buildList = this.voxelTool.buildList.filter(v =>
                !centerPositions.some(c => v.centers.includes(c))
            );

            this.voxelTool.buildList.push(buildVoxel);

            // Track centers so overlap checks work even after loading a structure
            centerPositions.forEach(c => {
                this.voxelTool.voxelCenters.add(c);
                if (window.voxelCenters && typeof window.voxelCenters.add === "function") {
                    window.voxelCenters.add(c);
                }
            });

            // Remove matching preview entries from structurePreview
            this.voxelTool.structurePreview = this.voxelTool.structurePreview.filter(({ voxel }) => {
                const match = centerPositions.some(c => voxel.centers.includes(c));
                if (match && voxel.mesh) this.scene.remove(voxel.mesh);
                return !match;
            });
        } else {
            new Voxel(x, y, z, this.scene, type, false, previewVoxel.direction, color);
        }
        // Track separately
        centerPositions.forEach(center => this.voxelTool.voxelCenters.add(center));

        this.voxelTool.placed.push({ x, y, z, type: type });
        // console.log("voxel placed", this.voxelTool.placed);
    }

    undoLastVoxel() {
        if (this.voxelTool.placed.length === 0) return;

        const last = this.voxelTool.placed.pop();
        const { x, y, z, type } = last;

        // Remove mesh of the last placed voxel only (excluding previewVoxel)
        const toRemove = this.scene.children.filter(obj =>
            obj.name !== "voxel_hitbox" &&
            obj.isMesh &&
            obj !== this.voxelTool.preview?.mesh && // Exclude the preview voxel
            Math.abs(obj.position.x - x) < 0.01 &&
            Math.abs(obj.position.y - y) < 0.01 &&
            Math.abs(obj.position.z - z) < 0.01
        );
        toRemove.forEach(obj => this.scene.remove(obj));

        let voxelToDelete = Array.from(window.voxelMap).find(v =>
            v.centers.some(c => c === `${x},${y},${z}`)
        );
        if (!voxelToDelete) {
            voxelToDelete = Array.from(this.voxelTool.buildList).find(v =>
                v.centers.some(c => c === `${x},${y},${z}`)
            );
        }

        if (voxelToDelete) {
            const centersToRemove = new Set(voxelToDelete.centers);

            const hitboxesToRemove = this.scene.children.filter(obj =>
                obj.name === "voxel_hitbox" &&
                obj !== this.voxelTool.preview?.hitbox &&
                centersToRemove.has(`${obj.position.x},${obj.position.y},${obj.position.z}`)
            );

            hitboxesToRemove.forEach(obj => this.scene.remove(obj));

            window.voxelMap.delete(voxelToDelete);
            this.voxelTool.buildList = this.voxelTool.buildList.filter(v => v !== voxelToDelete);
            voxelToDelete.centers.forEach(c => {
                this.voxelTool.voxelCenters.delete(c);
                if (window.voxelCenters && typeof window.voxelCenters.delete === "function") {
                    window.voxelCenters.delete(c);
                }
            });
        }

    }

    // ==================== SETUP: EXPORT / LOAD / CLEAR ====================
    
    exportSetUp() {
        // Write sources first, then robots, then voxelMap, then buildList.
        const exportData = {
            baseLayer: {
                scene_x_min: this.scene_x_min,
                scene_x_max: this.scene_x_max,
                scene_y_min: this.scene_y_min,               
                scene_y_max: this.scene_y_max
            },
            sources: this.getSources().map(source => ({
                position: {
                    x: source.position.x,
                    y: source.position.y,
                    z: source.position.z
                },
                direction: {
                    x: source.direction.x,
                    y: source.direction.y,
                    z: source.direction.z
                }
            })),
            robots: this.getRobots().map(robot => ({
                origin: robot.origin,
                target: robot.target
            })),
            voxelMap: this.getPlacedVoxels()
                .filter(voxel =>
                    !voxel.centers.every(c => Number(c.split(',')[2]) === 0)
                )
                .map(voxel => ({
                    type: voxel.type,
                    centers: voxel.centers
                })),
            buildList: this.getBuildList().map(voxel => ({
                type: voxel.type,
                centers: voxel.centers
            }))
        };
        // Export as JSON
        const jsonData = JSON.stringify(exportData, null, 2);
        const blob = new Blob([jsonData], { type: "application/json" });
        const a = document.createElement("a");
        a.href = URL.createObjectURL(blob);
        a.download = "Simulation_Setup.json";
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
    }
    voxelArrayToSet(dataArray) {
        return new Set(dataArray.map(voxel => ({
            type: voxel.type,
            centers: [...voxel.centers]
        })));
    }
    
    async promptStructureLoadMode() {
        return new Promise(resolve => {
            const modal = document.getElementById("load-mode-modal");
            const buildBtn = document.getElementById("load-as-buildlist");
            const realBtn = document.getElementById("load-as-real");
            const cancelBtn = document.getElementById("load-cancel");

            modal.style.display = "flex";

            const cleanup = () => {
            buildBtn.removeEventListener("click", onBuild);
            realBtn.removeEventListener("click", onReal);
            cancelBtn.removeEventListener("click", onCancel);
            modal.style.display = "none";
            };

            const onBuild = () => { cleanup(); resolve("buildList"); };
            const onReal = () => { cleanup(); resolve("real"); };
            const onCancel = () => { cleanup(); resolve("cancel"); };

            buildBtn.addEventListener("click", onBuild);
            realBtn.addEventListener("click", onReal);
            cancelBtn.addEventListener("click", onCancel);
        });
    }

    async loadVoxelStructure() {
        const input = document.createElement("input");
        input.type = "file";
        input.accept = ".json";

        input.addEventListener("change", async (event) => {
            const file = event.target.files[0];
            if (!file) return;

            try {
                const text = await file.text();
                const jsondata = JSON.parse(text);
                const voxelStructure = this.voxelArrayToSet(jsondata);

                // Clear any previous structure preview
                if (this.voxelTool.structurePreview && this.voxelTool.structurePreview.length > 0) {
                    this.voxelTool.structurePreview.forEach(({ voxel }) => {
                        if (voxel.mesh) this.scene.remove(voxel.mesh);
                    });
                }
                this.voxelTool.structurePreview = [];
                this.voxelTool.structureOffset = { x: 0, y: 0, z: 0 };

                // Show preview of structure as yellow transparent voxels
                voxelStructure.forEach(voxelEntry => {
                    const { type, centers } = voxelEntry;
                    if (centers.length === 0) return;
                    const [x, y, z] = centers[0].split(',').map(Number);
                    let direction = new THREE.Vector3(0, 1, 0);
                    if (centers.length > 1) {
                        const [x2, y2] = centers[1].split(',').map(Number);
                        direction = new THREE.Vector3(x2 - x, y2 - y, 0).normalize();
                    }
                    // Create yellow transparent voxel as preview, mark with userData
                    const v = new Voxel(x, y, z, this.scene, type, true, direction, "#ffff00", false);
                    v.centers = centers;
                    if (v.mesh) {
                        v.mesh.material.transparent = true;
                        v.mesh.material.opacity = 0.4;
                        v.mesh.material.color.set("#ffff00");
                        v.mesh.userData.structurePreview = true;
                    }
                    this.voxelTool.structurePreview.push({ voxel: v, original: { x, y, z } });
                });

                // Now, allow moving preview with arrow keys and confirm with Enter
                // (handled in keydown listener)
                // Show instructions
                alert("Use arrow keys to move structure preview, Enter to confirm placement.");

            } catch (err) {
                console.error("Failed to load voxel structure:", err);
            }
        });

        input.click();
    }

    // ==================== BUILD VOXEL STRUCTURE (INSTANT PLACE) ====================   

    
    printVoxelMap() {
        console.log("🗺️ Current voxelMap:", Array.from(window.voxelMap));
        console.log("🗺️ Current source:", Array.from(this.sourceTool.placed));
        console.log("🗺️ Current robot:", Array.from(this.robotTool.placed));
        console.log("🗺️ Current tobuild:", Array.from(this.voxelTool.buildList));

        // Download robot movement history as JSON file, filtered to exclude pick/place actions
        // const movementExport = this.robotTool.placed.map((robot, index) => {
        //     const label = robot.label || `robot${index}`;
        //     const fullHistory = robot.movementHistory || [];
        //     const filtered = fullHistory.filter(entry =>
        //         !entry.action.startsWith("pisck") && !entry.action.startsWith("plsace")
        //     );
        //     return {
        //         robot: label,
        //         movementHistory: filtered
        //     };
        // });
        // const blob = new Blob([JSON.stringify(movementExport, null, 2)], { type: "application/json" });
        // const a = document.createElement("a");
        // a.href = URL.createObjectURL(blob);
        // a.download = "robot_movement_history.json";
        // document.body.appendChild(a);
        // a.click();
        // document.body.removeChild(a);
    }

    // ==================== ROBOT TOOL ====================
    updateRobotPreview(x, y, z) {
        
        const direction = this.robotTool.preview ? this.robotTool.preview.direction : new THREE.Vector3(0, 1, 0);

        const origin = new THREE.Vector3(x, y, z);
        const target = origin.clone().add(direction);
        const normal = new THREE.Vector3(0, 0, 1);

        if (this.robotTool.preview) {
            this.scene.remove(this.robotTool.preview.robotGroup);
        }

        const robot = new THREERobot(origin, target, normal, this.scene);
        robot.direction = direction.clone();  // Store direction on preview for later reuse
        this.robotTool.preview = robot;
    }
    

    placeRobotAtMousePosition() {
        if (!this.snappedMousePosition || !this.robotTool.preview) return;

        const robot = this.robotTool.preview;
        robot.preview = false;
        // Assign a unique label BEFORE pushing, so index reflects intended robot id
        robot.label = `robot${this.robotTool.placed.length}`;
        this.robotSwitchCount[robot.label] = 0;
        this.robotTool.placed.push(robot);
        // Store initial position and direction
        this.robotTool.initialPos.push({
            origin: robot.origin.position.clone(),
            target: robot.target.position.clone()
        });
        this.updateRobotSelector();
        // console.log("Robot placed", this.robotTool.placed);
        this.robotTool.preview = null;
    }

    undoLastRobot() {
        if (this.robotTool.placed.length === 0) return;

        const lastRobot = this.robotTool.placed.pop();
        this.robotTool.initialPos.pop();

        if (typeof lastRobot.delete === "function") {
            lastRobot.delete();
        }
    }
    startRobotBuilding() {
        console.log("Robot is starting the building sequence...");

        const voxelStructure = Array.from(this.voxelTool.buildList).map(voxel => ({
            type: voxel.type,
            centers: voxel.centers
        }));

        const sources = this.sourceTool.placed;
        const robots = this.robotTool.placed;

        if (!sources.length || !robots.length || voxelStructure.length === 0) {
            alert("Please place at least one robot, one source, and some voxels.");
            return;
        }

        const planner = new BuildingSequencePlanner(sources, voxelStructure, robots.length, window.gui.voxelTool.supportBuildList);

        for (let i = 0; i < robots.length; i++) {
            const robot = robots[i];
            const source = sources[i];
            const sequence = planner.robotSequences[`robot${i + 1}`];
            robot.manager = new BuildingManager(robot, sequence, source, this.scene);

            robot.manager.stepModeEnabled = this.stepByStepMode;
            robot.manager.stepFlag = false;
            
            // Replace onVoxelBuilt with new implementation to remove by reference or structure
            robot.manager.onVoxelBuilt = (placedVoxel) => {
            // Normalize centers as sorted strings so order doesn't matter
            const normalizeCenters = (obj) => {
                if (!obj || !Array.isArray(obj.centers)) return [];
                return obj.centers.map(c => String(c)).sort();
            };
            const equalCenters = (a, b) => {
                if (!a || !b) return false;
                if (a.type !== b.type) return false;
                const A = normalizeCenters(a);
                const B = normalizeCenters(b);
                if (A.length !== B.length) return false;
                for (let i = 0; i < A.length; i++) {
                if (A[i] !== B[i]) return false;
                }
                return true;
            };

            // 1) Remove by identity (shared instance case)
            let idx = this.voxelTool.buildList.indexOf(placedVoxel);
            if (idx !== -1) {
                this.voxelTool.buildList.splice(idx, 1);
                return;
            }

            // 2) Remove by structural equality (type + centers as a set, order-insensitive)
            this.voxelTool.buildList = this.voxelTool.buildList.filter(v => !equalCenters(v, placedVoxel));
            };
        }

        if (this.stepByStepMode) {
            this.stepByStepManagers = robots.map(r => r.manager);
            this.stepByStepManagers.forEach(manager => manager.start());
        } else {
            Promise.all(robots.map(r => r.manager.start()));
        }
        window.addEventListener("keydown", (event) => {
        if ((event.key === " " || event.key === "Enter") && this.stepByStepMode) {
            this.stepByStepManagers.forEach(manager => {
                if (manager.stepModeEnabled) {
                    manager.stepFlag = true;
                }
            });
        }
    });
    }

    clearPreviewRobot() {
        if (this.robotTool.preview) {
            this.robotTool.preview.delete();
            this.robotTool.preview = null;
        }
    }
    updateRobotSelector() {
        const labelMap = {};
        this.robotTool.placed.forEach((robot, index) => {
            const label = `robot${index}`;
            labelMap[label] = label;
        });
        if (Object.keys(labelMap).length === 0) {
            labelMap["none"] = "none";
        }

        this.robotLabelMap = labelMap;
        // --- FIX: Properly destroy the old controller and recreate a new one ---
        if (this.robotSelectorController) {
            this.robotSelectorController.destroy();
        }
        this.robotSelectorController = this.robotFolder.add(this, "selectedRobotLabel", labelMap).name("Controlled Robot");
        this.selectedRobotLabel = Object.keys(labelMap)[0];
    }
    // ==================== SOURCE TOOL ====================
    updateSourcePreview(x, y, z) {
        // If a preview source already exists, move it
        if (this.sourceTool.preview) {
            if (this.sourceTool.preview.group) {
                this.sourceTool.preview.group.position.set(x, y, z);
            }
            return;
        }

        let direction = this.sourceTool.preview ? this.sourceTool.preview.direction : new THREE.Vector3(0, 1, 0);
        this.sourceTool.preview = new Source(x, y, z, direction, this.scene, true);
    }


    placeSourceAtMousePosition() {
        if (!this.snappedMousePosition) return;

        const { x, y, z } = this.snappedMousePosition;
        const direction = this.sourceTool.preview.direction;

        const source = new Source(x, y, z, direction, this.scene, false);
        this.sourceTool.placed.push(source)
        this.sourceTool.Pos.push({ Pos: { x, y, z }, Dir: direction });

        // console.log("Source positions", this.sourceTool.Pos);
    }
    clearPreviewSource() {
        if (this.sourceTool.preview) {
            this.sourceTool.preview.delete();

            // this.scene.remove(this.sourceTool.preview.mesh);
            this.sourceTool.preview = null;
        }
    }

        // Undo last placed source
    undoLastSource() {
        console.log("undoing last source")
        if (this.sourceTool.placed.length === 0) return;
        const lastSource = this.sourceTool.placed.pop();
        if (typeof lastSource.delete === "function") {
            lastSource.delete();
        }
    }
    // ==================== VISUALIZATION ANIMATION ====================

    startCameraRotation() {
        // Prevent multiple animation loops
        if (this._cameraLoopRunning) return;
        this._cameraLoopRunning = true;
        // Use geometric center of floor/grid
        const centerX = (this.scene_x_min + this.scene_x_max) / 2;
        const centerY = (this.scene_y_min + this.scene_y_max) / 2;
        const center = new THREE.Vector3(centerX, centerY, 0);
        const radius = 20;
        const height = 8;
        let angle = 0;

        const animate = () => {
            if (!this.rotationEnabled || this.cameraRotationPaused) {
                requestAnimationFrame(animate); // keep loop going
                return;
            }

            angle += 0.002;
            const x = center.x + radius * Math.cos(angle);
            const y = center.y + radius * Math.sin(angle);
            const z = height;

            this.camera.position.set(x, y, z);
            this.camera.lookAt(center);

            requestAnimationFrame(animate);
        };

        animate();
    }
    toggleCameraRotation() {
        this.cameraRotationPaused = !this.cameraRotationPaused;
        if (this.visualToggleController) {
            this.visualToggleController.name(this.cameraRotationPaused ? "▶️ Resume Rotation" : "⏸ Pause Rotation");
        }
        if (!this.cameraRotationPaused) {
            this.startCameraRotation();
        }
    }
    colorVoxelsBySource() {
        const sources = this.sourceTool.placed;
        if (!sources.length) {
            console.warn("No sources placed.");
            return;
        }

        // Assign a unique color to each source
        const sourceColors = new Map();
        sources.forEach((source, i) => {
            const color = new THREE.Color().setHSL(i / sources.length, 1, 0.5);
            sourceColors.set(source, color);
        });

        // Flip flag for draw resolution
        let flipFlag = true;

        // Color voxels based on minimum distance between any of their centers and all sources
        this.voxelTool.buildList.forEach(voxel => {
            const centerVectors = voxel.centers.map(c => {
                const [x, y, z] = c.split(',').map(Number);
                return new THREE.Vector3(x, y, z);
            });

            let minDistance = Infinity;
            let closestSources = [];

            sources.forEach((source, i) => {
                const srcPos = source.position;
                const distances = centerVectors.map(vec => vec.distanceTo(srcPos));
                const minDistForSource = Math.min(...distances);

                if (minDistForSource < minDistance - 1e-6) {
                    minDistance = minDistForSource;
                    closestSources = [source];
                } else if (Math.abs(minDistForSource - minDistance) < 1e-6) {
                    closestSources.push(source);
                }
            });

            // Handle draw
            let assignedSource = closestSources[0];
            if (closestSources.length > 1) {
                console.warn(`Voxel at ${voxel.centers} is equidistant to multiple sources. Alternating color assignment.`);
                assignedSource = flipFlag ? closestSources[0] : closestSources[1];
                flipFlag = !flipFlag;
            }

            const assignedColor = sourceColors.get(assignedSource);
            if (!assignedColor) return;

            const matchingMeshes = this.scene.children.filter(obj =>
                obj.isMesh && voxel.centers.some(c => {
                    const [cx, cy, cz] = c.split(',').map(Number);
                    return Math.abs(obj.position.x - cx) < 0.01 &&
                           Math.abs(obj.position.y - cy) < 0.01 &&
                           Math.abs(obj.position.z - cz) < 0.01;
                })
            );

            matchingMeshes.forEach(mesh => {
                mesh.material.color.copy(assignedColor);
            });
        });
    }
// ==================== GETTER FUNCTION ====================
    getPlacedVoxels() { return Array.from(window.voxelMap.values()); }
    getBuildList() { return this.voxelTool.buildList; }
    getRobots() { return this.robotTool.initialPos; }
    getSources() { return this.sourceTool.placed; }


    // ==================== LOAD SETUP ====================
    loadSetup() {
        const input = document.createElement("input");
        input.type = "file";
        input.accept = ".json";

        input.addEventListener("change", async (event) => {
            const file = event.target.files[0];
            if (!file) return;

            try {
                const text = await file.text();
                const jsondata = JSON.parse(text);

                // Reset scene
                this.clearAll(); // Remove all placed objects and reset the scene state

                // Add floor mesh to the scene after reloading
                const planeGeometry = new THREE.BoxGeometry(this.scene_x_max - this.scene_x_min + 2, this.scene_y_max - this.scene_y_min + 2, 0.5);
                const planeMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff });
                const floor = new THREE.Mesh(planeGeometry, planeMaterial);
                floor.name = "floor";
                floor.position.z = -0.25;
                floor.receiveShadow = true;
                this.scene.add(floor);

                // Base layer
                if (jsondata.baseLayer) {
                    this.scene_x_min = jsondata.baseLayer.scene_x_min;
                    this.scene_x_max = jsondata.baseLayer.scene_x_max;
                    this.scene_y_min = jsondata.baseLayer.scene_y_min;
                    this.scene_y_max = jsondata.baseLayer.scene_y_max;
                }

                // --- Load in order: sources, robots, voxelMap, buildList ---

                // Sources
                this.sourceTool.placed = [];
                if (Array.isArray(jsondata.sources)) {
                    jsondata.sources.forEach(s => {
                        const source = new Source(
                            s.position.x,
                            s.position.y,
                            s.position.z,
                            new THREE.Vector3(s.direction.x, s.direction.y, s.direction.z),
                            this.scene,
                            false
                        );
                        this.sourceTool.placed.push(source);
                    });
                }

                // Robots
                this.robotTool.placed = [];
                this.robotTool.initialPos = [];
                if (Array.isArray(jsondata.robots)) {
                    jsondata.robots.forEach(r => {
                        const origin = new THREE.Vector3(r.origin.x, r.origin.y, r.origin.z);
                        const direction = new THREE.Vector3(r.target.x - r.origin.x, r.target.y - r.origin.y, 0).normalize();
                        const target = origin.clone().add(direction);
                        const robot = new THREERobot(origin, target, new THREE.Vector3(0, 0, 1), this.scene);
                        this.robotTool.placed.push(robot);
                        this.robotTool.initialPos.push({ origin, target });
                    });
                }

                // Voxel map
                if (Array.isArray(jsondata.voxelMap)) {
                    jsondata.voxelMap.forEach(voxel => {
                        const [x, y, z] = voxel.centers[0].split(',').map(Number);
                        let direction = new THREE.Vector3(0, 1, 0);
                        if (voxel.centers.length > 1) {
                            const [x2, y2] = voxel.centers[1].split(',').map(Number);
                            direction = new THREE.Vector3(x2 - x, y2 - y, 0).normalize();
                        }
                        new Voxel(x, y, z, this.scene, voxel.type, false, direction, undefined, false);
                    });
                }

                // Build list (only after placed voxels are loaded)
                this.voxelTool.buildList = [];
                if (Array.isArray(jsondata.buildList)) {
                    jsondata.buildList.forEach(voxel => {
                        const [x, y, z] = voxel.centers[0].split(',').map(Number);
                        let direction = new THREE.Vector3(0, 1, 0);
                        if (voxel.centers.length > 1) {
                            const [x2, y2] = voxel.centers[1].split(',').map(Number);
                            direction = new THREE.Vector3(x2 - x, y2 - y, 0).normalize();
                        }
                        const v = new Voxel(x, y, z, this.scene, voxel.type, true, direction, undefined, true);
                        v.toBuild = true;
                        v.centers = voxel.centers;
                        this.voxelTool.buildList.push(v);
                        // Track centers for overlap checking
                        v.centers.forEach(c => {
                            this.voxelTool.voxelCenters.add(c);
                            if (window.voxelCenters && typeof window.voxelCenters.add === "function") {
                                window.voxelCenters.add(c);
                            }
                        });
                    });
                }

            } catch (err) {
                console.error("Failed to load setup:", err);
            }
        });

        input.click();
    }

    // ==================== CLEAR ALL ====================
    clearAll() {
        // Remove all preview objects
        this.clearPreviewVoxel();
        this.clearPreviewRobot();
        this.clearPreviewSource();

        // Remove all placed voxels
        if (window.voxelMap && typeof window.voxelMap.clear === "function") {
            window.voxelMap.clear();
        }

        this.voxelTool.buildList = [];
        this.voxelTool.voxelCenters.clear();
        this.voxelTool.placed = [];
        
        // Remove all voxel meshes and hitboxes from the scene
        this.scene.children = this.scene.children.filter(obj => {
            if (obj.name === "voxel_hitbox" || obj.isMesh) {
                this.scene.remove(obj);
                return false;
            }
            return true;
        });

        // Remove all robots
        this.robotTool.placed.forEach(robot => {
            if (typeof robot.delete === "function") robot.delete();
        });
        this.robotTool.placed = [];
        this.robotTool.initialPos = [];

        // Remove all sources
        this.sourceTool.placed.forEach(source => {
            if (typeof source.delete === "function") source.delete();
        });
        this.sourceTool.placed = [];
        this.sourceTool.Pos = [];

        // Rebuild base layer
        this.createBaseLayer();
    }

    async resetAndReload(jsondata) {
        // Clear GUI DOM
        document.querySelector('.dg.main')?.remove();

        // Dynamically re-import scene module and reinitialize
        const { scene, camera, initScene } = await import('./scene.js');
        initScene(jsondata.baseLayer.scene_x, jsondata.baseLayer.scene_y);

        setTimeout(() => {
            const gui = new BuildingSoftwareGUI(scene, camera, jsondata.baseLayer.scene_x, jsondata.baseLayer.scene_y);
            gui.loadFromJSON(jsondata);
        }, 100);
    }
}

export async function buildVoxelStructureFromJsonUrl(url, scene) {
    try {
        const res = await fetch(url);
        const jsondata = await res.json();

        const voxelStructure = new Set(jsondata.map(voxel => ({
            type: voxel.type,
            centers: [...voxel.centers]
        })));

        voxelStructure.forEach(voxelEntry => {
            const { type, centers } = voxelEntry;
            if (centers.length === 0) return;
            const [x, y, z] = centers[0].split(',').map(Number);
            let direction = new THREE.Vector3(0, 1, 0);
            if (centers.length > 1) {
                const [x2, y2] = centers[1].split(',').map(Number);
                direction = new THREE.Vector3(x2 - x, y2 - y, 0).normalize();
            }

            const voxel = new Voxel(x, y, z, scene, type, true, direction, 0x0000ff, true);
            voxel.centers = centers;
            // window.voxelMap.add(voxel);

            window.gui.voxelTool.buildList.push(voxel);
            centers.forEach(c => window.gui.voxelTool.voxelCenters.add(c));
        });

    } catch (err) {
        console.error("❌ Failed to fetch and build voxel structure:", err);
    }
}
export async function buildStairsRobotSource(nbRobotFront, nbRobotBack, nbRobotLeft, nbRobotRight, structure, scene) {
    console.log("Building stairs robot source...");
    console.log("Number of robots on each face:", {
        front: nbRobotFront,
        back: nbRobotBack,
        left: nbRobotLeft,
        right: nbRobotRight
    });
    if (!structure || structure.length === 0) return;

    // ---- Bounds on bottom layer ----
    const allCenters = structure.flatMap(v => v.centers).map(c => c.split(',').map(Number));
    const minZ = Math.min(...allCenters.map(c => c[2]));
    const bottom = allCenters.filter(c => Math.abs(c[2] - minZ) < 1e-6);
    const xs = bottom.map(c => c[0]);
    const ys = bottom.map(c => c[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);

    // ---- Stair / voxel params (match your existing stacked 4×2) ----
    const stairType = 6;    // Stacked 4x2 in your code
    const stepZ    = 0.5;   // your half-layer vertical spacing
    const r2 = n => Number(n.toFixed(2));

    // Place one stacked 4×2 column at (sx,sy) starting at z, then fill down to ground.
    function placeStairColumn(sx, sy, z, dir) {
        const put = (zz) => {
            const v = new Voxel(sx, sy, zz, scene, stairType, true, dir, 0xf034e9, true);
            v.centers = [
                `${r2(sx)},${r2(sy)},${r2(zz)}`,
                `${r2(sx + dir.x)},${r2(sy + dir.y)},${r2(zz)}`,
                `${r2(sx)},${r2(sy)},${r2(zz + stepZ)}`,
                `${r2(sx + dir.x)},${r2(sy + dir.y)},${r2(zz + stepZ)}`
            ];
            window.gui.voxelTool.buildList.push(v);
            // Register centers like manual placement
            v.centers.forEach(c => {
              window.gui.voxelTool.voxelCenters.add(c);
              if (window.voxelCenters && typeof window.voxelCenters.add === "function") {
                window.voxelCenters.add(c);
              }
            });
        };
        put(z);
        for (let zz = z - 1; zz >= minZ; zz -= 1) put(zz);
    }

    // Build a ramp along one face, starting at origin, marching OUTWARD (not along face).
    function buildRamp(origin, out) {
        // Use outward direction for the stair march (match legacy function)
        const direction = out.clone();

        // Find the highest z just behind the first column
        const behindX = origin.x - direction.x;
        const behindY = origin.y - direction.y;
        const aligned = allCenters.filter(([cx, cy]) => Math.abs(cx - behindX) < 1e-6 && Math.abs(cy - behindY) < 1e-6);
        const maxZ = aligned.length ? Math.max(...aligned.map(c => c[2])) : Math.max(...allCenters.map(c => c[2]));

        // Starting point and Z (start slightly below top)
        let sx = origin.x;
        let sy = origin.y;
        let placeZ = maxZ - 3 * stepZ; // == 1.5 if stepZ=0.5
        let offsetSource = 0;
        let safety = 0;

        // March outward every layer: place a stacked 4x2 column, then drop one layer
        while (placeZ > minZ + 1e-6 && safety++ < 4000) {
            placeStairColumn(sx, sy, placeZ, direction);
            // Advance outward by two half-cells (== 1.0 in your grid), like legacy code
            sx += direction.x * 2;
            sy += direction.y * 2;
            placeZ -= 1;
            offsetSource += 2;
        }

        // Position the source one unit behind the last stair, identical to legacy logic
        const stairX = origin.x + direction.x * (offsetSource - 1);
        const stairY = origin.y + direction.y * (offsetSource - 1);
        const srcX = stairX + direction.x;
        const srcY = stairY + direction.y;
        const srcZ = minZ + stepZ; // 0.5
        const srcDir = new THREE.Vector3(direction.x, direction.y, 0);
        const source = new Source(srcX, srcY, srcZ, srcDir, scene, false);
        window.gui.sourceTool.placed.push(source);
        // Mirror manual placement bookkeeping
        if (window.gui.sourceTool && Array.isArray(window.gui.sourceTool.Pos)) {
            window.gui.sourceTool.Pos.push({ Pos: { x: srcX, y: srcY, z: srcZ }, Dir: srcDir });
        }

        // Place robot just beyond source, same offsets as legacy
        const originRobot = new THREE.Vector3(srcX, srcY, srcZ + stepZ).add(srcDir.clone().multiplyScalar(2));
        const targetRobot = originRobot.clone().sub(srcDir);
        const robot = new THREERobot(originRobot, targetRobot, new THREE.Vector3(0, 0, 1), scene);
        window.gui.robotTool.placed.push(robot);
        window.gui.robotTool.initialPos.push({ origin: originRobot, target: targetRobot });
    }

    // Face definitions matching legacy directions:
    // left:  (-1,0,0) from minX-1,minY; right: (1,0,0) from maxX+1,maxY
    // back:  (0,-1,0) from maxX,minY-1; front: (0,1,0) from minX,maxY+1
    const faces = {
      left: {
        bottomLeft:  { x: minX, y: minY - 1 },
        bottomRight: { x: maxX, y: minY - 1 },
        out: new THREE.Vector3(0, -1, 0)
      },
      right: {       
        bottomLeft:  { x: maxX, y: maxY + 1 },
        bottomRight: { x: minX, y: maxY + 1 },
        out: new THREE.Vector3(0,  1, 0)
      },
      back: {
        bottomLeft:  { x: maxX + 1, y: minY },
        bottomRight: { x: maxX + 1, y: maxY },
        out: new THREE.Vector3( 1, 0, 0)
      },
      front: {
        bottomLeft:  { x: minX - 1, y: maxY },
        bottomRight: { x: minX - 1, y: minY },
        out: new THREE.Vector3(-1, 0, 0)
      }
    };

      // Place one or two ramps per face depending on requested robot count
    function placeForFace(count, face) {
        if (count <= 0) return;
        // Always place at bottom-left first
        buildRamp(face.bottomLeft, face.out);
        if (count >= 2) {
            // Second at bottom-right
            buildRamp(face.bottomRight, face.out);
        }
    }

    placeForFace(nbRobotFront, faces.front);
    placeForFace(nbRobotBack,  faces.back);
    placeForFace(nbRobotLeft,  faces.left);
    placeForFace(nbRobotRight, faces.right);
}



export async function createElevatorSupport(nbRobotFront, nbRobotBack, nbRobotLeft, nbRobotRight, structure, scene) {
    // Cleaned: Only stacked 2x2 and stacked 4x2 voxels, alternating order every layer
    if (!structure || structure.length === 0) return;

    const allCenters = structure.flatMap(v => v.centers).map(c => c.split(',').map(Number));
    const minZ = Math.min(...allCenters.map(c => c[2]));
    const bottom = allCenters.filter(c => Math.abs(c[2] - minZ) < 1e-6);
    const xs = bottom.map(c => c[0]);
    const ys = bottom.map(c => c[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);

    const TYPE_4x2_STACKED = 6; // Stacked 4x2
    const TYPE_2x2_STACKED = 9; // Stacked 2x2
    const stepZ = 0.5;
    const r2 = n => Number(n.toFixed(2));

    // Helper: place voxel and compute correct centers for stacked 4x2 and 2x2
    function placeVoxel(sx, sy, zz, type, dir) {
        const v = new Voxel(sx, sy, zz, scene, type, true, dir, 0xf034e9, true);

        // Use the same stacked geometry conventions as manual placement
        const stepZ = 0.5;
        const r2 = n => Number(n.toFixed(2));

        const centers = [];
        if (type === 6) { // TYPE_4x2_STACKED
            centers.push(
            `${r2(sx)},${r2(sy)},${r2(zz)}`,
            `${r2(sx + dir.x)},${r2(sy + dir.y)},${r2(zz)}`,
            `${r2(sx)},${r2(sy)},${r2(zz + stepZ)}`,
            `${r2(sx + dir.x)},${r2(sy + dir.y)},${r2(zz + stepZ)}`
            );
        } else if (type === 9) { // TYPE_2x2_STACKED
            centers.push(
            `${r2(sx)},${r2(sy)},${r2(zz)}`,
            `${r2(sx)},${r2(sy)},${r2(zz + stepZ)}`
            );
        } else {
            // Fallback: single center
            centers.push(`${r2(sx)},${r2(sy)},${r2(zz)}`);
        }

        // Keep voxel instance in sync for visuals/debug
        v.centers = centers.slice();

        // Register centers globally so collision and coloring see them
        centers.forEach(c => {
            window.gui.voxelTool.voxelCenters.add(c);
            if (window.voxelCenters && typeof window.voxelCenters.add === "function") {
            window.voxelCenters.add(c);
            }
        });

        // CRITICAL: push plain entries the planner consumes (so robots get assigned and coloring works)
        window.gui.voxelTool.buildList.push({ type, centers: centers.slice() });

        // Optional: keep a separate list of supports for diagnostics/order
        if (Array.isArray(window.gui.voxelTool.supportBuildList)) {
            window.gui.voxelTool.supportBuildList.push({ type, centers: centers.slice() });
        }
    }

    // Place a support column with alternating stacked 2x2 and 4x2 voxels
    function placeSupportColumn(origin, out) {
        const dir = out.clone();
        const maxZ = Math.max(...allCenters.map(c => c[2]));
        const height = maxZ - minZ + stepZ;
        let placeZ = minZ;
        let even = true;
        placeZ += 1;

        while (placeZ < height - 1) {
            
            if (even) {
                // Even layer: 2x2 stacked first, then 4x2 stacked
                placeVoxel(origin.x, origin.y, placeZ, TYPE_2x2_STACKED, dir);
                // 4x2 stacked placed adjacent (1 voxel away in out direction)
                placeVoxel(origin.x + dir.x, origin.y + dir.y, placeZ, TYPE_4x2_STACKED, dir);
            } else {
                // Odd layer: 4x2 stacked first, then 2x2 stacked
                placeVoxel(origin.x, origin.y, placeZ, TYPE_4x2_STACKED, dir);
                // 2x2 stacked placed 2 voxels away in out direction
                placeVoxel(origin.x + dir.x * 2, origin.y + dir.y * 2, placeZ, TYPE_2x2_STACKED, dir);
            }
            even = !even;
            placeZ += 1;
        }
        // Source: place exactly 1 voxel behind outermost voxel in outward direction
        // Find outermost voxel position
        let srcX, srcY;
        if (!even) {
            // Last layer was even: last 4x2 at (origin.x + dir.x, origin.y + dir.y)
            srcX = origin.x + dir.x;
            srcY = origin.y + dir.y;
        } else {
            // Last layer was odd: last 2x2 at (origin.x + dir.x * 2, origin.y + dir.y * 2)
            srcX = origin.x + dir.x * 2;
            srcY = origin.y + dir.y * 2;
        }
        // Source is 1 voxel behind outermost voxel in outward direction
        const sourceX = origin.x;
        const sourceY = origin.y;
        const srcZ = minZ + stepZ;
        const srcDir = dir.clone();
        const source = new Source(sourceX, sourceY, srcZ, srcDir, scene, false);
        source.meta = source.meta || {};
        source.meta.supportType = "elevator";
        window.gui.sourceTool.placed.push(source);      
      if (window.gui.sourceTool && Array.isArray(window.gui.sourceTool.Pos)) {
            window.gui.sourceTool.Pos.push({ Pos: { x: sourceX, y: sourceY, z: srcZ }, Dir: srcDir });
        }
        // Robot spawn: 2 units further outward from source, facing inward
        const robotOrigin = new THREE.Vector3(sourceX, sourceY, srcZ + stepZ).add(dir.clone().multiplyScalar(2));
        const robotTarget = robotOrigin.clone().sub(dir);
        const robot = new THREERobot(robotOrigin, robotTarget, new THREE.Vector3(0, 0, 1), scene);
        window.gui.robotTool.placed.push(robot);
        window.gui.robotTool.initialPos.push({ origin: robotOrigin, target: robotTarget });
    }

    const faces = {
        left:  { bottomLeft: { x: minX, y: minY - 1 }, bottomRight: { x: maxX, y: minY - 1 }, out: new THREE.Vector3(0, -1, 0) },
        right: { bottomLeft: { x: maxX, y: maxY + 1 }, bottomRight: { x: minX, y: maxY + 1 }, out: new THREE.Vector3(0, 1, 0)  },
        back:  { bottomLeft: { x: maxX + 1, y: minY }, bottomRight: { x: maxX + 1, y: maxY }, out: new THREE.Vector3(1, 0, 0)  },
        front: { bottomLeft: { x: minX - 1, y: maxY }, bottomRight: { x: minX - 1, y: minY }, out: new THREE.Vector3(-1, 0, 0) }
    };

    function placeForFace(count, face) {
        if (count <= 0) return;
        placeSupportColumn(face.bottomLeft, face.out);
        if (count >= 2) placeSupportColumn(face.bottomRight, face.out);
    }

    placeForFace(nbRobotFront, faces.front);
    placeForFace(nbRobotBack, faces.back);
    placeForFace(nbRobotLeft, faces.left);
    placeForFace(nbRobotRight, faces.right);
}


// export async function buildStairsRobotSource(nbRobot, stairSize, side, structure, scene) {
//     if (!structure || structure.length === 0) return;

//     // Extract only voxels on the lowest Z layer
//     const layerZ = Math.min(...structure.flatMap(v => v.centers.map(c => parseFloat(c.split(',')[2]))));
//     const bottomVoxels = structure.filter(v =>
//         v.centers.some(c => parseFloat(c.split(',')[2]) === layerZ)
//     );

//     // Compute positions only from bottom-layer voxels
//     const bottomCenters = bottomVoxels.flatMap(v => v.centers.map(c => c.split(',').map(Number)));
//     const xs = bottomCenters.map(c => c[0]);
//     const ys = bottomCenters.map(c => c[1]);
//     const zs = bottomCenters.map(c => c[2]);

//     const minX = Math.min(...xs), maxX = Math.max(...xs);
//     const minY = Math.min(...ys), maxY = Math.max(...ys);
//     const minZ = Math.min(...zs);

//     // Updated robot positions and directions: align with bottom-left voxel of each face
//     const positions = [];
//     const directions = [];

//     if (nbRobot >= 1) {
//         positions.push({ x: minX - 1, y: minY });  // Left face, bottom
//         directions.push(new THREE.Vector3(-1, 0, 0)); // facing right
//     }
//     if (nbRobot >= 2) {
//         positions.push({ x: maxX + 1, y: maxY });  // Right face, top
//         directions.push(new THREE.Vector3(1, 0, 0)); // facing left
//     }
//     if (nbRobot >= 3) {
//         positions.push({ x: maxX, y: minY - 1 });  // Bottom face, right
//         directions.push(new THREE.Vector3(0, -1, 0)); // facing up
//     }
//     if (nbRobot >= 4) {
//         positions.push({ x: minX, y: maxY + 1 });  // Top face, left
//         directions.push(new THREE.Vector3(0, 1, 0)); // facing down
//     }

//     // Restore stair-building logic for each robot/face
//     for (let i = 0; i < positions.length; i++) {
//         let { x, y } = positions[i];
//         let direction = directions[i];

//         // Build stairs from highest Z above entry point
//         const alignedCenters = structure
//             .flatMap(v => v.centers)
//             .map(c => c.split(',').map(Number))
//             .filter(([cx, cy]) => Math.abs(cx - (x - direction.x)) < 0.01 && Math.abs(cy - (y - direction.y)) < 0.01);

//         const sortedZs = alignedCenters.map(c => c[2]).sort((a, b) => b - a);
//         const maxZ = sortedZs.length > 0 ? sortedZs[0] : Math.max(...structure.flatMap(v => v.centers.map(c => parseFloat(c.split(',')[2]))));

//         let stairX = x;
//         let stairY = y;
//         let placeZ = maxZ - 1.5;
//         let offsetSource = 0;

//         while (placeZ > minZ) {
//             const stairVoxel = new Voxel(stairX, stairY, placeZ, scene, 6, true, direction, 0xf034e9, true);
//             stairVoxel.centers = [
//                 `${Number(stairX).toFixed(2)},${Number(stairY).toFixed(2)},${Number(placeZ).toFixed(2)}`,
//                 `${Number(stairX + direction.x).toFixed(2)},${Number(stairY + direction.y).toFixed(2)},${Number(placeZ).toFixed(2)}`,
//                 `${Number(stairX).toFixed(2)},${Number(stairY).toFixed(2)},${Number(placeZ + 0.5).toFixed(2)}`,
//                 `${Number(stairX + direction.x).toFixed(2)},${Number(stairY + direction.y).toFixed(2)},${Number(placeZ + 0.5).toFixed(2)}`
//             ];
//             window.gui.voxelTool.buildList.push(stairVoxel);

//             let placeZbellow = placeZ;
//             while (placeZbellow > minZ) {
//                 placeZbellow -= 1;
//                 const Voxelbellow = new Voxel(stairX, stairY, placeZbellow, scene, 6, true, direction, 0xf034e9, true);
//                 Voxelbellow.centers = [
//                     `${Number(stairX).toFixed(2)},${Number(stairY).toFixed(2)},${Number(placeZbellow).toFixed(2)}`,
//                     `${Number(stairX + direction.x).toFixed(2)},${Number(stairY + direction.y).toFixed(2)},${Number(placeZbellow).toFixed(2)}`,
//                     `${Number(stairX).toFixed(2)},${Number(stairY).toFixed(2)},${Number(placeZbellow + 0.5).toFixed(2)}`,
//                     `${Number(stairX + direction.x).toFixed(2)},${Number(stairY + direction.y).toFixed(2)},${Number(placeZbellow + 0.5).toFixed(2)}`
//                 ];
//                 window.gui.voxelTool.buildList.push(Voxelbellow);
//             }

//             stairX += direction.x * 2;
//             stairY += direction.y * 2;
//             placeZ -= 1;
//             offsetSource += 2;
//         }

//         // Correct source position behind last stair
//         stairX = x + direction.x * (offsetSource - 1);
//         stairY = y + direction.y * (offsetSource - 1);
//         const srcX = stairX + direction.x;
//         const srcY = stairY + direction.y;
//         const srcZ = minZ + 0.5;

//         const source = new Source(srcX, srcY, srcZ, direction, scene, false);
//         window.gui.sourceTool.placed.push(source);

//         const origin = new THREE.Vector3(srcX, srcY, srcZ + 0.5).add(direction.clone().multiplyScalar(2));
//         const target = origin.clone().sub(direction.clone());
//         const robot = new THREERobot(origin, target, new THREE.Vector3(0, 0, 1), scene);
//         window.gui.robotTool.placed.push(robot);
//         window.gui.robotTool.initialPos.push({ origin, target });
//     }
// }
    /**
     * Export window._benchmark as a timestamped JSON file.
     */
    function downloadBenchmarkJSON() {
        try {
            const data = (typeof window !== "undefined" && window._benchmark) ? window._benchmark : {};
            const json = JSON.stringify(data, null, 2);
            const blob = new Blob([json], { type: "application/json" });
            const a = document.createElement("a");
            const ts = new Date().toISOString().replace(/[:.]/g, "-");
            a.href = URL.createObjectURL(blob);
            a.download = `benchmark-${ts}.json`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(a.href);
        } catch (e) {
            console.error("Failed to export benchmark JSON:", e);
            alert("Couldn't export benchmark JSON. See console for details.");
        }
    }