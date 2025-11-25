// ─── External Libraries ───────────────────────
import * as THREE from './three/build/three.module.min.js';
import { STLLoader } from './three/examples/jsm/loaders/STLLoader.js';

// ─── Project Modules: Core ─────────────────────
import { scene, initScene } from './scene.js';
import { translate } from './translater.js';
import { computeVoxelizationStats } from './perfomance.js';

// ─── Project Modules: Voxel Patterns ───────────
import {
  extractVoxelCenters,
  findOnly2x2, findOnly2x3, findOnly2x4,
  findAllPatterns,
  generateConstructibleVoxelMap
} from './Voxelizer.js';

import { Voxelizer } from './newVoxelizer.js';

const state = {
  scale: 1,
  voxelSize: 65,
  voxelCenters: null,
  geometryCache: null,
  stairHighlightMesh: null,
  allPatternObjects: [],
  currentVoxelMap: [],
  outlinesVisible: false
};

window.voxelSize = state.voxelSize;
window.voxelMaptoSend = [];

// Initialize Scene
initScene();

// ─────────────────────────────────────────────────────────────────────────────
// UI Interaction
// ─────────────────────────────────────────────────────────────────────────────

function onEvaluateClick() {
  if (!state.geometryCache || !state.voxelCenters) {
    alert("Voxelization must be done first.");
    return;
  }
  const stats = computeVoxelizationStats(state.geometryCache, state.voxelCenters, state.currentVoxelMap);
  console.log("Voxelization Performance Stats:");
  console.log(`- Mesh Volume: ${stats.meshVolume.toFixed(8)} m³`);
  console.log(`- Voxelized Volume: ${stats.voxelizedVolume.toFixed(8)} m³`);
  console.log(`- Number of Centers: ${stats.voxelCount}`);
  console.log(`- Number of Typed Voxels: ${stats.typedVoxelCount}`);
  console.log(`- Precision Ratio: ${(stats.precision * 100).toFixed(8)}%`);
  console.log(`- Optimal Voxelized Volume: ${stats.optimalVoxelizedVolume.toFixed(8)} m³`);
  console.log(`- Optimal Precision Ratio: ${(stats.optimalprecision * 100).toFixed(8)}%`);
  alert(`Precision: ${(stats.precision * 100).toFixed(8)}%\n#Typed Voxels: ${stats.typedVoxelCount}`);
}

function onVoxelizeClick() {
  if (!state.geometryCache) return console.warn("No geometry to voxelize.");
  state.voxelCenters = extractVoxelCenters(state.geometryCache, state.voxelSize);
  console.log(" Voxel Centers Extracted:", state.voxelCenters);
  renderVoxelCenters();
}

function toggleOutlines() {
  if (!state.voxelCenters?.length) return alert("Voxel centers not loaded.");

  const meshObject = scene.children.find(obj => obj.isMesh && obj.material?.transparent);

  if (!state.outlinesVisible) {
    showVoxelOutlines();
    document.getElementById('toggleVoxelOutlines').textContent = "Hide Voxel Outlines";
    if (meshObject) meshObject.visible = false;
  } else {
    state.allPatternObjects.forEach(obj => {
      if (obj.userData?.pattern === "voxelOutline") {
        obj.visible = false;
      }
    });
    document.getElementById('toggleVoxelOutlines').textContent = "Show Voxel Outlines";
    if (meshObject) meshObject.visible = true;
  }

  state.outlinesVisible = !state.outlinesVisible;
}

function launchSimulation() {
  if (!window.voxelMaptoSend || window.voxelMaptoSend.length === 0) {
    alert("Voxel map is empty. Please voxelize first.");
    return;
  }

  const raw = window.voxelMaptoSend || [];
  const structure = raw.map(({ type = 0, centers = [], singles = [] }) => ({
    type,
    centers: (centers.length ? centers : singles).map(pt =>
      typeof pt === 'string' ? pt : `${pt.x},${pt.y},${pt.z}`
    )
  }));

  const valid = structure.filter(entry => entry.centers.length > 0);
  if (valid.length === 0) {
    alert("No valid structure to simulate. Please run voxelization and translation.");
    return;
  }

  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;

  structure.forEach(({ centers }) => {
    centers.forEach(center => {
      const [x, y] = center.split(',').map(Number);
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    });
  });

  const blob = new Blob([JSON.stringify(structure, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);

  const numRobotsFront = parseInt(document.getElementById("numRobotsFront").value);
  const numRobotsBack  = parseInt(document.getElementById("numRobotsBack").value);
  const numRobotsLeft  = parseInt(document.getElementById("numRobotsLeft").value);
  const numRobotsRight = parseInt(document.getElementById("numRobotsRight").value);
  
  const supportType = (document.getElementById("supportType")?.value) || "stairs";    
  
  console.log("Robots per face:", { numRobotsFront, numRobotsBack, numRobotsLeft, numRobotsRight });

  const link = document.createElement("a");
  link.href = `../Simulation/Simulation.html?structureUrl=${encodeURIComponent(url)}&xmin=${minX}&xmax=${maxX}&ymin=${minY}&ymax=${maxY}&numRobotsFront=${numRobotsFront}&numRobotsBack=${numRobotsBack}&numRobotsLeft=${numRobotsLeft}&numRobotsRight=${numRobotsRight}&supportType=${encodeURIComponent(supportType)}`;
  link.target = "_blank";
  link.click();

  console.log("▶ Sending to simulation:");
  console.log("Structure:", structure);
  console.log("Bounding Box:", { minX, maxX, minY, maxY });
} 
  function setupUIHandlers() {
    document.getElementById('evaluateVoxelization').addEventListener('click', onEvaluateClick);
    document.getElementById('voxelizeButton').addEventListener('click', onVoxelizeClick);
    document.getElementById('patternType').addEventListener('change', showPatterns);
    document.getElementById('toggleVoxelOutlines').addEventListener('click', toggleOutlines);
    document.getElementById("simulateAssemblyButton").addEventListener("click", launchSimulation);

    function bindRobotSlider(id, valueId, face) {
      const el = document.getElementById(id);
      const val = document.getElementById(valueId);
      if (!el || !val) return;
      el.addEventListener('input', e => { val.textContent = e.target.value; });
      el.addEventListener('mouseenter', () => highlightStairFace(face));
      el.addEventListener('mouseleave', () => {
        if (state.stairHighlightMesh) { scene.remove(state.stairHighlightMesh); state.stairHighlightMesh = null; }
      });
    }

  bindRobotSlider('numRobotsFront', 'numRobotsFrontValue', 'front');
  bindRobotSlider('numRobotsBack',  'numRobotsBackValue',  'back');
  bindRobotSlider('numRobotsLeft',  'numRobotsLeftValue',  'left');
  bindRobotSlider('numRobotsRight', 'numRobotsRightValue', 'right');
  }
setupUIHandlers();

// ─────────────────────────────────────────────────────────────────────────────
// Visualization Utilities
// ─────────────────────────────────────────────────────────────────────────────

// Render small blue spheres at each voxel center
function renderVoxelCenters() {
  state.voxelCenters.forEach(pt => {
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(5, 12, 12),
      new THREE.MeshStandardMaterial({ color: 0x0000ff })
    );
    sphere.position.copy(pt);
    scene.add(sphere);
  });
}

// Render wireframe voxel outlines
function showVoxelOutlines() {
  if (!state.currentVoxelMap?.length) return;

  const nameToColorBase = new Map();
  let colorIndex = 0;

  state.currentVoxelMap.forEach(({ name }) => {
    const patternName = name || "type0";
    if (!nameToColorBase.has(patternName)) {
      let hue;
      if (patternName === "2x4x2") {
        hue = 0.55 + (colorIndex * 0.03) % 0.1; // range: 0.55–0.65 for soft blues
      } else {
        hue = (colorIndex * 0.2) % 1;
      }
      nameToColorBase.set(patternName, hue);
      colorIndex++;
    }
  });


  const bluePalette = [
    new THREE.Color("#03e2ff"),
    new THREE.Color("#1e00a5"),
    new THREE.Color("#847ad8"),
    new THREE.Color("#339CFF"),
    new THREE.Color("#024e9a"),
    new THREE.Color("#3caed4"),
    new THREE.Color("#5fb2d0"),
    new THREE.Color("#c6c1edff")
  ];

  const colorVariation = (patternName, variationIndex) => {
    return bluePalette[variationIndex % bluePalette.length];
  };

  let variationCount = {};

  state.currentVoxelMap.forEach(({ name, singles }) => {
    const patternName = name || "type0";
    const hue = nameToColorBase.get(patternName);
    variationCount[patternName] = (variationCount[patternName] || 0) + 1;
    const vIndex = variationCount[patternName] % 5;

    const color = colorVariation(patternName, vIndex);

    singles.forEach(pt => {
      const cube = new THREE.Mesh(
        new THREE.BoxGeometry(64.5, 64.5, 64.5),
        new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.75 })
      );
      cube.position.copy(pt);
      cube.userData.pattern = "voxelOutline";
      scene.add(cube);
      state.allPatternObjects.push(cube);
    });
  });
}

// Detect and show patterns
function showPatterns() {
  clearPatterns();

  const selected = document.getElementById('patternType').value;
  const patternFunc = patternFunctions[selected];

  if (!patternFunc) {
    console.warn(`Unknown pattern type: ${selected}`);
    return;
  }

  const { voxelMap } = patternFunc(state.voxelCenters, state.voxelSize);
  window.voxelMaptoSend = translate(voxelMap);
  state.currentVoxelMap = voxelMap; // store globally for reuse
  generatePatternButtons(voxelMap);
  renderPatternMap(voxelMap);
}

// Remove all previously displayed patterns
function clearPatterns() {
  state.allPatternObjects.forEach(obj => scene.remove(obj));
  state.allPatternObjects.length = 0;
}

// Generate buttons to toggle visibility of each pattern
function generatePatternButtons(voxelMap) {
  const group = document.getElementById('patternButtonGroup');
  const panel = document.getElementById('patternButtons');

  group.innerHTML = '';
  const types = new Set(voxelMap.map(v => v.name || "type0"));

  const nameToColorBase = new Map();
  let colorIndex = 0;

  types.forEach(name => {
    const hue = (colorIndex * 0.2) % 1;
    nameToColorBase.set(name, hue);
    colorIndex++;
  });

  types.forEach(name => {
    const btn = document.createElement('button');
    btn.textContent = name;
    btn.className = 'patternFilterBtn';
    btn.dataset.type = name;
    btn.onclick = () => {
      state.allPatternObjects.forEach(obj => {
        obj.visible = obj.userData?.pattern === name || name === 'all';
      });
    };

    const hue = nameToColorBase.get(name);
    const baseColor = new THREE.Color().setHSL(hue, 0.8, 0.5);
    btn.style.backgroundColor = `#${baseColor.getHexString()}`;
    btn.style.color = '#fff';

    group.appendChild(btn);
  });

  const allBtn = document.createElement('button');
  allBtn.textContent = "Show All";
  allBtn.className = 'patternFilterBtn';
  allBtn.dataset.type = "all";
  allBtn.onclick = () => state.allPatternObjects.forEach(obj => obj.visible = true);
  group.appendChild(allBtn);

  panel.style.display = 'block';
}

// Render voxelMap objects into the scene
function renderPatternMap(voxelMap) {
  voxelMap.forEach(({ name, singles, type }) => {
    const patternName = name || "type0";

    if (type === 'gadget' || type === 'module') {
      const geometry = new THREE.BufferGeometry().setFromPoints(singles);
      const line = new THREE.LineLoop(
        geometry,
        new THREE.LineBasicMaterial({ color: type === 'gadget' ? 0xffff00 : 0x00ff00 })
      );
      line.userData.pattern = patternName;
      scene.add(line);
      state.allPatternObjects.push(line);
    }

    if (type === 'single') {
      singles.forEach(pt => {
        const cube = new THREE.Mesh(
          new THREE.BoxGeometry(12.5, 12.5, 12.5),
          new THREE.MeshStandardMaterial({ color: 0xff0000 })
        );
        cube.position.copy(pt);
        cube.userData.pattern = patternName;
        scene.add(cube);
        state.allPatternObjects.push(cube);
      });
    }
  });
}

// Load STL file and parse it
function loadSTL(buffer) {
  const loader = new STLLoader();
  const geometry = loader.parse(buffer);

  geometry.scale(state.scale, state.scale, state.scale);
  geometry.computeBoundingBox();

  const bbox = geometry.boundingBox;
  geometry.translate(0, 0, -bbox.min.z);

  state.geometryCache = geometry;

  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshStandardMaterial({
      color: 0x001fff,
      opacity: 0.3,
      transparent: true,
      side: THREE.DoubleSide
    })
  );

  scene.add(mesh);
}

window.loadSTL = loadSTL;

// ─────────────────────────────────────────────────────────────────────────────
// File Upload
// ─────────────────────────────────────────────────────────────────────────────

const welcomeScreen = document.getElementById("welcomeScreen");
const mainInterface = document.getElementById("mainInterface");
const uploadStart = document.getElementById("uploadStart");

function enableInterface() {
  welcomeScreen.style.display = "none";
  mainInterface.style.display = "block";
  const panelScript = document.createElement("script");
  panelScript.src = "panel-controls.js";
  document.body.appendChild(panelScript);
}

function loadFromBuffer(buffer) {
  enableInterface();
  loadSTL(buffer);
}

uploadStart.addEventListener("change", (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (ev) => loadFromBuffer(ev.target.result);
  reader.readAsArrayBuffer(file);
});

document.querySelectorAll(".mesh-card").forEach(card => {
  card.addEventListener("click", () => {
    const meshPath = card.dataset.src;
    if (!meshPath) return;
    fetch(meshPath)
      .then(res => res.arrayBuffer())
      .then(loadFromBuffer);
  });
});

// ─────────────────────────────────────────────────────────────────────────────
// Stair face highlight hover handlers
// ─────────────────────────────────────────────────────────────────────────────

function highlightStairFace(direction) {
  if (!state.geometryCache) return;
  if (state.stairHighlightMesh) {
    scene.remove(state.stairHighlightMesh);
    state.stairHighlightMesh = null;
  }

  const bbox = state.geometryCache.boundingBox;
  const centerZ = (bbox.min.z + bbox.max.z) / 2;
  const sizeZ = bbox.max.z - bbox.min.z;

  const material = new THREE.MeshBasicMaterial({
    color: 0xff0000,
    transparent: true,
    opacity: 0.25,
    side: THREE.DoubleSide,
    depthWrite: false,
  });

  const planeSize = Math.max(bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y);
  const plane = new THREE.PlaneGeometry(planeSize, sizeZ);
  state.stairHighlightMesh = new THREE.Mesh(plane, material);

  switch (direction) {
    case "front":
      state.stairHighlightMesh.rotation.y = Math.PI/2;
      state.stairHighlightMesh.position.set(bbox.min.x, (bbox.min.y + bbox.max.y) / 2, centerZ);
      break;
    case "back":
      state.stairHighlightMesh.rotation.y =  Math.PI/2;
      state.stairHighlightMesh.position.set(bbox.max.x, (bbox.min.y + bbox.max.y) / 2, centerZ);
      break;
    case "left":
      state.stairHighlightMesh.rotation.x = -Math.PI / 2;
      state.stairHighlightMesh.position.set((bbox.min.x + bbox.max.x) / 2, bbox.min.y, centerZ);
      break;
    case "right":
      state.stairHighlightMesh.rotation.x = Math.PI / 2;
      state.stairHighlightMesh.position.set((bbox.min.x + bbox.max.x) / 2, bbox.max.y, centerZ);
      break;
  }

  scene.add(state.stairHighlightMesh);
}

const patternFunctions = {
  Only2x2: findOnly2x2,
  Only2x3: findOnly2x3,
  Only2x4: findOnly2x4,
  AllHierarchical: findAllPatterns,
  Constructible: (points, voxelSize) => generateConstructibleVoxelMap(points, voxelSize, ["2x4x2", "2x4", "2x3", "2x2"]),
  Voxelizer: Voxelizer
};