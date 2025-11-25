import * as THREE from './three/build/three.module.min.js';
import { OrbitControls } from './three/examples/jsm/controls/OrbitControls.js';

let scene, camera, renderer;

export function initScene(containerId = null) {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff); // White background

  camera = new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    0.1,
    10000000
  );
  camera.position.set(1000, -1000, 1000);
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);

  const container = containerId
    ? document.getElementById(containerId)
    : document.body;

  if (container) {
    container.appendChild(renderer.domElement);
  }

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0);
  controls.update();

  const light = new THREE.DirectionalLight(0xffffff, 1);
  light.position.set(-10, -5, 10);
  scene.add(light);
  scene.add(new THREE.AmbientLight(0x404040));

  const axesHelper = new THREE.AxesHelper(5);
  scene.add(axesHelper);

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  animate();
}

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}

export { scene, camera, renderer };