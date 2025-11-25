import * as THREE from "./three/build/three.module.min.js";
import { OrbitControls } from "./three/examples/jsm/controls/OrbitControls.js";

var container, scene, camera, renderer;
export{scene, camera, renderer};

export function initScene(xmin = 0, xmax = 10, ymin = 0, ymax = 10) {
    // Expand bounds by 1 unit on each side
    xmin -= 1;
    xmax += 1;
    ymin -= 1;
    ymax += 1;

    // Compute center of scene
    const centerX = (xmin + xmax) / 2;
    const centerY = (ymin + ymax) / 2;
    const center = new THREE.Vector3(centerX, centerY, 0);
	
	container = document.getElementById( 'webgl' );

	renderer = new THREE.WebGLRenderer({
		antialias: true,
		preserveDrawingBuffer: false,
	});
	renderer.sortObjects = false
	// renderer.shadowMap.enabled = true;
	// renderer.shadowMap.type = THREE.PCFSoftShadowMap;
	
	renderer.setPixelRatio(window.devicePixelRatio);
	renderer.setSize(window.innerWidth, window.innerHeight);
	renderer.setClearColor(0xdddddd);
	
	renderer.physicallyCorrectLights = true;
	renderer.outputEncoding = THREE.sRGBEncoding;
	renderer.toneMapping = THREE.ACESFilmicToneMapping;
	renderer.toneMappingExposure = 1.0;


	container.appendChild(renderer.domElement);

	scene = new THREE.Scene();
	// scene.background = new THREE.Color(0xbfd1e5);
	scene.background = new THREE.Color(0xffffff);

	scene.fog = new THREE.Fog(0xbfd1e5, 5, 200);

	const hemiLight = new THREE.HemisphereLight(0xffffff, 0x8d8d8d, 3);
	hemiLight.position.set(0, 100, 0);
	scene.add(hemiLight);

	const dirLight = new THREE.DirectionalLight(0xffffff, 4);
	dirLight.position.set(0, 12.5, 12.5);
	dirLight.castShadow = true;
	dirLight.shadow.camera.top = 50;
	dirLight.shadow.camera.bottom = -25;
	dirLight.shadow.camera.left = -25;
	dirLight.shadow.camera.right = 25;
	dirLight.shadow.camera.near = 0.1;
	dirLight.shadow.camera.far = 200;
	dirLight.shadow.radius = 3;
	dirLight.shadow.blurSamples = 8;
	dirLight.shadow.mapSize.set(1024, 1024);
	scene.add(dirLight);

	const planeGeometry = new THREE.BoxGeometry((xmax - xmin), (ymax - ymin), 0.5);
	const planeMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff });
	const floor = new THREE.Mesh(planeGeometry, planeMaterial);
	floor.name = "floor";
	floor.position.z = -0.25;
	floor.position.x = xmin + (xmax - xmin) / 2;
	floor.position.y = ymin + (ymax - ymin) / 2;
	floor.receiveShadow = true;
	scene.add(floor);

	// Add a small red cube at the center of the grid
	const cubeGeometry = new THREE.BoxGeometry(0.1, 0.1, 0.1);
	const cubeMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
	const redCube = new THREE.Mesh(cubeGeometry, cubeMaterial);
	redCube.position.set(center.x, center.y, 0); // placed at center of grid
	redCube.castShadow = true;
	scene.add(redCube);

	camera = new THREE.PerspectiveCamera(35, window.innerWidth / window.innerHeight, 1, 10000)
	camera.up.set(0, 0, 1);
  	camera.position.set(center.x + 10, center.y + 10, 10);
  	camera.lookAt(center);
  	camera.userData.center = center;
  	scene.add(camera);

	var cameraControls = new OrbitControls(camera, renderer.domElement);
	cameraControls.target.copy(camera.userData.center);
	cameraControls.update();
	cameraControls.addEventListener('change', () => renderer.render(scene, camera));
	cameraControls.enableDamping = true;
	cameraControls.dampingFactor = 0.95;
	// Add helper axis
	// const axesHelper = new THREE.AxesHelper(5);
	// scene.add(axesHelper);
  
	function onWindowResize() 
	{
		camera.aspect = window.innerWidth / window.innerHeight;
		camera.updateProjectionMatrix();
	
		renderer.setSize(window.innerWidth, window.innerHeight);
		renderer.render(scene, camera);
	}
	
	window.addEventListener('resize', onWindowResize, false);

	// createFullGrid(xmin, xmax, ymin, ymax, 1);
	animate();
}

function createFullGrid(xmin, xmax, ymin, ymax, step) {
    const gridGeometry = new THREE.BufferGeometry();
    const gridVertices = [];
    const gridColors = [];

    const majorLineColor = new THREE.Color(0x000000);
    const minorLineColor = new THREE.Color(0x000000);
    const gridHeights = [0.01, 0.51];

    for (const z of gridHeights) {
        for (let i = xmin+1; i <= xmax-1; i += step) {
            const isMinorLine = ((i * 2) % 2 !== 0);
            gridVertices.push(i, ymin+1, z, i, ymax-1, z);
            const color = isMinorLine ? minorLineColor : majorLineColor;
            gridColors.push(color.r, color.g, color.b, color.r, color.g, color.b);
        }

        for (let i = ymin+1; i <= ymax-1; i += step) {
            const isMinorLine = ((i * 2) % 2 !== 0);
            gridVertices.push(xmin+1, i, z, xmax-1, i, z);
            const color = isMinorLine ? minorLineColor : majorLineColor;
            gridColors.push(color.r, color.g, color.b, color.r, color.g, color.b);
        }
    }

    gridGeometry.setAttribute('position', new THREE.Float32BufferAttribute(gridVertices, 3));
    gridGeometry.setAttribute('color', new THREE.Float32BufferAttribute(gridColors, 3));
    const gridMaterial = new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 4 });
    const gridLines = new THREE.LineSegments(gridGeometry, gridMaterial);
    scene.add(gridLines);
}


function animate() {
    requestAnimationFrame( animate );

    render();
}

function render(now) {
	renderer.render(scene, camera);
	
	
}