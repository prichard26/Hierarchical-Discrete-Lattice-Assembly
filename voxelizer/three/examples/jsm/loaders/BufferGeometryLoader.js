// Compatible with global THREE only (no ES module imports)
import * as THREE from "../../../../three/build/three.module.min.js";

const BufferGeometryLoader = function (manager) {
	this.manager = manager !== undefined ? manager : THREE.DefaultLoadingManager;
};

BufferGeometryLoader.prototype = {

	load: function (url, onLoad, onProgress, onError) {

		const scope = this;
		const loader = new THREE.FileLoader(scope.manager);
		loader.setPath(scope.path || '');
		loader.setWithCredentials(scope.withCredentials || false);

		loader.load(url, function (text) {
			try {
				onLoad(scope.parse(JSON.parse(text)));
			} catch (e) {
				if (onError) {
					onError(e);
				} else {
					console.error(e);
				}
				scope.manager.itemError(url);
			}
		}, onProgress, onError);

	},

	parse: function (json) {
		const geometry = json.isInstancedBufferGeometry ?
			new THREE.InstancedBufferGeometry() :
			new THREE.BufferGeometry();

		// Load index if present
		if (json.data.index) {
			const indexArray = getTypedArray(json.data.index.type, json.data.index.array);
			geometry.setIndex(new THREE.BufferAttribute(indexArray, 1));
		}

		// Load attributes
		const attributes = json.data.attributes;
		for (const key in attributes) {
			const attr = attributes[key];
			const array = getTypedArray(attr.type, attr.array);
			const bufferAttr = new THREE.BufferAttribute(array, attr.itemSize, attr.normalized);
			geometry.setAttribute(key, bufferAttr);
		}

		// Optional bounding sphere
		if (json.data.boundingSphere) {
			const center = new THREE.Vector3().fromArray(json.data.boundingSphere.center);
			geometry.boundingSphere = new THREE.Sphere(center, json.data.boundingSphere.radius);
		}

		return geometry;
	}

};

// Helper: parse typed arrays
function getTypedArray(type, array) {
	switch (type) {
		case 'Float32Array': return new Float32Array(array);
		case 'Float64Array': return new Float64Array(array);
		case 'Uint32Array': return new Uint32Array(array);
		case 'Int32Array': return new Int32Array(array);
		case 'Uint16Array': return new Uint16Array(array);
		case 'Int16Array': return new Int16Array(array);
		case 'Uint8Array': return new Uint8Array(array);
		case 'Int8Array': return new Int8Array(array);
		default: throw new Error('Unsupported array type: ' + type);
	}
}

export { BufferGeometryLoader };