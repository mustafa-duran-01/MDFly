// ---------------------------------------------------------
// WebSocket Connection Setup
// ---------------------------------------------------------
const wsStatus = document.getElementById('ws-status');
let socket = null;

function connectWS() {
    socket = new WebSocket('ws://localhost:8765');

    socket.onopen = () => {
        wsStatus.textContent = 'ONLINE';
        wsStatus.className = 'status-label online';
        wsStatus.previousElementSibling.className = 'status-indicator online';
    };

    socket.onclose = () => {
        wsStatus.textContent = 'DISCONNECTED';
        wsStatus.className = 'status-label offline';
        wsStatus.previousElementSibling.className = 'status-indicator offline';
        setTimeout(connectWS, 1000); // Reconnect after 1 second
    };

    socket.onmessage = (event) => {
        const data = JSON.parse(event.data);
        updateTelemetry(data);
        updateThreeScene(data);
        updateCameraFeed(data);
    };
}

// ---------------------------------------------------------
// Telemetry Updates
// ---------------------------------------------------------
function updateTelemetry(data) {
    document.getElementById('val-airspeed').textContent = data.Va.toFixed(1);
    document.getElementById('val-altitude').textContent = (-data.position_d).toFixed(1);
    document.getElementById('val-roll').textContent = (data.roll * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('val-pitch').textContent = (data.pitch * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('val-yaw').textContent = (data.yaw * 180 / Math.PI).toFixed(1) + '°';
    
    // RPM from motor speed: omega * 60 / (2 * pi)
    const rpm = Math.round(data.motor_omega * 30.0 / Math.PI);
    document.getElementById('val-rpm').textContent = rpm;

    document.getElementById('val-energy-tot').textContent = data.energy_total.toFixed(0) + ' J';
    document.getElementById('val-energy-pot').textContent = data.energy_potential.toFixed(0) + ' J';
    document.getElementById('val-momentum-lin').textContent = data.momentum_linear_magnitude.toFixed(1) + ' N·s';
    document.getElementById('val-momentum-ang').textContent = data.momentum_angular_magnitude.toFixed(3) + ' kg·m²/s';
    document.getElementById('val-elevator').textContent = (data.elevator * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('val-throttle').textContent = (data.throttle * 100.0).toFixed(0) + '%';

    if (data.position_n !== undefined) {
        document.getElementById('val-gps-true').textContent = data.position_n.toFixed(1) + ' m';
    }
    if (data.gps_pos_n_noisy !== undefined) {
        document.getElementById('val-gps-noisy').textContent = data.gps_pos_n_noisy.toFixed(1) + ' m';
    }
}

// ---------------------------------------------------------
// Three.js 3D Flight Scene Initialization
// ---------------------------------------------------------
const container = document.getElementById('threejs-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0c16);
scene.fog = new THREE.FogExp2(0x0a0c16, 0.0015);

const camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 5000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.shadowMap.enabled = true;
container.appendChild(renderer.domElement);

// FPV Camera & WebGL Renderer setup
const fpvCanvas = document.getElementById('fpv-canvas');
const fpvRenderer = new THREE.WebGLRenderer({ canvas: fpvCanvas, antialias: true });
fpvRenderer.setSize(fpvCanvas.clientWidth, fpvCanvas.clientHeight);
fpvRenderer.shadowMap.enabled = true;
const fpvCamera = new THREE.PerspectiveCamera(73, 4 / 3, 0.1, 2000);

// Lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(100, 300, 100);
dirLight.castShadow = true;
scene.add(dirLight);

// Sky grid/stars helper
const starsGeometry = new THREE.BufferGeometry();
const starsCount = 500;
const starPositions = new Float32Array(starsCount * 3);
for (let i = 0; i < starsCount * 3; i++) {
    starPositions[i] = (Math.random() - 0.5) * 4000;
}
starsGeometry.setAttribute('position', new THREE.BufferAttribute(starPositions, 3));
const starsMaterial = new THREE.PointsMaterial({ color: 0xffffff, size: 2, sizeAttenuation: true });
const starField = new THREE.Points(starsGeometry, starsMaterial);
scene.add(starField);

// Grid Floor & Runway
const gridHelper = new THREE.GridHelper(2000, 50, 0x1e293b, 0x0f172a);
gridHelper.position.y = 0;
scene.add(gridHelper);

// Renders the Runway: long grey plane
const runwayWidth = 50;
const runwayLength = 1500;
const runwayGeo = new THREE.PlaneGeometry(runwayWidth, runwayLength);
const runwayMat = new THREE.MeshStandardMaterial({ color: 0x1c1e24, roughness: 0.8 });
const runway = new THREE.Mesh(runwayGeo, runwayMat);
runway.rotation.x = -Math.PI / 2;
runway.position.set(0, 0.01, 0); // slightly above grid to avoid z-fighting
scene.add(runway);

// Runway lines (dashed center line)
const linesGeo = new THREE.PlaneGeometry(2, runwayLength);
const linesMat = new THREE.MeshBasicMaterial({ color: 0xfacc15 });
const centerLine = new THREE.Mesh(linesGeo, linesMat);
centerLine.rotation.x = -Math.PI / 2;
centerLine.position.set(0, 0.02, 0);
scene.add(centerLine);

// 3D Host UAV Model (Delta wing X8 design)
const uavGroup = new THREE.Group();

const wingShape = new THREE.Shape();
wingShape.moveTo(0, 0);
wingShape.lineTo(-12, -8);
wingShape.lineTo(-10, -10);
wingShape.lineTo(0, -7);
wingShape.lineTo(10, -10);
wingShape.lineTo(12, -8);
wingShape.lineTo(0, 0);

const extrudeSettings = { depth: 0.6, bevelEnabled: true, bevelSegments: 2, steps: 1, bevelSize: 0.2, bevelThickness: 0.2 };
const wingGeo = new THREE.ExtrudeGeometry(wingShape, extrudeSettings);
const wingMat = new THREE.MeshStandardMaterial({ color: 0x00f2fe, roughness: 0.2, metalness: 0.8, emissive: 0x003344 });
const wing = new THREE.Mesh(wingGeo, wingMat);
wing.rotation.x = Math.PI / 2;
wing.position.set(0, 0, 4); // offset CG

// UAV Nose Cone
const noseGeo = new THREE.ConeGeometry(1.5, 5, 8);
const noseMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.4 });
const nose = new THREE.Mesh(noseGeo, noseMat);
nose.rotation.x = -Math.PI / 2;
nose.position.set(0, 0, 4.5);

// Twin winglets
const finGeo = new THREE.ConeGeometry(0.8, 4, 3);
const finMat = new THREE.MeshStandardMaterial({ color: 0xff5e62, emissive: 0x330000 });
const leftFin = new THREE.Mesh(finGeo, finMat);
leftFin.rotation.x = Math.PI / 2;
leftFin.position.set(-11, 2, -4);

const rightFin = leftFin.clone();
rightFin.position.set(11, 2, -4);

// Landing Gear Meshes
const gearGroup = new THREE.Group();
gearGroup.name = "landingGear";

const strutGeo = new THREE.CylinderGeometry(0.15, 0.15, 3, 6);
const strutMat = new THREE.MeshStandardMaterial({ color: 0x64748b, metalness: 0.8 });

const wheelGeo = new THREE.CylinderGeometry(0.6, 0.6, 0.4, 8);
const wheelMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.9 });

// Nose Gear
const noseStrut = new THREE.Mesh(strutGeo, strutMat);
noseStrut.position.set(0, -1.5, 3.5);
const noseWheel = new THREE.Mesh(wheelGeo, wheelMat);
noseWheel.rotation.z = Math.PI / 2;
noseWheel.position.set(0, -3.0, 3.5);
gearGroup.add(noseStrut);
gearGroup.add(noseWheel);

// Left Main Gear
const leftStrut = new THREE.Mesh(strutGeo, strutMat);
leftStrut.position.set(-3, -1.5, -2);
const leftWheel = new THREE.Mesh(wheelGeo, wheelMat);
leftWheel.rotation.z = Math.PI / 2;
leftWheel.position.set(-3, -3.0, -2);
gearGroup.add(leftStrut);
gearGroup.add(leftWheel);

// Right Main Gear
const rightStrut = new THREE.Mesh(strutGeo, strutMat);
rightStrut.position.set(3, -1.5, -2);
const rightWheel = new THREE.Mesh(wheelGeo, wheelMat);
rightWheel.rotation.z = Math.PI / 2;
rightWheel.position.set(3, -3.0, -2);
gearGroup.add(rightStrut);
gearGroup.add(rightWheel);

// Wrap all host geometries in a subgroup rotated 180 deg so they point along -Z (Three.js default forward)
const hostModelGroup = new THREE.Group();
hostModelGroup.add(wing);
hostModelGroup.add(nose);
hostModelGroup.add(leftFin);
hostModelGroup.add(rightFin);
hostModelGroup.add(gearGroup);
hostModelGroup.rotation.y = Math.PI;
hostModelGroup.scale.setScalar(2.8 / 24.0); // Scale down to real wingspan (2.8 meters)
uavGroup.add(hostModelGroup);

scene.add(uavGroup);

// Mount FPV camera to the host UAV nose (facing forward along -Z)
uavGroup.add(fpvCamera);
fpvCamera.position.set(0, 0.05, -0.55); // Repositioned to nose of the scaled fuselage
fpvCamera.rotation.set(0, 0, 0);

// Target UAV Model
const targetMesh = new THREE.Group();
const targetModelGroup = new THREE.Group();

const targetWing = new THREE.Mesh(wingGeo, new THREE.MeshStandardMaterial({ color: 0xff5e62, roughness: 0.2, metalness: 0.8, emissive: 0x441111 }));
targetWing.rotation.x = Math.PI / 2;
targetWing.position.set(0, 0, 4);
targetModelGroup.add(targetWing);

const targetNose = new THREE.Mesh(noseGeo, new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.4 }));
targetNose.rotation.x = -Math.PI / 2;
targetNose.position.set(0, 0, 4.5);
targetModelGroup.add(targetNose);

const targetLeftFin = new THREE.Mesh(finGeo, new THREE.MeshStandardMaterial({ color: 0x00f2fe, emissive: 0x003344 }));
targetLeftFin.rotation.x = Math.PI / 2;
targetLeftFin.position.set(-11, 2, -4);
targetModelGroup.add(targetLeftFin);

const targetRightFin = targetLeftFin.clone();
targetRightFin.position.set(11, 2, -4);
targetModelGroup.add(targetRightFin);

targetModelGroup.rotation.y = Math.PI; // Face -Z
targetModelGroup.scale.setScalar(2.8 / 24.0); // Scale down to real wingspan (2.8 meters)
targetMesh.add(targetModelGroup);

scene.add(targetMesh);

// Flight Path Trails
const trailPoints = [];
const trailMaxPoints = 300;
const trailGeo = new THREE.BufferGeometry();
const trailMat = new THREE.LineBasicMaterial({ color: 0x00f2fe, linewidth: 2.5 });
const trailLine = new THREE.Line(trailGeo, trailMat);
scene.add(trailLine);

const targetTrailPoints = [];
const targetTrailMaxPoints = 300;
const targetTrailGeo = new THREE.BufferGeometry();
const targetTrailMat = new THREE.LineBasicMaterial({ color: 0xff5e62, linewidth: 2.5 });
const targetTrailLine = new THREE.Line(targetTrailGeo, targetTrailMat);
scene.add(targetTrailLine);

// ---------------------------------------------------------
// Scene Update Loop
// ---------------------------------------------------------
function updateThreeScene(data) {
    // Coordinate conversions:
    // North (n) -> -z
    // East (e) -> x
    // Down (d) -> -y
    const hostX = data.position_e;
    const hostY = -data.position_d;
    const hostZ = -data.position_n;

    uavGroup.position.set(hostX, hostY, hostZ);

    // Apply aerospace Euler sequence: Yaw -> Pitch -> Roll
    uavGroup.rotation.set(0, 0, 0); // reset
    uavGroup.rotateY(-data.yaw);
    uavGroup.rotateX(data.pitch);
    uavGroup.rotateZ(-data.roll);

    // Scale landing gear based on gear_deploy status
    const gearDeploy = data.gear_deploy !== undefined ? data.gear_deploy : 1.0;
    const gear = uavGroup.getObjectByName("landingGear");
    if (gear) {
        gear.scale.set(1, gearDeploy, 1);
        gear.visible = (gearDeploy > 0.01);
    }

    // Render Target Object and Update target trail
    if (data.targets && data.targets.length > 0) {
        const tgt = data.targets[0];
        const tgtX = tgt.pos[1];
        const tgtY = -tgt.pos[2];
        const tgtZ = -tgt.pos[0];
        targetMesh.position.set(tgtX, tgtY, tgtZ);
        
        targetMesh.rotation.set(0, 0, 0); // reset
        if (tgt.yaw !== undefined) targetMesh.rotateY(-tgt.yaw);
        if (tgt.pitch !== undefined) targetMesh.rotateX(tgt.pitch);
        if (tgt.roll !== undefined) targetMesh.rotateZ(-tgt.roll);

        targetMesh.visible = true;

        targetTrailPoints.push(new THREE.Vector3(tgtX, tgtY, tgtZ));
        if (targetTrailPoints.length > targetTrailMaxPoints) {
            targetTrailPoints.shift();
        }
        targetTrailGeo.setFromPoints(targetTrailPoints);
    } else {
        targetMesh.visible = false;
    }

    // Update flight path trail for follower
    trailPoints.push(new THREE.Vector3(hostX, hostY, hostZ));
    if (trailPoints.length > trailMaxPoints) {
        trailPoints.shift();
    }
    trailGeo.setFromPoints(trailPoints);

    // -------------------------------------------------------
    // Dynamic Formation Camera: frames BOTH UAVs in view
    // -------------------------------------------------------
    const yaw = -data.yaw;

    let focusX = hostX, focusY = hostY, focusZ = hostZ;
    let camDist = 55.0;
    const camHeight = 20.0;

    if (data.targets && data.targets.length > 0) {
        const tgt = data.targets[0];
        // Target position in Three.js world space (NED → Three.js: E=X, -D=Y, -N=Z)
        const tgtX = tgt.pos[1];
        const tgtY = -tgt.pos[2];
        const tgtZ = -tgt.pos[0];

        // Midpoint between both UAVs
        focusX = (hostX + tgtX) * 0.5;
        focusY = (hostY + tgtY) * 0.5;
        focusZ = (hostZ + tgtZ) * 0.5;

        // Separation distance → auto-zoom so both fit in frame
        const sep = Math.sqrt(
            Math.pow(tgtX - hostX, 2) +
            Math.pow(tgtY - hostY, 2) +
            Math.pow(tgtZ - hostZ, 2)
        );
        camDist = Math.max(50, sep * 0.85 + 30);
    }

    // Position camera behind and above midpoint, slightly offset to the side (quarter view)
    // so the follower doesn't block the leader, making it very obvious one is chasing the other.
    const sideOffset = 0.28; // ~16 degrees side offset
    const cameraTarget = new THREE.Vector3(
        focusX - Math.sin(yaw + sideOffset) * camDist,
        focusY + camHeight,
        focusZ + Math.cos(yaw + sideOffset) * camDist
    );

    camera.position.x = THREE.MathUtils.lerp(camera.position.x, cameraTarget.x, 0.08);
    camera.position.y = THREE.MathUtils.lerp(camera.position.y, cameraTarget.y, 0.08);
    camera.position.z = THREE.MathUtils.lerp(camera.position.z, cameraTarget.z, 0.08);

    camera.lookAt(new THREE.Vector3(focusX, focusY + 2, focusZ));
}

// ---------------------------------------------------------
// 2D Nose Camera & Pseudo-YOLO Canvas Rendering
// ---------------------------------------------------------
const cameraContainer = document.getElementById('camera-feed');
// Create 2D canvas overlays dynamically inside camera container
const camCanvas = document.createElement('canvas');
camCanvas.width = 640;
camCanvas.height = 480;
camCanvas.style.width = '100%';
camCanvas.style.height = '100%';
camCanvas.style.position = 'absolute';
camCanvas.style.top = '0';
camCanvas.style.left = '0';
camCanvas.style.zIndex = '2';
cameraContainer.insertBefore(camCanvas, cameraContainer.firstChild);

const ctx = camCanvas.getContext('2d');

function updateCameraFeed(data) {
    ctx.clearRect(0, 0, 640, 480);
    
    // Draw artificial horizon line
    ctx.strokeStyle = 'rgba(0, 242, 254, 0.2)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, 240);
    ctx.lineTo(640, 240);
    ctx.stroke();

    const bboxDiv = document.getElementById('yolo-bbox');

    if (data.targets && data.targets.length > 0) {
        const tgt = data.targets[0];
        const bbox = tgt.bbox;

        if (bbox && bbox.visible) {
            // Draw a wireframe target representation in the camera viewport
            // We draw the bounding box in the 2D canvas
            ctx.strokeStyle = '#ff5e62';
            ctx.lineWidth = 2;
            
            // Draw corners
            const w = bbox.xmax - bbox.xmin;
            const h = bbox.ymax - bbox.ymin;
            ctx.strokeRect(bbox.xmin, bbox.ymin, w, h);
            
            // Draw central targeting circle
            ctx.beginPath();
            ctx.arc(bbox.xmin + w/2, bbox.ymin + h/2, 4, 0, 2 * Math.PI);
            ctx.fillStyle = '#ff5e62';
            ctx.fill();

            // Display CSS HTML overlay matching position
            bboxDiv.style.display = 'block';
            bboxDiv.style.left = (bbox.xmin / 6.4) + '%';
            bboxDiv.style.top = (bbox.ymin / 4.8) + '%';
            bboxDiv.style.width = (w / 6.4) + '%';
            bboxDiv.style.height = (h / 4.8) + '%';
            
            // Label
            document.getElementById('yolo-label').textContent = `${tgt.name}: 0.98`;
        } else {
            bboxDiv.style.display = 'none';
        }
    } else {
        bboxDiv.style.display = 'none';
    }
}

// ---------------------------------------------------------
// Initialization & Resize Listeners
// ---------------------------------------------------------
function resizeScene() {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);

    // Resize FPV
    if (fpvCamera && fpvRenderer && fpvCanvas) {
        fpvCamera.aspect = 4 / 3;
        fpvCamera.updateProjectionMatrix();
        fpvRenderer.setSize(fpvCanvas.clientWidth, fpvCanvas.clientHeight);
    }
}

window.addEventListener('resize', resizeScene);

// Start Render Loop
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
    if (fpvRenderer && scene && fpvCamera) {
        fpvRenderer.render(scene, fpvCamera);
    }
}

animate();
connectWS();
resizeScene();

// ---------------------------------------------------------
// Fullscreen Logic
// ---------------------------------------------------------
const btnFullscreen = document.getElementById('btn-fullscreen');
if (btnFullscreen) {
    btnFullscreen.addEventListener('click', () => {
        const container = document.getElementById('threejs-container');
        if (!document.fullscreenElement) {
            if (container.requestFullscreen) {
                container.requestFullscreen();
            } else if (container.webkitRequestFullscreen) { /* Safari */
                container.webkitRequestFullscreen();
            } else if (container.msRequestFullscreen) { /* IE11 */
                container.msRequestFullscreen();
            }
        } else {
            if (document.exitFullscreen) {
                document.exitFullscreen();
            } else if (document.webkitExitFullscreen) { /* Safari */
                document.webkitExitFullscreen();
            } else if (document.msExitFullscreen) { /* IE11 */
                document.msExitFullscreen();
            }
        }
    });
}
