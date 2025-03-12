import * as THREE from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/build/three.module.js';
import { OrbitControls } from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/examples/jsm/controls/OrbitControls.js';
import { AlvaARConnectorTHREE } from './alva_ar_three.js';
import { IMU } from './imu.js'
import { deg2rad, rad2deg } from './utils.js';

class ARCamView {
    constructor(container, width, height, x = 0, y = 0, z = -10, scale = 1.0){
        this.init(container, width, height, x, y, z, scale);
    }
    async init(container, width, height, x, y, z, scale) {    
        this.applyPose = AlvaARConnectorTHREE.Initialize(THREE);

        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setClearColor(0, 0);
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(window.devicePixelRatio);

        this.camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
        this.camera.rotation.reorder('YXZ');
        this.camera.updateProjectionMatrix();

        this.scene = new THREE.Scene();
        this.scene.add(new THREE.AmbientLight(0x808080));
        this.scene.add(new THREE.HemisphereLight(0x404040, 0xf0f0f0, 1));
        this.scene.add(this.camera);

        //case1: beta=o gamma=o alpha=0
        //case2: beta=O gamma=o alpha=90
        //case3: beta=O gamma=O alpha=90
        //case4: beta=o gamma=O alpha=0
        //case5: beta->o gamma->o alpha=90
        var imu = await IMU.Initialize();
        console.log('IMU initialized:', imu);
        // Introduce a delay to allow the IMU properties to be updated
        setTimeout(() => {
            console.log('X:', imu.X);
            console.log('Y:', imu.Y);
            console.log('Z:', imu.Z);
        }, 2000); // Adjust the delay as needed
        
        // Array to hold multiple objects
        this.objects = [];	
        this.camera.rotation.set(-Math.cos(imu.X) * Math.sin(-imu.Y), -Math.cos(imu.X) * Math.cos(imu.Y), Math.sin(-imu.X));
        for (let i = -2; i < 4; i++) {
            const object = this.createFlatArrow();
            object.position.set(0, 3 * i, -10);
            //imu = await IMU.Initialize();
            //setTimeout(() => {
                //var alpha = imu.Z * rad2deg, beta = imu.X * rad2deg, gamma = imu.Y * rad2deg;
                //console.log('X:', beta, 'Y:', gamma, 'Z:', alpha);
                //if (Math.abs(beta) < 10 && Math.abs(gamma) < 10 && (alpha < 10 || alpha > 350))
                    //object.position.set(0, y + 10 + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y)), -10 + 3 * i);
                //else
                    //without revision it would be (0, 10+3*i, -10-3*i)
                    /*
                    R=[
                    cos(alpha)cos(gamma)-sin(alpha)sin(beta)sin(gamma) -sin(alpha)cos(beta) cos(alpha)sin(gamma)+sin(alpha)sin(beta)cos(gamma)
                    sin(alpha)cos(gamma)+cos(alpha)sin(beta)sin(gamma) cos(alpha)cos(beta)  sin(alpha)sin(gamma)-cos(alpha)sin(beta)cos(gamma)
                    -cos(beta)sin(gamma)                               sin(beta)            cos(beta)cos(gamma)
                    ]
                    [            [
                    x'           x
                    y'   =   R*  y
                    z'           z
                    ]            ]
                    
                    object.position.set((Math.cos(imu.Z - Math.PI / 2) * Math.cos(imu.Y) - Math.sin(-imu.Z + Math.PI / 2) * Math.sin(-imu.X) * Math.sin(-imu.Y)) * 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2))
                        - Math.sin(-imu.Z + Math.PI / 2) * Math.cos(imu.X) * (-10 - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)))
                        + (Math.cos(imu.Z - Math.PI / 2) * Math.sin(-imu.Y) + Math.sin(-imu.Z + Math.PI / 2) * Math.sin(-imu.X) * Math.cos(imu.Y)) * (10 + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y))),
                        - Math.cos(imu.X) * Math.sin(-imu.Y) * 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2))
                        + Math.sin(-imu.X) * (-10 - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)))
                        + Math.cos(imu.X) * Math.cos(imu.Y) * (10 + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y))),
                        (Math.sin(-imu.Z + Math.PI / 2) * Math.cos(imu.Y) + Math.cos(imu.Z - Math.PI / 2) * Math.sin(-imu.X) * Math.sin(-imu.Y)) * 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2))
                        + Math.cos(imu.Z - Math.PI / 2) * Math.cos(imu.X) * (-10 - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)))
                        + (Math.sin(-imu.Z + Math.PI / 2) * Math.sin(-imu.Y) - Math.cos(imu.Z - Math.PI / 2) * Math.sin(-imu.X) * Math.cos(imu.Y)) * (10 + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y))));
                    */
                    /*R'=[
                    cos(alpha)cos(gamma)+sin(alpha)sin(beta)sin(gamma)  -sin(alpha)cos(gamma)+cos(alpha)sin(beta)sin(gamma) cos(beta)sin(gamma)
                    sin(alpha)cos(beta)                                 cos(alpha)cos(beta)                                 -sin(beta)
                    -cos(alpha)sin(gamma)+sin(alpha)sin(beta)cos(gamma) sin(alpha)sin(gamma)+cos(alpha)sin(beta)cos(gamma)  cos(beta)cos(gamma)] 
                    */
                    /*object.position.set((Math.cos(imu.Z - Math.PI / 2) * Math.cos(imu.Y) + Math.sin(-imu.Z + Math.PI / 2) * Math.sin(-imu.X) * Math.sin(imu.Y)) * (x + 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2)))
                        + (-Math.sin(-imu.Z + Math.PI / 2) * Math.cos(imu.Y) + Math.cos(imu.Z - Math.PI / 2) * Math.sin(-imu.X) * Math.sin(-imu.Y)) * (z - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)))
                        + Math.cos(imu.X) * Math.sin(-imu.Y) * (y - 10 + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y))),
                        (-Math.cos(imu.Z - Math.PI / 2) * Math.sin(-imu.Y) + Math.sin(-imu.Z + Math.PI / 2) * Math.sin(-imu.X) * Math.cos(imu.Y)) * (x + 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2)))
                        + (Math.sin(-imu.Z + Math.PI / 2) * Math.sin(-imu.Y) + Math.cos(imu.Z - Math.PI / 2) * Math.sin(-imu.X) * Math.cos(imu.Y)) * (z - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)))
                        + Math.cos(imu.X) * Math.cos(imu.Y) * (y - 10 + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y))),
                        Math.sin(-imu.Z + Math.PI / 2) * Math.cos(imu.X) * (x + 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2)))
                        + Math.cos(imu.Z - Math.PI / 2) * Math.cos(imu.X) * (z - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)))
                        - Math.sin(-imu.X) * (y - 10 + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y))));
                    */
            
                    //var xy=Math.sqrt(),yz,zx
            //object.position.set(x + 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2)), y + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y)), z - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)));
            
            //}, 2000);

            object.visible = false;
            this.objects.push(object); // Add the object to the objects array
            this.scene.add(object);
        }
/* 
        for (let i = 0; i < 5; i++) { 
            const object = this.createFlatArrow();
            const radius = 4; // 半径
            const angleStep = Math.PI / 8; // 间隔
            const angle = i * angleStep;
            const m = radius * Math.cos(angle);
            const n = radius * Math.sin(angle);
            object.rotation.z = angle - Math.PI / 2;
            object.position.set(m - 4 + 3 * i * (Math.cos(imu.Y) + Math.cos(imu.Z + Math.PI / 2)), n + 3 * i + i * 3 * (Math.sin(imu.X) + Math.sin(imu.Y)), z - 3 * i - i * 3 * (Math.cos(imu.X) + Math.cos(imu.Z + Math.PI / 2)));
            object.visible = false;
            this.objects.push(object); // Add the object to the objects array
            this.scene.add(object);
        }*/
        container.appendChild(this.renderer.domElement);

        const render = () => {
            requestAnimationFrame(render.bind(this));

            this.renderer.render(this.scene, this.camera);
        }

        render();
    }

	createFlatArrow() {
        const arrowGroup = new THREE.Group();

        // Create the circle background
        const circleGeometry = new THREE.CircleGeometry(0.5, 32);
        const circleMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00, side: THREE.DoubleSide });
        const circleMesh = new THREE.Mesh(circleGeometry, circleMaterial);
        arrowGroup.add(circleMesh);

        // Create the arrow shape
        const arrowShape = new THREE.Shape();
        arrowShape.moveTo(0.25, 0.15);  // Start at the bottom side
        arrowShape.lineTo(0.25, -0.15); // Bottom line
        arrowShape.lineTo(-0.25, 0);     // Point of the arrow
        arrowShape.lineTo(0.25, 0.15);  // Back to start, close the shape

        // Create geometry from the shape
        const arrowGeometry = new THREE.ShapeGeometry(arrowShape);
        const arrowMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide });
        const arrowMesh = new THREE.Mesh(arrowGeometry, arrowMaterial);
        arrowMesh.position.set(0, 0, 0.05);  // Slightly in front of the circle to avoid z-fighting
        arrowGroup.add(arrowMesh);

        arrowGroup.rotation.z = Math.PI / 2 * 3;  // 顺时针再旋转90度，使箭头指向前方

        return arrowGroup;
    }
    updateCameraPose(pose) {
        this.applyPose(pose, this.camera.quaternion, this.camera.position);
        this.objects.forEach(object => object.visible = true);
    }
    lostCamera() {
        this.objects.forEach(object => object.visible = false);
    }
}

class ARCamIMUView
{
    constructor( container, width, height )
    {
        this.applyPose = AlvaARConnectorTHREE.Initialize( THREE );

        this.renderer = new THREE.WebGLRenderer( { antialias: true, alpha: true } );
        this.renderer.setClearColor( 0, 0 );
        this.renderer.setSize( width, height );
        this.renderer.setPixelRatio( window.devicePixelRatio );

        this.camera = new THREE.PerspectiveCamera( 60, width / height, 0.01, 1000 );

        this.raycaster = new THREE.Raycaster();

        this.ground = new THREE.Mesh(
            new THREE.CircleGeometry( 1000, 64 ),
            new THREE.MeshBasicMaterial( {
                color: 0xffffff,
                transparent: true,
                depthTest: true,
                opacity: 0.1,
                side: THREE.DoubleSide
            } )
        );

        this.ground.rotation.x = Math.PI / 2; // 90 deg
        this.ground.position.y = -10;

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0x808080 ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.ground );
        this.scene.add( this.camera );

        container.appendChild( this.renderer.domElement );

        const render = () =>
        {
            requestAnimationFrame( render.bind( this ) );

            this.renderer.render( this.scene, this.camera );
        }

        render();
    }

    updateCameraPose( pose )
    {
        this.applyPose( pose, this.camera.quaternion, this.camera.position );

        this.ground.position.x = this.camera.position.x;
        this.ground.position.z = this.camera.position.z;

        this.scene.children.forEach( obj => obj.visible = true );
    }

    lostCamera()
    {
        this.scene.children.forEach( obj => obj.visible = false );
    }

    addObjectAt(x, y, scale = 1.0) {
    const el = this.renderer.domElement;
    const coord = new THREE.Vector2((x / el.offsetWidth) * 2 - 1, -(y / el.offsetHeight) * 2 + 1);
    this.raycaster.setFromCamera(coord, this.camera);

    const intersections = this.raycaster.intersectObjects([this.ground]);

    if (intersections.length > 0) {
        const point = intersections[0].point;

        const sphere = new THREE.Mesh(
            new THREE.SphereGeometry(scale, 32, 32), // 使用SphereGeometry创建球体
            new THREE.MeshNormalMaterial({ flatShading: true })
        );
        sphere.position.set(point.x, point.y, point.z);
        sphere.custom = true;

        this.scene.add(sphere);
    }
}


    reset()
    {
        this.scene.children.filter( o => o.custom ).forEach( o => this.scene.remove( o ) );
    }
}

class ARSimpleView
{
    constructor( container, width, height, mapView = null )
    {
        this.applyPose = AlvaARConnectorTHREE.Initialize( THREE );

        this.renderer = new THREE.WebGLRenderer( { antialias: true, alpha: true } );
        this.renderer.setClearColor( 0, 0 );
        this.renderer.setSize( width, height );
        this.renderer.setPixelRatio( window.devicePixelRatio );

        this.camera = new THREE.PerspectiveCamera( 75, width / height, 0.1, 1000 );
        this.camera.rotation.reorder( 'YXZ' );
        this.camera.updateProjectionMatrix();

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0x808080 ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.camera );

        this.body = document.body;

        container.appendChild( this.renderer.domElement );

        if( mapView )
        {
            this.mapView = mapView;
            this.mapView.camHelper = new THREE.CameraHelper( this.camera );
            this.mapView.scene.add( this.mapView.camHelper );
        }
    }

    updateCameraPose( pose )
    {
        this.applyPose( pose, this.camera.quaternion, this.camera.position );

        this.renderer.render( this.scene, this.camera );

        this.body.classList.add( "tracking" );
    }

    lostCamera()
    {
        this.body.classList.remove( "tracking" );
    }

    createObjectWithPose(pose, scale = 1.0) {
    const plane = new THREE.Mesh(
        new THREE.PlaneGeometry(scale, scale),
        new THREE.MeshBasicMaterial({
            color: 0xffffff,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.1
        })
    );

    const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(scale * 0.5, 32, 32), // 使用SphereGeometry创建球体
        new THREE.MeshNormalMaterial({ flatShading: true })
    );
    sphere.position.z = scale * 0.5;

    plane.add(sphere);
    plane.custom = true;

    this.applyPose(pose, plane.quaternion, plane.position);
    this.scene.add(plane);

    if (this.mapView) {
        this.mapView.scene.add(plane.clone());
    }
}


    reset()
    {
        this.scene.children.filter( o => o.custom ).forEach( o => this.scene.remove( o ) );

        if( this.mapView )
        {
            this.mapView.scene.children.filter( o => o.custom ).forEach( o => this.mapView.scene.remove( o ) );
        }
    }
}

class ARSimpleMap
{
    constructor( container, width, height )
    {
        this.renderer = new THREE.WebGLRenderer( { antialias: false } );
        this.renderer.setClearColor( new THREE.Color( 'rgb(255, 255, 255)' ) );
        this.renderer.setPixelRatio( window.devicePixelRatio );
        this.renderer.setSize( width, height, false );
        this.renderer.domElement.style.width = width + 'px';
        this.renderer.domElement.style.height = height + 'px';

        this.camera = new THREE.PerspectiveCamera( 50, width / height, 0.01, 1000 );
        this.camera.position.set( -1, 2, 2 );

        this.controls = new OrbitControls( this.camera, this.renderer.domElement, );
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.1;
        this.controls.minDistance = 0.1;
        this.controls.maxDistance = 1000;

        this.gridHelper = new THREE.GridHelper( 150, 100 );
        this.gridHelper.position.y = -1;

        this.axisHelper = new THREE.AxesHelper( 0.25 );

        this.camHelper = null;

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0xefefef ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.gridHelper );
        this.scene.add( this.axisHelper );

        container.appendChild( this.renderer.domElement );

        const render = () =>
        {
            this.controls.update();
            this.renderer.render( this.scene, this.camera );

            requestAnimationFrame( render );
        }

        render();
    }
}

export { ARCamView, ARCamIMUView, ARSimpleView, ARSimpleMap }