import { deg2rad, getScreenOrientation, isIOS } from "./utils.js";

class Quaternion
{
    static fromAxisAngle( axisX = 0, axisY = 0, axisZ = 0, angle = 0 )
    {
        const angle2 = angle / 2;
        const s = Math.sin( angle2 );

        return {
            x: axisX * s,
            y: axisY * s,
            z: axisZ * s,
            w: Math.cos( angle2 )
        };
    }

    static fromEuler( x = 0, y = 0, z = 0, order = 'XYZ' )
    {
        const cos = Math.cos;
        const sin = Math.sin;

        const c1 = cos( x / 2 );
        const c2 = cos( y / 2 );
        const c3 = cos( z / 2 );

        const s1 = sin( x / 2 );
        const s2 = sin( y / 2 );
        const s3 = sin( z / 2 );

        const q = { x: 0, y: 0, z: 0, w: 1 };

        switch( order )
        {
            case 'XYZ':
                q.x = s1 * c2 * c3 + c1 * s2 * s3;
                q.y = c1 * s2 * c3 - s1 * c2 * s3;
                q.z = c1 * c2 * s3 + s1 * s2 * c3;
                q.w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'YXZ':
                q.x = s1 * c2 * c3 + c1 * s2 * s3;
                q.y = c1 * s2 * c3 - s1 * c2 * s3;
                q.z = c1 * c2 * s3 - s1 * s2 * c3;
                q.w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            case 'ZXY':
                q.x = s1 * c2 * c3 - c1 * s2 * s3;
                q.y = c1 * s2 * c3 + s1 * c2 * s3;
                q.z = c1 * c2 * s3 + s1 * s2 * c3;
                q.w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'ZYX':
                q.x = s1 * c2 * c3 - c1 * s2 * s3;
                q.y = c1 * s2 * c3 + s1 * c2 * s3;
                q.z = c1 * c2 * s3 - s1 * s2 * c3;
                q.w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            case 'YZX':
                q.x = s1 * c2 * c3 + c1 * s2 * s3;
                q.y = c1 * s2 * c3 + s1 * c2 * s3;
                q.z = c1 * c2 * s3 - s1 * s2 * c3;
                q.w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'XZY':
                q.x = s1 * c2 * c3 - c1 * s2 * s3;
                q.y = c1 * s2 * c3 - s1 * c2 * s3;
                q.z = c1 * c2 * s3 + s1 * s2 * c3;
                q.w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            default:
                console.warn( 'CreateFromEuler() encountered an unknown order: ' + order );
        }

        return q;
    }

    static multiply( a, b )
    {
        const qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
        const qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;

        return {
            x: qax * qbw + qaw * qbx + qay * qbz - qaz * qby,
            y: qay * qbw + qaw * qby + qaz * qbx - qax * qbz,
            z: qaz * qbw + qaw * qbz + qax * qby - qay * qbx,
            w: qaw * qbw - qax * qbx - qay * qby - qaz * qbz,
        }
    }

    static dot( a, b )
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }
}

class KalmanFilter {
    constructor() {
        this.state = new Array(10).fill(0);
        this.state[3] = 1; // 初始化四元数 w 分量为 1

        this.covariance = Array.from({ length: 10 }, () => new Array(10).fill(0));
        for (let i = 0; i < 10; i++) {
            this.covariance[i][i] = 1;
        }

        this.processNoise = this.getProcessNoiseMatrix(0.01);
        this.observationNoise = this.getObservationNoiseMatrix();

        this.stateTransition = Array.from({ length: 10 }, () => new Array(10).fill(0));
        this.observationMatrix = this.getObservationMatrix();
    }
    
    getStateTransitionMatrix(dt) {
        const F = Array.from({ length: 10 }, () => new Array(10).fill(0));
        for (let i = 0; i < 10; i++) {
            F[i][i] = 1;
        }
        return F;
    }
    
    getProcessNoiseMatrix(dt) {
        const q = 0.01 * dt;
        return Array.from({ length: 10 }, (_, i) => new Array(10).fill(0).map((_, j) => (i === j ? q : 0)));
    }

    getObservationNoiseMatrix() {
        return Array.from({ length: 10 }, (_, i) => new Array(10).fill(0).map((_, j) => (i === j ? 0.1 : 0)));
    }

    getObservationMatrix() {
        return Array.from({ length: 10 }, (_, i) => new Array(10).fill(0).map((_, j) => (i === j ? 1 : 0)));
    }

    predict(dt) {
        const F = this.getStateTransitionMatrix(dt);
        this.state = this.multiplyMatrices(F, this.state.map(val => [val])).map(row => row[0]);

        const FP = this.multiplyMatrices(F, this.covariance);
        const FPF_T = this.multiplyMatrices(FP, this.transposeMatrix(F));
        this.covariance = this.addMatrices(FPF_T, this.processNoise);
    }

    update(measurement) {
        const H = this.observationMatrix;
        const HP = this.multiplyMatrices(H, this.covariance);
        const HPH_T = this.multiplyMatrices(HP, this.transposeMatrix(H));
        const S = this.addMatrices(HPH_T, this.observationNoise);
        const K = this.multiplyMatrices(
            this.multiplyMatrices(this.covariance, this.transposeMatrix(H)),
            this.invertMatrix(S)
        );
        console.log("S:", S);
        console.log("K:", K);
        const innovation = measurement.map((val, i) => val - this.state[i]);
        const K_innovation = this.multiplyMatrices(K, innovation.map(val => [val]));
        this.state = this.state.map((val, i) => val + K_innovation[i][0]);

        const KH = this.multiplyMatrices(K, H);
        const I_KH = this.covariance.map((row, i) => row.map((val, j) => i === j ? 1 - KH[i][j] : -KH[i][j]));
        this.covariance = this.multiplyMatrices(I_KH, this.covariance);
    }

    multiplyMatrices(a, b) {
        const result = Array.from({ length: a.length }, () => new Array(b[0].length).fill(0));
        for (let i = 0; i < a.length; i++) {
            for (let j = 0; j < b[0].length; j++) {
                for (let k = 0; k < a[0].length; k++) {
                    result[i][j] += a[i][k] * b[k][j];
                }
            }
        }
        return result;
    }

    addMatrices(a, b) {
        return a.map((row, i) => row.map((val, j) => val + b[i][j]));
    }

    transposeMatrix(matrix) {
        return matrix[0].map((_, i) => matrix.map(row => row[i]));
    }

    invertMatrix(matrix) {
        return matrix.map((row, i) => row.map((val, j) => (i === j && val !== 0 ? 1 / val : 0)));
    }
}

class IMU
{
    static Initialize()
    {
        return new Promise( ( resolve, reject ) =>
        {
            const finalize = () =>
            {
                if( window.isSecureContext === false )
                {
                    reject( "DeviceOrientation is only available in secure contexts (https)." );
                    return;
                }

                if( window.DeviceOrientationEvent === undefined )
                {
                    reject( "DeviceOrientation not supported." );
                    return;
                }

                if( window.DeviceMotionEvent === undefined )
                {
                    reject( "DeviceMotion not supported." );
                    return;
                }

                resolve( new IMU() );
            }
 
            if( window.DeviceMotionEvent !== undefined && typeof window.DeviceMotionEvent.requestPermission === 'function' )
            {
                window.DeviceMotionEvent.requestPermission().then( state =>
                {
                    if( state === "granted" )
                    {
                        finalize();
                    }
                    else
                    {
                        reject( "Permission denied by user." );
                    }
                }, error =>
                {
                    reject( error.toString() );
                } );
            }
            else if( window.ondevicemotion !== undefined )
            {
                finalize();
            }
            else
            {
                reject( "DeviceMotion is not supported." );
            }
        } );
    }

    constructor()
    {
        this.EPS = 0.000001;
        this.screenOrientation = null;
        this.screenOrientationAngle = 0;
        this.timestamps = [Date.now(), Date.now()];
        this.mill_timestamp = Date.now();
        this.position = [0, 0, 0]; // x, y, z
        this.velocity = [0, 0, 0]; // vx, vy, vz
        this.q = { x: 0, y: 0, z: 0, w: 1 };
        this.X = this.Y = this.Z = 0;
        this.acceleration = [0, 0, 0];
        this.angularVelocity = [0, 0, 0];
        this.prevState = {
            quaternion: [0, 0, 0, 1],
            angularVelocity: [0, 0, 0],
            acceleration: [0, 0, 0],
            position: [0, 0, 0],
            velocity: [0, 0, 0]
        };
        this.orientation = { x: 1, y: 0, z: 0, w: 1 };
        this.worldTransform = isIOS()
            ? Quaternion.fromAxisAngle( 1, 0, 0, -Math.PI / 2 ) // -90 degrees on x-axis
            : Quaternion.fromAxisAngle(0, 1, 0, Math.PI / 2); // 90 degrees on y-axis
        // use kf
        this.imuData = {
            quaternion: [0, 0, 0, 1], // 四元数 [q_x, q_y, q_z, q_w]
            angularVelocity: [0, 0, 0], // 角速度 [omega_x, omega_y, omega_z]
            acceleration: [0, 0, 0] // 加速度 [a_x, a_y, a_z]
        };
        this.KFimuData = {
            quaternion: [0, 0, 0, 1], // 四元数 [q_x, q_y, q_z, q_w]
            angularVelocity: [0, 0, 0], // 角速度 [omega_x, omega_y, omega_z]
            acceleration: [0, 0, 0], // 加速度 [a_x, a_y, a_z]
            position: [0, 0, 0],
            velocity: [0, 0, 0]
        };
        this.kf = new KalmanFilter();
        const handleDeviceOrientation = ( event ) =>
        {
            // axis orientation assumes device is placed on ground, screen upward
            const x = event.beta * deg2rad;    // X-axis (β) vertical tilt
            const y = event.gamma * deg2rad;   // Y-axis (γ) horizontal tilt
            const z = event.alpha * deg2rad;   // Z-axis (α) compass direction

            this.q = Quaternion.fromEuler(x, y, z, 'ZXY');
            const orientation = Quaternion.multiply(this.worldTransform, this.q);

            if( 8 * (1 - Quaternion.dot( this.orientation, orientation )) > this.EPS )
            {
                this.orientation = orientation;
            }
        }

        const handleDeviceMotion = ( event ) =>
        {
            let gx = event.rotationRate.beta * deg2rad;   // X-axis (β) deg to rad: rad/s
            let gy = event.rotationRate.gamma * deg2rad;  // Y-axis (γ) deg to rad: rad/s
            let gz = event.rotationRate.alpha * deg2rad;  // Z-axis (α) deg to rad: rad/s

            let ax = event.acceleration.x; // (m/s^2)
            let ay = event.acceleration.y; // (m/s^2)
            let az = event.acceleration.z; // (m/s^2)

            this.angularVelocity = [gx, gy, gz];
            this.acceleration = [ax, ay, az];
            
            this.imuData.quaternion = [this.q.x, this.q.y, this.q.z, this.q.w];
            this.imuData.angularVelocity = [...this.angularVelocity];
            this.imuData.acceleration = [...this.acceleration];

            this.prevState = JSON.parse(JSON.stringify(this.KFimuData));
            const timestamp = Date.now();
            this.timestamps[0] = this.timestamps[1];
            this.timestamps[1] = timestamp;
            this.mill_timestamp = timestamp;
            let dt = (this.timestamps[1] - this.timestamps[0]) / 1000;
            if (dt > 0) {
                this.kf.predict(dt);
                console.log("Before update:", this.kf.state);
                const measurement = [...this.imuData.quaternion, ...this.imuData.angularVelocity, ...this.imuData.acceleration];
                this.kf.update(measurement);
                console.log("After update:", this.kf.state);

                const updatedQuaternion = this.integrateQuaternion(this.prevState.quaternion, this.prevState.angularVelocity, dt);
                const newVelocity = this.prevState.velocity.map((v, i) => v + this.prevState.acceleration[i] * dt);
                const newPosition = this.prevState.position.map((p, i) => p + this.prevState.velocity[i] * dt + 0.5 * this.prevState.acceleration[i] * dt * dt);

                [this.q.x, this.q.y, this.q.z, this.q.w, gx, gy, gz, ax, ay, az] = this.kf.state;
                this.KFimuData.quaternion = updatedQuaternion;
                this.KFimuData.angularVelocity = [gx, gy, gz];
                this.KFimuData.acceleration = [ax, ay, az];
                this.KFimuData.velocity = newVelocity;
                this.KFimuData.position = newPosition;
            }
        };


        const handleScreenOrientation = ( event ) =>
        {
            this.screenOrientation = getScreenOrientation();

            if( this.screenOrientation === 'landscape_left' )
            {
                this.screenOrientationAngle = 90;
            }
            else if( this.screenOrientation === 'landscape_right' )
            {
                this.screenOrientationAngle = 270;
            }
            else
            {
                this.screenOrientationAngle = 0;
            }
        }

        window.addEventListener( 'devicemotion', handleDeviceMotion.bind( this ), false );
        window.addEventListener( 'deviceorientation', handleDeviceOrientation.bind( this ), false );
        window.addEventListener( 'orientationchange', handleScreenOrientation.bind( this ), false );
    }

    integrateQuaternion(q, omega, dt) {
        const wx = omega[0] * dt / 2;
        const wy = omega[1] * dt / 2;
        const wz = omega[2] * dt / 2;

        return [
            q[0] + wx * q[3] + wy * q[2] - wz * q[1],
            q[1] - wx * q[2] + wy * q[3] + wz * q[0],
            q[2] + wx * q[1] - wy * q[0] + wz * q[3],
            q[3] - wx * q[0] - wy * q[1] - wz * q[2]
        ];
    }
    clear()
    {
        this.motion.length = 0;
    }
}

export { IMU };
