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
        // 初始化状态向量 [q_x, q_y, q_z, q_w, omega_x, omega_y, omega_z, a_x, a_y, a_z]
        this.state = new Array(10).fill(0);
        this.state[3] = 1; // 初始化四元数的 w 分量为 1（表示无旋转）

        // 初始化状态协方差矩阵
        this.covariance = Array.from({ length: 10 }, () => new Array(10).fill(0));
        for (let i = 0; i < 10; i++) {
            this.covariance[i][i] = 1; // 初始协方差矩阵为单位矩阵
        }

        // 过程噪声协方差矩阵
        this.processNoise = Array.from({ length: 10 }, () => new Array(10).fill(0));
        for (let i = 0; i < 10; i++) {
            this.processNoise[i][i] = 0.01; // 假设过程噪声较小
        }

        // 观测噪声协方差矩阵
        this.observationNoise = Array.from({ length: 10 }, () => new Array(10).fill(0));
        for (let i = 0; i < 10; i++) {
            this.observationNoise[i][i] = 0.1; // 假设观测噪声较大
        }

        // 状态转移矩阵（假设为单位矩阵）
        this.stateTransition = Array.from({ length: 10 }, () => new Array(10).fill(0));
        for (let i = 0; i < 10; i++) {
            this.stateTransition[i][i] = 1;
        }

        // 观测矩阵（假设为单位矩阵）
        this.observationMatrix = Array.from({ length: 10 }, () => new Array(10).fill(0));
        for (let i = 0; i < 10; i++) {
            this.observationMatrix[i][i] = 1;
        }
    }

    // 矩阵乘法
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

    // 矩阵加法
    addMatrices(a, b) {
        const result = Array.from({ length: a.length }, () => new Array(a[0].length).fill(0));
        for (let i = 0; i < a.length; i++) {
            for (let j = 0; j < a[0].length; j++) {
                result[i][j] = a[i][j] + b[i][j];
            }
        }
        return result;
    }

    // 矩阵转置
    transposeMatrix(matrix) {
        return matrix[0].map((_, i) => matrix.map(row => row[i]));
    }

    // 矩阵求逆（简单实现，仅适用于对角矩阵）
    invertMatrix(matrix) {
        const result = Array.from({ length: matrix.length }, () => new Array(matrix[0].length).fill(0));
        for (let i = 0; i < matrix.length; i++) {
            for (let j = 0; j < matrix[0].length; j++) {
                result[i][j] = matrix[i][j] !== 0 ? 1 / matrix[i][j] : 0;
            }
        }
        return result;
    }

    // 预测步骤
    predict() {
        // 将状态向量转换为列向量（二维数组）
        const stateAsColumn = this.state.map(val => [val]);

        // 预测状态：F * x
        const predictedState = this.multiplyMatrices(this.stateTransition, stateAsColumn);
        this.state = predictedState.map(row => row[0]); // 转换回一维数组

        // 预测协方差：F * P * F^T + Q
        const FP = this.multiplyMatrices(this.stateTransition, this.covariance);
        const FPF_T = this.multiplyMatrices(FP, this.transposeMatrix(this.stateTransition));
        this.covariance = this.addMatrices(FPF_T, this.processNoise);
    }

    // 更新步骤
    update(measurement) {
        // 将状态向量转换为列向量（二维数组）
        const stateAsColumn = this.state.map(val => [val]);

        // 计算卡尔曼增益：K = P * H^T * (H * P * H^T + R)^{-1}
        const H = this.observationMatrix;
        const HP = this.multiplyMatrices(H, this.covariance);
        const HPH_T = this.multiplyMatrices(HP, this.transposeMatrix(H));
        const S = this.addMatrices(HPH_T, this.observationNoise);
        const K = this.multiplyMatrices(
            this.multiplyMatrices(this.covariance, this.transposeMatrix(H)),
            this.invertMatrix(S)
        );

        // 更新状态：x = x + K * (z - H * x)
        const Hx = this.multiplyMatrices(H, stateAsColumn);
        const innovation = measurement.map((val, i) => val - Hx[i][0]);
        const K_innovation = this.multiplyMatrices(K, innovation.map(val => [val]));
        this.state = this.state.map((val, i) => val + K_innovation[i][0]);

        // 更新协方差：P = (I - K * H) * P
        const I = Array.from({ length: 10 }, (_, i) => 
            Array.from({ length: 10 }, (_, j) => i === j ? 1 : 0)
        );
        const KH = this.multiplyMatrices(K, H);
        const I_KH = I.map((row, i) => row.map((val, j) => val - KH[i][j]));
        this.covariance = this.multiplyMatrices(I_KH, this.covariance);
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
        this.timestamp = Date.now();
        this.q = { x: 0, y: 0, z: 0, w: 1 };
        this.X = this.Y = this.Z = 0;
        this.GX = this.GY = this.GZ = 0;
        this.AX = this.AY = this.AZ = 0;
        this.accelerations = [0, 0, 0];
        this.orientation = { x: 1, y: 0, z: 0, w: 1 };
        this.worldTransform = isIOS()
            ? Quaternion.fromAxisAngle( 1, 0, 0, -Math.PI / 2 ) // -90 degrees on x-axis
            : Quaternion.fromAxisAngle(0, 1, 0, Math.PI / 2); // 90 degrees on y-axis
        // use kf
        this.imuData = {
            quaternion: [0, 0, 0, 0], // 四元数 [q_x, q_y, q_z, q_w]
            angularVelocity: [0, 0, 0], // 角速度 [omega_x, omega_y, omega_z]
            acceleration: [0, 0, 0] // 加速度 [a_x, a_y, a_z]
        };
        this.KFimuData = {
            quaternion: [0, 0, 0, 0], // 四元数 [q_x, q_y, q_z, q_w]
            angularVelocity: [0, 0, 0], // 角速度 [omega_x, omega_y, omega_z]
            acceleration: [0, 0, 0] // 加速度 [a_x, a_y, a_z]
        };
        const handleDeviceOrientation = ( event ) =>
        {
            // axis orientation assumes device is placed on ground, screen upward
            const x = event.beta * deg2rad;    // X-axis (β) vertical tilt
            const y = event.gamma * deg2rad;   // Y-axis (γ) horizontal tilt
            const z = event.alpha * deg2rad;   // Z-axis (α) compass direction
            this.X = x; this.Y = y; this.Z = z;
/*
            console.log('x: ' + x); //new added output
            console.log('y: ' + y);
            console.log('z: ' + z);
*/
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

            this.GX = gx; this.GY = gy; this.GZ = gz;
            this.accelerations.push(ax); this.accelerations.push(ay); this.accelerations.push(az);
            const lengthToKeep = 5;
            while (this.accelerations.length > lengthToKeep * 3) {
                this.accelerations.shift();
            }
            this.AX = ax; this.AY = ay; this.AZ = az;
            
            // 逐个字段赋值
            this.imuData.quaternion[0] = this.q.x; // q_x
            this.imuData.quaternion[1] = this.q.y; // q_y
            this.imuData.quaternion[2] = this.q.z; // q_z
            this.imuData.quaternion[3] = this.q.w; // q_w

            this.imuData.angularVelocity[0] = gx; // omega_x
            this.imuData.angularVelocity[1] = gy; // omega_y
            this.imuData.angularVelocity[2] = gz; // omega_z

            this.imuData.acceleration[0] = ax; // a_x
            this.imuData.acceleration[1] = ay; // a_y
            this.imuData.acceleration[2] = az; // a_z

            const kf = new KalmanFilter();
            // 将IMU数据转换为状态向量
            const measurement = [
                ...this.imuData.quaternion,
                ...this.imuData.angularVelocity,
                ...this.imuData.acceleration
            ];

            // 预测和更新
            kf.predict();
            kf.update(measurement);
            [this.q.x, this.q.y, this.q.z, this.q.w, gx, gy, gz, ax, ay, az] = kf.state;
            this.KFimuData.quaternion[0] = this.q.x; // q_x
            this.KFimuData.quaternion[1] = this.q.y; // q_y
            this.KFimuData.quaternion[2] = this.q.z; // q_z
            this.KFimuData.quaternion[3] = this.q.w; // q_w

            this.KFimuData.angularVelocity[0] = gx; // omega_x
            this.KFimuData.angularVelocity[1] = gy; // omega_y
            this.KFimuData.angularVelocity[2] = gz; // omega_z

            this.KFimuData.acceleration[0] = ax; // a_x
            this.KFimuData.acceleration[1] = ay; // a_y
            this.KFimuData.acceleration[2] = az; // a_z
/*
            console.log('alpha: ' + gz);
            console.log('beta: ' + gx);
            console.log('gamma: ' + gy);
            
            console.log('ax: ' + ax); //new added output
            console.log('ay: ' + ay);
            console.log('az: ' + az);
*/
            const timestamp = Date.now();
            this.timestamp = timestamp;
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

    clear()
    {
        this.motion.length = 0;
    }
}

export { IMU };
