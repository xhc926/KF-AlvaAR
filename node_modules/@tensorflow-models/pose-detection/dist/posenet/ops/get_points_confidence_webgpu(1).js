"use strict";
/**
 * @license
 * Copyright 2023 Google LLC.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =============================================================================
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.getPointsConfidenceWebGPU = void 0;
var tfwebgpu = require("@tensorflow/tfjs-backend-webgpu");
var tf = require("@tensorflow/tfjs-core");
var webgpu_util_1 = require("./webgpu_util");
var GetpointsConfidenceProgram = /** @class */ (function () {
    function GetpointsConfidenceProgram(bShape) {
        // A is heatmapScores, B is heatmapValues.
        this.variableNames = ['A', 'B'];
        this.size = true;
        var workgroupSizeX = 32;
        this.workgroupSize = [workgroupSizeX, 1, 1];
        this.outputShape = [bShape[0], 1];
        this.dispatchLayout =
            tfwebgpu.webgpu_util.flatDispatchLayout(this.outputShape);
        this.dispatch = tfwebgpu.webgpu_util.computeDispatch(this.dispatchLayout, this.outputShape, this.workgroupSize);
        this.shaderKey = 'getpointsConfidenceOp';
    }
    GetpointsConfidenceProgram.prototype.getUserCode = function () {
        return "\n        ".concat((0, webgpu_util_1.getMainHeaderString)('index'), " {\n          if (index < uniforms.size) {\n            let y = B[index * 2];\n            let x = B[index * 2 + 1];\n            let outIndex = y * uniforms.aShape.x * uniforms.aShape.z + x * uniforms.aShape.z + index;\n            result[index] = A[outIndex];\n          }\n        }\n        ");
    };
    return GetpointsConfidenceProgram;
}());
function getPointsConfidenceWebGPU(a, b) {
    var webgpuBackend = tf.backend();
    var program = new GetpointsConfidenceProgram(b.shape);
    var outInfo = webgpuBackend.runWebGPUProgram(program, [a, b], 'float32');
    var value = tf.engine().makeTensorFromTensorInfo(outInfo);
    return value;
}
exports.getPointsConfidenceWebGPU = getPointsConfidenceWebGPU;
//# sourceMappingURL=get_points_confidence_webgpu.js.map