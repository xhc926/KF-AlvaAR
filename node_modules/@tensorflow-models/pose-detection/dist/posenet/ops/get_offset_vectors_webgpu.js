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
exports.getOffsetVectorsWebGPU = void 0;
var tfwebgpu = require("@tensorflow/tfjs-backend-webgpu");
var tf = require("@tensorflow/tfjs-core");
var webgpu_util_1 = require("./webgpu_util");
var GetOffsetVectorsProgram = /** @class */ (function () {
    function GetOffsetVectorsProgram(outputShape) {
        // A is heatmapScores, B is heatMapCoords.
        this.variableNames = ['A', 'B'];
        this.size = true;
        this.supportedLastDimension = 2;
        // Only 2d tensor whose last dimension is 2 is supported.
        if (outputShape.length !== 2 ||
            outputShape[1] !== this.supportedLastDimension) {
            throw new Error("GetOffsetVectorsProgram only supports shape of [x, ".concat(this.supportedLastDimension, "], but current shape is ").concat(outputShape));
        }
        var workgroupSizeX = 32;
        this.workgroupSize = [workgroupSizeX, 1, 1];
        this.outputShape = outputShape;
        var computeDispatchInfo = [outputShape[0], 1];
        this.dispatchLayout =
            tfwebgpu.webgpu_util.flatDispatchLayout(computeDispatchInfo);
        this.dispatch = tfwebgpu.webgpu_util.computeDispatch(this.dispatchLayout, computeDispatchInfo, this.workgroupSize);
        this.shaderKey = 'GetOffsetVectors';
    }
    GetOffsetVectorsProgram.prototype.getUserCode = function () {
        return "\n    fn getOffsetPoint(y: i32, x: i32, index: i32) -> vec2<i32> {\n      let outIndexY = y * uniforms.bShape.x * uniforms.bShape.y + x * uniforms.bShape.y + index;\n      let outIndexX = outIndexY + uniforms.bShape.z;\n      let outY = i32(B[outIndexY]);\n      let outX = i32(B[outIndexX]);\n      return vec2<i32>(outY, outX);\n    }\n\n    ".concat((0, webgpu_util_1.getMainHeaderString)('index'), " {\n      if (index < uniforms.size) {\n        let indexY = index * ").concat(this.supportedLastDimension, ";\n        let indexX = indexY + 1;\n        let heatmapY = A[indexY];\n        let heatmapX = A[indexX];\n        let out = getOffsetPoint(i32(heatmapY), i32(heatmapX), index);\n        result[indexY] = f32(out[0]);\n        result[indexX] = f32(out[1]);\n      }\n    }\n    ");
    };
    return GetOffsetVectorsProgram;
}());
function getOffsetVectorsWebGPU(a, b) {
    var webgpuBackend = tf.backend();
    var program = new GetOffsetVectorsProgram(a.shape);
    var outInfo = webgpuBackend.runWebGPUProgram(program, [a, b], 'float32');
    var value = tf.engine().makeTensorFromTensorInfo(outInfo);
    return value;
}
exports.getOffsetVectorsWebGPU = getOffsetVectorsWebGPU;
//# sourceMappingURL=get_offset_vectors_webgpu.js.map