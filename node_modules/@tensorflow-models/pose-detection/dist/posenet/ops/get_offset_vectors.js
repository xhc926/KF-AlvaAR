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
exports.getOffsetVectorsGPU = void 0;
var tfwebgpu = require("@tensorflow/tfjs-backend-webgpu");
var tf = require("@tensorflow/tfjs-core");
var get_offset_vectors_webgpu_1 = require("./get_offset_vectors_webgpu");
function getOffsetVectorsGPU(a, b) {
    if (tf.backend() instanceof tfwebgpu.WebGPUBackend) {
        return (0, get_offset_vectors_webgpu_1.getOffsetVectorsWebGPU)(a, b);
    }
    throw new Error('getOffsetVectorsGPU is not supported in this backend!');
}
exports.getOffsetVectorsGPU = getOffsetVectorsGPU;
//# sourceMappingURL=get_offset_vectors.js.map