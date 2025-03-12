"use strict";
/**
 * @license
 * Copyright 2021 Google LLC. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =============================================================================
 */
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g;
    return g = { next: verb(0), "throw": verb(1), "return": verb(2) }, typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (_) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.load = void 0;
var tfconv = require("@tensorflow/tfjs-converter");
var tf = require("@tensorflow/tfjs-core");
var convert_image_to_tensor_1 = require("../shared/calculators/convert_image_to_tensor");
var image_utils_1 = require("../shared/calculators/image_utils");
var shift_image_value_1 = require("../shared/calculators/shift_image_value");
var decode_multiple_poses_1 = require("./calculators/decode_multiple_poses");
var decode_single_pose_1 = require("./calculators/decode_single_pose");
var flip_poses_1 = require("./calculators/flip_poses");
var scale_poses_1 = require("./calculators/scale_poses");
var constants_1 = require("./constants");
var detector_utils_1 = require("./detector_utils");
var load_utils_1 = require("./load_utils");
/**
 * PoseNet detector class.
 */
var PosenetDetector = /** @class */ (function () {
    function PosenetDetector(posenetModel, config) {
        this.posenetModel = posenetModel;
        // validate params.
        var inputShape = this.posenetModel.inputs[0].shape;
        tf.util.assert((inputShape[1] === -1) && (inputShape[2] === -1), function () { return "Input shape [".concat(inputShape[1], ", ").concat(inputShape[2], "] ") +
            "must both be equal to or -1"; });
        var validInputResolution = (0, load_utils_1.getValidInputResolutionDimensions)(config.inputResolution, config.outputStride);
        (0, detector_utils_1.assertValidOutputStride)(config.outputStride);
        (0, detector_utils_1.assertValidResolution)(validInputResolution, config.outputStride);
        this.inputResolution = validInputResolution;
        this.outputStride = config.outputStride;
        this.architecture = config.architecture;
    }
    /**
     * Estimates poses for an image or video frame.
     *
     * This does standard ImageNet pre-processing before inferring through the
     * model. The image should pixels should have values [0-255]. It returns a
     * single pose or multiple poses based on the maxPose parameter from the
     * `config`.
     *
     * @param image
     * ImageData|HTMLImageElement|HTMLCanvasElement|HTMLVideoElement The input
     * image to feed through the network.
     *
     * @param estimationConfig
     *       maxPoses: Optional. Max number of poses to estimate.
     *       When maxPoses = 1, a single pose is detected, it is usually much more
     *       efficient than maxPoses > 1. When maxPoses > 1, multiple poses are
     *       detected.
     *
     *       flipHorizontal: Optional. Default to false. When image data comes
     *       from camera, the result has to flip horizontally.
     *
     * @return An array of `Pose`s.
     */
    PosenetDetector.prototype.estimatePoses = function (image, estimationConfig) {
        if (estimationConfig === void 0) { estimationConfig = constants_1.SINGLE_PERSON_ESTIMATION_CONFIG; }
        return __awaiter(this, void 0, void 0, function () {
            return __generator(this, function (_a) {
                return [2 /*return*/, this.estimatePosesGPU(image, estimationConfig, false)];
            });
        });
    };
    /**
     * Estimates poses for an image or video frame, optionally supports gpu
     * rendering.
     *
     * This does standard ImageNet pre-processing before inferring through the
     * model. The image should pixels should have values [0-255]. It returns a
     * single pose or multiple poses based on the maxPose parameter from the
     * `config`.
     *
     * @param image
     * ImageData|HTMLImageElement|HTMLCanvasElement|HTMLVideoElement The input
     * image to feed through the network.
     *
     * @param estimationConfig
     *       maxPoses: Optional. Max number of poses to estimate.
     *       When maxPoses = 1, a single pose is detected, it is usually much more
     *       efficient than maxPoses > 1. When maxPoses > 1, multiple poses are
     *       detected.
     *
     *       flipHorizontal: Optional. Default to false. When image data comes
     *       from camera, the result has to flip horizontally.
     *
     * @param useGpuRenderer
     *        Whether rendering predict results with gpu or not.
     *
     * @return If not rendering with gpu, an array of poses, each pose contains an
     *     array of `Keypoint`s. Otherwise an array of tensor, and canvas info.
     */
    PosenetDetector.prototype.estimatePosesGPU = function (image, estimationConfig, useGpuRenderer) {
        if (estimationConfig === void 0) { estimationConfig = constants_1.SINGLE_PERSON_ESTIMATION_CONFIG; }
        if (useGpuRenderer === void 0) { useGpuRenderer = false; }
        return __awaiter(this, void 0, void 0, function () {
            var config, _a, imageTensor, padding, imageValueShifted, results, offsets, heatmap, displacementFwd, displacementBwd, heatmapScores, poses, _b, pose, score, pose, canvasInfo, scaledPoses, imageSize;
            return __generator(this, function (_c) {
                switch (_c.label) {
                    case 0:
                        config = (0, detector_utils_1.validateEstimationConfig)(estimationConfig);
                        if (image == null) {
                            return [2 /*return*/, useGpuRenderer ? [[], []] : []];
                        }
                        this.maxPoses = config.maxPoses;
                        _a = (0, convert_image_to_tensor_1.convertImageToTensor)(image, {
                            outputTensorSize: this.inputResolution,
                            keepAspectRatio: true,
                            borderMode: 'replicate'
                        }), imageTensor = _a.imageTensor, padding = _a.padding;
                        imageValueShifted = this.architecture === 'ResNet50' ?
                            tf.add(imageTensor, constants_1.RESNET_MEAN) :
                            (0, shift_image_value_1.shiftImageValue)(imageTensor, [-1, 1]);
                        results = this.posenetModel.predict(imageValueShifted);
                        if (this.architecture === 'ResNet50') {
                            offsets = tf.squeeze(results[2], [0]);
                            heatmap = tf.squeeze(results[3], [0]);
                            displacementFwd = tf.squeeze(results[0], [0]);
                            displacementBwd = tf.squeeze(results[1], [0]);
                        }
                        else {
                            offsets = tf.squeeze(results[0], [0]);
                            heatmap = tf.squeeze(results[1], [0]);
                            displacementFwd = tf.squeeze(results[2], [0]);
                            displacementBwd = tf.squeeze(results[3], [0]);
                        }
                        heatmapScores = tf.sigmoid(heatmap);
                        if (!(this.maxPoses === 1)) return [3 /*break*/, 5];
                        if (!useGpuRenderer) return [3 /*break*/, 2];
                        return [4 /*yield*/, (0, decode_single_pose_1.decodeSinglePoseGPU)(heatmapScores, offsets, this.outputStride)];
                    case 1:
                        _b = _c.sent(), pose = _b[0], score = _b[1];
                        poses = [pose, score];
                        return [3 /*break*/, 4];
                    case 2: return [4 /*yield*/, (0, decode_single_pose_1.decodeSinglePose)(heatmapScores, offsets, this.outputStride)];
                    case 3:
                        pose = _c.sent();
                        poses = [pose];
                        _c.label = 4;
                    case 4: return [3 /*break*/, 7];
                    case 5:
                        if (useGpuRenderer) {
                            throw new Error('GPU renderer only supports single pose!');
                        }
                        return [4 /*yield*/, (0, decode_multiple_poses_1.decodeMultiplePoses)(heatmapScores, offsets, displacementFwd, displacementBwd, this.outputStride, this.maxPoses, config.scoreThreshold, config.nmsRadius)];
                    case 6:
                        poses = _c.sent();
                        _c.label = 7;
                    case 7:
                        if (useGpuRenderer) {
                            // TODO: handle flipPosesHorizontal in GPU.
                            if (config.flipHorizontal === true) {
                                throw new Error('flipHorizontal is not supported!');
                            }
                            canvasInfo = this.getCanvasInfo((0, image_utils_1.getImageSize)(image), this.inputResolution, padding);
                        }
                        else {
                            imageSize = (0, image_utils_1.getImageSize)(image);
                            scaledPoses =
                                (0, scale_poses_1.scalePoses)(poses, imageSize, this.inputResolution, padding);
                            if (config.flipHorizontal) {
                                scaledPoses = (0, flip_poses_1.flipPosesHorizontal)(scaledPoses, imageSize);
                            }
                        }
                        imageTensor.dispose();
                        imageValueShifted.dispose();
                        tf.dispose(results);
                        offsets.dispose();
                        heatmap.dispose();
                        displacementFwd.dispose();
                        displacementBwd.dispose();
                        heatmapScores.dispose();
                        return [2 /*return*/, useGpuRenderer ? [poses, canvasInfo] : scaledPoses];
                }
            });
        });
    };
    PosenetDetector.prototype.getCanvasInfo = function (imageSize, inputResolution, padding) {
        var height = imageSize.height, width = imageSize.width;
        var scaleY = height / (inputResolution.height * (1 - padding.top - padding.bottom));
        var scaleX = width / (inputResolution.width * (1 - padding.left - padding.right));
        var offsetY = -padding.top * inputResolution.height;
        var offsetX = -padding.left * inputResolution.width;
        return [
            offsetX, offsetY, scaleX, scaleY, imageSize.width, imageSize.height
        ];
    };
    PosenetDetector.prototype.dispose = function () {
        this.posenetModel.dispose();
    };
    PosenetDetector.prototype.reset = function () {
        // No-op. There's no global state.
    };
    return PosenetDetector;
}());
/**
 * Loads the PoseNet model instance from a checkpoint, with the ResNet
 * or MobileNet architecture. The model to be loaded is configurable using the
 * config dictionary ModelConfig. Please find more details in the
 * documentation of the ModelConfig.
 *
 * @param config ModelConfig dictionary that contains parameters for
 * the PoseNet loading process. Please find more details of each parameters
 * in the documentation of the ModelConfig interface. The predefined
 * `MOBILENET_V1_CONFIG` and `RESNET_CONFIG` can also be used as references
 * for defining your customized config.
 */
function load(modelConfig) {
    if (modelConfig === void 0) { modelConfig = constants_1.MOBILENET_V1_CONFIG; }
    return __awaiter(this, void 0, void 0, function () {
        var config, defaultUrl_1, model_1, defaultUrl, model;
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    config = (0, detector_utils_1.validateModelConfig)(modelConfig);
                    if (!(config.architecture === 'ResNet50')) return [3 /*break*/, 2];
                    defaultUrl_1 = (0, load_utils_1.resNet50Checkpoint)(config.outputStride, config.quantBytes);
                    return [4 /*yield*/, tfconv.loadGraphModel(config.modelUrl || defaultUrl_1)];
                case 1:
                    model_1 = _a.sent();
                    return [2 /*return*/, new PosenetDetector(model_1, config)];
                case 2:
                    defaultUrl = (0, load_utils_1.mobileNetCheckpoint)(config.outputStride, config.multiplier, config.quantBytes);
                    return [4 /*yield*/, tfconv.loadGraphModel(config.modelUrl || defaultUrl)];
                case 3:
                    model = _a.sent();
                    return [2 /*return*/, new PosenetDetector(model, config)];
            }
        });
    });
}
exports.load = load;
//# sourceMappingURL=detector.js.map