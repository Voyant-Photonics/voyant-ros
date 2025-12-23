// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

import { Input } from "./types";
import { PointCloud, PackedElementField } from "@foxglove/schemas";

// Define constants
const BYTE_SIZE = {
    FLOAT32: 4,
    UINT8: 1,
    UINT32: 6,
    INT32: 5,
    FLOAT_32: 7,
};

const COLOR_MAP_SIZE = 255;
const MAX_HUE = 360;
const MAX_SATURATION = 100;
const MAX_VALUE = 100;
const ALPHA_MAX = 255;

// The input and output topics for your script
export const inputs = ["/point_cloud"];
export const output = "/rng_color_pointcloud";

type GlobalVariables = {
    range_band: number;
};

interface RGB {
    r: number;
    g: number;
    b: number;
}

/**
 * Converts HSV color values to RGB
 * @param h - Hue (0-360)
 * @param s - Saturation (0-100)
 * @param v - Value (0-100)
 * @returns RGB color object with values 0-255
 */
function hsvToRgb(h: number, s: number, v: number): RGB {
    // Normalize values to [0,1] range
    const hNorm = h / MAX_HUE;
    const sNorm = s / MAX_SATURATION;
    const vNorm = v / MAX_VALUE;

    const i = Math.floor(hNorm * 6);
    const f = hNorm * 6 - i;
    const p = vNorm * (1 - sNorm);
    const q = vNorm * (1 - f * sNorm);
    const t = vNorm * (1 - (1 - f) * sNorm);

    let r = 0,
        g = 0,
        b = 0;
    switch (i % 6) {
        case 0:
            r = vNorm;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = vNorm;
            b = p;
            break;
        case 2:
            r = p;
            g = vNorm;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = vNorm;
            break;
        case 4:
            r = t;
            g = p;
            b = vNorm;
            break;
        case 5:
            r = vNorm;
            g = p;
            b = q;
            break;
    }

    return {
        r: Math.round(r * 255),
        g: Math.round(g * 255),
        b: Math.round(b * 255),
    };
}

/**
 * Converts a Uint8Array (4 bytes) to a Float32 value
 * @param bytes - Uint8Array containing 4 bytes
 * @returns Float32 value
 */
function uint8ArrayToFloat32(bytes: Uint8Array): number {
    const dataView = new DataView(bytes.buffer);
    return dataView.getFloat32(0, true); // true for little-endian
}

/**
 * Creates a cyclic color map with evenly distributed hues
 * @param numColors - Number of colors in the map
 * @param saturation - Saturation value (0-100)
 * @param value - Brightness value (0-100)
 * @returns 2D array of RGB values [r,g,b]
 */
function createCyclicColorMap(
    numColors: number,
    saturation: number,
    value: number,
): number[][] {
    const colorMap: number[][] = [];

    for (let i = 0; i < numColors; i++) {
        const hue = (i / numColors) * MAX_HUE;
        const { r, g, b } = hsvToRgb(hue, saturation, value);
        colorMap.push([r, g, b]);
    }

    return colorMap;
}

/**
 * Maps range values to RGB colors using the provided color map
 * @param colorMap - 2D array of RGB values
 * @param rangeValues - Array of range values to map
 * @param bandSize - Size of each color band
 * @returns 2D array of RGB values for each input range value
 */
function mapRangeToRgb(
    colorMap: number[][],
    rangeValues: number[],
    bandSize: number,
): number[][] {
    // Normalize ranges into bands
    const normalizedValues = rangeValues.map(
        (val) => (val % bandSize) / bandSize,
    );

    // Map normalized values to color indices
    const colorIndices = normalizedValues.map((val) =>
        Math.floor(val * (COLOR_MAP_SIZE - 1)),
    );

    // Return RGB values from the color map
    return colorIndices.map((index) => {
        if (index >= 0 && index < colorMap.length) {
            return colorMap[index];
        }
        return [0, 0, 0]; // Fallback for out-of-range indices
    });
}

// Create a color map once for reuse
const rangeColorMap: number[][] = createCyclicColorMap(
    COLOR_MAP_SIZE,
    MAX_SATURATION,
    MAX_VALUE,
);

// Define point cloud field structure
const pointCloudFields: PackedElementField[] = [
    { name: "x", offset: 0, type: BYTE_SIZE.FLOAT_32 },
    { name: "y", offset: 4, type: BYTE_SIZE.FLOAT_32 },
    { name: "z", offset: 8, type: BYTE_SIZE.FLOAT_32 },
    { name: "v", offset: 16, type: BYTE_SIZE.FLOAT_32 },
    { name: "snr", offset: 20, type: BYTE_SIZE.FLOAT_32 },
    { name: "drop_reason", offset: 24, type: 2 },
    { name: "timestamp_nsecs", offset: 26, type: BYTE_SIZE.UINT32 },
    { name: "point_idx", offset: 30, type: BYTE_SIZE.INT32 },
    { name: "red", offset: 48, type: BYTE_SIZE.UINT8 },
    { name: "green", offset: 49, type: BYTE_SIZE.UINT8 },
    { name: "blue", offset: 50, type: BYTE_SIZE.UINT8 },
    { name: "alpha", offset: 51, type: BYTE_SIZE.UINT8 },
];

/**
 * Main script function that processes the point cloud data
 * @param event - Input event containing the point cloud message
 * @param globalVars - Global variables including range_band
 * @returns Modified point cloud with color data
 */
export default function script(
    event: Input<"/point_cloud">,
    globalVars: GlobalVariables,
): PointCloud {
    // Define interfaces that match the message structure
    interface ROS2PointCloudMessage {
        row_step: number;
        point_step: number;
        data: Uint8Array;
        header: {
            stamp: {
                sec: number;
                nsec: number;
            };
            frame_id: string;
        };
    }

    interface APIPointCloudMessage {
        point_stride: number;
        data: Uint8Array;
        timestamp: {
            sec: number;
            nsec: number;
        };
        frame_id: string;
        fields: any[];
    }

    // Type guard functions, this is used because TypeScript can't infer the type of the message during compilation, but we can check it at runtime
    // Ref: https://www.typescriptlang.org/docs/handbook/advanced-types.html#using-the-in-operator
    function isROS2PointCloud(message: any): message is ROS2PointCloudMessage {
        return "row_step" in message && "point_step" in message;
    }

    function isAPIPointCloud(message: any): message is APIPointCloudMessage {
        return "point_stride" in message;
    }

    if (isROS2PointCloud(event.message)) {
        // Process ROS2 point cloud message
        const {
            data,
            point_step: originalStride,
            header: ros_header,
        } = event.message;

        return processROS2PointCloud(
            data,
            originalStride,
            ros_header,
            globalVars,
        );
    } else if (isAPIPointCloud(event.message)) {
        // Process API point cloud message
        return processAPIPointCloud(event.message, globalVars);
    } else {
        // Throw an error if the message format is unknown i.e other than sensor_msgs/PointCloud2 or PointCloud
        throw new Error("Unknown point cloud message format");
    }
}

/**
 * Processes a ROS2 PointCloud2 message and adds color information based on SNR values
 * @param data Point cloud data as Uint8Array
 * @param originalStride Original point stride in bytes
 * @param ros_header ROS2 message header
 * @param globalVars Global variables for SNR bounds
 * @returns Modified PointCloud message with color information
 */
function processROS2PointCloud(
    data: Uint8Array,
    originalStride: number,
    ros_header: {
        stamp: {
            sec: number;
            nsec: number;
        };
        frame_id: string;
    },
    globalVars: GlobalVariables,
) {
    const originalData = data;
    const newStride = originalStride + 4 * BYTE_SIZE.UINT8; // Adding RGBA fields (4 bytes)
    const numPoints = Math.floor(originalData.length / originalStride);
    const newSize = numPoints * newStride;

    // Extract range values from the original point cloud
    const rangeValues: number[] = [];
    for (let i = 0; i < originalData.length; i += originalStride) {
        if (i + BYTE_SIZE.FLOAT32 <= originalData.length) {
            const rangeBytes = new Uint8Array(BYTE_SIZE.FLOAT32);
            rangeBytes.set(originalData.subarray(i, i + BYTE_SIZE.FLOAT32));
            rangeValues.push(uint8ArrayToFloat32(rangeBytes));
        }
    }

    // Map range values to colors
    const colorValues = mapRangeToRgb(
        rangeColorMap,
        rangeValues,
        globalVars.range_band,
    );

    // Create new data buffer for the output point cloud
    const newPointCloudData = new Uint8Array(newSize);

    // Copy original data and add color information
    for (let i = 0; i < numPoints; i++) {
        const originalOffset = i * originalStride;
        const newOffset = i * newStride;

        // Copy original point data
        newPointCloudData.set(
            originalData.subarray(
                originalOffset,
                originalOffset + originalStride,
            ),
            newOffset,
        );

        // Add color information
        const color = i < colorValues.length ? colorValues[i] : [0, 0, 0];
        newPointCloudData.set(
            [
                color[0] || 0, // Red
                color[1] || 0, // Green
                color[2] || 0, // Blue
                ALPHA_MAX, // Alpha (fully opaque)
            ],
            newOffset + 48,
        );
    }

    // Return the modified point cloud
    return {
        timestamp: {
            sec: ros_header.stamp.sec,
            nsec: ros_header.stamp.nsec,
        },
        frame_id: ros_header.frame_id,
        pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        point_stride: newStride,
        fields: pointCloudFields,
        data: newPointCloudData,
    };
}

/**
 * Processes an API PointCloud message and adds color information based on SNR values
 * @param api_message API PointCloud message
 * @param globalVars Global variables for SNR bounds
 * @returns Modified PointCloud message with color information
 */
function processAPIPointCloud(
    api_message: {
        data: Uint8Array;
        point_stride: number;
        timestamp: {
            sec: number;
            nsec: number;
        };
        frame_id: string;
        fields: any[];
    },
    globalVars: GlobalVariables,
) {
    // Get the original point cloud data as a Uint8Array
    const { data, point_stride: old_strid } = api_message;
    const new_strid = old_strid + 4 * BYTE_SIZE.UINT8; // 11 fields of 4 bytes each + 4 fields of 1 byte each (RGBA)
    const numPoints = Math.floor(data.length / old_strid);
    const new_size = numPoints * new_strid;
    const new_point_cloud_data = new Uint8Array(new_size);

    // Extract range values from the original point cloud
    const rangeValues: number[] = [];
    for (let i = 0; i < data.length; i += old_strid) {
        if (i + 12 <= data.length) {
            const rangeBytes = new Uint8Array(BYTE_SIZE.FLOAT32);
            rangeBytes.set(data.subarray(i + 8, i + 12));
            rangeValues.push(uint8ArrayToFloat32(rangeBytes));
        }
    }

    // Map range values to colors
    const colorValues = mapRangeToRgb(
        rangeColorMap,
        rangeValues,
        globalVars.range_band,
    );

    // Copy original data and add color information
    for (let i = 0; i < numPoints; i++) {
        const originalOffset = i * old_strid;
        const newOffset = i * new_strid;

        // Copy original point data
        new_point_cloud_data.set(
            data.subarray(originalOffset, originalOffset + old_strid),
            newOffset,
        );

        // Add color information
        const color = i < colorValues.length ? colorValues[i] : [0, 0, 0];
        new_point_cloud_data.set(
            [
                color[0] || 0, // Red
                color[1] || 0, // Green
                color[2] || 0, // Blue
                ALPHA_MAX, // Alpha (fully opaque)
            ],
            newOffset + old_strid,
        );
    }

    // Return the modified point cloud message
    return {
        timestamp: {
            sec: api_message.timestamp.sec,
            nsec: api_message.timestamp.nsec,
        },
        frame_id: api_message.frame_id,
        pose: {
            position: {
                x: 0,
                y: 0,
                z: 0,
            },
            orientation: {
                x: 0,
                y: 0,
                z: 0,
                w: 1,
            },
        },
        point_stride: new_strid,
        fields: api_message.fields.concat(
            { name: "red", offset: 44, type: 1 },
            { name: "green", offset: 45, type: 1 },
            { name: "blue", offset: 46, type: 1 },
            { name: "alpha", offset: 47, type: 1 },
        ),
        data: new_point_cloud_data,
    };
}
