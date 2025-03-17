// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

import { Input } from "./types";
import { PointCloud, PackedElementField } from "@foxglove/schemas";

// Define constants
const BYTE_SIZE = {
    FLOAT32: 4,
    UINT8: 1
};

const COLOR_MAP_SIZE = 255;
const MAX_HUE = 360;
const MAX_SATURATION = 100;
const MAX_VALUE = 100;
const ALPHA_MAX = 255;

// Export script configuration
export const inputs = ["/voyant_points"];
export const output = "/rng_color_pointcloud";

interface GlobalVariables {
    range_band: number;
}

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

    let r = 0, g = 0, b = 0;
    switch (i % 6) {
        case 0: r = vNorm; g = t; b = p; break;
        case 1: r = q; g = vNorm; b = p; break;
        case 2: r = p; g = vNorm; b = t; break;
        case 3: r = p; g = q; b = vNorm; break;
        case 4: r = t; g = p; b = vNorm; break;
        case 5: r = vNorm; g = p; b = q; break;
    }

    return {
        r: Math.round(r * 255),
        g: Math.round(g * 255),
        b: Math.round(b * 255)
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
    value: number
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
    bandSize: number
): number[][] {
    // Normalize ranges into bands
    const normalizedValues = rangeValues.map(val => (val % bandSize) / bandSize);

    // Map normalized values to color indices
    const colorIndices = normalizedValues.map(val => Math.floor(val * (COLOR_MAP_SIZE - 1)));

    // Return RGB values from the color map
    return colorIndices.map(index => {
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
    MAX_VALUE
);

// Define point cloud field structure
const pointCloudFields: PackedElementField[] = [
    { name: "x", offset: 0, type: 7 },
    { name: "y", offset: 4, type: 7 },
    { name: "z", offset: 8, type: 7 },
    { name: "v", offset: 16, type: 7 },
    { name: "snr", offset: 20, type: 7 },
    { name: "drop_reason", offset: 24, type: 2 },
    { name: "timestamp_nsecs", offset: 26, type: 6 },
    { name: "point_idx", offset: 30, type: 5 },
    { name: "red", offset: 48, type: 1 },
    { name: "green", offset: 49, type: 1 },
    { name: "blue", offset: 50, type: 1 },
    { name: "alpha", offset: 51, type: 1 }
];

/**
 * Main script function that processes the point cloud data
 * @param event - Input event containing the point cloud message
 * @param globalVars - Global variables including range_band
 * @returns Modified point cloud with color data
 */
export default function script(
    event: Input<"/voyant_points">,
    globalVars: GlobalVariables
): PointCloud {
    const originalData = event.message.data;
    const originalStride = event.message.point_step;
    const newStride = originalStride + (4 * BYTE_SIZE.UINT8); // Adding RGBA fields (4 bytes)
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
    const colorValues = mapRangeToRgb(rangeColorMap, rangeValues, globalVars.range_band);

    // Create new data buffer for the output point cloud
    const newPointCloudData = new Uint8Array(newSize);

    // Copy original data and add color information
    for (let i = 0; i < numPoints; i++) {
        const originalOffset = i * originalStride;
        const newOffset = i * newStride;

        // Copy original point data
        newPointCloudData.set(
            originalData.subarray(originalOffset, originalOffset + originalStride),
            newOffset
        );

        // Add color information
        const color = i < colorValues.length ? colorValues[i] : [0, 0, 0];
        newPointCloudData.set([
            color[0] || 0,    // Red
            color[1] || 0,    // Green
            color[2] || 0,    // Blue
            ALPHA_MAX         // Alpha (fully opaque)
        ], newOffset + 48);
    }

    // Return the modified point cloud
    return {
        timestamp: {
            sec: event.message.header.stamp.sec,
            nsec: event.message.header.stamp.nsec,
        },
        frame_id: event.message.header.frame_id,
        pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        point_stride: newStride,
        fields: pointCloudFields,
        data: newPointCloudData,
    };
}
