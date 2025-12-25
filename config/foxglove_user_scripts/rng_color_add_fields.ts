// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

import { Input } from "./types";
import { PointCloud, PackedElementField } from "@foxglove/schemas";

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

// Output point cloud field structure with XYZRGBA
const XYZRGBA_FIELDS: PackedElementField[] = [
    { name: "x", offset: 0, type: 7 }, // FLOAT32
    { name: "y", offset: 4, type: 7 }, // FLOAT32
    { name: "z", offset: 8, type: 7 }, // FLOAT32
    { name: "red", offset: 12, type: 1 }, // UINT8
    { name: "green", offset: 13, type: 1 }, // UINT8
    { name: "blue", offset: 14, type: 1 }, // UINT8
    { name: "alpha", offset: 15, type: 1 }, // UINT8
];

const XYZRGBA_STRIDE = 16;

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

    // Type guard functions
    // This is used because TypeScript can't infer the type of the message during compilation, but we can check it at runtime
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
        throw new Error("Unknown point cloud message format");
    }
}

/**
 * Extracts XYZ and range values from point cloud data
 * Range is calculated as sqrt(x^2 + y^2 + z^2)
 * @param data Raw point cloud data
 * @param stride Point stride in bytes
 * @param numPoints Number of points in the cloud
 * @param XYZ_OFFSET Offset of the XYZ fields in bytes
 * @returns Object containing xyz positions and range values
 */
function extractXYZAndRange(
    data: Uint8Array,
    stride: number,
    numPoints: number,
    XYZ_OFFSET: number,
): { xyz: Float32Array; rangeValues: number[] } {
    const xyz = new Float32Array(numPoints * 3);
    const rangeValues: number[] = [];

    for (let i = 0; i < numPoints; i++) {
        const pointOffset = i * stride;

        // Extract XYZ (12 bytes starting at XYZ_OFFSET)
        const xyzView = new DataView(
            data.buffer,
            data.byteOffset + pointOffset,
            stride,
        );
        const x = xyzView.getFloat32(XYZ_OFFSET, true);
        const y = xyzView.getFloat32(XYZ_OFFSET + 4, true);
        const z = xyzView.getFloat32(XYZ_OFFSET + 8, true);

        xyz[i * 3] = x;
        xyz[i * 3 + 1] = y;
        xyz[i * 3 + 2] = z;

        // Calculate range
        const range = Math.sqrt(x * x + y * y + z * z);
        rangeValues.push(range);
    }

    return { xyz, rangeValues };
}

/**
 * Creates XYZRGBA point cloud from positions and colors
 * @param xyz Float32Array of XYZ positions (length = numPoints * 3)
 * @param colors RGB color values for each point
 * @param numPoints Number of points in the cloud
 * @returns New Uint8Array with XYZRGBA data
 */
function createXYZRGBAPointCloud(
    xyz: Float32Array,
    colors: number[][],
    numPoints: number,
): Uint8Array {
    const newSize = numPoints * XYZRGBA_STRIDE;
    const newData = new Uint8Array(newSize);

    for (let i = 0; i < numPoints; i++) {
        const offset = i * XYZRGBA_STRIDE;

        // Write XYZ (12 bytes)
        const xyzView = new DataView(newData.buffer, offset, 12);
        xyzView.setFloat32(0, xyz[i * 3], true); // x
        xyzView.setFloat32(4, xyz[i * 3 + 1], true); // y
        xyzView.setFloat32(8, xyz[i * 3 + 2], true); // z

        // Write RGBA (4 bytes)
        const color = i < colors.length ? colors[i] : [0, 0, 0];
        newData[offset + 12] = color[0] || 0; // r
        newData[offset + 13] = color[1] || 0; // g
        newData[offset + 14] = color[2] || 0; // b
        newData[offset + 15] = ALPHA_MAX; // alpha
    }

    return newData;
}

/**
 * Processes a ROS2 PointCloud2 message and adds color information based on range values
 * @param data Point cloud data as Uint8Array
 * @param originalStride Original point stride in bytes
 * @param ros_header ROS2 message header
 * @param globalVars Global variables for range bounds
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
    const XYZ_OFFSET = 0;
    const numPoints = Math.floor(data.length / originalStride);

    // Extract XYZ and range values
    const { xyz, rangeValues } = extractXYZAndRange(
        data,
        originalStride,
        numPoints,
        XYZ_OFFSET,
    );

    // Map range values to colors
    const colorValues = mapRangeToRgb(
        rangeColorMap,
        rangeValues,
        globalVars.range_band,
    );

    // Create XYZRGBA point cloud
    const xyzrgbaData = createXYZRGBAPointCloud(xyz, colorValues, numPoints);

    // Return the modified point cloud message
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
        point_stride: XYZRGBA_STRIDE,
        fields: XYZRGBA_FIELDS,
        data: xyzrgbaData,
    };
}

/**
 * Processes an API PointCloud message and adds color information based on range values
 * @param api_message API PointCloud message
 * @param globalVars Global variables for range bounds
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
    const XYZ_OFFSET = 8;
    const { data, point_stride: old_strid } = api_message;
    const numPoints = Math.floor(data.length / old_strid);

    // Extract XYZ and range values
    const { xyz, rangeValues } = extractXYZAndRange(
        data,
        old_strid,
        numPoints,
        XYZ_OFFSET,
    );

    // Map range values to colors
    const colorValues = mapRangeToRgb(
        rangeColorMap,
        rangeValues,
        globalVars.range_band,
    );

    // Create XYZRGBA point cloud
    const xyzrgbaData = createXYZRGBAPointCloud(xyz, colorValues, numPoints);

    // Return the modified point cloud message
    return {
        timestamp: {
            sec: api_message.timestamp.sec,
            nsec: api_message.timestamp.nsec,
        },
        frame_id: api_message.frame_id,
        pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        point_stride: XYZRGBA_STRIDE,
        fields: XYZRGBA_FIELDS,
        data: xyzrgbaData,
    };
}
