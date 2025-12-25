// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

import { Input } from "./types";
import { PointCloud, PackedElementField } from "@foxglove/schemas";

// The input and output topics for your script
export const inputs = ["/point_cloud"];
export const output = "/dop_color_pointcloud";

// Type definitions
type GlobalVariables = {
    min_dop_bound: number;
    max_dop_bound: number;
};
type RGBColor = [number, number, number];

// Color constants
const DOPPLER_COLORS: RGBColor[] = [
    [0.0, 100.0, 255.0], // Blue for negative doppler (moving towards)
    [49.0, 49.0, 49.0], // Gray for zero doppler
    [255.0, 0.0, 0.0], // Red for positive doppler (moving away)
];

// Pre-compute the color map
const dopColorMap = createLinearColorMap(DOPPLER_COLORS);

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
 * Creates a linear color gradient map between given color points
 * @param colors Array of RGB triplets defining the gradient points
 * @returns 2D array with 256 RGB color values
 */
function createLinearColorMap(colors: RGBColor[]): number[][] {
    // Extract colors from the input array
    const [colorLow, colorMid, colorHigh] = colors;
    const colorMap: number[][] = [];

    // Create 256 interpolated colors
    for (let i = 0; i < 256; i++) {
        const normVal = i / 255.0;
        let r, g, b;

        if (normVal < 0.5) {
            // Interpolate between low and mid
            const factor = normVal * 2.0;
            r = interpolate(colorLow[0], colorMid[0], factor);
            g = interpolate(colorLow[1], colorMid[1], factor);
            b = interpolate(colorLow[2], colorMid[2], factor);
        } else {
            // Interpolate between mid and high
            const factor = (normVal - 0.5) * 2.0;
            r = interpolate(colorMid[0], colorHigh[0], factor);
            g = interpolate(colorMid[1], colorHigh[1], factor);
            b = interpolate(colorMid[2], colorHigh[2], factor);
        }

        colorMap.push([r, g, b]);
    }

    return colorMap;
}

/**
 * Linear interpolation between two values
 * @param a Starting value
 * @param b Ending value
 * @param t Interpolation factor (0.0 to 1.0)
 * @returns Interpolated value
 */
function interpolate(a: number, b: number, t: number): number {
    return a + t * (b - a);
}

/**
 * Maps doppler velocity values to RGB colors using a predefined color map
 * @param colorMap The color mapping array (256 RGB values)
 * @param velocities Array of doppler velocity values
 * @param minDoppler Minimum doppler value for normalization (0 = auto)
 * @param maxDoppler Maximum doppler value for normalization (0 = auto)
 * @returns Array of RGB color values corresponding to input velocities
 */
function mapDopplerToRGB(
    colorMap: number[][],
    velocities: number[],
    minDoppler: number,
    maxDoppler: number,
): number[][] {
    // Auto-range if both bounds are set to 0
    if (minDoppler == 0.0 && maxDoppler == 0.0) {
        minDoppler = Math.min(...velocities);
        maxDoppler = Math.max(...velocities);
    }

    // Ensure min is negative (moving away) and max is positive (moving toward)
    const minDop = -Math.abs(minDoppler);
    const maxDop = Math.abs(maxDoppler);
    const dopplerRange = maxDop - minDop;

    return velocities.map((velocity) => {
        // Normalize velocity to 0-255 range
        const normalizedValue = Math.max(
            0,
            Math.min(
                255,
                Math.round(((velocity - minDop) / dopplerRange) * 255),
            ),
        );

        // Return the corresponding color from the map with safe fallback
        return colorMap[normalizedValue] || [0, 0, 0];
    });
}

/**
 * Converts a 4-byte Uint8Array to a Float32 value
 * @param bytes 4-byte Uint8Array containing the float data
 * @returns Parsed float32 value
 */
function uint8ArrayToFloat32(bytes: Uint8Array): number {
    return new DataView(bytes.buffer).getFloat32(0, true); // true for little-endian
}

/**
 * Main script function to process point cloud data
 * @param event Input point cloud message
 * @param globalVars User-configurable parameters
 * @returns Processed point cloud with color data
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
 * Extracts XYZ and doppler values from point cloud data
 * @param data Raw point cloud data
 * @param stride Point stride in bytes
 * @param numPoints Number of points in the cloud
 * @param XYZ_OFFSET Offset of the XYZ fields in bytes
 * @param DOP_OFFSET Offset of the doppler field in bytes
 * @returns Object containing xyz positions and doppler velocities
 */
function extractXYZAndDoppler(
    data: Uint8Array,
    stride: number,
    numPoints: number,
    XYZ_OFFSET: number,
    DOP_OFFSET: number,
): { xyz: Float32Array; dopplerValues: number[] } {
    const xyz = new Float32Array(numPoints * 3);
    const dopplerValues: number[] = [];

    for (let i = 0; i < numPoints; i++) {
        const pointOffset = i * stride;

        // Extract XYZ (12 bytes starting at XYZ_OFFSET)
        const xyzView = new DataView(
            data.buffer,
            data.byteOffset + pointOffset,
            stride,
        );
        xyz[i * 3] = xyzView.getFloat32(XYZ_OFFSET, true); // x
        xyz[i * 3 + 1] = xyzView.getFloat32(XYZ_OFFSET + 4, true); // y
        xyz[i * 3 + 2] = xyzView.getFloat32(XYZ_OFFSET + 8, true); // z

        // Extract doppler velocity
        const dopplerBytes = new Uint8Array(4);
        dopplerBytes.set(
            data.subarray(
                pointOffset + DOP_OFFSET,
                pointOffset + DOP_OFFSET + 4,
            ),
        );
        dopplerValues.push(uint8ArrayToFloat32(dopplerBytes));
    }

    return { xyz, dopplerValues };
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
        if (i < colors.length && colors[i]) {
            const [r, g, b] = colors[i].map((v) =>
                Math.round(Math.max(0, Math.min(255, v))),
            );
            newData[offset + 12] = r;
            newData[offset + 13] = g;
            newData[offset + 14] = b;
            newData[offset + 15] = 255; // alpha
        } else {
            // Fallback color if data is missing
            newData[offset + 12] = 128; // r
            newData[offset + 13] = 128; // g
            newData[offset + 14] = 128; // b
            newData[offset + 15] = 255; // alpha
        }
    }

    return newData;
}

/**
 * Processes a ROS2 PointCloud2 message and adds color information based on Doppler values
 * @param data Point cloud data as Uint8Array
 * @param originalStride Original point stride in bytes
 * @param ros_header ROS2 message header
 * @param globalVars Global variables for Doppler bounds
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
    const DOP_OFFSET = 16;
    const numPoints = data.length / originalStride;

    // Extract XYZ and Doppler values
    const { xyz, dopplerValues } = extractXYZAndDoppler(
        data,
        originalStride,
        numPoints,
        XYZ_OFFSET,
        DOP_OFFSET,
    );

    // Map Doppler values to RGB colors
    const colorMap = createLinearColorMap(DOPPLER_COLORS);
    const rgbColors = mapDopplerToRGB(
        colorMap,
        dopplerValues,
        globalVars.min_dop_bound,
        globalVars.max_dop_bound,
    );

    // Create XYZRGBA point cloud
    const xyzrgbaData = createXYZRGBAPointCloud(xyz, rgbColors, numPoints);

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
 * Processes an API PointCloud message and adds color information based on Doppler values
 * @param api_message API PointCloud message
 * @param globalVars Global variables for Doppler bounds
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
    const DOP_OFFSET = 20;

    // Get the original point cloud data as a Uint8Array
    const {
        data,
        point_stride: old_strid,
        timestamp: ts,
        frame_id: fid,
    } = api_message;

    const numPoints = Math.floor(data.length / old_strid);

    // Extract XYZ and Doppler values
    const { xyz, dopplerValues } = extractXYZAndDoppler(
        data,
        old_strid,
        numPoints,
        XYZ_OFFSET,
        DOP_OFFSET,
    );

    // Map the Doppler values to RGB colors
    const dopplerColors = mapDopplerToRGB(
        dopColorMap,
        dopplerValues,
        globalVars.min_dop_bound,
        globalVars.max_dop_bound,
    );

    // Create XYZRGBA point cloud
    const xyzrgbaData = createXYZRGBAPointCloud(xyz, dopplerColors, numPoints);

    // Return the modified point cloud message
    return {
        timestamp: {
            sec: ts.sec,
            nsec: ts.nsec,
        },
        frame_id: fid,
        pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
        point_stride: XYZRGBA_STRIDE,
        fields: XYZRGBA_FIELDS,
        data: xyzrgbaData,
    };
}
