// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

import { Input } from "./types";
import { PointCloud, PackedElementField } from "@foxglove/schemas";

// The input and output topics for your script
export const inputs = ["/point_cloud"];
export const output = "/ref_color_pointcloud";

type GlobalVariables = {
    min_ref_bound: number;
    max_ref_bound: number;
};

// Reflectance color configuration (same as SNR)
const REF_COLORS: [number, number, number][] = [
    [7, 107, 236], // Low reflectance (blue)
    [255, 0, 127], // Mid reflectance (magenta)
    [255, 204, 0], // High reflectance (yellow)
];

// Pre-compute the color map
const colorMap = createLinearColorMap(REF_COLORS);

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

// Fallback gray color for points without reflectance data
const FALLBACK_COLOR: number[] = [128, 128, 128];

/**
 * Creates a linear color mapping between three colors
 * @param colors Array of three [r,g,b] colors representing low, mid, and high values
 * @returns A 256-entry color map with interpolated values
 */
function createLinearColorMap(colors: [number, number, number][]): number[][] {
    if (colors.length !== 3) {
        throw new Error("Color map requires exactly 3 colors (low, mid, high)");
    }

    const [colorLow, colorMid, colorHigh] = colors;
    const colorMap: number[][] = [];

    // Create 256 interpolated colors
    for (let i = 0; i < 256; i++) {
        const normVal = i / 255;
        let r, g, b;

        if (normVal < 0.5) {
            // Interpolate between low and mid colors
            const factor = normVal * 2;
            r = colorLow[0] + factor * (colorMid[0] - colorLow[0]);
            g = colorLow[1] + factor * (colorMid[1] - colorLow[1]);
            b = colorLow[2] + factor * (colorMid[2] - colorLow[2]);
        } else {
            // Interpolate between mid and high colors
            const factor = (normVal - 0.5) * 2;
            r = colorMid[0] + factor * (colorHigh[0] - colorMid[0]);
            g = colorMid[1] + factor * (colorHigh[1] - colorMid[1]);
            b = colorMid[2] + factor * (colorHigh[2] - colorMid[2]);
        }

        colorMap.push([Math.round(r), Math.round(g), Math.round(b)]);
    }

    return colorMap;
}

/**
 * Converts reflectance values to RGB colors using the provided color map
 * @param colorMap The color map to use for conversion
 * @param refValues Array of reflectance values (in dB)
 * @param minRef Minimum reflectance value for normalization (0 = auto)
 * @param maxRef Maximum reflectance value for normalization (0 = auto)
 * @returns Array of RGB values corresponding to the input reflectance values
 */
function mapRefToRgb(
    colorMap: number[][],
    refValues: number[],
    minRef: number,
    maxRef: number,
): number[][] {
    // Auto-range if both bounds are set to 0
    if (minRef == 0.0 && maxRef == 0.0) {
        minRef = Math.min(...refValues);
        maxRef = Math.max(...refValues);
    }

    // Find actual min/max within bounds
    const validMin = Math.max(minRef, Math.min(...refValues));
    const validMax = Math.min(maxRef, Math.max(...refValues));
    const range = validMax - validMin;

    return refValues.map((value) => {
        // Clip to min/max bounds
        const clippedValue = Math.min(Math.max(value, validMin), validMax);

        // Normalize to 0-255 range
        const normalizedValue =
            range === 0
                ? 128 // Default to middle value if range is zero
                : ((clippedValue - validMin) / range) * 255;

        // Get color map index (ensuring it's within bounds)
        const index = Math.min(Math.max(Math.round(normalizedValue), 0), 255);

        return colorMap[index];
    });
}

/**
 * Converts a 4-byte Uint8Array to a Float32 value
 * @param bytes 4-byte Uint8Array
 * @returns Float32 value
 */
function bytesToFloat32(bytes: Uint8Array): number {
    const dataView = new DataView(bytes.buffer, bytes.byteOffset, 4);
    return dataView.getFloat32(0, true); // true for little-endian
}

/**
 * Main script function to process point cloud data and color it based on reflectance values
 * @param event Input event containing LiDAR point cloud data
 * @param globalVars Global variables for reflectance bounds
 * @returns Modified point cloud with added color information
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
 * Extracts XYZ and reflectance values from point cloud data
 * @param data Raw point cloud data
 * @param stride Point stride in bytes
 * @param numPoints Number of points in the cloud
 * @param XYZ_OFFSET Offset of the XYZ fields in bytes
 * @param REF_OFFSET Offset of the reflectance field in bytes
 * @returns Object containing xyz positions and reflectance values
 */
function extractXYZAndReflectance(
    data: Uint8Array,
    stride: number,
    numPoints: number,
    XYZ_OFFSET: number,
    REF_OFFSET: number,
): { xyz: Float32Array; refValues: number[] } {
    const xyz = new Float32Array(numPoints * 3);
    const refValues: number[] = [];

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

        // Extract reflectance and convert to dB
        const refBytes = data.slice(
            pointOffset + REF_OFFSET,
            pointOffset + REF_OFFSET + 4,
        );
        const refValue = bytesToFloat32(refBytes);
        // Convert to dB scale
        const refDb = 10 * Math.log10(refValue);
        refValues.push(refDb);
    }

    return { xyz, refValues };
}

/**
 * Extracts only XYZ values from point cloud data (for non-reflectance formats)
 * @param data Raw point cloud data
 * @param stride Point stride in bytes
 * @param numPoints Number of points in the cloud
 * @param XYZ_OFFSET Offset of the XYZ fields in bytes
 * @returns Float32Array of XYZ positions
 */
function extractXYZOnly(
    data: Uint8Array,
    stride: number,
    numPoints: number,
    XYZ_OFFSET: number,
): Float32Array {
    const xyz = new Float32Array(numPoints * 3);

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
    }

    return xyz;
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
 * Processes a ROS2 PointCloud2 message and adds color information based on reflectance values
 * @param data Point cloud data as Uint8Array
 * @param originalStride Original point stride in bytes
 * @param ros_header ROS2 message header
 * @param globalVars Global variables for reflectance bounds
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
    const REF_OFFSET = 34;
    const VOYANT_POINT_STRIDE = 48; // Standard VoyantPoint without reflectance
    const numPoints = data.length / originalStride;

    // Check if this is the extended format with reflectance
    // Only VoyantPointMdlExtended (stride > 48) has reflectance data
    const hasReflectance = originalStride > VOYANT_POINT_STRIDE;

    let xyz: Float32Array;
    let rgbColors: number[][];

    if (hasReflectance) {
        // Extract XYZ and reflectance values
        const result = extractXYZAndReflectance(
            data,
            originalStride,
            numPoints,
            XYZ_OFFSET,
            REF_OFFSET,
        );
        xyz = result.xyz;

        // Convert reflectance values to RGB colors
        rgbColors = mapRefToRgb(
            colorMap,
            result.refValues,
            globalVars.min_ref_bound,
            globalVars.max_ref_bound,
        );
    } else {
        // No reflectance data - extract only XYZ and use fallback gray for all points
        xyz = extractXYZOnly(data, originalStride, numPoints, XYZ_OFFSET);
        rgbColors = Array(numPoints).fill(FALLBACK_COLOR);
    }

    // Create the new point cloud data with added color information
    const coloredPointCloud = createXYZRGBAPointCloud(
        xyz,
        rgbColors,
        numPoints,
    );

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
        data: coloredPointCloud,
    };
}

/**
 * Processes an API PointCloud message and adds color information based on reflectance values
 * API messages always have reflectance data available
 * @param api_message API PointCloud message
 * @param globalVars Global variables for reflectance bounds
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
    const REF_OFFSET = 28;

    // Get the original point cloud data as a Uint8Array
    const {
        data,
        point_stride: old_strid,
        timestamp: ts,
        frame_id: fid,
    } = api_message;

    const numPoints = Math.floor(data.length / old_strid);

    // Extract XYZ and reflectance values (always available in API messages)
    const { xyz, refValues } = extractXYZAndReflectance(
        data,
        old_strid,
        numPoints,
        XYZ_OFFSET,
        REF_OFFSET,
    );

    // Convert reflectance values to RGB colors
    const rgbColors = mapRefToRgb(
        colorMap,
        refValues,
        globalVars.min_ref_bound,
        globalVars.max_ref_bound,
    );

    // Create the new point cloud data with added color information
    const coloredPointCloud = createXYZRGBAPointCloud(
        xyz,
        rgbColors,
        numPoints,
    );

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
        data: coloredPointCloud,
    };
}
