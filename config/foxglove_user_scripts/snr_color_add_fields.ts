// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

import { Input } from "./types";
import { PointCloud, PackedElementField } from "@foxglove/schemas";

// The input and output topics for your script
export const inputs = ["/point_cloud"];
export const output = "/snr_color_pointcloud";

type GlobalVariables = {
    min_snr_bound: number;
    max_snr_bound: number;
};

// SNR color configuration
const SNR_COLORS: [number, number, number][] = [
    [7, 107, 236], // Low SNR (blue)
    [255, 0, 127], // Mid SNR (magenta)
    [255, 204, 0], // High SNR (yellow)
];

// Pre-compute the color map
const colorMap = createLinearColorMap(SNR_COLORS);

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
 * Converts SNR values to RGB colors using the provided color map
 * @param colorMap The color map to use for conversion
 * @param snrValues Array of SNR values (in dB)
 * @param minSnr Minimum SNR value for normalization
 * @param maxSnr Maximum SNR value for normalization
 * @returns Array of RGB values corresponding to the input SNR values
 */
function mapSnrToRgb(
    colorMap: number[][],
    snrValues: number[],
    minSnr: number,
    maxSnr: number,
): number[][] {
    // Find actual min/max within bounds
    const validMin = Math.max(minSnr, Math.min(...snrValues));
    const validMax = Math.min(maxSnr, Math.max(...snrValues));
    const range = validMax - validMin;

    return snrValues.map((value) => {
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
 * Main script function to process point cloud data and color it based on SNR values
 * @param event Input event containing LiDAR point cloud data
 * @param globalVars Global variables for SNR bounds
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
        const SNR_OFFSET = 20;
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
            SNR_OFFSET,
        );
    } else if (isAPIPointCloud(event.message)) {
        // Process API point cloud message
        const SNR_OFFSET = 24;
        return processAPIPointCloud(event.message, globalVars, SNR_OFFSET);
    } else {
        throw new Error("Unknown point cloud message format");
    }
}

/**
 * Extracts SNR values from point cloud data and converts to dB
 * @param data Original point cloud data
 * @param stride Original point stride
 * @param numPoints Number of points
 * @param SNR_OFFSET Offset of the SNR field in bytes
 * @returns Array of SNR values in dB
 */
function extractSnrValues(
    data: Uint8Array,
    stride: number,
    numPoints: number,
    SNR_OFFSET: number,
): number[] {
    const snrValues: number[] = [];

    for (let i = 0; i < numPoints; i++) {
        const pointOffset = i * stride;
        const snrBytes = data.slice(
            pointOffset + SNR_OFFSET,
            pointOffset + SNR_OFFSET + 4,
        );
        const snrValue = bytesToFloat32(snrBytes);
        // Convert to dB scale
        const snrDb = 10 * Math.log10(snrValue);
        snrValues.push(snrDb);
    }

    return snrValues;
}

/**
 * Creates a new point cloud with colorized points
 * @param sourceData Original point cloud data
 * @param colors RGB color values for each point
 * @param oldStride Original point stride in bytes
 * @param newStride New point stride in bytes
 * @param numPoints Number of points in the cloud
 * @returns New Uint8Array with original data plus colors
 */
function createColorizedPointCloud(
    sourceData: Uint8Array,
    colors: number[][],
    oldStride: number,
    newStride: number,
    numPoints: number,
): Uint8Array {
    const newSize = numPoints * newStride;
    const newData = new Uint8Array(newSize);

    for (let i = 0; i < numPoints; i++) {
        const srcOffset = i * oldStride;
        const dstOffset = i * newStride;

        // Copy original point data
        newData.set(
            sourceData.subarray(srcOffset, srcOffset + oldStride),
            dstOffset,
        );

        // Add color data
        if (i < colors.length && colors[i]) {
            const [r, g, b] = colors[i].map((v) =>
                Math.round(Math.max(0, Math.min(255, v))),
            );
            newData.set([r, g, b, 255], dstOffset + oldStride);
        } else {
            // Fallback color if data is missing
            // TODO: Make sure this never get executed
            newData.set([128, 128, 128, 255], dstOffset + oldStride);
        }
    }

    return newData;
}

/**
 * Processes a ROS2 PointCloud2 message and adds color information based on SNR values
 * @param data Point cloud data as Uint8Array
 * @param originalStride Original point stride in bytes
 * @param ros_header ROS2 message header
 * @param globalVars Global variables for SNR bounds
 * @param SNR_OFFSET Offset of the SNR field in bytes
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
    SNR_OFFSET: number,
) {
    // Constants for point cloud field types
    const FIELD_TYPE = {
        UINT8: 1,
        UINT16: 2,
        UINT32: 5,
        INT32: 6,
        FLOAT32: 7,
    };

    // Define the point cloud fields including the added RGBA fields
    const POINT_CLOUD_FIELDS: PackedElementField[] = [
        { name: "x", offset: 0, type: FIELD_TYPE.FLOAT32 },
        { name: "y", offset: 4, type: FIELD_TYPE.FLOAT32 },
        { name: "z", offset: 8, type: FIELD_TYPE.FLOAT32 },
        { name: "v", offset: 16, type: FIELD_TYPE.FLOAT32 },
        { name: "snr", offset: 20, type: FIELD_TYPE.FLOAT32 },
        { name: "drop_reason", offset: 24, type: FIELD_TYPE.UINT16 },
        { name: "timestamp_nsecs", offset: 26, type: FIELD_TYPE.INT32 },
        { name: "point_idx", offset: 30, type: FIELD_TYPE.UINT32 },
        { name: "red", offset: 48, type: FIELD_TYPE.UINT8 },
        { name: "green", offset: 49, type: FIELD_TYPE.UINT8 },
        { name: "blue", offset: 50, type: FIELD_TYPE.UINT8 },
        { name: "alpha", offset: 51, type: FIELD_TYPE.UINT8 },
    ];

    const numPoints = data.length / originalStride;
    const newStride = originalStride + 4; // Adding 4 bytes for RGBA

    // Extract SNR values from all points
    const snrValues = extractSnrValues(
        data,
        originalStride,
        numPoints,
        SNR_OFFSET,
    );

    // Convert SNR values to RGB colors
    const rgbColors = mapSnrToRgb(
        colorMap,
        snrValues,
        globalVars.min_snr_bound,
        globalVars.max_snr_bound,
    );

    // Create the new point cloud data with added color information
    const coloredPointCloud = createColorizedPointCloud(
        data,
        rgbColors,
        originalStride,
        newStride,
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
        point_stride: newStride,
        fields: POINT_CLOUD_FIELDS,
        data: coloredPointCloud,
    };
}

/**
 * Processes an API PointCloud message and adds color information based on SNR values
 * @param api_message API PointCloud message
 * @param globalVars Global variables for SNR bounds
 * @param SNR_OFFSET Offset of the SNR field in bytes
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
    SNR_OFFSET: number,
) {
    // Get the original point cloud data as a Uint8Array
    const {
        data,
        point_stride: old_strid,
        fields: cloud_fields,
        timestamp: ts,
        frame_id: fid,
    } = api_message;

    const numPoints = Math.floor(data.length / old_strid);
    const new_strid = old_strid + 4; // Adding 4 bytes for RGBA

    // Extract SNR values from all points
    const snrValues = extractSnrValues(data, old_strid, numPoints, SNR_OFFSET);

    // Convert SNR values to RGB colors
    const rgbColors = mapSnrToRgb(
        colorMap,
        snrValues,
        globalVars.min_snr_bound,
        globalVars.max_snr_bound,
    );

    // Create the new point cloud data with added color information
    const coloredPointCloud = createColorizedPointCloud(
        data,
        rgbColors,
        old_strid,
        new_strid,
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
        point_stride: new_strid,
        fields: cloud_fields.concat(
            { name: "red", offset: 44, type: 1 },
            { name: "green", offset: 45, type: 1 },
            { name: "blue", offset: 46, type: 1 },
            { name: "alpha", offset: 47, type: 1 },
        ),
        data: coloredPointCloud,
    };
}
