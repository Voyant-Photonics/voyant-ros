// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

import { Input } from "./types";
import { PointCloud, PackedElementField } from "@foxglove/schemas";

// Constants for point cloud field types
const FIELD_TYPE = {
  UINT8: 1,
  UINT16: 2,
  UINT32: 5,
  INT32: 6,
  FLOAT32: 7,
};

// SNR color configuration
const SNR_COLORS: [number, number, number][] = [
  [7, 107, 236], // Low SNR (blue)
  [255, 0, 127], // Mid SNR (magenta)
  [255, 204, 0], // High SNR (yellow)
];

// Define input and output topics
export const inputs = ["/voyant_points"];
export const output = "/snr_color_pointcloud";

// Global variables interface
interface GlobalVariables {
  min_snr_bound: number;
  max_snr_bound: number;
}

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
 * @param snrValues Array of SNR values
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

/**
 * Main script function to process point cloud data and color it based on SNR values
 * @param event Input event containing LiDAR point cloud data
 * @param globalVars Global variables for SNR bounds
 * @returns Modified point cloud with added color information
 */
export default function script(
  event: Input<"/voyant_points">,
  globalVars: GlobalVariables,
): PointCloud {
  const { data, point_step: originalStride } = event.message;
  const numPoints = data.length / originalStride;

  // Calculate new stride with RGBA fields
  const newStride = originalStride + 4; // Adding 4 bytes for RGBA
  const newDataSize = numPoints * newStride;

  // Extract SNR values from all points
  const snrValues = extractSnrValues(data, originalStride, numPoints);

  // Create SNR color map and convert SNR values to RGB colors
  const colorMap = createLinearColorMap(SNR_COLORS);
  const rgbColors = mapSnrToRgb(
    colorMap,
    snrValues,
    globalVars.min_snr_bound,
    globalVars.max_snr_bound,
  );

  // Create the new point cloud data with added color information
  const coloredPointCloud = createColoredPointCloud(
    data,
    rgbColors,
    originalStride,
    newStride,
    numPoints,
  );

  // Return the modified point cloud message
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
    fields: POINT_CLOUD_FIELDS,
    data: coloredPointCloud,
  };
}

/**
 * Extracts SNR values from point cloud data
 * @param data Original point cloud data
 * @param stride Original point stride
 * @param numPoints Number of points
 * @returns Array of SNR values
 */
function extractSnrValues(
  data: Uint8Array,
  stride: number,
  numPoints: number,
): number[] {
  const snrValues: number[] = [];
  const SNR_OFFSET = 20; // Offset to SNR value in each point

  for (let i = 0; i < numPoints; i++) {
    const pointOffset = i * stride;
    const snrBytes = data.slice(
      pointOffset + SNR_OFFSET,
      pointOffset + SNR_OFFSET + 4,
    );
    const snrValue = bytesToFloat32(snrBytes);
    snrValues.push(snrValue);
  }

  return snrValues;
}

/**
 * Creates a new point cloud with added color information
 * @param originalData Original point cloud data
 * @param rgbColors RGB colors to apply to each point
 * @param originalStride Original point stride
 * @param newStride New point stride (with RGBA fields)
 * @param numPoints Number of points
 * @returns New point cloud data with colors
 */
function createColoredPointCloud(
  originalData: Uint8Array,
  rgbColors: number[][],
  originalStride: number,
  newStride: number,
  numPoints: number,
): Uint8Array {
  const coloredData = new Uint8Array(numPoints * newStride);
  const RGBA_OFFSET = 48; // Offset to RGBA values in new point structure

  for (let i = 0; i < numPoints; i++) {
    // Copy original point data
    const srcOffset = i * originalStride;
    const destOffset = i * newStride;
    coloredData.set(
      originalData.subarray(srcOffset, srcOffset + originalStride),
      destOffset,
    );

    // Add RGB and alpha values
    const [r, g, b] = rgbColors[i];
    coloredData.set([r, g, b, 255], destOffset + RGBA_OFFSET);
  }

  return coloredData;
}
