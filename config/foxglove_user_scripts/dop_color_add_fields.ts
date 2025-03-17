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

// Constants for input/output topics and field types
export const inputs = ["/voyant_points"];
export const output = "/dop_color_pointcloud";

// Type definitions
type GlobalVariables = {
    min_dop_bound: number;
    max_dop_bound: number;
};

type RGBColor = [number, number, number];

// Data structures for point cloud fields
const POINT_FIELDS: PackedElementField[] = [
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

// Color constants
const DOPPLER_COLORS: RGBColor[] = [
    [255.0, 0.0, 0.0], // Red for negative doppler (moving away)
    [49.0, 49.0, 49.0], // Gray for zero doppler
    [0.0, 100.0, 255.0], // Blue for positive doppler (moving toward)
];

// Pre-compute the color map
const dopColorMap = createLinearColorMap(DOPPLER_COLORS);

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
 * @param minDoppler Minimum doppler value for normalization
 * @param maxDoppler Maximum doppler value for normalization
 * @returns Array of RGB color values corresponding to input velocities
 */
function mapDopplerToRGB(
    colorMap: number[][],
    velocities: number[],
    minDoppler: number,
    maxDoppler: number,
): number[][] {
    // Ensure min is negative (moving away) and max is positive (moving toward)
    const minDop = -Math.abs(minDoppler);
    const maxDop = Math.abs(maxDoppler);
    const dopplerRange = maxDop - minDop;

    return velocities.map((velocity) => {
        // Normalize velocity to 0-255 range
        const normalizedValue = Math.max(
            0,
            Math.min(255, Math.round(((velocity - minDop) / dopplerRange) * 255)),
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
    event: Input<"/voyant_points">,
    globalVars: GlobalVariables,
): PointCloud {
    const { data } = event.message;
    const oldStride = event.message.point_step;
    const newStride = oldStride + 4; // Adding 4 bytes for RGBA
    const numPoints = Math.floor(data.length / oldStride);

    // Extract doppler values from point cloud
    const dopplerValues = extractDopplerValues(data, oldStride, numPoints);

    // Map doppler values to RGB colors
    const dopplerColors = mapDopplerToRGB(
        dopColorMap,
        dopplerValues,
        globalVars.min_dop_bound,
        globalVars.max_dop_bound,
    );

    // Create new point cloud with color data
    const newPointCloud = createColorizedPointCloud(
        data,
        dopplerColors,
        oldStride,
        newStride,
        numPoints,
    );

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
        fields: POINT_FIELDS,
        data: newPointCloud,
    };
}

/**
 * Extracts doppler velocity values from point cloud data
 * @param data Raw point cloud data
 * @param stride Point stride in bytes
 * @param numPoints Number of points in the cloud
 * @returns Array of doppler velocity values
 */
function extractDopplerValues(
    data: Uint8Array,
    stride: number,
    numPoints: number,
): number[] {
    const dopplerValues: number[] = [];
    const DOPPLER_OFFSET = 16; // Doppler field offset in bytes

    for (let i = 0; i < numPoints; i++) {
        const pointOffset = i * stride;
        const dopplerBytes = new Uint8Array(4);
        dopplerBytes.set(
            data.subarray(
                pointOffset + DOPPLER_OFFSET,
                pointOffset + DOPPLER_OFFSET + 4,
            ),
        );
        dopplerValues.push(uint8ArrayToFloat32(dopplerBytes));
    }

    return dopplerValues;
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
            newData.set([128, 128, 128, 255], dstOffset + oldStride);
        }
    }

    return newData;
}
