import { Input, Message } from "./types";
import { PointCloud } from "@foxglove/schemas";

// The input and output topics for your script
export const inputs = ["/point_cloud"];
export const output = "/dummy_cloud";

function appendToUint8Array(arr: Uint8Array, data: Uint8Array) {
    const newArray = new Uint8Array(arr.length + data.length);
    newArray.set(arr); // copy old data
    newArray.set(data, arr.length); // copy new data after end of old data
    return newArray;
}

export default function script(event: Input<"/point_cloud">): PointCloud {
    // // Get the original point cloud data as a Uint8Array
    const data = event.message.data;
    const dopplerValues: number[] = [];

    // // Create a new Uint8Array to modify the RGB values
    const new_strid = 11 * 4 + 4 * 1; // 11 fields of 4 bytes each + 4 fields of 1 byte each (RGBA)
    const old_strid = 11 * 4;
    const new_size = ~~((data.length / old_strid) * new_strid); // reason: we want to add rgba field to the new pointcloud. The original pointcloud without the rgba field is going to be same in the new pointcloud, except the new one will have extra 4 uint8 fields of red,green, blue and alpha channels.
    const new_point_cloud_data = new Uint8Array(new_size);

    const numPoints = data.length / old_strid;
    // Iterate over the original point cloud data
    for (let i = 0; i < numPoints; i++) {
        // Calculate the offset for the current point in the original data (i * 44)
        const originalOffset = i * old_strid;

        // Copy the original 44 bytes of data into the new array (this includes the original 11 fields)
        new_point_cloud_data.set(
            data.subarray(originalOffset, originalOffset + old_strid),
            i * new_strid,
        );

        // Append RGBA values at the end of each point (indices 44-47)
        new_point_cloud_data[i * new_strid + 44] = 255; // R (Red)
        new_point_cloud_data[i * new_strid + 45] = 255; // G (Green)
        new_point_cloud_data[i * new_strid + 46] = 0; // B (Blue)
        new_point_cloud_data[i * new_strid + 47] = 255; // A (Alpha)
    }
    // Now, we return the modified point cloud message
    return {
        timestamp: {
            sec: event.message.timestamp.sec,
            nsec: event.message.timestamp.nsec,
        },
        frame_id: event.message.frame_id,
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
        fields: event.message.fields.concat(
            { name: "red", offset: 44, type: 1 },
            { name: "green", offset: 45, type: 1 },
            { name: "blue", offset: 46, type: 1 },
            { name: "alpha", offset: 47, type: 1 },
        ),
        data: new_point_cloud_data,
    };
}
