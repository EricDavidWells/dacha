

// How far past the halfway point (45 degrees) between two 90 degree steps the
// camera must roll before we switch to the next step. Without this, a camera
// mounted near a boundary (e.g. sideways) flickers between two orientations
// due to accelerometer noise.
const ROTATION_HYSTERESIS_DEGREES = 10;

// If the in-plane component of gravity is smaller than this fraction of the
// total, the camera is pointing nearly straight up/down so the roll angle is
// mostly noise and we keep the last rotation.
const MIN_ROLL_GRAVITY_FRACTION = 0.2;

// Last displayed rotation per camera (keyed by the 'key' argument).
const LAST_ROTATIONS: { [key: string]: number } = {};

// Wraps an angle in degrees to the range [-180, 180).
function wrap_degrees(a: number): number {
    return ((((a + 180) % 360) + 360) % 360) - 180;
}

// Determines how a camera's frames should be displayed based on its accelerometer.
//
// 'rotation' is one of 0/90/180/270 and is the clockwise rotation to apply
// after mirroring the image horizontally (we display images as seen by the
// user looking at the camera from in front of it). See orientation_transform().
//
// 'key' should uniquely identify the camera so that hysteresis state is
// tracked separately per camera.
export function camera_orientation(camera_status, key: string = '') {

    let accel = camera_status.accelerometer.value;

    let x = accel.values[0];
    let y = accel.values[1];
    let z = accel.values[2];

    // Accelerometer reads 'up' in image coordinates. 0 degrees is when 'up'
    // points towards the top of the image (y < 0).
    let in_plane = Math.sqrt(x * x + y * y);
    let roll = (180 / Math.PI) * Math.atan2(x, -y);

    let last_rotation = LAST_ROTATIONS[key];
    let rotation = last_rotation;

    if (last_rotation === undefined) {
        rotation = (Math.round(roll / 90) * 90 + 360) % 360;
    } else if (in_plane >= MIN_ROLL_GRAVITY_FRACTION * Math.sqrt(x * x + y * y + z * z)) {
        // Schmitt trigger: only leave the current step once we are past the
        // midpoint by more than the hysteresis margin.
        if (Math.abs(wrap_degrees(roll - last_rotation)) > 45 + ROTATION_HYSTERESIS_DEGREES) {
            rotation = (Math.round(roll / 90) * 90 + 360) % 360;
        }
    }

    LAST_ROTATIONS[key] = rotation;

    let horizon_angle = Math.round((180 / Math.PI) * Math.atan2(z, in_plane));

    return {
        rotation,
        horizon_angle
    };
}

// Gets the 2d affine transform which maps from frame pixel coordinates to
// display coordinates for a given 'rotation' from camera_orientation().
//
// Returns the SVG style matrix(a, b, c, d, e, f) components where:
//   display_x = a * x + c * y + e
//   display_y = b * x + d * y + f
// and the size of the display area (width/height are swapped for 90/270).
export function orientation_transform(rotation: number, frame_width: number, frame_height: number) {
    let w = frame_width;
    let h = frame_height;

    switch (rotation) {
        case 90:
            return { matrix: [0, -1, -1, 0, h, w], width: h, height: w };
        case 180:
            return { matrix: [1, 0, 0, -1, 0, h], width: w, height: h };
        case 270:
            return { matrix: [0, 1, 1, 0, 0, 0], width: h, height: w };
        default:
            return { matrix: [-1, 0, 0, 1, w, 0], width: w, height: h };
    }
}

export function apply_orientation_transform(matrix: number[], x: number, y: number): [number, number] {
    let [a, b, c, d, e, f] = matrix;
    return [a * x + c * y + e, b * x + d * y + f];
}
