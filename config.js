export const CONFIG = {
    // Canvas settings
    CANVAS_WIDTH: 1200,
    CANVAS_HEIGHT: 1200,

    // Intersection settings
    INTERSECTION_SIZE: 120,
    ROAD_WIDTH: 60,
    LANE_WIDTH: 30,

    // Car settings
    CAR_WIDTH: 16,
    CAR_HEIGHT: 8,
    CAR_COLORS: [
        "#FF0000", // red
        "#00FF00", // green
        "#0000FF", // blue
        "#FFFF00", // yellow
        "#FFA500", // orange
        "#FFFFFF", // white
        "#000000", // black
        "#888888"  // gray
    ],

    // Directions
    DIRECTIONS: {
        NORTH: 'north',
        SOUTH: 'south',
        EAST: 'east',
        WEST: 'west'
    },

    // Turn types
    TURN_TYPES: {
        STRAIGHT: 'straight'
    },

    // Modes
    MODES: {
        FIXED: 'fixed',
        ADAPTIVE: 'adaptive'
    },

    // Light settings
    LIGHT_SIZE: 12,

    // Light states
    LIGHT_STATES: {
        RED: 'red',
        YELLOW: 'yellow',
        GREEN: 'green'
    },

    // Default settings
    DEFAULT_SETTINGS: {
        GREEN_DURATION: 100000,      // 100 seconds
        YELLOW_DURATION: 5000,       // 5 seconds
        RED_DURATION: 100000,        // 100 seconds
        CAR_SPAWN_RATE: 4,           // cars per 10 seconds
        CAR_SPEED: 25,               // pixels per second
        TURN_RATE: 0.25,             // 25% chance to turn
        DETECTOR_DISTANCE: 200,      // Increased range for adaptive mode
        MIN_GREEN_TIME: 5000         // 5 seconds minimum green
    },

    // Adaptive mode settings
    ADAPTIVE_SETTINGS: {
    DETECTOR_DISTANCE_RANGE: [100, 1000] // 100 to 1000 pixels
    }
};