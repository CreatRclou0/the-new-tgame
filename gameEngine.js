import { Intersection } from './intersection.js';
import { TrafficLightController } from './trafficLights.js';
import { CarManager } from './cars.js';
import { SensorSystem } from './sensors.js';
import { Statistics } from './statistics.js';
import { CONFIG } from './config.js';
// ...existing code...

export class GameEngine {
    constructor(canvas, ctx) {
        this.canvas = canvas;
        this.ctx = ctx;
        // Game components
        this.intersection = new Intersection(CONFIG.CANVAS_WIDTH / 2, CONFIG.CANVAS_HEIGHT / 2);
        this.trafficLights = new TrafficLightController();
        this.carManager = new CarManager(this.intersection);
        this.sensorSystem = new SensorSystem(this.intersection);
        this.statistics = new Statistics();
        // Game state
        this.mode = CONFIG.MODES.FIXED;
        this.settings = { ...CONFIG.DEFAULT_SETTINGS };
    // ...existing code...
    }

    initialize() {
    // ...existing code...
        this.intersection.setCarManager(this.carManager);
        this.trafficLights.initialize(this.mode, this.settings);
        this.carManager.initialize(this.settings);
        this.sensorSystem.initialize(this.settings.DETECTOR_DISTANCE);
        this.statistics.initialize();
        // Listen for car completion events
        this.carManager.onCarCompleted = (car) => {
            this.statistics.recordCarCompletion(car);
        };
        console.log('Game engine initialized');
    }

    update(deltaTime) {
        // Update traffic lights
        this.trafficLights.update(deltaTime, this.mode, this.settings);
        
        // Update cars
        this.carManager.update(deltaTime, this.trafficLights.getLightStates());
        
        // Update sensors and adaptive logic
        const sensorData = this.sensorSystem.update(this.carManager.getCars());
        
        // Only update adaptive logic if in adaptive mode
        if (this.mode === CONFIG.MODES.ADAPTIVE) {
            this.trafficLights.updateAdaptiveLogic(sensorData, deltaTime);
        }
        
        // Update statistics
        this.statistics.update(this.carManager.getCars(), deltaTime);
    }

    render() {
        // Clear canvas
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Render intersection
        this.intersection.render(this.ctx);
        
        // Render sensor detection zones (only in adaptive mode)
        if (this.mode === CONFIG.MODES.ADAPTIVE) {
            this.sensorSystem.render(this.ctx);
        }
        
        // Render cars
        this.carManager.render(this.ctx);
        
        // Render traffic lights
        this.trafficLights.render(this.ctx, this.intersection);
    }

    reset() {
        this.carManager.reset();
        this.trafficLights.reset();
        this.sensorSystem.reset();
        this.statistics.reset();
        console.log('Game reset');
    }

    // Settings update methods
    updateMode(mode) {
        this.mode = mode;
        this.trafficLights.setMode(mode);
        console.log(`Mode changed to: ${mode}`);
    }

    updateSetting(key, value) {
        this.settings[key] = value;
        
        // Apply setting changes to relevant systems
        switch (key) {
            case 'CAR_SPAWN_RATE':
            case 'CAR_SPEED':
            case 'TURN_RATE':
                this.carManager.updateSettings(this.settings);
                break;
            case 'DETECTOR_DISTANCE':
                this.sensorSystem.updateDetectorDistance(value);
                break;
            case 'GREEN_DURATION':
            case 'YELLOW_DURATION':
            case 'RED_DURATION':
            case 'YELLOW_DURATION':
            case 'MIN_GREEN_TIME':
                this.trafficLights.updateSettings(this.settings);
                break;
        }
    }

    updateSettings(settings) {
        this.settings = { ...this.settings, ...settings };
        this.trafficLights.updateSettings(this.settings);
        this.carManager.updateSettings(this.settings);
    }

    // Getters for UI
    getStatistics() {
        return this.statistics.getStats();
    }

    getLightStates() {
        return this.trafficLights.getLightStates();
    }

    getCurrentMode() {
        return this.mode;
    }

    getSettings() {
        return { ...this.settings };
    }

    // Add these properties to your CarManager class
    setLaneMapping(laneMapping, paths, getPathIndex) {
        this.laneMapping = laneMapping;
        this.paths = paths;
        this.getPathIndex = getPathIndex;
    }

    // Call this when spawning a car
    assignPathToCar(car) {
        // car should have dir, lane, move properties
        const pathIndex = this.getPathIndex(car.dir, car.lane, car.move);
        if (pathIndex !== null && this.paths[pathIndex]) {
            car.path = this.paths[pathIndex];
            car.pathIndex = pathIndex;
            car.pathProgress = 0; // index of next waypoint
        }
    }

    // Example path-following logic for each car (call in update loop)
    updateCarPathFollowing(car, deltaTime) {
        if (!car.path || car.pathProgress >= car.path.length) return;

        // Lookahead: next waypoint
        const target = car.path[car.pathProgress];
        const dx = target.x - car.x;
        const dy = target.y - car.y;
        const distance = Math.sqrt(dx * dx + dy * dy);

        // Move towards target
        const speed = car.speed * deltaTime;
        if (distance < speed) {
            car.x = target.x;
            car.y = target.y;
            car.pathProgress++;
        } else {
            car.x += (dx / distance) * speed;
            car.y += (dy / distance) * speed;
        }

        // Optional: Snap to lane center after turn
        if (car.pathProgress === car.path.length) {
            // Snap to center of exit lane if needed
        }
    }
}

