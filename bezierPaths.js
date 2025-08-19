// Bézier curve turning system for traffic intersection
import { CONFIG } from './config.js';

export class BezierPathSystem {
    constructor() {
        this.cx = CONFIG.CANVAS_WIDTH / 2;  // 600
        this.cy = CONFIG.CANVAS_HEIGHT / 2; // 600
        this.halfSize = CONFIG.INTERSECTION_SIZE / 2; // 60
        this.laneOffset = CONFIG.LANE_WIDTH / 2; // 15
        this.r = this.halfSize - this.laneOffset; // 45
        this.k = 0.5522847498307936 * this.r; // ≈ 24.8528137424
        
        this.initializePaths();
    }

    initializePaths() {
        // Cubic Bézier control points for ALL TURNS
        this.turnPaths = {
            // SOUTH approach (moving up)
            "S->E_right": {  // bottom-right corner, exit Eastbound
                P0: [615, 660], P1: [615, 660 - this.k], P2: [660 - this.k, 615], P3: [660, 615]
            },
            "S->W_left": {   // bottom-left corner, exit Westbound
                P0: [615, 660], P1: [615, 660 - this.k], P2: [540, 585 + this.k], P3: [540, 585]
            },

            // NORTH approach (moving down)
            "N->W_right": {  // top-left corner, exit Westbound
                P0: [585, 540], P1: [585, 540 + this.k], P2: [540 + this.k, 585], P3: [540, 585]
            },
            "N->E_left": {   // top-right corner, exit Eastbound
                P0: [585, 540], P1: [585, 540 + this.k], P2: [660, 615 - this.k], P3: [660, 615]
            },

            // EAST approach (moving left)
            "E->N_right": {  // top-right corner, exit Northbound
                P0: [660, 585], P1: [660 - this.k, 585], P2: [615, 540 + this.k], P3: [615, 540]
            },
            "E->S_left": {   // bottom-right corner, exit Southbound
                P0: [660, 585], P1: [660 - this.k, 585], P2: [585 + this.k, 660], P3: [585, 660]
            },

            // WEST approach (moving right)
            "W->S_right": {  // bottom-left corner, exit Southbound
                P0: [540, 615], P1: [540 + this.k, 615], P2: [585, 615 + this.k], P3: [585, 660]
            },
            "W->N_left": {   // top-left corner, exit Northbound
                P0: [540, 615], P1: [540 + this.k, 615], P2: [615 - this.k, 540], P3: [615, 540]
            }
        };

        // Straight paths (degenerate Bézier with collinear controls)
        this.straightPaths = {
            "S->N": { P0: [615, 660], P1: [615, 630], P2: [615, 570], P3: [615, 540] },
            "N->S": { P0: [585, 540], P1: [585, 570], P2: [585, 630], P3: [585, 660] },
            "E->W": { P0: [660, 585], P1: [630, 585], P2: [570, 585], P3: [540, 585] },
            "W->E": { P0: [540, 615], P1: [570, 615], P2: [630, 615], P3: [660, 615] }
        };

        // Combine all paths
        this.allPaths = { ...this.turnPaths, ...this.straightPaths };
    }

    // Cubic Bézier evaluation: B(t) = (1−t)³P₀ + 3(1−t)²tP₁ + 3(1−t)t²P₂ + t³P₃
    evaluateBezier(path, t) {
        const { P0, P1, P2, P3 } = path;
        const t2 = t * t;
        const t3 = t2 * t;
        const mt = 1 - t;
        const mt2 = mt * mt;
        const mt3 = mt2 * mt;

        const x = mt3 * P0[0] + 3 * mt2 * t * P1[0] + 3 * mt * t2 * P2[0] + t3 * P3[0];
        const y = mt3 * P0[1] + 3 * mt2 * t * P1[1] + 3 * mt * t2 * P2[1] + t3 * P3[1];

        return { x, y };
    }

    // Tangent vector: dB/dt for heading calculation
    evaluateBezierTangent(path, t) {
        const { P0, P1, P2, P3 } = path;
        const t2 = t * t;
        const mt = 1 - t;
        const mt2 = mt * mt;

        const dx = -3 * mt2 * P0[0] + 3 * mt2 * P1[0] - 6 * mt * t * P1[0] + 6 * mt * t * P2[0] - 3 * t2 * P2[0] + 3 * t2 * P3[0];
        const dy = -3 * mt2 * P0[1] + 3 * mt2 * P1[1] - 6 * mt * t * P1[1] + 6 * mt * t * P2[1] - 3 * t2 * P2[1] + 3 * t2 * P3[1];

        return { dx, dy };
    }

    // Get path for a specific movement
    getPath(fromDirection, toDirection, turnType) {
        const key = this.getPathKey(fromDirection, toDirection, turnType);
        return this.allPaths[key] || null;
    }

    // Generate path key based on movement
    getPathKey(fromDirection, toDirection, turnType) {
        const dirMap = {
            [CONFIG.DIRECTIONS.NORTH]: 'N',
            [CONFIG.DIRECTIONS.SOUTH]: 'S',
            [CONFIG.DIRECTIONS.EAST]: 'E',
            [CONFIG.DIRECTIONS.WEST]: 'W'
        };

        const from = dirMap[fromDirection];
        const to = dirMap[toDirection];

        if (turnType === CONFIG.TURN_TYPES.STRAIGHT) {
            return `${from}->${to}`;
        } else {
            return `${from}->${to}_${turnType}`;
        }
    }

    // Calculate path length (approximate using segments)
    calculatePathLength(path, segments = 100) {
        if (!path || !path.P0 || !path.P3) return 0;
        
        let length = 0;
        let prevPoint = this.evaluateBezier(path, 0);

        for (let i = 1; i <= segments; i++) {
            const t = i / segments;
            const point = this.evaluateBezier(path, t);
            const dx = point.x - prevPoint.x;
            const dy = point.y - prevPoint.y;
            length += Math.sqrt(dx * dx + dy * dy);
            prevPoint = point;
        }

        return length;
    }

    // Debug: render all paths
    renderPaths(ctx, showTurns = true, showStraights = true) {
        ctx.save();
        ctx.lineWidth = 3;
        ctx.setLineDash([5, 5]);

        // Render turn paths in red
        if (showTurns) {
            ctx.strokeStyle = 'rgba(255, 0, 0, 0.8)';
            Object.entries(this.turnPaths).forEach(([key, path]) => {
                this.renderPath(ctx, path);
                this.renderPathLabel(ctx, path, key);
            });
        }

        // Render straight paths in blue
        if (showStraights) {
            ctx.strokeStyle = 'rgba(0, 0, 255, 0.8)';
            Object.entries(this.straightPaths).forEach(([key, path]) => {
                this.renderPath(ctx, path);
                this.renderPathLabel(ctx, path, key);
            });
        }

        ctx.restore();
    }

    // Render a single path
    renderPath(ctx, path) {
        const { P0, P1, P2, P3 } = path;
        
        ctx.beginPath();
        ctx.moveTo(P0[0], P0[1]);
        ctx.bezierCurveTo(P1[0], P1[1], P2[0], P2[1], P3[0], P3[1]);
        ctx.stroke();

        // Draw control points
        ctx.fillStyle = 'rgba(255, 255, 0, 0.8)';
        [P0, P1, P2, P3].forEach((point, index) => {
            ctx.beginPath();
            ctx.arc(point[0], point[1], index === 0 || index === 3 ? 6 : 3, 0, Math.PI * 2);
            ctx.fill();
        });
    }
    
    // Render path labels for debugging
    renderPathLabel(ctx, path, key) {
        const { P0, P3 } = path;
        const midX = (P0[0] + P3[0]) / 2;
        const midY = (P0[1] + P3[1]) / 2;
        
        ctx.save();
        ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
        ctx.font = '12px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(key, midX, midY);
        ctx.restore();
    }

    // Get all available movements from a direction
    getAvailableMovements(fromDirection) {
        const movements = [];
        const dirMap = {
            [CONFIG.DIRECTIONS.NORTH]: 'N',
            [CONFIG.DIRECTIONS.SOUTH]: 'S',
            [CONFIG.DIRECTIONS.EAST]: 'E',
            [CONFIG.DIRECTIONS.WEST]: 'W'
        };

        const from = dirMap[fromDirection];
        
        Object.keys(this.allPaths).forEach(key => {
            if (key.startsWith(`${from}->`)) {
                const parts = key.split('->');
                const toPart = parts[1];
                const [to, turn] = toPart.includes('_') ? toPart.split('_') : [toPart, 'straight'];
                
                const toDirection = Object.keys(dirMap).find(dir => dirMap[dir] === to);
                movements.push({
                    toDirection,
                    turnType: turn === 'straight' ? CONFIG.TURN_TYPES.STRAIGHT : turn,
                    pathKey: key
                });
            }
        });

        return movements;
    }
}