/*
 * LIDAR Visualization using Webots Display Node
 * Displays LIDAR data on the robot's display device
 * Robot stays centered - map moves around it
 * With temporal filtering for both obstacles and free cells
 */

#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/supervisor.h>
#include <webots/display.h>
#include <webots/motor.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

// Configuration
#define TIME_STEP 32
#define GRID_SIZE 500          // 200x200 grid cells
#define GRID_RESOLUTION 0.02   // cm per cell
#define DISPLAY_WIDTH 500      // Display width in pixels
#define DISPLAY_HEIGHT 500     // Display height in pixels

// Grid cell types
#define CELL_UNKNOWN 0
#define CELL_FREE 1
#define CELL_OBSTACLE 2
#define CELL_ROBOT 3

// Temporal filtering thresholds
#define OBSTACLE_THRESHOLD 5   // Number of hits to mark as obstacle
#define FREE_THRESHOLD 5       // Number of passes to mark as free
#define COUNTER_DECAY 1        // Reduce counter each cycle

#define NEARBY_RADIUS_INNER 5
#define NEARBY_RADIUS_OUTER 15

#define MAX_QUEUE 1000
#define MIN_BLOB_SIZE 10

// Cost Map
#define OBSTACLE_COST 100
#define FREE_COST 1
#define UNKNOWN_COST 50
#define INFLATION_RADIUS 8

// Frontiers
#define MAX_FRONTIERS 100
#define MIN_FRONTIER_SIZE 20      // Minimum size for a valid frontier cluster
#define MIN_FRONTIER_CLEARANCE 8  // Minimum clearance from obstacles for frontier centers
#define FRONTIER_SAFETY_CHECK_RADIUS 15  // Radius to check for obstacle proximity

// Colors (0xRRGGBB format for Webots)
#define COLOR_UNKNOWN 0x404040   // Dark gray
#define COLOR_FREE 0xC8C8C8      // Light gray
#define COLOR_OBSTACLE 0x000000   // Black
#define COLOR_ROBOT 0x0064FF      // Blue
#define COLOR_BACKGROUND 0x303030 // Background

// Global variables
static unsigned int grid[GRID_SIZE][GRID_SIZE];
static unsigned int obstacle_counter[GRID_SIZE][GRID_SIZE];
static unsigned int free_counter[GRID_SIZE][GRID_SIZE];  // New: counter for free cells
static double cost_map[GRID_SIZE][GRID_SIZE];
static WbDeviceTag display;
static WbDeviceTag lidar;
static WbDeviceTag motors[4];
static WbNodeRef robot_node;


// ============ Initialize ===============
void init_grid() {
    memset(grid, CELL_UNKNOWN, sizeof(grid));
    memset(obstacle_counter, 0, sizeof(obstacle_counter));
    memset(free_counter, 0, sizeof(free_counter));  // Initialize free counters
}

void init_motors() {
    motors[0] = wb_robot_get_device("fl_wheel_joint");
    motors[1] = wb_robot_get_device("rl_wheel_joint");
    motors[2] = wb_robot_get_device("fr_wheel_joint");
    motors[3] = wb_robot_get_device("rr_wheel_joint");
    
    // Set motors to velocity control mode
    for (int i = 0; i < 4; i++) {
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 0.0);
    }
}



// =========== UTILITIES =============

// Convert world to grid coordinates
void world_to_grid(double wx, double wy, int* gx, int* gy) {
    *gx = (int)((wx / GRID_RESOLUTION) + GRID_SIZE / 2);
    *gy = (int)((wy / GRID_RESOLUTION) + GRID_SIZE / 2);
}

// Check if cell is valid
int is_valid_cell(int x, int y) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

// Clamp helper
double clamp(double val, double min, double max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// Helper: clamp a value between min and max             FIX   !!!!!!!@#!!@!@^&@%!
static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}



// ============== PID Controllers ==================

double pid_rotate(double angle_error) {
    static double Kp_rotate = 20.0; 
    static double Ki_rotate = 0.1;
    static double Kd_rotate = 0.01;
    
    static double prev_error = 0.0;
    static double integral = 0.0;
    
    // Anti-windup: limit integral accumulation
    double integral_limit = 2.0;
    integral += angle_error * (TIME_STEP / 1000.0);  // Scale by timestep
    integral = clamp(integral, -integral_limit, integral_limit);
    
    double derivative = (angle_error - prev_error) / (TIME_STEP / 1000.0);
    prev_error = angle_error;
    
    double omega = Kp_rotate * angle_error + Ki_rotate * integral + Kd_rotate * derivative;
    
    return omega;
}

double pid_translate(double distance_error) {
    static double Kp_translate = 5.0;
    static double Ki_translate = 0;
    static double Kd_translate = 0.5;
    
    static double prev_error = 0.0;
    static double integral = 0.0;
    
    // Anti-windup
    double integral_limit = 1.0;
    integral += distance_error * (TIME_STEP / 1000.0);
    integral = clamp(integral, -integral_limit, integral_limit);
    
    double derivative = (distance_error - prev_error) / (TIME_STEP / 1000.0);
    prev_error = distance_error;
    
    double speed = Kp_translate * distance_error + Ki_translate * integral + Kd_translate * derivative;
    
    return clamp(speed, 0, 5.0);  // Only forward motion
}

// Normalize angle to [-PI, PI]
double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}



// =============== ROBOT DRIVE MECHANICS ===============

// Navigate a differential drive robot to a target point.

bool diff_drive(int x_goal,   // in Grid Coord
                    int y_goal,  // in Grid Coord
                    double distance_threshold,
                    double angle_threshold,
                    double b,
                    double r) {
                    
    // Get robot pose
    const double* position = wb_supervisor_node_get_position(robot_node);
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    // Calculate distance and direction to goal
    double dx = x_goal - robot_gx;
    double dy = y_goal - robot_gy;
    double distance = sqrt(dx * dx + dy * dy);
    
    // Compute the angle to the goal
    double angle_to_goal = atan2(dy, dx);
    
    // Normalize angle difference between -π to π
    double angle_diff = normalize_angle( angle_to_goal - robot_theta); 
    
    // Stop condition - goal reached
    if (distance < distance_threshold) {
        for (int i = 0; i < 4; i++) {
            wb_motor_set_velocity(motors[i], 0.0);
        }
        return true;
    }
    
    double omega, speed;
    
    // First rotate until aligned
    if (fabs(angle_diff) > angle_threshold) {
        omega = pid_rotate(angle_diff);
        //speed = pid_translate(distance);
        speed = 0.0;
    } else {
        omega = pid_rotate(angle_diff);
        speed = pid_translate(distance);
    }
    
    // Differential drive kinematics
    double omega_l = (speed - omega * b / 2.0) / r;
    double omega_r = (speed + omega * b / 2.0) / r;
    
    // Apply velocities to motors
    wb_motor_set_velocity(motors[0], omega_l);  // fl
    wb_motor_set_velocity(motors[1], omega_l);  // rl
    wb_motor_set_velocity(motors[2], omega_r);  // fr
    wb_motor_set_velocity(motors[3], omega_r);  // rr
    
    return false;
}







// Bresenham's line algorithm with temporal filtering
void draw_line_on_grid(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        if (is_valid_cell(x0, y0)) {
            // Mark free space along ray (except endpoint)
            if (x0 != x1 || y0 != y1) {
                // Increment free counter for cells along the ray
                if (grid[y0][x0] != CELL_OBSTACLE) {
                    free_counter[y0][x0]++;
                    
                    // Only mark as free after consistent observations
                    if (free_counter[y0][x0] >= FREE_THRESHOLD) {
                        grid[y0][x0] = CELL_FREE;
                    }
                    
                    // Reset obstacle counter when we see free space
                    obstacle_counter[y0][x0] = 0;
                }
            } else {
                // End point: increment obstacle counter
                obstacle_counter[y0][x0]++;
                
                // Only mark as obstacle after consistent observations
                if (obstacle_counter[y0][x0] >= OBSTACLE_THRESHOLD) {
                    grid[y0][x0] = CELL_OBSTACLE;
                    free_counter[y0][x0] = 0;  // Reset free counter
                }
            }
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

void clear_nearby_obstacles(int inner_radius_cells, int outer_radius_cells) {
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);

    for (int dy = -outer_radius_cells; dy <= outer_radius_cells; dy++) {
        for (int dx = -outer_radius_cells; dx <= outer_radius_cells; dx++) {
            int gx = robot_gx + dx;
            int gy = robot_gy + dy;
            if (!is_valid_cell(gx, gy)) continue;

            // Compute squared distance from the robot
            int dist_sq = dx * dx + dy * dy;

            // Keep cells only between inner and outer circular radii
            if (dist_sq > outer_radius_cells * outer_radius_cells) continue;
            if (dist_sq < inner_radius_cells * inner_radius_cells) continue;

            if (grid[gy][gx] == CELL_OBSTACLE) {
                grid[gy][gx] = CELL_UNKNOWN;
                //obstacle_counter[gy][gx] = 0;
                //free_counter[gy][gx] = 0;
            }
        }
    }
}

typedef struct {
    int x;
    int y;
} Point;

void filter_connected_components(int min_size) {
    bool visited[GRID_SIZE][GRID_SIZE] = {false};

    // 8-connectivity directions
    int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};

    Point queue[MAX_QUEUE];

    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == CELL_OBSTACLE && !visited[y][x]) {
                // Flood fill (BFS)
                int head = 0, tail = 0, count = 0;
                Point component[MAX_QUEUE];

                queue[tail++] = (Point){x, y};
                visited[y][x] = true;

                while (head < tail) {
                    Point p = queue[head++];
                    component[count++] = p;

                    for (int i = 0; i < 8; i++) {
                        int nx = p.x + dx[i];
                        int ny = p.y + dy[i];

                        if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
                            continue;

                        if (!visited[ny][nx] && grid[ny][nx] == CELL_OBSTACLE) {
                            queue[tail++] = (Point){nx, ny};
                            visited[ny][nx] = true;
                        }
                    }
                }

                // Remove small clusters
                if (count < min_size) {
                    for (int i = 0; i < count; i++) {
                        grid[component[i].y][component[i].x] = CELL_FREE;
                        obstacle_counter[component[i].y][component[i].x] = 0;
                    }
                }
            }
        }
    }
}

// Process LIDAR data and update grid
void process_lidar() {
    // Get robot pose
    const double* position = wb_supervisor_node_get_position(robot_node);
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    // Get LIDAR data
    const float* ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    //double max_range = wb_lidar_get_max_range(lidar);
    double max_range = 2;
    // Process LIDAR rays
    for (int i = 0; i < resolution; i += 2) { // Process every other ray for speed
        double range = ranges[i];
        
        if (range < 0.05 || range > max_range * 0.95) {
            continue;
        }
        
        // Calculate ray angle
        double ray_angle = fov/2 - (i * fov / resolution);
        double world_angle = robot_theta + ray_angle;
        
        // Calculate end point
        double end_x = robot_x + range * cos(world_angle);
        double end_y = robot_y + range * sin(world_angle);
        
        int end_gx, end_gy;
        world_to_grid(end_x, end_y, &end_gx, &end_gy);
        
        // Draw ray on grid
        draw_line_on_grid(robot_gx, robot_gy, end_gx, end_gy);
    }
    
    // Mark robot position
    if (is_valid_cell(robot_gx, robot_gy)) {
        grid[robot_gy][robot_gx] = CELL_ROBOT;
    }
}

// Decay counters for cells that haven't been permanently established
void decay_counters() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            // Only decay obstacle counter if cell is NOT a confirmed obstacle
            if (grid[y][x] != CELL_OBSTACLE && obstacle_counter[y][x] > 0) {
                obstacle_counter[y][x] -= COUNTER_DECAY;
                
                // If counter reaches zero, reset to unknown
                if (obstacle_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
            
            // Only decay free counter if cell is NOT a confirmed free cell
            if (grid[y][x] != CELL_FREE && free_counter[y][x] > 0) {
                free_counter[y][x] -= COUNTER_DECAY;
                
                // If counter reaches zero, reset to unknown
                if (free_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
        }
    }
}

// Structure to hold frontier information
typedef struct {
    int x;
    int y;
    int size;
    double score;  // Score based on size and distance
} FrontierCentroid;

// Fast frontier detection using edge detection
static unsigned char frontier_edge_map[GRID_SIZE][GRID_SIZE];

// Quick check if cell is at boundary between free and unknown
static inline bool is_frontier_edge(int x, int y) {
    // Must be a free cell
    if (grid[y][x] != CELL_FREE) return false;
    
    // Quick 4-connectivity check for unknown neighbors
    if ((y > 0 && grid[y-1][x] == CELL_UNKNOWN) ||
        (y < GRID_SIZE-1 && grid[y+1][x] == CELL_UNKNOWN) ||
        (x > 0 && grid[y][x-1] == CELL_UNKNOWN) ||
        (x < GRID_SIZE-1 && grid[y][x+1] == CELL_UNKNOWN)) {
        return true;
    }
    return false;
}

// Build frontier edge map - much faster than checking every cell repeatedly
void build_frontier_edge_map() {
    memset(frontier_edge_map, 0, sizeof(frontier_edge_map));
    
    // Only scan the area around the robot for efficiency
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    // Scan radius around robot
    int scan_radius = 150;  // Adjust based on your needs
    int min_x = fmax(0, robot_gx - scan_radius);
    int max_x = fmin(GRID_SIZE - 1, robot_gx + scan_radius);
    int min_y = fmax(0, robot_gy - scan_radius);
    int max_y = fmin(GRID_SIZE - 1, robot_gy + scan_radius);
    
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            if (is_frontier_edge(x, y)) {
                frontier_edge_map[y][x] = 1;
            }
        }
    }
}

// Check if a frontier centroid has safe clearance from obstacles
bool has_safe_clearance(int cx, int cy, int min_clearance) {
    // Check in a square region around the centroid
    for (int dy = -min_clearance; dy <= min_clearance; dy++) {
        for (int dx = -min_clearance; dx <= min_clearance; dx++) {
            int check_x = cx + dx;
            int check_y = cy + dy;
            
            if (!is_valid_cell(check_x, check_y)) continue;
            
            if (grid[check_y][check_x] == CELL_OBSTACLE) {
                // Calculate actual distance
                double dist = sqrt(dx * dx + dy * dy);
                if (dist <= min_clearance) {
                    return false;  // Too close to obstacle
                }
            }
        }
    }
    return true;  // Safe clearance from all obstacles
}

// Efficient frontier clustering using region growing
int find_frontier_centroids(FrontierCentroid* centroids, int max_frontiers) {
    // First build the edge map
    build_frontier_edge_map();
    
    // Get robot position for scoring
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    // Use a simple visited array
    static bool visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));
    
    int num_frontiers = 0;
    
    // Scan for frontier clusters
    int scan_radius = 150;
    int min_x = fmax(0, robot_gx - scan_radius);
    int max_x = fmin(GRID_SIZE - 1, robot_gx + scan_radius);
    int min_y = fmax(0, robot_gy - scan_radius);
    int max_y = fmin(GRID_SIZE - 1, robot_gy + scan_radius);
    
    // Simple queue for BFS
    static Point queue[1000];  // Smaller queue for individual frontiers
    
    for (int y = min_y; y <= max_y && num_frontiers < max_frontiers; y++) {
        for (int x = min_x; x <= max_x && num_frontiers < max_frontiers; x++) {
            if (frontier_edge_map[y][x] && !visited[y][x]) {
                // Start a new frontier cluster
                int head = 0, tail = 0;
                int sum_x = 0, sum_y = 0, count = 0;
                
                queue[tail++] = (Point){x, y};
                visited[y][x] = true;
                
                // Region growing with 8-connectivity
                while (head < tail && tail < 1000) {
                    Point p = queue[head++];
                    sum_x += p.x;
                    sum_y += p.y;
                    count++;
                    
                    // Check 8 neighbors
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            
                            int nx = p.x + dx;
                            int ny = p.y + dy;
                            
                            if (nx >= min_x && nx <= max_x && 
                                ny >= min_y && ny <= max_y &&
                                !visited[ny][nx] && 
                                frontier_edge_map[ny][nx]) {
                                
                                queue[tail++] = (Point){nx, ny};
                                visited[ny][nx] = true;
                                
                                if (tail >= 1000) break;  // Prevent overflow
                            }
                        }
                    }
                }
                
                // Only keep significant frontiers
                if (count >= MIN_FRONTIER_SIZE) {
                    int cx = sum_x / count;
                    int cy = sum_y / count;
                    
                    // Check if centroid has safe clearance from obstacles
                    if (has_safe_clearance(cx, cy, MIN_FRONTIER_CLEARANCE)) {
                        centroids[num_frontiers].x = cx;
                        centroids[num_frontiers].y = cy;
                        centroids[num_frontiers].size = count;
                        
                        // Calculate score based on size and distance
                        double dist = sqrt((cx - robot_gx) * (cx - robot_gx) + 
                                         (cy - robot_gy) * (cy - robot_gy));
                        
                        // Bonus for larger frontiers, penalty for distance
                        centroids[num_frontiers].score = (count * 2.0) / (1.0 + dist * 0.02);
                        
                        num_frontiers++;
                    }
                }
            }
        }
    }
    
    // Sort frontiers by score (bubble sort for simplicity)
    for (int i = 0; i < num_frontiers - 1; i++) {
        for (int j = 0; j < num_frontiers - i - 1; j++) {
            if (centroids[j].score < centroids[j + 1].score) {
                FrontierCentroid temp = centroids[j];
                centroids[j] = centroids[j + 1];
                centroids[j + 1] = temp;
            }
        }
    }
    
    return num_frontiers;
}

// Get closest frontier to a given position
FrontierCentroid* get_best_frontier(FrontierCentroid* frontiers, int num_frontiers) {
    // Return the highest scoring frontier (already sorted)
    return (num_frontiers > 0) ? &frontiers[0] : NULL;
}

// Generate cost map from occupancy grid
void generate_cost_map() {
    // Step 1: Initialize cost map
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == CELL_OBSTACLE)
                cost_map[y][x] = OBSTACLE_COST;
            else if (grid[y][x] == CELL_UNKNOWN)
                cost_map[y][x] = UNKNOWN_COST;
            else
                cost_map[y][x] = FREE_COST;
        }
    }

    // Step 2: Obstacle inflation (simple distance-based approximation)
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == CELL_FREE) {
                double min_dist = INFLATION_RADIUS + 1; // start with larger than max
                // Search nearby cells
                for (int dy = -INFLATION_RADIUS; dy <= INFLATION_RADIUS; dy++) {
                    for (int dx = -INFLATION_RADIUS; dx <= INFLATION_RADIUS; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (!is_valid_cell(nx, ny)) continue;
                        if (grid[ny][nx] == CELL_OBSTACLE) {
                            double dist = sqrt(dx * dx + dy * dy);
                            if (dist < min_dist)
                                min_dist = dist;
                        }
                    }
                }
                // Apply exponential decay cost: closer to obstacle = higher cost
                if (min_dist <= INFLATION_RADIUS) {
                    double inflated = FREE_COST + (OBSTACLE_COST - FREE_COST) * exp(-min_dist / INFLATION_RADIUS);
                    cost_map[y][x] = clamp(inflated, FREE_COST, OBSTACLE_COST);
                }
            }
        }
    }
}



// Map cost to color: white -> yellow -> orange -> red
static unsigned int cost_to_color(double cost) {
    double minC = FREE_COST;
    double maxC = OBSTACLE_COST;
    double norm = (cost - minC) / (maxC - minC);
    
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;

    int r, g, b;
    double t = norm;
    r = (int)(255 * (1-t)) ;
    g = (int)(255 * (1-t)) ; // 255 constant
    b = (int)(255 * (1-t)) ; // 255 -> 0
    
    /*

    if (norm < 0.33) {
        // White -> Yellow
        double t = norm / 0.33; // 0 -> 1
        r = 255;
        g = (int)(255 - (1-t)); // 255 constant
        b = (int)(255 * (1 - t)); // 255 -> 0
    } else if (norm < 0.66) {
        // Yellow -> Orange
        double t = (norm - 0.33) / 0.33; // 0 -> 1
        r = 255;
        g = (int)(255 - (255 - 165) * t); // 255 -> 165
        b = 0;
    } else {
        // Orange -> Red
        double t = (norm - 0.66) / 0.34; // 0 -> 1
        r = 255;
        g = (int)(165 * (1 - t)); // 165 -> 0
        b = 0;
    }*/
    
    
    // Clamp just in case
    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;

    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}


// Map frontier score -> color gradient (green -> yellow -> red)
static unsigned int frontier_score_to_color(double score, double min_score, double max_score) {
    // Avoid divide by zero
    double norm = 0.0;
    if (max_score > min_score) norm = (score - min_score) / (max_score - min_score);
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;

    int r, g, b;
    // Green (low) -> Yellow (mid) -> Red (high)
    if (norm <= 0.2) {
        // interpolate green (0,255,0) -> yellow (255,255,0)
        double t = (norm / 0.5); // 0..1
        r = (int)(255.0 * t);    // 0 -> 255
        g = 255;
        b = 0;
    } else {
        // interpolate yellow (255,255,0) -> red (255,0,0)
        double t = (norm - 0.2) / 0.2; // 0..1
        r = 255;
        g = (int)(255.0 * (1.0 - t));  // 255 -> 0
        b = 0;
    }

    // clamp
    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;

    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}






// Render grid with robot at center - map moves around it
void render_display(int frame_count) {
    // Clear display
    wb_display_set_color(display, COLOR_BACKGROUND);
    wb_display_fill_rectangle(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);

    // Get robot's current grid position
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);

    // Calculate viewport - what portion of the grid to show
    // The robot will be at the center of the display
    double scale = (double)DISPLAY_WIDTH / (double)GRID_SIZE;
    int viewport_size = (int)(DISPLAY_WIDTH / scale);  // How many grid cells fit on screen
    
    // For better zoom, you can adjust this:
    scale = 3.0;  // Zoom factor - increase for closer view
    viewport_size = (int)(DISPLAY_WIDTH / scale);
    
    int half_viewport = viewport_size / 2;
    int cell_size = (int)(scale + 1);  // Pixel size of each grid cell

    // Calculate grid bounds to render (centered on robot)
    int grid_start_x = robot_gx - half_viewport;
    int grid_end_x = robot_gx + half_viewport;
    int grid_start_y = robot_gy - half_viewport;
    int grid_end_y = robot_gy + half_viewport;

    // Render the grid cells
    for (int gy = grid_start_y; gy <= grid_end_y; gy++) {
        for (int gx = grid_start_x; gx <= grid_end_x; gx++) {
            // Skip invalid cells
            if (!is_valid_cell(gx, gy)) continue;

            unsigned int color = COLOR_UNKNOWN;

            // Determine cell color based on type and cost
            if (grid[gy][gx] == CELL_UNKNOWN) {
                color = COLOR_UNKNOWN;
            }
            else if (grid[gy][gx] == CELL_OBSTACLE) {
                color = COLOR_OBSTACLE;
            }
            else if (grid[gy][gx] == CELL_ROBOT) {
                // Robot will be drawn separately at center
                continue;
            }
            else {
                // Use cost map for free cells
                double c = cost_map[gy][gx];
                color = cost_to_color(c);
            }

            // Calculate screen position relative to robot
            int screen_x = (int)((gx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((gy - grid_start_y + 1) * scale);

            // Draw the cell
            if (grid[gy][gx] != CELL_UNKNOWN) {
                wb_display_set_color(display, color);
                wb_display_fill_rectangle(display, screen_x, screen_y, cell_size, cell_size);
            }
        }
    }

    // Draw frontiers (colored by score; top-scoring frontier highlighted with yellow ring)
    static FrontierCentroid frontiers[MAX_FRONTIERS];
    static int num_frontiers = 0;
    static int frontier_update_counter = 0;
    
    // Update frontiers periodically
    if (++frontier_update_counter >= 20) {
        frontier_update_counter = 0;
        num_frontiers = find_frontier_centroids(frontiers, MAX_FRONTIERS);
    }

    // Determine min/max scores for normalization
    double min_score = 1e9, max_score = -1e9;
    for (int i = 0; i < num_frontiers; i++) {
        if (frontiers[i].score < min_score) min_score = frontiers[i].score;
        if (frontiers[i].score > max_score) max_score = frontiers[i].score;
    }
    if (min_score == 1e9) { min_score = 0.0; max_score = 1.0; } // fallback

    // Draw all frontiers (other frontiers first, selected drawn last)
    for (int i = 0; i < num_frontiers; i++) {
        int fx = frontiers[i].x;
        int fy = frontiers[i].y;
        
        // Check if frontier is visible in viewport
        if (fx >= grid_start_x && fx <= grid_end_x &&
            fy >= grid_start_y && fy <= grid_end_y) {
            
            int screen_x = (int)((fx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * scale);
            
            // Color based on normalized score
            unsigned int fcolor = frontier_score_to_color(frontiers[i].score, min_score, max_score);
            wb_display_set_color(display, fcolor);

            // Draw frontier centroid as a small cross
            int marker_half = 6; // half-length of cross arms
            wb_display_fill_rectangle(display, screen_x - 1, screen_y - marker_half, 2, marker_half * 2);
            wb_display_fill_rectangle(display, screen_x - marker_half, screen_y - 1, marker_half * 2, 2);
        }
    }

    // Highlight selected frontier (best, index 0) with a yellow ring and thicker cross
    if (num_frontiers > 0) {
        int fx = frontiers[0].x;
        int fy = frontiers[0].y;
        if (fx >= grid_start_x && fx <= grid_end_x &&
            fy >= grid_start_y && fy <= grid_end_y) {

            int screen_x = (int)((fx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * scale);

            // Draw thicker contrasting cross (white)
            wb_display_set_color(display, 0xFFFF00);
            int thick = 2;
            int size = 8;
            wb_display_fill_rectangle(display, screen_x - thick, screen_y - size, thick*2, size*2);
            wb_display_fill_rectangle(display, screen_x - size, screen_y - thick, size*2, thick*2);

            // Draw yellow ring around the cross (drawn on top)
            wb_display_set_color(display, 0xFFFF00);

            // ring size in pixels (adjust for visibility)
            int ring_diameter = 8;
            // Correct centering: Webots expects top-left corner, so offset by half diameter
            int ring_x = screen_x ;
            int ring_y = screen_y ;
            
            // Draw two ovals (slightly different sizes) for a thicker ring
            wb_display_draw_oval(display, ring_x, ring_y, ring_diameter, ring_diameter);
        }
    }

    
    // Draw robot at center of display
    int center_x = DISPLAY_WIDTH / 2;
    int center_y = DISPLAY_HEIGHT / 2;
    int robot_size = (int)(scale * 6);  // Make robot slightly larger for visibility
    
    // Draw robot as a circle or square at center
    wb_display_set_color(display, COLOR_ROBOT);
    wb_display_fill_rectangle(display, 
                              center_x - robot_size/2, 
                              center_y - robot_size/2, 
                              robot_size, 
                              robot_size);
    
    // Draw robot orientation indicator
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    double robot_theta = atan2(orientation[3], orientation[0]);
    int arrow_length = robot_size;
    int arrow_end_x = center_x + (int)(arrow_length * cos(robot_theta));
    int arrow_end_y = center_y - (int)(arrow_length * sin(robot_theta));  // Negative because Y is flipped
    
    wb_display_set_color(display, 0xFFFF00);  // Yellow for direction
    wb_display_draw_line(display, center_x, center_y, arrow_end_x, arrow_end_y);

    // Draw text info
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_set_font(display, "Arial", 12, 0);
    wb_display_draw_text(display, "Robot-Centric View", 6, 5);

    char info[256];
    sprintf(info, "Pos: (%d,%d) Zoom: %.1fx", robot_gx, robot_gy, scale);
    wb_display_draw_text(display, info, 6, 20);
    
    sprintf(info, "Frontiers: %d (clearance: %dm)", num_frontiers, MIN_FRONTIER_CLEARANCE);
    wb_display_draw_text(display, info, 6, 35);

    // Legend
    int legend_x = DISPLAY_WIDTH - 110;
    int legend_y = 6;
    int box = 10;
    
    // Free (low cost)
    wb_display_set_color(display, cost_to_color(FREE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "free", legend_x + 14, legend_y + 9);

    // High cost
    wb_display_set_color(display, cost_to_color(OBSTACLE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y + 14, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "obstacle", legend_x + 14, legend_y + 23);

    // Unknown
    wb_display_set_color(display, COLOR_UNKNOWN);
    wb_display_fill_rectangle(display, legend_x, legend_y + 28, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "unknown", legend_x + 14, legend_y + 37);
    
    // Selected frontier indicator
    wb_display_set_color(display, 0xFFFF00);
    wb_display_fill_rectangle(display, legend_x, legend_y + 42, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "target", legend_x + 14, legend_y + 51);
}






















// ============ MAIN ================



int main(int argc, char **argv) {
    // Initialize Webots
    wb_robot_init();

    
    // Get devices
    robot_node = wb_supervisor_node_get_self();
    
    // Initialize display
    display = wb_robot_get_device("display");
    if (!display) {
        printf("Error: No display device found!\n");
        printf("Make sure your robot has a Display node named 'display'\n");
        return 1;
    }
    
    // Initialize LIDAR
    lidar = wb_robot_get_device("laser");
    if (!lidar) {
        printf("Error: No lidar device found!\n");
        return 1;
    }
    wb_lidar_enable(lidar, TIME_STEP);
    
    printf("LIDAR Display Visualization Started\n");
    printf("Robot-Centric View: Robot stays centered, map moves\n");
    printf("Temporal filtering enabled for both obstacles and free cells\n");
    
    // Initialize grid
    init_grid();
    //init_motors();    
    int frame_count = 0;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    // Main loop
    while (wb_robot_step(TIME_STEP) != -1) {
        frame_count++;
        
        // Process LIDAR data
        process_lidar();
        generate_cost_map();
        /*
        diff_drive(250,   // X Goal, in Grid Coord
                    250,  // Y Goal, in Grid Coord
                    5,   // Dist Thresh
                    0.2,  // Rotate Thresh
                    1,  // base
                    1); // Wheel Radius
        // Decay counters periodically for temporal filtering
        if (frame_count % 30 == 0) {
            decay_counters();  // Decay both obstacle and free counters
            filter_connected_components(MIN_BLOB_SIZE);
        }*/
        
        // Clear nearby obstacles periodically
        if (frame_count % 100 == 0) {
            clear_nearby_obstacles(NEARBY_RADIUS_INNER, NEARBY_RADIUS_OUTER);
        }
        
        // Update display every few frames
        if (frame_count % 5 == 0) {
            render_display(frame_count);
        }
    }
    
    wb_robot_cleanup();
    return 0;
}
