// File Name: automaze_C.c
// Version: 1.6.0
// Last Modified: 22.11.2025

// Change Log:
// Fixed the jerky motion due to mistuned PID translate, 20.11.2025
// Fixed the memory leak issue in render display, 22.11.2025
// Fixed the Cost map EDT and obstacle inflation, 22.11.2025

#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/supervisor.h>
#include <webots/display.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// ============== A* IMPLEMENTATION (from AStar.h/AStar.c) ==============
typedef struct __ASNeighborList *ASNeighborList;
typedef struct __ASPath *ASPath;

typedef struct {
    size_t  nodeSize;
    void    (*nodeNeighbors)(ASNeighborList neighbors, void *node, void *context);
    float   (*pathCostHeuristic)(void *fromNode, void *toNode, void *context);
    int     (*earlyExit)(size_t visitedCount, void *visitingNode, void *goalNode, void *context);
    int     (*nodeComparator)(void *node1, void *node2, void *context);
} ASPathNodeSource;

struct __ASNeighborList {
    const ASPathNodeSource *source;
    size_t capacity;
    size_t count;
    float *costs;
    void *nodeKeys;
};

struct __ASPath {
    size_t nodeSize;
    size_t count;
    float cost;
    int8_t nodeKeys[];
};

typedef struct {
    unsigned isClosed:1;
    unsigned isOpen:1;
    unsigned isGoal:1;
    unsigned hasParent:1;
    unsigned hasEstimatedCost:1;
    float estimatedCost;
    float cost;
    size_t openIndex;
    size_t parentIndex;
    int8_t nodeKey[];
} NodeRecord;

struct __VisitedNodes {
    const ASPathNodeSource *source;
    void *context;
    size_t nodeRecordsCapacity;
    size_t nodeRecordsCount;
    void *nodeRecords;
    size_t *nodeRecordsIndex;
    size_t openNodesCapacity;
    size_t openNodesCount;
    size_t *openNodes;
};
typedef struct __VisitedNodes *VisitedNodes;

typedef struct {
    VisitedNodes nodes;
    size_t index;
} Node;

static const Node NodeNull = {NULL, -1};

// A* Helper Functions
static inline VisitedNodes VisitedNodesCreate(const ASPathNodeSource *source, void *context) {
    VisitedNodes nodes = calloc(1, sizeof(struct __VisitedNodes));
    nodes->source = source;
    nodes->context = context;
    return nodes;
}

static inline void VisitedNodesDestroy(VisitedNodes visitedNodes) {
    free(visitedNodes->nodeRecordsIndex);
    free(visitedNodes->nodeRecords);
    free(visitedNodes->openNodes);
    free(visitedNodes);
}

static inline int NodeIsNull(Node n) {
    return (n.nodes == NodeNull.nodes) && (n.index == NodeNull.index);
}

static inline Node NodeMake(VisitedNodes nodes, size_t index) {
    return (Node){nodes, index};
}

static inline NodeRecord *NodeGetRecord(Node node) {
    return node.nodes->nodeRecords + (node.index * (node.nodes->source->nodeSize + sizeof(NodeRecord)));
}

static inline void *GetNodeKey(Node node) {
    return NodeGetRecord(node)->nodeKey;
}

static inline int NodeIsInOpenSet(Node n) {
    return NodeGetRecord(n)->isOpen;
}

static inline int NodeIsInClosedSet(Node n) {
    return NodeGetRecord(n)->isClosed;
}

static inline void RemoveNodeFromClosedSet(Node n) {
    NodeGetRecord(n)->isClosed = 0;
}

static inline void AddNodeToClosedSet(Node n) {
    NodeGetRecord(n)->isClosed = 1;
}

static inline float GetNodeRank(Node n) {
    NodeRecord *record = NodeGetRecord(n);
    return record->estimatedCost + record->cost;
}

static inline float GetNodeCost(Node n) {
    return NodeGetRecord(n)->cost;
}

static inline float GetNodeEstimatedCost(Node n) {
    return NodeGetRecord(n)->estimatedCost;
}

static inline void SetNodeEstimatedCost(Node n, float estimatedCost) {
    NodeRecord *record = NodeGetRecord(n);
    record->estimatedCost = estimatedCost;
    record->hasEstimatedCost = 1;
}

static inline int NodeHasEstimatedCost(Node n) {
    return NodeGetRecord(n)->hasEstimatedCost;
}

static inline void SetNodeIsGoal(Node n) {
    if (!NodeIsNull(n)) {
        NodeGetRecord(n)->isGoal = 1;
    }
}

static inline int NodeIsGoal(Node n) {
    return !NodeIsNull(n) && NodeGetRecord(n)->isGoal;
}

static inline Node GetParentNode(Node n) {
    NodeRecord *record = NodeGetRecord(n);
    if (record->hasParent) {
        return NodeMake(n.nodes, record->parentIndex);
    } else {
        return NodeNull;
    }
}

static inline int NodeRankCompare(Node n1, Node n2) {
    const float rank1 = GetNodeRank(n1);
    const float rank2 = GetNodeRank(n2);
    if (rank1 < rank2) return -1;
    else if (rank1 > rank2) return 1;
    else return 0;
}

static inline float GetPathCostHeuristic(Node a, Node b) {
    if (a.nodes->source->pathCostHeuristic && !NodeIsNull(a) && !NodeIsNull(b)) {
        return a.nodes->source->pathCostHeuristic(GetNodeKey(a), GetNodeKey(b), a.nodes->context);
    } else {
        return 0;
    }
}

static inline int NodeKeyCompare(Node node, void *nodeKey) {
    if (node.nodes->source->nodeComparator) {
        return node.nodes->source->nodeComparator(GetNodeKey(node), nodeKey, node.nodes->context);
    } else {
        return memcmp(GetNodeKey(node), nodeKey, node.nodes->source->nodeSize);
    }
}



static inline Node GetNode(VisitedNodes nodes, void *nodeKey) {
    if (!nodeKey) return NodeNull;
    
    size_t first = 0;
    if (nodes->nodeRecordsCount > 0) {
        size_t last = nodes->nodeRecordsCount-1;
        while (first <= last) {
            const size_t mid = (first + last) / 2;
            const int comp = NodeKeyCompare(NodeMake(nodes, nodes->nodeRecordsIndex[mid]), nodeKey);
            if (comp < 0) {
                first = mid + 1;
            } else if (comp > 0 && mid > 0) {
                last = mid - 1;
            } else if (comp > 0) {
                break;
            } else {
                return NodeMake(nodes, nodes->nodeRecordsIndex[mid]);
            }
        }
    }
    
    if (nodes->nodeRecordsCount == nodes->nodeRecordsCapacity) {
        nodes->nodeRecordsCapacity = 1 + (nodes->nodeRecordsCapacity * 2);
        nodes->nodeRecords = realloc(nodes->nodeRecords, nodes->nodeRecordsCapacity * (sizeof(NodeRecord) + nodes->source->nodeSize));
        nodes->nodeRecordsIndex = realloc(nodes->nodeRecordsIndex, nodes->nodeRecordsCapacity * sizeof(size_t));
    }
    
    Node node = NodeMake(nodes, nodes->nodeRecordsCount);
    nodes->nodeRecordsCount++;
    
    // FIX: Use nodeRecordsCount instead of nodeRecordsCapacity for memmove
    if (first < nodes->nodeRecordsCount - 1) {
        memmove(&nodes->nodeRecordsIndex[first+1], 
                &nodes->nodeRecordsIndex[first], 
                (nodes->nodeRecordsCount - first - 1) * sizeof(size_t));
    }
    nodes->nodeRecordsIndex[first] = node.index;
    
    NodeRecord *record = NodeGetRecord(node);
    memset(record, 0, sizeof(NodeRecord));
    memcpy(record->nodeKey, nodeKey, nodes->source->nodeSize);
    
    return node;
}




static inline void SwapOpenSetNodesAtIndexes(VisitedNodes nodes, size_t index1, size_t index2) {
    if (index1 != index2) {
        NodeRecord *record1 = NodeGetRecord(NodeMake(nodes, nodes->openNodes[index1]));
        NodeRecord *record2 = NodeGetRecord(NodeMake(nodes, nodes->openNodes[index2]));
        
        const size_t tempOpenIndex = record1->openIndex;
        record1->openIndex = record2->openIndex;
        record2->openIndex = tempOpenIndex;
        
        const size_t tempNodeIndex = nodes->openNodes[index1];
        nodes->openNodes[index1] = nodes->openNodes[index2];
        nodes->openNodes[index2] = tempNodeIndex;
    }
}

static inline void DidRemoveFromOpenSetAtIndex(VisitedNodes nodes, size_t index) {
    size_t smallestIndex = index;
    do {
        if (smallestIndex != index) {
            SwapOpenSetNodesAtIndexes(nodes, smallestIndex, index);
            index = smallestIndex;
        }
        const size_t leftIndex = (2 * index) + 1;
        const size_t rightIndex = (2 * index) + 2;
        
        if (leftIndex < nodes->openNodesCount && NodeRankCompare(NodeMake(nodes, nodes->openNodes[leftIndex]), NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) {
            smallestIndex = leftIndex;
        }
        
        if (rightIndex < nodes->openNodesCount && NodeRankCompare(NodeMake(nodes, nodes->openNodes[rightIndex]), NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) {
            smallestIndex = rightIndex;
        }
    } while (smallestIndex != index);
}

static inline void RemoveNodeFromOpenSet(Node n) {
    NodeRecord *record = NodeGetRecord(n);
    if (record->isOpen) {
        record->isOpen = 0;
        n.nodes->openNodesCount--;
        
        const size_t index = record->openIndex;
        SwapOpenSetNodesAtIndexes(n.nodes, index, n.nodes->openNodesCount);
        DidRemoveFromOpenSetAtIndex(n.nodes, index);
    }
}

static inline void DidInsertIntoOpenSetAtIndex(VisitedNodes nodes, size_t index) {
    while (index > 0) {
        const size_t parentIndex = floorf((index-1) / 2);
        
        if (NodeRankCompare(NodeMake(nodes, nodes->openNodes[parentIndex]), NodeMake(nodes, nodes->openNodes[index])) < 0) {
            break;
        } else {
            SwapOpenSetNodesAtIndexes(nodes, parentIndex, index);
            index = parentIndex;
        }
    }
}

static inline void AddNodeToOpenSet(Node n, float cost, Node parent) {
    NodeRecord *record = NodeGetRecord(n);
    
    if (!NodeIsNull(parent)) {
        record->hasParent = 1;
        record->parentIndex = parent.index;
    } else {
        record->hasParent = 0;
    }
    
    if (n.nodes->openNodesCount == n.nodes->openNodesCapacity) {
        n.nodes->openNodesCapacity = 1 + (n.nodes->openNodesCapacity * 2);
        n.nodes->openNodes = realloc(n.nodes->openNodes, n.nodes->openNodesCapacity * sizeof(size_t));
    }
    
    const size_t openIndex = n.nodes->openNodesCount;
    n.nodes->openNodes[openIndex] = n.index;
    n.nodes->openNodesCount++;
    
    record->openIndex = openIndex;
    record->isOpen = 1;
    record->cost = cost;
    
    DidInsertIntoOpenSetAtIndex(n.nodes, openIndex);
}

static inline int HasOpenNode(VisitedNodes nodes) {
    return nodes->openNodesCount > 0;
}

static inline Node GetOpenNode(VisitedNodes nodes) {
    return NodeMake(nodes, nodes->openNodes[0]);
}

static inline ASNeighborList NeighborListCreate(const ASPathNodeSource *source) {
    ASNeighborList list = calloc(1, sizeof(struct __ASNeighborList));
    list->source = source;
    return list;
}

static inline void NeighborListDestroy(ASNeighborList list) {
    free(list->costs);
    free(list->nodeKeys);
    free(list);
}

static inline float NeighborListGetEdgeCost(ASNeighborList list, size_t index) {
    return list->costs[index];
}

static void *NeighborListGetNodeKey(ASNeighborList list, size_t index) {
    return list->nodeKeys + (index * list->source->nodeSize);
}

// A* Public Functions
void ASNeighborListAdd(ASNeighborList list, void *node, float edgeCost) {
    if (list->count == list->capacity) {
        list->capacity = 1 + (list->capacity * 2);
        list->costs = realloc(list->costs, sizeof(float) * list->capacity);
        list->nodeKeys = realloc(list->nodeKeys, list->source->nodeSize * list->capacity);
    }
    list->costs[list->count] = edgeCost;
    memcpy(list->nodeKeys + (list->count * list->source->nodeSize), node, list->source->nodeSize);
    list->count++;
}

ASPath ASPathCreate(const ASPathNodeSource *source, void *context, void *startNodeKey, void *goalNodeKey) {
    if (!startNodeKey || !source || !source->nodeNeighbors || source->nodeSize == 0) {
        return NULL;
    }
    
    VisitedNodes visitedNodes = VisitedNodesCreate(source, context);
    ASNeighborList neighborList = NeighborListCreate(source);
    Node current = GetNode(visitedNodes, startNodeKey);
    Node goalNode = GetNode(visitedNodes, goalNodeKey);
    ASPath path = NULL;
    
    SetNodeIsGoal(goalNode);
    SetNodeEstimatedCost(current, GetPathCostHeuristic(current, goalNode));
    AddNodeToOpenSet(current, 0, NodeNull);
    
    while (HasOpenNode(visitedNodes) && !NodeIsGoal((current = GetOpenNode(visitedNodes)))) {
        if (source->earlyExit) {
            const int shouldExit = source->earlyExit(visitedNodes->nodeRecordsCount, GetNodeKey(current), goalNodeKey, context);
            if (shouldExit > 0) {
                SetNodeIsGoal(current);
                break;
            } else if (shouldExit < 0) {
                break;
            }
        }
        
        RemoveNodeFromOpenSet(current);
        AddNodeToClosedSet(current);
        
        neighborList->count = 0;
        source->nodeNeighbors(neighborList, GetNodeKey(current), context);
        
        for (size_t n=0; n<neighborList->count; n++) {
            const float cost = GetNodeCost(current) + NeighborListGetEdgeCost(neighborList, n);
            Node neighbor = GetNode(visitedNodes, NeighborListGetNodeKey(neighborList, n));
            
            if (!NodeHasEstimatedCost(neighbor)) {
                SetNodeEstimatedCost(neighbor, GetPathCostHeuristic(neighbor, goalNode));
            }
            
            if (NodeIsInOpenSet(neighbor) && cost < GetNodeCost(neighbor)) {
                RemoveNodeFromOpenSet(neighbor);
            }
            
            if (NodeIsInClosedSet(neighbor) && cost < GetNodeCost(neighbor)) {
                RemoveNodeFromClosedSet(neighbor);
            }
            
            if (!NodeIsInOpenSet(neighbor) && !NodeIsInClosedSet(neighbor)) {
                AddNodeToOpenSet(neighbor, cost, current);
            }
        }
    }
    
    if (NodeIsNull(goalNode)) {
        SetNodeIsGoal(current);
    }
    
    if (NodeIsGoal(current)) {
        size_t count = 0;
        Node n = current;
        
        while (!NodeIsNull(n)) {
            count++;
            n = GetParentNode(n);
        }
        
        path = malloc(sizeof(struct __ASPath) + (count * source->nodeSize));
        path->nodeSize = source->nodeSize;
        path->count = count;
        path->cost = GetNodeCost(current);
        
        n = current;
        for (size_t i=count; i>0; i--) {
            memcpy(path->nodeKeys + ((i - 1) * source->nodeSize), GetNodeKey(n), source->nodeSize);
            n = GetParentNode(n);
        }
    }
    
    NeighborListDestroy(neighborList);
    VisitedNodesDestroy(visitedNodes);
    
    return path;
}

void ASPathDestroy(ASPath path) {
    free(path);
}

ASPath ASPathCopy(ASPath path) {
    if (path) {
        const size_t size = sizeof(struct __ASPath) + (path->count * path->nodeSize);
        ASPath newPath = malloc(size);
        memcpy(newPath, path, size);
        return newPath;
    } else {
        return NULL;
    }
}

float ASPathGetCost(ASPath path) {
    return path? path->cost : INFINITY;
}

size_t ASPathGetCount(ASPath path) {
    return path? path->count : 0;
}

void *ASPathGetNode(ASPath path, size_t index) {
    return (path && index < path->count)? (path->nodeKeys + (index * path->nodeSize)) : NULL;
}
















































// ============== WEBOTS ROBOT CONTROLLER ==============

// Configuration
#define TIME_STEP 32
#define GRID_SIZE 500
#define GRID_RESOLUTION 0.02
#define DISPLAY_WIDTH 500
#define DISPLAY_HEIGHT 500

// Grid cell types
#define CELL_UNKNOWN 0
#define CELL_FREE 1
#define CELL_OBSTACLE 2
#define CELL_ROBOT 3

// Temporal filtering thresholds
#define OBSTACLE_THRESHOLD 2
#define FREE_THRESHOLD 3
#define COUNTER_DECAY 1

#define NEARBY_RADIUS_INNER 5
#define NEARBY_RADIUS_OUTER 15

#define MAX_QUEUE 1000
#define MIN_BLOB_SIZE 8

// Cost Map
#define OBSTACLE_COST 200
#define FREE_COST 1
#define UNKNOWN_COST 30
#define INFLATION_RADIUS 10

// Frontiers
#define MAX_FRONTIERS 100
#define MIN_FRONTIER_SIZE 20
#define MIN_FRONTIER_CLEARANCE 5
#define FRONTIER_SAFETY_CHECK_RADIUS 5

// Colors
#define COLOR_UNKNOWN 0x404040
#define COLOR_FREE 0xC8C8C8
#define COLOR_OBSTACLE 0x000000
#define COLOR_ROBOT 0x0064FF
#define COLOR_BACKGROUND 0x303030

// IR Safety Configuration
#define IR_SAFETY_THRESHOLD 0.10  // Stop if any sensor detects obstacle closer than this (in meters)
#define IR_CRITICAL_THRESHOLD 0.05  // Emergency stop threshold



// ====== Structures =======

// State Machine
typedef enum {
    STATE_START, // Check if too close to an obstacle
    STATE_FRONTIER_EXPLORE,
    STATE_NAV_TO_BLUE,
    STATE_NAV_TO_YELLOW,
    STATE_COLLISION,
    STATE_WANDER
} State;


// Point structure
typedef struct {
    int x;
    int y;
} Point;

// Grid Node for A*
typedef struct {
    int x;
    int y;
} GridNode;

// Path structure
typedef struct {
    GridNode* nodes;
    int count;
    int capacity;
} GridPath;

// A* Context
typedef struct {
    double* cost_map_ptr;
    unsigned int* grid_ptr;
    int grid_size;
} AStarContext;

// Frontier structure
typedef struct {
    int x;
    int y;
    int size;
    double score;
} FrontierCentroid;

// Global variables
static unsigned int grid[GRID_SIZE][GRID_SIZE];
static unsigned int obstacle_counter[GRID_SIZE][GRID_SIZE];
static unsigned int free_counter[GRID_SIZE][GRID_SIZE];
static double cost_map[GRID_SIZE][GRID_SIZE];
static unsigned char frontier_edge_map[GRID_SIZE][GRID_SIZE];
static WbDeviceTag display;
static WbDeviceTag lidar;
static WbDeviceTag motors[4];
static WbNodeRef robot_node;


static FrontierCentroid frontiers[MAX_FRONTIERS];
static int num_frontiers = 0;
static double frontier_min_score = 0.0;
static double frontier_max_score = 1.0;

static WbDeviceTag ir_sensors[4];
static const char* ir_sensor_names[4] = {
    "fl_range", "fr_range", "rl_range", "rr_range"
};
static bool ir_safety_triggered = false;

// Path following variables
static GridPath* current_path = NULL;
static int current_waypoint = 0;
static int path_update_counter = 0;


static double pid_rotate_integral = 0.0;
static double pid_rotate_prev_error = 0.0;
static double pid_translate_integral = 0.0;
static double pid_translate_prev_error = 0.0;

// ============ INITIALIZATION ===============

void init_grid() {
    memset(grid, CELL_UNKNOWN, sizeof(grid));
    memset(obstacle_counter, 0, sizeof(obstacle_counter));
    memset(free_counter, 0, sizeof(free_counter));
}

void init_motors() {
    motors[0] = wb_robot_get_device("fl_wheel_joint");
    motors[1] = wb_robot_get_device("rl_wheel_joint");
    motors[2] = wb_robot_get_device("fr_wheel_joint");
    motors[3] = wb_robot_get_device("rr_wheel_joint");
    
    for (int i = 0; i < 4; i++) {
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 0.0);
    }
}

void init_ir_sensors() {
    for (int i = 0; i < 4; i++) {
        ir_sensors[i] = wb_robot_get_device(ir_sensor_names[i]);
        if (ir_sensors[i]) {
            wb_distance_sensor_enable(ir_sensors[i], TIME_STEP);
            printf("IR sensor %s initialized\n", ir_sensor_names[i]);
        } else {
            printf("Warning: IR sensor %s not found!\n", ir_sensor_names[i]);
        }
    }
}

bool check_ir_safety(double* min_distance, int* sensor_index) {
    *min_distance = INFINITY;
    *sensor_index = -1;
    bool safe = true;
    
    for (int i = 0; i < 4; i++) {
        if (!ir_sensors[i]) continue;
        
        double distance = wb_distance_sensor_get_value(ir_sensors[i]);
        
        // Convert sensor reading to meters (depends on your sensor model)
        // For standard Webots distance sensors, the value is already in meters
        // If your sensor returns different units, adjust here
        
        if (distance < *min_distance) {
            *min_distance = distance;
            *sensor_index = i;
        }
        
        if (distance < IR_SAFETY_THRESHOLD) {
            safe = false;
        }
    }
    
    return safe;
}

void emergency_stop() {
    for (int i = 0; i < 4; i++) {
        wb_motor_set_velocity(motors[i], 0.0);
    }
    printf("EMERGENCY STOP: IR safety triggered!\n");
}

// =========== UTILITIES =============

void world_to_grid(double wx, double wy, int* gx, int* gy) {
    *gx = (int)((wx / GRID_RESOLUTION) + GRID_SIZE / 2);
    *gy = (int)((wy / GRID_RESOLUTION) + GRID_SIZE / 2);
}

int is_valid_cell(int x, int y) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

double clamp(double val, double min, double max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}

// ============== A* PATHFINDING CALLBACKS ==============

void grid_node_neighbors(ASNeighborList neighbors, void *node, void *context) {
    GridNode *current = (GridNode*)node;
    AStarContext *ctx = (AStarContext*)context;
    
    int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    double costs[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};
    
    for (int i = 0; i < 8; i++) {
        int nx = current->x + dx[i];
        int ny = current->y + dy[i];
        
        if (nx < 0 || nx >= ctx->grid_size || ny < 0 || ny >= ctx->grid_size)
            continue;
            
        unsigned int cell = ctx->grid_ptr[ny * ctx->grid_size + nx];
        if (cell == CELL_OBSTACLE)
            continue;
            
        double base_cost = ctx->cost_map_ptr[ny * ctx->grid_size + nx];
        double edge_cost = costs[i] * base_cost;
        
        GridNode neighbor = {nx, ny};
        ASNeighborListAdd(neighbors, &neighbor, edge_cost);
    }
}

float grid_path_cost_heuristic(void *fromNode, void *toNode, void *context) {
    GridNode *from = (GridNode*)fromNode;
    GridNode *to = (GridNode*)toNode;
    
    float dx = (float)(to->x - from->x);
    float dy = (float)(to->y - from->y);
    
    return sqrtf(dx * dx + dy * dy);
}

int grid_node_comparator(void *node1, void *node2, void *context) {
    GridNode *n1 = (GridNode*)node1;
    GridNode *n2 = (GridNode*)node2;
    
    if (n1->y < n2->y) return -1;
    if (n1->y > n2->y) return 1;
    if (n1->x < n2->x) return -1;
    if (n1->x > n2->x) return 1;
    return 0;
}

int grid_early_exit(size_t visitedCount, void *visitingNode, void *goalNode, void *context) {
    if (visitedCount > 10000) {
        return -1;
    }
    return 0;
}

// ============== A* PATHFINDING ==============

GridPath* find_path_astar(int start_x, int start_y, int goal_x, int goal_y) {
    if (!is_valid_cell(start_x, start_y) || !is_valid_cell(goal_x, goal_y)) {
        printf("Invalid start or goal position\n");
        return NULL;
    }
    
    if (grid[goal_y][goal_x] == CELL_OBSTACLE) {
        printf("Goal position is an obstacle\n");
        return NULL;
    }
    
    AStarContext context;
    context.cost_map_ptr = (double*)cost_map;
    context.grid_ptr = (unsigned int*)grid;
    context.grid_size = GRID_SIZE;
    
    ASPathNodeSource source = {
        .nodeSize = sizeof(GridNode),
        .nodeNeighbors = grid_node_neighbors,
        .pathCostHeuristic = grid_path_cost_heuristic,
        .earlyExit = grid_early_exit,
        .nodeComparator = grid_node_comparator
    };
    
    GridNode start_node = {start_x, start_y};
    GridNode goal_node = {goal_x, goal_y};
    
    ASPath path = ASPathCreate(&source, &context, &start_node, &goal_node);
    
    if (!path) {
        printf("No path found from (%d,%d) to (%d,%d)\n", 
               start_x, start_y, goal_x, goal_y);
        return NULL;
    }
    
    size_t path_count = ASPathGetCount(path);
    float path_cost = ASPathGetCost(path);
    
    printf("Path found! Length: %zu, Cost: %.2f\n", path_count, path_cost);
    
    GridPath* grid_path = malloc(sizeof(GridPath));
    grid_path->count = path_count;
    grid_path->capacity = path_count;
    grid_path->nodes = malloc(sizeof(GridNode) * path_count);
    
    for (size_t i = 0; i < path_count; i++) {
        GridNode* node = (GridNode*)ASPathGetNode(path, i);
        grid_path->nodes[i] = *node;
    }
    
    ASPathDestroy(path);
    
    return grid_path;
}

void destroy_grid_path(GridPath* path) {
    if (path) {
        free(path->nodes);
        free(path);
    }
}


// =============== ROBOT CONTROL ===============
double pid_rotate(double angle_error) {
    static double Kp_rotate = 1.0;  
    static double Ki_rotate = 0.0001;  
    static double Kd_rotate = 0.001;  
    
    double integral_limit = 1.0;  
    pid_rotate_integral += angle_error * (TIME_STEP / 1000.0);
    pid_rotate_integral = clamp(pid_rotate_integral, -integral_limit, integral_limit);
    
    double derivative = (angle_error - pid_rotate_prev_error) / (TIME_STEP / 1000.0);
    pid_rotate_prev_error = angle_error;
    
    double omega = Kp_rotate * angle_error + Ki_rotate * pid_rotate_integral + Kd_rotate * derivative;
    
    return omega;
}

double pid_translate(double distance_error) {
    static double Kp_translate = 0.02;  
    static double Kd_translate = 0.0;  
    
    double derivative = (distance_error - pid_translate_prev_error) / (TIME_STEP / 1000.0);
    pid_translate_prev_error = distance_error;
    
    double speed = Kp_translate * distance_error + Kd_translate * derivative;
    
    return clamp(speed, 0, 2.0);
}

void reset_pid_controllers() {
    pid_rotate_integral = 0.0;
    pid_rotate_prev_error = 0.0;
    pid_translate_integral = 0.0;
    pid_translate_prev_error = 0.0;
}


double rotate_drive(double rotations, double angle_threshold, double b, double r) {
    // Rotation in radians (full rotations)
    double radians_target = rotations * 2.0 * M_PI;

    // Current orientation based on rotation matrix
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    double robot_theta = atan2(orientation[3], orientation[0]);

    // Angle difference
    double angle_diff = normalize_angle(radians_target - robot_theta);

    // --- stop condition ---
    if (fabs(angle_diff) < angle_threshold) {
        // Stop motors
        wb_motor_set_velocity(motors[0], 0);
        wb_motor_set_velocity(motors[1], 0);
        wb_motor_set_velocity(motors[2], 0);
        wb_motor_set_velocity(motors[3], 0);

        return 1.0; // success
    }

    // PID angular velocity command
    double omega = pid_rotate(angle_diff);

    // Convert body angular velocity to wheel velocities
    double omega_l = (-omega * b / 2.0) / r;
    double omega_r = (+omega * b / 2.0) / r;

    // Set wheel velocities correctly
    wb_motor_set_velocity(motors[0], omega_l); // left front
    wb_motor_set_velocity(motors[1], omega_l); // left rear
    wb_motor_set_velocity(motors[2], omega_r); // right front
    wb_motor_set_velocity(motors[3], omega_r); // right rear

    return 0.0; // still rotating
}



bool diff_drive(int x_goal, int y_goal, double distance_threshold, 
                double angle_threshold, double b, double r) {
    // IR Safety Check
    double min_ir_distance;
    int triggered_sensor;
    bool ir_safe = 1;
    //bool ir_safe = check_ir_safety(&min_ir_distance, &triggered_sensor);
    
    if (!ir_safe) {
        if (min_ir_distance < IR_CRITICAL_THRESHOLD) {
            // Critical distance - emergency stop
            //emergency_stop();
            ir_safety_triggered = true;
            printf("CRITICAL: IR sensor %s detected obstacle at %.3fm - EMERGENCY STOP\n", 
                   ir_sensor_names[triggered_sensor], min_ir_distance);
            return false;
        } else {
            // Warning distance - slow down significantly
            printf("WARNING: IR sensor %s detected obstacle at %.3fm - reducing speed\n", 
                   ir_sensor_names[triggered_sensor], min_ir_distance);
            ir_safety_triggered = true;
        }
    } else {
        ir_safety_triggered = false;
    }
    
    const double* position = wb_supervisor_node_get_position(robot_node);
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    double dx = x_goal - robot_gx;
    double dy = y_goal - robot_gy;
    double distance = sqrt(dx * dx + dy * dy);
    
    double angle_to_goal = atan2(dy, dx);
    double angle_diff = normalize_angle(angle_to_goal - robot_theta);
    
    if (distance < distance_threshold) {
        for (int i = 0; i < 4; i++) {
            wb_motor_set_velocity(motors[i], 0.0);
        }
        return true;
    }
    
    double omega, speed;
    
    if (fabs(angle_diff) > angle_threshold) {
        omega = pid_rotate(angle_diff);
        speed = 0.0;
    } else {
        omega = pid_rotate(angle_diff);
        speed = pid_translate(distance);
    }
    
    // Apply IR safety speed reduction
    if (ir_safety_triggered && min_ir_distance < IR_SAFETY_THRESHOLD) {
        // Scale speed based on proximity
        double safety_factor = (min_ir_distance - IR_CRITICAL_THRESHOLD) / 
                               (IR_SAFETY_THRESHOLD - IR_CRITICAL_THRESHOLD);
        safety_factor = clamp(safety_factor, 0.0, 1.0);
        speed *= safety_factor * 0.3;  // Reduce to max 30% of normal speed
        omega *= 0.5;  // Also reduce rotation speed
    }
    
    double omega_l = (speed - omega * b / 2.0) / r;
    double omega_r = (speed + omega * b / 2.0) / r;
    
    wb_motor_set_velocity(motors[0], omega_l);
    wb_motor_set_velocity(motors[1], omega_l);
    wb_motor_set_velocity(motors[2], omega_r);
    wb_motor_set_velocity(motors[3], omega_r);
    
    return false;
}




bool follow_path() {
    if (!current_path || current_waypoint >= current_path->count) {
        return false;
    }
    
    // Track if we're making progress
    static int stuck_counter = 0;
    static int last_waypoint = -1;
    
    if (current_waypoint == last_waypoint) {
        stuck_counter++;
        if (stuck_counter > 50) { // Stuck for 50 frames
            printf("Path following stuck, replanning...\n");
            destroy_grid_path(current_path);
            current_path = NULL;
            stuck_counter = 0;
            last_waypoint = -1;
            return true; // Force replan
        }
    } else {
        stuck_counter = 0;
        last_waypoint = current_waypoint;
    }
    
    // Rest of the path following logic...
    const double* position = wb_supervisor_node_get_position(robot_node);
    int robot_gx, robot_gy;
    world_to_grid(position[0], position[1], &robot_gx, &robot_gy);
    
    // Skip close waypoints
    while (current_waypoint < current_path->count - 1) {
        GridNode* next_wp = &current_path->nodes[current_waypoint];
        double dx = next_wp->x - robot_gx;
        double dy = next_wp->y - robot_gy;
        double dist = sqrt(dx * dx + dy * dy);
        
        if (dist < 10.0) {
            current_waypoint++;
            reset_pid_controllers();
        } else {
            break;
        }
    }
    
    GridNode* waypoint = &current_path->nodes[current_waypoint];
    
    bool reached = diff_drive(
        waypoint->x, waypoint->y,
        10.0, 0.3,
        0.287, 0.0825
    );
    
    if (reached) {
        current_waypoint++;
        printf("Reached waypoint %d/%d\n", current_waypoint, current_path->count);
        
        if (current_waypoint >= current_path->count) {
            printf("Path completed!\n");
            return true;
        }
    }
    
    return false;
}





// =============== MAPPING AND PERCEPTION ===============

void draw_line_on_grid(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        if (is_valid_cell(x0, y0)) {
            if (x0 != x1 || y0 != y1) {
                if (grid[y0][x0] != CELL_OBSTACLE) {
                    free_counter[y0][x0]++;
                    if (free_counter[y0][x0] >= FREE_THRESHOLD) {
                        grid[y0][x0] = CELL_FREE;
                    }
                    obstacle_counter[y0][x0] = 0;
                }
            } else {
                obstacle_counter[y0][x0]++;
                if (obstacle_counter[y0][x0] >= OBSTACLE_THRESHOLD) {
                    grid[y0][x0] = CELL_OBSTACLE;
                    free_counter[y0][x0] = 0;
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

            int dist_sq = dx * dx + dy * dy;
            if (dist_sq > outer_radius_cells * outer_radius_cells) continue;
            if (dist_sq < inner_radius_cells * inner_radius_cells) continue;

            if (grid[gy][gx] == CELL_OBSTACLE) {
                grid[gy][gx] = CELL_UNKNOWN;
            }
        }
    }
}

void filter_connected_components(int min_size) {
    static bool visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));

    int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};

    Point queue[MAX_QUEUE];

    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] == CELL_OBSTACLE && !visited[y][x]) {
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

void process_lidar() {
    const double* position = wb_supervisor_node_get_position(robot_node);
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    const float* ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    double max_range = 1.5;
    
    for (int i = 0; i < resolution; i += 2) {
        double range = ranges[i];
        
        if (range < 0.05 || range > max_range * 0.95) {
            continue;
        }
        
        double ray_angle = fov/2 - (i * fov / resolution);
        double world_angle = robot_theta + ray_angle;
        
        double end_x = robot_x + range * cos(world_angle);
        double end_y = robot_y + range * sin(world_angle);
        
        int end_gx, end_gy;
        world_to_grid(end_x, end_y, &end_gx, &end_gy);
        
        draw_line_on_grid(robot_gx, robot_gy, end_gx, end_gy);
    }
    
    if (is_valid_cell(robot_gx, robot_gy)) {
        grid[robot_gy][robot_gx] = CELL_ROBOT;
    }
}

void decay_counters() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] != CELL_OBSTACLE && obstacle_counter[y][x] > 0) {
                obstacle_counter[y][x] -= COUNTER_DECAY;
                if (obstacle_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
            
            if (grid[y][x] != CELL_FREE && free_counter[y][x] > 0) {
                free_counter[y][x] -= COUNTER_DECAY;
                if (free_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
        }
    }
}














inline void relax(double dist_transform[GRID_SIZE][GRID_SIZE],
                  int x, int y, int nx, int ny, double cost)
{
    double new_dist = dist_transform[ny][nx] + cost;
    if (new_dist < dist_transform[y][x])
        dist_transform[y][x] = new_dist;
}

void generate_cost_map() {
    static double dist_transform[GRID_SIZE][GRID_SIZE];
    const double INF = GRID_SIZE * GRID_SIZE * 2.0;
    const double SQRT2 = 1.414213562;

    // ---- Step 1: Initialize ----
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            dist_transform[y][x] =
                (grid[y][x] == CELL_OBSTACLE) ? 0.0 : INF;
        }
    }

    // Offsets and costs: 8 directions
    const int dx[8]   = {-1,  0, -1,  1,  1,  0,  1, -1};
    const int dy[8]   = { 0, -1, -1, -1,  0,  1,  1,  1};
    const double w[8] = { 1,   1,  SQRT2, SQRT2, 1,  1, SQRT2, SQRT2 };

    // ---- Step 2: Forward pass ----
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (dist_transform[y][x] == 0) continue;

            for (int k = 0; k < 4; k++) {  // first 4 directions = forward neighbors
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx >= 0 && ny >= 0 && nx < GRID_SIZE && ny < GRID_SIZE)
                    relax(dist_transform, x, y, nx, ny, w[k]);
            }
        }
    }

    // ---- Step 3: Backward pass ----
    for (int y = GRID_SIZE - 1; y >= 0; y--) {
        for (int x = GRID_SIZE - 1; x >= 0; x--) {
            if (dist_transform[y][x] == 0) continue;

            for (int k = 4; k < 8; k++) {  // last 4 directions = backward neighbors
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx >= 0 && ny >= 0 && nx < GRID_SIZE && ny < GRID_SIZE)
                    relax(dist_transform, x, y, nx, ny, w[k]);
            }
        }
    }

    // ---- Step 4: Convert to cost map ----
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {

            if (grid[y][x] == CELL_OBSTACLE) {
                cost_map[y][x] = OBSTACLE_COST;
                continue;
            }

            if (grid[y][x] == CELL_UNKNOWN) {
                cost_map[y][x] = UNKNOWN_COST;
                continue;
            }

            double dist = dist_transform[y][x];

            if (dist <= INFLATION_RADIUS) {
                double t = dist / INFLATION_RADIUS;
                cost_map[y][x] = OBSTACLE_COST - t * (OBSTACLE_COST - FREE_COST);
            } else {
                cost_map[y][x] = FREE_COST;
            }
        }
    }
}










// =============== FRONTIER EXPLORATION ===============

static inline bool is_frontier_edge(int x, int y) {
    if (grid[y][x] != CELL_FREE) return false;
    
    if ((y > 0 && grid[y-1][x] == CELL_UNKNOWN) ||
        (y < GRID_SIZE-1 && grid[y+1][x] == CELL_UNKNOWN) ||
        (x > 0 && grid[y][x-1] == CELL_UNKNOWN) ||
        (x < GRID_SIZE-1 && grid[y][x+1] == CELL_UNKNOWN)) {
        return true;
    }
    return false;
}

void build_frontier_edge_map() {
    memset(frontier_edge_map, 0, sizeof(frontier_edge_map));
    
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    int scan_radius = 150;
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

bool has_safe_clearance(int cx, int cy, int min_clearance) {
    for (int dy = -min_clearance; dy <= min_clearance; dy++) {
        for (int dx = -min_clearance; dx <= min_clearance; dx++) {
            int check_x = cx + dx;
            int check_y = cy + dy;
            
            if (!is_valid_cell(check_x, check_y)) continue;
            
            if (grid[check_y][check_x] == CELL_OBSTACLE) {
                double dist = sqrt(dx * dx + dy * dy);
                if (dist <= min_clearance) {
                    return false;
                }
            }
        }
    }
    return true;
}

int find_frontier_centroids(FrontierCentroid* centroids, int max_frontiers) {
    build_frontier_edge_map();
    
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    static bool visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));
    
    int num_frontiers = 0;
    
    int scan_radius = 150;
    int min_x = fmax(0, robot_gx - scan_radius);
    int max_x = fmin(GRID_SIZE - 1, robot_gx + scan_radius);
    int min_y = fmax(0, robot_gy - scan_radius);
    int max_y = fmin(GRID_SIZE - 1, robot_gy + scan_radius);
    
    static Point queue[1000];
    
    for (int y = min_y; y <= max_y && num_frontiers < max_frontiers; y++) {
        for (int x = min_x; x <= max_x && num_frontiers < max_frontiers; x++) {
            if (frontier_edge_map[y][x] && !visited[y][x]) {
                int head = 0, tail = 0;
                int sum_x = 0, sum_y = 0, count = 0;
                
                queue[tail++] = (Point){x, y};
                visited[y][x] = true;
                
                while (head < tail && tail < 1000) {
                    Point p = queue[head++];
                    sum_x += p.x;
                    sum_y += p.y;
                    count++;
                    
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
                                
                                if (tail >= 1000) break;
                            }
                        }
                    }
                }
                
                if (count >= MIN_FRONTIER_SIZE) {
                    int cx = sum_x / count;
                    int cy = sum_y / count;
                    
                    if (has_safe_clearance(cx, cy, MIN_FRONTIER_CLEARANCE)) {
                        centroids[num_frontiers].x = cx;
                        centroids[num_frontiers].y = cy;
                        centroids[num_frontiers].size = count;
                        
                        double dist = sqrt((cx - robot_gx) * (cx - robot_gx) + 
                                         (cy - robot_gy) * (cy - robot_gy));
                        
                        centroids[num_frontiers].score = (count * 2.0) / (1.0 + dist * 0.02);
                        
                        num_frontiers++;
                    }
                }
            }
        }
    }
    
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

FrontierCentroid* get_best_frontier(FrontierCentroid* frontiers, int num_frontiers) {
    return (num_frontiers > 0) ? &frontiers[0] : NULL;
}

void update_path_to_frontier() {
    static FrontierCentroid frontiers[MAX_FRONTIERS];
    int num_frontiers = find_frontier_centroids(frontiers, MAX_FRONTIERS);
    
    if (num_frontiers == 0) {
        printf("No frontiers found - exploration complete!\n");
        return;
    }
    
    FrontierCentroid* best_frontier = get_best_frontier(frontiers, num_frontiers);
    if (!best_frontier) {
        return;
    }
    
    const double* position = wb_supervisor_node_get_position(robot_node);
    int robot_gx, robot_gy;
    world_to_grid(position[0], position[1], &robot_gx, &robot_gy);
    
    if (current_path) {
        destroy_grid_path(current_path);
        current_path = NULL;
    }
    
    printf("Planning path to frontier at (%d, %d) with score %.2f\n", 
           best_frontier->x, best_frontier->y, best_frontier->score);
    
    current_path = find_path_astar(robot_gx, robot_gy, 
                                   best_frontier->x, best_frontier->y);
    current_waypoint = 0;
    
    if (!current_path) {
        printf("Failed to find path to frontier\n");
    }
}










// =============== DISPLAY RENDERING ===============

// Map cost to color
static unsigned int cost_to_color(double cost) {
    double minC = FREE_COST;
    double maxC = OBSTACLE_COST;
    double norm = (cost - minC) / (maxC - minC);
    
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;

    int r, g, b;
    double t = norm;

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
    }
    
    
    // Clamp safety
    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;

    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}

static unsigned int frontier_score_to_color(double score, double min_score, double max_score) {
    double norm = 0.0;
    if (max_score > min_score) norm = (score - min_score) / (max_score - min_score);
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;

    int r, g, b;
    if (norm <= 0.2) {
        double t = (norm / 0.5);
        r = (int)(255.0 * t);
        g = 255;
        b = 0;
    } else {
        double t = (norm - 0.2) / 0.2;
        r = 255;
        g = (int)(255.0 * (1.0 - t));
        b = 0;
    }

    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;

    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}

void draw_path_on_display(WbDeviceTag display, GridPath* path, int robot_gx, int robot_gy, 
                          double scale, int grid_start_x, int grid_start_y) {
    if (!path) return;
    
    wb_display_set_color(display, 0x00FF00);
    
    for (int i = 0; i < path->count - 1; i++) {
        GridNode* n1 = &path->nodes[i];
        GridNode* n2 = &path->nodes[i + 1];
        
        int x1 = (int)((n1->x - grid_start_x) * scale);
        int y1 = DISPLAY_HEIGHT - (int)((n1->y - grid_start_y + 1) * scale);
        int x2 = (int)((n2->x - grid_start_x) * scale);
        int y2 = DISPLAY_HEIGHT - (int)((n2->y - grid_start_y + 1) * scale);
        
        wb_display_draw_line(display, x1, y1, x2, y2);
    }
    
    wb_display_set_color(display, 0x00FFFF);
    for (int i = 0; i < path->count; i++) {
        GridNode* n = &path->nodes[i];
        int x = (int)((n->x - grid_start_x) * scale);
        int y = DISPLAY_HEIGHT - (int)((n->y - grid_start_y + 1) * scale);
        
        if (i == current_waypoint) {
            wb_display_set_color(display, 0xFF00FF);
            wb_display_fill_oval(display, x - 3, y - 3, 6, 6);
        } else {
            wb_display_set_color(display, 0x0000FF);
            wb_display_draw_oval(display, x - 2, y - 2, 4, 4);
        }
    }
}

// Display IR sensor status
void display_ir_status(int frame_count) {
    double min_distance;
    int sensor_idx;
    check_ir_safety(&min_distance, &sensor_idx);
    
    int status_x = 6;
    int status_y = DISPLAY_HEIGHT - 60;
    
    // Background box for IR status
    if (ir_safety_triggered) {
        wb_display_set_color(display, 0xFF0000);
        wb_display_fill_rectangle(display, status_x - 2, status_y - 2, 180, 55);
    }
    
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_set_font(display, "Arial", 10, 0);
    wb_display_draw_text(display, "IR Sensors:", status_x, status_y);
    
    for (int i = 0; i < 4; i++) {
        if (!ir_sensors[i]) continue;
        
        double dist = wb_distance_sensor_get_value(ir_sensors[i]);
        char sensor_info[64];
        sprintf(sensor_info, "%s: %.3fm", ir_sensor_names[i], dist);
        
        // Color code based on distance
        if (dist < IR_CRITICAL_THRESHOLD) {
            wb_display_set_color(display, 0xFF0000);  // Red
        } else if (dist < IR_SAFETY_THRESHOLD) {
            wb_display_set_color(display, 0xFFAA00);  // Orange
        } else {
            wb_display_set_color(display, 0x00FF00);  // Green
        }
        
        wb_display_draw_text(display, sensor_info, status_x, status_y + 12 + (i * 10));
    }
    
    // Status message
    if (ir_safety_triggered) {
        wb_display_set_color(display, 0xFFFF00);
        wb_display_draw_text(display, "SAFETY ACTIVE", status_x + 100, status_y + 20);
    }
}














void render_display_with_path(int frame_count) {
    // Lazy one-time font initialization to avoid repeated font allocations
    static bool font_initialized = false;
    if (!font_initialized) {
        wb_display_set_font(display, "Arial", 12, 0);
        font_initialized = true;
    }

    // Clear background
    wb_display_set_color(display, COLOR_BACKGROUND);
    wb_display_fill_rectangle(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);

    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);

    double scale = 3.0;
    int viewport_size = (int)(DISPLAY_WIDTH / scale);
    int half_viewport = viewport_size / 2;
    int cell_size = (int)(scale + 1);

    int grid_start_x = robot_gx - half_viewport;
    int grid_end_x = robot_gx + half_viewport;
    int grid_start_y = robot_gy - half_viewport;
    int grid_end_y = robot_gy + half_viewport;

    // Draw visible cells. Minimize set_color calls by tracking last_color.
    unsigned int last_color = 0xFFFFFFFF; // impossible to match
    for (int gy = grid_start_y; gy <= grid_end_y; gy++) {
        for (int gx = grid_start_x; gx <= grid_end_x; gx++) {
            if (!is_valid_cell(gx, gy)) continue;

            unsigned int color;
            unsigned int cell = grid[gy][gx];

            if (cell == CELL_UNKNOWN) {
                continue; // skip unknown for performance
            } else if (cell == CELL_OBSTACLE) {
                color = COLOR_OBSTACLE;
            } else if (cell == CELL_ROBOT) {
                continue; // robot rendered later at center
            } else { // free or others
                double c = cost_map[gy][gx];
                color = cost_to_color(c);
            }

            if (color != last_color) {
                wb_display_set_color(display, color);
                last_color = color;
            }

            int screen_x = (int)((gx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((gy - grid_start_y + 1) * scale);
            wb_display_fill_rectangle(display, screen_x, screen_y, cell_size, cell_size);
        }
    }

    // Draw path (cheap early-out)
    if (current_path && current_path->count > 0) {
        // set color once for path lines
        wb_display_set_color(display, 0x00FF00);
        for (int i = 0; i < current_path->count - 1; ++i) {
            GridNode* n1 = &current_path->nodes[i];
            GridNode* n2 = &current_path->nodes[i + 1];
            int x1 = (int)((n1->x - grid_start_x) * scale);
            int y1 = DISPLAY_HEIGHT - (int)((n1->y - grid_start_y + 1) * scale);
            int x2 = (int)((n2->x - grid_start_x) * scale);
            int y2 = DISPLAY_HEIGHT - (int)((n2->y - grid_start_y + 1) * scale);
            wb_display_draw_line(display, x1, y1, x2, y2);
        }

        // draw nodes (reuse a single color change)
        wb_display_set_color(display, 0x0000FF);
        for (int i = 0; i < current_path->count; ++i) {
            GridNode* n = &current_path->nodes[i];
            int x = (int)((n->x - grid_start_x) * scale);
            int y = DISPLAY_HEIGHT - (int)((n->y - grid_start_y + 1) * scale);
            if (i == current_waypoint) {
                wb_display_set_color(display, 0xFF00FF);
                wb_display_fill_oval(display, x - 3, y - 3, 6, 6);
                wb_display_set_color(display, 0x0000FF); // restore
            } else {
                wb_display_draw_oval(display, x - 2, y - 2, 4, 4);
            }
        }
    }

    // Frontiers: update every N calls (keeps same behaviour).
    static FrontierCentroid frontiers[MAX_FRONTIERS];
    static int num_frontiers = 0;
    static int frontier_update_counter = 0;
    if (++frontier_update_counter >= 20) {
        frontier_update_counter = 0;
        num_frontiers = find_frontier_centroids(frontiers, MAX_FRONTIERS);
    }

    if (num_frontiers > 0) {
        double min_score = 1e9, max_score = -1e9;
        for (int i = 0; i < num_frontiers; ++i) {
            if (frontiers[i].score < min_score) min_score = frontiers[i].score;
            if (frontiers[i].score > max_score) max_score = frontiers[i].score;
        }
        if (min_score == 1e9) { min_score = 0.0; max_score = 1.0; }

        // draw frontier markers
        for (int i = 0; i < num_frontiers; ++i) {
            int fx = frontiers[i].x;
            int fy = frontiers[i].y;
            if (fx < grid_start_x || fx > grid_end_x || fy < grid_start_y || fy > grid_end_y) continue;

            int screen_x = (int)((fx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * scale);

            unsigned int fcolor = frontier_score_to_color(frontiers[i].score, min_score, max_score);
            wb_display_set_color(display, fcolor);
            int marker_half = 6;
            wb_display_fill_rectangle(display, screen_x - 1, screen_y - marker_half, 2, marker_half * 2);
            wb_display_fill_rectangle(display, screen_x - marker_half, screen_y - 1, marker_half * 2, 2);
        }

        // highlight best frontier (index 0)
        int fx = frontiers[0].x;
        int fy = frontiers[0].y;
        if (fx >= grid_start_x && fx <= grid_end_x &&
            fy >= grid_start_y && fy <= grid_end_y) {
            int screen_x = (int)((fx - grid_start_x) * scale);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * scale);
            wb_display_set_color(display, 0xFFFF00);
            int thick = 2, size = 8;
            wb_display_fill_rectangle(display, screen_x - thick, screen_y - size, thick*2, size*2);
            wb_display_fill_rectangle(display, screen_x - size, screen_y - thick, size*2, thick*2);
            wb_display_draw_oval(display, screen_x - size/2, screen_y - size/2, size, size);
        }
    }

    // IR status (keeps existing layout but only sets font once globally)
    display_ir_status(frame_count);

    // draw robot at center (single color calls)
    int center_x = DISPLAY_WIDTH / 2;
    int center_y = DISPLAY_HEIGHT / 2;
    int robot_size = (int)(scale * 6);
    wb_display_set_color(display, COLOR_ROBOT);
    wb_display_fill_rectangle(display,
                              center_x - robot_size/2,
                              center_y - robot_size/2,
                              robot_size, robot_size);

    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    double robot_theta = atan2(orientation[3], orientation[0]);
    int arrow_length = robot_size;
    int arrow_end_x = center_x + (int)(arrow_length * cos(robot_theta));
    int arrow_end_y = center_y - (int)(arrow_length * sin(robot_theta));
    wb_display_set_color(display, 0xFFFF00);
    wb_display_draw_line(display, center_x, center_y, arrow_end_x, arrow_end_y);

    // Info text (we set font once earlier)
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "A* Pathfinding + Frontiers", 6, 5);

    char info[256];
    sprintf(info, "Pos: (%d,%d) Waypoint: %d/%d",
            robot_gx, robot_gy, current_waypoint,
            current_path ? current_path->count : 0);
    wb_display_draw_text(display, info, 6, 20);

    sprintf(info, "Frontiers: %d", (int)(num_frontiers));
    wb_display_draw_text(display, info, 6, 35);

    // Legend (set color then text)
    int legend_x = DISPLAY_WIDTH - 110;
    int legend_y = 6;
    int box = 10;
    wb_display_set_color(display, cost_to_color(FREE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "free", legend_x + 14, legend_y + 9);

    wb_display_set_color(display, cost_to_color(OBSTACLE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y + 14, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "obstacle", legend_x + 14, legend_y + 23);

    wb_display_set_color(display, COLOR_UNKNOWN);
    wb_display_fill_rectangle(display, legend_x, legend_y + 28, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "unknown", legend_x + 14, legend_y + 37);

    wb_display_set_color(display, 0xFFFF00);
    wb_display_fill_rectangle(display, legend_x, legend_y + 42, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "target", legend_x + 14, legend_y + 51);

    wb_display_set_color(display, 0x00FF00);
    wb_display_fill_rectangle(display, legend_x, legend_y + 56, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "path", legend_x + 14, legend_y + 65);
}
































// ============ MAIN ================

int main(int argc, char **argv) {
    wb_robot_init();
    
    robot_node = wb_supervisor_node_get_self();
    
    display = wb_robot_get_device("display");
    if (!display) {
        printf("Error: No display device found!\n");
        return 1;
    }
    
    lidar = wb_robot_get_device("laser");
    if (!lidar) {
        printf("Error: No lidar device found!\n");
        return 1;
    }
    wb_lidar_enable(lidar, TIME_STEP);
    
    init_motors();
    init_ir_sensors();
    init_grid();
   
    
    int frame_count = 0;
    
    
    rotate_drive(2, 0.3, 0.287, 0.0825);
    
    
    
    
    
    
    
    // ============= MAIN LOOOOOP =============
    
    while (wb_robot_step(TIME_STEP) != -1) {
        frame_count++;
        
        
        
        process_lidar();
        
        if (frame_count % 10 == 0) {
            decay_counters();
            filter_connected_components(MIN_BLOB_SIZE);
        }
        
        if (frame_count % 100 == 0) {
            //clear_nearby_obstacles(NEARBY_RADIUS_INNER, NEARBY_RADIUS_OUTER);
        }
        
        generate_cost_map();
        
        if (current_path) {
            bool path_complete = follow_path();
            //bool path_complete = 0;
            
            if (path_complete) {
                printf("Goal reached! Searching for new frontier...\n");
                destroy_grid_path(current_path);
                current_path = NULL;
            }
            
            if (frame_count % 100 == 0) {
                destroy_grid_path(current_path);
                current_path = NULL;
            }
        }
        
        if (!current_path && frame_count % 50 == 0) {
            path_update_counter++;
            update_path_to_frontier();
        }
        
        if (frame_count % 5 == 0) {
            render_display_with_path(frame_count);
        }
    }
    
    if (current_path) {
        destroy_grid_path(current_path);
    }
    
    wb_robot_cleanup();
    return 0;
}
