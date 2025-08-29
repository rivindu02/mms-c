// Main.c  —  Micromouse
// Build (Windows MinGW):
//   gcc -std=c11 -O3 -o myAlgorithm.exe Main.c API.c
//   // if DLL issues, add: -static -static-libgcc -static-libstdc++
//
// Notes:
//  * Uses API_* functions from API.c/API.h
//  * Acknowledges MMS reset at startup
//  * Floods from goal, exploration + return to start
//  * Post-exploration: computes & visualizes optimal path using only explored cells

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "API.h"

// ----------------- Config -----------------
#define MAZE_SIZE     16
#define MAX_DISTANCE  9999

// Directions
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

// ----------------- Types/Globals -----------------
typedef struct {
    int distance;
    int visited;         // 0/1
    int walls[4];        // N,E,S,W  (0 = open, 1 = wall)
    int visit_count;
} Cell;

static Cell maze[MAZE_SIZE][MAZE_SIZE];

static int robot_x = 0, robot_y = 0, robot_dir = NORTH;
static int goal_x1 = 7, goal_y1 = 7, goal_x2 = 8, goal_y2 = 8;
static int center_reached = 0;
static int returned_to_start = 0;

static const int dx[4] = { 0, 1, 0,-1 };
static const int dy[4] = { 1, 0,-1, 0 };

static int exploration_steps = 0;
static int theoretical_minimum = 0;

// ----------------- Small utils -----------------
static void logf_(const char* s) { fprintf(stderr, "%s\n", s); fflush(stderr); }
static void logi_(const char* s, int v) { fprintf(stderr, "%s%d\n", s, v); fflush(stderr); }

// simple int->string
static void itoa10(int v, char* buf, int buflen) { snprintf(buf, buflen, "%d", v); }

// ----------------- Init -----------------
static void initializeMaze(void) {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].distance = MAX_DISTANCE;
            maze[x][y].visited = 0;
            maze[x][y].visit_count = 0;
            for (int i = 0; i < 4; i++) maze[x][y].walls[i] = 0;
        }
    }
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i][0].walls[SOUTH] = 1;
        maze[i][MAZE_SIZE-1].walls[NORTH] = 1;
        maze[0][i].walls[WEST] = 1;
        maze[MAZE_SIZE-1][i].walls[EAST] = 1;
    }
    robot_x = 0; robot_y = 0; robot_dir = NORTH;
    maze[0][0].visited = 1;
    maze[0][0].visit_count = 1;

    exploration_steps = 0;
    theoretical_minimum = 0;

    logf_("[DEBUG] Maze initialized");
}

// ----------------- BFS queue (C) -----------------
typedef struct { int x, y; } P2;
#define QMAX (MAZE_SIZE*MAZE_SIZE + 16)
typedef struct {
    P2 q[QMAX];
    int head, tail;
} Q;

static void q_init(Q* s){ s->head = s->tail = 0; }
static int  q_empty(Q* s){ return s->head == s->tail; }
static void q_push(Q* s, P2 v){ if (s->tail < QMAX) s->q[s->tail++] = v; }
static P2  q_pop(Q* s){ return s->q[s->head++]; }

// ----------------- Flood fill (from goal or start) -----------------
static void FloodFill(void) {
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            maze[x][y].distance = MAX_DISTANCE;

    if (!center_reached) {
        maze[goal_x1][goal_y1].distance = 0;
        maze[goal_x2][goal_y1].distance = 0;
        maze[goal_x1][goal_y2].distance = 0;
        maze[goal_x2][goal_y2].distance = 0;
    } else {
        maze[0][0].distance = 0;
    }

    Q q; q_init(&q);
    if (!center_reached) {
        q_push(&q, (P2){goal_x1, goal_y1});
        q_push(&q, (P2){goal_x2, goal_y1});
        q_push(&q, (P2){goal_x1, goal_y2});
        q_push(&q, (P2){goal_x2, goal_y2});
    } else {
        q_push(&q, (P2){0,0});
    }

    int updates = 0;

    while (!q_empty(&q)) {
        P2 p = q_pop(&q);
        int x = p.x, y = p.y;

        for (int dir = 0; dir < 4; dir++) {
            if (maze[x][y].walls[dir]) continue;

            int nx = x + dx[dir];
            int ny = y + dy[dir];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

            int nd = maze[x][y].distance + 1;
            if (nd < maze[nx][ny].distance) {
                maze[nx][ny].distance = nd;
                q_push(&q, (P2){nx, ny});
                updates++;
            }
        }
    }
    fprintf(stderr, "[DEBUG] flood fill: %d updates\n", updates); fflush(stderr);
}

// ----------------- Direction choice -----------------
static int getDirection(void) {
    int best_dir = robot_dir;
    int min_distance = MAX_DISTANCE;
    int min_visits = 999;
    int found_unvisited = 0;

    int priority[4];
    priority[0] = robot_dir;
    priority[1] = (robot_dir + 1) & 3;
    priority[2] = (robot_dir + 3) & 3;
    priority[3] = (robot_dir + 2) & 3;

    // pass 1: prefer unvisited neighbors
    for (int p = 0; p < 4; p++) {
        int dir = priority[p];
        if (maze[robot_x][robot_y].walls[dir]) continue;
        int nx = robot_x + dx[dir], ny = robot_y + dy[dir];
        if (nx<0||nx>=MAZE_SIZE||ny<0||ny>=MAZE_SIZE) continue;

        if (maze[nx][ny].visit_count == 0) {
            found_unvisited = 1;
            if (maze[nx][ny].distance < min_distance) {
                min_distance = maze[nx][ny].distance;
                best_dir = dir;
            }
        }
    }

    if (!found_unvisited) {
        for (int p = 0; p < 4; p++) {
            int dir = priority[p];
            if (maze[robot_x][robot_y].walls[dir]) continue;
            int nx = robot_x + dx[dir], ny = robot_y + dy[dir];
            if (nx<0||nx>=MAZE_SIZE||ny<0||ny>=MAZE_SIZE) continue;

            int nd = maze[nx][ny].distance;
            int nv = maze[nx][ny].visit_count;

            if (nd < min_distance ||
               (nd == min_distance && nv < min_visits) ||
               (nd == min_distance && nv == min_visits && dir == robot_dir)) {
                min_distance = nd;
                min_visits = nv;
                best_dir = dir;
            }
        }
    }
    return best_dir;
}

// ----------------- Sensors → map -----------------
static void updateWalls(void) {
    int front = API_wallFront();
    int left  = API_wallLeft();
    int right = API_wallRight();

    if (front) {
        maze[robot_x][robot_y].walls[robot_dir] = 1;
        int nx = robot_x + dx[robot_dir], ny = robot_y + dy[robot_dir];
        if (nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE)
            maze[nx][ny].walls[(robot_dir+2)&3] = 1;
    }
    if (left) {
        int ld = (robot_dir + 3) & 3;
        maze[robot_x][robot_y].walls[ld] = 1;
        int nx = robot_x + dx[ld], ny = robot_y + dy[ld];
        if (nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE)
            maze[nx][ny].walls[(ld+2)&3] = 1;
    }
    if (right) {
        int rd = (robot_dir + 1) & 3;
        maze[robot_x][robot_y].walls[rd] = 1;
        int nx = robot_x + dx[rd], ny = robot_y + dy[rd];
        if (nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE)
            maze[nx][ny].walls[(rd+2)&3] = 1;
    }

    maze[robot_x][robot_y].visited = 1;
    maze[robot_x][robot_y].visit_count++;

    fprintf(stderr, "[DEBUG] Walls updated at (%d,%d) F:%d L:%d R:%d\n",
            robot_x, robot_y, front, left, right);
    fflush(stderr);
}

// ----------------- Motion helpers -----------------
static void turnToDirection(int target_dir) {
    while (robot_dir != target_dir) {
        int diff = (target_dir - robot_dir + 4) & 3;
        if (diff == 1) {
            API_turnRight();
            robot_dir = (robot_dir + 1) & 3;
        } else if (diff == 3) {
            API_turnLeft();
            robot_dir = (robot_dir + 3) & 3;
        } else { // 2
            API_turnRight(); API_turnRight();
            robot_dir = (robot_dir + 2) & 3;
        }
    }
}

static int moveForward_(void) {
    if (API_wallFront()) {
        logf_("[DEBUG] Front wall detected, cannot move");
        return 0;
    }
    if (!API_moveForward()) return 0; // crash handling if needed

    robot_x += dx[robot_dir];
    robot_y += dy[robot_dir];
    if (robot_x < 0) robot_x = 0;
    if (robot_x >= MAZE_SIZE) robot_x = MAZE_SIZE-1;
    if (robot_y < 0) robot_y = 0;
    if (robot_y >= MAZE_SIZE) robot_y = MAZE_SIZE-1;
    return 1;
}

static int isAtGoal(void) {
    if (!center_reached) {
        return ((robot_x==goal_x1||robot_x==goal_x2) &&
                (robot_y==goal_y1||robot_y==goal_y2));
    } else {
        return (robot_x==0 && robot_y==0);
    }
}

// ----------------- Path analysis (explored-only) -----------------
static void calculateOptimalPathFromExploredAreas(void) {
    fprintf(stderr, "\n[INFO] Calculating optimal path (explored cells only)...\n");

    for (int x=0;x<MAZE_SIZE;x++)
        for (int y=0;y<MAZE_SIZE;y++)
            maze[x][y].distance = MAX_DISTANCE;

    int goal_found = 0;
    if (maze[goal_x1][goal_y1].visited) { maze[goal_x1][goal_y1].distance = 0; goal_found = 1; }
    if (maze[goal_x2][goal_y1].visited) { maze[goal_x2][goal_y1].distance = 0; goal_found = 1; }
    if (maze[goal_x1][goal_y2].visited) { maze[goal_x1][goal_y2].distance = 0; goal_found = 1; }
    if (maze[goal_x2][goal_y2].visited) { maze[goal_x2][goal_y2].distance = 0; goal_found = 1; }

    if (!goal_found) {
        fprintf(stderr, "ERROR: No goal cells were visited during exploration!\n");
        return;
    }

    Q q; q_init(&q);
    if (maze[goal_x1][goal_y1].distance==0) q_push(&q,(P2){goal_x1,goal_y1});
    if (maze[goal_x2][goal_y1].distance==0) q_push(&q,(P2){goal_x2,goal_y1});
    if (maze[goal_x1][goal_y2].distance==0) q_push(&q,(P2){goal_x1,goal_y2});
    if (maze[goal_x2][goal_y2].distance==0) q_push(&q,(P2){goal_x2,goal_y2});

    int updates = 0;

    while (!q_empty(&q)) {
        P2 p = q_pop(&q);
        int x = p.x, y = p.y;

        for (int dir=0; dir<4; dir++) {
            if (maze[x][y].walls[dir]) continue;
            int nx = x + dx[dir], ny = y + dy[dir];
            if (nx<0||nx>=MAZE_SIZE||ny<0||ny>=MAZE_SIZE) continue;
            if (!maze[nx][ny].visited) continue;

            int nd = maze[x][y].distance + 1;
            if (nd < maze[nx][ny].distance) {
                maze[nx][ny].distance = nd;
                q_push(&q,(P2){nx,ny});
                updates++;
            }
        }
    }

    theoretical_minimum = maze[0][0].distance;
    fprintf(stderr, "[PATH] optimal steps (explored): %d | updates: %d\n",
            theoretical_minimum, updates);
    fflush(stderr);
}

static void analyzeMazePerformance(void) {
    int cells_visited = 0, total_cells = MAZE_SIZE*MAZE_SIZE;
    for (int x=0;x<MAZE_SIZE;x++)
        for (int y=0;y<MAZE_SIZE;y++)
            if (maze[x][y].visited) cells_visited++;

    double eff = (double)cells_visited * 100.0 / (double)total_cells;

    fprintf(stderr, "\n=== PERFORMANCE ANALYSIS ===\n");
    fprintf(stderr, "Exploration Efficiency: %.1f%% (%d/%d)\n",
            eff, cells_visited, total_cells);
    fprintf(stderr, "Total Exploration Steps: %d\n", exploration_steps);

    if (theoretical_minimum < MAX_DISTANCE) {
        fprintf(stderr, "Best Path Through Explored Areas: %d steps\n", theoretical_minimum);
    } else {
        fprintf(stderr, "No path found through explored areas.\n");
    }

    fprintf(stderr, "============================\n");
    fflush(stderr);
}

static void printOptimalDistanceMap(void) {
    fprintf(stderr, "\n[MAP] OPTIMAL DISTANCES (explored only)\n    ");
    for (int x=0;x<MAZE_SIZE;x++) fprintf(stderr, "%3d", x);
    fprintf(stderr, "\n");
    for (int y=MAZE_SIZE-1;y>=0;y--) {
        fprintf(stderr, "%3d", y);
        for (int x=0;x<MAZE_SIZE;x++) {
            if (!maze[x][y].visited) fprintf(stderr, "  -");
            else if (maze[x][y].distance == MAX_DISTANCE) fprintf(stderr, "  ∞");
            else fprintf(stderr, "%3d", maze[x][y].distance);
        }
        fprintf(stderr, "\n");
    }
    fflush(stderr);
}

static void visualizeOptimalPathThroughExploredAreas(void) {
    if (theoretical_minimum >= MAX_DISTANCE) {
        fprintf(stderr, "Cannot visualize path: none found.\n");
        return;
    }

    API_clearAllColor();

    if (maze[goal_x1][goal_y1].visited) API_setColor(goal_x1, goal_y1, 'R');
    if (maze[goal_x2][goal_y1].visited) API_setColor(goal_x2, goal_y1, 'R');
    if (maze[goal_x1][goal_y2].visited) API_setColor(goal_x1, goal_y2, 'R');
    if (maze[goal_x2][goal_y2].visited) API_setColor(goal_x2, goal_y2, 'R');

    int x = 0, y = 0;
    if (!maze[x][y].visited) {
        fprintf(stderr, "ERROR: start not marked visited!\n");
        return;
    }

    API_setColor(x, y, 'G'); // start
    int steps = 0, max_steps = theoretical_minimum + 5;

    while (!((x==goal_x1||x==goal_x2) && (y==goal_y1||y==goal_y2)) && steps < max_steps) {
        int best_dir = -1;
        int curd = maze[x][y].distance;
        int nextx = x, nexty = y;

        for (int dir=0; dir<4; dir++) {
            if (maze[x][y].walls[dir]) continue;
            int nx = x + dx[dir], ny = y + dy[dir];
            if (nx<0||nx>=MAZE_SIZE||ny<0||ny>=MAZE_SIZE) continue;
            if (!maze[nx][ny].visited) continue;
            if (maze[nx][ny].distance < curd && maze[nx][ny].distance < MAX_DISTANCE) {
                curd = maze[nx][ny].distance;
                best_dir = dir;
                nextx = nx; nexty = ny;
            }
        }

        if (best_dir < 0) {
            fprintf(stderr, "Blocked at (%d,%d)\n", x, y);
            break;
        }

        x = nextx; y = nexty; steps++;
        if (!((x==goal_x1||x==goal_x2) && (y==goal_y1||y==goal_y2)))
            API_setColor(x, y, 'B');
    }

    fprintf(stderr, "Traced path length: %d (theoretical %d)\n", steps, theoretical_minimum);
    fflush(stderr);
}

static void showOptimalDistances(void) {
    char buf[16];
    for (int x=0;x<MAZE_SIZE;x++) {
        for (int y=0;y<MAZE_SIZE;y++) {
            if (maze[x][y].visited && maze[x][y].distance < MAX_DISTANCE &&
                maze[x][y].distance < 100) {
                itoa10(maze[x][y].distance, buf, sizeof(buf));
                API_setText(x, y, buf);
            }
        }
    }
}

// ----------------- Main exploration loop -----------------
static void Exploration(void) {
    logf_("[DEBUG] Starting exploration");
    exploration_steps = 0;
    int steps = 0, max_steps = 1000;

    while (steps < max_steps && (!center_reached || !returned_to_start)) {
        fprintf(stderr, "[DEBUG] Step %d: Robot at (%d,%d)\n", steps, robot_x, robot_y);
        fflush(stderr);

        updateWalls();
        FloodFill();

        if (isAtGoal()) {
            if (!center_reached) {
                logf_("[DEBUG] CENTER REACHED! Switching to return mode");
                center_reached = 1;
                for (int x=0;x<MAZE_SIZE;x++)
                    for (int y=0;y<MAZE_SIZE;y++)
                        maze[x][y].visit_count = 0;
                API_setColor(robot_x, robot_y, 'G');
            } else {
                logf_("[DEBUG] RETURNED TO START! Exploration complete!");
                returned_to_start = 1;
                API_setColor(robot_x, robot_y, 'G');
                break;
            }
        }

        int ndir = getDirection();
        fprintf(stderr, "[DEBUG] direction: %d\n", ndir); fflush(stderr);

        turnToDirection(ndir);
        if (moveForward_()) {
            exploration_steps++;
        } else {
            // try alternatives
            int moved = 0;
            for (int alt=0; alt<4 && !moved; alt++) {
                if (alt == ndir) continue;
                if (maze[robot_x][robot_y].walls[alt]) continue;
                int nx = robot_x + dx[alt], ny = robot_y + dy[alt];
                if (nx<0||nx>=MAZE_SIZE||ny<0||ny>=MAZE_SIZE) continue;
                turnToDirection(alt);
                if (moveForward_()) { exploration_steps++; moved = 1; }
            }
            if (!moved) { logf_("[DEBUG] All directions blocked!"); break; }
        }

        if (center_reached) API_setColor(robot_x, robot_y, 'B');
        else                API_setColor(robot_x, robot_y, 'Y');

        steps++;
    }

    fprintf(stderr, "[DEBUG] Exploration completed in %d moves\n", exploration_steps);
    fflush(stderr);
}

// ----------------- main -----------------
int main(void) {
    API_ackReset(); // MMS handshake

    logf_("[DEBUG] === ULTIMATE MICROMOUSE (C) ===");
    logf_("[DEBUG] Based on the algorithms");
    logf_("[DEBUG] WITH PERFECT PATH ANALYSIS SYSTEM");

    initializeMaze();
    Exploration();

    if (center_reached && returned_to_start) {
        logf_("[DEBUG] Exploration successful! Starting path analysis...");
        calculateOptimalPathFromExploredAreas();
        analyzeMazePerformance();
        printOptimalDistanceMap();
        visualizeOptimalPathThroughExploredAreas();
        showOptimalDistances();

        fprintf(stderr, "Optimal steps (explored): %d | exploration moves: %d\n",
                theoretical_minimum, exploration_steps);
        fflush(stderr);
    } else {
        logf_("[DEBUG] Exploration incomplete - path analysis not available");
    }
    return 0;
}
