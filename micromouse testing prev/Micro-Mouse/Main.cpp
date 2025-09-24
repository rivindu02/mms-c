#include <iostream>
#include <string>
#include <queue>
#include <array>
#include "API.h"

void log(const std::string& text) {
    std::cerr << text << std::endl;
}


class MicromouseSolver {
private:
    static const int MAZE_SIZE = 16;
    
    struct Cell {
        bool visited = false;
        bool wallNorth = false;
        bool wallEast = false;
        bool wallSouth = false;
        bool wallWest = false;
        int distance = 999999;
    };

    enum Direction {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3
    };

    std::array<std::array<Cell, MAZE_SIZE>, MAZE_SIZE> maze;
    int currentX = 0;
    int currentY = 0;
    Direction currentDir = NORTH;
    
    const int targetX1 = 7;
    const int targetY1 = 7;
    const int targetX2 = 8;
    const int targetY2 = 8;

    // Update wall information in the maze representation
    void updateWalls() {
        bool frontWall = API::wallFront();
        bool rightWall = API::wallRight();
        bool leftWall = API::wallLeft();
        
        // Update current cell walls
        switch (currentDir) {
            case NORTH:
                maze[currentX][currentY].wallNorth = frontWall;
                maze[currentX][currentY].wallEast = rightWall;
                maze[currentX][currentY].wallWest = leftWall;
                
                // Update adjacent cells
                if (currentY < MAZE_SIZE - 1) {
                    maze[currentX][currentY + 1].wallSouth = frontWall;
                    if (frontWall) API::setWall(currentX, currentY, 'n');
                }
                if (currentX < MAZE_SIZE - 1) {
                    maze[currentX + 1][currentY].wallWest = rightWall;
                    if (rightWall) API::setWall(currentX, currentY, 'e');
                }
                if (currentX > 0) {
                    maze[currentX - 1][currentY].wallEast = leftWall;
                    if (leftWall) API::setWall(currentX, currentY, 'w');
                }
                break;
                
            case EAST:
                maze[currentX][currentY].wallEast = frontWall;
                maze[currentX][currentY].wallSouth = rightWall;
                maze[currentX][currentY].wallNorth = leftWall;
                
                if (currentX < MAZE_SIZE - 1) {
                    maze[currentX + 1][currentY].wallWest = frontWall;
                    if (frontWall) API::setWall(currentX, currentY, 'e');
                }
                if (currentY > 0) {
                    maze[currentX][currentY - 1].wallNorth = rightWall;
                    if (rightWall) API::setWall(currentX, currentY, 's');
                }
                if (currentY < MAZE_SIZE - 1) {
                    maze[currentX][currentY + 1].wallSouth = leftWall;
                    if (leftWall) API::setWall(currentX, currentY, 'n');
                }
                break;
                
            case SOUTH:
                maze[currentX][currentY].wallSouth = frontWall;
                maze[currentX][currentY].wallWest = rightWall;
                maze[currentX][currentY].wallEast = leftWall;
                
                if (currentY > 0) {
                    maze[currentX][currentY - 1].wallNorth = frontWall;
                    if (frontWall) API::setWall(currentX, currentY, 's');
                }
                if (currentX > 0) {
                    maze[currentX - 1][currentY].wallEast = rightWall;
                    if (rightWall) API::setWall(currentX, currentY, 'w');
                }
                if (currentX < MAZE_SIZE - 1) {
                    maze[currentX + 1][currentY].wallWest = leftWall;
                    if (leftWall) API::setWall(currentX, currentY, 'e');
                }
                break;
                
            case WEST:
                maze[currentX][currentY].wallWest = frontWall;
                maze[currentX][currentY].wallNorth = rightWall;
                maze[currentX][currentY].wallSouth = leftWall;
                
                if (currentX > 0) {
                    maze[currentX - 1][currentY].wallEast = frontWall;
                    if (frontWall) API::setWall(currentX, currentY, 'w');
                }
                if (currentY < MAZE_SIZE - 1) {
                    maze[currentX][currentY + 1].wallSouth = rightWall;
                    if (rightWall) API::setWall(currentX, currentY, 'n');
                }
                if (currentY > 0) {
                    maze[currentX][currentY - 1].wallNorth = leftWall;
                    if (leftWall) API::setWall(currentX, currentY, 's');
                }
                break;
        }
        
        // Visualize visited cells
        API::setColor(currentX, currentY, 'G');
        // Display distance in cell
        API::setText(currentX, currentY, std::to_string(maze[currentX][currentY].distance));
    }

    void floodFill() {
        std::queue<std::pair<int, int>> q;
        
        // Reset distances
        for (int i = 0; i < MAZE_SIZE; i++) {
            for (int j = 0; j < MAZE_SIZE; j++) {
                maze[i][j].distance = 999999;
            }
        }
        
        // Set target distances to 0
        maze[targetX1][targetY1].distance = 0;
        maze[targetX2][targetY2].distance = 0;
        q.push({targetX1, targetY1});
        q.push({targetX2, targetY2});
        
        while (!q.empty()) {
            auto [x, y] = q.front();
            q.pop();
            
            // Check all four directions
            if (!maze[x][y].wallNorth && y + 1 < MAZE_SIZE && 
                maze[x][y + 1].distance > maze[x][y].distance + 1) {
                maze[x][y + 1].distance = maze[x][y].distance + 1;
                q.push({x, y + 1});
            }
            if (!maze[x][y].wallEast && x + 1 < MAZE_SIZE && 
                maze[x + 1][y].distance > maze[x][y].distance + 1) {
                maze[x + 1][y].distance = maze[x][y].distance + 1;
                q.push({x + 1, y});
            }
            if (!maze[x][y].wallSouth && y > 0 && 
                maze[x][y - 1].distance > maze[x][y].distance + 1) {
                maze[x][y - 1].distance = maze[x][y].distance + 1;
                q.push({x, y - 1});
            }
            if (!maze[x][y].wallWest && x > 0 && 
                maze[x - 1][y].distance > maze[x][y].distance + 1) {
                maze[x - 1][y].distance = maze[x][y].distance + 1;
                q.push({x - 1, y});
            }
        }
    }

    Direction getNextMove() {
        int minDistance = 999999;
        Direction bestDir = currentDir;
        
        if (!maze[currentX][currentY].wallNorth && currentY + 1 < MAZE_SIZE) {
            if (maze[currentX][currentY + 1].distance < minDistance) {
                minDistance = maze[currentX][currentY + 1].distance;
                bestDir = NORTH;
            }
        }
        if (!maze[currentX][currentY].wallEast && currentX + 1 < MAZE_SIZE) {
            if (maze[currentX + 1][currentY].distance < minDistance) {
                minDistance = maze[currentX + 1][currentY].distance;
                bestDir = EAST;
            }
        }
        if (!maze[currentX][currentY].wallSouth && currentY > 0) {
            if (maze[currentX][currentY - 1].distance < minDistance) {
                minDistance = maze[currentX][currentY - 1].distance;
                bestDir = SOUTH;
            }
        }
        if (!maze[currentX][currentY].wallWest && currentX > 0) {
            if (maze[currentX - 1][currentY].distance < minDistance) {
                minDistance = maze[currentX - 1][currentY].distance;
                bestDir = WEST;
            }
        }
        
        return bestDir;
    }

public:
    void solve() {
        // Initialize
        if (API::wasReset()) {
            API::ackReset();
            currentX = 0;
            currentY = 0;
            currentDir = NORTH;
            
            for (int i = 0; i < MAZE_SIZE; i++) {
                for (int j = 0; j < MAZE_SIZE; j++) {
                    maze[i][j].visited = false;
                    maze[i][j].distance = 999999;
                }
            }
            maze[0][0].visited = true;
        }
        
        while (!(currentX == targetX1 && currentY == targetY1) && 
               !(currentX == targetX2 && currentY == targetY2)) {
            
            // Update wall information and visualize
            updateWalls();
            
            // Recalculate distances
            floodFill();
            
            // Get next move
            Direction nextDir = getNextMove();
            
            // Turn to face the right direction
            while (currentDir != nextDir) {
                API::turnRight();
                currentDir = static_cast<Direction>((currentDir + 1) % 4);
            }
            
            // Move forward
            API::moveForward();
            maze[currentX][currentY].visited = true;
            
            // Update position
            switch (currentDir) {
                case NORTH: currentY++; break;
                case EAST:  currentX++; break;
                case SOUTH: currentY--; break;
                case WEST:  currentX--; break;
            }
        }
        
        // Mark final cell as reached
        API::setColor(currentX, currentY, 'B');
    }
};


int main(int argc, char* argv[]) {
    log("Running...");
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");
    while (true) {
        if (!API::wallLeft()) {
            API::turnLeft();
        }
        while (API::wallFront()) {
            API::turnRight();
        }
        //API::moveForward();
        MicromouseSolver solver;
        solver.solve();
        return 0;
    }
}

