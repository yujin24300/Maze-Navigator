# Maze-Navigator 🧭
This project implements a 3D maze navigation system with smooth camera control, wall collision prevention, A* pathfinding, and automatic path-following.  
The program displays both a main camera (left screen) and a top-down minimap (right screen), ensuring consistent camera orientation in both views.

<br>

## 📸 Preview
![Image](https://github.com/user-attachments/assets/83bd48b3-a7c3-4f21-ad82-d72d2b2d5eff)

<br>

## ✨ Features
### 🎮 Controls
- **A / D keys** — Rotate camera left/right (affects both main view and minimap camera)
- **W / S keys** — Move forward/backward, with wall collision prevention
- **Q key** — Use A* algorithm to find the shortest path to the goal and visualize it as a red line
- **SpaceBar** — Automatically move along the found path with smooth rotation at corners
### 🧠 A* Pathfinding Overview
The **A\*** algorithm finds the shortest path from the start node to the goal node by minimizing:<div align="center">
f(n) = g(n) + h(n)
</div>

- **g(n)** — Actual cost from the start node to the current node  
- **h(n)** — Estimated cost from the current node to the goal (Manhattan distance in this case)  
- **f(n)** — Total estimated cost

<br>

## 🛠 Implementation Details

### 1. Camera Rotation (A/D keys)
- Used a `cameraYaw` variable to store rotation angle
- Converted angle to radians and updated `viewDirection`
- Both main view and minimap camera are rotated simultaneously

### 2. Wall Collision Prevention
- Implemented `isCollision()` to check if the target position is a wall
- Movement is blocked if collision is detected

### 3. Shortest Path Search (Q key)
- Runs `Astar()` to compute the shortest path from current position to the goal
- Priority queue (Open List) for efficient smallest f-value extraction
- Closed List to store visited nodes
- Nodes store `(x, y, g, h, f, parent)`

### 4. Path Visualization
- Draw red cubes along the computed path
- Adjust size, position, and rotation for each segment

### 5. Automatic Path Following (SpaceBar)
- Generated multiple intermediate points between key path nodes
- Used `Smoothstep` to create smooth turns at corners
- Controlled movement states: STOPPED → MOVING → PAUSED
- Movement resumes or restarts depending on state

<br>

## 🚀 Troubleshooting

### 1. Incorrect Pathfinding
- **Issue:** Initial A* implementation used simple vectors for open/closed lists without ordering by priority
- **Effect:** Sometimes failed to reach the goal in complex mazes
- **Fix:** Switched Open List to **priority queue** and added `parent` attribute to Node for proper backtracking

### 2. Multiple Q Key Calls
- **Issue:** Pressing `Q` quickly triggered multiple A* executions  
- **Effect:** Displayed duplicated path outputs
- **Fix:** Added logic to ensure a single execution per key press

### 3. Automatic Path Following Glitches
- **Issue:** Early implementation set `cameraPos` directly without generating intermediate points  
- **Effect:** Camera flickered or moved backward unexpectedly
- **Fix:** Precomputed and stored intermediate positions, directions, and angles, then iterated smoothly

### 4. SpaceBar Input Handling
- **Issue:** Short key presses were processed multiple times  
- **Effect:** State changes occurred too quickly, causing incorrect behavior
- **Fix:** Used `GetAsyncKeyState` and `spacePressed` flag to handle one press at a time
