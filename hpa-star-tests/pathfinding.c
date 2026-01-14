#include "pathfinding.h"
#include "raylib.h"
#include <stdlib.h>

// State
Entrance entrances[MAX_ENTRANCES];
int entranceCount = 0;
GraphEdge graphEdges[MAX_EDGES];
int graphEdgeCount = 0;

// Adjacency list for fast edge lookup: adjList[node][i] gives edge index
// adjListCount[node] gives number of edges for that node
static int adjList[MAX_ENTRANCES][MAX_EDGES_PER_NODE];
static int adjListCount[MAX_ENTRANCES];
Point path[MAX_PATH];
int pathLength = 0;
int nodesExplored = 0;
double lastPathTime = 0.0;
double hpaAbstractTime = 0.0;
double hpaRefinementTime = 0.0;
Point startPos = {-1, -1};
Point goalPos = {-1, -1};
AStarNode nodeData[MAX_GRID_HEIGHT][MAX_GRID_WIDTH];
bool chunkDirty[MAX_CHUNKS_Y][MAX_CHUNKS_X];

// HPA* abstract graph search state
AbstractNode abstractNodes[MAX_ABSTRACT_NODES];
int abstractPath[MAX_ENTRANCES + 2];
int abstractPathLength = 0;

// Binary heap for priority queue (used in abstract graph search)
typedef struct {
    int* nodes;      // Array of node indices
    int size;        // Current number of elements
    int capacity;    // Max capacity
} BinaryHeap;

static BinaryHeap heap;
static int heapStorage[MAX_ABSTRACT_NODES];

static void HeapInit(void) {
    heap.nodes = heapStorage;
    heap.size = 0;
    heap.capacity = MAX_ABSTRACT_NODES;
}

static void HeapSwap(int i, int j) {
    int temp = heap.nodes[i];
    heap.nodes[i] = heap.nodes[j];
    heap.nodes[j] = temp;
}

static void HeapBubbleUp(int idx) {
    while (idx > 0) {
        int parent = (idx - 1) / 2;
        if (abstractNodes[heap.nodes[idx]].f < abstractNodes[heap.nodes[parent]].f) {
            HeapSwap(idx, parent);
            idx = parent;
        } else {
            break;
        }
    }
}

static void HeapBubbleDown(int idx) {
    while (1) {
        int left = 2 * idx + 1;
        int right = 2 * idx + 2;
        int smallest = idx;
        
        if (left < heap.size && abstractNodes[heap.nodes[left]].f < abstractNodes[heap.nodes[smallest]].f) {
            smallest = left;
        }
        if (right < heap.size && abstractNodes[heap.nodes[right]].f < abstractNodes[heap.nodes[smallest]].f) {
            smallest = right;
        }
        
        if (smallest != idx) {
            HeapSwap(idx, smallest);
            idx = smallest;
        } else {
            break;
        }
    }
}

static void HeapPush(int node) {
    if (heap.size >= heap.capacity) return;
    heap.nodes[heap.size] = node;
    HeapBubbleUp(heap.size);
    heap.size++;
}

static int HeapPop(void) {
    if (heap.size == 0) return -1;
    int result = heap.nodes[0];
    heap.size--;
    if (heap.size > 0) {
        heap.nodes[0] = heap.nodes[heap.size];
        HeapBubbleDown(0);
    }
    return result;
}



// Movement direction mode
bool use8Dir = true;

int Heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// 8-directional heuristic (Chebyshev/diagonal distance)
static int Heuristic8Dir(int x1, int y1, int x2, int y2) {
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    int minD = dx < dy ? dx : dy;
    int maxD = dx > dy ? dx : dy;
    return 10 * maxD + 4 * minD;
}

void MarkChunkDirty(int cellX, int cellY) {
    int cx = cellX / chunkWidth;
    int cy = cellY / chunkHeight;
    if (cx >= 0 && cx < chunksX && cy >= 0 && cy < chunksY) {
        chunkDirty[cy][cx] = true;
        needsRebuild = true;
    }
}

static void AddEntrance(int x, int y, int chunk1, int chunk2) {
    if (entranceCount < MAX_ENTRANCES)
        entrances[entranceCount++] = (Entrance){x, y, chunk1, chunk2};
}

static void AddEntrancesForRun(int startX, int startY, int length, int horizontal, int chunk1, int chunk2) {
    int remaining = length, pos = 0;
    while (remaining > 0) {
        int segLen = (remaining > MAX_ENTRANCE_WIDTH) ? MAX_ENTRANCE_WIDTH : remaining;
        int mid = pos + segLen / 2;
        int ex = horizontal ? startX + mid : startX;
        int ey = horizontal ? startY : startY + mid;
        AddEntrance(ex, ey, chunk1, chunk2);
        pos += segLen;
        remaining -= segLen;
    }
}

void BuildEntrances(void) {
    entranceCount = 0;
    // Horizontal borders (between rows of chunks)
    for (int cy = 0; cy < chunksY - 1; cy++) {
        for (int cx = 0; cx < chunksX; cx++) {
            int borderY = (cy + 1) * chunkHeight;
            int startX = cx * chunkWidth;
            int chunk1 = cy * chunksX + cx;
            int chunk2 = (cy + 1) * chunksX + cx;
            int runStart = -1;
            for (int i = 0; i < chunkWidth; i++) {
                int x = startX + i;
                bool open = (grid[borderY - 1][x] == CELL_WALKABLE && grid[borderY][x] == CELL_WALKABLE);
                if (open && runStart < 0) runStart = i;
                else if (!open && runStart >= 0) {
                    AddEntrancesForRun(startX + runStart, borderY, i - runStart, 1, chunk1, chunk2);
                    runStart = -1;
                }
            }
            if (runStart >= 0)
                AddEntrancesForRun(startX + runStart, borderY, chunkWidth - runStart, 1, chunk1, chunk2);
        }
    }
    // Vertical borders (between columns of chunks)
    for (int cy = 0; cy < chunksY; cy++) {
        for (int cx = 0; cx < chunksX - 1; cx++) {
            int borderX = (cx + 1) * chunkWidth;
            int startY = cy * chunkHeight;
            int chunk1 = cy * chunksX + cx;
            int chunk2 = cy * chunksX + (cx + 1);
            int runStart = -1;
            for (int i = 0; i < chunkHeight; i++) {
                int y = startY + i;
                bool open = (grid[y][borderX - 1] == CELL_WALKABLE && grid[y][borderX] == CELL_WALKABLE);
                if (open && runStart < 0) runStart = i;
                else if (!open && runStart >= 0) {
                    AddEntrancesForRun(borderX, startY + runStart, i - runStart, 0, chunk1, chunk2);
                    runStart = -1;
                }
            }
            if (runStart >= 0)
                AddEntrancesForRun(borderX, startY + runStart, chunkHeight - runStart, 0, chunk1, chunk2);
        }
    }
    for (int cy = 0; cy < chunksY; cy++)
        for (int cx = 0; cx < chunksX; cx++)
            chunkDirty[cy][cx] = false;
    needsRebuild = false;
}

int AStarChunk(int sx, int sy, int gx, int gy, int minX, int minY, int maxX, int maxY) {
    for (int y = minY; y < maxY; y++)
        for (int x = minX; x < maxX; x++)
            nodeData[y][x] = (AStarNode){999999, 999999, -1, -1, false, false};

    nodeData[sy][sx].g = 0;
    if (use8Dir) {
        nodeData[sy][sx].f = Heuristic8Dir(sx, sy, gx, gy);
    } else {
        nodeData[sy][sx].f = Heuristic(sx, sy, gx, gy) * 10;
    }
    nodeData[sy][sx].open = true;

    int dx4[] = {0, 1, 0, -1};
    int dy4[] = {-1, 0, 1, 0};
    int dx8[] = {0, 1, 1, 1, 0, -1, -1, -1};
    int dy8[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    
    int* dx = use8Dir ? dx8 : dx4;
    int* dy = use8Dir ? dy8 : dy4;
    int numDirs = use8Dir ? 8 : 4;

    while (1) {
        int bestX = -1, bestY = -1, bestF = 999999;
        for (int y = minY; y < maxY; y++)
            for (int x = minX; x < maxX; x++)
                if (nodeData[y][x].open && nodeData[y][x].f < bestF) {
                    bestF = nodeData[y][x].f;
                    bestX = x;
                    bestY = y;
                }
        if (bestX < 0) return -1;
        if (bestX == gx && bestY == gy) return nodeData[gy][gx].g;
        nodeData[bestY][bestX].open = false;
        nodeData[bestY][bestX].closed = true;
        for (int i = 0; i < numDirs; i++) {
            int nx = bestX + dx[i], ny = bestY + dy[i];
            if (nx < minX || nx >= maxX || ny < minY || ny >= maxY) continue;
            if (grid[ny][nx] == CELL_WALL || nodeData[ny][nx].closed) continue;
            
            // Prevent corner cutting for diagonal movement
            if (use8Dir && dx[i] != 0 && dy[i] != 0) {
                if (grid[bestY][bestX + dx[i]] == CELL_WALL || grid[bestY + dy[i]][bestX] == CELL_WALL)
                    continue;
            }
            
            int moveCost = (dx[i] != 0 && dy[i] != 0) ? 14 : 10;
            int ng = nodeData[bestY][bestX].g + moveCost;
            if (ng < nodeData[ny][nx].g) {
                nodeData[ny][nx].g = ng;
                if (use8Dir) {
                    nodeData[ny][nx].f = ng + Heuristic8Dir(nx, ny, gx, gy);
                } else {
                    nodeData[ny][nx].f = ng + Heuristic(nx, ny, gx, gy) * 10;
                }
                nodeData[ny][nx].open = true;
            }
        }
    }
}

void BuildGraph(void) {
    graphEdgeCount = 0;
    
    // Clear adjacency list
    for (int i = 0; i < entranceCount; i++) {
        adjListCount[i] = 0;
    }
    
    double startTime = GetTime();
    for (int chunk = 0; chunk < chunksX * chunksY; chunk++) {
        int cx = chunk % chunksX;
        int cy = chunk / chunksX;
        int minX = cx * chunkWidth;
        int minY = cy * chunkHeight;
        int maxX = (cx + 1) * chunkWidth + 1;
        int maxY = (cy + 1) * chunkHeight + 1;
        if (maxX > gridWidth) maxX = gridWidth;
        if (maxY > gridHeight) maxY = gridHeight;

        int chunkEnts[128];
        int numEnts = 0;
        for (int i = 0; i < entranceCount && numEnts < 128; i++) {
            if (entrances[i].chunk1 == chunk || entrances[i].chunk2 == chunk)
                chunkEnts[numEnts++] = i;
        }
        for (int i = 0; i < numEnts; i++) {
            for (int j = i + 1; j < numEnts; j++) {
                int e1 = chunkEnts[i], e2 = chunkEnts[j];
                
                // Check if edge already exists (can happen when entrances share 2 chunks)
                bool exists = false;
                for (int k = 0; k < adjListCount[e1] && !exists; k++) {
                    int edgeIdx = adjList[e1][k];
                    if (graphEdges[edgeIdx].to == e2) exists = true;
                }
                if (exists) continue;
                
                int cost = AStarChunk(entrances[e1].x, entrances[e1].y, entrances[e2].x, entrances[e2].y, minX, minY, maxX, maxY);
                if (cost >= 0 && graphEdgeCount < MAX_EDGES - 1) {
                    int edgeIdx1 = graphEdgeCount;
                    int edgeIdx2 = graphEdgeCount + 1;
                    graphEdges[graphEdgeCount++] = (GraphEdge){e1, e2, cost};
                    graphEdges[graphEdgeCount++] = (GraphEdge){e2, e1, cost};
                    
                    // Add to adjacency list
                    if (adjListCount[e1] < MAX_EDGES_PER_NODE) {
                        adjList[e1][adjListCount[e1]++] = edgeIdx1;
                    }
                    if (adjListCount[e2] < MAX_EDGES_PER_NODE) {
                        adjList[e2][adjListCount[e2]++] = edgeIdx2;
                    }
                }
            }
        }
    }
    TraceLog(LOG_INFO, "Built graph: %d edges in %.2fms", graphEdgeCount, (GetTime() - startTime) * 1000);
}

// ============== Incremental Update Functions ==============

// Get the set of chunks affected by dirty chunks (dirty + their neighbors)
static void GetAffectedChunks(bool affectedChunks[MAX_CHUNKS_Y][MAX_CHUNKS_X]) {
    for (int cy = 0; cy < chunksY; cy++)
        for (int cx = 0; cx < chunksX; cx++)
            affectedChunks[cy][cx] = false;
    
    for (int cy = 0; cy < chunksY; cy++) {
        for (int cx = 0; cx < chunksX; cx++) {
            if (chunkDirty[cy][cx]) {
                affectedChunks[cy][cx] = true;
                // Mark neighbors as affected too (they share borders)
                if (cy > 0) affectedChunks[cy-1][cx] = true;
                if (cy < chunksY-1) affectedChunks[cy+1][cx] = true;
                if (cx > 0) affectedChunks[cy][cx-1] = true;
                if (cx < chunksX-1) affectedChunks[cy][cx+1] = true;
            }
        }
    }
}

// Check if an entrance touches any affected chunk
static bool EntranceTouchesAffected(int entranceIdx, bool affectedChunks[MAX_CHUNKS_Y][MAX_CHUNKS_X]) {
    int c1 = entrances[entranceIdx].chunk1;
    int c2 = entrances[entranceIdx].chunk2;
    int cy1 = c1 / chunksX, cx1 = c1 % chunksX;
    int cy2 = c2 / chunksX, cx2 = c2 % chunksX;
    return affectedChunks[cy1][cx1] || affectedChunks[cy2][cx2];
}

// Rebuild entrances for affected chunks (simpler approach - no keeping/remapping)
static void RebuildAffectedEntrances(bool affectedChunks[MAX_CHUNKS_Y][MAX_CHUNKS_X]) {
    // Remove entrances that touch any affected chunk
    int newCount = 0;
    for (int i = 0; i < entranceCount; i++) {
        if (!EntranceTouchesAffected(i, affectedChunks)) {
            entrances[newCount++] = entrances[i];
        }
    }
    int keptCount = newCount;
    
    // Rebuild entrances for borders where at least one chunk is affected
    // Horizontal borders (between cy and cy+1)
    for (int cy = 0; cy < chunksY - 1; cy++) {
        for (int cx = 0; cx < chunksX; cx++) {
            if (!affectedChunks[cy][cx] && !affectedChunks[cy+1][cx]) continue;
            
            int borderY = (cy + 1) * chunkHeight;
            int startX = cx * chunkWidth;
            int chunk1 = cy * chunksX + cx;
            int chunk2 = (cy + 1) * chunksX + cx;
            int runStart = -1;
            
            for (int i = 0; i < chunkWidth; i++) {
                int x = startX + i;
                bool open = (grid[borderY - 1][x] == CELL_WALKABLE && grid[borderY][x] == CELL_WALKABLE);
                if (open && runStart < 0) runStart = i;
                else if (!open && runStart >= 0) {
                    int length = i - runStart;
                    int pos = 0;
                    while (length > 0 && newCount < MAX_ENTRANCES) {
                        int segLen = (length > MAX_ENTRANCE_WIDTH) ? MAX_ENTRANCE_WIDTH : length;
                        int mid = pos + segLen / 2;
                        entrances[newCount++] = (Entrance){startX + runStart + mid, borderY, chunk1, chunk2};
                        pos += segLen;
                        length -= segLen;
                    }
                    runStart = -1;
                }
            }
            if (runStart >= 0) {
                int length = chunkWidth - runStart;
                int pos = 0;
                while (length > 0 && newCount < MAX_ENTRANCES) {
                    int segLen = (length > MAX_ENTRANCE_WIDTH) ? MAX_ENTRANCE_WIDTH : length;
                    int mid = pos + segLen / 2;
                    entrances[newCount++] = (Entrance){startX + runStart + mid, borderY, chunk1, chunk2};
                    pos += segLen;
                    length -= segLen;
                }
            }
        }
    }
    
    // Vertical borders (between cx and cx+1)
    for (int cy = 0; cy < chunksY; cy++) {
        for (int cx = 0; cx < chunksX - 1; cx++) {
            if (!affectedChunks[cy][cx] && !affectedChunks[cy][cx+1]) continue;
            
            int borderX = (cx + 1) * chunkWidth;
            int startY = cy * chunkHeight;
            int chunk1 = cy * chunksX + cx;
            int chunk2 = cy * chunksX + (cx + 1);
            int runStart = -1;
            
            for (int i = 0; i < chunkHeight; i++) {
                int y = startY + i;
                bool open = (grid[y][borderX - 1] == CELL_WALKABLE && grid[y][borderX] == CELL_WALKABLE);
                if (open && runStart < 0) runStart = i;
                else if (!open && runStart >= 0) {
                    int length = i - runStart;
                    int pos = 0;
                    while (length > 0 && newCount < MAX_ENTRANCES) {
                        int segLen = (length > MAX_ENTRANCE_WIDTH) ? MAX_ENTRANCE_WIDTH : length;
                        int mid = pos + segLen / 2;
                        entrances[newCount++] = (Entrance){borderX, startY + runStart + mid, chunk1, chunk2};
                        pos += segLen;
                        length -= segLen;
                    }
                    runStart = -1;
                }
            }
            if (runStart >= 0) {
                int length = chunkHeight - runStart;
                int pos = 0;
                while (length > 0 && newCount < MAX_ENTRANCES) {
                    int segLen = (length > MAX_ENTRANCE_WIDTH) ? MAX_ENTRANCE_WIDTH : length;
                    int mid = pos + segLen / 2;
                    entrances[newCount++] = (Entrance){borderX, startY + runStart + mid, chunk1, chunk2};
                    pos += segLen;
                    length -= segLen;
                }
            }
        }
    }
    
    entranceCount = newCount;
    TraceLog(LOG_INFO, "Incremental entrances: kept %d, rebuilt to %d total", keptCount, newCount);
}

// Storage for old entrances before rebuild (used for edge remapping)
static Entrance oldEntrances[MAX_ENTRANCES];
static int oldEntranceCount = 0;

// Save entrances before rebuilding (call before RebuildAffectedEntrances)
static void SaveOldEntrances(void) {
    oldEntranceCount = entranceCount;
    for (int i = 0; i < entranceCount; i++) {
        oldEntrances[i] = entrances[i];
    }
}

// Find new index for an entrance by matching position
static int FindEntranceByPosition(int x, int y) {
    for (int i = 0; i < entranceCount; i++) {
        if (entrances[i].x == x && entrances[i].y == y) {
            return i;
        }
    }
    return -1;  // Not found (entrance was removed)
}

// Get shared chunk between two entrances, returns -1 if none
static int GetSharedChunk(int e1, int e2) {
    int c1a = entrances[e1].chunk1, c1b = entrances[e1].chunk2;
    int c2a = entrances[e2].chunk1, c2b = entrances[e2].chunk2;
    if (c1a == c2a || c1a == c2b) return c1a;
    if (c1b == c2a || c1b == c2b) return c1b;
    return -1;
}

// Check if an entrance (by new index) touches any affected chunk
static bool NewEntranceTouchesAffected(int idx, bool affectedChunks[MAX_CHUNKS_Y][MAX_CHUNKS_X]) {
    int c1 = entrances[idx].chunk1;
    int c2 = entrances[idx].chunk2;
    int cy1 = c1 / chunksX, cx1 = c1 % chunksX;
    int cy2 = c2 / chunksX, cx2 = c2 % chunksX;
    return affectedChunks[cy1][cx1] || affectedChunks[cy2][cx2];
}

// Rebuild graph edges - keep edges in unaffected chunks, rebuild affected
static void RebuildAffectedEdges(bool affectedChunks[MAX_CHUNKS_Y][MAX_CHUNKS_X]) {
    // Step 1: Keep edges where both entrances don't touch any affected chunk
    int newEdgeCount = 0;
    
    for (int i = 0; i < graphEdgeCount; i++) {
        int oldE1 = graphEdges[i].from;
        int oldE2 = graphEdges[i].to;
        
        // Find new indices by position lookup
        int newE1 = FindEntranceByPosition(oldEntrances[oldE1].x, oldEntrances[oldE1].y);
        int newE2 = FindEntranceByPosition(oldEntrances[oldE2].x, oldEntrances[oldE2].y);
        
        // Skip if either entrance no longer exists
        if (newE1 < 0 || newE2 < 0) continue;
        
        // Skip if either entrance touches an affected chunk
        if (NewEntranceTouchesAffected(newE1, affectedChunks)) continue;
        if (NewEntranceTouchesAffected(newE2, affectedChunks)) continue;
        
        // Keep this edge with remapped indices
        graphEdges[newEdgeCount++] = (GraphEdge){newE1, newE2, graphEdges[i].cost};
    }
    
    int keptEdges = newEdgeCount;
    graphEdgeCount = newEdgeCount;
    
    // Step 2: Rebuild adjacency lists from kept edges
    for (int i = 0; i < entranceCount; i++) {
        adjListCount[i] = 0;
    }
    for (int i = 0; i < keptEdges; i++) {
        int e1 = graphEdges[i].from;
        if (adjListCount[e1] < MAX_EDGES_PER_NODE) {
            adjList[e1][adjListCount[e1]++] = i;
        }
    }
    
    // Step 3: Rebuild edges for all chunks
    // For unaffected chunks, edges were already kept in step 1
    // For affected chunks, we need to run A* to compute new edges
    for (int cy = 0; cy < chunksY; cy++) {
        for (int cx = 0; cx < chunksX; cx++) {
            int chunk = cy * chunksX + cx;
            int minX = cx * chunkWidth;
            int minY = cy * chunkHeight;
            int maxX = (cx + 1) * chunkWidth + 1;
            int maxY = (cy + 1) * chunkHeight + 1;
            if (maxX > gridWidth) maxX = gridWidth;
            if (maxY > gridHeight) maxY = gridHeight;
            
            // Find entrances for this chunk
            int chunkEnts[128];
            int numEnts = 0;
            for (int i = 0; i < entranceCount && numEnts < 128; i++) {
                if (entrances[i].chunk1 == chunk || entrances[i].chunk2 == chunk)
                    chunkEnts[numEnts++] = i;
            }
            
            // Build edges between all pairs
            for (int i = 0; i < numEnts; i++) {
                for (int j = i + 1; j < numEnts; j++) {
                    int e1 = chunkEnts[i], e2 = chunkEnts[j];
                    
                    // Check if edge already exists (from kept edges)
                    bool exists = false;
                    for (int k = 0; k < adjListCount[e1] && !exists; k++) {
                        int edgeIdx = adjList[e1][k];
                        if (graphEdges[edgeIdx].to == e2) exists = true;
                    }
                    if (exists) continue;
                    
                    int cost = AStarChunk(entrances[e1].x, entrances[e1].y, 
                                         entrances[e2].x, entrances[e2].y, 
                                         minX, minY, maxX, maxY);
                    if (cost >= 0 && graphEdgeCount < MAX_EDGES - 1) {
                        int edgeIdx1 = graphEdgeCount;
                        int edgeIdx2 = graphEdgeCount + 1;
                        graphEdges[graphEdgeCount++] = (GraphEdge){e1, e2, cost};
                        graphEdges[graphEdgeCount++] = (GraphEdge){e2, e1, cost};
                        
                        if (adjListCount[e1] < MAX_EDGES_PER_NODE) {
                            adjList[e1][adjListCount[e1]++] = edgeIdx1;
                        }
                        if (adjListCount[e2] < MAX_EDGES_PER_NODE) {
                            adjList[e2][adjListCount[e2]++] = edgeIdx2;
                        }
                    }
                }
            }
        }
    }
    
    TraceLog(LOG_INFO, "Incremental edges: kept %d, total now %d", keptEdges, graphEdgeCount);
}

void UpdateDirtyChunks(void) {
    // Check if any chunks are dirty
    bool anyDirty = false;
    for (int cy = 0; cy < chunksY && !anyDirty; cy++)
        for (int cx = 0; cx < chunksX && !anyDirty; cx++)
            if (chunkDirty[cy][cx]) anyDirty = true;
    
    if (!anyDirty) return;
    
    double startTime = GetTime();
    
    // Get affected chunks (dirty + neighbors)
    bool affectedChunks[MAX_CHUNKS_Y][MAX_CHUNKS_X];
    GetAffectedChunks(affectedChunks);
    
    int dirtyCount = 0, affectedCount = 0;
    for (int cy = 0; cy < chunksY; cy++) {
        for (int cx = 0; cx < chunksX; cx++) {
            if (chunkDirty[cy][cx]) dirtyCount++;
            if (affectedChunks[cy][cx]) affectedCount++;
        }
    }
    
    // Save old entrances for edge remapping
    SaveOldEntrances();
    
    // Rebuild entrances for affected chunks
    RebuildAffectedEntrances(affectedChunks);
    
    // Rebuild edges (keeps edges in unaffected chunks, rebuilds affected)
    RebuildAffectedEdges(affectedChunks);
    
    // Clear dirty flags
    for (int cy = 0; cy < chunksY; cy++)
        for (int cx = 0; cx < chunksX; cx++)
            chunkDirty[cy][cx] = false;
    needsRebuild = false;
    
    double elapsed = (GetTime() - startTime) * 1000.0;
    TraceLog(LOG_INFO, "Incremental update: %d dirty, %d affected chunks in %.2fms", 
             dirtyCount, affectedCount, elapsed);
}

void RunAStar(void) {
    if (startPos.x < 0 || goalPos.x < 0) return;
    pathLength = 0;
    nodesExplored = 0;
    double startTime = GetTime();

    for (int y = 0; y < gridHeight; y++)
        for (int x = 0; x < gridWidth; x++)
            nodeData[y][x] = (AStarNode){999999, 999999, -1, -1, false, false};

    nodeData[startPos.y][startPos.x].g = 0;
    if (use8Dir) {
        nodeData[startPos.y][startPos.x].f = Heuristic8Dir(startPos.x, startPos.y, goalPos.x, goalPos.y);
    } else {
        nodeData[startPos.y][startPos.x].f = Heuristic(startPos.x, startPos.y, goalPos.x, goalPos.y) * 10;
    }
    nodeData[startPos.y][startPos.x].open = true;

    // Direction arrays
    int dx4[] = {0, 1, 0, -1};
    int dy4[] = {-1, 0, 1, 0};
    int dx8[] = {0, 1, 1, 1, 0, -1, -1, -1};
    int dy8[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    
    int* dx = use8Dir ? dx8 : dx4;
    int* dy = use8Dir ? dy8 : dy4;
    int numDirs = use8Dir ? 8 : 4;

    while (1) {
        int bestX = -1, bestY = -1, bestF = 999999;
        for (int y = 0; y < gridHeight; y++)
            for (int x = 0; x < gridWidth; x++)
                if (nodeData[y][x].open && nodeData[y][x].f < bestF) {
                    bestF = nodeData[y][x].f;
                    bestX = x;
                    bestY = y;
                }
        if (bestX < 0) break;
        if (bestX == goalPos.x && bestY == goalPos.y) {
            int cx = goalPos.x, cy = goalPos.y;
            while (cx >= 0 && cy >= 0 && pathLength < MAX_PATH) {
                path[pathLength++] = (Point){cx, cy};
                int px = nodeData[cy][cx].parentX;
                int py = nodeData[cy][cx].parentY;
                cx = px; cy = py;
            }
            break;
        }
        nodeData[bestY][bestX].open = false;
        nodeData[bestY][bestX].closed = true;
        nodesExplored++;
        for (int i = 0; i < numDirs; i++) {
            int nx = bestX + dx[i], ny = bestY + dy[i];
            if (nx < 0 || nx >= gridWidth || ny < 0 || ny >= gridHeight) continue;
            if (grid[ny][nx] == CELL_WALL || nodeData[ny][nx].closed) continue;
            
            // For diagonal movement, check that we can actually move diagonally
            // (not cutting corners through walls)
            if (use8Dir && dx[i] != 0 && dy[i] != 0) {
                if (grid[bestY][bestX + dx[i]] == CELL_WALL || grid[bestY + dy[i]][bestX] == CELL_WALL)
                    continue;
            }
            
            // Cost: 10 for cardinal, 14 for diagonal
            int moveCost = (dx[i] != 0 && dy[i] != 0) ? 14 : 10;
            int ng = nodeData[bestY][bestX].g + moveCost;
            
            if (ng < nodeData[ny][nx].g) {
                nodeData[ny][nx].g = ng;
                if (use8Dir) {
                    nodeData[ny][nx].f = ng + Heuristic8Dir(nx, ny, goalPos.x, goalPos.y);
                } else {
                    nodeData[ny][nx].f = ng + Heuristic(nx, ny, goalPos.x, goalPos.y) * 10;
                }
                nodeData[ny][nx].parentX = bestX;
                nodeData[ny][nx].parentY = bestY;
                nodeData[ny][nx].open = true;
            }
        }
    }
    lastPathTime = (GetTime() - startTime) * 1000.0;
    TraceLog(LOG_INFO, "A* (%s): time=%.2fms, nodes=%d, path=%d", 
             use8Dir ? "8-dir" : "4-dir", lastPathTime, nodesExplored, pathLength);
}

// Get chunk index from cell coordinates
static int GetChunk(int x, int y) {
    int cx = x / chunkWidth;
    int cy = y / chunkHeight;
    if (cx < 0) cx = 0;
    if (cx >= chunksX) cx = chunksX - 1;
    if (cy < 0) cy = 0;
    if (cy >= chunksY) cy = chunksY - 1;
    return cy * chunksX + cx;
}

// Get chunk bounds
static void GetChunkBounds(int chunk, int* minX, int* minY, int* maxX, int* maxY) {
    int cx = chunk % chunksX;
    int cy = chunk / chunksX;
    *minX = cx * chunkWidth;
    *minY = cy * chunkHeight;
    *maxX = (cx + 1) * chunkWidth;
    *maxY = (cy + 1) * chunkHeight;
    if (*maxX > gridWidth) *maxX = gridWidth;
    if (*maxY > gridHeight) *maxY = gridHeight;
}

// Reconstruct cell-level path between two points within a chunk
static int ReconstructLocalPath(int sx, int sy, int gx, int gy, Point* outPath, int maxLen) {
    // Get bounds that cover both start and goal chunks
    int startChunk = GetChunk(sx, sy);
    int goalChunk = GetChunk(gx, gy);
    
    int minX1, minY1, maxX1, maxY1;
    int minX2, minY2, maxX2, maxY2;
    GetChunkBounds(startChunk, &minX1, &minY1, &maxX1, &maxY1);
    GetChunkBounds(goalChunk, &minX2, &minY2, &maxX2, &maxY2);
    
    // Union of both chunk bounds
    int minX = minX1 < minX2 ? minX1 : minX2;
    int minY = minY1 < minY2 ? minY1 : minY2;
    int maxX = maxX1 > maxX2 ? maxX1 : maxX2;
    int maxY = maxY1 > maxY2 ? maxY1 : maxY2;
    
    // Expand bounds generously to allow paths that may need to go around obstacles
    // Entrances on chunk borders may need to path through adjacent chunks
    int expandX = chunkWidth / 2;
    int expandY = chunkHeight / 2;
    minX -= expandX; if (minX < 0) minX = 0;
    minY -= expandY; if (minY < 0) minY = 0;
    maxX += expandX; if (maxX > gridWidth) maxX = gridWidth;
    maxY += expandY; if (maxY > gridHeight) maxY = gridHeight;
    
    // Run A* in this region
    for (int y = minY; y < maxY; y++)
        for (int x = minX; x < maxX; x++)
            nodeData[y][x] = (AStarNode){999999, 999999, -1, -1, false, false};
    
    nodeData[sy][sx].g = 0;
    if (use8Dir) {
        nodeData[sy][sx].f = Heuristic8Dir(sx, sy, gx, gy);
    } else {
        nodeData[sy][sx].f = Heuristic(sx, sy, gx, gy) * 10;
    }
    nodeData[sy][sx].open = true;
    
    int dx4[] = {0, 1, 0, -1};
    int dy4[] = {-1, 0, 1, 0};
    int dx8[] = {0, 1, 1, 1, 0, -1, -1, -1};
    int dy8[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    
    int* dx = use8Dir ? dx8 : dx4;
    int* dy = use8Dir ? dy8 : dy4;
    int numDirs = use8Dir ? 8 : 4;
    
    while (1) {
        int bestX = -1, bestY = -1, bestF = 999999;
        for (int y = minY; y < maxY; y++)
            for (int x = minX; x < maxX; x++)
                if (nodeData[y][x].open && nodeData[y][x].f < bestF) {
                    bestF = nodeData[y][x].f;
                    bestX = x;
                    bestY = y;
                }
        if (bestX < 0) return 0;  // No path
        if (bestX == gx && bestY == gy) {
            // Reconstruct path
            int len = 0;
            int cx = gx, cy = gy;
            while (cx >= 0 && cy >= 0 && len < maxLen) {
                outPath[len++] = (Point){cx, cy};
                int px = nodeData[cy][cx].parentX;
                int py = nodeData[cy][cx].parentY;
                cx = px;
                cy = py;
            }
            return len;
        }
        nodeData[bestY][bestX].open = false;
        nodeData[bestY][bestX].closed = true;
        for (int i = 0; i < numDirs; i++) {
            int nx = bestX + dx[i], ny = bestY + dy[i];
            if (nx < minX || nx >= maxX || ny < minY || ny >= maxY) continue;
            if (grid[ny][nx] == CELL_WALL || nodeData[ny][nx].closed) continue;
            
            // Prevent corner cutting for diagonal movement
            if (use8Dir && dx[i] != 0 && dy[i] != 0) {
                if (grid[bestY][bestX + dx[i]] == CELL_WALL || grid[bestY + dy[i]][bestX] == CELL_WALL)
                    continue;
            }
            
            int moveCost = (dx[i] != 0 && dy[i] != 0) ? 14 : 10;
            int ng = nodeData[bestY][bestX].g + moveCost;
            if (ng < nodeData[ny][nx].g) {
                nodeData[ny][nx].g = ng;
                if (use8Dir) {
                    nodeData[ny][nx].f = ng + Heuristic8Dir(nx, ny, gx, gy);
                } else {
                    nodeData[ny][nx].f = ng + Heuristic(nx, ny, gx, gy) * 10;
                }
                nodeData[ny][nx].parentX = bestX;
                nodeData[ny][nx].parentY = bestY;
                nodeData[ny][nx].open = true;
            }
        }
    }
}

void RunHPAStar(void) {
    if (startPos.x < 0 || goalPos.x < 0) return;
    if (entranceCount == 0) return;  // Need to build entrances first
    
    pathLength = 0;
    abstractPathLength = 0;
    nodesExplored = 0;
    hpaAbstractTime = 0.0;
    hpaRefinementTime = 0.0;
    double startTime = GetTime();
    
    int startChunk = GetChunk(startPos.x, startPos.y);
    int goalChunk = GetChunk(goalPos.x, goalPos.y);
    
    // Special case: start and goal in same chunk - just do local A*
    if (startChunk == goalChunk) {
        int minX, minY, maxX, maxY;
        GetChunkBounds(startChunk, &minX, &minY, &maxX, &maxY);
        pathLength = ReconstructLocalPath(startPos.x, startPos.y, goalPos.x, goalPos.y, path, MAX_PATH);
        lastPathTime = (GetTime() - startTime) * 1000.0;
        return;
    }
    
    // Temporary entrance indices for start and goal
    int startNode = entranceCount;      // Index for start as temp node
    int goalNode = entranceCount + 1;   // Index for goal as temp node
    int totalNodes = entranceCount + 2;
    
    // Initialize abstract nodes
    for (int i = 0; i < totalNodes; i++) {
        abstractNodes[i] = (AbstractNode){999999, 999999, -1, false, false};
    }
    
    // Build temporary edges from start to entrances in its chunk
    // and from goal to entrances in its chunk
    // We'll store these costs in arrays
    int startEdgeCosts[128];
    int startEdgeTargets[128];
    int startEdgeCount = 0;
    
    int goalEdgeCosts[128];
    int goalEdgeTargets[128];
    int goalEdgeCount = 0;
    
    int minX, minY, maxX, maxY;
    
    double connectStartTime = GetTime();
    
    // Connect start to entrances in start chunk
    GetChunkBounds(startChunk, &minX, &minY, &maxX, &maxY);
    if (maxX < gridWidth) maxX++;
    if (maxY < gridHeight) maxY++;
    
    for (int i = 0; i < entranceCount && startEdgeCount < 128; i++) {
        if (entrances[i].chunk1 == startChunk || entrances[i].chunk2 == startChunk) {
            int cost = AStarChunk(startPos.x, startPos.y, entrances[i].x, entrances[i].y, 
                                  minX > 0 ? minX - 1 : 0, minY > 0 ? minY - 1 : 0, maxX, maxY);
            if (cost >= 0) {
                startEdgeTargets[startEdgeCount] = i;
                startEdgeCosts[startEdgeCount] = cost;
                startEdgeCount++;
                nodesExplored++;
            }
        }
    }
    
    // Connect goal to entrances in goal chunk
    GetChunkBounds(goalChunk, &minX, &minY, &maxX, &maxY);
    if (maxX < gridWidth) maxX++;
    if (maxY < gridHeight) maxY++;
    
    for (int i = 0; i < entranceCount && goalEdgeCount < 128; i++) {
        if (entrances[i].chunk1 == goalChunk || entrances[i].chunk2 == goalChunk) {
            int cost = AStarChunk(goalPos.x, goalPos.y, entrances[i].x, entrances[i].y,
                                  minX > 0 ? minX - 1 : 0, minY > 0 ? minY - 1 : 0, maxX, maxY);
            if (cost >= 0) {
                goalEdgeTargets[goalEdgeCount] = i;
                goalEdgeCosts[goalEdgeCount] = cost;
                goalEdgeCount++;
                nodesExplored++;
            }
        }
    }
    
    double connectTime = (GetTime() - connectStartTime) * 1000.0;
    
    // A* on abstract graph using binary heap
    double abstractStartTime = GetTime();
    HeapInit();
    
    abstractNodes[startNode].g = 0;
    abstractNodes[startNode].f = Heuristic(startPos.x, startPos.y, goalPos.x, goalPos.y);
    abstractNodes[startNode].open = true;
    HeapPush(startNode);
    
    while (heap.size > 0) {
        // Pop best node from heap
        int best = HeapPop();
        
        // Skip if already closed (duplicate entry in heap)
        if (abstractNodes[best].closed) continue;
        
        if (best == goalNode) {
            // Reconstruct abstract path
            int current = goalNode;
            while (current >= 0 && abstractPathLength < MAX_ENTRANCES + 2) {
                abstractPath[abstractPathLength++] = current;
                current = abstractNodes[current].parent;
            }
            break;
        }
        
        abstractNodes[best].open = false;
        abstractNodes[best].closed = true;
        nodesExplored++;
        
        // Expand neighbors
        if (best == startNode) {
            // Expand from start to its connected entrances
            for (int i = 0; i < startEdgeCount; i++) {
                int neighbor = startEdgeTargets[i];
                if (abstractNodes[neighbor].closed) continue;
                int ng = abstractNodes[best].g + startEdgeCosts[i];
                if (ng < abstractNodes[neighbor].g) {
                    abstractNodes[neighbor].g = ng;
                    abstractNodes[neighbor].f = ng + Heuristic(entrances[neighbor].x, entrances[neighbor].y, goalPos.x, goalPos.y);
                    abstractNodes[neighbor].parent = best;
                    abstractNodes[neighbor].open = true;
                    HeapPush(neighbor);
                }
            }
        } else if (best < entranceCount) {
            // Expand from a regular entrance using adjacency list
            for (int i = 0; i < adjListCount[best]; i++) {
                int edgeIdx = adjList[best][i];
                int neighbor = graphEdges[edgeIdx].to;
                if (abstractNodes[neighbor].closed) continue;
                int ng = abstractNodes[best].g + graphEdges[edgeIdx].cost;
                if (ng < abstractNodes[neighbor].g) {
                    abstractNodes[neighbor].g = ng;
                    abstractNodes[neighbor].f = ng + Heuristic(entrances[neighbor].x, entrances[neighbor].y, goalPos.x, goalPos.y);
                    abstractNodes[neighbor].parent = best;
                    abstractNodes[neighbor].open = true;
                    HeapPush(neighbor);
                }
            }
            // Also check if this entrance can reach goal
            for (int i = 0; i < goalEdgeCount; i++) {
                if (goalEdgeTargets[i] == best) {
                    int neighbor = goalNode;
                    if (abstractNodes[neighbor].closed) continue;
                    int ng = abstractNodes[best].g + goalEdgeCosts[i];
                    if (ng < abstractNodes[neighbor].g) {
                        abstractNodes[neighbor].g = ng;
                        abstractNodes[neighbor].f = ng;  // h=0 at goal
                        abstractNodes[neighbor].parent = best;
                        abstractNodes[neighbor].open = true;
                        HeapPush(neighbor);
                    }
                }
            }
        }
    }
    hpaAbstractTime = (GetTime() - abstractStartTime) * 1000.0;
    
    // Now refine abstract path to cell-level path
    double refineStartTime = GetTime();
    if (abstractPathLength > 0) {
        // Abstract path is in reverse order (goal to start)
        // We need to walk it forward and reconstruct local paths
        Point tempPath[MAX_PATH];
        
        for (int i = abstractPathLength - 1; i > 0; i--) {
            int fromNode = abstractPath[i];
            int toNode = abstractPath[i - 1];
            
            int fx, fy, tx, ty;
            
            // Get coordinates for from node
            if (fromNode == startNode) {
                fx = startPos.x;
                fy = startPos.y;
            } else {
                fx = entrances[fromNode].x;
                fy = entrances[fromNode].y;
            }
            
            // Get coordinates for to node
            if (toNode == goalNode) {
                tx = goalPos.x;
                ty = goalPos.y;
            } else {
                tx = entrances[toNode].x;
                ty = entrances[toNode].y;
            }
            
            // Reconstruct local path for this segment
            int localLen = ReconstructLocalPath(fx, fy, tx, ty, tempPath, MAX_PATH);
            
            if (localLen == 0) {
                // No path found for this segment - shouldn't happen with valid abstract path
                TraceLog(LOG_WARNING, "HPA* refinement failed: no path from (%d,%d) to (%d,%d)", fx, fy, tx, ty);
                continue;
            }
            
            // tempPath is in reverse order: [0]=destination, [localLen-1]=source
            // We iterate from source to destination (high index to low)
            // Skip source point for subsequent segments (it's the destination of previous segment)
            int skipSource = (i == abstractPathLength - 1) ? 0 : 1;
            for (int j = localLen - 1 - skipSource; j >= 0 && pathLength < MAX_PATH; j--) {
                path[pathLength++] = tempPath[j];
            }
        }
        
        // Reverse path so it goes from goal to start (matching RunAStar behavior)
        for (int i = 0; i < pathLength / 2; i++) {
            Point tmp = path[i];
            path[i] = path[pathLength - 1 - i];
            path[pathLength - 1 - i] = tmp;
        }
    }
    hpaRefinementTime = (GetTime() - refineStartTime) * 1000.0;
    
    lastPathTime = (GetTime() - startTime) * 1000.0;
    TraceLog(LOG_INFO, "HPA*: total=%.2fms (connect=%.2fms, search=%.2fms, refine=%.2fms), nodes=%d, path=%d", 
             lastPathTime, connectTime, hpaAbstractTime, hpaRefinementTime, nodesExplored, pathLength);
}

// ============== JPS Implementation ==============

static bool IsWalkable(int x, int y) {
    if (x < 0 || x >= gridWidth || y < 0 || y >= gridHeight) return false;
    return grid[y][x] == CELL_WALKABLE;
}

// Jump in a cardinal direction (4-dir or 8-dir)
static bool Jump(int x, int y, int dx, int dy, int gx, int gy, int* jx, int* jy) {
    int nx = x + dx;
    int ny = y + dy;
    
    if (!IsWalkable(nx, ny)) return false;
    
    if (nx == gx && ny == gy) {
        *jx = nx;
        *jy = ny;
        return true;
    }
    
    // Diagonal movement
    if (dx != 0 && dy != 0) {
        // Check for forced neighbors
        if ((!IsWalkable(nx - dx, ny) && IsWalkable(nx - dx, ny + dy)) ||
            (!IsWalkable(nx, ny - dy) && IsWalkable(nx + dx, ny - dy))) {
            *jx = nx;
            *jy = ny;
            return true;
        }
        // Recursively jump in cardinal directions
        int tempX, tempY;
        if (Jump(nx, ny, dx, 0, gx, gy, &tempX, &tempY)) {
            *jx = nx;
            *jy = ny;
            return true;
        }
        if (Jump(nx, ny, 0, dy, gx, gy, &tempX, &tempY)) {
            *jx = nx;
            *jy = ny;
            return true;
        }
    } else {
        // Horizontal movement
        if (dx != 0) {
            if ((!IsWalkable(nx, ny + 1) && IsWalkable(nx + dx, ny + 1)) ||
                (!IsWalkable(nx, ny - 1) && IsWalkable(nx + dx, ny - 1))) {
                *jx = nx;
                *jy = ny;
                return true;
            }
        }
        // Vertical movement
        if (dy != 0) {
            if ((!IsWalkable(nx + 1, ny) && IsWalkable(nx + 1, ny + dy)) ||
                (!IsWalkable(nx - 1, ny) && IsWalkable(nx - 1, ny + dy))) {
                *jx = nx;
                *jy = ny;
                return true;
            }
        }
    }
    
    // Continue jumping in this direction
    return Jump(nx, ny, dx, dy, gx, gy, jx, jy);
}

void RunJPS(void) {
    if (startPos.x < 0 || goalPos.x < 0) return;
    
    pathLength = 0;
    nodesExplored = 0;
    double startTime = GetTime();
    
    // Initialize node data
    for (int y = 0; y < gridHeight; y++)
        for (int x = 0; x < gridWidth; x++)
            nodeData[y][x] = (AStarNode){999999, 999999, -1, -1, false, false};
    
    nodeData[startPos.y][startPos.x].g = 0;
    if (use8Dir) {
        nodeData[startPos.y][startPos.x].f = Heuristic8Dir(startPos.x, startPos.y, goalPos.x, goalPos.y);
    } else {
        nodeData[startPos.y][startPos.x].f = Heuristic(startPos.x, startPos.y, goalPos.x, goalPos.y) * 10;
    }
    nodeData[startPos.y][startPos.x].open = true;
    
    // Direction arrays
    int dx4[] = {0, 1, 0, -1};
    int dy4[] = {-1, 0, 1, 0};
    int dx8[] = {0, 1, 1, 1, 0, -1, -1, -1};
    int dy8[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    
    int* dx = use8Dir ? dx8 : dx4;
    int* dy = use8Dir ? dy8 : dy4;
    int numDirs = use8Dir ? 8 : 4;
    
    while (1) {
        // Find node with lowest f
        int bestX = -1, bestY = -1, bestF = 999999;
        for (int y = 0; y < gridHeight; y++)
            for (int x = 0; x < gridWidth; x++)
                if (nodeData[y][x].open && nodeData[y][x].f < bestF) {
                    bestF = nodeData[y][x].f;
                    bestX = x;
                    bestY = y;
                }
        
        if (bestX < 0) break;  // No path
        
        if (bestX == goalPos.x && bestY == goalPos.y) {
            // Reconstruct path
            int cx = goalPos.x, cy = goalPos.y;
            while (cx >= 0 && cy >= 0 && pathLength < MAX_PATH) {
                path[pathLength++] = (Point){cx, cy};
                int px = nodeData[cy][cx].parentX;
                int py = nodeData[cy][cx].parentY;
                
                // For JPS, we need to fill in intermediate points
                if (px >= 0 && py >= 0) {
                    int stepX = (px > cx) ? 1 : (px < cx) ? -1 : 0;
                    int stepY = (py > cy) ? 1 : (py < cy) ? -1 : 0;
                    int ix = cx + stepX;
                    int iy = cy + stepY;
                    while ((ix != px || iy != py) && pathLength < MAX_PATH) {
                        path[pathLength++] = (Point){ix, iy};
                        ix += stepX;
                        iy += stepY;
                    }
                }
                cx = px;
                cy = py;
            }
            break;
        }
        
        nodeData[bestY][bestX].open = false;
        nodeData[bestY][bestX].closed = true;
        nodesExplored++;
        
        // Explore neighbors using JPS
        for (int i = 0; i < numDirs; i++) {
            int jx, jy;
            
            if (use8Dir) {
                // Try to jump in this direction
                if (!Jump(bestX, bestY, dx[i], dy[i], goalPos.x, goalPos.y, &jx, &jy))
                    continue;
            } else {
                // For 4-dir, just do regular A* neighbor expansion (JPS needs diagonals)
                jx = bestX + dx[i];
                jy = bestY + dy[i];
                if (!IsWalkable(jx, jy)) continue;
            }
            
            if (nodeData[jy][jx].closed) continue;
            
            // Calculate cost
            int dist = abs(jx - bestX) + abs(jy - bestY);
            if (use8Dir) {
                // Diagonal distance
                int ddx = abs(jx - bestX);
                int ddy = abs(jy - bestY);
                dist = 10 * (ddx > ddy ? ddx : ddy) + 4 * (ddx < ddy ? ddx : ddy);
            } else {
                dist *= 10;
            }
            
            int ng = nodeData[bestY][bestX].g + dist;
            
            if (ng < nodeData[jy][jx].g) {
                nodeData[jy][jx].g = ng;
                if (use8Dir) {
                    nodeData[jy][jx].f = ng + Heuristic8Dir(jx, jy, goalPos.x, goalPos.y);
                } else {
                    nodeData[jy][jx].f = ng + Heuristic(jx, jy, goalPos.x, goalPos.y) * 10;
                }
                nodeData[jy][jx].parentX = bestX;
                nodeData[jy][jx].parentY = bestY;
                nodeData[jy][jx].open = true;
            }
        }
    }
    
    lastPathTime = (GetTime() - startTime) * 1000.0;
    TraceLog(LOG_INFO, "JPS (%s): time=%.2fms, nodes=%d, path=%d", 
             use8Dir ? "8-dir" : "4-dir", lastPathTime, nodesExplored, pathLength);
}
