#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "terrain.h"
#include <stdbool.h>

#define MAX_ENTRANCE_WIDTH 6
#define MAX_ENTRANCES (4096*4)
#define MAX_PATH 65536*2
#define MAX_EDGES 65536*2

#define CHUNKS_X (GRID_WIDTH / CHUNK_SIZE)
#define CHUNKS_Y (GRID_HEIGHT / CHUNK_SIZE)

typedef struct { int x, y; } Point;

typedef struct {
    int x, y;
    int chunk1, chunk2;
} Entrance;

typedef struct {
    int g, f;
    int parentX, parentY;
    bool open, closed;
} AStarNode;

typedef struct {
    int from, to;
    int cost;
} GraphEdge;

// External state (defined in pathfinding.c)
extern Entrance entrances[MAX_ENTRANCES];
extern int entranceCount;
extern GraphEdge graphEdges[MAX_EDGES];
extern int graphEdgeCount;
extern Point path[MAX_PATH];
extern int pathLength;
extern int nodesExplored;
extern double lastPathTime;
extern Point startPos;
extern Point goalPos;
extern AStarNode nodeData[GRID_HEIGHT][GRID_WIDTH];
extern bool chunkDirty[CHUNKS_Y][CHUNKS_X];

// Functions
int Heuristic(int x1, int y1, int x2, int y2);
void MarkChunkDirty(int cellX, int cellY);
void BuildEntrances(void);
void BuildGraph(void);
void RunAStar(void);
int AStarChunk(int sx, int sy, int gx, int gy, int minX, int minY, int maxX, int maxY);

#endif // PATHFINDING_H
