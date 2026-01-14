#ifndef GRID_H
#define GRID_H

#include <stdbool.h>

// Maximum grid dimensions (for static array allocation)
#define MAX_GRID_WIDTH  (128*4)
#define MAX_GRID_HEIGHT (128*4)
#define DEFAULT_CHUNK_SIZE  32

// Runtime grid and chunk dimensions
extern int gridWidth;
extern int gridHeight;
extern int chunkWidth;
extern int chunkHeight;
extern int chunksX;
extern int chunksY;

// For static array sizing
#define MAX_CHUNKS_X (MAX_GRID_WIDTH / 8)   // minimum chunk size of 8
#define MAX_CHUNKS_Y (MAX_GRID_HEIGHT / 8)

typedef enum { CELL_WALKABLE, CELL_WALL } CellType;

extern CellType grid[MAX_GRID_HEIGHT][MAX_GRID_WIDTH];
extern bool needsRebuild;

// Initialize grid with specific dimensions and default chunk size (32x32)
void InitGridWithSize(int width, int height);

// Initialize grid with specific dimensions and chunk size
void InitGridWithSizeAndChunkSize(int width, int height, int chunkW, int chunkH);

// Initialize grid from ASCII map with default chunk size
// '.' = walkable, '#' = wall, newlines separate rows
// Dimensions are auto-detected from the string
// Returns 1 on success, 0 on failure
int InitGridFromAscii(const char* ascii);

// Initialize grid from ASCII map with custom chunk size
// If chunkW/chunkH are 0, uses grid dimensions (1 chunk = whole grid)
int InitGridFromAsciiWithChunkSize(const char* ascii, int chunkW, int chunkH);

#endif // GRID_H
