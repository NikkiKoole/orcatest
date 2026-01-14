#include "grid.h"
#include <string.h>

CellType grid[MAX_GRID_HEIGHT][MAX_GRID_WIDTH];
bool needsRebuild = false;

// Runtime dimensions - default to max
int gridWidth = MAX_GRID_WIDTH;
int gridHeight = MAX_GRID_HEIGHT;
int chunkWidth = DEFAULT_CHUNK_SIZE;
int chunkHeight = DEFAULT_CHUNK_SIZE;
int chunksX = MAX_GRID_WIDTH / DEFAULT_CHUNK_SIZE;
int chunksY = MAX_GRID_HEIGHT / DEFAULT_CHUNK_SIZE;

void InitGridWithSizeAndChunkSize(int width, int height, int chunkW, int chunkH) {
    // Clamp to max dimensions
    if (width > MAX_GRID_WIDTH) width = MAX_GRID_WIDTH;
    if (height > MAX_GRID_HEIGHT) height = MAX_GRID_HEIGHT;
    if (width < 1) width = 1;
    if (height < 1) height = 1;
    
    // Clamp chunk size
    if (chunkW < 1) chunkW = width;
    if (chunkH < 1) chunkH = height;
    if (chunkW > width) chunkW = width;
    if (chunkH > height) chunkH = height;
    
    gridWidth = width;
    gridHeight = height;
    chunkWidth = chunkW;
    chunkHeight = chunkH;
    chunksX = (gridWidth + chunkWidth - 1) / chunkWidth;   // ceiling division
    chunksY = (gridHeight + chunkHeight - 1) / chunkHeight;
    
    // Clear the grid
    for (int y = 0; y < gridHeight; y++)
        for (int x = 0; x < gridWidth; x++)
            grid[y][x] = CELL_WALKABLE;
    
    needsRebuild = true;
}

void InitGridWithSize(int width, int height) {
    InitGridWithSizeAndChunkSize(width, height, DEFAULT_CHUNK_SIZE, DEFAULT_CHUNK_SIZE);
}

int InitGridFromAsciiWithChunkSize(const char* ascii, int chunkW, int chunkH) {
    // First pass: find dimensions
    int width = 0;
    int height = 0;
    int currentWidth = 0;
    
    for (const char* p = ascii; *p; p++) {
        if (*p == '\n') {
            if (currentWidth > width) width = currentWidth;
            if (currentWidth > 0) height++;
            currentWidth = 0;
        } else {
            currentWidth++;
        }
    }
    // Handle last line without newline
    if (currentWidth > 0) {
        if (currentWidth > width) width = currentWidth;
        height++;
    }
    
    if (width == 0 || height == 0) return 0;
    
    // If chunk size is 0, use grid dimensions (1 chunk = whole grid)
    if (chunkW <= 0) chunkW = width;
    if (chunkH <= 0) chunkH = height;
    
    // Initialize grid with these dimensions
    InitGridWithSizeAndChunkSize(width, height, chunkW, chunkH);
    
    // Second pass: fill grid
    int x = 0, y = 0;
    for (const char* p = ascii; *p; p++) {
        if (*p == '\n') {
            x = 0;
            y++;
        } else {
            if (x < gridWidth && y < gridHeight) {
                if (*p == '#') {
                    grid[y][x] = CELL_WALL;
                } else {
                    grid[y][x] = CELL_WALKABLE;
                }
            }
            x++;
        }
    }
    
    return 1;
}

int InitGridFromAscii(const char* ascii) {
    return InitGridFromAsciiWithChunkSize(ascii, DEFAULT_CHUNK_SIZE, DEFAULT_CHUNK_SIZE);
}
