#ifndef TERRAIN_H
#define TERRAIN_H

#include "raylib.h"

typedef enum { CELL_WALKABLE, CELL_WALL } CellType;

// Grid dimensions - must match main file
#define GRID_WIDTH  (128*4)
#define GRID_HEIGHT (128*4)
#define CHUNK_SIZE  32

// External grid reference (defined in main file)
extern CellType grid[GRID_HEIGHT][GRID_WIDTH];
extern bool needsRebuild;

// Terrain generation functions
void InitGrid(void);
void GenerateSparse(float density);
void GeneratePerlin(void);
void GenerateCity(void);
void GenerateMixed(void);

// Perlin noise utilities
void InitPerlin(int seed);
float Perlin(float x, float y);
float OctavePerlin(float x, float y, int octaves, float persistence);

#endif // TERRAIN_H
