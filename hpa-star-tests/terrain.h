#ifndef TERRAIN_H
#define TERRAIN_H

#include "grid.h"

// Terrain generation functions
void InitGrid(void);
void GenerateSparse(float density);
void GeneratePerlin(void);
void GenerateCity(void);
void GenerateMixed(void);
void GenerateConcentricMaze(void);

// Perlin noise utilities
void InitPerlin(int seed);
float Perlin(float x, float y);
float OctavePerlin(float x, float y, int octaves, float persistence);

#endif // TERRAIN_H
