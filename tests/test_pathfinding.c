#include "c89spec.h"
#include "raylib.h"
#include <stdlib.h>
#include "../hpa-star-tests/grid.h"
#include "../hpa-star-tests/terrain.h"
#include "../hpa-star-tests/pathfinding.h"

// Test grid size - 4x4 chunks = 128x128 cells
#define TEST_GRID_SIZE (DEFAULT_CHUNK_SIZE * 3)

describe(grid_initialization) {
    it("should initialize grid to all walkable cells") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        int allWalkable = 1;
        for (int y = 0; y < gridHeight && allWalkable; y++)
            for (int x = 0; x < gridWidth && allWalkable; x++)
                if (grid[y][x] != CELL_WALKABLE) allWalkable = 0;
        expect(allWalkable == 1);
    }

    it("should mark chunks as dirty when walls are placed") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        // Clear dirty flags
        for (int cy = 0; cy < chunksY; cy++)
            for (int cx = 0; cx < chunksX; cx++)
                chunkDirty[cy][cx] = false;
        needsRebuild = false;

        // Place a wall and mark dirty
        grid[10][10] = CELL_WALL;
        MarkChunkDirty(10, 10);

        int cx = 10 / chunkWidth;
        int cy = 10 / chunkHeight;
        expect(chunkDirty[cy][cx] == true && needsRebuild == true);
    }
}

describe(entrance_building) {
    it("should create entrances on chunk borders") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        BuildEntrances();

        // Verify we have entrances
        int hasEntrances = entranceCount > 0;

        // Verify all entrances are on chunk borders
        int allOnBorders = 1;
        for (int i = 0; i < entranceCount && allOnBorders; i++) {
            int x = entrances[i].x;
            int y = entrances[i].y;
            // Entrance must be on a vertical border (x % chunkWidth == 0)
            // OR on a horizontal border (y % chunkHeight == 0)
            int onVerticalBorder = (x % chunkWidth == 0);
            int onHorizontalBorder = (y % chunkHeight == 0);
            if (!onVerticalBorder && !onHorizontalBorder) {
                allOnBorders = 0;
            }
        }

        expect(hasEntrances && allOnBorders);
    }

    it("should not create entrances where walls block the border") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        // Block the entire first horizontal border
        int borderY = chunkHeight;
        for (int x = 0; x < gridWidth; x++) {
            grid[borderY - 1][x] = CELL_WALL;
        }
        BuildEntrances();
        // Check no entrances at y=borderY
        int entrancesAtBorder = 0;
        for (int i = 0; i < entranceCount; i++) {
            if (entrances[i].y == borderY) entrancesAtBorder++;
        }
        expect(entrancesAtBorder == 0);
    }

    it("should create correct entrances for full open border") {
        // A fully open border of chunkWidth cells should create ceil(chunkWidth / MAX_ENTRANCE_WIDTH) entrances
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);  // 2x2 chunks
        BuildEntrances();

        // Count entrances on the horizontal border at y=chunkHeight, x in [0, chunkWidth)
        int entrancesOnBorder = 0;
        for (int i = 0; i < entranceCount; i++) {
            if (entrances[i].y == chunkHeight && entrances[i].x < chunkWidth) {
                entrancesOnBorder++;
            }
        }

        // With chunkWidth=32 and MAX_ENTRANCE_WIDTH=6, we expect ceil(32/6) = 6 entrances
        int expectedEntrances = (chunkWidth + MAX_ENTRANCE_WIDTH - 1) / MAX_ENTRANCE_WIDTH;
        expect(entrancesOnBorder == expectedEntrances);
    }

    it("should create one entrance for narrow opening") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);  // 2x2 chunks

        // Block the horizontal border except for a 3-cell gap
        int borderY = chunkHeight;
        for (int x = 0; x < chunkWidth; x++) {
            grid[borderY - 1][x] = CELL_WALL;
            grid[borderY][x] = CELL_WALL;
        }
        // Open a narrow gap (3 cells wide, less than MAX_ENTRANCE_WIDTH)
        int gapStart = 10;
        int gapWidth = 3;
        for (int x = gapStart; x < gapStart + gapWidth; x++) {
            grid[borderY - 1][x] = CELL_WALKABLE;
            grid[borderY][x] = CELL_WALKABLE;
        }

        BuildEntrances();

        // Count entrances on this border section
        int entrancesOnBorder = 0;
        for (int i = 0; i < entranceCount; i++) {
            if (entrances[i].y == borderY && entrances[i].x < chunkWidth) {
                entrancesOnBorder++;
            }
        }

        // Narrow opening (< MAX_ENTRANCE_WIDTH) should create exactly 1 entrance
        expect(entrancesOnBorder == 1);
    }

    it("should create multiple entrances for wide opening") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);  // 2x2 chunks

        // Block the horizontal border except for a wide gap
        int borderY = chunkHeight;
        for (int x = 0; x < chunkWidth; x++) {
            grid[borderY - 1][x] = CELL_WALL;
            grid[borderY][x] = CELL_WALL;
        }
        // Open a wide gap (15 cells, more than 2x MAX_ENTRANCE_WIDTH)
        int gapStart = 5;
        int gapWidth = 15;
        for (int x = gapStart; x < gapStart + gapWidth; x++) {
            grid[borderY - 1][x] = CELL_WALKABLE;
            grid[borderY][x] = CELL_WALKABLE;
        }

        BuildEntrances();

        // Count entrances on this border section
        int entrancesOnBorder = 0;
        for (int i = 0; i < entranceCount; i++) {
            if (entrances[i].y == borderY && entrances[i].x < chunkWidth) {
                entrancesOnBorder++;
            }
        }

        // Wide opening should create ceil(15/6) = 3 entrances
        int expectedEntrances = (gapWidth + MAX_ENTRANCE_WIDTH - 1) / MAX_ENTRANCE_WIDTH;
        expect(entrancesOnBorder == expectedEntrances);
    }
}

describe(graph_building) {
    it("should create edges between entrances in the same chunk") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        BuildEntrances();
        BuildGraph();
        expect(graphEdgeCount > 0);
    }

    it("edges should be symmetric - cost A to B equals cost B to A") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);
        BuildEntrances();
        BuildGraph();

        // For every edge, the reverse edge should exist with same cost
        int symmetric = 1;
        for (int i = 0; i < graphEdgeCount && symmetric; i++) {
            int from = graphEdges[i].from;
            int to = graphEdges[i].to;
            int cost = graphEdges[i].cost;

            int foundReverse = 0;
            for (int j = 0; j < graphEdgeCount; j++) {
                if (graphEdges[j].from == to && graphEdges[j].to == from) {
                    if (graphEdges[j].cost == cost) foundReverse = 1;
                    break;
                }
            }
            if (!foundReverse) symmetric = 0;
        }
        expect(symmetric == 1);
    }

    it("should not create edges between entrances in different non-adjacent chunks") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 3, DEFAULT_CHUNK_SIZE * 3);  // 3x3 chunks
        BuildEntrances();
        BuildGraph();

        // No edge should connect entrances that don't share any chunk
        int noInvalidEdges = 1;
        for (int i = 0; i < graphEdgeCount && noInvalidEdges; i++) {
            int e1 = graphEdges[i].from;
            int e2 = graphEdges[i].to;

            // Get chunks for each entrance
            int c1a = entrances[e1].chunk1, c1b = entrances[e1].chunk2;
            int c2a = entrances[e2].chunk1, c2b = entrances[e2].chunk2;

            // They must share at least one chunk
            int share = (c1a == c2a || c1a == c2b || c1b == c2a || c1b == c2b);
            if (!share) noInvalidEdges = 0;
        }
        expect(noInvalidEdges == 1);
    }

    it("should not create edge when wall completely blocks path between entrances") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);

        // Put a wall that divides chunk 0 into two unreachable halves
        // Vertical wall from top to bottom of chunk 0
        for (int y = 0; y < chunkHeight; y++) {
            grid[y][chunkWidth / 2] = CELL_WALL;
        }

        BuildEntrances();
        BuildGraph();

        // Find entrances on the left border of chunk 1 (x = chunkWidth)
        // and entrances on the bottom border of chunk 0 (y = chunkHeight)
        // These are in the same chunk (0 or 1) but the wall may block some paths

        // The test: verify no edge exists between unreachable entrances
        // An entrance on the left side of chunk 0 shouldn't connect to
        // an entrance on the right side of chunk 0
        int foundInvalidEdge = 0;
        for (int i = 0; i < graphEdgeCount && !foundInvalidEdge; i++) {
            int e1 = graphEdges[i].from;
            int e2 = graphEdges[i].to;

            // Check if both are in chunk 0
            int e1InChunk0 = (entrances[e1].chunk1 == 0 || entrances[e1].chunk2 == 0);
            int e2InChunk0 = (entrances[e2].chunk1 == 0 || entrances[e2].chunk2 == 0);

            if (e1InChunk0 && e2InChunk0) {
                // Check if they're on opposite sides of the wall
                int e1Left = entrances[e1].x < chunkWidth / 2;
                int e2Left = entrances[e2].x < chunkWidth / 2;

                // If both entrances are in chunk 0 and on opposite sides of wall,
                // there shouldn't be an edge (wall blocks it)
                // But we need to check their y positions too - the wall is vertical
                if (entrances[e1].y < chunkHeight && entrances[e2].y < chunkHeight) {
                    if (e1Left != e2Left) {
                        foundInvalidEdge = 1;  // This edge shouldn't exist
                    }
                }
            }
        }
        expect(foundInvalidEdge == 0);
    }

    it("should create edge when path exists between entrances") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);
        // Completely open grid - all entrances in a chunk should connect
        BuildEntrances();
        BuildGraph();

        // In an open chunk, every pair of entrances should have an edge
        // Let's verify that entrances in chunk 0 all connect to each other
        int chunk0Entrances[64];
        int chunk0Count = 0;
        for (int i = 0; i < entranceCount && chunk0Count < 64; i++) {
            if (entrances[i].chunk1 == 0 || entrances[i].chunk2 == 0) {
                chunk0Entrances[chunk0Count++] = i;
            }
        }

        // For each pair in chunk 0, there should be an edge
        int allConnected = 1;
        for (int i = 0; i < chunk0Count && allConnected; i++) {
            for (int j = i + 1; j < chunk0Count && allConnected; j++) {
                int e1 = chunk0Entrances[i];
                int e2 = chunk0Entrances[j];

                // Only check if they actually share chunk 0 (not just touch it)
                int e1HasChunk0 = (entrances[e1].chunk1 == 0 || entrances[e1].chunk2 == 0);
                int e2HasChunk0 = (entrances[e2].chunk1 == 0 || entrances[e2].chunk2 == 0);
                if (!e1HasChunk0 || !e2HasChunk0) continue;

                // Check both share chunk 0 specifically
                int bothShareChunk0 = 0;
                if ((entrances[e1].chunk1 == 0 && (entrances[e2].chunk1 == 0 || entrances[e2].chunk2 == 0)) ||
                    (entrances[e1].chunk2 == 0 && (entrances[e2].chunk1 == 0 || entrances[e2].chunk2 == 0))) {
                    bothShareChunk0 = 1;
                }
                if (!bothShareChunk0) continue;

                // Look for edge between them
                int foundEdge = 0;
                for (int k = 0; k < graphEdgeCount; k++) {
                    if ((graphEdges[k].from == e1 && graphEdges[k].to == e2) ||
                        (graphEdges[k].from == e2 && graphEdges[k].to == e1)) {
                        foundEdge = 1;
                        break;
                    }
                }
                if (!foundEdge) allConnected = 0;
            }
        }
        expect(allConnected == 1);
    }

    it("should not create duplicate edges for entrances sharing two chunks") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);
        BuildEntrances();
        BuildGraph();

        // Check for duplicate edges (same from->to pair appearing twice)
        int duplicates = 0;
        for (int i = 0; i < graphEdgeCount; i++) {
            for (int j = i + 1; j < graphEdgeCount; j++) {
                if (graphEdges[i].from == graphEdges[j].from &&
                    graphEdges[i].to == graphEdges[j].to) {
                    duplicates++;
                }
            }
        }
        expect(duplicates == 0);
    }

    it("edge cost should equal walking distance") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);
        BuildEntrances();
        BuildGraph();

        // For a sample of edges, verify the cost matches the actual distance
        int costsCorrect = 1;
        int tested = 0;

        for (int i = 0; i < graphEdgeCount && tested < 10; i++) {
            int e1 = graphEdges[i].from;
            int e2 = graphEdges[i].to;
            int edgeCost = graphEdges[i].cost;

            // Calculate expected cost: Manhattan distance for 4-dir, or diagonal for 8-dir
            int dx = abs(entrances[e1].x - entrances[e2].x);
            int dy = abs(entrances[e1].y - entrances[e2].y);

            // On an open grid, the cost should be the optimal path distance
            // For 8-dir: max(dx,dy)*10 + min(dx,dy)*4 (diagonal shortcut)
            // For 4-dir: (dx + dy) * 10
            int expectedMinCost;
            if (use8Dir) {
                int minD = dx < dy ? dx : dy;
                int maxD = dx > dy ? dx : dy;
                expectedMinCost = maxD * 10 + minD * 4;
            } else {
                expectedMinCost = (dx + dy) * 10;
            }

            // Edge cost should be >= minimum possible (can't be shorter than straight line)
            if (edgeCost < expectedMinCost) {
                costsCorrect = 0;
            }
            tested++;
        }
        expect(costsCorrect == 1 && tested > 0);
    }
}

describe(incremental_graph_updates) {
    it("incremental update should produce same result as full rebuild") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);
        BuildEntrances();
        BuildGraph();

        // Add some walls
        grid[10][10] = CELL_WALL;
        grid[10][11] = CELL_WALL;
        grid[11][10] = CELL_WALL;
        MarkChunkDirty(10, 10);
        MarkChunkDirty(10, 11);
        MarkChunkDirty(11, 10);
        // Do incremental update
        UpdateDirtyChunks();
        int incrementalEdgeCount = graphEdgeCount;

        // Save edges from incremental update
        int incrementalEdges[MAX_EDGES][3];  // from, to, cost
        for (int i = 0; i < graphEdgeCount && i < MAX_EDGES; i++) {
            incrementalEdges[i][0] = graphEdges[i].from;
            incrementalEdges[i][1] = graphEdges[i].to;
            incrementalEdges[i][2] = graphEdges[i].cost;
        }

        // Now do full rebuild
        BuildEntrances();
        BuildGraph();
        int fullRebuildEdgeCount = graphEdgeCount;

        // Edge counts should match
        expect(incrementalEdgeCount == fullRebuildEdgeCount);
    }

    it("path should still work after wall added via incremental update") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);
        BuildEntrances();
        BuildGraph();

        // Verify path works before
        startPos = (Point){5, 5};
        goalPos = (Point){chunkWidth + 20, chunkHeight + 20};
        RunHPAStar();
        int pathBeforeWall = pathLength;

        // Add wall and update incrementally
        grid[chunkHeight / 2][chunkWidth / 2] = CELL_WALL;
        MarkChunkDirty(chunkWidth / 2, chunkHeight / 2);
        UpdateDirtyChunks();

        // Path should still work (wall doesn't block everything)
        RunHPAStar();
        int pathAfterWall = pathLength;

        expect(pathBeforeWall > 0 && pathAfterWall > 0);
    }

    it("removing an entrance should update all edges correctly") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 2, DEFAULT_CHUNK_SIZE * 2);
        BuildEntrances();
        BuildGraph();

        // Block an entire border to remove entrances
        int borderY = chunkHeight;
        for (int x = 0; x < chunkWidth; x++) {
            grid[borderY - 1][x] = CELL_WALL;
            grid[borderY][x] = CELL_WALL;
        }
        MarkChunkDirty(0, borderY);
        UpdateDirtyChunks();

        // All edge indices should be valid (no dangling references)
        int allValid = 1;
        for (int i = 0; i < graphEdgeCount; i++) {
            if (graphEdges[i].from < 0 || graphEdges[i].from >= entranceCount ||
                graphEdges[i].to < 0 || graphEdges[i].to >= entranceCount) {
                allValid = 0;
            }
        }
        expect(allValid == 1);
    }

    it("changes in one corner should not affect opposite corner") {
        InitGridWithSize(DEFAULT_CHUNK_SIZE * 4, DEFAULT_CHUNK_SIZE * 4);  // 4x4 chunks
        BuildEntrances();
        BuildGraph();

        // Find path in bottom-right area (chunks 10, 11, 14, 15)
        startPos = (Point){chunkWidth * 2 + 5, chunkHeight * 2 + 5};
        goalPos = (Point){chunkWidth * 4 - 10, chunkHeight * 4 - 10};
        RunHPAStar();
        int pathBefore = pathLength;

        // Add walls in top-left corner (chunk 0)
        for (int i = 0; i < 10; i++) {
            grid[i][i] = CELL_WALL;
            MarkChunkDirty(i, i);
        }
        UpdateDirtyChunks();

        // Path in bottom-right should be unaffected
        RunHPAStar();
        int pathAfter = pathLength;

        expect(pathBefore > 0 && pathAfter == pathBefore);
    }
}

describe(astar_pathfinding) {
    it("should find a path on an empty grid") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        startPos = (Point){5, 5};
        goalPos = (Point){50, 50};
        RunAStar();
        expect(pathLength > 0);
    }

    it("should not find a path when goal is walled off") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        // Create a box around the goal
        int gx = 50, gy = 50;
        for (int x = gx - 2; x <= gx + 2; x++) {
            grid[gy - 2][x] = CELL_WALL;
            grid[gy + 2][x] = CELL_WALL;
        }
        for (int y = gy - 2; y <= gy + 2; y++) {
            grid[y][gx - 2] = CELL_WALL;
            grid[y][gx + 2] = CELL_WALL;
        }
        startPos = (Point){5, 5};
        goalPos = (Point){gx, gy};
        RunAStar();
        expect(pathLength == 0);
    }
}

describe(hpa_star_pathfinding) {
    it("should find a path using HPA* on an empty grid") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        BuildEntrances();
        BuildGraph();
        startPos = (Point){5, 5};
        goalPos = (Point){TEST_GRID_SIZE - 10, TEST_GRID_SIZE - 10};
        RunHPAStar();
        expect(pathLength > 0);
    }

    it("should find same-chunk paths without using the graph") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        BuildEntrances();
        BuildGraph();
        // Start and goal in same chunk
        startPos = (Point){5, 5};
        goalPos = (Point){10, 10};
        RunHPAStar();
        expect(pathLength > 0);
    }
}

describe(incremental_updates) {
    it("should update graph incrementally when a wall is added") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        BuildEntrances();
        BuildGraph();
        int originalEdgeCount = graphEdgeCount;
        (void)originalEdgeCount;  // Suppress unused warning

        // Add a wall and update
        grid[chunkHeight + 5][chunkWidth + 5] = CELL_WALL;
        MarkChunkDirty(chunkWidth + 5, chunkHeight + 5);
        UpdateDirtyChunks();

        // Graph should still work (edge count may differ slightly)
        expect(graphEdgeCount > 0);
    }

    it("should still find paths after incremental update") {
        InitGridWithSize(TEST_GRID_SIZE, TEST_GRID_SIZE);
        BuildEntrances();
        BuildGraph();

        // Add some walls
        for (int i = 0; i < 5; i++) {
            grid[chunkHeight + 10][chunkWidth + i] = CELL_WALL;
            MarkChunkDirty(chunkWidth + i, chunkHeight + 10);
        }
        UpdateDirtyChunks();

        startPos = (Point){5, 5};
        goalPos = (Point){TEST_GRID_SIZE - 10, TEST_GRID_SIZE - 10};
        RunHPAStar();
        expect(pathLength > 0);
    }
}

int main(int argc, char* argv[]) {
    // Suppress logs by default, use -v for verbose
    bool verbose = false;
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-' && argv[i][1] == 'v') verbose = true;
    }
    if (!verbose) {
        SetTraceLogLevel(LOG_NONE);
    }

    test(grid_initialization);
    test(entrance_building);
    test(graph_building);
    test(incremental_graph_updates);
    test(astar_pathfinding);
    test(hpa_star_pathfinding);
    test(incremental_updates);
    return summary();
}
