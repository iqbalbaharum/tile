
/******************************************************************************
 * Tile-related data structure
 * Version
 * 1.0      iqbal
 *****************************************************************************/

#ifndef TILES_H
#define TILES_H

// #define PATHFINDING_ZOOM 23
// #define MAX_ZOOM 25

typedef struct {
    double latitude;
    double longitude;
    int tileX;
    int tileY;
} tile_t;

typedef enum {
    UP=0,
    DOWN,
    LEFT,
    RIGHT
} move_to_t;

typedef enum {
     F_NONE = 0,
     F_UP = 1,
     F_DOWN,
     F_LEFT,
     F_RIGHT,
     F_UPLEFT,
     F_DOWNLEFT,
     F_UPRIGHT,
     F_DOWNRIGHT
} nav_flow_t;

#endif
