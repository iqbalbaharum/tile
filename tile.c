
/******************************************************************************
* This handle all tile related API
* Version
* 1.0      iqbal
*****************************************************************************/
#include "gwan.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"

#include "tiles.h"

#define PATHFINDING_ZOOM 23
#define MAX_PATH 1024
#define R 6371000
#define TO_RAD (3.1415926536 / 180)
#define TO_DEG (180 / 3.1415926536)

// prototype
void param_split(tile_t **tile, char *param);
char * printQuadKey(xbuf_t *reply, int *pKey);
int *convertToQuadKey(double lat, double lon, int zoom);
tile_t *convertToCoordinate(int *pKey);
//
int long2tilex(double lon, int z);
int lat2tiley(double lat, int z);
double tilex2long(int x, int z);
double tiley2lat(int y, int z);
int *tileXYToQuadKey(int lat, int lon, int z);
tile_t *QuadKeyTotileXY(int *pKey);

// Path
int getCommonZoom(int *pSourcekey, int *pDestinationKey);
nav_flow_t getFlowMovement(int *pSourceKey, int *pDestinationKey);
int * pathHorizontal(int *pSourceKey, move_to_t move);
int * pathVertical(int *pSourceKey, move_to_t move);
void horizontalTable(int *pPath, move_to_t move, int level);
void verticalTable(int *pPath, move_to_t move, int level);
void insertPath(int **ppPath, int *pNewPath, int *pos);
int findPath(xbuf_t *reply, int **ppPath, int *pNeedlePath, int size);
int mergePath(xbuf_t *reply, int **ppPath, int **ppSourcePath, int **ppDestinationPath, int srcPos, int dstPos);

// distance
double distanceInMeter(double th1, double ph1, double th2, double ph2);
double coordinateToBearing(double latitude1, double longitude1, double latitude2, double longitude2);

// Main execution
int main (int argc, char *argv[])
{
     xbuf_t *reply = get_reply(argv);

     http_t *http = (void*)get_env(argv, HTTP_HEADERS);

     if(http->h_method == HTTP_GET) {

          // get list
          char *source = 0, *destination = 0;
          get_arg("src=", &source, argc, argv);
          get_arg("dest=", &destination, argc, argv);

          tile_t *source_tile = (tile_t *) malloc(sizeof(tile_t));
          tile_t *destination_tile = (tile_t *) malloc(sizeof(tile_t));

          param_split(&source_tile, source);
          param_split(&destination_tile, destination);

          int *pSourceKey = NULL;
          int *pDestinationKey = NULL;
          int **ppPath = NULL;
          int **ppFromSourcePath = NULL;
          int **ppFromDestinationPath = NULL;
          int totalSource = 0;
          int totalDestination = 0;
          int totalPaths = 0;

          nav_flow_t flow = F_NONE;

          tile_t *pTiles = NULL;

          ppPath = malloc(sizeof(int*) * MAX_PATH * MAX_PATH);
          ppFromSourcePath = malloc(sizeof(int*) * MAX_PATH);
          ppFromDestinationPath = malloc(sizeof(int*) * MAX_PATH);

          pSourceKey = convertToQuadKey(source_tile->latitude, source_tile->longitude, PATHFINDING_ZOOM);
          pDestinationKey = convertToQuadKey(destination_tile->latitude, destination_tile->longitude, PATHFINDING_ZOOM);

          if(pSourceKey == NULL || pDestinationKey == NULL) {
               // TERMINATE
          }

          insertPath(ppFromSourcePath, pSourceKey, &totalSource);
          insertPath(ppFromDestinationPath, pDestinationKey, &totalDestination);

          // // find common level to decide which is the navigation flow
          flow = getFlowMovement(pSourceKey, pDestinationKey);
          // xbuf_xcat(reply, "%d\n", flow);
          if(flow == F_NONE) {
               // TERMINATE
               return HTTP_400_BAD_REQUEST;
          }

          unsigned int index = 0;
          int commonSourceIndex = 0;
          int commonDestinationIndex = 0;

          move_to_t srcMove = DOWN;
          move_to_t dstMove = UP;
          switch(flow) {
               case F_UP:
                    srcMove = UP;
                    dstMove = DOWN;
               break;
               case F_DOWN:
                    srcMove = DOWN;
                    dstMove = UP;
               break;
               case F_LEFT:
                    srcMove = LEFT;
                    dstMove = RIGHT;
               break;
               case F_RIGHT:
                    srcMove = RIGHT;
                    dstMove = LEFT;
               break;
               case F_UPLEFT:
                    srcMove = UP;
                    dstMove = RIGHT;
               break;
               case F_DOWNLEFT:
                    srcMove = DOWN;
                    dstMove = RIGHT;
               break;
               case F_UPRIGHT:
                    srcMove = UP;
                    dstMove = LEFT;
               break;
               case F_DOWNRIGHT:
                    srcMove = DOWN;
                    dstMove = LEFT;
               break;
               case F_NONE:
               break;
          }

          while(index < MAX_PATH) {

               // xbuf_xcat(reply, "%d\n", index);

               if(ppFromSourcePath[index] == NULL || ppFromDestinationPath[index] == NULL) {
                    break;
               }

               int *pNewSourcePath = NULL;
               if(srcMove == UP || srcMove == DOWN) {
                    pNewSourcePath = pathVertical(ppFromSourcePath[index], srcMove);
               } else {
                    pNewSourcePath = pathHorizontal(ppFromSourcePath[index], srcMove);
               }


               int *pNewDestinationPath = NULL;
               if(dstMove == UP || dstMove == DOWN) {
                    pNewDestinationPath = pathVertical(ppFromDestinationPath[index], dstMove);
               } else {
                    pNewDestinationPath = pathHorizontal(ppFromDestinationPath[index], dstMove);
               }

               // printQuadKey(reply, pNewSourcePath);
               // printQuadKey(reply, pNewDestinationPath);

               // check if can find similar path inside the other side paths
               // if YES: Split array and comibine together
               // NO: keep adding on box
               commonDestinationIndex = findPath(reply, ppFromDestinationPath, pNewSourcePath, totalDestination);

               // insert new path originated from Source
               if(commonDestinationIndex == -1) {
                    insertPath(ppFromSourcePath, pNewSourcePath, &totalSource);
               } else {
                    // get position for the similar object
                    commonSourceIndex = findPath(reply, ppFromSourcePath, pNewSourcePath, totalSource);
                    totalPaths = mergePath(reply, ppPath, ppFromSourcePath, ppFromDestinationPath, totalSource, commonDestinationIndex);
                    break;
               }

               // printQuadKey(reply, ppFromSourcePath[1]);

               commonSourceIndex = findPath(reply, ppFromSourcePath, pNewDestinationPath, totalSource);
               // insert new path originated from Destination
               if(commonSourceIndex == -1) {
                    insertPath(ppFromDestinationPath, pNewDestinationPath, &totalDestination);
               } else {
                    totalPaths = mergePath(reply, ppPath, ppFromSourcePath, ppFromDestinationPath, commonSourceIndex, totalDestination);
                    break;
               }

               ++index;
          }

          if(ppPath != NULL) {

               // building the root node
               // result
               //   - response                    NODE
               //   - origin
               //     - destination
               //   - steps                       NODE
               //        - (array)
               //             - start
               //                  - lat
               //                  - lon
               //                  - key
               //             - end
               //                  - lat
               //                  - lon
               //                  - key
               //             - distance
               //             - direction
               jsn_t *results = jsn_add_node(0, "result");


               // add first item
               jsn_add_string(results, "response", "OK");

               //
               jsn_add_real(results, "origin", source_tile->latitude);
               jsn_add_real(results, "destination", destination_tile->latitude);

               // add a node
               jsn_t *steps = jsn_add_array(results, "steps", 0);
               // loop paths to add to navigation
               // navigation is box-by-box
               jsn_t *node;

               int i;
               for(i = 0; i < (totalPaths - 1); ++i) {

                    xbuf_t pathBuf;
                    xbuf_init(&pathBuf);

                    node = jsn_add_node(steps, "");
                    if(ppPath[i] != NULL) {

                         double oriLat, dstLat;
                         double oriLon, dstLon;

                         tile_t *startTile = malloc(sizeof(tile_t));
                         startTile = convertToCoordinate(ppPath[i]);

                         tile_t *endTile = malloc(sizeof(tile_t));
                         endTile = convertToCoordinate(ppPath[i + 1]);

                         // for start point, take the requested coordinate
                         jsn_t *start = jsn_add_node(node, "start");
                         if(i == 0) {
                              oriLat = source_tile->latitude;
                              oriLon = source_tile->longitude;
                              jsn_add_string(start, "key", printQuadKey(&pathBuf, pSourceKey));
                         } else {
                              oriLat = startTile->latitude;
                              oriLon = startTile->longitude;
                              jsn_add_string(start, "key", printQuadKey(&pathBuf, ppPath[i]));
                         }

                         jsn_add_real(start, "lat", oriLat);
                         jsn_add_real(start, "lon", oriLon);


                         xbuf_empty(&pathBuf);

                         // for end point, take the destination coordinate
                         jsn_t *end  = jsn_add_node(node, "end");
                         if((i + 1) == totalPaths) {
                              dstLat = destination_tile->latitude;
                              dstLon = destination_tile->longitude;
                              jsn_add_string(end, "key", printQuadKey(&pathBuf, pDestinationKey));
                         } else {
                              dstLat = endTile->latitude;
                              dstLon = endTile->longitude;
                              jsn_add_string(end, "key", printQuadKey(&pathBuf, ppPath[i + 1]));
                         }

                         jsn_add_real(end, "lat", dstLat);
                         jsn_add_real(end, "lon", dstLon);

                         // Using harversine Formula to calculate distance
                         // between 'stop' to 'next stop'
                         jsn_add_real(node, "distance_in_meter", round(distanceInMeter(oriLat, oriLon, dstLat, dstLon)));
                         jsn_add_real(node, "bearing", coordinateToBearing(oriLat, oriLon, dstLat, dstLon));

                         xbuf_empty(&pathBuf);

                         free(endTile);
                         free(startTile);
                    }
               }

               xbuf_t xbuf;
               xbuf_init(&xbuf);
               char *text = jsn_totext(&xbuf, results, 0);
               xbuf_xcat(reply, "%s", text);
          }

          free(pTiles);
          free(ppPath);
          free(ppFromDestinationPath);
          free(ppFromSourcePath);
          free(pDestinationKey);
          free(pSourceKey);
          free(source_tile);
          free(destination_tile);
     }

     return HTTP_200_OK;
}

/******************************************************************************
* split URL parameters
*****************************************************************************/
void param_split(tile_t **tile, char *param) {

     int index = 0;
     char separator[2] = ",";
     char *token;

     token = strtok(param, separator);

     while(index < 2) {

          double coord = atof(token);

          if(index == 0) {
               (*tile)->latitude = coord;
          } else {
               (*tile)->longitude = coord;
          }

          token = strtok(NULL, separator);

          ++index;
     }
}

char * printQuadKey(xbuf_t *reply, int *pKey) {

     if(pKey == NULL) {
          return NULL;
     }

     char *key;
     key = (char *) malloc(PATHFINDING_ZOOM);
     memset(key, '\0', PATHFINDING_ZOOM + 1);
     //"%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\n" 23
     //%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d 19
     key = xbuf_xcat(reply, "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d",
          pKey[0],
          pKey[1],
          pKey[2],
          pKey[3],
          pKey[4],
          pKey[5],
          pKey[6],
          pKey[7],
          pKey[8],
          pKey[9],
          pKey[10],
          pKey[11],
          pKey[12],
          pKey[13],
          pKey[14],
          pKey[15],
          pKey[16],
          pKey[17],
          pKey[18],
          pKey[19]
     );

     return key;
}

/******************************************************************************
* quadkey
*****************************************************************************/
int *convertToQuadKey(double lat, double lon, int zoom) {

     int   pos_x = 0,
     pos_y = 0;

     int *qkey = NULL;

     pos_x = long2tilex(lon, zoom);
     pos_y = lat2tiley(lat, zoom);

     qkey = tileXYToQuadKey(pos_x, pos_y, zoom);

     return qkey;
}

tile_t *convertToCoordinate(int *pKey) {

     tile_t *pTile = malloc(sizeof(tile_t));
     if(pTile) {
          pTile = QuadKeyTotileXY(pKey);
          // get coordinate
          if(pTile) {
               pTile->longitude = tilex2long(pTile->tileX, PATHFINDING_ZOOM);
               pTile->latitude = tiley2lat(pTile->tileY, PATHFINDING_ZOOM);
          }
     }

     return pTile;
}

/******************************************************************************
* tile
*****************************************************************************/
int long2tilex(double lon, int z) {
     return (int)(floor((lon + 180.0) / 360.0 * pow(2.0, z)));
}

int lat2tiley(double lat, int z) {
     return (int)(floor((1.0 - log( tan(lat * M_PI/180.0) + 1.0 / cos(lat * M_PI/180.0)) / M_PI) / 2.0 * pow(2.0, z)));
}

double tilex2long(int x, int z) {
     return x / pow(2.0, z) * 360.0 - 180;
}

double tiley2lat(int y, int z) {
     double n = M_PI - 2.0 * M_PI * y / pow(2.0, z);
     return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

// convert tiles x,y to quadkey
int *tileXYToQuadKey(int x, int y, int z) {

     // build array based which can fill all PATHFINDING_ZOOM
     int *key = (int *) malloc(PATHFINDING_ZOOM * sizeof(int));
     memset(key, -1, PATHFINDING_ZOOM);

     int index;
     for(index = z; index > 0; --index) {

          // char digit = '0';
          int digit = 0;

          int mask = 1 << (index - 1);
          if ((x & mask) != 0)
          {
               digit++;
          }
          if ((y & mask) != 0)
          {
               digit++;
               digit++;
          }

          // store in reverse
          key[PATHFINDING_ZOOM - index] = digit;
     }

     return key;
}

tile_t *QuadKeyTotileXY(int *pKey) {

     tile_t *tile = NULL;

     tile = (tile_t *) malloc(sizeof(tile_t));

     if(!tile) {
          return NULL;
     }

     // set default value for tile X and tile Y
     tile->tileX = 0;
     tile->tileY = 0;

     int index;
     for(index = PATHFINDING_ZOOM; index > 0; --index) {

          int mask = 1 << (index - 1);
          switch(pKey[PATHFINDING_ZOOM - index]) {
               case 0:
               break;
               case 1:
                    tile->tileX |= mask;
               break;
               case 2:
                    tile->tileY |= mask;
               break;
               case 3:
                    tile->tileX |= mask;
                    tile->tileY |= mask;
               break;
          }
     }

     return tile;
}

/******************************************************************************
* Detect navigation flow from A -> B
*****************************************************************************/
nav_flow_t getFlowMovement(int *pSourceKey, int *pDestinationKey) {

     nav_flow_t flow = F_NONE;

     int commonZoom;
     commonZoom = getCommonZoom(pSourceKey, pDestinationKey);

     int i;
     for(i = commonZoom; i < PATHFINDING_ZOOM; ++i) {

          if(pSourceKey[i] == pDestinationKey[i]) {
               continue;
          }

          switch(pSourceKey[i]) {
               case 0:

                    switch (pDestinationKey[i]) {
                         case 1:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_RIGHT;
                                   break;
                                   case F_DOWN:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_UP:
                                        flow = F_UPRIGHT;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                         case 2:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_DOWN;
                                   break;
                                   case F_RIGHT:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_LEFT:
                                        flow = F_DOWNLEFT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         case 3:
                              switch(flow) {
                                   case F_NONE:
                                   case F_RIGHT:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   case F_LEFT:
                                        flow = F_DOWNLEFT;
                                        goto EndWhile;
                                   break;
                                   case F_DOWN:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_UP:
                                        flow = F_UPRIGHT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                    }

               break;
               case 1:

                    switch (pDestinationKey[i]) {
                         case 0:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_RIGHT;
                                   break;
                                   case F_DOWN:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_UP:
                                        flow = F_UPRIGHT;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                         case 2:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_DOWNLEFT;
                                        goto EndWhile;
                                   break;
                                   case F_RIGHT:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_LEFT:
                                   case F_DOWN:
                                        flow = F_DOWNLEFT;
                                        goto EndWhile;
                                   break;
                                   case F_UP:
                                        flow = F_UPLEFT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         case 3:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_DOWN;
                                   break;
                                   case F_RIGHT:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_LEFT:
                                        flow = F_DOWNLEFT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                    }

               break;
               case 2:

                    switch (pDestinationKey[i]) {
                         case 0:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_UP;
                                   break;
                                   case F_LEFT:
                                        flow = F_UPLEFT;
                                        goto EndWhile;
                                   break;
                                   case F_RIGHT:
                                        flow = F_UPRIGHT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                         case 1:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_UPRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_UP:
                                   case F_RIGHT:
                                        flow = F_UPRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_LEFT:
                                        flow = F_UPLEFT;
                                        goto EndWhile;
                                   case F_DOWN:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         case 3:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_RIGHT;
                                   break;
                                   case F_UP:
                                        flow = F_UPRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_DOWN:
                                        flow = F_DOWNRIGHT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                    }

               break;
               case 3:

                    switch (pDestinationKey[i]) {
                         case 0:
                              switch(flow) {
                                   case F_RIGHT:
                                        flow = F_UPRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_LEFT:
                                   case F_NONE:
                                   case F_UP:
                                        flow = F_UPLEFT;
                                        goto EndWhile;
                                   break;
                                   case F_DOWN:
                                        flow = F_DOWNLEFT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                         case 1:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_UP;
                                   break;
                                   case F_RIGHT:
                                        flow = F_UPRIGHT;
                                        goto EndWhile;
                                   break;
                                   case F_LEFT:
                                        flow = F_UPLEFT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         case 2:
                              switch(flow) {
                                   case F_NONE:
                                        flow = F_LEFT;
                                   break;
                                   case F_UP:
                                        flow = F_UPLEFT;
                                        goto EndWhile;
                                   break;
                                   case F_DOWN:
                                        flow = F_DOWNLEFT;
                                        goto EndWhile;
                                   break;
                                   default:
                                        break;
                              }
                         break;
                    }

               break;
               default:
               break;
          }
     }

EndWhile: ;

     return flow;
}

/******************************************************************************
* Comparing tiles at multiple zoom and get similar tiles
*****************************************************************************/
int getCommonZoom(int *pSourceKey, int *pDestinationKey) {

     int i;
     for(i = 0; i < PATHFINDING_ZOOM; ++i) {

          if(pSourceKey[i] == pDestinationKey[i]) {
               continue;
          } else {
               break;
          }
     }

     return i;
}

/******************************************************************************
* Get the next tile from the current key using horzontal movement flow of LEFT
* or RIGHT
*****************************************************************************/
int * pathHorizontal(int *pSourceKey, move_to_t move) {

     if(move == UP || move == DOWN) {
          return NULL;
     }

     // build new path array
     int *pNewPath = malloc(PATHFINDING_ZOOM * sizeof(int));
     // firstly, copy data into new array
     if(pNewPath != NULL) {
          memcpy(pNewPath, pSourceKey, PATHFINDING_ZOOM * sizeof(int));
          horizontalTable(pNewPath, move, PATHFINDING_ZOOM - 1);
     }

     return pNewPath;
}

/******************************************************************************
* Get the next tile from the current key using vertical movement flow of UP
* or DOWN
*****************************************************************************/
int * pathVertical(int *pSourceKey, move_to_t move) {

     if(move == LEFT || move == RIGHT) {
          return NULL;
     }

     // build new path array
     int *pNewPath = (int *) malloc(PATHFINDING_ZOOM * sizeof(int));
     // firstly, copy data into new array
     // firstly, copy data into new array
     if(pNewPath != NULL) {
          memcpy(pNewPath, pSourceKey, PATHFINDING_ZOOM * sizeof(int));
          verticalTable(pNewPath, move, PATHFINDING_ZOOM - 1);
     }

     return pNewPath;
}

/*
* HORIZONTAL (LEFT-> RIGHT)
* --------------------------
* 00 -> 01
* 01 -> 10
* 10 -> 11
* 11 -> 10 (update 2 ZOOM before)
* 12 -> 13
* 13 -> 02 (update 2 ZOOM before)
* 20 -> 21
* 21 -> 30
* 22 -> 23
* 23 -> 32
* 30 -> 31
* 31 -> 20 (update 2 ZOOM before)
* 32 -> 33
* 33 -> 22 (update 2 ZOOM before)
* 10 -> 01
* 11 -> 10
* 12 -> 03
* 13 -> 12
* 20 -> 31 (update 2 ZOOM before)
* 21 -> 20
* 22 -> 33 (update 2 ZOOM before)
* 23 -> 22
* 30 -> 21
* 31 -> 30
* 32 -> 23
* 33 -> 32
*/
void horizontalTable(int *pPath, move_to_t move, int level) {

     // check two last key[PATHFINDING_ZOOM], key[PATHFINDING_ZOOM - 1]
     switch(pPath[level - 1]) {
          /*
           * HORIZONTAL (LEFT-> RIGHT)
           * --------------------------
           * 00 -> 01
           * 01 -> 10
           * 02 -> 03
           * 03 -> 12
           * HORIZONTAL (RIGHT -> LEFT)
           * --------------------------
           * 00 -> 11 (update 2 ZOOM before)
           * 01 -> 00
           * 02 -> 13 (update 2 ZOOM before)
           * 03 -> 02
           */
          case 0:
          switch(pPath[level]) {
               case 0:
                    if(move == RIGHT) {
                         pPath[level] = 1;
                    } else {
                         pPath[level - 1] = 1;
                         pPath[level] = 1;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    }
               break;
               case 1:
                    if(move == RIGHT) {
                         pPath[level - 1] = 1;
                         pPath[level] = 0;
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 2:
                    if(move == RIGHT) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 1;
                         pPath[level] = 3;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    }
               break;
               case 3:
                    if(move == RIGHT) {
                         pPath[level - 1] = 1;
                         pPath[level] = 2;
                    } else {
                         pPath[level] = 2;
                    }
               break;
          }

          break;


          /*
           * HORIZONTAL (LEFT-> RIGHT)
           * --------------------------
           * 10 -> 11
           * 11 -> 10 (update 2 ZOOM before)
           * 12 -> 13
           * 13 -> 02 (update 2 ZOOM before)
           * HORIZONTAL (RIGHT -> LEFT)
           * --------------------------
           * 10 -> 01
           * 11 -> 10
           * 12 -> 03
           * 13 -> 12
           */
          case 1:

          switch(pPath[level]) {
               case 0:
                    if(move == RIGHT) {
                         pPath[level] = 1;
                    } else {
                         pPath[level - 1] = 0;
                         pPath[level] = 1;
                    }
               break;
               case 1:
                    if(move == RIGHT) {
                         pPath[level - 1] = 1;
                         pPath[level] = 0;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 2:
                    if(move == RIGHT) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 0;
                         pPath[level] = 3;
                    }
               break;
               case 3:
                    if(move == RIGHT) {
                         pPath[level - 1] = 0;
                         pPath[level] = 2;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 2;
                    }
               break;
          }

          break;

          /*
           * HORIZONTAL (LEFT-> RIGHT)
           * --------------------------
           * 20 -> 21
           * 21 -> 30
           * 22 -> 23
           * 23 -> 32
           * HORIZONTAL (RIGHT -> LEFT)
           * --------------------------
           * 20 -> 31 (update 2 ZOOM before)
           * 21 -> 20
           * 22 -> 33 (update 2 ZOOM before)
           * 23 -> 22
           */
          case 2:

          switch(pPath[level]) {
               case 0:
                    if(move == RIGHT) {
                         pPath[level] = 1;
                    } else {
                         pPath[level - 1] = 3;
                         pPath[level] = 1;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    }
               break;
               case 1:
                    if(move == RIGHT) {
                         pPath[level - 1] = 3;
                         pPath[level] = 0;
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 2:
                    if(move == RIGHT) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 3;
                         pPath[level] = 3;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    }
               break;
               case 3:
                    if(move == RIGHT) {
                         pPath[level - 1] = 3;
                         pPath[level] = 2;
                    } else {
                         pPath[level] = 2;
                    }
               break;
          }

          break;

          /*
           * HORIZONTAL (LEFT-> RIGHT)
           * --------------------------
           * 30 -> 31
           * 31 -> 20 (update 2 ZOOM before)
           * 32 -> 33
           * 33 -> 22 (update 2 ZOOM before)
           * HORIZONTAL (RIGHT -> LEFT)
           * --------------------------
           * 30 -> 21
           * 31 -> 30
           * 32 -> 23
           * 33 -> 32
           */
          case 3:

          switch(pPath[level]) {
               case 0:
                    if(move == RIGHT) {
                         pPath[level] = 1;
                    } else {
                         pPath[level - 1] = 2;
                         pPath[level] = 1;
                    }
               break;
               case 1:
                    if(move == RIGHT) {
                         pPath[level - 1] = 2;
                         pPath[level] = 0;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 2:
                    if(move == RIGHT) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 2;
                         pPath[level] = 3;
                    }
               break;
               case 3:
                    if(move == RIGHT) {
                         pPath[level - 1] = 2;
                         pPath[level] = 2;
                         // update record level above
                         horizontalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 2;
                    }
               break;
          }

          break;
     }
}

/*
* VERTICAL (TOP -> BOTTOM)
* 00 -> 02
* 01 -> 03
* 02 -> 20
* 03 -> 21
* 10 -> 12
* 11 -> 13
* 12 -> 30
* 13 -> 31
* 20 -> 22
* 21 -> 33
* 22 -> 00 (update 2 ZOOM before)
* 23 -> 01 (update 2 ZOOM before)
* 30 -> 32
* 31 -> 33
* 32 -> 10 (update 2 ZOOM before)
* 33 -> 11 (update 2 ZOOM before)
*
* VERTICAL (BOTTOM -> TOP)
* 00 -> 22 (update 2 ZOOM before)
* 01 -> 23 (update 2 ZOOM before)
* 02 -> 00
* 03 -> 01
* 10 -> 32 (update 2 ZOOM before)
* 11 -> 33 (update 2 ZOOM before)
* 12 -> 10
* 13 -> 11
* 20 -> 02
* 21 -> 03
* 22 -> 20
* 23 -> 21
* 30 -> 12
* 31 -> 13
* 32 -> 30
* 33 -> 31
*/
void verticalTable(int *pPath, move_to_t move, int level) {

     switch(pPath[level - 1]) {

          /*
           * VERTICAL (TOP -> BOTTOM)
           * --------------------------
           * 00 -> 02
           * 01 -> 03
           * 02 -> 20
           * 03 -> 21
           * VERTICAL (BOTTOM -> TOP)
           * --------------------------
           * 00 -> 22 (update 2 ZOOM before)
           * 01 -> 23 (update 2 ZOOM before)
           * 02 -> 00
           * 03 -> 01
           */
          case 0:

          switch(pPath[level]) {
               case 0:
                    if(move == DOWN) {
                         pPath[level] = 2;
                    } else {
                         pPath[level - 1] = 2;
                         pPath[level] = 2;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    }
               break;
               case 1:
                    if(move == DOWN) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 2;
                         pPath[level] = 3;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    }
               break;
               case 2:
                    if(move == DOWN) {
                         pPath[level - 1] = 2;
                         pPath[level] = 0;
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 3:
                    if(move == DOWN) {
                         pPath[level - 1] = 2;
                         pPath[level] = 1;
                    } else {
                         pPath[level] = 1;
                    }
               break;
          }

          break;

          /*
          * VERTICAL (TOP -> BOTTOM)
          * 10 -> 12
          * 11 -> 13
          * 12 -> 30
          * 13 -> 31
          * VERTICAL (BOTTOM -> TOP)
          * 10 -> 32 (update 2 ZOOM before)
          * 11 -> 33 (update 2 ZOOM before)
          * 12 -> 10
          * 13 -> 11
          */
          case 1:
          switch(pPath[level]) {
               case 0:
                    if(move == DOWN) {
                         pPath[level] = 2;
                    } else {
                         pPath[level - 1] = 3;
                         pPath[level] = 2;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    }
               break;
               case 1:
                    if(move == DOWN) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 3;
                         pPath[level] = 3;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    }
               break;
               case 2:
                    if(move == DOWN) {
                         pPath[level - 1] = 3;
                         pPath[level] = 0;
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 3:
                    if(move == DOWN) {
                         pPath[level - 1] = 3;
                         pPath[level] = 1;
                    } else {
                         pPath[level] = 1;
                    }
               break;
          }

          break;

          /*
          * VERTICAL (TOP -> BOTTOM)
          * 20 -> 22
          * 21 -> 23
          * 22 -> 00 (update 2 ZOOM before)
          * 23 -> 01 (update 2 ZOOM before)
          * VERTICAL (BOTTOM -> TOP)
          * 20 -> 02
          * 21 -> 03
          * 22 -> 20
          * 23 -> 21
          */
          case 2:
          switch(pPath[level]) {
               case 0:
                    if(move == DOWN) {
                         pPath[level] = 2;
                    } else {
                         pPath[level - 1] = 0;
                         pPath[level] = 2;
                    }
               break;
               case 1:
                    if(move == DOWN) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 0;
                         pPath[level] = 3;
                    }
               break;
               case 2:
                    if(move == DOWN) {
                         pPath[level - 1] = 0;
                         pPath[level] = 0;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 3:
                    if(move == DOWN) {
                         pPath[level - 1] = 0;
                         pPath[level] = 1;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 1;
                    }
               break;
          }

          break;

          /*
          * VERTICAL (TOP -> BOTTOM)
          * 30 -> 32
          * 31 -> 33
          * 32 -> 10 (update 2 ZOOM before)
          * 33 -> 11 (update 2 ZOOM before)
          * VERTICAL (BOTTOM -> TOP)
          * 30 -> 12
          * 31 -> 13
          * 32 -> 30
          * 33 -> 31
          */
          case 3:
          switch(pPath[level]) {
               case 0:
                    if(move == DOWN) {
                         pPath[level] = 2;
                    } else {
                         pPath[level - 1] = 1;
                         pPath[level] = 2;
                    }
               break;
               case 1:
                    if(move == DOWN) {
                         pPath[level] = 3;
                    } else {
                         pPath[level - 1] = 1;
                         pPath[level] = 3;
                    }
               break;
               case 2:
                    if(move == DOWN) {
                         pPath[level - 1] = 1;
                         pPath[level] = 0;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 0;
                    }
               break;
               case 3:
                    if(move == DOWN) {
                         pPath[level - 1] = 1;
                         pPath[level] = 1;
                         // update record level above
                         verticalTable(pPath, move, (level - 2));
                    } else {
                         pPath[level] = 1;
                    }
               break;
          }

          break;
     }
}

/******************************************************************************
* Get the next tile from the current key using vertical movement flow of UP
* or DOWN
*****************************************************************************/
void insertPath(int **ppPath, int *pNewPath, int *pos) {

     // no similar key, so insert the new key to path
     ppPath[(*pos)] = malloc(PATHFINDING_ZOOM * sizeof(int));
     memcpy(ppPath[(*pos)], pNewPath, PATHFINDING_ZOOM * sizeof(int));
     // increase the position to reflect a new data
     ++(*pos);
}

/******************************************************************************
* Find generated path in the list
*****************************************************************************/
int findPath(xbuf_t *reply, int **ppPath, int *pNewPath, int size) {

     int index;
     for(index = 0; index < size; ++index) {

          if(ppPath[index] != NULL) {

               // PULL DOWN THE SPEED
               // NEED TO REPAIR
               bool bKey = true;
               int x;
               for(x = 0; x < PATHFINDING_ZOOM; ++x) {
                    if(ppPath[index][x] != pNewPath[x]) {
                         bKey = false;
                         break;
                    }
               }

               if(bKey) {
                    return index;
               }
          }
     }

     return -1;
}

/******************************************************************************
* merge Source array to destination array
* return size of the array
*****************************************************************************/
int mergePath(xbuf_t *reply, int **ppPath, int **ppSourcePath, int **ppDestinationPath, int srcPos, int dstPos) {

     // copy source to main array
     int soIndex;
     for(soIndex = 0; soIndex <= srcPos; ++soIndex) {
          ppPath[soIndex] = malloc(PATHFINDING_ZOOM * sizeof(int));
          memcpy(ppPath[soIndex], ppSourcePath[soIndex], PATHFINDING_ZOOM * sizeof(int));
     }

     int deIndex;
     for(deIndex = (dstPos - 1); deIndex >= 0; --deIndex) {
          ppPath[soIndex] = malloc(PATHFINDING_ZOOM * sizeof(int));
          memcpy(ppPath[soIndex], ppDestinationPath[deIndex], PATHFINDING_ZOOM * sizeof(int));
          ++soIndex;
     }

     // xbuf_xcat(reply, "%d\n", dstPos);
     // printQuadKey(reply, ppPath[2]);

     return soIndex;
}

/******************************************************************************
* Distance from 2 coordinate
*****************************************************************************/
double distanceInMeter(double th1, double ph1, double th2, double ph2)
{
	double dx, dy, dz;
	ph1 -= ph2;
	ph1 *= TO_RAD, th1 *= TO_RAD, th2 *= TO_RAD;

	dz = sin(th1) - sin(th2);
	dx = cos(ph1) * cos(th1) - cos(th2);
	dy = sin(ph1) * cos(th1);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;
}

double coordinateToBearing(double latitude1, double longitude1, double latitude2, double longitude2) {

     double x, y, dy;
     dy = (longitude2 - longitude1) * TO_RAD;
     latitude1 *= TO_RAD;
     latitude2 *= TO_RAD;

     y = sin(dy) * cos(latitude2);
     x = cos(latitude1)* sin(latitude2) - sin(latitude1) * cos(latitude2)* cos(dy);
     int degree = atan2(y, x) * TO_DEG;

     return (degree > 0) ? degree : (degree + 360);
}
