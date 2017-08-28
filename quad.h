
#ifndef QUAD_H
#define QUAD_H

#ifdef _cplusplus
extern "C" {
#endif

unsigned int long_to_tileX(double lon, int zoom);
unsigned int lat_to_tileY(double lat, int zoom);
double tileX_to_long(int x, int zoom);
double tileY_to_lat(int y, int zoom);

#ifdef _cplusplus
}
#endif

#endif
