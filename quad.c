/******************************************************************************
* This handle all quad processing API
* Version
* 1.0      iqbal
*****************************************************************************/
#include "gwan.h"

// prototype


unsigned int long_to_tileX(double lon, int zoom) {
     return (int)(floor((lon + 180.0) / 360.0 * pow(2.0, z)));
}

unsigned int lat_to_tileY(double lat, int zoom) {
     return (int)(floor((1.0 - log( tan(lat * M_PI/180.0) + 1.0 / cos(lat * M_PI/180.0)) / M_PI) / 2.0 * pow(2.0, z)));
}

double tileX_to_long(int x, int zoom) {
     return x / pow(2.0, z) * 360.0 - 180;
}

double tileY_to_lat(int y, int zoom) {
     double n = M_PI - 2.0 * M_PI * y / pow(2.0, z);
     return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

// Main execution
int main (int argc, char *argv[])
{

}
