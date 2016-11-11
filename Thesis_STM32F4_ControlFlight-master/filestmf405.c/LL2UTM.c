#include "LL2UTM.h"
#include "project.h"

//------------------------------
const double a = 6378137.0;                   // m, semi-major axis of the ellipsoid
const double b = 6356752.3142;                // m, semi-major axis of the ellipsoid
const double f = (a - b) / a;                 // flattening or ellipticity
const double ee1 = (a*a - b*b) / (a*a);       // first ecentricity
const double ee2 = (a*a - b*b) / (b*b);       // second ecentricity
const double n = (a - b) / (a + b);           //
const double FE = 500000.0;                   // false easting
const double k0 = 0.9996;                     // central scale factor for UTM
const double AA = a * (1.0-n) * (1.0 + n*n*(1.25 + 1.265625*n*n));
const double BB = 1.5*a * n * ((1.0-n)*(1.0 + 0.875*n*n) + 0.859375*n*n*n*n);
const double CC = 0.9375*a * n * n * (1.0 - n) * (1.0 + 0.75*n*n);
const double DD = 35.0 * a * n * n * n * ((1.0 - n) + 0.6875*n*n) / 48.0;
const double EE = 0.615234375 * n * n * n * n * (1.0 - n);
//------------------------------

//------------------------------
double rho;                                   // radius of curvature in the meridian
double v;                                     // radius of curvature in the prime vertical
double S;                                     // meridional arc
double nmn[5];                                // 1-n, n-n^2, n^2-n^3,n^3-n^4, n^4-n^5
double T[10];                                 //
double A[5];
double Long0;                                 // longitude of the center origin
double dLong[3];                              // difference of longitude from the central meridian, n = 0, 1, 2
double cosLat[7];                             // cos^n, n = 0, 1, 2, 3, 4, 5, 6
double e2cosLat[3];                           // (e2 * cosLat)^n = 0, 1, 2
double tanLat[3];                             // tan^n, n = 0, 1, 2, 3
//------------------------------

//------------------------------
int LatLong2UTM(const double Lat, const double Long, double *UTMNorthing, double *UTMEasting, char *UTMLetter)
{
	double LongTemp = (Long+180.0)-(int)((Long+180.0)/360.0)*360.0-180.0;       // -180.00 .. 179.9;
    int UTMZone = (int)((LongTemp + 180.0)/6.0) + 1;
    double Latrad = Lat * deg2rad;

	if((84.0 >= Lat) && (Lat >= 72.0)) *UTMLetter = 'X';
	else if((72.0 > Lat) && (Lat >= 64.0)) *UTMLetter = 'W';
	else if((64.0 > Lat) && (Lat >= 56.0)) *UTMLetter = 'V';
	else if((56.0 > Lat) && (Lat >= 48.0)) *UTMLetter = 'U';
	else if((48.0 > Lat) && (Lat >= 40.0)) *UTMLetter = 'T';
	else if((40.0 > Lat) && (Lat >= 32.0)) *UTMLetter = 'S';
	else if((32.0 > Lat) && (Lat >= 24.0)) *UTMLetter = 'R';
	else if((24.0 > Lat) && (Lat >= 16.0)) *UTMLetter = 'Q';
	else if((16.0 > Lat) && (Lat >= 8.0)) *UTMLetter = 'P';
	else if(( 8.0 > Lat) && (Lat >= 0.0)) *UTMLetter = 'N';
	else if(( 0.0 > Lat) && (Lat >= -8.0)) *UTMLetter = 'M';
	else if((-8.0 > Lat) && (Lat >= -16.0)) *UTMLetter = 'L';
	else if((-16.0 > Lat) && (Lat >= -24.0)) *UTMLetter = 'K';
	else if((-24.0 > Lat) && (Lat >= -32.0)) *UTMLetter = 'J';
	else if((-32.0 > Lat) && (Lat >= -40.0)) *UTMLetter = 'H';
	else if((-40.0 > Lat) && (Lat >= -48.0)) *UTMLetter = 'G';
	else if((-48.0 > Lat) && (Lat >= -56.0)) *UTMLetter = 'F';
	else if((-56.0 > Lat) && (Lat >= -64.0)) *UTMLetter = 'E';
	else if((-64.0 > Lat) && (Lat >= -72.0)) *UTMLetter = 'D';
	else if((-72.0 > Lat) && (Lat >= -80.0)) *UTMLetter = 'C';
	else *UTMLetter = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits

    Long0 = (UTMZone - 1.0)*6.0 - 180.0 + 3.0;                      // +3 to put origin in middle of zone
    dLong[1] = (Long - Long0) * deg2rad;
    dLong[2] = dLong[1] * dLong[1];

    S = AA*Latrad - BB*sin(2.0*Latrad) + CC*sin(4.0*Latrad) - DD*sin(6.0*Latrad) + EE*sin(8.0*Latrad);

    tanLat[1] = tan(Latrad);
    tanLat[2] = tanLat[1] * tanLat[1];

    cosLat[1] = cos(Latrad);
    cosLat[2] = cosLat[1] * cosLat[1];
    cosLat[4] = cosLat[2] * cosLat[2];
    cosLat[6] = cosLat[2] * cosLat[4];

    e2cosLat[2] = ee2 * cosLat[2];

    rho = a * (1.0 - ee1) / pow(1.0 - ee1*pow(sin(Latrad), 2.0), 1.5);
    v = rho * (1.0 + e2cosLat[2]);

    T[1] = S * k0;
    T[6] = v * cosLat[1] * k0;
    T[2] = T[6] * sin(Latrad) / 2.0;
    T[3] = T[2] * cosLat[2]
           * (5 - tanLat[2] + (9.0 + 4.0*e2cosLat[2])*e2cosLat[2])
           / 12.0;
    T[4] = T[2] * cosLat[4]
           * (61.0 + tanLat[2]*(tanLat[2] - 58.0)
                   + (((88.0*e2cosLat[2] + 324.0)*e2cosLat[2] + 445.0)*e2cosLat[2] + 270.0)*e2cosLat[2]
                   - tanLat[2]*(((192.0*e2cosLat[2] + 600.0)*e2cosLat[2] + 680.0)*e2cosLat[2] + 330.0)*e2cosLat[2])
           / 360.0;
    T[5] = T[2] * cosLat[6]
           * (1385.0 + ((543.0 - tanLat[2])*tanLat[2] - 3111.0)*tanLat[2])
           / 20160.0;
    T[7] = T[6] * cosLat[2]
           * (1 - tanLat[2] + e2cosLat[2])
           / 6.0;
    T[8] = T[6] * cosLat[4]
           * (5.0 + (tanLat[2] - 18.0)*tanLat[2]
                  + ((4.0*e2cosLat[2] + 13.0)*e2cosLat[2] + 14.0)*e2cosLat[2]
                  - tanLat[2]*((24.0*e2cosLat[2] + 64.0)*e2cosLat[2] + 58.0)*e2cosLat[2])
           / 120.0;
    T[9] = T[6] * cosLat[6]
           * (61.0 + ((179.0 - tanLat[2])*tanLat[2] - 479.0)*tanLat[2])
           / 5040.0;

    // calculate UTM coordinates
    *UTMNorthing = T[1] + (((T[5]*dLong[2] + T[4])*dLong[2] + T[3])*dLong[2] + T[2])*dLong[2];
    if (Latrad < 0.0)
        *UTMNorthing += 10000000.0;
    *UTMNorthing = floor(*UTMNorthing*1000.0) / 1000.0;     // mm

    *UTMEasting = FE + dLong[1] * (T[6] + ((T[9]*dLong[2] + T[8])*dLong[2] + T[7])*dLong[2]);
    *UTMEasting = floor(*UTMEasting*1000.0) / 1000.0;       // mm

    return UTMZone;
}
//------------------------------
