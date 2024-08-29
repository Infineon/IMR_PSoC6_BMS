/*
 * tle9012_data.c
 *
 *	Library for TLE9012 Li-Ion battery monitoring and balancing system IC. See main file Lib_TLE9012.c for how to use guide
 *	Created by R. Santeler @ Infineon 2023
 *
*******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related materials ("Software") is owned by Cypress Semiconductor Corporation or one of its affiliates ("Cypress") and is protected by and subject to worldwide patent protection (United States and foreign), United States copyright laws and international treaty provisions.  Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive, non-transferable license to copy, modify, and compile the Software source code solely for use in connection with Cypress's integrated circuit products.  Any reproduction, modification, translation, compilation, or representation of this Software except as specified above is prohibited without the express written permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to the Software without notice. Cypress does not assume any liability arising out of the application or use of the Software or any product or circuit described in the Software. Cypress does not authorize its products for use in any products where a malfunction or failure of the Cypress product may reasonably be expected to result in significant property damage, injury or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the manufacturer of such system or application assumes all risk of such use and in doing so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h> // includes strtol

#include "tle9012_data.h"

// Conversion lookup tables
coord_t LT_NTC_10k[LT_NTC_10K_SIZE] =
{    // kOhm   , °C
	{  0.309250, 150.000000},
	{  0.315670, 149.000000},
	{  0.322260, 148.000000},
	{  0.329010, 147.000000},
	{  0.335930, 146.000000},
	{  0.343020, 145.000000},
	{  0.350300, 144.000000},
	{  0.357770, 143.000000},
	{  0.365430, 142.000000},
	{  0.373280, 141.000000},
	{  0.381340, 140.000000},
	{  0.389610, 139.000000},
	{  0.398100, 138.000000},
	{  0.406810, 137.000000},
	{  0.415750, 136.000000},
	{  0.424930, 135.000000},
	{  0.434350, 134.000000},
	{  0.444020, 133.000000},
	{  0.453950, 132.000000},
	{  0.464150, 131.000000},
	{  0.474630, 130.000000},
	{  0.485390, 129.000000},
	{  0.496450, 128.000000},
	{  0.507800, 127.000000},
	{  0.519470, 126.000000},
	{  0.531460, 125.000000},
	{  0.543790, 124.000000},
	{  0.556460, 123.000000},
	{  0.569480, 122.000000},
	{  0.582860, 121.000000},
	{  0.596630, 120.000000},
	{  0.610780, 119.000000},
	{  0.625340, 118.000000},
	{  0.640310, 117.000000},
	{  0.655710, 116.000000},
	{  0.671550, 115.000000},
	{  0.687850, 114.000000},
	{  0.704630, 113.000000},
	{  0.721890, 112.000000},
	{  0.739660, 111.000000},
	{  0.757950, 110.000000},
	{  0.776780, 109.000000},
	{  0.796170, 108.000000},
	{  0.816130, 107.000000},
	{  0.836700, 106.000000},
	{  0.857880, 105.000000},
	{  0.879690, 104.000000},
	{  0.902170, 103.000000},
	{  0.925340, 102.000000},
	{  0.949210, 101.000000},
	{  0.973810, 100.000000},
	{  0.999170,  99.000000},
	{  1.025300,  98.000000},
	{  1.052300,  97.000000},
	{  1.080100,  96.000000},
	{  1.108800,  95.000000},
	{  1.138300,  94.000000},
	{  1.168800,  93.000000},
	{  1.200300,  92.000000},
	{  1.232800,  91.000000},
	{  1.266300,  90.000000},
	{  1.300900,  89.000000},
	{  1.336700,  88.000000},
	{  1.373500,  87.000000},
	{  1.411600,  86.000000},
	{  1.451000,  85.000000},
	{  1.491600,  84.000000},
	{  1.533600,  83.000000},
	{  1.576900,  82.000000},
	{  1.621800,  81.000000},
	{  1.668100,  80.000000},
	{  1.716000,  79.000000},
	{  1.765500,  78.000000},
	{  1.816600,  77.000000},
	{  1.869600,  76.000000},
	{  1.924300,  75.000000},
	{  1.980900,  74.000000},
	{  2.039500,  73.000000},
	{  2.100100,  72.000000},
	{  2.162900,  71.000000},
	{  2.227800,  70.000000},
	{  2.295100,  69.000000},
	{  2.364700,  68.000000},
	{  2.436800,  67.000000},
	{  2.511400,  66.000000},
	{  2.588800,  65.000000},
	{  2.668900,  64.000000},
	{  2.752000,  63.000000},
	{  2.838000,  62.000000},
	{  2.927200,  61.000000},
	{  3.019700,  60.000000},
	{  3.115700,  59.000000},
	{  3.215100,  58.000000},
	{  3.318300,  57.000000},
	{  3.425400,  56.000000},
	{  3.536500,  55.000000},
	{  3.651800,  54.000000},
	{  3.771400,  53.000000},
	{  3.895700,  52.000000},
	{  4.024700,  51.000000},
	{  4.158700,  50.000000},
	{  4.297800,  49.000000},
	{  4.442400,  48.000000},
	{  4.592700,  47.000000},
	{  4.748800,  46.000000},
	{  4.911200,  45.000000},
	{  5.080000,  44.000000},
	{  5.255500,  43.000000},
	{  5.438000,  42.000000},
	{  5.627900,  41.000000},
	{  5.825500,  40.000000},
	{  6.031200,  39.000000},
	{  6.245200,  38.000000},
	{  6.468100,  37.000000},
	{  6.700100,  36.000000},
	{  6.941800,  35.000000},
	{  7.193600,  34.000000},
	{  7.455900,  33.000000},
	{  7.729300,  32.000000},
	{  8.014200,  31.000000},
	{  8.311300,  30.000000},
	{  8.621100,  29.000000},
	{  8.944200,  28.000000},
	{  9.281200,  27.000000},
	{  9.632900,  26.000000},
	{ 10.000000,  25.000000},
	{ 10.383000,  24.000000},
	{ 10.783000,  23.000000},
	{ 11.201000,  22.000000},
	{ 11.637000,  21.000000},
	{ 12.093000,  20.000000},
	{ 12.570000,  19.000000},
	{ 13.068000,  18.000000},
	{ 13.589000,  17.000000},
	{ 14.133000,  16.000000},
	{ 14.703000,  15.000000},
	{ 15.298000,  14.000000},
	{ 15.922000,  13.000000},
	{ 16.574000,  12.000000},
	{ 17.258000,  11.000000},
	{ 17.973000,  10.000000},
	{ 18.722000,   9.000000},
	{ 19.507000,   8.000000},
	{ 20.330000,   7.000000},
	{ 21.192000,   6.000000},
	{ 22.096000,   5.000000},
	{ 23.044000,   4.000000},
	{ 24.038000,   3.000000},
	{ 25.082000,   2.000000},
	{ 26.177000,   1.000000},
	{ 27.326000,   0.000000},
	{ 28.533000,  -1.000000},
	{ 29.800000,  -2.000000},
	{ 31.132000,  -3.000000},
	{ 32.531000,  -4.000000},
	{ 34.001000,  -5.000000},
	{ 35.547000,  -6.000000},
	{ 37.173000,  -7.000000},
	{ 38.883000,  -8.000000},
	{ 40.682000,  -9.000000},
	{ 42.576000, -10.000000},
	{ 44.570000, -11.000000},
	{ 46.669000, -12.000000},
	{ 48.880000, -13.000000},
	{ 51.210000, -14.000000},
	{ 53.665000, -15.000000},
	{ 56.253000, -16.000000},
	{ 58.983000, -17.000000},
	{ 61.861000, -18.000000},
	{ 64.899000, -19.000000},
	{ 68.104000, -20.000000},
	{ 71.489000, -21.000000},
	{ 75.062000, -22.000000},
	{ 78.837000, -23.000000},
	{ 82.825000, -24.000000},
	{ 87.041000, -25.000000},
	{ 91.498000, -26.000000},
	{ 96.211000, -27.000000},
	{101.200000, -28.000000},
	{106.470000, -29.000000},
	{112.060000, -30.000000},
	{117.970000, -31.000000},
	{124.230000, -32.000000},
	{130.870000, -33.000000},
	{137.900000, -34.000000},
	{145.360000, -35.000000},
	{153.260000, -36.000000},
	{161.650000, -37.000000},
	{170.550000, -38.000000},
	{180.000000, -39.000000},
	{190.030000, -40.000000},
};

//****************************************************************************
// interpLookupTable_tle9012
//
// Returns a y value for an x based on an lookup table and linear approximation between nearest points
// Based on https://stackoverflow.com/questions/7091294/how-to-build-a-lookup-table-in-c-sdcc-compiler-with-linear-interpolation
//
// lookupTable 		-> Pointer to an array of coord_t, where each line represents a data point {x,y}. X values must be sorted lowest to highest
// lookupTableSize	-> Soze of the array of coord_t
// x				-> The x value to be converted to y value
//****************************************************************************
double interpLookupTable_tle9012(const coord_t* lookupTable, uint16_t lookupTableSize, double x)
{
	// Go thru each line of lookup table until the x value is bigger than current but lower than next line
	for(uint16_t i = 0; i < lookupTableSize-1; i++ ) {
        if ( lookupTable[i].x <= x && lookupTable[i+1].x >= x ) {
            double diffx = x - lookupTable[i].x;
            double diffn = lookupTable[i+1].x - lookupTable[i].x;

            return lookupTable[i].y + ( lookupTable[i+1].y - lookupTable[i].y ) * diffx / diffn;
        }
    }

    return 0; // Not in Range
}
double interpLookupTableReverse_tle9012(const coord_t* lookupTable, uint16_t lookupTableSize, double y)
{
	// Go thru each line of lookup table until the x value is bigger than current but lower than next line
	for(uint16_t i = lookupTableSize-1; i > 0; i-- ) {
        if ( lookupTable[i].y <= y && lookupTable[i-1].y >= y ) {
            double diffy = y - lookupTable[i].y;
            double diffn = lookupTable[i-1].y - lookupTable[i].y;

            return lookupTable[i].x + ( lookupTable[i-1].x - lookupTable[i].x ) * diffy / diffn;
        }
    }

    return 0; // Not in Range
}


//****************************************************************************
// xtoi_tle9012
//
// Returns int value for hexadecimal string
//
// Copyright (c) Leigh Brasington 2012.  http://www.leighb.com/xtoi.htm
//****************************************************************************
int xtoi_tle9012(char *hexstring)
{
    int i = 0;
    char c = 0;

    // get rid of '0x'
    if ((*hexstring == '0') && (*(hexstring+1) == 'x'))
        hexstring += 2;

    while(*hexstring)
    {
        c = toupper((unsigned char)*hexstring++);
        if ((c < '0') || (c > 'F') || ((c > '9') && (c < 'A')))
            break;
        c -= '0';
        if (c > 9)
            c -= 7;
        i = (i << 4) + c;
    }
    return i;
}
