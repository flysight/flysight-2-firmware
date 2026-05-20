/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2025 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#include "activelook_mode0.h"
#include "activelook_client.h"    // For FS_ActiveLook_Client_WriteWithoutResp
#include "config.h"               // For FS_Config_Get()
#include "flight_params.h"
#include "gnss.h"                 // For FS_GNSS_GetData()
#include "nav.h"                  // For calcDirection, calcDistance, calcRelBearing
#include "vbat.h"
#include "app_common.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>                 // For atan2, fabs

/* --------------------------------------------------------------------------
   1. Unit System Definitions
   -------------------------------------------------------------------------- */

// Enum to categorize the physical quantity a parameter represents
typedef enum {
    FS_UNIT_TYPE_SPEED,
    FS_UNIT_TYPE_DISTANCE,
    FS_UNIT_TYPE_ALTITUDE,
    FS_UNIT_TYPE_ANGLE,
    FS_UNIT_TYPE_NONE
} FS_ParamUnitType_t;

// Structure to hold conversion factor and unit suffix
typedef struct {
    double      multiplier;
    const char* suffix;
} UnitConversionInfo_t;

// --- Conversion Constants ---
#define M_PER_S_TO_KMH      3.6
#define M_PER_S_TO_MPH      2.23694
#define METERS_TO_KM        0.001
#define METERS_TO_MILES     0.000621371
#define METERS_TO_FEET      3.28084

// Minimum announced altitude (mm)
#define ALT_MIN_MM (1500L * 1000L)

/* --------------------------------------------------------------------------
   2. Data Structures for Each Line
   -------------------------------------------------------------------------- */

/**
 * A function pointer type that, given GNSS data, returns a double
 * representing the value we want displayed. e.g. heading, alt, etc.
 */
typedef double (*LineValueFn_t)(const FS_GNSS_Data_t*);

/* Simple "getter" functions for each known line type */
static double LN_HSpeed(const FS_GNSS_Data_t *d) {
    double base_hspeed = (double)d->gSpeed / 100.0; // cm/s to m/s
    double correction_factor = FS_FlightParams_GetSASCorrectionFactor(d->hMSL);
    return base_hspeed * correction_factor;
}
static double LN_VSpeed(const FS_GNSS_Data_t *d) {
    double base_vspeed = (double)d->velD / 1000.0; // mm/s to m/s
    double correction_factor = FS_FlightParams_GetSASCorrectionFactor(d->hMSL);
    return base_vspeed * correction_factor;
}
static double LN_GlideRatio(const FS_GNSS_Data_t *d) {
    if (d->velD != 0) {
        return (double)(d->gSpeed / 100.0) / (double)(d->velD / 1000.0);
    }
    return 0.0; // Or some indicator for undefined glide ratio
}
static double LN_InvGlideRatio(const FS_GNSS_Data_t *d) {
    if (d->gSpeed != 0) {
        return (double)(d->velD / 1000.0) / (double)(d->gSpeed / 100.0);
    }
    return 0.0; // Or some indicator
}
static double LN_TotalSpeed(const FS_GNSS_Data_t *d) {
    double base_speed = (double)d->speed / 100.0; // cm/s to m/s
    double correction_factor = FS_FlightParams_GetSASCorrectionFactor(d->hMSL);
    return base_speed * correction_factor;
}
static double LN_DirToDest(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    // Check if destination is set and within reasonable range if needed
    if ((calcDistance(d->lat, d->lon, cfg->lat, cfg->lon) < cfg->max_dist) || (cfg->max_dist == 0)) {
         return (double)calcDirection(d->lat, d->lon, cfg->lat, cfg->lon, d->heading);
    }
    return 0.0; // Indicate no destination or out of range
}
static double LN_DistToDest(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    return (double)calcDistance(d->lat, d->lon, cfg->lat, cfg->lon); // in meters
}
static double LN_DirToBearing(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    // Assuming bearing is configured
    return (double)calcRelBearing(cfg->bearing, d->heading / 100000); // heading is 1e-5 deg
}
static double LN_DiveAngle(const FS_GNSS_Data_t *d) {
    if (d->gSpeed > 0) { // Avoid division by zero
        // velD is positive down, gSpeed is horizontal speed (always positive)
        // atan2(y, x) -> atan2(vertical_speed, horizontal_speed)
        // Need to convert cm/s to m/s or be consistent
        return atan2((double)(d->velD / 1000.0), (double)(d->gSpeed / 100.0)) * (180.0 / M_PI); // Result in degrees
    }
    return 0.0;
}
static double LN_Altitude(const FS_GNSS_Data_t *d) {
    const FS_Config_Data_t *cfg = FS_Config_Get();
    return ((double)d->hMSL - (double)cfg->dz_elev) / 1000.0; // mm to m, relative to DZ elev
}
static double LN_Heading(const FS_GNSS_Data_t *d) {
    return (double)d->heading / 100000.0; // 1e-5 deg to deg
}

typedef struct {
    uint8_t             typeId;
    const char         *label;
    FS_ParamUnitType_t  unitType;
    LineValueFn_t       fn;
    uint8_t             icon_id;
} AL_Mode0_LineMap_t;

static const AL_Mode0_LineMap_t s_lineMap[] =
{
    { FS_CONFIG_MODE_HORIZONTAL_SPEED,         "HSpd", FS_UNIT_TYPE_SPEED,    LN_HSpeed,        100 },
    { FS_CONFIG_MODE_VERTICAL_SPEED,           "VSpd", FS_UNIT_TYPE_SPEED,    LN_VSpeed,        101 },
    { FS_CONFIG_MODE_GLIDE_RATIO,              "GR",   FS_UNIT_TYPE_NONE,     LN_GlideRatio,    102 },
    { FS_CONFIG_MODE_INVERSE_GLIDE_RATIO,      "IGR",  FS_UNIT_TYPE_NONE,     LN_InvGlideRatio, 103 },
    { FS_CONFIG_MODE_TOTAL_SPEED,              "Spd",  FS_UNIT_TYPE_SPEED,    LN_TotalSpeed,    104 },
    { FS_CONFIG_MODE_DIRECTION_TO_DESTINATION, "Dir",  FS_UNIT_TYPE_ANGLE,    LN_DirToDest,     105 },
    { FS_CONFIG_MODE_DISTANCE_TO_DESTINATION,  "Dist", FS_UNIT_TYPE_DISTANCE, LN_DistToDest,    106 },
    { FS_CONFIG_MODE_DIRECTION_TO_BEARING,     "Brg",  FS_UNIT_TYPE_ANGLE,    LN_DirToBearing,  107 },
    { FS_CONFIG_MODE_DIVE_ANGLE,               "Dive", FS_UNIT_TYPE_ANGLE,    LN_DiveAngle,     108 },
    { FS_CONFIG_MODE_ALTITUDE,                 "Alt",  FS_UNIT_TYPE_ALTITUDE, LN_Altitude,      109 },
    { 13,                                      "Hdg",  FS_UNIT_TYPE_ANGLE,    LN_Heading,       110 },
};
static const unsigned s_lineMapCount = sizeof(s_lineMap) / sizeof(s_lineMap[0]);

typedef struct {
    const AL_Mode0_LineMap_t *mapEntry;
} AL_Mode0_LineSpec_t;

static AL_Mode0_LineSpec_t s_lineSpecs[4];

/**
 * We'll also keep a "setup" step variable for multi-step layout creation.
 */
static int s_step = 0;

/* --------------------------------------------------------------------------
   3. Unit Conversion Logic (to be factored out later)
   -------------------------------------------------------------------------- */

/**
 * Gets the conversion multiplier and unit suffix string based on parameter type
 * and the selected unit system.
 */
static UnitConversionInfo_t AL_GetUnitConversion(
        FS_ParamUnitType_t type,
        FS_Config_UnitSystem_t system) {
    UnitConversionInfo_t info = {1.0, ""}; // Default: no conversion, no suffix

    switch (type) {
        case FS_UNIT_TYPE_SPEED: // Base unit: m/s
            if (system == FS_UNIT_SYSTEM_METRIC) {
                info.multiplier = M_PER_S_TO_KMH;
                info.suffix = "km/h";
            } else { // Imperial
                info.multiplier = M_PER_S_TO_MPH;
                info.suffix = "mph";
            }
            break;
        case FS_UNIT_TYPE_DISTANCE: // Base unit: m
             if (system == FS_UNIT_SYSTEM_METRIC) {
                info.multiplier = METERS_TO_KM;
                info.suffix = "km";
            } else { // Imperial
                info.multiplier = METERS_TO_MILES;
                info.suffix = "mi";
            }
           break;
        case FS_UNIT_TYPE_ALTITUDE: // Base unit: m
            if (system == FS_UNIT_SYSTEM_METRIC) {
                // info.multiplier = 1.0; // Default is already 1.0
                info.suffix = "m";
            } else { // Imperial
                info.multiplier = METERS_TO_FEET;
                info.suffix = "ft";
            }
            break;
        case FS_UNIT_TYPE_ANGLE: // Base unit: degrees
            // No conversion needed between metric/imperial for angles
            info.suffix = "deg";
            break;
        case FS_UNIT_TYPE_NONE:  // Unitless
            // No conversion, no suffix (defaults are correct)
            break;
    }
    return info;
}

/* --------------------------------------------------------------------------
   4. Helpers for Sending Commands / Building Layout
   -------------------------------------------------------------------------- */

static void AL_SendRaw(const uint8_t *data, uint16_t length)
{
    FS_ActiveLook_Client_WriteWithoutResp(data, length);
}

static tBleStatus AL_SendRawReliable(const uint8_t *data, uint16_t length)
{
    return FS_ActiveLook_Client_WriteWithResp(data, length);
}

/**
 * Build status layout
 */
static uint8_t AL_BuildStatus(uint8_t layoutId,
                              uint8_t *outBuf)
{
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x60;  // "layoutSave"
    outBuf[idx++] = 0x00;  // 1B length
    uint8_t lenPos = idx++;

    outBuf[idx++] = layoutId;
    uint8_t addCmdSizePos = idx++;

    // Status bar — same geometry as when battery icon worked (height=40, y=220)
    outBuf[idx++] = 0x00; outBuf[idx++] = 0x00;  // clip X = 0
    outBuf[idx++] = 0x00;                         // clip Y = 0
    outBuf[idx++] = 0x01; outBuf[idx++] = 0x30;  // width = 304
    outBuf[idx++] = 0x28;                         // height = 40
    outBuf[idx++] = 6;                            // FG = grey 6
    outBuf[idx++] = 0;                            // BG = black
    outBuf[idx++] = 12;                           // font 12 (B612 22px)
    outBuf[idx++] = 1;                            // textValid
    outBuf[idx++] = 1; outBuf[idx++] = 0x05;     // text X = 261
    outBuf[idx++] = 35;                           // text Y = 35
    outBuf[idx++] = 4;                            // rotation = 4
    outBuf[idx++] = 0;                            // opacity = 0 (transparent bg, icons visible behind text)

    uint8_t extra[64];
    uint8_t e = 0;

    extra[e++] = 0x03; extra[e++] = 6;           // color grey 6
    extra[e++] = 0x00;                            // image sub-cmd: glasses (112)
    extra[e++] = 112;
    extra[e++] = 0x00; extra[e++] = 0xFA;        // x = 250
    extra[e++] = 0x00; extra[e++] = 0x0A;        // y = 10

    extra[e++] = 0x00;                            // image sub-cmd: flysight (113)
    extra[e++] = 113;
    extra[e++] = 0x00; extra[e++] = 0x99;        // x = 153
    extra[e++] = 0x00; extra[e++] = 0x0B;        // y = 11

    extra[e++] = 0x00;                            // image sub-cmd: satellite (111)
    extra[e++] = 111;
    extra[e++] = 0x00; extra[e++] = 0x36;        // x = 54
    extra[e++] = 0x00; extra[e++] = 0x08;        // y = 8

    outBuf[addCmdSizePos] = e;
    memcpy(&outBuf[idx], extra, e);
    idx += e;

    outBuf[idx++] = 0xAA;
    outBuf[lenPos] = idx;

    return idx;
}

static uint8_t AL_BuildLayout(uint8_t layoutId,
                              const char *headingText,
                              const char *unitsText,
                              uint8_t icon_id,
                              uint8_t cellIndex,
                              uint8_t *outBuf)
{
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x60;
    outBuf[idx++] = 0x00;
    uint8_t lenPos = idx++;

    outBuf[idx++] = layoutId;
    uint8_t addCmdSizePos = idx++;

    outBuf[idx++] = 0x00; outBuf[idx++] = 0x00;  // clip X = 0
    outBuf[idx++] = 0x00;                         // clip Y = 0
    outBuf[idx++] = 0x00; outBuf[idx++] = 152;   // width
    outBuf[idx++] = 110;                          // height
    outBuf[idx++] = 15;                           // FG = white
    outBuf[idx++] = 0;                            // BG = black
    outBuf[idx++] = 11;                           // font 11 (B612 Mono 34px)
    outBuf[idx++] = 1;                            // textValid
    outBuf[idx++] = 0; outBuf[idx++] = 140;      // text X = 140
    outBuf[idx++] = 70;                           // text Y = 70
    outBuf[idx++] = 4;                            // rotation = 4 (centered)
    outBuf[idx++] = 0;                            // opacity = 0 (transparent)

    uint8_t extra[120];
    uint8_t e = 0;

    // --- Icon: visually top-left (high x, high y), grey 6 ---
    if (icon_id > 0) {
        extra[e++] = 0x03; extra[e++] = 6;       // color grey 6
        extra[e++] = 0x00;                        // image sub-cmd
        extra[e++] = icon_id;
        extra[e++] = 0; extra[e++] = 115;        // x = 115
        extra[e++] = 0; extra[e++] = 75;         // y = 75
    } else {
        extra[e++] = 0x03; extra[e++] = 6;       // color grey 6
        extra[e++] = 0x04; extra[e++] = 10;      // font 10 (B612 18px)
        extra[e++] = 0x09;                        // text sub-cmd
        extra[e++] = 0; extra[e++] = 140;
        extra[e++] = 0; extra[e++] = 80;
        size_t lblLen = strlen(headingText);
        extra[e++] = (uint8_t)lblLen;
        memcpy(&extra[e], headingText, lblLen);
        e += lblLen;
    }

    // --- Units: visually bottom-right, grey 6 ---
    if (strlen(unitsText) > 0) {
        char unitBuf[8];
        snprintf(unitBuf, sizeof(unitBuf), "%4s", unitsText);
        size_t untLen = strlen(unitBuf);
        extra[e++] = 0x03; extra[e++] = 6;        // color grey 6
        extra[e++] = 0x04; extra[e++] = 10;       // font 10 (B612 18px)
        extra[e++] = 0x09;                         // text sub-cmd
        extra[e++] = 0; extra[e++] = 60;          // x = 60
        extra[e++] = 0; extra[e++] = 28;          // y = 28
        extra[e++] = (uint8_t)untLen;
        memcpy(&extra[e], unitBuf, untLen);
        e += untLen;
    }

    // --- Divider lines (barely visible) ---
    extra[e++] = 0x03; extra[e++] = 1;            // color grey 1

    // Cells 0,2 (vis left column): vertical divider at center of screen
    // x=0 appeared at far right of screen, so center must be at high x
    // Cell width = 152, so x=151 = the left visual edge of the left cell
    if (cellIndex == 0 || cellIndex == 2) {
        extra[e++] = 0x05;                         // line sub-cmd
        extra[e++] = 0; extra[e++] = 151;         // x0 = 151
        extra[e++] = 0; extra[e++] = 0;           // y0 = 0
        extra[e++] = 0; extra[e++] = 151;         // x1 = 151
        extra[e++] = 0; extra[e++] = 110;         // y1 = 110
    }

    // Cells 0,1 (vis top row): horizontal line at visual bottom edge = row divider
    // Layout-internal: y=0 = vis bottom, y=115 = vis top
    // We want the line at the cell's vis-bottom edge = y=0
    if (cellIndex == 0 || cellIndex == 1) {
        extra[e++] = 0x05;                         // line sub-cmd
        extra[e++] = 0; extra[e++] = 0;           // x0 = 0
        extra[e++] = 0; extra[e++] = 0;           // y0 = 0 (vis bottom edge)
        extra[e++] = 0; extra[e++] = 152;         // x1 = 152
        extra[e++] = 0; extra[e++] = 0;           // y1 = 0
    }

    outBuf[addCmdSizePos] = e;
    memcpy(&outBuf[idx], extra, e);
    idx += e;

    outBuf[idx++] = 0xAA;
    outBuf[lenPos] = idx;
    return idx;
}

/**
 * Build page: 2x2 grid of cells + status bar on top
 */
static uint8_t AL_BuildPage(uint8_t pageId, uint8_t *outBuf)
{
    const FS_Config_Data_t *cfg = FS_Config_Get();

    // Page positions for each cell: [top-left, top-right, bottom-left, bottom-right]
    // Each entry: x_hi, x_lo, y
    static const uint8_t cellPos[][3] = {
        { 0x00, 0x00, 110 },   // cell 0: x=0,   y=110 (upper-left)
        { 0x00, 152,  110 },   // cell 1: x=152, y=110 (upper-right)
        { 0x00, 0x00, 0   },   // cell 2: x=0,   y=0   (lower-left)
        { 0x00, 152,  0   },   // cell 3: x=152, y=0   (lower-right)
    };

    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x80;
    outBuf[idx++] = 0x00;
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // Status bar (layout 14) above grid
    outBuf[idx++] = 14;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 220;

    // Cell layouts
    for (int i = 0; i < cfg->num_al_lines && i < 4; i++)
    {
        outBuf[idx++] = 10 + i;
        outBuf[idx++] = cellPos[i][0];
        outBuf[idx++] = cellPos[i][1];
        outBuf[idx++] = cellPos[i][2];
    }

    outBuf[idx++] = 0xAA;
    outBuf[lenPos] = idx;
    return idx;
}

/**
 * Build a single pageUpdate command with 4 lines separated by '\0'.
 */
static uint8_t AL_BuildPageUpdate(uint8_t pageId,
                                  const char *heading,
                                  const char *line1,
                                  const char *line2,
                                  const char *line3,
                                  const char *line4,
                                  uint8_t *outBuf)
{
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x86; // "pageClearAndDisplay"
    outBuf[idx++] = 0x00;
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // heading
    size_t lh = strlen(heading);
    memcpy(&outBuf[idx], heading, lh);
    idx += lh;
    outBuf[idx++] = 0;

    // line1
    size_t l1 = strlen(line1);
    memcpy(&outBuf[idx], line1, l1);
    idx += l1;
    outBuf[idx++] = 0;

    // line2
    size_t l2 = strlen(line2);
    memcpy(&outBuf[idx], line2, l2);
    idx += l2;
    outBuf[idx++] = 0;

    // line3
    size_t l3 = strlen(line3);
    memcpy(&outBuf[idx], line3, l3);
    idx += l3;
    outBuf[idx++] = 0;

    // line4
    size_t l4 = strlen(line4);
    memcpy(&outBuf[idx], line4, l4);
    idx += l4;
    outBuf[idx++] = 0;

    outBuf[idx++] = 0xAA;
    outBuf[lenPos] = idx;
    return idx;
}

/* --------------------------------------------------------------------------
   3. Mode0 Implementation
   -------------------------------------------------------------------------- */

// Helper to find the LineMap entry by typeId
static const AL_Mode0_LineMap_t* FindLineMapEntry(uint8_t typeId) {
    for (unsigned k = 0; k < s_lineMapCount; k++) {
        if (s_lineMap[k].typeId == typeId) {
            return &s_lineMap[k];
        }
    }
    return NULL;
}


/**
 * FS_ActiveLook_Mode0_Init()
 *
 *  - Reset our s_step to 0
 *  - Read the config => al_line_1..4
 *  - For each of the 4 lines, find a matching entry in s_lineMap
 *    (by typeId) and store it in s_lineSpecs[i].
 */
void FS_ActiveLook_Mode0_Init(void)
{
    // Read config
    const FS_Config_Data_t *cfg = FS_Config_Get();

    s_step = 0;

    for (int i = 0; i < cfg->num_al_lines && i < 4; i++)
    {
        s_lineSpecs[i].mapEntry = FindLineMapEntry(cfg->al_lines[i].mode);
    }
}

/**
 * FS_ActiveLook_Mode0_Setup()
 *
 * Multi-step routine. In each step (0-3), builds a layout (#10-13).
 * It now dynamically determines the unit suffix based on the line's parameter type
 * and the chosen unit system.
 * Step 4 builds the page definition.
 */
FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void)
{
    // Read config
    const FS_Config_Data_t *cfg = FS_Config_Get();

    uint8_t buf[128];
    uint8_t length;
    FS_ActiveLook_SetupStatus_t status = FS_AL_SETUP_IN_PROGRESS;

    if (s_step < cfg->num_al_lines) {
        // Steps 0..N-1: Build cell layouts
        int lineIndex = s_step;
        uint8_t layoutId = 10 + lineIndex;
        const AL_Mode0_LineMap_t* mapEntry = s_lineSpecs[lineIndex].mapEntry;
        const char* label = mapEntry ? mapEntry->label : "?";
        const char* unitSuffix = "";

        if (mapEntry) {
            UnitConversionInfo_t unitInfo = AL_GetUnitConversion(
                    mapEntry->unitType,
                    cfg->al_lines[lineIndex].units);
            unitSuffix = unitInfo.suffix;
        }

        uint8_t icon = mapEntry ? mapEntry->icon_id : 0;
        length = AL_BuildLayout(layoutId, label, unitSuffix, icon, lineIndex, buf);
        if (length > 0 && AL_SendRawReliable(buf, length) == BLE_STATUS_SUCCESS) {
            s_step++;
        }
    } else if (s_step == cfg->num_al_lines) {
        length = AL_BuildStatus(14, buf);
        if (length > 0 && AL_SendRawReliable(buf, length) == BLE_STATUS_SUCCESS) {
            s_step++;
        }
    } else if (s_step == cfg->num_al_lines + 1) {
        length = AL_BuildPage(10, buf);
        if (length > 0 && AL_SendRawReliable(buf, length) == BLE_STATUS_SUCCESS) {
            s_step++;
            status = FS_AL_SETUP_DONE;
        }
    } else {
        status = FS_AL_SETUP_DONE;
    }

    return status;
}

/**
 * FS_ActiveLook_Mode0_Update()
 *
 * Called periodically to update the display.
 * - Gets current GNSS data.
 * - For each line:
 * - Finds the corresponding map entry using stored typeId.
 * - Calls the base value function (fn).
 * - Gets the correct unit conversion info (multiplier).
 * - Calculates the display value by applying the multiplier.
 * - Formats the display value into a string.
 * - Builds and sends the "pageClearAndDisplay" command with the 4 formatted value strings.
 */
void FS_ActiveLook_Mode0_Update(void)
{
    // Read config
    const FS_Config_Data_t *cfg = FS_Config_Get();

    const FS_GNSS_Data_t *gnss = FS_GNSS_GetData();
    const FS_VBAT_Data_t *vbat = FS_VBAT_GetData();
    uint8_t alBatt = FS_ActiveLook_Client_GetBatteryLevel();

    char lineValueStr[4][16]; // Buffer for formatted value strings

    // Calculate altitude above ground level once
    double altitude_agl_mm = (double)gnss->hMSL - (double)cfg->dz_elev;

    // For each line, calculate, convert, and format the value
    for (int i = 0; i < cfg->num_al_lines; i++)
    {
        double baseVal = 0.0;
        double displayVal = 0.0;
        bool display_invalid = false; // Flag to indicate if "----" should be shown

        const AL_Mode0_LineMap_t* mapEntry = s_lineSpecs[i].mapEntry;

        if (mapEntry && mapEntry->fn) {
            // 1. If nav is disabled but line is a navigation mode, mark it invalid
            //    so we display "----"
            if (!cfg->enable_nav &&
               (mapEntry->typeId == FS_CONFIG_MODE_DIRECTION_TO_DESTINATION ||
                mapEntry->typeId == FS_CONFIG_MODE_DISTANCE_TO_DESTINATION ||
                mapEntry->typeId == FS_CONFIG_MODE_DIRECTION_TO_BEARING ||
                mapEntry->typeId == FS_CONFIG_MODE_LEFT_RIGHT))
            {
                display_invalid = true;
            }

            // 2. Get base value if not invalid
            if (!display_invalid) {
                baseVal = mapEntry->fn(gnss);
            }

            // 3. Check validity limits
            if (!display_invalid && gnss->gpsFix != 3) {
                display_invalid = true;
            }

            // Check ALT_MIN for Altitude display
            if (!display_invalid && mapEntry->typeId == FS_CONFIG_MODE_ALTITUDE) {
                if (altitude_agl_mm < ALT_MIN_MM) {
                    display_invalid = true;
                }
            }

            // Check end_nav for Navigation parameters
            if (!display_invalid &&
                (mapEntry->typeId == FS_CONFIG_MODE_DIRECTION_TO_DESTINATION ||
                 mapEntry->typeId == FS_CONFIG_MODE_DISTANCE_TO_DESTINATION ||
                 mapEntry->typeId == FS_CONFIG_MODE_DIRECTION_TO_BEARING))
            {
                // end_nav is in mm in config struct, compare directly
                if ((cfg->end_nav != 0) && (altitude_agl_mm < cfg->end_nav)) {
                    display_invalid = true;
                }
            }

            // Check max_dist for Destination parameters (only if not already invalid)
            if (!display_invalid &&
                (mapEntry->typeId == FS_CONFIG_MODE_DIRECTION_TO_DESTINATION ||
                 mapEntry->typeId == FS_CONFIG_MODE_DISTANCE_TO_DESTINATION))
            {
                // baseVal for DistToDest is already the distance in meters
                // For DirToDest, we need to recalculate distance if baseVal isn't distance
                double distance_m;
                if (mapEntry->typeId == FS_CONFIG_MODE_DISTANCE_TO_DESTINATION) {
                    distance_m = baseVal;
                } else {
                     // Recalculate distance if the primary value isn't distance
                     distance_m = (double)calcDistance(gnss->lat, gnss->lon, cfg->lat, cfg->lon);
                }

                // max_dist is in meters in config struct
                if ((cfg->max_dist != 0) && (distance_m > cfg->max_dist)) {
                    display_invalid = true;
                }
            }

            if (!display_invalid) {
                // 4. Get conversion info based on type and chosen system
                UnitConversionInfo_t unitInfo = AL_GetUnitConversion(
                        mapEntry->unitType,
                        cfg->al_lines[i].units);

                // 5. Calculate display value
                displayVal = baseVal * unitInfo.multiplier;

                // 6. Format the display value
                int dec = (int)(cfg->al_lines[i].decimals);
                if (dec > 1) dec = 1;
                // Drop decimal for large values to fit cell
                if (dec > 0 && (displayVal >= 10000.0 || displayVal <= -1000.0))
                    dec = 0;
                int width = (dec > 0) ? 6 : 5;
                snprintf(
                        lineValueStr[i],
                        sizeof(lineValueStr[i]),
                        "%*.*f",
                        width, dec,
                        displayVal);
            }
        } else {
            // Handle case where line type or function is invalid
            display_invalid = true;
        }

        // If any limit check failed, display placeholder
        if (display_invalid) {
             int dec2 = (int)(cfg->al_lines[i].decimals);
             if (dec2 > 1) dec2 = 1;
             int width2 = (dec2 > 0) ? 6 : 5;
             snprintf(lineValueStr[i], sizeof(lineValueStr[i]), "%*s", width2, "--.-");
        }

    }

    // Build header text
    char battLevels[30];

    char alBattStr[5];
    if (alBatt == 255)
        snprintf(alBattStr, sizeof(alBattStr), "??");
    else
        snprintf(alBattStr, sizeof(alBattStr), "%d", alBatt);

    int fs_pct = (100 * (vbat->voltage - 3300)) / (4200 - 3200);
    fs_pct = MAX(0, MIN(100, fs_pct));

    sprintf(battLevels, " %s%%    %d%%    %d", alBattStr, fs_pct, gnss->numSV);

    // Build the final packet with the 4 lines
    uint8_t buf[128];
    uint8_t length = AL_BuildPageUpdate(10,
                                        battLevels,
                                        lineValueStr[0],
                                        lineValueStr[1],
                                        lineValueStr[2],
                                        lineValueStr[3],
                                        buf);
    AL_SendRaw(buf, length);
}
