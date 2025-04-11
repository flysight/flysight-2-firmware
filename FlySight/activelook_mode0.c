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

/**
 * Updated table entry: includes unitType for conversion purposes.
 * 'units' field removed, as it's now dynamic based on user choice.
 */
typedef struct {
    uint8_t             typeId;   // ID from config (e.g., FS_CONFIG_MODE_HORIZONTAL_SPEED)
    const char         *label;    // Base text label (e.g., "HSpd:")
    FS_ParamUnitType_t  unitType; // Type of quantity (Speed, Distance, etc.)
    LineValueFn_t       fn;       // Function returning the value in base units (m/s, m, deg)
} AL_Mode0_LineMap_t;

/*
   A dictionary of possible line types based on FS_CONFIG_MODE_* in config.h
   and implementations in audio_control.c.
*/
static const AL_Mode0_LineMap_t s_lineMap[] =
{
    { FS_CONFIG_MODE_HORIZONTAL_SPEED,         "HSpd:", FS_UNIT_TYPE_SPEED,    LN_HSpeed       }, // 0
    { FS_CONFIG_MODE_VERTICAL_SPEED,           "VSpd:", FS_UNIT_TYPE_SPEED,    LN_VSpeed       }, // 1
    { FS_CONFIG_MODE_GLIDE_RATIO,              "GR:",   FS_UNIT_TYPE_NONE,     LN_GlideRatio   }, // 2
    { FS_CONFIG_MODE_INVERSE_GLIDE_RATIO,      "1/GR:", FS_UNIT_TYPE_NONE,     LN_InvGlideRatio}, // 3
    { FS_CONFIG_MODE_TOTAL_SPEED,              "Spd:",  FS_UNIT_TYPE_SPEED,    LN_TotalSpeed   }, // 4
    { FS_CONFIG_MODE_DIRECTION_TO_DESTINATION, "Dir:",  FS_UNIT_TYPE_ANGLE,    LN_DirToDest    }, // 5
    { FS_CONFIG_MODE_DISTANCE_TO_DESTINATION,  "Dist:", FS_UNIT_TYPE_DISTANCE, LN_DistToDest   }, // 6
    { FS_CONFIG_MODE_DIRECTION_TO_BEARING,     "Brg:",  FS_UNIT_TYPE_ANGLE,    LN_DirToBearing }, // 7
    // Mode 8, 9, 10?
    { FS_CONFIG_MODE_DIVE_ANGLE,               "Dive:", FS_UNIT_TYPE_ANGLE,    LN_DiveAngle    }, // 11
    { FS_CONFIG_MODE_ALTITUDE,                 "Alt:",  FS_UNIT_TYPE_ALTITUDE, LN_Altitude     }, // 12
    { 13,                                      "Hdg:",  FS_UNIT_TYPE_ANGLE,    LN_Heading      }, // Mode 13 (Heading)
};
static const unsigned s_lineMapCount = sizeof(s_lineMap) / sizeof(s_lineMap[0]);

/**
 * Updated line spec: stores only typeId and label.
 * Units and value calculation/conversion happen dynamically.
 */
typedef struct {
    uint8_t      typeId; // ID from config (e.g., FS_CONFIG_MODE_HORIZONTAL_SPEED)
    const char  *label;  // Base label (e.g., "HSpd:")
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

/* A helper to do a single WriteWithoutResp to the BLE client. */
static void AL_SendRaw(const uint8_t *data, uint16_t length)
{
    FS_ActiveLook_Client_WriteWithoutResp(data, length);
}

/**
 * Build your "layoutSave" command.
 * This is the same code you've used; simplified for brevity.
 */
static uint8_t AL_BuildLayout(uint8_t layoutId,
                              const char *headingText,
                              const char *unitsText,
                              uint8_t *outBuf)
{
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x60;  // "layoutSave"
    outBuf[idx++] = 0x00;  // 1B length
    uint8_t lenPos = idx++;

    // layout ID
    outBuf[idx++] = layoutId;

    // Additional commands size placeholder
    uint8_t addCmdSizePos = idx++;

    // X, Y, Width, Height, color, etc. Hard-coded example
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x01;
    outBuf[idx++] = 0x30;
    outBuf[idx++] = 0x28;
    outBuf[idx++] = 15;
    outBuf[idx++] = 0;
    outBuf[idx++] = 2;
    outBuf[idx++] = 1;
    outBuf[idx++] = 0;
    outBuf[idx++] = 200;
    outBuf[idx++] = 40;
    outBuf[idx++] = 4;
    outBuf[idx++] = 1;

    // Build sub-commands for label & units
    uint8_t extra[64];
    uint8_t e = 0;

    // color
    extra[e++] = 0x03;
    extra[e++] = 15;

    // font
    extra[e++] = 0x04;
    extra[e++] = 1;

    // text => position
    extra[e++] = 0x09;
    extra[e++] = 1;   // x hi
    extra[e++] = 5;   // x lo
    extra[e++] = 0;   // y hi
    extra[e++] = 35;  // y lo

    // label
    size_t lblLen = strlen(headingText);
    extra[e++] = (uint8_t)lblLen;
    memcpy(&extra[e], headingText, lblLen);
    e += lblLen;

    // font again
    extra[e++] = 0x04;
    extra[e++] = 1;

    // text => position
    extra[e++] = 0x09;
    extra[e++] = 0;   // x hi
    extra[e++] = 80;  // x lo
    extra[e++] = 0;   // y hi
    extra[e++] = 35;  // y lo

    // units
    size_t untLen = strlen(unitsText);
    extra[e++] = (uint8_t)untLen;
    memcpy(&extra[e], unitsText, untLen);
    e += untLen;

    // copy extras
    outBuf[addCmdSizePos] = e;
    memcpy(&outBuf[idx], extra, e);
    idx += e;

    outBuf[idx++] = 0xAA; // footer
    outBuf[lenPos] = idx; // fill length

    return idx;
}

/**
 * Build a page referencing layout #10..#13.
 */
static uint8_t AL_BuildPage(uint8_t pageId, uint8_t *outBuf)
{
    // Read config
    const FS_Config_Data_t *cfg = FS_Config_Get();

    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x80;
    outBuf[idx++] = 0x00;
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    for (int i = 0; i < cfg->num_al_lines; i++)
    {
        outBuf[idx++] = 10 + i;
        outBuf[idx++] = 0x00;
        outBuf[idx++] = 0x00;
        outBuf[idx++] = 168 - 40 * i;
    }

    outBuf[idx++] = 0xAA;
    outBuf[lenPos] = idx;
    return idx;
}

/**
 * Build a single pageUpdate command with 4 lines separated by '\0'.
 */
static uint8_t AL_BuildPageUpdate(uint8_t pageId,
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

    // For each line i, find its base label and store typeId
    for (int i = 0; i < cfg->num_al_lines; i++)
    {
        const AL_Mode0_LineMap_t* entry = FindLineMapEntry(
                cfg->al_lines[i].mode);
        if (entry) {
            s_lineSpecs[i].typeId = entry->typeId;
            s_lineSpecs[i].label  = entry->label;
        } else {
            // Fallback if typeId is not found in the map
            s_lineSpecs[i].typeId = 0;
            s_lineSpecs[i].label  = "?";
        }
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

    if (s_step >= 0 && s_step < cfg->num_al_lines) { // Build layouts
        int lineIndex = s_step; // Corresponds to s_lineSpecs[lineIndex]
        uint8_t layoutId = 10 + lineIndex;
        const AL_Mode0_LineMap_t* mapEntry = FindLineMapEntry(
                s_lineSpecs[lineIndex].typeId);
        const char* label = s_lineSpecs[lineIndex].label;
        const char* unitSuffix = ""; // Default empty suffix

        if (mapEntry) {
            UnitConversionInfo_t unitInfo = AL_GetUnitConversion(
                    mapEntry->unitType,
                    cfg->al_lines[s_step].units);
            unitSuffix = unitInfo.suffix;
        } else {
             // Use fallback label "?" if entry not found (already set in Init)
             // unitSuffix remains ""
        }

        length = AL_BuildLayout(layoutId, label, unitSuffix, buf);
        if (length > 0) {
            AL_SendRaw(buf, length);
            s_step++;
        } else {
            // Handle layout build error? Maybe retry or abort?
             status = FS_AL_SETUP_DONE; // Or some error status if defined
        }
    } else if (s_step == cfg->num_al_lines) { // Build page
        length = AL_BuildPage(10, buf); // Page ID 10 references layouts 10-13
        if (length > 0) {
            AL_SendRaw(buf, length);
            s_step++;
            status = FS_AL_SETUP_DONE; // Final step completed successfully
        } else {
             // Handle page build error?
            status = FS_AL_SETUP_DONE; // Consider setup failed/done
        }
    } else { // Steps > 4: Already done
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
    char lineValueStr[4][16]; // Buffer for formatted value strings

    // Calculate altitude above ground level once
    double altitude_agl_mm = (double)gnss->hMSL - (double)cfg->dz_elev;

    // For each line, calculate, convert, and format the value
    for (int i = 0; i < cfg->num_al_lines; i++)
    {
        double baseVal = 0.0;
        double displayVal = 0.0;
        bool display_invalid = false; // Flag to indicate if "----" should be shown

        const AL_Mode0_LineMap_t* mapEntry = FindLineMapEntry(s_lineSpecs[i].typeId);

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
                snprintf(
                        lineValueStr[i],
                        sizeof(lineValueStr[i]),
                        "%.*f",
                        (int)(cfg->al_lines[i].decimals),
                        displayVal);
            }
        } else {
            // Handle case where line type or function is invalid
            display_invalid = true;
        }

        // If any limit check failed, display "----"
        if (display_invalid) {
             snprintf(lineValueStr[i], sizeof(lineValueStr[i]), "----");
        }
    }

    // Build the final packet with the 4 lines
    uint8_t buf[128];
    uint8_t length = AL_BuildPageUpdate(10,
                                        lineValueStr[0],
                                        lineValueStr[1],
                                        lineValueStr[2],
                                        lineValueStr[3],
                                        buf);
    AL_SendRaw(buf, length);
}
