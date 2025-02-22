#include "activelook_mode0.h"
#include "activelook_client.h"    // For FS_ActiveLook_Client_WriteWithoutResp
#include "config.h"               // For FS_Config_Get()
#include "gnss.h"                 // For FS_GNSS_GetData()
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
   1. Data Structures for Each Line
   -------------------------------------------------------------------------- */

/**
 * A function pointer type that, given GNSS data, returns a double
 * representing the value we want displayed. e.g. heading, alt, etc.
 */
typedef double (*LineValueFn_t)(const FS_GNSS_Data_t*);

/* Simple "getter" functions for each known line type */
static double LN_HSpeed(const FS_GNSS_Data_t *d)    { return (double)d->gSpeed / 100.0;     }
static double LN_VSpeed(const FS_GNSS_Data_t *d)    { return (double)d->velD   / 1000.0;    }
static double LN_Heading(const FS_GNSS_Data_t *d)   { return (double)d->heading;            }
static double LN_Altitude(const FS_GNSS_Data_t *d)  { return (double)d->hMSL   / 1000.0;    }

/**
 * A table entry describing one line type:
 *   - typeId:  which numeric ID from the config (0,1,7,12, etc.)
 *   - label:   text label to show on the layout, e.g. "HSpd:"
 *   - units:   text units after the label, e.g. "m/s"
 *   - fn:      function that returns the numeric value for the line
 */
typedef struct {
    uint8_t       typeId;
    const char   *label;
    const char   *units;
    LineValueFn_t fn;
} AL_Mode0_LineMap_t;

/*
   A dictionary of possible line types.
   Fill in as many as you want to support.
*/
static const AL_Mode0_LineMap_t s_lineMap[] =
{
    {  0, "HSpd:",  "m/s",  LN_HSpeed   },
    {  1, "VSpd:",  "m/s",  LN_VSpeed   },
    { 12, "Alt:",   "m",    LN_Altitude },
    { 13, "Hdg:",   "deg",  LN_Heading  },
};
static const unsigned s_lineMapCount = sizeof(s_lineMap) / sizeof(s_lineMap[0]);

/**
 * For the four lines in this mode, we store a "resolved" spec.
 * That includes a pointer to the label, units, and function pointer
 * to compute the value. We fill this in FS_ActiveLook_Mode0_Init()
 * based on the user config (al_line_1..4).
 */
typedef struct {
    const char    *label;
    const char    *units;
    LineValueFn_t  fn;
} AL_Mode0_LineSpec_t;

static AL_Mode0_LineSpec_t s_lineSpecs[4];

/**
 * We'll also keep a "setup" step variable for multi-step layout creation.
 */
static int s_step = 0;

/* --------------------------------------------------------------------------
   2. Helpers for Sending Commands / Building Layout
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
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x80;
    outBuf[idx++] = 0x00;
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // Sublayout #1 => layout=10
    outBuf[idx++] = 10;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 168;

    // Sublayout #2 => layout=11
    outBuf[idx++] = 11;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 128;

    // Sublayout #3 => layout=12
    outBuf[idx++] = 12;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 88;

    // Sublayout #4 => layout=13
    outBuf[idx++] = 13;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 48;

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
    s_step = 0;

    // Read config
    const FS_Config_Data_t *cfg = FS_Config_Get();

    // Grab the line IDs the user wants
    uint8_t lineTypes[4] = {
        cfg->al_line_1,
        cfg->al_line_2,
        cfg->al_line_3,
        cfg->al_line_4
    };

    // For each line i, pick a default label/units/fn if not found
    for (int i=0; i<4; i++)
    {
        s_lineSpecs[i].label = "?";
        s_lineSpecs[i].units = "";
        s_lineSpecs[i].fn    = NULL;

        // Search the dictionary for a match
        for (unsigned k=0; k < s_lineMapCount; k++)
        {
            if (s_lineMap[k].typeId == lineTypes[i])
            {
                s_lineSpecs[i].label = s_lineMap[k].label;
                s_lineSpecs[i].units = s_lineMap[k].units;
                s_lineSpecs[i].fn    = s_lineMap[k].fn;
                break;
            }
        }
    }
}

/**
 * FS_ActiveLook_Mode0_Setup()
 *
 * Multi-step routine to define each layout #10..13 with the user-chosen
 * label/units, then define the page referencing them.
 * We do one BLE command per call, returning FS_AL_SETUP_IN_PROGRESS
 * until we're done.
 */
FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void)
{
    uint8_t buf[128];
    uint8_t length;

    switch (s_step)
    {
    case 0:
        length = AL_BuildLayout(10,
                                s_lineSpecs[0].label,
                                s_lineSpecs[0].units,
                                buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 1:
        length = AL_BuildLayout(11,
                                s_lineSpecs[1].label,
                                s_lineSpecs[1].units,
                                buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 2:
        length = AL_BuildLayout(12,
                                s_lineSpecs[2].label,
                                s_lineSpecs[2].units,
                                buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 3:
        length = AL_BuildLayout(13,
                                s_lineSpecs[3].label,
                                s_lineSpecs[3].units,
                                buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 4:
        // Finally define the page referencing #10..#13
        length = AL_BuildPage(10, buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_DONE;

    default:
        // If we get here, we've already finished or are in error
        return FS_AL_SETUP_DONE;
    }
}

/**
 * FS_ActiveLook_Mode0_Update()
 *
 * Called periodically (e.g. from a timer or when new GNSS data arrives)
 * to fill the 4 lines with fresh values.
 *
 * We call each line's function pointer to compute a double, then format
 * it into a string, then send a single "pageClearAndDisplay" command
 * containing all 4 lines.
 */
void FS_ActiveLook_Mode0_Update(void)
{
    const FS_GNSS_Data_t *gnss = FS_GNSS_GetData();
    char line[4][16];

    // For each line, call the function pointer to get a numeric value
    for (int i=0; i<4; i++)
    {
        double val = 0.0;
        if (s_lineSpecs[i].fn)
            val = s_lineSpecs[i].fn(gnss);

        // Format as integer or float, up to you
        snprintf(line[i], sizeof(line[i]), "%ld", (long)val);
    }

    // Build the final packet with the 4 lines
    uint8_t buf[128];
    uint8_t length = AL_BuildPageUpdate(10,
                                        line[0],
                                        line[1],
                                        line[2],
                                        line[3],
                                        buf);
    AL_SendRaw(buf, length);
}
