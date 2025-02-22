#include "main.h"
#include "activelook.h"
#include "activelook_client.h"
#include "activelook_mode0.h"
#include "gnss.h"
#include <stdio.h>
#include <string.h>

// A static variable that persists across calls, local to Mode0
static int s_step = 0;

/*******************************************************************************
 * A helper to do a single WriteWithoutResp, building or storing a raw packet.
 * We do only one call per pass of the state machine to avoid spamming BLE.
 ******************************************************************************/
static void AL_SendRaw(const uint8_t *data, uint16_t length)
{
    FS_ActiveLook_Client_WriteWithoutResp(data, length);
}

/*******************************************************************************
 * Build a "layoutSave" command for one row, with or without an icon, etc.
 * We'll keep it simple: no icon here, just the normal text area. Adjust as needed.
 *
 * The snippet below demonstrates how you might build the layout data.
 * If you want an icon, add sub-commands for 'img', 'txt', etc. in "extra[]".
 *
 * Return the final size of the packet in 'outLen', so the caller can do:
 *    AL_SendRaw(buffer, outLen);
 ******************************************************************************/
static uint8_t AL_BuildLayout(
    uint8_t layoutId,
    const char *headingText,
    const char *unitsText,
    uint8_t  *outBuf
)
{
    // Start building the buffer
    uint8_t idx = 0;
    outBuf[idx++] = 0xFF;  // Start
    outBuf[idx++] = 0x60;  // "layoutSave"
    outBuf[idx++] = 0x00;  // format => 1-byte length
    uint8_t lenPos = idx++; // We'll fill the total length later

    // layout ID
    outBuf[idx++] = layoutId;

    // We'll fill the additional commands size later
    uint8_t addCmdSizePos = idx++;

    // X (2 bytes, MSB first)
    // Suppose x=0 => 0x00, 0x00
    outBuf[idx++] = 0x00;  // x hi
    outBuf[idx++] = 0x00;  // x lo

    // Y (1 byte!)
    outBuf[idx++] = 0x00;

    // Width (2 bytes, MSB first). E.g. 304 => 0x01,0x30
    outBuf[idx++] = 0x01;  // width hi
    outBuf[idx++] = 0x30;  // width lo

    // Height (1 byte!)
    // e.g. 40 => 0x28
    outBuf[idx++] = 0x28;

    // ForeColor, BackColor, Font
    outBuf[idx++] = 15; // foreColor
    outBuf[idx++] = 0;  // backColor
    outBuf[idx++] = 2;  // font

    // TextValid
    outBuf[idx++] = 1;  // 1 => use dynamic text argument

    // Text X (2 bytes, MSB first), say 200
    outBuf[idx++] = 0;
    outBuf[idx++] = 200;

    // Text Y (1 byte!), e.g. 40
    outBuf[idx++] = 40;

    // Text Rotation (1 byte)
    outBuf[idx++] = 4;

    // Text Opacity (1 byte)
    outBuf[idx++] = 1;

    //-------------------------------------------------------
    // Additional sub-commands for icon or units, if desired:
    //-------------------------------------------------------
    uint8_t extra[64];
    uint8_t e = 0;

    // Example: color=15
    extra[e++] = 0x03;  // sub-cmd ID "color"
    extra[e++] = 15;    // color=15

    // Example: font=1
    extra[e++] = 0x04;
    extra[e++] = 1;     // smaller font

    // Example: txt => position= (260,35)
    extra[e++] = 0x09;  // "text"
    // x=80 => 2 bytes
    extra[e++] = 1;
    extra[e++] = 5;
    // y=0 => 2 bytes
    extra[e++] = 0;
    extra[e++] = 35;

    // Then one byte with length, then the string
    size_t headingLen = strlen(headingText);
    extra[e++] = (uint8_t)headingLen;
    memcpy(&extra[e], headingText, headingLen);
    e += headingLen;

    // Example: font=1
    extra[e++] = 0x04;
    extra[e++] = 1;     // smaller font

    // Example: txt => position= (80,35)
    extra[e++] = 0x09;  // "text"
    // x=80 => 2 bytes
    extra[e++] = 0;
    extra[e++] = 80;
    // y=0 => 2 bytes
    extra[e++] = 0;
    extra[e++] = 35;

    // Then one byte with length, then the string
    size_t unitsLen = strlen(unitsText);
    extra[e++] = (uint8_t)unitsLen;
    memcpy(&extra[e], unitsText, unitsLen);
    e += unitsLen;

    // Done building sub-commands
    outBuf[addCmdSizePos] = e;
    memcpy(&outBuf[idx], extra, e);
    idx += e;

    // Footer
    outBuf[idx++] = 0xAA;

    // Fill total length
    outBuf[lenPos] = idx;
    return idx;
}

/*******************************************************************************
 * Build a "pageSave" command with 4 sublayouts. We'll call it ID=10, referencing
 * layoutIds=1..4, each at offset x=0,y=0 inside the page.
 *
 * Return the final length so caller can AL_SendRaw() it.
 ******************************************************************************/
static uint8_t AL_BuildPage(uint8_t pageId, uint8_t *outBuf)
{
    /* pageSave => command=0x80.
       Format => 0x00 => 1-byte length.
       The structure is: pageId, then for each sublayout:
         layoutId(1B), x(2B?), y(2B?), ...
       The exact number of bytes for x,y depends on your firmware doc.
       We'll assume x=2 bytes, y=2 bytes for safety. Or if doc says otherwise, adapt.
    */
    uint8_t idx = 0;

    outBuf[idx++] = 0xFF;  // start
    outBuf[idx++] = 0x80;  // "pageSave"
    outBuf[idx++] = 0x00;  // format => 1B length
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // We have 4 sublayouts: layout #1..4
    // Each sublayout => layoutId(1B), x(2B), y(2B)
    // We'll set x=0,y=0 for each.

    // sublayout #1
    outBuf[idx++] = 10; // layoutId
    outBuf[idx++] = 0x00; // x hi
    outBuf[idx++] = 0x00; // x lo
    outBuf[idx++] = 168; // y lo

    // sublayout #2
    outBuf[idx++] = 11;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 128;

    // sublayout #3
    outBuf[idx++] = 12;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 88;

    // sublayout #4
    outBuf[idx++] = 13;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 0x00;
    outBuf[idx++] = 48;

    // Footer
    outBuf[idx++] = 0xAA;

    // Fill length
    outBuf[lenPos] = idx;

    return idx;
}

/*******************************************************************************
 * Build a single "pageClearAndDisplay" command with 4 text arguments,
 * separated by '\0'. That is the entire update in one WWR call.
 ******************************************************************************/
static uint8_t AL_BuildPageUpdate(uint8_t pageId,
                                  const char *line1,
                                  const char *line2,
                                  const char *line3,
                                  const char *line4,
                                  uint8_t *outBuf)
{
    /* layoutClearAndDisplay or pageClearAndDisplay => 0x6A
       structure => [ pageId, line1\0 line2\0 line3\0 line4\0 ]
    */
    uint8_t idx = 0;

    outBuf[idx++] = 0xFF;
    outBuf[idx++] = 0x86; // "pageClearAndDisplay"
    outBuf[idx++] = 0x00; // format => 1B length
    uint8_t lenPos = idx++;

    outBuf[idx++] = pageId;

    // Copy line1 plus a null
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

    // Footer
    outBuf[idx++] = 0xAA;

    // fill length
    outBuf[lenPos] = idx;
    return idx;
}

// Mode0's init() => reset s_step
void FS_ActiveLook_Mode0_Init(void)
{
    s_step = 0;
}

// Mode0's setup() => multi-step approach
FS_ActiveLook_SetupStatus_t FS_ActiveLook_Mode0_Setup(void)
{
    uint8_t buf[128];
    uint8_t length;

    switch (s_step)
    {
    case 0:
        length = AL_BuildLayout(10, "Hdg:", "deg", buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 1:
        length = AL_BuildLayout(11, "Vh:", "m/s", buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 2:
        length = AL_BuildLayout(12, "Vd:", "m/s", buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 3:
        length = AL_BuildLayout(13, "Ele:", "m", buf);
        AL_SendRaw(buf, length);
        s_step++;
        return FS_AL_SETUP_IN_PROGRESS;

    case 4:
        length = AL_BuildPage(10, buf);
        AL_SendRaw(buf, length);
        s_step++;
        // We are done with setup
        return FS_AL_SETUP_DONE;

    default:
        // If we get here, we’re already done
        return FS_AL_SETUP_DONE;
    }
}

// Mode0's update() => build the new page and send
void FS_ActiveLook_Mode0_Update(void)
{
    char line1[16], line2[16], line3[16], line4[16];
    uint8_t buf[128];
    uint8_t length;

	/* Build 4 strings from our GNSS data. Then do a single "pageClearAndDisplay" */
    const FS_GNSS_Data_t *data = FS_GNSS_GetData();
    snprintf(line1, sizeof(line1), "%ld",(long)(data->heading));
    snprintf(line2, sizeof(line2), "%ld",(long)(data->gSpeed/100));
    snprintf(line3, sizeof(line3), "%ld",(long)(data->velD/1000));
    snprintf(line4, sizeof(line4), "%ld",(long)(data->hMSL/1000));

    length = AL_BuildPageUpdate(10, line1, line2, line3, line4, buf);
    AL_SendRaw(buf, length);
}

