//==============================================================================
//
//  sensor_epsonV340.c - Epson IMU sensor protocol specific code for V340 
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, 
//  SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
//  SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
//
//==============================================================================
#include "sensor_epsonCommon.h"
#include "hcl.h"
#include "hcl_gpio.h"


/*****************************************************************************
** Function name:       registerDump
** Description:         Read all registers for debug purpose
** Parameters:          None
** Return value:        None
*****************************************************************************/
void registerDump(void)
{
    unsigned int debug = TRUE;
    printf("\r\nRegister Dump:\r\n");
    registerRead16(0x00, 0x00, debug);
    registerRead16(0x00, 0x02, debug);
    registerRead16(0x00, 0x04, debug);
    printf("\r\n");
    registerRead16(0x00, 0x06, debug);
    registerRead16(0x00, 0x08, debug);
    registerRead16(0x00, 0x0A, debug);
    printf("\r\n");
    registerRead16(0x00, 0x0C, debug);
    registerRead16(0x00, 0x0E, debug);
    registerRead16(0x00, 0x10, debug);
    printf("\r\n");
    registerRead16(0x00, 0x12, debug);
    registerRead16(0x00, 0x32, debug);
    registerRead16(0x00, 0x34, debug);
    printf("\r\n");
    registerRead16(0x00, 0x36, debug);
    registerRead16(0x00, 0x38, debug);
    registerRead16(0x00, 0x3A, debug);
    printf("\r\n");
    registerRead16(0x00, 0x3C, debug);
    registerRead16(0x00, 0x3E, debug);
    registerRead16(0x00, 0x50, debug);
    printf("\r\n");
    registerRead16(0x00, 0x6A, debug);
    registerRead16(0x00, 0x6C, debug);
    registerRead16(0x00, 0x6E, debug);
    printf("\r\n");
    registerRead16(0x00, 0x70, debug);
    registerRead16(0x00, 0x72, debug);
    registerRead16(0x00, 0x74, debug);
    printf("\r\n");
    registerRead16(0x00, 0x76, debug);
    registerRead16(0x00, 0x78, debug);
    registerRead16(0x00, 0x7A, debug);
    printf("\r\n");
}
