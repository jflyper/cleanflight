/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */


// Get target build configuration
#include "platform.h"

#ifdef VTX_SINGULARITY

//External dependencies
#include "config/config_master.h"
//#include "config/config_eeprom.h"
#include "drivers/vtx_rtc6705.h"

void vtxSingularityInit(void)
{
    rtc6705Init();
    if (masterConfig.vtx_mode == 0) {
        rtc6705SetChannel(masterConfig.vtx_band, masterConfig.vtx_channel);
    } else if (masterConfig.vtx_mode == 1) {
        rtc6705SetFreq(masterConfig.vtx_mhz);
    }
}
#endif
