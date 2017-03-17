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

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#include "build/version.h"

#include "build/debug.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_vtx.h"

#include "common/utils.h"
#include "common/printf.h"

#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "io/vtx_string.h"
#include "drivers/vtx_gen6705.h"

#include "drivers/vtx_debug.h"

#ifdef CMS

#ifdef VTX_GEN6705 // XXX Not really for gen6705

static bool featureRead = false;
static uint8_t cmsx_featureVtxRc = 0; //, cmsx_vtxBand, cmsx_vtxChannel;

static vtxDeviceParam_t *pDevParam = NULL;

static long cmsx_Vtx_FeatureRead(void)
{
    if (!featureRead) {
        cmsx_featureVtxRc = feature(FEATURE_VTXRC) ? 1 : 0;
        featureRead = true;
    }

    return 0;
}

static long cmsx_Vtx_FeatureWriteback(void)
{
    if (featureRead) {
        if (cmsx_featureVtxRc)
            featureSet(FEATURE_VTXRC);
        else
            featureClear(FEATURE_VTXRC);
    }

    return 0;
}

static void cmsx_Vtx_ConfigRead(void)
{
    // Empty
}

static void cmsx_Vtx_ConfigWriteback(void)
{
    // Empty
}

static long cmsx_Vtx_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    cmsx_Vtx_ConfigWriteback();

    return 0;
}

// Device status
static uint8_t vtxCurBand;
static uint8_t vtxCurChan;
static uint16_t vtxCurFreq;
static uint8_t vtxCurPower;
static uint8_t vtxCurFselMode;

// CMS values
static uint8_t vtxCmsBand;
static uint8_t vtxCmsChan;
static uint16_t vtxCmsFreq;
static uint8_t vtxCmsPower;
static uint16_t vtxCmsFreqRef;
static uint8_t vtxCmsFselMode;

char vtxCmsStatusString[15] = "- -- ---- ----";
//                             m bc ffff pppp
//                             01234567890123

static OSD_TAB_mutable_t vtxCmsEntBand;
static OSD_TAB_mutable_t vtxCmsEntChan;
static OSD_TAB_mutable_t vtxCmsEntPower;
static OSD_UINT16_t vtxCmsEntFreqRef = { &vtxCmsFreqRef, 5600, 5900, 0 };

static void vtxCmsUpdateStatusString(void)
{
    vtxCmsStatusString[0] = '*'; // Place holder for opmodel
    vtxCmsStatusString[1] = ' ';

    switch (vtxCurFselMode) {
    case 0: // band/chan
    case 2: // rc
        vtxCmsStatusString[2] = pDevParam->bandLetters[vtxCurBand];
        vtxCmsStatusString[3] = vtxCmsEntChan.names[vtxCurChan][0];
        break;

    case 1: // direct
        vtxCmsStatusString[2] = vtxCmsStatusString[3] = '-';
        break;
    }

    vtxCmsStatusString[4] = ' ';

    tfp_sprintf(&vtxCmsStatusString[5], "%4d", vtxCurFreq);

    vtxCmsStatusString[9] = ' ';
    strncpy(&vtxCmsStatusString[10], vtxCmsEntPower.names[vtxCurPower], 5);
}

static void vtxCmsUpdateStatus(void)
{
    switch (vtxCurFselMode) {
    case 0: // band/chan
    case 2: // direct
        if (vtxCommonGetBandChan(&vtxCurBand, &vtxCurChan)) {
            dprintf(("vtxCmsUpdateStatus: got vtxCurBand %d vtxCurChan %d\r\n", vtxCurBand, vtxCurChan));
            vtxCmsBand = vtxCurBand;
            vtxCmsChan = vtxCurChan;
            // Update frequency translation
            vtxCurFreq = pDevParam->freqTable[(vtxCurBand - 1) * pDevParam->numChan + (vtxCurChan - 1)];
        } else {
            dprintf(("vtxCmsUpdateStatus: vtxCommonGetBandChan failed\r\n"));
        }
        break;

    case 1: // direct
        if (vtxCommonGetFreq(&vtxCurFreq)) {
            vtxCmsFreq = vtxCurFreq;
        }
        break;
    }

    if (vtxCommonGetPowerIndex(&vtxCurPower)) {
        vtxCmsPower = vtxCurPower;
    }
}

static void vtxCmsUpdateFreqRef(void)
{
    if (vtxCmsBand > 0 && vtxCmsChan > 0)
        vtxCmsFreqRef = pDevParam->freqTable[(vtxCmsBand - 1) * pDevParam->numChan + (vtxCmsChan - 1)];
    else
        vtxCmsFreqRef = 0;
}

static const char * const vtxCmsFselModeNames[] = {
    "BANDCHAN",
    "DIRECT  ",
    "VTXRC   "
};

static OSD_TAB_t vtxCmsEntFselMode = { &vtxCmsFselMode, 2, vtxCmsFselModeNames };

static long vtxCmsConfigFselMode(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    vtxCommonSetFselMode(vtxCmsFselMode);
    vtxCommonGetFselMode(&vtxCurFselMode);

    vtxCmsUpdateStatus();

    return 0;
}

static OSD_Entry vtxCmsMenuConfigEntries[] = {
    { "- VTX CONFIG -", OME_Label, NULL, NULL, 0 },
    { "FSEL MODE", OME_TAB,    vtxCmsConfigFselMode, &vtxCmsEntFselMode, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu vtxCmsMenuConfig = {
    .GUARD_text = "VTXCFG",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = vtxCmsMenuConfigEntries
};

static long vtxCmsConfigBand(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (vtxCmsBand == 0)
        // Bounce back
        vtxCmsBand = 1;
    else
        vtxCmsUpdateFreqRef();

    return 0;
}

static long vtxCmsConfigChan(displayPort_t *pDisp, const void *self)
{   
    UNUSED(pDisp);
    UNUSED(self);
    
    if (vtxCmsChan == 0)
        // Bounce back 
        vtxCmsChan = 1;
    else
        vtxCmsUpdateFreqRef();
    
    return 0;
}

static long vtxCmsCommence(displayPort_t *pDisp, const void *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    switch (vtxCurFselMode) {
    case 0: // Band/Chan
        dprintf(("vtxCmsCommence: band/chan mode\r\n"));
        vtxCommonSetBandChan(vtxCmsBand, vtxCmsChan);
        vtxCommonGetBandChan(&vtxCurBand, &vtxCurChan);
        break;

    case 1: // Direct
        dprintf(("vtxCmsCommence: direct mode\r\n"));
        vtxCommonSetFreq(vtxCmsFreq);
        vtxCommonGetFreq(&vtxCurFreq);
        break;

    case 2: // VTXRC
        dprintf(("vtxCmsCommence: direct mode\r\n"));
        break;
    }

    vtxCommonSetPowerByIndex(vtxCmsPower);
    vtxCommonGetPowerIndex(&vtxCurPower);

    vtxCmsUpdateStatusString();

    return MENU_CHAIN_BACK;
}

static long vtxCmsConfigPower(displayPort_t *pDisp, const void *self)
{   
    UNUSED(pDisp);
    UNUSED(self);

    if (vtxCmsPower == 0)
        // Bounce back
        vtxCmsPower = 1;

    dprintf(("vtxCmsConfigPower: vtxCmsPower %d\r\n", vtxCmsPower));

    return 0;
}

OSD_Entry vtxCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },
    { "YES",     OME_Funcall, vtxCmsCommence, NULL, 0 },
    { "BACK",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

CMS_Menu vtxCmsMenuCommence = {
    .GUARD_text = "XVTXCOM",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = vtxCmsMenuCommenceEntries,
};

// XXX Lots of duplicate entries here; room for optimization.
// XXX Need to restructure the OSD_Entry [] into OSD_Entry *[] for maximum flexibility?

static OSD_Entry cmsx_menuVtxBandChanModeEntries[] =
{
    { "-- VTX6705 --", OME_Label, NULL, NULL, 0},
    { "",              OME_Label, NULL, vtxCmsStatusString, DYNAMIC },
    { "BAND", OME_TAB, vtxCmsConfigBand, &vtxCmsEntBand, 0 },
    { "CHAN", OME_TAB, vtxCmsConfigChan, &vtxCmsEntChan, 0 },
    { "FREQ", OME_UINT16, NULL, &vtxCmsEntFreqRef, DYNAMIC},
    { "POWER", OME_TAB, vtxCmsConfigPower, &vtxCmsEntPower, 0 },
    { "SET",    OME_Submenu, cmsMenuChange, &vtxCmsMenuCommence, 0 },
    { "CONFIG", OME_Submenu, cmsMenuChange, &vtxCmsMenuConfig, 0 },

#ifdef outdated_sirinfpv
    {"LOW POWER", OME_Bool, NULL, &masterConfig.vtx_power, 0},
#endif // USE_RTC6705

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

static OSD_Entry cmsx_menuVtxDirectModeEntries[] =
{
    { "-- VTX6705 --", OME_Label, NULL, NULL, 0},
    { "",              OME_Label, NULL, vtxCmsStatusString, DYNAMIC },
    { "FREQ",          OME_UINT16, NULL, &(OSD_UINT16_t){ &vtxCmsFreq, 5000, 5900, 1 }, 0 },
    { "POWER", OME_TAB, vtxCmsConfigPower, &vtxCmsEntPower, 0 },
    { "SET",    OME_Submenu, cmsMenuChange, &vtxCmsMenuCommence, 0 },
    { "CONFIG", OME_Submenu, cmsMenuChange, &vtxCmsMenuConfig, 0 },
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

static OSD_Entry cmsx_menuVtxRcModeEntries[] =
{
    { "-- VTX6705 --", OME_Label, NULL, NULL, 0},
    { "",              OME_Label, NULL, vtxCmsStatusString, DYNAMIC },
    { "CONFIG", OME_Submenu, cmsMenuChange, &vtxCmsMenuConfig, 0 },
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

static OSD_Entry cmsx_menuVtxNullEntries[] = 
{
    { "-- VTX6705 --", OME_Label, NULL, NULL, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuVtx; // Forward

static long cmsx_Vtx_onEnter(void)
{
    // Setup band, channel and power tables
    if ((pDevParam = vtxCommonGetDeviceParam()) == NULL) {
        cmsx_menuVtx.entries = cmsx_menuVtxNullEntries;
        return 0;
    }

    vtxCmsEntBand.val = &vtxCmsBand;
    vtxCmsEntBand.max = pDevParam->numBand;
    vtxCmsEntBand.names = pDevParam->bandNames;

    vtxCmsEntChan.val = &vtxCmsChan;
    vtxCmsEntChan.max = pDevParam->numChan;
    vtxCmsEntChan.names = pDevParam->chanNames;

    vtxCmsEntPower.val = &vtxCmsPower;
    vtxCmsEntPower.max = pDevParam->numPower;
    vtxCmsEntPower.names = pDevParam->powerNames;

    dprintf(("cmsx_Vtx_onEnter: power got max %d names[1] %s\r\n",
        vtxCmsEntPower.max, vtxCmsEntPower.names[1]));

    if (vtxCommonGetFselMode(&vtxCurFselMode)) {
        dprintf(("cmsx_Vtx_onEnter: got vtxCurFselMode %d\r\n", vtxCurFselMode));
        vtxCmsFselMode = vtxCurFselMode;
    } else {
        dprintf(("cmsx_Vtx_onEnter: vtxCommonGetFselMode failed\r\n"));
    }

    cmsx_Vtx_FeatureRead();
    cmsx_Vtx_ConfigRead();

    vtxCmsUpdateStatus();

    switch (vtxCurFselMode) {
    case 0: // Band/Chan
        cmsx_menuVtx.entries = cmsx_menuVtxBandChanModeEntries;
        vtxCmsUpdateFreqRef();
#if 0
        if (vtxCommonGetBandChan(&vtxCurBand, &vtxCurChan)) {
            vtxCmsBand = vtxCurBand;
            vtxCmsChan = vtxCurChan;
            vtxCmsUpdateFreqRef();
        }
#endif
        break;

    case 1: // Direct
        cmsx_menuVtx.entries = cmsx_menuVtxDirectModeEntries;
#if 0
        if (vtxCommonGetFreq(&vtxCurFreq)) {
            dprintf(("cmsx_Vtx_onEnter: got vtxCurFreq %d\r\n", vtxCurFreq));
            vtxCmsFreq = vtxCurFreq;
        }
#endif
        break;

    case 2: // VTXRC
        cmsx_menuVtx.entries = cmsx_menuVtxRcModeEntries;
        break;
    }

#if 0
    if (vtxCommonGetPowerIndex(&vtxCurPower)) {
        vtxCmsPower = vtxCurPower;
    }
#endif

    vtxCmsUpdateStatusString();

    return 0;
}

CMS_Menu cmsx_menuVtx = {
    .GUARD_text = "MENUVTXBC",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .onGlobalExit = cmsx_Vtx_FeatureWriteback,
    .entries = cmsx_menuVtxNullEntries,
};
#endif // VTX_RTC6705

#endif // CMS
