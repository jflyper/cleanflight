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
#include "io/vtx_gen6705.h"

#include "drivers/vtx_debug.h"

#ifdef CMS

#ifdef VTX_GEN6705 // XXX Not really for gen6705

static bool featureRead = false;
static uint8_t cmsx_featureVtxRc = 0; //, cmsx_vtxBand, cmsx_vtxChannel;

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
#ifdef VTX
    cmsx_vtxBand = masterConfig.vtx_band;
    cmsx_vtxChannel = masterConfig.vtx_channel + 1;
#endif // VTX

#ifdef USE_RTC6705
    cmsx_vtxBand = masterConfig.vtx_channel / 8;
    cmsx_vtxChannel = masterConfig.vtx_channel % 8 + 1;
#endif // USE_RTC6705
}

static void cmsx_Vtx_ConfigWriteback(void)
{
#ifdef VTX
    masterConfig.vtx_band = cmsx_vtxBand;
    masterConfig.vtx_channel = cmsx_vtxChannel - 1;
#endif // VTX

#ifdef USE_RTC6705
    masterConfig.vtx_channel = cmsx_vtxBand * 8 + cmsx_vtxChannel - 1;
#endif // USE_RTC6705
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

char vtxCmsStatusString[31] = "- -- ---- ----";
//                             m bc ffff pppp
//                             01234567890123

static void vtxCmsUpdateStatusString(void)
{
    vtxCmsStatusString[0] = '*'; // Place holder for opmodel
    vtxCmsStatusString[1] = ' ';

    switch (vtxCurFselMode) {
    case 0: // band/chan
    case 2: // rc
        vtxCmsStatusString[2] = vtx58BandLetter[vtxCurBand];
        vtxCmsStatusString[3] = vtx58ChannelNames[vtxCurChan][0];
        break;

    case 1: // direct
        vtxCmsStatusString[2] = vtxCmsStatusString[3] = '-';
        break;
    }

    vtxCmsStatusString[4] = ' ';

    // XXX Power is missing

    switch (vtxCurFselMode) {
    case 0: // band/chan
    case 2: // rc
        vtxCurFreq = vtx58FreqTable[vtxCurBand - 1][vtxCurChan - 1];
        tfp_sprintf(&vtxCmsStatusString[5], "%4d", vtxCurFreq);
        break;

    case 1:
        tfp_sprintf(&vtxCmsStatusString[5], "%4d", vtxCurFreq);
        break;
    }
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

    // XXX Handle power also
}

static void vtxCmsUpdateFreqRef(void)
{
    if (vtxCmsBand > 0 && vtxCmsChan > 0)
        vtxCmsFreqRef = vtx58FreqTable[vtxCmsBand - 1][vtxCmsChan - 1];
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
    case 0: // band/chan
        dprintf(("vtxCmsCommence: band/chan mode\r\n"));
        vtxCommonSetBandChan(vtxCmsBand, vtxCmsChan);
        vtxCommonGetBandChan(&vtxCurBand, &vtxCurChan);
        break;

    case 1: // direct
        dprintf(("vtxCmsCommence: direct mode\r\n"));
        vtxCommonSetFreq(vtxCmsFreq);
        vtxCommonGetFreq(&vtxCurFreq);
        break;

    case 2: // rc
        dprintf(("vtxCmsCommence: direct mode\r\n"));
        break;
    }

    //vtxCommonSetRFPower(vtxPowerTable[trampCmsPower-1]); XXX Not yet
    //vtxCommonGetRFPower(vtxPowerTable[trampCmsPower-1]); XXX Not yet

    vtxCmsUpdateStatusString();

    return MENU_CHAIN_BACK;
}

static OSD_TAB_t vtxCmsEntBand = { &vtxCmsBand, 4, vtx58BandNames, NULL };
static OSD_TAB_t vtxCmsEntChan = { &vtxCmsChan, 7, vtx58ChannelNames, NULL };
static OSD_UINT16_t vtxCmsEntFreqRef = { &vtxCmsFreqRef, 5600, 5900, 0 };

static const char * const vtxCmsPowerNames[] =
//static char * const vtxCmsPowerNames[] =
{
    "----",
    "MIN ",
    "LOW ",
    "HIGH",
    "MAX "
};

#if 0
static char * const vtxCmsPowerNamesAlt[] = {
    "-------",
    "ALTMIN ",
    "ALTLOW ",
    "ALTHIGH",
    "ALTMAX "
};
#endif

//static OSD_TAB_mutable_t vtxCmsEntPower = { &vtxCmsPower, 3, vtxCmsPowerNames, NULL };
static OSD_TAB_t vtxCmsEntPower = { &vtxCmsPower, 4, vtxCmsPowerNames };

static long vtxCmsConfigPower(displayPort_t *pDisp, const void *self)
{   
    UNUSED(pDisp);
    UNUSED(self);

    if (vtxCmsPower == 0)
        // Bounce back
        vtxCmsPower = 1;

    return 0;
}

static OSD_Entry vtxCmsMenuCommenceEntries[] = {
    { "CONFIRM", OME_Label,   NULL,          NULL, 0 },
    { "YES",     OME_Funcall, vtxCmsCommence, NULL, 0 },
    { "BACK",    OME_Back, NULL, NULL, 0 },
    { NULL,      OME_END, NULL, NULL, 0 }
};

static CMS_Menu vtxCmsMenuCommence = {
    .GUARD_text = "XVTXCOM",
    .GUARD_type = OME_MENU,
    .onEnter = NULL,
    .onExit = NULL,
    .onGlobalExit = NULL,
    .entries = vtxCmsMenuCommenceEntries,
};

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

CMS_Menu cmsx_menuVtx; // Forward

static long cmsx_Vtx_onEnter(void)
{
#if 0
    // Test writability of vtxCmsEntPower
    vtxCmsEntPower.names = vtxCmsPowerNamesAlt;
#endif

    if (vtxCommonGetFselMode(&vtxCurFselMode)) {
        dprintf(("cmsx_Vtx_onEnter: got vtxCurFselMode %d\r\n", vtxCurFselMode));
        vtxCmsFselMode = vtxCurFselMode;
    } else {
        dprintf(("cmsx_Vtx_onEnter: vtxCommonGetFselMode failed\r\n"));
    }

    switch (vtxCurFselMode) {
    case 0: // Band/Chan
        cmsx_menuVtx.entries = cmsx_menuVtxBandChanModeEntries;
        if (vtxCommonGetBandChan(&vtxCurBand, &vtxCurChan)) {
            vtxCmsBand = vtxCurBand;
            vtxCmsChan = vtxCurChan;
            vtxCmsUpdateFreqRef();
        }
        break;

    case 1: // Direct
        cmsx_menuVtx.entries = cmsx_menuVtxDirectModeEntries;
        if (vtxCommonGetFreq(&vtxCurFreq)) {
            dprintf(("cmsx_Vtx_onEnter: got vtxCurFreq %d\r\n", vtxCurFreq));
            vtxCmsFreq = vtxCurFreq;
        }
        break;

    case 2: // VTXRC
        cmsx_menuVtx.entries = cmsx_menuVtxRcModeEntries;
        break;
    }

    cmsx_Vtx_FeatureRead();
    cmsx_Vtx_ConfigRead();

   vtxCmsUpdateStatusString();

    return 0;
}

CMS_Menu cmsx_menuVtx = {
    .GUARD_text = "MENUVTXBC",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .onGlobalExit = cmsx_Vtx_FeatureWriteback,
    .entries = NULL,
};
#endif // VTX_RTC6705

#endif // CMS
