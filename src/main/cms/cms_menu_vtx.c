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

#ifdef CMS

#ifdef VTX_GEN6705

static bool featureRead = false;
static uint8_t cmsx_featureVtxRc = 0, cmsx_vtxBand, cmsx_vtxChannel;

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

static const char * const vtxBandNames[] = {
    "BOSCAM A",
    "BOSCAM B",
    "BOSCAM E",
    "FATSHARK",
    "RACEBAND",
};

static OSD_TAB_t entryVtxBand = {&cmsx_vtxBand,4,&vtxBandNames[0]};
static OSD_UINT8_t entryVtxChannel =  {&cmsx_vtxChannel, 1, 8, 1};

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

uint8_t vtxCurBand;
uint8_t vtxCurChan;
uint8_t vtxCurFreq;

uint8_t vtxCmsBand;
uint8_t vtxCmsChan;
uint16_t vtxCmsFreqRef;

char vtxCmsStatusString[31] = "- -- ---- ----";
//                             m bc ffff pppp
//                             01234567890123

static void vtxCmsUpdateStatusString(void)
{
    vtxCmsStatusString[0] = '*'; // Place holder for opmodel
    vtxCmsStatusString[1] = ' ';
    vtxCmsStatusString[2] = vtx58BandLetter[vtxCurBand];
    vtxCmsStatusString[3] = vtx58ChannelNames[vtxCurChan][0];
    vtxCmsStatusString[4] = ' ';

    vtxCurFreq = vtx58FreqTable[vtxCurBand - 1][vtxCurChan - 1];

debug[3] = vtxCurFreq;
    tfp_sprintf(&vtxCmsStatusString[5], "%4d", vtxCurFreq);
}

static void vtxCmsUpdateFreqRef(void)
{
    if (vtxCmsBand > 0 && vtxCmsChan > 0)
        vtxCmsFreqRef = vtx58FreqTable[vtxCmsBand - 1][vtxCmsChan - 1];
}

static long cmsx_Vtx_onEnter(void)
{
    cmsx_Vtx_FeatureRead();
    cmsx_Vtx_ConfigRead();

    (void)gen6705GetBandChan(&vtxCurBand, &vtxCurChan);

    vtxCmsUpdateStatusString();

    vtxCmsBand = vtxCurBand;
    vtxCmsChan = vtxCurChan;
    vtxCmsUpdateFreqRef();

    return 0;
}

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

    gen6705SetBandChan(vtxCmsBand, vtxCmsChan);
    //gen6705SetRFPower(vtxPowerTable[trampCmsPower-1]);

    gen6705GetBandChan(&vtxCurBand, &vtxCurChan);
    vtxCmsUpdateStatusString();

    return MENU_CHAIN_BACK;
}

static OSD_TAB_t vtxCmsEntBand = { &vtxCmsBand, 5, vtx58BandNames, NULL };
static OSD_TAB_t vtxCmsEntChan = { &vtxCmsChan, 8, vtx58ChannelNames, NULL };
static OSD_UINT16_t vtxCmsEntFreqRef = { &vtxCmsFreqRef, 5600, 5900, 0 };

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

#warning XXX VTX and USE_RTC6705 are obsolete; fix this menu 

#ifdef outdated_singularity
static OSD_UINT8_t entryVtxMode =  {&masterConfig.vtx_mode, 0, 2, 1};
static OSD_UINT16_t entryVtxMhz =  {&masterConfig.vtx_mhz, 5600, 5950, 1};
#endif

static OSD_Entry cmsx_menuVtxEntries[] =
{
    { "-- VTX6705 --", OME_Label, NULL, NULL, 0},
    { "",              OME_Label, NULL,             vtxCmsStatusString, DYNAMIC },

    {"RC CTRL", OME_Bool, NULL, &cmsx_featureVtxRc, 0}, // Shouln't be here

#ifdef outdated_singularity
    {"VTX MODE", OME_UINT8, NULL, &entryVtxMode, 0},
    {"VTX MHZ", OME_UINT16, NULL, &entryVtxMhz, 0},
#endif // VTX

    { "BAND", OME_TAB, vtxCmsConfigBand, &vtxCmsEntBand, 0},
    { "CHAN", OME_TAB, vtxCmsConfigChan, &vtxCmsEntChan, 0},
    { "(FREQ)", OME_UINT16, NULL, &vtxCmsEntFreqRef, DYNAMIC},
    { "SET",    OME_Submenu, cmsMenuChange, &vtxCmsMenuCommence, 0 },

#ifdef outdated_sirinfpv
    {"LOW POWER", OME_Bool, NULL, &masterConfig.vtx_power, 0},
#endif // USE_RTC6705

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuVtx = {
    .GUARD_text = "MENUVTX",
    .GUARD_type = OME_MENU,
    .onEnter = cmsx_Vtx_onEnter,
    .onExit= cmsx_Vtx_onExit,
    .onGlobalExit = cmsx_Vtx_FeatureWriteback,
    .entries = cmsx_menuVtxEntries
};
#endif // VTX_RTC6705

#endif // CMS
