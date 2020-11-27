/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __AMD_ASIC_TYPE_H__
#define __AMD_ASIC_TYPE_H__

enum amd_asic_type {
    CHIP_TAHITI = 0,
    CHIP_PITCAIRN,          /* 1 */      // AMDGPU_FAMILY_SI
    CHIP_VERDE,             /* 2 */      // AMDGPU_FAMILY_SI
    CHIP_OLAND,             /* 3 */      // AMDGPU_FAMILY_SI
    CHIP_HAINAN,            /* 4 */      // AMDGPU_FAMILY_SI
    CHIP_BONAIRE,           /* 5 */      // 0x0113 AMDGPU_FAMILY_CI DCE_VERSION_8_0
    CHIP_KAVERI,            /* 6 */      // AMDGPU_FAMILY_CI DCE_VERSION_8_0
    CHIP_KABINI,            /* 7 */      // AMDGPU_FAMILY_CI DCE_VERSION_8_0
    CHIP_HAWAII,            /* 8 */      // 0x0127 AMDGPU_FAMILY_CI DCE_VERSION_8_0
    CHIP_MULLINS,           /* 9 */      // AMDGPU_FAMILY_CI DCE_VERSION_8_0

    CHIP_TOPAZ,             /* 10 */     // 0x0100 AMDGPU_FAMILY_VI
    CHIP_TONGA,             /* 11 */     // 0x0113 AMDGPU_FAMILY_VI DCE_VERSION_10_0
    CHIP_FIJI,              /* 12 */     // 0x013b AMDGPU_FAMILY_VI DCE_VERSION_10_0
    CHIP_CARRIZO,           /* 13 */     // 0x0100 AMDGPU_FAMILY_VI
    CHIP_STONEY,            /* 14 */     // 0x0160 AMDGPU_FAMILY_VI
    
    CHIP_POLARIS10,         /* 15 */     // 0x014f AMDGPU_FAMILY_VI DCE_VERSION_11_2 dce112_create_resource_pool
    CHIP_POLARIS11,         /* 16 */     // 0x0159 AMDGPU_FAMILY_VI DCE_VERSION_11_2 dce112_create_resource_pool
    CHIP_POLARIS12,         /* 17 */     // 0x0163 AMDGPU_FAMILY_VI DCE_VERSION_11_2 dce112_create_resource_pool
    CHIP_VEGAM,             /* 18 */     // 0x016d AMDGPU_FAMILY_VI DCE_VERSION_11_22 dce112_create_resource_pool
    CHIP_VEGA10,            /* 19 */     // 0x0100 AMDGPU_FAMILY_AI DCE_VERSION_12_0 dce120_create_resource_pool
    CHIP_VEGA12,            /* 20 */     // 0x0113 AMDGPU_FAMILY_AI DCE_VERSION_12_0 dce120_create_resource_pool
    CHIP_VEGA20,            /* 21 */     // 0x0127 AMDGPU_FAMILY_AI DCE_VERSION_12_1 dce120_create_resource_pool
    CHIP_RAVEN,             /* 22 */     // 0x???? AMDGPU_FAMILY_AI DCE_VERSION_12_0 dce120_create_resource_pool
    CHIP_ARCTURUS,          /* 23 */     // 0x0131 AMDGPU_FAMILY_AI DCE_VERSION_12_0 dce120_create_resource_pool
    CHIP_RENOIR,            /* 24 */     // 0x0190 AMDGPU_FAMILY_AI DCN_VERSION_2_1 dcn21_create_resource_pool
    CHIP_NAVI10,            /* 25 */     // 0x0100 AMDGPU_FAMILY_NV DCN_VERSION_2_0 dcn20_create_resource_poo
    CHIP_NAVI14,            /* 26 */     // 0x011f AMDGPU_FAMILY_NV DCN_VERSION_2_0 dcn20_create_resource_poo
    CHIP_NAVI12,            /* 27 */     // 0x0109 AMDGPU_FAMILY_NV DCN_VERSION_2_0 dcn20_create_resource_poo
    CHIP_SIENNA_CICHLID,    /* 28 */     // 0x0127 AMDGPU_FAMILY_NV DCN_VERSION_3_0 dcn30_create_resource_pool
    CHIP_NAVY_FLOUNDER,     /* 29 */     // 0x0131 AMDGPU_FAMILY_NV DCN_VERSION_3_0 dcn30_create_resource_pool
    CHIP_LAST,
};

#endif /*__AMD_ASIC_TYPE_H__ */
