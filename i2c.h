/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LIGHTS_I2C_H
#define _UAPI_LIGHTS_I2C_H

#include <linux/i2c.h>
#include "device.h"

struct i2c_bus_rec {
    bool                        valid;
    /* id used by atom */
    uint8_t                     i2c_id;
    /* id used by atom */
    // enum amdgpu_hpd_id          hpd;
    /* can be used with hw i2c engine */
    bool                        hw_capable;
    /* uses multi-media i2c engine */
    bool                        mm_i2c;
    /* regs and bits */
    uint32_t                    mask_clk_reg;
    uint32_t                    mask_data_reg;
    uint32_t                    a_clk_reg;
    uint32_t                    a_data_reg;
    uint32_t                    en_clk_reg;
    uint32_t                    en_data_reg;
    uint32_t                    y_clk_reg;
    uint32_t                    y_data_reg;
    uint32_t                    mask_clk_mask;
    uint32_t                    mask_data_mask;
    uint32_t                    a_clk_mask;
    uint32_t                    a_data_mask;
    uint32_t                    en_clk_mask;
    uint32_t                    en_data_mask;
    uint32_t                    y_clk_mask;
    uint32_t                    y_data_mask;
};

struct i2c_chan {
    struct i2c_adapter          adapter;
    struct gpu_device           *gpu_dev;
    // struct drm_device           *dev;
    // struct i2c_algo_bit_data    bit;
    struct i2c_bus_rec          rec;
    // struct drm_dp_aux           aux;
    // bool                        has_aux;
    struct mutex                mutex;
};

void atombios_i2c_init(struct gpu_device *);
void atombios_i2c_fini(struct gpu_device *);

#endif
