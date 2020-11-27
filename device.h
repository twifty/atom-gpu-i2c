/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LIGHTS_GPU_DEVICE_H
#define _UAPI_LIGHTS_GPU_DEVICE_H

#include <linux/pci.h>

#include "atom/atom.h"
#include "asic-types.h"
#include "i2c.h"

// struct amdgpu_fw_vram_usage {
//     u64 start_offset;
//     u64 size;
//     struct amdgpu_bo *reserved_bo;
//     void *va;
// };

#define AMDGPU_BIOS_NUM_SCRATCH     16
#define AMDGPU_MAX_I2C_BUS          16

struct gpu_device {
    struct pci_dev              *pdev;
    enum amd_asic_type          asic_type;
    resource_size_t             rmmio_base;
    resource_size_t             rmmio_size;
    void __iomem                *rmmio;
    spinlock_t                  mmio_idx_lock;
    // struct amdgpu_mmio_remap    rmmio_remap;

    void __iomem                *rio_mem;
    resource_size_t             rio_mem_size;

    bool                        is_atom_fw;
    uint8_t                     *bios;
    uint32_t                    bios_size;
    struct amdgpu_bo            *stolen_vga_memory;
    uint32_t                    bios_scratch_reg_offset;
    uint32_t                    bios_scratch[AMDGPU_BIOS_NUM_SCRATCH];

    struct atom_context         *atom_context;
    struct card_info            *atom_card_info;

    struct i2c_chan             *i2c_bus[AMDGPU_MAX_I2C_BUS];
    // struct amdgpu_fw_vram_usage fw_vram_usage;
};

struct gpu_device *gpu_device_create (struct pci_dev *pci_dev, unsigned long asic_type);

void gpu_device_destroy (struct gpu_device *gpu_dev);

#endif
