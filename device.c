// SPDX-License-Identifier: GPL-2.0


#include "debug.h"
#include "device.h"
// #include "asic-types.h"
// #include "atom/atomfirmware.h"

#define AMD_IS_VALID_VBIOS(p) ((p)[0] == 0x55 && (p)[1] == 0xAA)
#define mmMM_INDEX      0x0
// #define mmMM_INDEX_HI   0x6
#define mmMM_DATA       0x1
#define mmBIOS_SCRATCH_0 0x05C9

static bool atom_bios_validate(
    uint8_t *bios,
    size_t size
){
    uint16_t tmp, bios_header_start;

    if (!bios || size < 0x49) {
        LIGHTS_ERR("vbios mem is null or mem size is wrong");
        return false;
    }

    if (!AMD_IS_VALID_VBIOS(bios)) {
        LIGHTS_ERR("BIOS signature incorrect %x %x", bios[0], bios[1]);
        return false;
    }

    bios_header_start = bios[0x48] | (bios[0x49] << 8);
    if (!bios_header_start) {
        LIGHTS_ERR("Can't locate bios header");
        return false;
    }

    tmp = bios_header_start + 4;
    if (size < tmp) {
        LIGHTS_ERR("BIOS header is broken");
        return false;
    }

    if (!memcmp(bios + tmp, "ATOM", 4) || !memcmp(bios + tmp, "MOTA", 4)) {
        LIGHTS_ERR("ATOMBIOS detected");
        return true;
    }

    return false;
}

static bool atom_bios_read(
    struct gpu_device *gpu_dev
){
    uint8_t __iomem *bios;
    size_t size;

    gpu_dev->bios = NULL;
    /* XXX: some cards may return 0 for rom size? ddx has a workaround */
    bios = pci_map_rom(gpu_dev->pdev, &size);
    if (!bios) {
        return false;
    }

    gpu_dev->bios = kzalloc(size, GFP_KERNEL);
    if (gpu_dev->bios == NULL) {
        pci_unmap_rom(gpu_dev->pdev, bios);
        return false;
    }
    gpu_dev->bios_size = size;
    memcpy_fromio(gpu_dev->bios, bios, size);
    pci_unmap_rom(gpu_dev->pdev, bios);

    if (!atom_bios_validate(gpu_dev->bios, size)) {
        kfree(gpu_dev->bios);
        return false;
    }

    return true;
}

static bool gpu_device_read_bios(
    struct gpu_device *gpu_dev
){
    if (atom_bios_read(gpu_dev))
        goto success;

    LIGHTS_ERR("Unable to locate a BIOS ROM");

    return false;

success:
    gpu_dev->is_atom_fw = (gpu_dev->asic_type >= CHIP_VEGA10) ? true : false;

    return true;
}


static uint32_t amdgpu_mm_rreg(
    struct gpu_device *adev,
    uint32_t reg,
    uint32_t acc_flags
){
    uint32_t ret;

    // if (!(acc_flags & AMDGPU_REGS_NO_KIQ) && amdgpu_sriov_runtime(adev))
    //     return amdgpu_kiq_rreg(adev, reg);

    if ((reg * 4) < adev->rmmio_size)
        ret = readl(((void __iomem *)adev->rmmio) + (reg * 4));
    else {
        unsigned long flags;

        spin_lock_irqsave(&adev->mmio_idx_lock, flags);
        writel((reg * 4), ((void __iomem *)adev->rmmio) + (mmMM_INDEX * 4));
        ret = readl(((void __iomem *)adev->rmmio) + (mmMM_DATA * 4));
        spin_unlock_irqrestore(&adev->mmio_idx_lock, flags);
    }
    // trace_amdgpu_mm_rreg(adev->pdev->device, reg, ret);
    return ret;
}

static inline void amdgpu_mm_wreg_mmio(
    struct gpu_device *adev,
    uint32_t reg,
    uint32_t v,
    uint32_t acc_flags
){
    // trace_amdgpu_mm_wreg(adev->pdev->device, reg, v);

    if ((reg * 4) < adev->rmmio_size)
        writel(v, ((void __iomem *)adev->rmmio) + (reg * 4));
    else {
        unsigned long flags;

        spin_lock_irqsave(&adev->mmio_idx_lock, flags);
        writel((reg * 4), ((void __iomem *)adev->rmmio) + (mmMM_INDEX * 4));
        writel(v, ((void __iomem *)adev->rmmio) + (mmMM_DATA * 4));
        spin_unlock_irqrestore(&adev->mmio_idx_lock, flags);
    }
}

static void amdgpu_mm_wreg(
    struct gpu_device *adev,
    uint32_t reg,
    uint32_t v,
    uint32_t acc_flags
){
    // if (!(acc_flags & AMDGPU_REGS_NO_KIQ) && amdgpu_sriov_runtime(adev))
    //     return amdgpu_kiq_wreg(adev, reg, v);

    amdgpu_mm_wreg_mmio(adev, reg, v, acc_flags);
}

static uint32_t amdgpu_io_rreg(
    struct gpu_device *adev,
    uint32_t reg
){
    if ((reg * 4) < adev->rio_mem_size)
        return ioread32(adev->rio_mem + (reg * 4));
    else {
        iowrite32((reg * 4), adev->rio_mem + (mmMM_INDEX * 4));
        return ioread32(adev->rio_mem + (mmMM_DATA * 4));
    }
}

static void amdgpu_io_wreg(
    struct gpu_device *adev,
    uint32_t reg,
    uint32_t v
){
    if ((reg * 4) < adev->rio_mem_size)
        iowrite32(v, adev->rio_mem + (reg * 4));
    else {
        iowrite32((reg * 4), adev->rio_mem + (mmMM_INDEX * 4));
        iowrite32(v, adev->rio_mem + (mmMM_DATA * 4));
    }
}


static uint32_t cail_pll_read(
    struct card_info *info,
    uint32_t reg
){
    return 0;
}

static void cail_pll_write(
    struct card_info *info,
    uint32_t reg,
    uint32_t val
){

}

static uint32_t cail_mc_read(
    struct card_info *info,
    uint32_t reg
){
    return 0;
}

static void cail_mc_write(
    struct card_info *info,
    uint32_t reg,
    uint32_t val
){

}

static void cail_reg_write(
    struct card_info *info,
    uint32_t reg,
    uint32_t val
){
    // struct amdgpu_device *adev = info->dev->dev_private;

    // WREG32(reg, val);
    amdgpu_mm_wreg(info->gpu_dev, reg, val, 0);
}

static uint32_t cail_reg_read(
    struct card_info *info,
    uint32_t reg
){
    // struct amdgpu_device *adev = info->dev->dev_private;
    // uint32_t r;
    //
    // r = RREG32(reg);
    // return r;
    return amdgpu_mm_rreg(info->gpu_dev, reg, 0);
}

static void cail_ioreg_write(
    struct card_info *info,
    uint32_t reg,
    uint32_t val
){
    // struct amdgpu_device *adev = info->dev->dev_private;
    //
    // WREG32_IO(reg, val);
    amdgpu_io_wreg(info->gpu_dev, reg, val);
}

static uint32_t cail_ioreg_read(
    struct card_info *info,
    uint32_t reg
){
    // struct amdgpu_device *adev = info->dev->dev_private;
    // uint32_t r;
    //
    // r = RREG32_IO(reg);
    // return r;
    return amdgpu_io_rreg(info->gpu_dev, reg);
}

static void atom_bios_free(
    struct gpu_device *gpu_dev
){
    if (gpu_dev->atom_context) {
        kfree(gpu_dev->atom_context->scratch);
        kfree(gpu_dev->atom_context->iio);
    }
    kfree(gpu_dev->atom_context);
    gpu_dev->atom_context = NULL;
    kfree(gpu_dev->atom_card_info);
    gpu_dev->atom_card_info = NULL;
}


#define get_index_into_master_table(master_table, table_name) (offsetof(struct master_table, table_name) / sizeof(uint16_t))

// static void amdgpu_atomfirmware_scratch_regs_init(
//     struct gpu_device *gpu_dev
// ){
//     int index = get_index_into_master_table(
//         atom_master_list_of_data_tables_v2_1,
//         firmwareinfo
//     );
//     uint16_t data_offset;
//
//     if (amdgpu_atom_parse_data_header(gpu_dev->atom_context, index, NULL, NULL, NULL, &data_offset)) {
//         struct atom_firmware_info_v3_1 *firmware_info =
//             (struct atom_firmware_info_v3_1 *)(gpu_dev->atom_context->bios + data_offset);
//
//         gpu_dev->bios_scratch_reg_offset = le32_to_cpu(firmware_info->bios_scratch_reg_startaddr);
//     }
// }
//
// static int amdgpu_atomfirmware_allocate_fb_scratch(
//     struct gpu_device *gpu_dev
// ){
//     struct atom_context *ctx = gpu_dev->atom_context;
//     int index = get_index_into_master_table(
//         atom_master_list_of_data_tables_v2_1,
//         vram_usagebyfirmware
//     );
//     struct vram_usagebyfirmware_v2_1 *firmware_usage;
//     uint32_t start_addr, size;
//     uint16_t data_offset;
//     int usage_bytes = 0;
//
//     if (amdgpu_atom_parse_data_header(ctx, index, NULL, NULL, NULL, &data_offset)) {
//         firmware_usage = (struct vram_usagebyfirmware_v2_1 *)(ctx->bios + data_offset);
//         LIGHTS_DBG("atom firmware requested %08x %dkb fw %dkb drv",
//               le32_to_cpu(firmware_usage->start_address_in_kb),
//               le16_to_cpu(firmware_usage->used_by_firmware_in_kb),
//               le16_to_cpu(firmware_usage->used_by_driver_in_kb));
//
//         start_addr = le32_to_cpu(firmware_usage->start_address_in_kb);
//         size = le16_to_cpu(firmware_usage->used_by_firmware_in_kb);
//
//         // if ((uint32_t)(start_addr & ATOM_VRAM_OPERATION_FLAGS_MASK) ==
//         //     (uint32_t)(ATOM_VRAM_BLOCK_SRIOV_MSG_SHARE_RESERVATION <<
//         //     ATOM_VRAM_OPERATION_FLAGS_SHIFT)) {
//         //     /* Firmware request VRAM reservation for SR-IOV */
//         //     adev->fw_vram_usage.start_offset = (start_addr &
//         //         (~ATOM_VRAM_OPERATION_FLAGS_MASK)) << 10;
//         //     adev->fw_vram_usage.size = size << 10;
//         //     /* Use the default scratch size */
//         //     usage_bytes = 0;
//         // } else {
//             usage_bytes = le16_to_cpu(firmware_usage->used_by_driver_in_kb) << 10;
//         // }
//     }
//     ctx->scratch_size_bytes = 0;
//     if (usage_bytes == 0)
//         usage_bytes = 20 * 1024;
//     /* allocate some scratch memory */
//     ctx->scratch = kzalloc(usage_bytes, GFP_KERNEL);
//     if (!ctx->scratch)
//         return -ENOMEM;
//
//     ctx->scratch_size_bytes = usage_bytes;
//
//     return 0;
// }


// #define RREG32(reg) amdgpu_mm_rreg(adev, (reg), 0)
// #define WREG32(reg, v) amdgpu_mm_wreg(adev, (reg), (v), 0)

static void amdgpu_atombios_scratch_regs_init(
    struct gpu_device *gpu_dev
){
    uint32_t bios_2_scratch, bios_6_scratch;

    gpu_dev->bios_scratch_reg_offset = mmBIOS_SCRATCH_0;

    bios_2_scratch = amdgpu_mm_rreg(gpu_dev, gpu_dev->bios_scratch_reg_offset + 2, 0);
    bios_6_scratch = amdgpu_mm_rreg(gpu_dev, gpu_dev->bios_scratch_reg_offset + 6, 0);

    /* let the bios control the backlight */
    bios_2_scratch &= ~ATOM_S2_VRI_BRIGHT_ENABLE;

    /* tell the bios not to handle mode switching */
    bios_6_scratch |= ATOM_S6_ACC_BLOCK_DISPLAY_SWITCH;

    /* clear the vbios dpms state */
    bios_2_scratch &= ~ATOM_S2_DEVICE_DPMS_STATE;

    amdgpu_mm_wreg(gpu_dev, gpu_dev->bios_scratch_reg_offset + 2, bios_2_scratch, 0);
    amdgpu_mm_wreg(gpu_dev, gpu_dev->bios_scratch_reg_offset + 6, bios_6_scratch, 0);
}

static int amdgpu_atombios_allocate_fb_scratch(
    struct gpu_device *gpu_dev
){
    struct atom_context *ctx = gpu_dev->atom_context;
    int index = GetIndexIntoMasterTable(DATA, VRAM_UsageByFirmware);
    uint16_t data_offset;
    int usage_bytes = 0;
    struct _ATOM_VRAM_USAGE_BY_FIRMWARE *firmware_usage;
    u64 start_addr;
    u64 size;

    if (amdgpu_atom_parse_data_header(ctx, index, NULL, NULL, NULL, &data_offset)) {
        firmware_usage = (struct _ATOM_VRAM_USAGE_BY_FIRMWARE *)(ctx->bios + data_offset);

        LIGHTS_DBG("atom firmware requested %08x %dkb\n",
              le32_to_cpu(firmware_usage->asFirmwareVramReserveInfo[0].ulStartAddrUsedByFirmware),
              le16_to_cpu(firmware_usage->asFirmwareVramReserveInfo[0].usFirmwareUseInKb));

        start_addr = firmware_usage->asFirmwareVramReserveInfo[0].ulStartAddrUsedByFirmware;
        size = firmware_usage->asFirmwareVramReserveInfo[0].usFirmwareUseInKb;

        if ((uint32_t)(start_addr & ATOM_VRAM_OPERATION_FLAGS_MASK) ==
            (uint32_t)(ATOM_VRAM_BLOCK_SRIOV_MSG_SHARE_RESERVATION <<
            ATOM_VRAM_OPERATION_FLAGS_SHIFT)) {
            /* Firmware request VRAM reservation for SR-IOV */
            // adev->fw_vram_usage.start_offset = (start_addr &
            //     (~ATOM_VRAM_OPERATION_FLAGS_MASK)) << 10;
            // adev->fw_vram_usage.size = size << 10;
            /* Use the default scratch size */
            usage_bytes = 0;
        } else {
            usage_bytes = le16_to_cpu(firmware_usage->asFirmwareVramReserveInfo[0].usFirmwareUseInKb) * 1024;
        }
    }
    ctx->scratch_size_bytes = 0;
    if (usage_bytes == 0)
        usage_bytes = 20 * 1024;
    /* allocate some scratch memory */
    ctx->scratch = kzalloc(usage_bytes, GFP_KERNEL);
    if (!ctx->scratch)
        return -ENOMEM;

    ctx->scratch_size_bytes = usage_bytes;

    return 0;
}

static int atom_bios_init(
    struct gpu_device *gpu_dev
){
    struct card_info *atom_card_info = kzalloc(sizeof(*atom_card_info), GFP_KERNEL);

    if (!atom_card_info)
        return -ENOMEM;

    gpu_dev->atom_card_info = atom_card_info;
    // atom_card_info->dev = gpu_dev->ddev;
    atom_card_info->gpu_dev = gpu_dev;
    atom_card_info->reg_read = cail_reg_read;
    atom_card_info->reg_write = cail_reg_write;
    /* needed for iio ops */
    if (gpu_dev->rio_mem) {
        atom_card_info->ioreg_read = cail_ioreg_read;
        atom_card_info->ioreg_write = cail_ioreg_write;
    } else {
        LIGHTS_DBG("PCI I/O BAR is not found. Using MMIO to access ATOM BIOS");
        atom_card_info->ioreg_read = cail_reg_read;
        atom_card_info->ioreg_write = cail_reg_write;
    }
    atom_card_info->mc_read = cail_mc_read;
    atom_card_info->mc_write = cail_mc_write;
    atom_card_info->pll_read = cail_pll_read;
    atom_card_info->pll_write = cail_pll_write;

    gpu_dev->atom_context = amdgpu_atom_parse(atom_card_info, gpu_dev->bios);
    if (!gpu_dev->atom_context) {
        atom_bios_free(gpu_dev);
        return -ENOMEM;
    }

    mutex_init(&gpu_dev->atom_context->mutex);

    // if (gpu_dev->is_atom_fw) {
    //     amdgpu_atomfirmware_scratch_regs_init(gpu_dev);
    //     amdgpu_atomfirmware_allocate_fb_scratch(gpu_dev);
    // } else {
        amdgpu_atombios_scratch_regs_init(gpu_dev);
        amdgpu_atombios_allocate_fb_scratch(gpu_dev);
    // }

    return 0;
}


struct gpu_device *gpu_device_create (
    struct pci_dev *pci_dev,
    unsigned long asic_type
){
    struct gpu_device *gpu_dev = kzalloc(sizeof(*gpu_dev), GFP_KERNEL);
    error_t err;
    int i;

    if (IS_ERR_OR_NULL(gpu_dev))
        return ERR_PTR(-ENOMEM);

    spin_lock_init(&gpu_dev->mmio_idx_lock);

    gpu_dev->pdev = pci_dev;
    gpu_dev->asic_type = asic_type;
    gpu_dev->rmmio_base = pci_resource_start(pci_dev, 5);
    gpu_dev->rmmio_size = pci_resource_len(pci_dev, 5);
    gpu_dev->rmmio = ioremap(gpu_dev->rmmio_base, gpu_dev->rmmio_size);

    if (gpu_dev->rmmio == NULL) {
        err = -ENOMEM;
        goto error;
    }

    LIGHTS_INFO("register mmio base: 0x%08X\n", (uint32_t)gpu_dev->rmmio_base);
    LIGHTS_INFO("register mmio size: %u\n", (unsigned)gpu_dev->rmmio_size);

    /* io port mapping */
    for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
        if (pci_resource_flags(pci_dev, i) & IORESOURCE_IO) {
            gpu_dev->rio_mem_size = pci_resource_len(pci_dev, i);
            gpu_dev->rio_mem = pci_iomap(pci_dev, i, gpu_dev->rio_mem_size);
            break;
        }
    }

    if (gpu_dev->rio_mem == NULL)
        LIGHTS_INFO("PCI I/O BAR is not found.\n");

    if (!gpu_device_read_bios(gpu_dev))
        goto error;

    err = atom_bios_init(gpu_dev);
    if (err) {
        LIGHTS_ERR("amdgpu_atombios_init failed");
        goto error;
    }

    if (gpu_dev->is_atom_fw) {
        /* Initialize clocks */
        // err = amdgpu_atomfirmware_get_clock_info(gpu_dev);
        // if (err) {
        //     LIGHTS_ERR("amdgpu_atomfirmware_get_clock_info failed");
        //     goto error;
        // }
    } else {
        /* Initialize clocks */
        // err = amdgpu_atombios_get_clock_info(gpu_dev);
        // if (err) {
        //     LIGHTS_ERR("amdgpu_atombios_get_clock_info failed");
        //     goto error;
        // }
        /* init i2c buses */

        // amdgpu_atombios_i2c_init(gpu_dev);
    }

    return 0;

error:
    gpu_device_destroy(gpu_dev);

    return ERR_PTR(err);
}

void gpu_device_destroy (
    struct gpu_device *gpu_dev
){
    kfree(gpu_dev);
}
