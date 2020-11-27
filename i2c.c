/* SPDX-License-Identifier: GPL-2.0 */

#include "debug.h"
#include "i2c.h"
#include "atom/atom.h"
#include "atom/atombios.h"

#define TARGET_HW_I2C_CLOCK 50

/* these are a limitation of ProcessI2cChannelTransaction not the hw */
#define ATOM_MAX_HW_I2C_WRITE 3
#define ATOM_MAX_HW_I2C_READ  255


static void amdgpu_atombios_copy_swap(
    u8 *dst,
    u8 *src,
    u8 num_bytes,
    bool to_le
){
#ifdef __BIG_ENDIAN
    u32 src_tmp[5], dst_tmp[5];
    int i;
    u8 align_num_bytes = ALIGN(num_bytes, 4);

    if (to_le) {
        memcpy(src_tmp, src, num_bytes);
        for (i = 0; i < align_num_bytes / 4; i++)
            dst_tmp[i] = cpu_to_le32(src_tmp[i]);
        memcpy(dst, dst_tmp, align_num_bytes);
    } else {
        memcpy(src_tmp, src, align_num_bytes);
        for (i = 0; i < align_num_bytes / 4; i++)
            dst_tmp[i] = le32_to_cpu(src_tmp[i]);
        memcpy(dst, dst_tmp, num_bytes);
    }
#else
    memcpy(dst, src, num_bytes);
#endif
}

static int amdgpu_atombios_i2c_process_i2c_ch(
    struct i2c_chan *chan,
    u8 slave_addr,
    u8 flags,
    u8 *buf,
    u8 num
){
    // struct drm_device *dev = chan->dev;
    // struct amdgpu_device *adev = dev->dev_private;
    struct gpu_device *gpu_dev = chan->gpu_dev;
    PROCESS_I2C_CHANNEL_TRANSACTION_PS_ALLOCATION args;
    int index = GetIndexIntoMasterTable(COMMAND, ProcessI2cChannelTransaction);
    unsigned char *base;
    u16 out = cpu_to_le16(0);
    int r = 0;

    memset(&args, 0, sizeof(args));

    mutex_lock(&chan->mutex);

    base = (unsigned char *)gpu_dev->atom_context->scratch;

    if (flags & HW_I2C_WRITE) {
        if (num > ATOM_MAX_HW_I2C_WRITE) {
            LIGHTS_ERR("hw i2c: tried to write too many bytes (%d vs 3)", num);
            r = -EINVAL;
            goto done;
        }
        if (buf == NULL)
            args.ucRegIndex = 0;
        else
            args.ucRegIndex = buf[0];
        if (num)
            num--;
        if (num) {
            if (buf) {
                memcpy(&out, &buf[1], num);
            } else {
                LIGHTS_ERR("hw i2c: missing buf with num > 1");
                r = -EINVAL;
                goto done;
            }
        }
        args.lpI2CDataOut = cpu_to_le16(out);
    } else {
        args.ucRegIndex = 0;
        args.lpI2CDataOut = 0;
    }

    args.ucFlag = flags;
    args.ucI2CSpeed = TARGET_HW_I2C_CLOCK;
    args.ucTransBytes = num;
    args.ucSlaveAddr = slave_addr << 1;
    args.ucLineNumber = chan->rec.i2c_id;

    amdgpu_atom_execute_table(gpu_dev->atom_context, index, (uint32_t *)&args);

    /* error */
    if (args.ucStatus != HW_ASSISTED_I2C_STATUS_SUCCESS) {
        LIGHTS_ERR("hw_i2c error");
        r = -EIO;
        goto done;
    }

    if (!(flags & HW_I2C_WRITE))
        amdgpu_atombios_copy_swap(buf, base, num, false);

done:
    mutex_unlock(&chan->mutex);

    return r;
}

static int amdgpu_atombios_i2c_xfer(
    struct i2c_adapter *i2c_adap,
    struct i2c_msg *msgs,
    int num
){
    struct i2c_chan *i2c = i2c_get_adapdata(i2c_adap);
    struct i2c_msg *p;
    int i, remaining, current_count, buffer_offset, max_bytes, ret;
    u8 flags;

    /* check for bus probe */
    p = &msgs[0];
    if ((num == 1) && (p->len == 0)) {
        ret = amdgpu_atombios_i2c_process_i2c_ch(i2c, p->addr, HW_I2C_WRITE, NULL, 0);
        if (ret)
            return ret;
        else
            return num;
    }

    for (i = 0; i < num; i++) {
        p = &msgs[i];
        remaining = p->len;
        buffer_offset = 0;
        /* max_bytes are a limitation of ProcessI2cChannelTransaction not the hw */
        if (p->flags & I2C_M_RD) {
            max_bytes = ATOM_MAX_HW_I2C_READ;
            flags = HW_I2C_READ;
        } else {
            max_bytes = ATOM_MAX_HW_I2C_WRITE;
            flags = HW_I2C_WRITE;
        }
        while (remaining) {
            if (remaining > max_bytes)
                current_count = max_bytes;
            else
                current_count = remaining;
            ret = amdgpu_atombios_i2c_process_i2c_ch(i2c, p->addr, flags, &p->buf[buffer_offset], current_count);
            if (ret)
                return ret;
            remaining -= current_count;
            buffer_offset += current_count;
        }
    }

    return num;
}

static u32 amdgpu_atombios_i2c_func(
    struct i2c_adapter *adap
){
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm amdgpu_atombios_i2c_algo = {
    .master_xfer = amdgpu_atombios_i2c_xfer,
    .functionality = amdgpu_atombios_i2c_func,
};

struct i2c_chan *amdgpu_i2c_create(
    // struct drm_device *dev,
    struct gpu_device *gpu_dev,
    const struct i2c_bus_rec *rec,
    const char *name
){
    struct i2c_chan *i2c;
    int ret;

    /* don't add the mm_i2c bus unless hw_i2c is enabled */
    // if (rec->mm_i2c && (amdgpu_hw_i2c == 0))
    //     return NULL;

    i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
    if (i2c == NULL)
        return NULL;

    i2c->gpu_dev = gpu_dev;
    i2c->rec = *rec;
    i2c->adapter.owner = THIS_MODULE;
    i2c->adapter.class = I2C_CLASS_DDC;
    // i2c->adapter.dev.parent = &dev->pdev->dev;
    // i2c->dev = dev;

    i2c_set_adapdata(&i2c->adapter, i2c);
    mutex_init(&i2c->mutex);

    snprintf(i2c->adapter.name, sizeof(i2c->adapter.name), "AMD LIGHTS i2c hw bus %s", name);
    i2c->adapter.algo = &amdgpu_atombios_i2c_algo;
    ret = i2c_add_adapter(&i2c->adapter);
    if (ret)
        goto out_free;

    // if (rec->hw_capable &&
    //     amdgpu_hw_i2c) {
    //     /* hw i2c using atom */
    //     snprintf(i2c->adapter.name, sizeof(i2c->adapter.name),
    //          "AMDGPU i2c hw bus %s", name);
    //     i2c->adapter.algo = &amdgpu_atombios_i2c_algo;
    //     ret = i2c_add_adapter(&i2c->adapter);
    //     if (ret)
    //         goto out_free;
    // } else {
    //     /* set the amdgpu bit adapter */
    //     snprintf(i2c->adapter.name, sizeof(i2c->adapter.name),
    //          "AMDGPU i2c bit bus %s", name);
    //     i2c->adapter.algo_data = &i2c->bit;
    //     i2c->bit.pre_xfer = amdgpu_i2c_pre_xfer;
    //     i2c->bit.post_xfer = amdgpu_i2c_post_xfer;
    //     i2c->bit.setsda = amdgpu_i2c_set_data;
    //     i2c->bit.setscl = amdgpu_i2c_set_clock;
    //     i2c->bit.getsda = amdgpu_i2c_get_data;
    //     i2c->bit.getscl = amdgpu_i2c_get_clock;
    //     i2c->bit.udelay = 10;
    //     i2c->bit.timeout = usecs_to_jiffies(2200);    /* from VESA */
    //     i2c->bit.data = i2c;
    //     ret = i2c_bit_add_bus(&i2c->adapter);
    //     if (ret) {
    //         DRM_ERROR("Failed to register bit i2c %s\n", name);
    //         goto out_free;
    //     }
    // }

    return i2c;

out_free:
    kfree(i2c);
    return NULL;
}

static void amdgpu_i2c_destroy(
    struct i2c_chan *i2c
){
    if (!i2c)
        return;

    // WARN_ON(i2c->has_aux);
    i2c_del_adapter(&i2c->adapter);
    kfree(i2c);
}

static void amdgpu_atombios_lookup_i2c_gpio_quirks(
    struct gpu_device *gpu_dev,
    ATOM_GPIO_I2C_ASSIGMENT *gpio,
    u8 index
){

}

static struct i2c_bus_rec amdgpu_atombios_get_bus_rec_for_i2c_gpio(
    ATOM_GPIO_I2C_ASSIGMENT *gpio
){
    struct i2c_bus_rec i2c;

    memset(&i2c, 0, sizeof(i2c));

    i2c.mask_clk_reg = le16_to_cpu(gpio->usClkMaskRegisterIndex);
    i2c.mask_data_reg = le16_to_cpu(gpio->usDataMaskRegisterIndex);
    i2c.en_clk_reg = le16_to_cpu(gpio->usClkEnRegisterIndex);
    i2c.en_data_reg = le16_to_cpu(gpio->usDataEnRegisterIndex);
    i2c.y_clk_reg = le16_to_cpu(gpio->usClkY_RegisterIndex);
    i2c.y_data_reg = le16_to_cpu(gpio->usDataY_RegisterIndex);
    i2c.a_clk_reg = le16_to_cpu(gpio->usClkA_RegisterIndex);
    i2c.a_data_reg = le16_to_cpu(gpio->usDataA_RegisterIndex);
    i2c.mask_clk_mask = (1 << gpio->ucClkMaskShift);
    i2c.mask_data_mask = (1 << gpio->ucDataMaskShift);
    i2c.en_clk_mask = (1 << gpio->ucClkEnShift);
    i2c.en_data_mask = (1 << gpio->ucDataEnShift);
    i2c.y_clk_mask = (1 << gpio->ucClkY_Shift);
    i2c.y_data_mask = (1 << gpio->ucDataY_Shift);
    i2c.a_clk_mask = (1 << gpio->ucClkA_Shift);
    i2c.a_data_mask = (1 << gpio->ucDataA_Shift);

    if (gpio->sucI2cId.sbfAccess.bfHW_Capable)
        i2c.hw_capable = true;
    else
        i2c.hw_capable = false;

    if (gpio->sucI2cId.ucAccess == 0xa0)
        i2c.mm_i2c = true;
    else
        i2c.mm_i2c = false;

    i2c.i2c_id = gpio->sucI2cId.ucAccess;

    if (i2c.mask_clk_reg)
        i2c.valid = true;
    else
        i2c.valid = false;

    return i2c;
}

void atombios_i2c_init(
    struct gpu_device *gpu_dev
){
    struct atom_context *ctx = gpu_dev->atom_context;
    ATOM_GPIO_I2C_ASSIGMENT *gpio;
    struct i2c_bus_rec i2c;
    int index = GetIndexIntoMasterTable(DATA, GPIO_I2C_Info);
    struct _ATOM_GPIO_I2C_INFO *i2c_info;
    uint16_t data_offset, size;
    int i, num_indices;
    char stmp[32];

    if (amdgpu_atom_parse_data_header(ctx, index, &size, NULL, NULL, &data_offset)) {
        i2c_info = (struct _ATOM_GPIO_I2C_INFO *)(ctx->bios + data_offset);
        num_indices = (size - sizeof(ATOM_COMMON_TABLE_HEADER)) / sizeof(ATOM_GPIO_I2C_ASSIGMENT);
        gpio = &i2c_info->asGPIO_Info[0];

        for (i = 0; i < num_indices; i++) {
            amdgpu_atombios_lookup_i2c_gpio_quirks(gpu_dev, gpio, i);

            i2c = amdgpu_atombios_get_bus_rec_for_i2c_gpio(gpio);

            if (i2c.valid) {
                    // for (i = 0; i < table_count; i++) {
                    //     pin = &header->asGPIO_Info[i];
                    //
                    //     AURA_DBG("line: %d, hw: %s, eng: %d",
                    //         header->asGPIO_Info[i].sucI2cId.sbfAccess.bfI2C_LineMux,
                    //         header->asGPIO_Info[i].sucI2cId.sbfAccess.bfHW_Capable ? "true" : "false",
                    //         header->asGPIO_Info[i].sucI2cId.sbfAccess.bfHW_EngineID
                    //     );
                    //     AURA_DBG("    clk_mask %x, clk_en %x, clk_y %x, clk_a %x, data_mask %x, data_en %x, data_y %x, data_a %x",
                    //         le16_to_cpu(pin->usClkMaskRegisterIndex),
                    //         le16_to_cpu(pin->usClkEnRegisterIndex),
                    //         le16_to_cpu(pin->usClkY_RegisterIndex),
                    //         le16_to_cpu(pin->usClkA_RegisterIndex),
                    //         le16_to_cpu(pin->usDataMaskRegisterIndex),
                    //         le16_to_cpu(pin->usDataEnRegisterIndex),
                    //         le16_to_cpu(pin->usDataY_RegisterIndex),
                    //         le16_to_cpu(pin->usDataA_RegisterIndex)
                    //     );
                    // }

                sprintf(stmp, "0x%x", i2c.i2c_id);
                gpu_dev->i2c_bus[i] = amdgpu_i2c_create(gpu_dev, &i2c, stmp);
            }

            gpio = (ATOM_GPIO_I2C_ASSIGMENT *)((u8 *)gpio + sizeof(ATOM_GPIO_I2C_ASSIGMENT));
        }
    }
}

void atombios_i2c_fini(
    struct gpu_device *gpu_dev
){
    int i;

    for (i = 0; i < AMDGPU_MAX_I2C_BUS; i++) {
        if (gpu_dev->i2c_bus[i]) {
            amdgpu_i2c_destroy(gpu_dev->i2c_bus[i]);
            gpu_dev->i2c_bus[i] = NULL;
        }
    }
}
