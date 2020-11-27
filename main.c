// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/pci.h>

#include "debug.h"
#include "pci_ids.h"
#include "device.h"

static struct gpu_device *gpu_dev = NULL;

static int find_pci_dev (
    void
){
    struct pci_dev *pci_dev = NULL;
    const struct pci_device_id *match;

    while (NULL != (pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_dev))) {
        match = pci_match_id(pciidlist, pci_dev);
        if (match) {
            gpu_dev = gpu_device_create(pci_dev, match->driver_data);
            if (IS_ERR_OR_NULL(gpu_dev)) {
                gpu_dev = NULL;
                LIGHTS_ERR("Failed to create the gpu device");
                return -ENODEV;
            }

            return 0;
        }
    }

    return -ENODEV;
}

static int __init aura_module_init (
    void
){
    int err = find_pci_dev();

    if (err) {
        LIGHTS_ERR("Failed to find a valid pci device");
        return 0;
    }

    atombios_i2c_init(gpu_dev);

    return 0;
}

static void __exit aura_module_exit (
    void
){
    if (gpu_dev){
        atombios_i2c_fini(gpu_dev);
        gpu_device_destroy(gpu_dev);
    }
}

module_init(aura_module_init);
module_exit(aura_module_exit);


MODULE_AUTHOR("Owen Parry <waldermort@gmail.com>");
MODULE_DESCRIPTION("ASUS AURA SMBus driver");
MODULE_LICENSE("GPL");
