/*
 * Xenium Modchip - https://github.com/Ryzee119/OpenXenium
 *
 * Copyright (c) 2021 Mike Davis
 * Copyright (c) 2021 Ryzee119
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/option.h"
#include "qemu/datadir.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/loader.h"
#include "hw/char/serial.h"
#include "hw/isa/isa.h"
#include "qapi/error.h"

#define XENIUM_REGISTER_BASE 0xEE
#define XENIUM_REGISTER0 0
#define XENIUM_REGISTER1 1

#define DEBUG
#ifdef DEBUG
# define DPRINTF(format, ...) printf(format, ## __VA_ARGS__)
#else
# define DPRINTF(format, ...) do { } while (0)
#endif

#define XENIUM_FLASH_SIZE (2 * 1024 * 1024)
#define XENIUM_MAX_BANK_SIZE (1024 * 1024)
#define MCPX_SIZE (512)

extern MemoryRegion *rom_memory__;
uint8_t xenium_raw[XENIUM_FLASH_SIZE];
uint8_t mcpx_raw[MCPX_SIZE];

typedef struct XeniumBank {
    unsigned int offset;
    unsigned int size;
} XeniumBank_t;

static const XeniumBank_t XeniumBank[11] = 
{
    {0, 1 * 1024 * 1024},   //TSOP
    {0x180000, 256 * 1024}, //Bootloader
    {0x100000, 512 * 1024}, //XeniumOS
    {0x000000, 256 * 1024}, //Bank 1 256k
    {0x040000, 256 * 1024}, //Bank 2 256k
    {0x080000, 256 * 1024}, //Bank 3 256k
    {0x0C0000, 256 * 1024}, //Bank 4 256k
    {0x000000, 512 * 1024}, //Bank 1 512k
    {0x080000, 512 * 1024}, //Bank 2 512k
    {0x000000, 1024 * 1024}, //Bank 1 1M
    {0x1C0000, 256 * 1024}, //Recovery + More XeniumOS Data + User settings
};

// Dumped using this script https://gist.github.com/LoveMHz/8c20b0bb7fcd88588a1740657396075c
uint8_t XeniumFlashCFI[] = {
    /* 00h */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 10h */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 20h */ 0x51, 0x51, 0x52, 0x52, 0x59, 0x59, 0x02, 0x02, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
    /* 30h */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x27, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03,
    /* 40h */ 0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x15, 0x15,
    /* 50h */ 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40,
    /* 60h */ 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
    /* 70h */ 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* 80h */ 0x50, 0x50, 0x52, 0x52, 0x49, 0x49, 0x31, 0x31, 0x33, 0x33, 0x0C, 0x0C, 0x02, 0x02, 0x01, 0x01,
    /* 90h */ 0x01, 0x01, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03,
    /* A0h */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* B0h */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06,
    /* C0h */ 0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x15, 0x15,
    /* D0h */ 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40,
    /* E0h */ 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
    /* F0h */ 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

typedef enum {
    XENIUM_MEMORY_STATE_NORMAL,
    XENIUM_MEMORY_STATE_CFI
} XeniumMemoryState;

typedef struct XeniumState {
    ISADevice dev;
    SysBusDevice dev_sysbus;
    MemoryRegion io;
    MemoryRegion flash_mem;

    // SPI
    bool sck;
    bool cs;
    bool mosi;
    bool miso_1;    // pin 1
    bool miso_4;    // pin 4

    unsigned char led;              // XXXXXBGR
    unsigned short bank_control;    // determines flash address mask

    bool recovery;  // 0 is active

    XeniumMemoryState flash_state;
} XeniumState;

#define XENIUM_DEVICE(obj) \
    OBJECT_CHECK(XeniumState, (obj), "modchip-xenium")

static void xenium_io_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned int size)
{
    XeniumState *s = opaque;

    DPRINTF("%s: Write 0x%llX to IO register 0x%llX\n",
        __func__, val, XENIUM_REGISTER_BASE + addr);

    switch(addr) {
        case XENIUM_REGISTER0:
            assert((val >> 3) == 0);    // un-known/used
            s->led = val;
            DPRINTF("%s: Set LED color(s) to %d\n", __func__, s->led);
        break;
        case XENIUM_REGISTER1:
            assert((val & (1 << 7)) == 0);    // un-known/used
            s->sck = val & (1 << 6);
            s->cs = val & (1 << 5);
            s->mosi = val & (1 << 4);
            s->bank_control = val & 0xF;
            unsigned int flash_size = XeniumBank[s->bank_control].size;
            DPRINTF("%s: Set Bank to %d, Offset: %08x, Size: %d bytes\n", __func__, s->bank_control,
                                                                          XeniumBank[s->bank_control].offset,
                                                                          flash_size);

            //We're on a new bank, so copy the flash data from this bank into our ROM device.
            void *flash_mem = memory_region_get_ram_ptr(&s->flash_mem);
            for (int i = 0; i < XENIUM_MAX_BANK_SIZE; i += flash_size)
            {
                memcpy(flash_mem + i, xenium_raw + XeniumBank[s->bank_control].offset, flash_size);
            }
            memory_region_flush_rom_device(&s->flash_mem, 0, XENIUM_MAX_BANK_SIZE);
        break;
        default: assert(false);
    }
}

static uint64_t xenium_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    XeniumState *s = opaque;
    uint32_t val = 0;

    switch(addr) {
        case XENIUM_REGISTER0:
            val = 0x55;     // genuine xenium!
        break;
        case XENIUM_REGISTER1:
            val = (s->recovery << 7) |
                (s->miso_1 << 5) |
                (s->miso_4 << 4) |
                s->bank_control;
        break;
        default: assert(false);
    }

    DPRINTF("%s: Read 0x%X from IO register 0x%llX\n",
        __func__, val, XENIUM_REGISTER_BASE + addr);

    return val;
}

static const MemoryRegionOps xenium_io_ops = {
    .read  = xenium_io_read,
    .write = xenium_io_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static uint64_t flash_read(void *opaque, hwaddr offset, unsigned size)
{
    //FIXME: If flash_write has had the write sequences, we should be in MMIO mode. So this callback should get called
    //for flash reads.
    //Handle flash ID (Manuf and chip ID)
    //Busy flags?
    //When finished put back in ROMD mode memory_region_rom_device_set_romd(&s->flash_mem, true);
    XeniumState *s = opaque;
    DPRINTF("%s offset: %08x size: %d\n", __FUNCTION__, (uint32_t)offset, size);

    if(s->flash_state == XENIUM_MEMORY_STATE_CFI) {
        return XeniumFlashCFI[(size == 1 ? offset : offset << 1) % sizeof(XeniumFlashCFI)];
    }

    return 0;
}

static void flash_write(void *opaque, hwaddr offset, uint64_t value,
        unsigned int size)
{
    XeniumState *s = opaque;

    //FIXME: I think here we check all the flash write commands, if they match the right sequence we need to disable ROMD mode (MMIO mode),
    //and then handle flash responses in the flash_read callback (flash busy, flash ID etc)
    //1: memory_region_rom_device_set_romd(&s->flash_mem, false);
    //2: Handle chip erase, sector erase etc.
    //3: Handle responses in flash_read callback
    DPRINTF("%s offset: %08x value: %02x size: %d\n", __FUNCTION__, (uint32_t)offset, (uint8_t)value, size);

    // Reset
    if(offset == 0x00 && value == 0xF0 && size == 1) {
        DPRINTF("%s Flash Reset (Entering Normal flash state)\n", __FUNCTION__);

        s->flash_state = XENIUM_MEMORY_STATE_NORMAL;
        memory_region_rom_device_set_romd(&s->flash_mem, true);
    }
    // Enter CFI Mode
    else if(offset == 0xAA && value == 0x98 && size == 1) {
        DPRINTF("%s Entering CFI Mode flash state\n", __FUNCTION__);

        s->flash_state = XENIUM_MEMORY_STATE_CFI;
        memory_region_rom_device_set_romd(&s->flash_mem, false);
    }
}

static const MemoryRegionOps xenium_flash_ops = {
    .read = flash_read,
    .write = flash_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void xenium_realize(DeviceState *dev, Error **errp)
{
    XeniumState *s = XENIUM_DEVICE(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    Error *err = NULL;

    //Read Xenium Flash Dump (2MB file)
    int fd = qemu_open("D:/xenium_flash.bin", O_RDONLY | O_BINARY, NULL);
    assert(fd >= 0);
    read(fd, xenium_raw, XENIUM_FLASH_SIZE);
    close(fd);

    //Read MCPX Dump (512 bytes)
    const char *bootrom_file =
        object_property_get_str(qdev_get_machine(), "bootrom", NULL);

    if ((bootrom_file != NULL) && *bootrom_file) {
        char *filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bootrom_file);
        assert(filename);

        /* Read in MCPX ROM over last 512 bytes of BIOS data */
        int fd = qemu_open(filename, O_RDONLY | O_BINARY, NULL);
        assert(fd >= 0);
        read(fd, mcpx_raw, MCPX_SIZE);
        close(fd);
        g_free(filename);
    }

    // default state
    s->bank_control = 1;    // bootloader
    s->recovery = 1;        // inactive
    s->led = 1;             // red
    s->flash_state = XENIUM_MEMORY_STATE_NORMAL; // Default flash state

    //Create a ROM device for the Xenuim Flash
    unsigned int flash_size = XeniumBank[s->bank_control].size;

    memory_region_init_rom_device(&s->flash_mem, OBJECT(s), &xenium_flash_ops, s,
        "xenium.bios", XENIUM_MAX_BANK_SIZE, &err);

    void *flash_mem = memory_region_get_ram_ptr(&s->flash_mem);

    //Mirror the flash data over 1MB. This wastes some memory but I always use 1MB as this is the lrgest bank Xenium supports, so I dont have to
    //keep changing the memory aliases each time the bank size changes.
    for (int i = 0; i < XENIUM_MAX_BANK_SIZE; i += flash_size)
    {
        memcpy(flash_mem + i, xenium_raw + XeniumBank[s->bank_control].offset, flash_size);
    }

    //Setup memory aliases to mirror the 1MB ROM over the entire Flash ROM mapped region. (0xFF000000 to 0xFFFFFFFF)
    const uint32_t rom_start = 0xFF000000;
    uint32_t map_loc;
    for (map_loc = rom_start; map_loc >= rom_start; map_loc += XENIUM_MAX_BANK_SIZE) {
        MemoryRegion *mr_bios = g_malloc(sizeof(MemoryRegion));
        assert(mr_bios != NULL);
        memory_region_init_alias(mr_bios, NULL, "xenium.bios.alias", &s->flash_mem, 0, XENIUM_MAX_BANK_SIZE);
        memory_region_add_subregion(rom_memory__, map_loc, mr_bios);
    }

    //Add MCPX memory and alias it to 0xFFFFFE00 in Xbox memory
    MemoryRegion *mr_mcpx = g_malloc(sizeof(MemoryRegion));
    memory_region_init_ram(mr_mcpx, NULL, "xbox.mcpx", flash_size, &err);
    void *mcpx_data = memory_region_get_ram_ptr(mr_mcpx);
    memcpy(mcpx_data, flash_mem, flash_size);
    memcpy(mcpx_data + flash_size - MCPX_SIZE, mcpx_raw, MCPX_SIZE);
    MemoryRegion *mr_mcpx_alias = g_malloc(sizeof(MemoryRegion));
    memory_region_init_alias(mr_mcpx_alias, NULL, "xbox.mcpx.alias", mr_mcpx, 0, flash_size);
    memory_region_add_subregion(rom_memory__, -flash_size, mr_mcpx_alias);

    //Register Xenium Chip IO
    memory_region_init_io(&s->io, OBJECT(s), &xenium_io_ops, s, "xenium.io", 2);   // 0xEE & 0xEF
    isa_register_ioport(isa, &s->io, XENIUM_REGISTER_BASE);
}

static Property xenium_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_xenium = {
    .name = "modchip-xenium",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void xenium_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xenium_realize;
    dc->vmsd = &vmstate_xenium;
    device_class_set_props(dc, xenium_properties);
}

static void xenium_initfn(Object *o)
{
    XeniumState *self = XENIUM_DEVICE(o);
}

static const TypeInfo xenium_type_info = {
    .name          = "modchip-xenium",
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(XeniumState),
    .instance_init = xenium_initfn,
    .class_init    = xenium_class_init,
};

static void xenium_register_types(void)
{
    type_register_static(&xenium_type_info);
}

type_init(xenium_register_types)
