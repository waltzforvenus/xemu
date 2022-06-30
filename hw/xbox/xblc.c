/*
 * QEMU USB Xbox Live Communicator (XBLC) Device
 *
 * Copyright (c) 2022 Ryan Wendland
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
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "ui/console.h"
#include "hw/usb.h"
#include "hw/usb/desc.h"
#include "ui/xemu-input.h"
#include "audio/audio.h"
#include "qemu/fifo8.h"

#define DEBUG_XBLC
#ifdef DEBUG_XBLC
#define DPRINTF printf
#else
#define DPRINTF(...)
#endif

#define TYPE_USB_XBLC "usb-xblc"
#define USB_XBLC(obj) OBJECT_CHECK(USBXBLCState, (obj), TYPE_USB_XBLC)

#define XBLC_INTERFACE_CLASS    0x78
#define XBLC_INTERFACE_SUBCLASS 0x00
#define XBLC_EP_OUT             0x04
#define XBLC_EP_IN              0x05

#define XBLC_SET_SAMPLE_RATE    0x00
#define XBLC_SET_AGC            0x01

#define XBLC_FIFO_SIZE (48 * 1000) //~1 second worth of audio at 24kHz

typedef struct {
    uint16_t sample_rate;
    uint16_t bytes_per_frame;
} xblc_sample_rates_t;

static const xblc_sample_rates_t xblc_sample_rates[5] = {
    {8000,  (8000 * 2 / 1000)},
    {11025, (11025 * 2 / 1000)},
    {16000, (16000 * 2 / 1000)},
    {22050, (22050 * 2 / 1000)},
    {24000, (24000 * 2 / 1000)},
};

typedef struct USBXBLCState {
    USBDevice dev;
    uint8_t   device_index;
    uint8_t   auto_gain_control;
    uint16_t  sample_rate;
    QEMUSoundCard card;

    struct {
        struct audsettings as;
        Volume vol;
        SWVoiceOut* voice;
        uint8_t packet[48];
        Fifo8 fifo;
    } out;

    struct {
        struct audsettings as;
        Volume vol;
        SWVoiceIn *voice;
        uint8_t packet[48];
        Fifo8 fifo;
    } in;

    SWVoiceIn     *microphone;
} USBXBLCState;

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings desc_strings = {
    [STR_MANUFACTURER] = "Xemu",
    [STR_PRODUCT]      = "Microsoft Xbox Live Communicator",
    [STR_SERIALNUMBER] = "1",
};

static const USBDescIface desc_iface[]= {
    {
        .bInterfaceNumber              = 0,
        .bNumEndpoints                 = 1,
        .bInterfaceClass               = XBLC_INTERFACE_CLASS,
        .bInterfaceSubClass            = XBLC_INTERFACE_SUBCLASS,
        .bInterfaceProtocol            = 0x00,
        .eps = (USBDescEndpoint[]) {
            {
                .bEndpointAddress      = USB_DIR_OUT | XBLC_EP_OUT,
                .bmAttributes          = USB_ENDPOINT_XFER_ISOC,
                .wMaxPacketSize        = 48,
                .is_audio              = 1,
                .bInterval             = 1,
                .bRefresh              = 0,
                .bSynchAddress         = 0,
            }
        },
    },
    {
        .bInterfaceNumber              = 1,
        .bNumEndpoints                 = 1,
        .bInterfaceClass               = XBLC_INTERFACE_CLASS,
        .bInterfaceSubClass            = XBLC_INTERFACE_SUBCLASS,
        .bInterfaceProtocol            = 0x00,
        .eps = (USBDescEndpoint[]) {
            {
                .bEndpointAddress      = USB_DIR_IN | XBLC_EP_IN,
                .bmAttributes          = USB_ENDPOINT_XFER_ISOC,
                .wMaxPacketSize        = 48,
                .is_audio              = 1,
                .bInterval             = 1,
                .bRefresh              = 0,
                .bSynchAddress         = 0,
            }
        },
    }
};

static const USBDescDevice desc_device = {
    .bcdUSB                        = 0x0110,
    .bMaxPacketSize0               = 8,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 2,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE,
            .bMaxPower             = 100,
            .nif = ARRAY_SIZE(desc_iface),
            .ifs = desc_iface,
        },
    },
};

static const USBDesc desc_xblc = {
    .id = {
        .idVendor          = 0x045e,
        .idProduct         = 0x0283,
        .bcdDevice         = 0x0110,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full = &desc_device,
    .str  = desc_strings,
};

static void usb_xblc_handle_reset(USBDevice *dev)
{
    DPRINTF("[XBLC] Reset\n");
    //FIXME: What is the default sample rate/agc of the headset
}

static void output_callback(void *opaque, int avail)
{
    USBXBLCState *s = (USBXBLCState *)opaque;
    const uint8_t *data;
    uint32_t written, len;

    DPRINTF("[XBLC] Output callback want %d bytes: ", avail);

    while (avail && !fifo8_is_empty(&s->out.fifo)) {
        
        len = MIN(fifo8_num_used(&s->out.fifo), avail);
        if (len > 0) {
            data = fifo8_pop_buf(&s->out.fifo, avail, &len);
            written = AUD_write(s->out.voice, data, len);
            avail -= written;
        }
    }
}

static void input_callback(void *opaque, int avail)
{
    USBXBLCState *s = (USBXBLCState *)opaque;
    uint32_t read;

    DPRINTF("[XBLC] Input callback has %d bytes: ", avail);

    while (avail && !fifo8_is_full(&s->in.fifo)) {

        read = AUD_read(s->in.voice, s->in.packet, MIN(sizeof(s->in.packet), fifo8_num_free(&s->in.fifo)));
        if (read > 0) {
            avail -= read;
            fifo8_push_all(&s->in.fifo, s->in.packet, read);
        }
    }
}

static void xblc_audio_stream_init(USBDevice *dev, uint16_t sample_rate)
{
    USBXBLCState *s = (USBXBLCState *)dev;

    s->out.as.freq = sample_rate;
    s->out.as.nchannels = 1;
    s->out.as.fmt = AUDIO_FORMAT_S16;
    s->out.as.endianness = 0; //LE
    s->out.vol.channels = 1;

    s->in.as.freq = sample_rate;
    s->in.as.nchannels = 1;
    s->in.as.fmt = AUDIO_FORMAT_S16;
    s->in.as.endianness = 0; //LE
    s->in.vol.channels = 1;

    s->out.voice = AUD_open_out(&s->card, s->out.voice, TYPE_USB_XBLC "-speaker",
                                s, output_callback, &s->out.as);

    s->in.voice = AUD_open_in(&s->card, s->in.voice, TYPE_USB_XBLC "-microphone",
                                s, input_callback, &s->in.as);
                    
    //audio_set_volume_out(s->out.voice, &s->out.vol);
    AUD_set_active_out(s->out.voice, 0);

}

static void usb_xblc_handle_control(USBDevice *dev, USBPacket *p,
               int request, int value, int index, int length, uint8_t *data)
{
    USBXBLCState *s = (USBXBLCState *)dev;

    if (usb_desc_handle_control(dev, p, request, value, index, length, data) >= 0) {
        DPRINTF("[XBLC] Handled by usb_desc_handle_control\n");
        return;
    }

    switch (request) {
    case VendorInterfaceOutRequest | USB_REQ_SET_FEATURE:
        if (index == XBLC_SET_SAMPLE_RATE)
        {
            DPRINTF("[XBLC] Set Sample Rate to %04x\n", value & 0xFF);
            s->sample_rate = xblc_sample_rates[value & 0xFF].sample_rate;
            xblc_audio_stream_init(dev, s->sample_rate);
            return;
        }
        else if (index == XBLC_SET_AGC)
        {
            DPRINTF("[XBLC] Set Auto Gain Control to %d\n", value);
            s->auto_gain_control = (value) ? 1 : 0;
            return;
        }
        // Fallthrough       
    default:
        DPRINTF("[XBLC] USB stalled on request 0x%x value 0x%x\n", request, value);
        p->status = USB_RET_STALL;
        assert(false);
        return;
    }
}

static void usb_xblc_handle_data(USBDevice *dev, USBPacket *p)
{
    USBXBLCState *s = (USBXBLCState *)dev;
    uint32_t len;

    static int k = 0;
    //DPRINTF("k: %d ", k++);

    switch (p->pid) {
    case USB_TOKEN_IN:
        if (p->ep->nr == XBLC_EP_IN) {
            len = MIN(fifo8_num_used(&s->out.fifo), p->iov.size);
            if (len)
            {
                uint8_t *packet = fifo8_pop_buf(&s->in.fifo, len, &len);
                usb_packet_copy(p, packet, len);
                DPRINTF("[XBLC] Sent microphone data %d of %ld bytes\n", len, p->iov.size);
            }
            else
            {
                DPRINTF("[XBLC] Tried to send microphone data %d of %ld bytes. fifo underrun\n", len, p->iov.size);
            }
        } else {
            assert(false);
        }
        break;
    case USB_TOKEN_OUT:
        if (p->ep->nr == XBLC_EP_OUT) {
            usb_packet_copy(p, s->out.packet, p->iov.size);
            if (fifo8_num_free(&s->out.fifo) >= p->iov.size)
            {
                fifo8_push_all(&s->out.fifo, s->out.packet, p->iov.size);
                DPRINTF("[XBLC] Got speaker data %d bytes: ", p->iov.size);
            }
            else
            {
                DPRINTF("[XBLC] OUT FIFO OVERFLOW\n");
            }
        } else {
            assert(false);
        }
        break;
    default:
        p->status = USB_RET_STALL;
        assert(false);
        break;
    }
}

static void usb_xbox_communicator_unrealize(USBDevice *dev)
{
    USBXBLCState *s = USB_XBLC(dev);

    AUD_set_active_out(s->out.voice, false);
    AUD_set_active_in(s->in.voice, false);
    AUD_close_out(&s->card, s->out.voice);
    AUD_close_in(&s->card, s->in.voice);
    AUD_remove_card(&s->card);
}

static void usb_xblc_class_initfn(ObjectClass *klass, void *data)
{
    srand((unsigned) 1024);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);
    uc->handle_reset   = usb_xblc_handle_reset;
    uc->handle_control = usb_xblc_handle_control;
    uc->handle_data    = usb_xblc_handle_data;
    uc->handle_attach  = usb_desc_attach;
}

static void usb_xbox_communicator_realize(USBDevice *dev, Error **errp)
{
    USBXBLCState *s = USB_XBLC(dev);
    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    AUD_register_card(TYPE_USB_XBLC, &s->card);

    fifo8_create(&s->in.fifo, XBLC_FIFO_SIZE);
    fifo8_create(&s->out.fifo, XBLC_FIFO_SIZE);
}

static Property xblc_properties[] = {
    DEFINE_PROP_UINT8("index", USBXBLCState, device_index, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_usb_xbox = {
    .name = TYPE_USB_XBLC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_USB_DEVICE(dev, USBXBLCState),
        // FIXME
        VMSTATE_END_OF_LIST()
    },
};

static void usb_xbox_communicator_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->product_desc   = "Microsoft Xbox Live Communicator";
    uc->usb_desc       = &desc_xblc;
    uc->realize        = usb_xbox_communicator_realize;
    uc->unrealize      = usb_xbox_communicator_unrealize;
    usb_xblc_class_initfn(klass, data);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
    dc->vmsd  = &vmstate_usb_xbox;
    device_class_set_props(dc, xblc_properties);
    dc->desc  = "Microsoft Xbox Controller";
}

static const TypeInfo info_xblc = {
    .name          = TYPE_USB_XBLC,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(USBXBLCState),
    .class_init    = usb_xbox_communicator_class_initfn,
};

static void usb_xblc_register_types(void)
{
    type_register_static(&info_xblc);
}

type_init(usb_xblc_register_types)
