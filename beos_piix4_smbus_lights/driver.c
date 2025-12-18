//
// BeOS R5/Bebox kernel driver that:
//   - Finds the Intel PIIX4 SMBus controller (8086:7113)
//   - Talks to an RP2040 I2C slave at 0x17
//   - Periodically writes CPU0/CPU1 usage (0..255) into RP2040 registers:
//        0x00 -> SLAVE_CPU0      (brightness 0..255)
//        0x01 -> SLAVE_CPU1      (brightness 0..255)
//   - Uses these color registers to set bar colors:
//        SLAVE_COLOR0_R/G/B  (bar 0 / CPU0)
//        SLAVE_COLOR1_R/G/B  (bar 1 / CPU1)
//   - Meter thread starts on first open() of /dev/misc/piix4_smbus_lights
//   - IOCTL support to change the two colors at runtime.
//
// Install:
//   /boot/home/config/add-ons/kernel/drivers/bin/piix4_smbus_lights
// Symlink:
//   /boot/home/config/add-ons/kernel/drivers/dev/misc/piix4_smbus_lights
//     -> ../../bin/piix4_smbus_lights

#include <KernelExport.h>
#include <Drivers.h>
#include <PCI.h>
#include <OS.h>
#include <string.h>

#define DRIVER_NAME "piix4_smbus_lights"
#define DEVICE_NAME "misc/piix4_smbus_lights"

#define DLOG(fmt, args...) dprintf(DRIVER_NAME ": " fmt, ##args)

/* ---------- RP2040 device config ---------- */

#define I2C_ADDR_LEDS   0x17  /* RP2040 I2C slave address (7-bit) */

#define SLAVE_CPU0      0
#define SLAVE_CPU1      1
#define SLAVE_COLOR0_R  2
#define SLAVE_COLOR0_G  3
#define SLAVE_COLOR0_B  4
#define SLAVE_COLOR1_R  5
#define SLAVE_COLOR1_G  6
#define SLAVE_COLOR1_B  7
/* 8,9 exist but are left to firmware */

/* Update interval (microseconds) */
#define UPDATE_INTERVAL_US 200000   /* 200 ms */

/* ---------- IOCTL interface ---------- */

/* User/kernel shared color struct */
typedef struct led_color {
    uint8 r;
    uint8 g;
    uint8 b;
} led_color_t;

/* IOCTL codes (4-char constants, common style in BeOS) */
#define LED_IOCTL_SET_COLOR0  'lc00'
#define LED_IOCTL_SET_COLOR1  'lc01'

/* Current colors (defaults: green for both) */
static led_color_t gColor0 = { 0, 255, 0 };
static led_color_t gColor1 = { 0, 255, 0 };

/* ---------- Globals ---------- */

static pci_module_info *gPCI = NULL;

static bool      gHasSMBus    = false;
static uint16    gSMBBase     = 0;      /* I/O base */
static thread_id gMeterThread = -1;
static bool      gStopThread  = false;
static int32     gOpenCount   = 0;
static int32     gCpuCount    = 1;

/* PIIX4 PCI IDs */

#define PIIX4_VENDOR_ID        0x8086
#define PIIX4_DEVICE_ID_SMB    0x7113

/* PCI config offsets */

#define PCI_SMBBA_OFFSET       0x90    /* SMBus Base Address */
#define PCI_SMBHSTCFG          0xD2    /* SMBus Host Configuration */

/* SMBus base mask: IO base is bits 15:5 */

#define SMB_BASE_MASK          0xFFE0

/* SMBus host registers (offsets from gSMBBase) */

#define SMBHSTSTS   (gSMBBase + 0)
#define SMBHSTCNT   (gSMBBase + 2)
#define SMBHSTCMD   (gSMBBase + 3)
#define SMBHSTADD   (gSMBBase + 4)
#define SMBHSTDAT0  (gSMBBase + 5)
#define SMBHSTDAT1  (gSMBBase + 6)
#define SMBBLKDAT   (gSMBBase + 7)

/* SMBHSTSTS bits */

#define SMBHSTSTS_HOST_BUSY 0x01
#define SMBHSTSTS_INTR      0x02
#define SMBHSTSTS_DEV_ERR   0x04
#define SMBHSTSTS_BUS_ERR   0x08
#define SMBHSTSTS_FAILED    0x10

/* SMBHSTCNT bits / protocol codes
 * PIIX4 encodes protocol in bits 4..2, not 0..2:
 *   QUICK      0b000 << 2 = 0x00
 *   BYTE       0b001 << 2 = 0x04
 *   BYTE_DATA  0b010 << 2 = 0x08
 *   WORD_DATA  0b011 << 2 = 0x0C
 *   BLOCK      0b101 << 2 = 0x14
 */

#define SMBHSTCNT_START         0x40

#define SMBUS_PROTO_QUICK       0x00
#define SMBUS_PROTO_BYTE        0x04
#define SMBUS_PROTO_BYTE_DATA   0x08
#define SMBUS_PROTO_WORD_DATA   0x0C
#define SMBUS_PROTO_BLOCK       0x14


/* ---------- SMBus helpers ---------- */

static status_t
smbus_wait_ready(bigtime_t timeout)
{
    bigtime_t start;
    uint8 sts;

    start = system_time();
    for (;;) {
        sts = gPCI->read_io_8(SMBHSTSTS);
        if (!(sts & SMBHSTSTS_HOST_BUSY))
            return B_OK;
        if ((system_time() - start) > timeout)
            return B_TIMED_OUT;
        snooze(50);
    }
}

/* SMBus "write byte data":
 *   addr7: 7-bit I2C address
 *   command: register
 *   value: data byte
 */
static status_t
smbus_write_byte_data(uint8 addr7, uint8 command, uint8 value)
{
    status_t s;
    uint8 sts;
    bigtime_t start;

    if (gStopThread)
        return B_ERROR;

    if (!gHasSMBus) {
        DLOG("smbus_write_byte_data: SMBus not initialized\n");
        return B_ERROR;
    }

    s = smbus_wait_ready(500000);  /* 0.5s timeout */
    if (s != B_OK) {
        DLOG("smbus_wait_ready timeout (%ld)\n", s);
        return s;
    }

    /* Clear status bits by writing 1s to them */
    sts = gPCI->read_io_8(SMBHSTSTS);
    gPCI->write_io_8(SMBHSTSTS, sts);

    /* Address (7-bit <<1, R/W=0 for write) */
    gPCI->write_io_8(SMBHSTADD, (addr7 << 1) | 0);

    /* Command = register index */
    gPCI->write_io_8(SMBHSTCMD, command);

    /* Data byte */
    gPCI->write_io_8(SMBHSTDAT0, value);

    /* Protocol: BYTE_DATA + START */
    gPCI->write_io_8(SMBHSTCNT, SMBUS_PROTO_BYTE_DATA | SMBHSTCNT_START);

    /* Poll completion */
    start = system_time();
    for (;;) {
        sts = gPCI->read_io_8(SMBHSTSTS);
        if (sts & SMBHSTSTS_INTR)
            break;
        if (sts & (SMBHSTSTS_DEV_ERR | SMBHSTSTS_BUS_ERR | SMBHSTSTS_FAILED))
            break;
        if ((system_time() - start) > 500000) {
            DLOG("SMBus transaction timeout (addr 0x%02x, reg 0x%02x)\n",
                 addr7, command);
            return B_TIMED_OUT;
        }
        snooze(50);
    }

    /* Clear status again */
    gPCI->write_io_8(SMBHSTSTS, sts);

    if (sts & (SMBHSTSTS_DEV_ERR | SMBHSTSTS_BUS_ERR | SMBHSTSTS_FAILED)) {
        DLOG("SMBus error: sts=0x%02x (addr 0x%02x, reg 0x%02x)\n",
             sts, addr7, command);
        return B_ERROR;
    }

    return B_OK;
}


/* ---------- Meter thread (GeekView-style CPU usage) ---------- */

static int32
meter_thread_func(void *arg)
{
    system_info sysinfo;
    bigtime_t last_time;
    bigtime_t now;
    bigtime_t last_active[2];
    bigtime_t delta_time;
    bigtime_t delta_active0;
    bigtime_t delta_active1;
    double usage0;
    double usage1;
    int32 count;
    int v0;
    int v1;
    status_t s;

    (void)arg;

    if (gStopThread)
        return 0;

    DLOG("meter_thread_func: starting (RP2040 LED meter)\n");

    /* One-time: set both bars to the current colors (default green) */
    s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR0_R, gColor0.r);
    if (s != B_OK) DLOG("set COLOR0_R failed (%ld)\n", s);
    s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR0_G, gColor0.g);
    if (s != B_OK) DLOG("set COLOR0_G failed (%ld)\n", s);
    s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR0_B, gColor0.b);
    if (s != B_OK) DLOG("set COLOR0_B failed (%ld)\n", s);

    s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR1_R, gColor1.r);
    if (s != B_OK) DLOG("set COLOR1_R failed (%ld)\n", s);
    s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR1_G, gColor1.g);
    if (s != B_OK) DLOG("set COLOR1_G failed (%ld)\n", s);
    s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR1_B, gColor1.b);
    if (s != B_OK) DLOG("set COLOR1_B failed (%ld)\n", s);

    last_time = system_time();
    get_system_info(&sysinfo);

    count = sysinfo.cpu_count;
    if (count > 2)
        count = 2;
    if (count < 1)
        count = 1;

    last_active[0] = sysinfo.cpu_infos[0].active_time;
    if (count > 1)
        last_active[1] = sysinfo.cpu_infos[1].active_time;
    else
        last_active[1] = last_active[0];

    /* Initialize brightness to 0 */
    (void)smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_CPU0, 0);
    (void)smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_CPU1, 0);

    while (!gStopThread) {
        snooze(UPDATE_INTERVAL_US);

        now = system_time();
        get_system_info(&sysinfo);

        count = sysinfo.cpu_count;
        if (count > 2)
            count = 2;
        if (count < 1)
            count = 1;

        delta_time = now - last_time;
        if (delta_time <= 0)
            continue;

        /* CPU0 */
        delta_active0 = sysinfo.cpu_infos[0].active_time - last_active[0];
        usage0 = (double)delta_active0 / (double)delta_time;
        if (usage0 < 0.0) usage0 = 0.0;
        if (usage0 > 1.0) usage0 = 1.0;

        /* CPU1 or mirror CPU0 if only one CPU */
        if (count > 1) {
            delta_active1 = sysinfo.cpu_infos[1].active_time - last_active[1];
            usage1 = (double)delta_active1 / (double)delta_time;
            if (usage1 < 0.0) usage1 = 0.0;
            if (usage1 > 1.0) usage1 = 1.0;
        } else {
            usage1 = usage0;
        }

        last_active[0] = sysinfo.cpu_infos[0].active_time;
        if (count > 1)
            last_active[1] = sysinfo.cpu_infos[1].active_time;
        else
            last_active[1] = last_active[0];

        last_time = now;

        /* Map usage [0,1] to 0..255 */
        v0 = (int)(usage0 * 255.0 + 0.5);
        v1 = (int)(usage1 * 255.0 + 0.5);

        if (v0 < 0)   v0 = 0;
        if (v0 > 255) v0 = 255;
        if (v1 < 0)   v1 = 0;
        if (v1 > 255) v1 = 255;

        s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_CPU0, (uint8)v0);
        if (s != B_OK) DLOG("write SLAVE_CPU0 failed (%ld)\n", s);

        s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_CPU1, (uint8)v1);
        if (s != B_OK) DLOG("write SLAVE_CPU1 failed (%ld)\n", s);
    }

    DLOG("meter_thread_func: exiting\n");
    return 0;
}


/* ---------- PIIX4 SMBus discovery ---------- */

static status_t
init_piix4_smbus(void)
{
    pci_info info;
    int32 index;
    bool found;
    uint16 smba;
    uint8 hstcfg;

    DLOG("init_piix4_smbus: probing for PIIX4 SMBus\n");

    index = 0;
    found = false;

    while (gPCI->get_nth_pci_info(index, &info) == B_OK) {
        if (info.vendor_id == PIIX4_VENDOR_ID &&
            info.device_id == PIIX4_DEVICE_ID_SMB) {
            found = true;
            break;
        }
        index++;
    }

    if (!found) {
        DLOG("init_piix4_smbus: PIIX4 SMBus not found\n");
        return B_ERROR;
    }

    smba = (uint16)gPCI->read_pci_config(info.bus, info.device, info.function,
                                         PCI_SMBBA_OFFSET, 2);
    smba &= SMB_BASE_MASK;
    if (smba == 0) {
        DLOG("init_piix4_smbus: SMBus base is 0\n");
        return B_ERROR;
    }

    hstcfg = (uint8)gPCI->read_pci_config(info.bus, info.device, info.function,
                                          PCI_SMBHSTCFG, 1);
    if (!(hstcfg & 0x01)) {
        DLOG("init_piix4_smbus: enabling SMBus I/O space\n");
        hstcfg |= 0x01;
        gPCI->write_pci_config(info.bus, info.device, info.function,
                               PCI_SMBHSTCFG, 1, hstcfg);
    }

    gSMBBase  = smba;
    gHasSMBus = true;

    DLOG("init_piix4_smbus: PIIX4 SMBus at I/O 0x%04x\n", gSMBBase);

    return B_OK;
}


/* ---------- Device hooks ---------- */

static const char *gDeviceNames[] = {
    DEVICE_NAME,
    NULL
};

static status_t
piix4_open(const char *name, uint32 flags, void **cookie)
{
    (void)flags;
    (void)cookie;

    DLOG("open('%s')\n", name);

    if (gOpenCount == 0 && gMeterThread < 0) {
        DLOG("starting meter thread on first open\n");
        gStopThread = false;
        gMeterThread = spawn_kernel_thread(meter_thread_func,
                                           DRIVER_NAME "_meter",
                                           B_NORMAL_PRIORITY,
                                           NULL);
        if (gMeterThread < 0) {
            DLOG("spawn_kernel_thread failed: %ld\n", gMeterThread);
            return gMeterThread;
        }
        resume_thread(gMeterThread);
    }

    gOpenCount++;
    return B_OK;
}

static status_t
piix4_close(void *cookie)
{
    (void)cookie;
    DLOG("close\n");

    if (gOpenCount > 0)
        gOpenCount--;

    return B_OK;
}

static status_t
piix4_free(void *cookie)
{
    (void)cookie;
    return B_OK;
}

static status_t
piix4_ioctl(void *cookie, uint32 op, void *buffer, size_t len)
{
    (void)cookie;

    if (op == LED_IOCTL_SET_COLOR0 || op == LED_IOCTL_SET_COLOR1) {
        led_color_t *col;
        status_t s;

        if (buffer == NULL || len < sizeof(led_color_t))
            return B_BAD_VALUE;

        col = (led_color_t *)buffer;

        if (op == LED_IOCTL_SET_COLOR0) {
            gColor0 = *col;
            /* push immediately to RP2040 */
            s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR0_R, gColor0.r);
            if (s != B_OK) DLOG("ioctl COLOR0_R failed (%ld)\n", s);
            s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR0_G, gColor0.g);
            if (s != B_OK) DLOG("ioctl COLOR0_G failed (%ld)\n", s);
            s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR0_B, gColor0.b);
            if (s != B_OK) DLOG("ioctl COLOR0_B failed (%ld)\n", s);
        } else { /* LED_IOCTL_SET_COLOR1 */
            gColor1 = *col;
            s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR1_R, gColor1.r);
            if (s != B_OK) DLOG("ioctl COLOR1_R failed (%ld)\n", s);
            s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR1_G, gColor1.g);
            if (s != B_OK) DLOG("ioctl COLOR1_G failed (%ld)\n", s);
            s = smbus_write_byte_data(I2C_ADDR_LEDS, SLAVE_COLOR1_B, gColor1.b);
            if (s != B_OK) DLOG("ioctl COLOR1_B failed (%ld)\n", s);
        }

        return B_OK;
    }

    (void)buffer;
    (void)len;
    return B_DEV_INVALID_IOCTL;
}

static status_t
piix4_read(void *cookie, off_t pos, void *buffer, size_t *len)
{
    (void)cookie;
    (void)pos;
    (void)buffer;
    (void)len;
    return B_ERROR;
}

static status_t
piix4_write(void *cookie, off_t pos, const void *buffer, size_t *len)
{
    (void)cookie;
    (void)pos;
    (void)buffer;
    (void)len;
    return B_ERROR;
}

static device_hooks gDeviceHooks = {
    piix4_open,
    piix4_close,
    piix4_free,
    piix4_ioctl,   /* control/ioctl */
    piix4_read,
    piix4_write,
    NULL,   /* select */
    NULL,   /* deselect */
    NULL,   /* readv */
    NULL    /* writev */
};


/* ---------- Driver entry points ---------- */

status_t
init_hardware(void)
{
    DLOG("init_hardware\n");
    return B_OK;
}

status_t
init_driver(void)
{
    status_t status;
    system_info si;

    DLOG("init_driver\n");

    if (gPCI == NULL) {
        status = get_module(B_PCI_MODULE_NAME, (module_info **)&gPCI);
        if (status != B_OK) {
            DLOG("get_module(PCI) failed: %ld\n", status);
            return status;
        }
    }

    status = init_piix4_smbus();
    if (status != B_OK) {
        DLOG("init_piix4_smbus failed: %ld\n", status);
        return status;
    }

    get_system_info(&si);
    gCpuCount = si.cpu_count;
    if (gCpuCount < 1)
        gCpuCount = 1;
    DLOG("init_driver: cpu_count = %ld\n", gCpuCount);

    gStopThread  = false;
    gMeterThread = -1;
    gOpenCount   = 0;

    return B_OK;
}

void
uninit_driver(void)
{
    status_t dummy;

    DLOG("uninit_driver\n");

    if (gMeterThread >= 0) {
        gStopThread = true;
        wait_for_thread(gMeterThread, &dummy);
        DLOG("meter thread joined (status %ld)\n", dummy);
        gMeterThread = -1;
    }

    if (gPCI != NULL) {
        put_module(B_PCI_MODULE_NAME);
        gPCI = NULL;
    }
}

const char **
publish_devices(void)
{
    DLOG("publish_devices\n");
    return gDeviceNames;
}

device_hooks *
find_device(const char *name)
{
    DLOG("find_device('%s')\n", name);
    if (strcmp(name, DEVICE_NAME) == 0)
        return &gDeviceHooks;
    return NULL;
}
