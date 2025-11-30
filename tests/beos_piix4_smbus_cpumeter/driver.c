// piix4_oled_cpumeter.c
//
// BeOS R5 Pro kernel driver that:
//   - Finds the Intel PIIX4 SMBus controller (8086:7113)
//   - Drives an SSD1306 128x32 OLED at I2C addr 0x3C
//   - Shows a 2-CPU usage meter:
//       * CPU0 label on page 0, bar on page 1
//       * CPU1 label on page 2, bar on page 3
//       * Each bar has 16 horizontal segments
//   - Meter thread starts on first open() of /dev/misc/piix4_oled_cpumeter
//
// Install:
//   /boot/home/config/add-ons/kernel/drivers/bin/piix4_oled_cpumeter
// Symlink:
//   /boot/home/config/add-ons/kernel/drivers/dev/misc/piix4_oled_cpumeter
//     -> ../../bin/piix4_oled_cpumeter

#include <KernelExport.h>
#include <Drivers.h>
#include <PCI.h>
#include <OS.h>
#include <string.h>

#define DRIVER_NAME "piix4_oled_cpumeter"
#define DEVICE_NAME "misc/piix4_oled_cpumeter"

#define DLOG(fmt, args...) dprintf(DRIVER_NAME ": " fmt, ##args)

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

/* SSD1306 parameters */

#define OLED_I2C_ADDR   0x3C    /* 7-bit address */
#define OLED_WIDTH      128
#define OLED_HEIGHT     32

/* Layout:
 *   page 0: "CPU0" label
 *   page 1: CPU0 bar
 *   page 2: "CPU1" label
 *   page 3: CPU1 bar
 */

#define CPU0_LABEL_PAGE 0
#define CPU0_BAR_PAGE   1
#define CPU1_LABEL_PAGE 2
#define CPU1_BAR_PAGE   3

/* Bar geometry */

#define LABEL_WIDTH     32   /* left strip for label text */
#define NUM_SEGMENTS    16   /* 16 horizontal segments per bar */

/* Update interval for meter (microseconds) */

#define UPDATE_INTERVAL_US 100000   /* 200 ms */


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

/* "write byte data" SMBus transaction:
 *   addr7: 7-bit I2C address
 *   command: control byte (0x00 for cmd, 0x40 for data)
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
        DLOG("smbus_write_byte_data called but SMBus not initialized\n");
        return B_ERROR;
    }

    s = smbus_wait_ready(500000);  /* 0.5s timeout */
    if (s != B_OK) {
        DLOG("smbus_wait_ready timeout (%ld)\n", s);
        return s;
    }

    sts = gPCI->read_io_8(SMBHSTSTS);
    gPCI->write_io_8(SMBHSTSTS, sts);

    gPCI->write_io_8(SMBHSTADD, (addr7 << 1) | 0);  /* write */

    gPCI->write_io_8(SMBHSTCMD, command);
    gPCI->write_io_8(SMBHSTDAT0, value);

    gPCI->write_io_8(SMBHSTCNT, SMBUS_PROTO_BYTE_DATA | SMBHSTCNT_START);

    start = system_time();
    for (;;) {
        sts = gPCI->read_io_8(SMBHSTSTS);
        if (sts & SMBHSTSTS_INTR)
            break;
        if (sts & (SMBHSTSTS_DEV_ERR | SMBHSTSTS_BUS_ERR | SMBHSTSTS_FAILED))
            break;
        if ((system_time() - start) > 500000) {
            DLOG("SMBus transaction timeout (addr 0x%02x, cmd 0x%02x)\n",
                 addr7, command);
            return B_TIMED_OUT;
        }
        snooze(50);
    }

    gPCI->write_io_8(SMBHSTSTS, sts);

    if (sts & (SMBHSTSTS_DEV_ERR | SMBHSTSTS_BUS_ERR | SMBHSTSTS_FAILED)) {
        DLOG("SMBus error: sts=0x%02x (addr 0x%02x, cmd 0x%02x)\n",
             sts, addr7, command);
        return B_ERROR;
    }

    return B_OK;
}

static void
ssd1306_cmd(uint8 cmd)
{
    (void)smbus_write_byte_data(OLED_I2C_ADDR, 0x00, cmd);
}

static void
ssd1306_data(uint8 data)
{
    (void)smbus_write_byte_data(OLED_I2C_ADDR, 0x40, data);
}


/* ---------- SSD1306 init + clear ---------- */

static void
ssd1306_init(void)
{
    DLOG("ssd1306_init\n");

    ssd1306_cmd(0xAE);        /* display off */

    ssd1306_cmd(0x20);        /* memory addressing mode */
    ssd1306_cmd(0x02);        /* page addressing mode */

    ssd1306_cmd(0xB0);        /* page 0 */
    ssd1306_cmd(0xC0);        /* COM scan direction (flipped vs 0xC8) */
    ssd1306_cmd(0x00);        /* low column start */
    ssd1306_cmd(0x10);        /* high column start */
    ssd1306_cmd(0x40);        /* start line = 0 */

    ssd1306_cmd(0x81);        /* contrast control */
    ssd1306_cmd(0x7F);        /* contrast value */

    ssd1306_cmd(0xA0);        /* segment remap (flipped vs 0xA1) */
    ssd1306_cmd(0xA6);        /* normal display */

    ssd1306_cmd(0xA8);        /* multiplex ratio */
    ssd1306_cmd(0x1F);        /* 1/32 duty (128x32) */

    ssd1306_cmd(0xA4);        /* display follows RAM */
    ssd1306_cmd(0xD3);        /* display offset */
    ssd1306_cmd(0x00);        /* no offset */

    ssd1306_cmd(0xD5);        /* display clock divide */
    ssd1306_cmd(0xF0);        /* fast for nicer photos */

    ssd1306_cmd(0xD9);        /* pre-charge */
    ssd1306_cmd(0xF1);

    ssd1306_cmd(0xDA);        /* COM pins config */
    ssd1306_cmd(0x02);        /* 128x32 */

    ssd1306_cmd(0xDB);        /* VCOMH deselect */
    ssd1306_cmd(0x40);

    ssd1306_cmd(0x8D);        /* charge pump */
    ssd1306_cmd(0x14);        /* enable */

    ssd1306_cmd(0xAF);        /* display ON */

    DLOG("ssd1306_init complete\n");
}

static void
ssd1306_clear(void)
{
    int pages;
    int page;
    int col;

    pages = OLED_HEIGHT / 8;   /* 4 pages for 128x32 */

    for (page = 0; page < pages; page++) {
        ssd1306_cmd(0xB0 + page);
        ssd1306_cmd(0x00);
        ssd1306_cmd(0x10);
        for (col = 0; col < OLED_WIDTH; col++) {
            ssd1306_data(0x00);
        }
    }
}


/* ---------- 5x7 font for labels ---------- */

static const uint8 GLYPH_SPACE[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8 GLYPH_C[5]     = { 0x3E, 0x41, 0x41, 0x41, 0x22 };
static const uint8 GLYPH_P[5]     = { 0x7F, 0x09, 0x09, 0x09, 0x06 };
static const uint8 GLYPH_U[5]     = { 0x3F, 0x40, 0x40, 0x40, 0x3F };
static const uint8 GLYPH_0[5]     = { 0x3E, 0x45, 0x49, 0x51, 0x3E };
/* Pre-rotated + horizontally flipped "1" for flipped display */
static const uint8 GLYPH_1[5]     = { 0x00, 0x42, 0x7F, 0x40, 0x00 };

static const uint8 *
glyph_for_char(char c)
{
    switch (c) {
        case 'C': return GLYPH_C;
        case 'P': return GLYPH_P;
        case 'U': return GLYPH_U;
        case '0': return GLYPH_0;
        case '1': return GLYPH_1;
        case ' ': return GLYPH_SPACE;
        default:  return GLYPH_SPACE;
    }
}

static void
ssd1306_draw_text(int page, int start_col, const char *text)
{
    int col;
    const char *p;
    const uint8 *g;
    int i;

    col = start_col;

    ssd1306_cmd(0xB0 + page);
    ssd1306_cmd(0x00 | (col & 0x0F));
    ssd1306_cmd(0x10 | (col >> 4));

    for (p = text; *p; p++) {
        g = glyph_for_char(*p);
        for (i = 0; i < 5; i++) {
            ssd1306_data(g[i]);
        }
        ssd1306_data(0x00);   /* 1 column spacing */
        col += 6;
    }
}


/* ---------- Segmented bar drawing (16 segments) ---------- */

static void
draw_usage_bar(int page, double usage)
{
    int label_width;
    int margin;
    int bar_start;
    int usable_width;
    int seg_width;
    int full_segments;
    int col;
    int seg_index;
    uint8 v;

    if (usage < 0.0)
        usage = 0.0;
    if (usage > 1.0)
        usage = 1.0;

    label_width  = 6 * 4;     /* "CPUx" ~ 4 chars * 6 columns */
    margin       = 2;
    bar_start    = label_width + margin;
    usable_width = OLED_WIDTH - bar_start - margin;

    if (usable_width <= 0)
        return;

    seg_width = usable_width / NUM_SEGMENTS;
    if (seg_width <= 0)
        seg_width = 1;

    full_segments = (int)(usage * NUM_SEGMENTS + 0.5);
    if (full_segments < 0)
        full_segments = 0;
    if (full_segments > NUM_SEGMENTS)
        full_segments = NUM_SEGMENTS;

    ssd1306_cmd(0xB0 + page);
    ssd1306_cmd(0x00 | (bar_start & 0x0F));
    ssd1306_cmd(0x10 | (bar_start >> 4));

    for (col = 0; col < usable_width; col++) {
        v = 0x00;
        seg_index = col / seg_width;

        if (seg_index < full_segments) {
            /* Fill most of the segment, leave 1-column gap at end if wide enough */
            if (seg_width > 2) {
                if ((col % seg_width) < (seg_width - 1))
                    v = 0xFF;
                else
                    v = 0x00;
            } else {
                v = 0xFF;
            }
        } else {
            v = 0x00;
        }

        ssd1306_data(v);
    }
}


/* ---------- Meter thread (uses system_info.cpu_infos like GeekView) ---------- */

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

    (void)arg;

    if (gStopThread)
        return 0;

    DLOG("meter_thread_func: starting (OLED segmented)\n");

    ssd1306_init();
    ssd1306_clear();

    /* Static labels on label pages */
    ssd1306_draw_text(CPU0_LABEL_PAGE, 0, "CPU0");
    ssd1306_draw_text(CPU1_LABEL_PAGE, 0, "CPU1");

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

    /* Initial bars at 0% */
    draw_usage_bar(CPU0_BAR_PAGE, 0.0);
    draw_usage_bar(CPU1_BAR_PAGE, 0.0);

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

        delta_active0 = sysinfo.cpu_infos[0].active_time - last_active[0];
        usage0 = (double)delta_active0 / (double)delta_time;
        if (usage0 < 0.0)
            usage0 = 0.0;
        if (usage0 > 1.0)
            usage0 = 1.0;

        if (count > 1) {
            delta_active1 = sysinfo.cpu_infos[1].active_time - last_active[1];
            usage1 = (double)delta_active1 / (double)delta_time;
            if (usage1 < 0.0)
                usage1 = 0.0;
            if (usage1 > 1.0)
                usage1 = 1.0;
        } else {
            usage1 = usage0;
        }

        last_active[0] = sysinfo.cpu_infos[0].active_time;
        if (count > 1)
            last_active[1] = sysinfo.cpu_infos[1].active_time;
        else
            last_active[1] = last_active[0];

        last_time = now;

        draw_usage_bar(CPU0_BAR_PAGE, usage0);
        draw_usage_bar(CPU1_BAR_PAGE, usage1);
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
    (void)op;
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
