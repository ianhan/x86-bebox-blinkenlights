// gcc cpu_oled_meter_128x32.c -o cpu_oled_meter_128x32 -li2c
// run as root: ./cpu_oled_meter_128x32

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <time.h>

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>   // from i2c-tools

#define OLED_I2C_ADDR  0x3C      // change to 0x3D if your module uses that
#define OLED_WIDTH     128
#define OLED_HEIGHT    32        // fixed for 128x32 panel

// Layout: 4 pages (0..3), each 8px tall
// page 0: "CPU0" label
// page 1: CPU0 bar
// page 2: "CPU1" label
// page 3: CPU1 bar
#define CPU0_LABEL_PAGE 0
#define CPU0_BAR_PAGE   1
#define CPU1_LABEL_PAGE 2
#define CPU1_BAR_PAGE   3

// How often to refresh bars (milliseconds)
#define UPDATE_INTERVAL_MS 200

// ===== SSD1306 low-level =====

static void ssd1306_cmd(int fd, uint8_t cmd)
{
    if (i2c_smbus_write_byte_data(fd, 0x00, cmd) < 0) {
        perror("ssd1306_cmd");
    }
}

static void ssd1306_data(int fd, uint8_t data)
{
    if (i2c_smbus_write_byte_data(fd, 0x40, data) < 0) {
        perror("ssd1306_data");
    }
}

static void ssd1306_init(int fd)
{
    // Init sequence tuned for SSD1306 128x32, fast refresh 0xF0
    ssd1306_cmd(fd, 0xAE);        // display off

    ssd1306_cmd(fd, 0x20);        // memory addressing mode
    ssd1306_cmd(fd, 0x02);        // page addressing mode

    ssd1306_cmd(fd, 0xB0);        // page 0
    ssd1306_cmd(fd, 0xC0);        // COM output scan direction remapped
    ssd1306_cmd(fd, 0x00);        // low column start
    ssd1306_cmd(fd, 0x10);        // high column start
    ssd1306_cmd(fd, 0x40);        // start line = 0

    ssd1306_cmd(fd, 0x81);        // contrast control
    ssd1306_cmd(fd, 0x7F);        // contrast value

    ssd1306_cmd(fd, 0xA0);        // segment remap
    ssd1306_cmd(fd, 0xA6);        // normal display (not inverted)

    // 128x32: multiplex ratio 1/32
    ssd1306_cmd(fd, 0xA8);        // multiplex ratio
    ssd1306_cmd(fd, 0x1F);        // 1/32 duty

    ssd1306_cmd(fd, 0xA4);        // display follows RAM content
    ssd1306_cmd(fd, 0xD3);        // display offset
    ssd1306_cmd(fd, 0x00);        // no offset

    // Fast refresh for easier photos: oscillator/divider = 0xF0
    ssd1306_cmd(fd, 0xD5);        // display clock divide
    ssd1306_cmd(fd, 0xF0);        // max osc, divide-by-1

    ssd1306_cmd(fd, 0xD9);        // pre-charge
    ssd1306_cmd(fd, 0xF1);

    ssd1306_cmd(fd, 0xDA);        // COM pins hardware config for 128x32
    ssd1306_cmd(fd, 0x02);

    ssd1306_cmd(fd, 0xDB);        // VCOMH deselect
    ssd1306_cmd(fd, 0x40);

    ssd1306_cmd(fd, 0x8D);        // charge pump
    ssd1306_cmd(fd, 0x14);        // enable

    ssd1306_cmd(fd, 0xAF);        // display ON
}

static void ssd1306_clear(int fd)
{
    // One-time full clear at startup (4 pages for 32px)
    int pages = OLED_HEIGHT / 8;  // 4
    for (int page = 0; page < pages; page++) {
        ssd1306_cmd(fd, 0xB0 + page);
        ssd1306_cmd(fd, 0x00);
        ssd1306_cmd(fd, 0x10);
        for (int col = 0; col < OLED_WIDTH; col++) {
            ssd1306_data(fd, 0x00);
        }
    }
}

// ===== 5x7 font for labels ("CPU0", "CPU1") =====

static const uint8_t GLYPH_SPACE[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t GLYPH_C[5]     = { 0x3E, 0x41, 0x41, 0x41, 0x22 };
static const uint8_t GLYPH_P[5]     = { 0x7F, 0x09, 0x09, 0x09, 0x06 };
static const uint8_t GLYPH_U[5]     = { 0x3F, 0x40, 0x40, 0x40, 0x3F };
static const uint8_t GLYPH_0[5]     = { 0x3E, 0x45, 0x49, 0x51, 0x3E };
static const uint8_t GLYPH_1[5]     = { 0x00, 0x42, 0x7F, 0x40, 0x00 };

static const uint8_t *glyph_for_char(char c)
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

// Draw a single 5x7 character at (page, start_col)
// Uses page addressing; character height is 7 pixels.
static void ssd1306_draw_text(int fd, int page, int start_col, const char *text)
{
    int col = start_col;

    ssd1306_cmd(fd, 0xB0 + page);            // select page
    ssd1306_cmd(fd, 0x00 | (col & 0x0F));    // lower column
    ssd1306_cmd(fd, 0x10 | (col >> 4));      // higher column

    for (const char *p = text; *p; p++) {
        const uint8_t *g = glyph_for_char(*p);
        for (int i = 0; i < 5; i++) {
            ssd1306_data(fd, g[i]);
        }
        // 1 column spacing
        ssd1306_data(fd, 0x00);
        col += 6;
    }
}

// ===== Bar drawing (segmented LED-style) =====

// Draw a segmented horizontal bar on 'page' with usage in [0.0, 1.0].
//
// Visual style:
//   [CPUx] ███ ██ ██      (segments of 6px filled, 2px gap)
// Only the bar row (page) is updated each frame.
static void draw_usage_bar(int fd, int page, double usage)
{
    if (usage < 0.0) usage = 0.0;
    if (usage > 1.0) usage = 1.0;

    const int label_width = 6 * 4;           // "CPUx" ~ 4 chars * 6 columns
    const int margin = 2;
    const int bar_start = label_width + margin;
    const int usable_width = OLED_WIDTH - bar_start - margin;

    int filled = (int)((usage * usable_width + 0.5) / 8) * 8;

    ssd1306_cmd(fd, 0xB0 + page);               // select bar page
    ssd1306_cmd(fd, 0x00 | (bar_start & 0x0F)); // lower column
    ssd1306_cmd(fd, 0x10 | (bar_start >> 4));   // higher column

    for (int x = 0; x < usable_width; x++) {
        uint8_t v = 0x00;

        if (x < filled) {
            // Segmented look: 6 columns on, 2 columns off -> 8-column "LEDs"
            if ((x % 8) < 6) {
                v = 0xFF;   // lit segment
            } else {
                v = 0x00;   // gap between segments
            }
        } else {
            v = 0x00;
        }

        ssd1306_data(fd, v);
    }
}

// ===== CPU usage from /proc/stat =====

static int read_cpu_times(int cpu, unsigned long long *idle, unsigned long long *total)
{
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return -1;

    char line[256];
    char target[16];
    snprintf(target, sizeof(target), "cpu%d", cpu);

    int found = 0;
    while (fgets(line, sizeof(line), f)) {
        if (strncmp(line, target, strlen(target)) == 0 &&
            line[strlen(target)] == ' ') {

            unsigned long long user, nice, system, idle_v;
            unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;
            int n = sscanf(line + strlen(target),
                           "%llu %llu %llu %llu %llu %llu %llu %llu",
                           &user, &nice, &system, &idle_v,
                           &iowait, &irq, &softirq, &steal);
            if (n >= 4) {
                *idle = idle_v + ((n > 4) ? iowait : 0);
                *total = user + nice + system + idle_v + iowait + irq + softirq + steal;
                found = 1;
            }
            break;
        }
    }

    fclose(f);
    return found ? 0 : -1;
}

static double compute_cpu_usage(unsigned long long prev_idle,
                                unsigned long long prev_total,
                                unsigned long long cur_idle,
                                unsigned long long cur_total)
{
    unsigned long long delta_idle  = cur_idle  - prev_idle;
    unsigned long long delta_total = cur_total - prev_total;

    if (delta_total == 0) return 0.0;

    double idle_frac  = (double)delta_idle / (double)delta_total;
    double usage_frac = 1.0 - idle_frac;
    if (usage_frac < 0.0) usage_frac = 0.0;
    if (usage_frac > 1.0) usage_frac = 1.0;
    return usage_frac;
}

static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

// ===== main =====

int main(void)
{
    const char *dev = "/dev/i2c-0";
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open /dev/i2c-0");
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, OLED_I2C_ADDR) < 0) {
        perror("ioctl I2C_SLAVE");
        close(fd);
        return 1;
    }

    ssd1306_init(fd);
    ssd1306_clear(fd);

    // Static labels: drawn once, never touched again
    ssd1306_draw_text(fd, CPU0_LABEL_PAGE, 0, "CPU0");
    ssd1306_draw_text(fd, CPU1_LABEL_PAGE, 0, "CPU1");

    // Prime CPU stats
    unsigned long long prev_idle[2]  = {0, 0};
    unsigned long long prev_total[2] = {0, 0};
    read_cpu_times(0, &prev_idle[0], &prev_total[0]);
    read_cpu_times(1, &prev_idle[1], &prev_total[1]);

    // Main loop: update bars only
    while (1) {
        unsigned long long cur_idle[2], cur_total[2];

        if (read_cpu_times(0, &cur_idle[0], &cur_total[0]) == 0 &&
            read_cpu_times(1, &cur_idle[1], &cur_total[1]) == 0) {

            double usage0 = compute_cpu_usage(prev_idle[0], prev_total[0],
                                              cur_idle[0], cur_total[0]);
            double usage1 = compute_cpu_usage(prev_idle[1], prev_total[1],
                                              cur_idle[1], cur_total[1]);

            prev_idle[0]  = cur_idle[0];
            prev_total[0] = cur_total[0];
            prev_idle[1]  = cur_idle[1];
            prev_total[1] = cur_total[1];

            // Only redraw the two bar rows (pages) – minimal display traffic
            draw_usage_bar(fd, CPU0_BAR_PAGE, usage0);
            draw_usage_bar(fd, CPU1_BAR_PAGE, usage1);
        }

        sleep_ms(UPDATE_INTERVAL_MS);
    }

    // not reached
    close(fd);
    return 0;
}
