// gcc ssd1306_hello.c -o ssd1306_hello
// run as root: ./ssd1306_hello

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>   // from i2c-tools

#define OLED_I2C_ADDR 0x3C   // change to 0x3D if your module uses that

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
    // Minimal-ish init for 128x64 SSD1306
    ssd1306_cmd(fd, 0xAE);        // display off

    ssd1306_cmd(fd, 0x20);        // memory addressing mode
    ssd1306_cmd(fd, 0x02);        // page addressing mode

    ssd1306_cmd(fd, 0xB0);        // page 0
    ssd1306_cmd(fd, 0xC8);        // COM output scan direction remapped
    ssd1306_cmd(fd, 0x00);        // low column start
    ssd1306_cmd(fd, 0x10);        // high column start
    ssd1306_cmd(fd, 0x40);        // start line = 0

    ssd1306_cmd(fd, 0x81);        // contrast control
    ssd1306_cmd(fd, 0x7F);        // contrast value

    ssd1306_cmd(fd, 0xA1);        // segment remap
    ssd1306_cmd(fd, 0xA6);        // normal display (not inverted)

    ssd1306_cmd(fd, 0xA8);        // multiplex ratio
    ssd1306_cmd(fd, 0x1F);        // 1/64 duty

    ssd1306_cmd(fd, 0xA4);        // display follows RAM content
    ssd1306_cmd(fd, 0xD3);        // display offset
    ssd1306_cmd(fd, 0x00);        // no offset

    ssd1306_cmd(fd, 0xD5);        // display clock divide
    ssd1306_cmd(fd, 0xF0);

    ssd1306_cmd(fd, 0xD9);        // pre-charge
    ssd1306_cmd(fd, 0xF1);

    ssd1306_cmd(fd, 0xDA);        // COM pins hardware config
    ssd1306_cmd(fd, 0x02);

    ssd1306_cmd(fd, 0xDB);        // VCOMH deselect
    ssd1306_cmd(fd, 0x40);

    ssd1306_cmd(fd, 0x8D);        // charge pump
    ssd1306_cmd(fd, 0x14);        // enable

    ssd1306_cmd(fd, 0xAF);        // display ON
}

static void ssd1306_clear(int fd)
{
    for (int page = 0; page < 4; page++) {
        ssd1306_cmd(fd, 0xB0 + page);      // set page
        ssd1306_cmd(fd, 0x00);             // low column = 0
        ssd1306_cmd(fd, 0x10);             // high column = 0

        for (int col = 0; col < 128; col++) {
            ssd1306_data(fd, 0x00);
        }
    }
}

static void ssd1306_set_cursor_page0_col(int fd, uint8_t col)
{
    // We only draw one text row on page 0 for this demo
    ssd1306_cmd(fd, 0xB0 + 1);            // page 1

    ssd1306_cmd(fd, 0x00 | (col & 0x0F)); // lower column
    ssd1306_cmd(fd, 0x10 | (col >> 4));   // higher column
}

static void draw_hello(int fd)
{
    // 5x7 glyphs, 1 column of spacing between characters
    // Bit 0 = top row, Bit 6 = bottom of glyph, Bit 7 unused
    const uint8_t H[5] = { 0x7F, 0x08, 0x08, 0x08, 0x7F };
    const uint8_t E[5] = { 0x7F, 0x49, 0x49, 0x49, 0x41 };
    const uint8_t L[5] = { 0x7F, 0x40, 0x40, 0x40, 0x40 };
    const uint8_t O[5] = { 0x3E, 0x41, 0x41, 0x41, 0x3E };

    const uint8_t *letters[] = { H, E, L, L, O };
    const int num_letters = 5;

    // crude centering: 6 columns per letter (5 + 1 space)
    int total_cols = num_letters * 6;
    int start_col = (128 - total_cols) / 2;
    if (start_col < 0) start_col = 0;

    ssd1306_set_cursor_page0_col(fd, start_col);

    for (int i = 0; i < num_letters; i++) {
        const uint8_t *g = letters[i];
        for (int col = 0; col < 5; col++) {
            ssd1306_data(fd, g[col]);
        }
        // one column of spacing
        ssd1306_data(fd, 0x00);
    }
}

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
    draw_hello(fd);

    close(fd);
    return 0;
}
