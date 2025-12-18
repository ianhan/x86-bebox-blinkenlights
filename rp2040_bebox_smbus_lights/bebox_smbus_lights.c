#include <pico/stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include <pico/i2c_slave.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <hardware/i2c.h>
#include "ws2812.pio.h"

#define NUM_PIXELS 13
#define WS2812_PIN0 5
#define WS2812_PIN1 7

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

static const uint I2C_SLAVE_SDA_PIN = 12;
static const uint I2C_SLAVE_SCL_PIN = 13;


uint phosphorbar[2][NUM_PIXELS];

static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

#define DECAY_AMOUNT 4
static inline uint32_t urgb_decaycolor(uint32_t color)
{
    uint8_t c0 = color & 0xff;
    uint8_t c1 = color >> 8 & 0xff;
    uint8_t c2 = color >> 16 & 0xff;
    if (c0 >= DECAY_AMOUNT)
        c0 -= DECAY_AMOUNT;
    else c0 = 0;
    if (c1 >= DECAY_AMOUNT)
        c1 -= DECAY_AMOUNT;
    else c1 = 0;
    if (c2 >= DECAY_AMOUNT)
        c2 -= DECAY_AMOUNT;
    else c2 = 0;
    return (c0 | c1 << 8 | c2 << 16);
}
static inline uint32_t urgb_invertlitcolor(uint32_t color)
{
    uint8_t c0 = color & 0xff;
    uint8_t c1 = color >> 8 & 0xff;
    uint8_t c2 = color >> 16 & 0xff;
    if (c0)
        c0 = 0xff - c0;
    if (c1)
        c1 = 0xff - c1;
    if (c2)
        c2 = 0xff - c2;
    return (c0 | c1 << 8 | c2 << 16);
}
static inline uint32_t urgb_incrementcolor(uint32_t color, uint32_t colormax)
{
    uint8_t cmax0 = colormax & 0xff;
    uint8_t cmax1 = colormax >> 8 & 0xff;
    uint8_t cmax2 = colormax >> 16 & 0xff;

    uint8_t c0 = color & 0xff;
    uint8_t c1 = color >> 8 & 0xff;
    uint8_t c2 = color >> 16 & 0xff;
    if (c0 + DECAY_AMOUNT <= cmax0)
        c0 += DECAY_AMOUNT;
    else c0 = cmax0;
    if (c1 + DECAY_AMOUNT <= cmax1)
        c1 += DECAY_AMOUNT;
    else c1 = cmax1;
    if (c2 + DECAY_AMOUNT <= cmax2)
        c2 += DECAY_AMOUNT;
    else c2 = cmax2;
    return (c0 | c1 << 8 | c2 << 16);
}


static uint8_t leds_to_light(uint8_t v)
{
    // +127 for rounding-to-nearest; remove it if you want floor()
    return (uint16_t)(((uint32_t)v * (uint32_t)NUM_PIXELS + 127u) / 255u);
}

#define SLAVE_CPU0 0
#define SLAVE_CPU1 1
#define SLAVE_COLOR0_R 2
#define SLAVE_COLOR0_G 3
#define SLAVE_COLOR0_B 4
#define SLAVE_COLOR1_R 5
#define SLAVE_COLOR1_G 6
#define SLAVE_COLOR1_B 7
#define SLAVE_CPU0_LEDS 8
#define SLAVE_CPU1_LEDS 9

// The slave implements a 256 byte memory. To write a series of bytes, the master first
// writes the memory address, followed by the data. The address is automatically incremented
// for each byte transferred, looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.
static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} context;

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!context.mem_address_written) {
            // writes always start with the memory address
            context.mem_address = i2c_read_byte_raw(i2c);
            context.mem_address_written = true;
        } else {
            // save into memory
            context.mem[context.mem_address] = i2c_read_byte_raw(i2c);

            // go ahead and update these now
            switch(context.mem_address)
            {
                case SLAVE_CPU0:
                    context.mem[SLAVE_CPU0_LEDS] = leds_to_light(context.mem[SLAVE_CPU0]);
                    break;
                case SLAVE_CPU1:
                    context.mem[SLAVE_CPU1_LEDS] = leds_to_light(context.mem[SLAVE_CPU1]);
                    break;
            }
            context.mem_address++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte_raw(i2c, context.mem[context.mem_address]);
        context.mem_address++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.mem_address_written = false;
        break;
    default:
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main() {
    PIO pio[2];
    uint sm[2];
    uint offset[2];
    bool success;
    stdio_init_all();

    setup_slave();

    success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio[0], &sm[0], &offset[0], WS2812_PIN0, 1, true);
    hard_assert(success);
    success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio[1], &sm[1], &offset[1], WS2812_PIN1, 1, true);
    hard_assert(success);

    ws2812_program_init(pio[0], sm[0], offset[0], WS2812_PIN0, 800000, 0);
    ws2812_program_init(pio[1], sm[1], offset[1], WS2812_PIN1, 800000, 0);

    context.mem[SLAVE_CPU0_LEDS] = leds_to_light(255);
    context.mem[SLAVE_CPU1_LEDS] = leds_to_light(255);
    context.mem[SLAVE_COLOR0_B] = 0xFF;
    context.mem[SLAVE_COLOR1_R] = 0xFF;

    while (1) {
        uint32_t color = urgb_u32(context.mem[SLAVE_COLOR0_R], context.mem[SLAVE_COLOR0_G], context.mem[SLAVE_COLOR0_B]);
        for (uint32_t pixel = 0; pixel < NUM_PIXELS; pixel++)
        {
            phosphorbar[0][pixel] = pixel < context.mem[SLAVE_CPU0_LEDS] ? urgb_incrementcolor(phosphorbar[0][pixel], color) : urgb_decaycolor(phosphorbar[0][pixel]);
            put_pixel(pio[0], sm[0], phosphorbar[0][pixel]);
        }

        color = urgb_u32(context.mem[SLAVE_COLOR1_R], context.mem[SLAVE_COLOR1_G], context.mem[SLAVE_COLOR1_B]);
        for (uint32_t pixel = 0; pixel < NUM_PIXELS; pixel++)
        {
            phosphorbar[1][pixel] = (NUM_PIXELS-1-pixel < context.mem[SLAVE_CPU1_LEDS]) ? urgb_incrementcolor(phosphorbar[1][pixel], color) : urgb_decaycolor(phosphorbar[1][pixel]);
            put_pixel(pio[1], sm[1], phosphorbar[1][pixel]);
        }

        sleep_ms(10);
    }

    pio_remove_program_and_unclaim_sm(&ws2812_program, pio[0], sm[0], offset[0]);
    pio_remove_program_and_unclaim_sm(&ws2812_program, pio[1], sm[1], offset[1]);
}