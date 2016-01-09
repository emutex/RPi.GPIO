/*
Copyright (c) 2012-2015 Ben Croston

Adapted for use on UP board (http://www.up-board.org/)
Copyright (c) 2015 Emutex Ltd
Author: Dan O'Donovan <dan@emutex.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "c_gpio.h"

struct gpio_chip_mmr {
    uint64_t phys;
    void *virt;
    unsigned size;
    int fd;
};
	
struct up_gpio_pin_desc {
	struct gpio_chip_mmr *mmr;
	unsigned pad;
	unsigned offset;
};

static struct gpio_chip_mmr cht_gpio_mmr_SW = {
    .phys = 0xfed80000,
    .size = 0x8000
};
static struct gpio_chip_mmr cht_gpio_mmr_N  = {
    .phys = 0xfed88000,
    .size = 0x8000
};
static struct gpio_chip_mmr cht_gpio_mmr_E  = {
    .phys = 0xfed90000,
    .size = 0x8000
};
static struct gpio_chip_mmr cht_gpio_mmr_SE = {
    .phys = 0xfed98000,
    .size = 0x8000
};

#define CHT_INTSTAT_OFFSET 0x300

#define CHT_PADREG0_INTSEL_SHIFT   28
#define CHT_PADREG0_INTSEL_MASK    (0xf << CHT_PADREG0_INTSEL_SHIFT)

#define CHT_PADREG1_INVRXTX_MASK   0xf0
#define CHT_PADREG1_INVRXTX_RXDATA 0x40
#define CHT_PADREG1_INTCFG_MASK    7
#define CHT_PADREG1_INTCFG_FALLING 1
#define CHT_PADREG1_INTCFG_RISING  2
#define CHT_PADREG1_INTCFG_BOTH    3
#define CHT_PADREG1_INTCFG_LEVEL   4

#define CHT_PADREG_OFFSET(pad) \
    (0x4400 + (0x400 * ((pad) / 15)) + (0x8 * ((pad) % 15)))

#define CHT_PAD(m, p)               \
{                                   \
    .mmr = (m),                     \
    .pad = (p),                     \
    .offset = CHT_PADREG_OFFSET(p), \
}

#define CHT_PADREG_0_GPIO_TX_MASK 0x2
#define CHT_PADREG_0_GPIO_RX_MASK 0x1

#define CHT_PADREG(base, offset) \
    *((volatile uint32_t *)(base + offset))

static struct up_gpio_pin_desc up_gpio_pins[] = {
	CHT_PAD(&cht_gpio_mmr_SW, 61), /*  0 */
	CHT_PAD(&cht_gpio_mmr_SW, 65), /*  1 */
	CHT_PAD(&cht_gpio_mmr_SW, 60), /*  2 */
	CHT_PAD(&cht_gpio_mmr_SW, 63), /*  3 */
	CHT_PAD(&cht_gpio_mmr_E,  21), /*  4 */
	CHT_PAD(&cht_gpio_mmr_E,  24), /*  5 */
	CHT_PAD(&cht_gpio_mmr_E,  15), /*  6 */
	CHT_PAD(&cht_gpio_mmr_SE, 79), /*  7 */
	CHT_PAD(&cht_gpio_mmr_SE,  7), /*  8 */
	CHT_PAD(&cht_gpio_mmr_SE,  3), /*  9 */
	CHT_PAD(&cht_gpio_mmr_SE,  6), /* 10 */
	CHT_PAD(&cht_gpio_mmr_SE,  4), /* 11 */
	CHT_PAD(&cht_gpio_mmr_SE,  5), /* 12 */
	CHT_PAD(&cht_gpio_mmr_SE,  1), /* 13 */
	CHT_PAD(&cht_gpio_mmr_SW, 20), /* 14 */
	CHT_PAD(&cht_gpio_mmr_SW, 16), /* 15 */
	CHT_PAD(&cht_gpio_mmr_N,   6), /* 16 */
	CHT_PAD(&cht_gpio_mmr_E,  18), /* 17 */
	CHT_PAD(&cht_gpio_mmr_SW, 31), /* 18 */
	CHT_PAD(&cht_gpio_mmr_SW, 35), /* 19 */
	CHT_PAD(&cht_gpio_mmr_SW, 33), /* 20 */
	CHT_PAD(&cht_gpio_mmr_SW, 30), /* 21 */
	CHT_PAD(&cht_gpio_mmr_N,   3), /* 22 */
	CHT_PAD(&cht_gpio_mmr_N,   2), /* 23 */
	CHT_PAD(&cht_gpio_mmr_N,   1), /* 24 */
	CHT_PAD(&cht_gpio_mmr_SW, 21), /* 25 */
	CHT_PAD(&cht_gpio_mmr_N,   7), /* 26 */
	CHT_PAD(&cht_gpio_mmr_SW, 17), /* 27 */
};	  

/************* /sys/class/gpio functions ************/
static int sysfs_gpio_export(unsigned int gpio)
{
    int fd, len;
    char str_gpio[3];
    char filename[33];

    snprintf(filename, sizeof(filename),
             "/sys/class/gpio/gpio%d/value", gpio);
    if (0 == access(filename, F_OK))
        return 0; // Already exported

    if ((fd = open("/sys/class/gpio/export", O_WRONLY)) < 0)
       return -1;

    len = snprintf(str_gpio, sizeof(str_gpio), "%d", gpio);
    if (write(fd, str_gpio, len) != len)
    {
        close(fd);
        return -1;
    }

    close(fd);

    // arbitary delay to allow udev time to set user permissions
    struct timespec delay;
    delay.tv_sec = 0;
    delay.tv_nsec = 50000000L; // 50ms
    nanosleep(&delay, NULL);

    return 0;
}

static int sysfs_gpio_unexport(unsigned int gpio)
{
    int fd, len;
    char str_gpio[3];

    if ((fd = open("/sys/class/gpio/unexport", O_WRONLY)) < 0)
        return -1;

    len = snprintf(str_gpio, sizeof(str_gpio), "%d", gpio);
    if (write(fd, str_gpio, len) != len)
    {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static int sysfs_gpio_set_direction(unsigned int gpio, unsigned int in_flag)
{
    int fd;
    char filename[33];
    char *str_dir;

    snprintf(filename, sizeof(filename),
             "/sys/class/gpio/gpio%d/direction", gpio);
    if ((fd = open(filename, O_WRONLY)) < 0)
        return -1;

    if (in_flag)
        str_dir = "in";
    else
        str_dir = "out";

    if (write(fd, str_dir, strlen(str_dir)) != strlen(str_dir))
    {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static int sysfs_gpio_get_direction(unsigned int gpio)
{
    int fd;
    char filename[33];
    char str_dir[3];
    unsigned int in_flag = 0;

    snprintf(filename, sizeof(filename),
             "/sys/class/gpio/gpio%d/direction", gpio);
    if ((fd = open(filename, O_RDONLY)) < 0)
        return -1;

    if (read(fd, str_dir, sizeof(str_dir)) > 0)
        in_flag = (str_dir[0] == 'i');

    close(fd);
    return in_flag;
}

static int sysfs_gpio_set_value(unsigned int gpio, unsigned int value)
{
    int fd;
    char filename[33];
    char *str_val;

    snprintf(filename, sizeof(filename),
             "/sys/class/gpio/gpio%d/value", gpio);
    if ((fd = open(filename, O_WRONLY)) < 0)
        return -1;

    if (value)
        str_val = "1";
    else
        str_val = "0";

    if (write(fd, str_val, strlen(str_val)) != strlen(str_val))
    {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static int sysfs_gpio_get_value(unsigned int gpio)
{
    int fd;
    char filename[33];
    char str_val[2];
    int value = 0;

    snprintf(filename, sizeof(filename),
             "/sys/class/gpio/gpio%d/value", gpio);
    if ((fd = open(filename, O_RDONLY)) < 0)
        return -1;

    if (read(fd, str_val, sizeof(str_val)) > 0)
        value = (str_val[0] == '1');

    close(fd);
    return value;
}

int setup(void)
{
    unsigned int gpio;

    for (gpio = 0; gpio < sizeof(up_gpio_pins)/sizeof(up_gpio_pins[0]); gpio++)
    {
        struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

        if (sysfs_gpio_export(gpio))
            return SETUP_EXPORT_FAIL;

        if (!(pd->mmr->virt))
        {
            if ((pd->mmr->fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
                return SETUP_DEVMEM_FAIL;

            pd->mmr->virt = mmap(NULL, pd->mmr->size, PROT_READ | PROT_WRITE,
                                MAP_SHARED, pd->mmr->fd, pd->mmr->phys);
            if (MAP_FAILED == pd->mmr->virt)
                return SETUP_MMAP_FAIL;
        }
    }

    return SETUP_OK;
}

void clear_event_detect(int gpio)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    if (pd->mmr->virt)
    {
        unsigned intr_line = CHT_PADREG(pd->mmr->virt, pd->offset);
        intr_line &= CHT_PADREG0_INTSEL_MASK;
        intr_line >>= CHT_PADREG0_INTSEL_SHIFT;
        
        CHT_PADREG(pd->mmr->virt, CHT_INTSTAT_OFFSET) = (1 << intr_line);
    }
}

int eventdetected(int gpio)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    if (pd->mmr->virt)
    {
        unsigned intr_line = CHT_PADREG(pd->mmr->virt, pd->offset);
        intr_line &= CHT_PADREG0_INTSEL_MASK;
        intr_line >>= CHT_PADREG0_INTSEL_SHIFT;
        
        return !!(CHT_PADREG(pd->mmr->virt, CHT_INTSTAT_OFFSET)
                  & (1 << intr_line));
    }
    return 0;
}

void set_rising_event(int gpio, int enable)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    uint32_t value = CHT_PADREG(pd->mmr->virt, pd->offset + 0x4);
    value &= ~(CHT_PADREG1_INTCFG_MASK | CHT_PADREG1_INVRXTX_MASK);
    value |= CHT_PADREG1_INTCFG_RISING;
    CHT_PADREG(pd->mmr->virt, pd->offset + 0x4) = value;
}

void set_falling_event(int gpio, int enable)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    uint32_t value = CHT_PADREG(pd->mmr->virt, pd->offset + 0x4);
    value &= ~(CHT_PADREG1_INTCFG_MASK | CHT_PADREG1_INVRXTX_MASK);
    value |= CHT_PADREG1_INTCFG_FALLING;
    CHT_PADREG(pd->mmr->virt, pd->offset + 0x4) = value;
}

void set_high_event(int gpio, int enable)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    uint32_t value = CHT_PADREG(pd->mmr->virt, pd->offset + 0x4);
    value &= ~(CHT_PADREG1_INTCFG_MASK | CHT_PADREG1_INVRXTX_MASK);
    value |= CHT_PADREG1_INTCFG_LEVEL;
    CHT_PADREG(pd->mmr->virt, pd->offset + 0x4) = value;
}

void set_low_event(int gpio, int enable)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    uint32_t value = CHT_PADREG(pd->mmr->virt, pd->offset + 0x4);
    value &= ~(CHT_PADREG1_INTCFG_MASK | CHT_PADREG1_INVRXTX_MASK);
    value |= (CHT_PADREG1_INTCFG_LEVEL | CHT_PADREG1_INVRXTX_RXDATA);
    CHT_PADREG(pd->mmr->virt, pd->offset + 0x4) = value;
}

void set_pullupdn(int gpio, int pud)
{
    // TODO (current board design has fixed external pull-ups)
}

void setup_gpio(int gpio, int direction, int pud)
{
    set_pullupdn(gpio, pud);
    sysfs_gpio_set_direction(gpio, (direction == INPUT));
}

int gpio_function(int gpio)
{
    return sysfs_gpio_get_direction(gpio); // 0=input, 1=output
}

void output_gpio(int gpio, int value)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    if (pd->mmr->virt)
    {
        if (value)
            CHT_PADREG(pd->mmr->virt, pd->offset) |= CHT_PADREG_0_GPIO_TX_MASK;
        else
            CHT_PADREG(pd->mmr->virt, pd->offset) &= ~CHT_PADREG_0_GPIO_TX_MASK;
    }
    else
        sysfs_gpio_set_value(gpio, value);
}

int input_gpio(int gpio)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];
    if (pd->mmr->virt)
        return !!(CHT_PADREG(pd->mmr->virt, pd->offset)
                  & CHT_PADREG_0_GPIO_RX_MASK);
    else
        return sysfs_gpio_get_value(gpio);
}

void cleanup(void)
{
    unsigned int gpio;

    for (gpio = 0; gpio < sizeof(up_gpio_pins)/sizeof(up_gpio_pins[0]); gpio++)
    {
        struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

        sysfs_gpio_unexport(gpio);

        if (pd->mmr->virt)
        {
            munmap(pd->mmr->virt, pd->mmr->size);
            close(pd->mmr->fd);
            pd->mmr->virt = NULL;
        }
    }
}
