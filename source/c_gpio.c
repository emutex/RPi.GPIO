/*
Copyright (c) 2012-2015 Ben Croston

Adapted for use on UP board (http://www.up-board.org/)
Copyright (c) 2016 Emutex Ltd
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

#define N_GPIO 28

struct up_gpio_pin_desc {
    int val_fd;
    int dir_fd;
    int initialised;
};

static struct up_gpio_pin_desc up_gpio_pins[N_GPIO];

static int export_fd = -1;

/************* /sys/class/gpio functions ************/

int setup(void)
{
    if ((export_fd = open("/sys/class/gpio/export", O_WRONLY)) < 0)
       return SETUP_EXPORT_FAIL;

    return SETUP_OK;
}

int init_gpio(int gpio)
{
    int len;
    char str_gpio[3];
    char filename[33];
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

    snprintf(filename, sizeof(filename),
             "/sys/class/gpio/gpio%d/value", gpio);
    if (0 != access(filename, F_OK))
    {
        len = snprintf(str_gpio, sizeof(str_gpio), "%d", gpio);
        lseek(export_fd, 0, SEEK_SET);
        if (write(export_fd, str_gpio, len) != len)
            return SETUP_EXPORT_FAIL;
    }

    // arbitary delay to allow udev time to set user permissions
    struct timespec delay;
    delay.tv_sec = 0;
    delay.tv_nsec = 50000000L; // 50ms
    nanosleep(&delay, NULL);

    snprintf(filename, sizeof(filename),
             "/sys/class/gpio/gpio%d/direction", gpio);
    if ((pd->dir_fd = open(filename, O_RDWR | O_SYNC)) < 0)
        return SETUP_EXPORT_FAIL;

    snprintf(filename, sizeof(filename),
        "/sys/class/gpio/gpio%d/value", gpio);
    if ((pd->val_fd = open(filename, O_RDWR | O_SYNC)) < 0)
        return SETUP_EXPORT_FAIL;

    pd->initialised = 1;

    return SETUP_OK;
}

/*
 * NOTE - pull up/down resistors can not be configured on
 * on the external I/O pins on the UP board, so the pud
 * parameter below is ignored
 */
void setup_gpio(int gpio, int direction, int pud)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

    if (!pd->initialised)
        init_gpio(gpio);

    if (pd->dir_fd >= 0)
    {
        lseek(pd->dir_fd, 0, SEEK_SET);
        if (direction == INPUT)
            write(pd->dir_fd, "in", 2);
        else
            write(pd->dir_fd, "out", 3);
    }
}

int gpio_function(int gpio)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

    if (!pd->initialised)
        init_gpio(gpio);

    if (pd->dir_fd >= 0)
    {
        char str_dir[3];

        lseek(pd->dir_fd, 0, SEEK_SET);
        if (read(pd->dir_fd, str_dir, sizeof(str_dir)) > 0)
            return (str_dir[0] == 'i') ? 0 : 1;
    }
    /*
     * TODO - use pinctrl debugfs information to see if the
     * pin is currently in GPIO mode or an alternative mode?
     * Avoid if it requires root access.
     */

    return 0;
}

void output_gpio(int gpio, int value)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

    if (!pd->initialised)
        init_gpio(gpio);

    if (pd->val_fd >= 0)
    {
        lseek(pd->val_fd, 0, SEEK_SET);
        write(pd->val_fd, value ? "1" : "0", 1);
    }
}

int input_gpio(int gpio)
{
    struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

    if (!pd->initialised)
        init_gpio(gpio);

    if (pd->val_fd >= 0)
    {
        char str_val[2];

        lseek(pd->val_fd, 0, SEEK_SET);
        if (read(pd->val_fd, str_val, sizeof(str_val)) > 0)
            return (str_val[0] == '1');
    }

    return 0;
}

void cleanup(void)
{
    unsigned int gpio;
    int unexport_fd;

    close(export_fd);
    export_fd = -1;

    if ((unexport_fd = open("/sys/class/gpio/unexport", O_WRONLY)) < 0)
       return;

    for (gpio = 0; gpio < sizeof(up_gpio_pins)/sizeof(up_gpio_pins[0]); gpio++)
    {
        int len;
        char str_gpio[3];
        struct up_gpio_pin_desc *pd = &up_gpio_pins[gpio];

        if (pd->initialised)
        {
            pd->initialised = 0;

            if (pd->dir_fd >= 0)
                close(pd->dir_fd);

            if (pd->val_fd >= 0)
                close(pd->val_fd);

            len = snprintf(str_gpio, sizeof(str_gpio), "%d", gpio);
            lseek(unexport_fd, 0, SEEK_SET);
            if (write(unexport_fd, str_gpio, len) != len)
            {
                close(unexport_fd);
                return;
            }
        }
    }

    close(unexport_fd);
}
