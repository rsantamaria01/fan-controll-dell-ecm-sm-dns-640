/*
 * edge640fan.c - Fan speed control for Dell EMC SD-WAN Edge 640
 * Uses FreeBSD SMBus ioctl interface via /dev/smb0
 * to reach MAX6620 fan controller at I2C address 0x1b
 *
 * Usage: edge640fan <rpm>
 *   rpm: target fan speed (e.g. 3000, 4000, 5000)
 *   rpm=0: read current status only
 *
 * Compile: cc edge640fan.c -o edge640fan
 * Run:     ./edge640fan 3000
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <dev/smbus/smb.h>

/* MAX6620 registers */
#define MAX6620_GCONFIG      0x00
#define MAX6620_FAN_FAULT    0x01
#define MAX6620_FAN1_CONFIG  0x02
#define MAX6620_FAN2_CONFIG  0x03
#define MAX6620_FAN1_TACH_HI 0x10
#define MAX6620_FAN1_TACH_LO 0x11
#define MAX6620_FAN2_TACH_HI 0x12
#define MAX6620_FAN2_TACH_LO 0x13
#define MAX6620_FAN1_TGT_HI  0x20
#define MAX6620_FAN1_TGT_LO  0x21
#define MAX6620_FAN2_TGT_HI  0x22
#define MAX6620_FAN2_TGT_LO  0x23

/* MAX6620 I2C address — shifted left 1 for FreeBSD SMBus */
#define MAX6620_ADDR         0x1b
#define MAX6620_ADDR_W       (MAX6620_ADDR << 1)
#define MAX6620_ADDR_R       ((MAX6620_ADDR << 1) | 1)

/* MAX6620 tach clock = 8192 Hz, 2 pulses per rev */
#define TACH_FREQ   8192
#define TACH_PULSES 2

static int smb_fd = -1;

static int smb_open(const char *dev) {
    smb_fd = open(dev, O_RDWR);
    if (smb_fd < 0) {
        perror("open smb device");
        return -1;
    }
    return 0;
}

static void smb_close(void) {
    if (smb_fd >= 0)
        close(smb_fd);
}

static int i2c_write_byte(uint8_t reg, uint8_t val) {
    struct smbcmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.slave = MAX6620_ADDR_W;
    cmd.cmd   = reg;
    cmd.wdata.byte = val;
    if (ioctl(smb_fd, SMB_WRITEB, &cmd) < 0) {
        fprintf(stderr, "SMB_WRITEB reg=0x%02x val=0x%02x: %s\n",
                reg, val, strerror(errno));
        return -1;
    }
    return 0;
}

static int i2c_read_byte(uint8_t reg) {
    struct smbcmd cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.slave = MAX6620_ADDR_W;
    cmd.cmd   = reg;
    if (ioctl(smb_fd, SMB_READB, &cmd) < 0) {
        fprintf(stderr, "SMB_READB reg=0x%02x: %s\n", reg, strerror(errno));
        return -1;
    }
    return (uint8_t)cmd.rdata.byte;
}

static uint16_t rpm_to_tach(int rpm) {
    if (rpm <= 0) return 0x7FF;
    uint32_t tach = (TACH_FREQ * 60) / (rpm * TACH_PULSES);
    if (tach > 0x7FF) tach = 0x7FF;
    return (uint16_t)tach;
}

static int tach_to_rpm(uint16_t tach) {
    if (tach == 0 || tach >= 0x7FF) return 0;
    return (TACH_FREQ * 60) / (tach * TACH_PULSES);
}

static void print_status(void) {
    int gcfg  = i2c_read_byte(MAX6620_GCONFIG);
    int fault = i2c_read_byte(MAX6620_FAN_FAULT);
    int f1hi  = i2c_read_byte(MAX6620_FAN1_TACH_HI);
    int f1lo  = i2c_read_byte(MAX6620_FAN1_TACH_LO);
    int f2hi  = i2c_read_byte(MAX6620_FAN2_TACH_HI);
    int f2lo  = i2c_read_byte(MAX6620_FAN2_TACH_LO);

    if (gcfg < 0) {
        fprintf(stderr, "Failed to read MAX6620\n");
        return;
    }

    printf("Global Config:      0x%02x\n", gcfg);
    printf("Fan Fault Register: 0x%02x\n", fault);

    if (f1hi >= 0 && f1lo >= 0) {
        uint16_t tach = ((f1hi & 0xFF) << 3) | ((f1lo >> 5) & 0x07);
        printf("Fan 1 tach: %4d -> ~%d RPM\n", tach, tach_to_rpm(tach));
    }
    if (f2hi >= 0 && f2lo >= 0) {
        uint16_t tach = ((f2hi & 0xFF) << 3) | ((f2lo >> 5) & 0x07);
        printf("Fan 2 tach: %4d -> ~%d RPM\n", tach, tach_to_rpm(tach));
    }
}

static int set_fan_speed(int rpm) {
    uint16_t tach    = rpm_to_tach(rpm);
    uint8_t  tach_hi = (tach >> 3) & 0xFF;
    uint8_t  tach_lo = (tach & 0x07) << 5;

    printf("Setting fans to ~%d RPM (tach=0x%03x hi=0x%02x lo=0x%02x)\n",
           rpm, tach, tach_hi, tach_lo);

    /* Enable SMBus speed control mode */
    if (i2c_write_byte(MAX6620_GCONFIG, 0x08) < 0) return -1;
    usleep(10000);

    /* Fan 1 config: enable tach-based speed control */
    if (i2c_write_byte(MAX6620_FAN1_CONFIG, 0x08) < 0) return -1;
    /* Fan 2 config: enable tach-based speed control */
    if (i2c_write_byte(MAX6620_FAN2_CONFIG, 0x08) < 0) return -1;

    /* Set fan 1 target tach */
    if (i2c_write_byte(MAX6620_FAN1_TGT_HI, tach_hi) < 0) return -1;
    if (i2c_write_byte(MAX6620_FAN1_TGT_LO, tach_lo) < 0) return -1;

    /* Set fan 2 target tach */
    if (i2c_write_byte(MAX6620_FAN2_TGT_HI, tach_hi) < 0) return -1;
    if (i2c_write_byte(MAX6620_FAN2_TGT_LO, tach_lo) < 0) return -1;

    printf("Done.\n");
    return 0;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <rpm>    (0 = status only)\n", argv[0]);
        printf("Example: %s 3000\n", argv[0]);
        return 1;
    }

    int rpm = atoi(argv[1]);

    if (smb_open("/dev/smb0") < 0)
        return 1;

    printf("--- Current status ---\n");
    print_status();

    if (rpm > 0) {
        printf("\n--- Setting speed ---\n");
        if (set_fan_speed(rpm) < 0) {
            fprintf(stderr, "Failed to set fan speed\n");
            smb_close();
            return 1;
        }
        sleep(2);
        printf("\n--- Updated status ---\n");
        print_status();
    }

    smb_close();
    return 0;
}