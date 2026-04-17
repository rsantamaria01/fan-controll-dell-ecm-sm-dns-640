/*
 * edge640fan.c - Fan speed control for Dell EMC SD-WAN Edge 640
 * Talks directly to Intel Denverton SMBus Unit 1 (0x19e0) via MMIO
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
#include <sys/mman.h>
#include <sys/types.h>
#include <time.h>

/* Intel SMBus controller MMIO register offsets */
#define SMBHSTSTS   0x00   /* Host Status */
#define SMBHSTCNT   0x02   /* Host Control */
#define SMBHSTCMD   0x03   /* Host Command */
#define SMBHSTADD   0x04   /* Host Address */
#define SMBHSTDAT0  0x05   /* Host Data 0 */
#define SMBHSTDAT1  0x06   /* Host Data 1 */
#define SMBBLKDAT   0x07   /* Block Data */

/* Host Status bits */
#define SMBHSTSTS_BYTE_DONE  0x80
#define SMBHSTSTS_INUSE      0x40
#define SMBHSTSTS_DEV_ERR    0x04
#define SMBHSTSTS_BUS_ERR    0x02
#define SMBHSTSTS_INTR       0x02
#define SMBHSTSTS_HOST_BUSY  0x01

/* Host Control bits */
#define SMBHSTCNT_START      0x40
#define SMBHSTCNT_BYTE_DATA  0x08  /* Byte data transaction */
#define SMBHSTCNT_BYTE       0x04  /* Byte transaction */

/* MAX6620 registers */
#define MAX6620_GCONFIG      0x00   /* Global config */
#define MAX6620_FAN_FAULT    0x01   /* Fan fault */
#define MAX6620_FAN1_CONFIG  0x02   /* Fan 1 config */
#define MAX6620_FAN2_CONFIG  0x03   /* Fan 2 config */
#define MAX6620_FAN1_TACH_HI 0x10  /* Fan 1 tach count high */
#define MAX6620_FAN1_TACH_LO 0x11  /* Fan 1 tach count low */
#define MAX6620_FAN2_TACH_HI 0x12  /* Fan 2 tach count high */
#define MAX6620_FAN2_TACH_LO 0x13  /* Fan 2 tach count low */
#define MAX6620_FAN1_TGT_HI  0x20  /* Fan 1 target tach high */
#define MAX6620_FAN1_TGT_LO  0x21  /* Fan 1 target tach low */
#define MAX6620_FAN2_TGT_HI  0x22  /* Fan 2 target tach high */
#define MAX6620_FAN2_TGT_LO  0x23  /* Fan 2 target tach low */

/* MAX6620 fan controller I2C address */
#define MAX6620_ADDR         0x1b

/* Second SMBus controller MMIO base from dmesg:
 * none4@pci0:0:31:5 mem 0x777ff50000 (same BAR region, different function)
 * We need to find the actual MMIO base for device 31:5
 * Primary controller (31:4) is at 0x777ff50000
 * Secondary (31:5) is typically at next BAR
 */
//#define SMB_MMIO_BASE_PRI    0x777ff50000ULL  /* Primary - device 31:4 */
#define SMB_MMIO_BASE_PRI    0xfe010000ULL  /* Secondary - device 31:5 */

/* MAX6620 tach frequency: uses 8192 Hz clock, 2 pulses per rev */
#define MAX6620_TACH_FREQ    8192
#define MAX6620_TACH_PULSES  2

static volatile uint8_t *smb_base = NULL;
static int mem_fd = -1;

static uint8_t smb_read8(uint32_t offset) {
    return smb_base[offset];
}

static void smb_write8(uint32_t offset, uint8_t val) {
    smb_base[offset] = val;
}

static int smb_wait_ready(void) {
    int timeout = 1000;
    while (timeout--) {
        uint8_t status = smb_read8(SMBHSTSTS);
        if (!(status & SMBHSTSTS_HOST_BUSY))
            return 0;
        usleep(1000);
    }
    return -1;
}

static int smb_clear_status(void) {
    smb_write8(SMBHSTSTS, 0xFF);
    usleep(500);
    return 0;
}

static int i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val) {
    if (smb_wait_ready() < 0) {
        fprintf(stderr, "SMBus busy timeout\n");
        return -1;
    }
    smb_clear_status();

    smb_write8(SMBHSTADD, (addr << 1) & 0xFE);  /* Write = bit0 clear */
    smb_write8(SMBHSTCMD, reg);
    smb_write8(SMBHSTDAT0, val);
    smb_write8(SMBHSTCNT, SMBHSTCNT_START | SMBHSTCNT_BYTE_DATA);

    if (smb_wait_ready() < 0) {
        fprintf(stderr, "SMBus transaction timeout\n");
        return -1;
    }

    uint8_t status = smb_read8(SMBHSTSTS);
    if (status & SMBHSTSTS_DEV_ERR) {
        fprintf(stderr, "SMBus device error (no ACK from 0x%02x)\n", addr);
        return -1;
    }
    return 0;
}

static int i2c_read_byte(uint8_t addr, uint8_t reg) {
    if (smb_wait_ready() < 0) {
        fprintf(stderr, "SMBus busy timeout\n");
        return -1;
    }
    smb_clear_status();

    smb_write8(SMBHSTADD, (addr << 1) & 0xFE);  /* Write phase */
    smb_write8(SMBHSTCMD, reg);
    smb_write8(SMBHSTCNT, SMBHSTCNT_START | SMBHSTCNT_BYTE_DATA);

    if (smb_wait_ready() < 0) {
        fprintf(stderr, "SMBus transaction timeout\n");
        return -1;
    }

    smb_clear_status();
    smb_write8(SMBHSTADD, ((addr << 1) | 0x01));  /* Read = bit0 set */
    smb_write8(SMBHSTCNT, SMBHSTCNT_START | SMBHSTCNT_BYTE_DATA);

    if (smb_wait_ready() < 0) {
        fprintf(stderr, "SMBus read timeout\n");
        return -1;
    }

    uint8_t status = smb_read8(SMBHSTSTS);
    if (status & SMBHSTSTS_DEV_ERR) {
        fprintf(stderr, "SMBus device error reading 0x%02x\n", addr);
        return -1;
    }

    return (int)smb_read8(SMBHSTDAT0);
}

/* Convert RPM to MAX6620 tach count */
static uint16_t rpm_to_tach(int rpm) {
    if (rpm <= 0) return 0x7FF;  /* Max value = slowest */
    /* tach_count = (TACH_FREQ * 60) / (rpm * TACH_PULSES) */
    uint32_t tach = (MAX6620_TACH_FREQ * 60) / (rpm * MAX6620_TACH_PULSES);
    if (tach > 0x7FF) tach = 0x7FF;
    return (uint16_t)tach;
}

/* Convert MAX6620 tach count to RPM */
static int tach_to_rpm(uint16_t tach) {
    if (tach == 0 || tach == 0x7FF) return 0;
    return (MAX6620_TACH_FREQ * 60) / (tach * MAX6620_TACH_PULSES);
}

static int map_smbus(uint64_t base) {
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    smb_base = (volatile uint8_t *)mmap(NULL, 0x100,
        PROT_READ | PROT_WRITE, MAP_SHARED,
        mem_fd, (off_t)base);

    if (smb_base == MAP_FAILED) {
        perror("mmap");
        close(mem_fd);
        return -1;
    }
    return 0;
}

static void unmap_smbus(void) {
    if (smb_base && smb_base != MAP_FAILED)
        munmap((void *)smb_base, 0x100);
    if (mem_fd >= 0)
        close(mem_fd);
}

static void print_status(void) {
    int gcfg = i2c_read_byte(MAX6620_ADDR, MAX6620_GCONFIG);
    int fault = i2c_read_byte(MAX6620_ADDR, MAX6620_FAN_FAULT);
    int f1hi = i2c_read_byte(MAX6620_ADDR, MAX6620_FAN1_TACH_HI);
    int f1lo = i2c_read_byte(MAX6620_ADDR, MAX6620_FAN1_TACH_LO);
    int f2hi = i2c_read_byte(MAX6620_ADDR, MAX6620_FAN2_TACH_HI);
    int f2lo = i2c_read_byte(MAX6620_ADDR, MAX6620_FAN2_TACH_LO);

    if (gcfg < 0) {
        fprintf(stderr, "Failed to read MAX6620 - wrong bus or address\n");
        return;
    }

    printf("MAX6620 Global Config: 0x%02x\n", gcfg);
    printf("Fan Fault Register:    0x%02x\n", fault);

    if (f1hi >= 0 && f1lo >= 0) {
        uint16_t tach1 = ((f1hi & 0xFF) << 3) | ((f1lo >> 5) & 0x07);
        printf("Fan 1 tach count: %d -> ~%d RPM\n", tach1, tach_to_rpm(tach1));
    }
    if (f2hi >= 0 && f2lo >= 0) {
        uint16_t tach2 = ((f2hi & 0xFF) << 3) | ((f2lo >> 5) & 0x07);
        printf("Fan 2 tach count: %d -> ~%d RPM\n", tach2, tach_to_rpm(tach2));
    }
}

static int set_fan_speed(int rpm) {
    uint16_t tach = rpm_to_tach(rpm);
    uint8_t tach_hi = (tach >> 3) & 0xFF;
    uint8_t tach_lo = (tach & 0x07) << 5;

    printf("Setting fans to ~%d RPM (tach=0x%03x hi=0x%02x lo=0x%02x)\n",
           rpm, tach, tach_hi, tach_lo);

    /* Set fan 1 target */
    if (i2c_write_byte(MAX6620_ADDR, MAX6620_FAN1_TGT_HI, tach_hi) < 0) return -1;
    if (i2c_write_byte(MAX6620_ADDR, MAX6620_FAN1_TGT_LO, tach_lo) < 0) return -1;

    /* Set fan 2 target */
    if (i2c_write_byte(MAX6620_ADDR, MAX6620_FAN2_TGT_HI, tach_hi) < 0) return -1;
    if (i2c_write_byte(MAX6620_ADDR, MAX6620_FAN2_TGT_LO, tach_lo) < 0) return -1;

    printf("Done.\n");
    return 0;
}

int main(int argc, char *argv[]) {
    int rpm = 0;

    if (argc < 2) {
        printf("Usage: %s <rpm>    (0 = status only)\n", argv[0]);
        printf("Example: %s 3000\n", argv[0]);
        return 1;
    }

    rpm = atoi(argv[1]);

    /* Try primary SMBus MMIO base first, then offset for secondary */
    uint64_t bases[] = {
        0xfe010000ULL,
        0xfe010000ULL + 0x100,
        0xfe010000ULL + 0x200,
    };

    int found = 0;
    for (int i = 0; i < 3; i++) {
        printf("Trying MMIO base 0x%llx...\n", (unsigned long long)bases[i]);
        if (map_smbus(bases[i]) < 0)
            continue;

        /* Quick probe - try to read MAX6620 global config */
        int gcfg = i2c_read_byte(MAX6620_ADDR, MAX6620_GCONFIG);
        if (gcfg >= 0) {
            printf("Found MAX6620 at MMIO base 0x%llx\n", (unsigned long long)bases[i]);
            found = 1;
            break;
        }
        unmap_smbus();
    }

    if (!found) {
        fprintf(stderr, "MAX6620 not found on any SMBus MMIO base\n");
        fprintf(stderr, "Try running: pciconf -lv | grep -A3 19e0\n");
        fprintf(stderr, "to find correct MMIO address and recompile with correct SMB_MMIO_BASE\n");
        return 1;
    }

    print_status();

    if (rpm > 0) {
        set_fan_speed(rpm);
        sleep(2);
        printf("\nUpdated status:\n");
        print_status();
    }

    unmap_smbus();
    return 0;
}