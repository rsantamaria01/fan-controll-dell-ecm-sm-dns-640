/*
 * edge640fan.c - Fan speed control for Dell EMC SD-WAN Edge 640
 * Talks directly to Intel iSMT SMBus 2.0 controller (0x19ac) via MMIO
 * to reach MAX6620 fan controller at I2C address 0x1b
 *
 * iSMT uses descriptor-based transactions, not simple register writes.
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

/* iSMT MMIO register offsets */
#define ISMT_GR_GCTRL       0x000   /* General Control */
#define ISMT_GR_SMTICL      0x008   /* SMT Interrupt Cause Location */
#define ISMT_GR_ERRINTMSK   0x010   /* Error Interrupt Mask */
#define ISMT_GR_ERRSTS       0x018   /* Error Status */
#define ISMT_GR_DMASTSA     0x020   /* DMA Status */
#define ISMT_MSTR_MDBA      0x100   /* Master Descriptor Base Address low */
#define ISMT_MSTR_MDBA_HI   0x104   /* Master Descriptor Base Address high */
#define ISMT_MSTR_MCTRL     0x108   /* Master Control */
#define ISMT_MSTR_MSTS      0x10C   /* Master Status */
#define ISMT_MSTR_MDS       0x110   /* Master Descriptor Size */
#define ISMT_MSTR_RPOLICY   0x114   /* Retry Policy */

/* General Control bits */
#define ISMT_GCTRL_SRST     (1 << 1)  /* Soft Reset */

/* Master Control bits */
#define ISMT_MCTRL_SS       (1 << 0)  /* Start/Stop */
#define ISMT_MCTRL_MEIE     (1 << 1)  /* Master Error Interrupt Enable */
#define ISMT_MCTRL_FMHP     (0 << 2)  /* Fetch Mode: Head Pointer */

/* Master Status bits */
#define ISMT_MSTS_HMTP      0x00     /* Head/Tail pointer */

/* Descriptor control bits */
#define ISMT_DESC_CWRL      (1 << 1)  /* Command with Read/Write Length */
#define ISMT_DESC_BREAD     (1 << 2)  /* Byte Read */
#define ISMT_DESC_BWRITE    (0 << 2)  /* Byte Write */
#define ISMT_DESC_I2C       (1 << 5)  /* I2C vs SMBus mode */
#define ISMT_DESC_INT       (1 << 6)  /* Interrupt on completion */
#define ISMT_DESC_SOE       (1 << 7)  /* Stop on Error */

/* Descriptor status bits */
#define ISMT_DESC_SCS       (1 << 0)  /* Success */
#define ISMT_DESC_NAK       (1 << 3)  /* NAK received */
#define ISMT_DESC_CRC       (1 << 4)  /* CRC error */

#define ISMT_DESC_ADDR_RW(addr, rw)  (((addr) << 1) | (rw))

/* iSMT descriptor structure (32 bytes) */
struct ismt_desc {
    uint8_t  tgtaddr_rw;   /* Target address + R/W bit */
    uint8_t  wr_len_cmd;   /* Write length or command code */
    uint8_t  rd_len;       /* Read length */
    uint8_t  control;      /* Control bits */
    uint8_t  status;       /* Status (written by hardware) */
    uint8_t  retry;        /* Retry count */
    uint8_t  rxbytes;      /* Received bytes count */
    uint8_t  txbytes;      /* Transmitted bytes count */
    uint64_t dma_buffer;   /* DMA buffer physical address */
    uint8_t  reserved[16]; /* Padding to 32 bytes */
} __attribute__((packed));

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

#define MAX6620_ADDR         0x1b

/* iSMT MMIO base for pci0:0:18:0 */
#define ISMT_MMIO_BASE       0x777ff54000ULL

/* Tach constants */
#define TACH_FREQ    8192
#define TACH_PULSES  2

static volatile uint8_t  *ismt_base = NULL;
static struct ismt_desc  *desc_ring = NULL;
static uint8_t           *dma_buf   = NULL;
static int                mem_fd    = -1;

/* Map a physical address */
static volatile void *map_phys(int fd, uint64_t base, size_t size) {
    volatile void *p = mmap(NULL, size,
        PROT_READ | PROT_WRITE, MAP_SHARED,
        fd, (off_t)base);
    if (p == MAP_FAILED) {
        perror("mmap");
        return NULL;
    }
    return p;
}

static uint32_t ismt_read32(uint32_t off) {
    return *(volatile uint32_t *)(ismt_base + off);
}

static void ismt_write32(uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(ismt_base + off) = val;
}

static uint64_t ismt_read64(uint32_t off) {
    return *(volatile uint64_t *)(ismt_base + off);
}

static void ismt_write64(uint32_t off, uint64_t val) {
    *(volatile uint64_t *)(ismt_base + off) = val;
}

static int ismt_init(void) {
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    ismt_base = map_phys(mem_fd, ISMT_MMIO_BASE, 0x200);
    if (!ismt_base) return -1;

    /* Allocate descriptor ring (1 descriptor = 32 bytes) and DMA buf */
    /* Use mmap anonymous for descriptor + data buffer */
    desc_ring = (struct ismt_desc *)map_phys(mem_fd, 0x10000, 4096);
    if (desc_ring == MAP_FAILED) {
        perror("mmap desc_ring");
        return -1;
    }
    memset(desc_ring, 0, 4096);
    dma_buf = (uint8_t *)desc_ring + 256; /* DMA buffer after descriptor */

    /* Get physical address of descriptor ring via /proc/self/pagemap trick
     * On FreeBSD use contigmalloc equivalent - for now use virtual addr
     * NOTE: this will only work if VA=PA (identity mapped) which is
     * NOT guaranteed. For a proper implementation we'd need a kernel module.
     * Try it anyway - worst case transactions will fail silently.
     */

    /* Soft reset */
    ismt_write32(ISMT_GR_GCTRL, ISMT_GCTRL_SRST);
    usleep(10000);
    ismt_write32(ISMT_GR_GCTRL, 0);
    usleep(10000);

    printf("iSMT GCTRL:  0x%08x\n", ismt_read32(ISMT_GR_GCTRL));
    printf("iSMT MCTRL:  0x%08x\n", ismt_read32(ISMT_MSTR_MCTRL));
    printf("iSMT MSTS:   0x%08x\n", ismt_read32(ISMT_MSTR_MSTS));
    printf("iSMT MDS:    0x%08x\n", ismt_read32(ISMT_MSTR_MDS));

    return 0;
}

static void ismt_cleanup(void) {
    if (desc_ring && desc_ring != MAP_FAILED)
        munmap(desc_ring, 4096);
    if (ismt_base && ismt_base != MAP_FAILED)
        munmap((void *)ismt_base, 0x200);
    if (mem_fd >= 0)
        close(mem_fd);
}

/*
 * Simple SMBus byte write via iSMT
 * Sets up descriptor and polls for completion
 */
static int ismt_smbus_write_byte(uint8_t addr, uint8_t cmd, uint8_t val) {
    /* Setup DMA buffer: [cmd_byte, data_byte] */
    dma_buf[0] = cmd;
    dma_buf[1] = val;

    /* Get physical address - FreeBSD vtophys equivalent */
    /* We'll use the virtual address directly and hope for the best
     * on a system with direct mapping */
    uint64_t dma_phys = 0x10000 + 256;

    /* Setup descriptor */
    memset(desc_ring, 0, sizeof(struct ismt_desc));
    desc_ring->tgtaddr_rw = ISMT_DESC_ADDR_RW(addr, 0); /* Write */
    desc_ring->wr_len_cmd = 2;     /* cmd + 1 data byte */
    desc_ring->rd_len     = 0;
    desc_ring->control    = ISMT_DESC_SOE;
    desc_ring->status     = 0;
    desc_ring->dma_buffer = dma_phys;

    /* Set descriptor base address */
    uint64_t desc_phys = 0x10000;
    ismt_write64(ISMT_MSTR_MDBA, desc_phys);
    ismt_write32(ISMT_MSTR_MDS, 1); /* 1 descriptor */

    /* Start transaction */
    ismt_write32(ISMT_MSTR_MCTRL, ISMT_MCTRL_SS);

    /* Poll for completion */
    int timeout = 1000;
    while (timeout--) {
        usleep(1000);
        if (desc_ring->status & (ISMT_DESC_SCS | ISMT_DESC_NAK))
            break;
    }

    if (timeout <= 0) {
        fprintf(stderr, "iSMT write timeout addr=0x%02x cmd=0x%02x\n", addr, cmd);
        return -1;
    }

    if (desc_ring->status & ISMT_DESC_NAK) {
        fprintf(stderr, "iSMT NAK addr=0x%02x cmd=0x%02x\n", addr, cmd);
        return -1;
    }

    return 0;
}

static int ismt_smbus_read_byte(uint8_t addr, uint8_t cmd) {
    dma_buf[0] = cmd;
    memset(dma_buf + 1, 0, 1);

    uint64_t dma_phys = 0x10000 + 256;

    memset(desc_ring, 0, sizeof(struct ismt_desc));
    desc_ring->tgtaddr_rw = ISMT_DESC_ADDR_RW(addr, 0); /* Write cmd first */
    desc_ring->wr_len_cmd = 1;    /* command byte */
    desc_ring->rd_len     = 1;    /* read 1 byte back */
    desc_ring->control    = ISMT_DESC_SOE;
    desc_ring->status     = 0;
    desc_ring->dma_buffer = dma_phys;

    uint64_t desc_phys = 0x10000;
    ismt_write64(ISMT_MSTR_MDBA, desc_phys);
    ismt_write32(ISMT_MSTR_MDS, 1);
    ismt_write32(ISMT_MSTR_MCTRL, ISMT_MCTRL_SS);

    int timeout = 1000;
    while (timeout--) {
        usleep(1000);
        if (desc_ring->status & (ISMT_DESC_SCS | ISMT_DESC_NAK))
            break;
    }

    printf("DEBUG desc status=0x%02x rxbytes=%d\n", desc_ring->status, desc_ring->rxbytes);
    printf("DEBUG MSTS=0x%08x MCTRL=0x%08x\n", ismt_read32(ISMT_MSTR_MSTS), ismt_read32(ISMT_MSTR_MCTRL));

    if (timeout <= 0) {
        fprintf(stderr, "iSMT read timeout addr=0x%02x cmd=0x%02x\n", addr, cmd);
        return -1;
    }

    if (desc_ring->status & ISMT_DESC_NAK) {
        fprintf(stderr, "iSMT NAK addr=0x%02x cmd=0x%02x\n", addr, cmd);
        return -1;
    }

    return (int)dma_buf[1];
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
    int gcfg  = ismt_smbus_read_byte(MAX6620_ADDR, MAX6620_GCONFIG);
    int fault = ismt_smbus_read_byte(MAX6620_ADDR, MAX6620_FAN_FAULT);
    int f1hi  = ismt_smbus_read_byte(MAX6620_ADDR, MAX6620_FAN1_TACH_HI);
    int f1lo  = ismt_smbus_read_byte(MAX6620_ADDR, MAX6620_FAN1_TACH_LO);
    int f2hi  = ismt_smbus_read_byte(MAX6620_ADDR, MAX6620_FAN2_TACH_HI);
    int f2lo  = ismt_smbus_read_byte(MAX6620_ADDR, MAX6620_FAN2_TACH_LO);

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

    if (ismt_smbus_write_byte(MAX6620_ADDR, MAX6620_GCONFIG, 0x08) < 0) return -1;
    usleep(10000);
    if (ismt_smbus_write_byte(MAX6620_ADDR, MAX6620_FAN1_CONFIG, 0x08) < 0) return -1;
    if (ismt_smbus_write_byte(MAX6620_ADDR, MAX6620_FAN2_CONFIG, 0x08) < 0) return -1;
    if (ismt_smbus_write_byte(MAX6620_ADDR, MAX6620_FAN1_TGT_HI, tach_hi) < 0) return -1;
    if (ismt_smbus_write_byte(MAX6620_ADDR, MAX6620_FAN1_TGT_LO, tach_lo) < 0) return -1;
    if (ismt_smbus_write_byte(MAX6620_ADDR, MAX6620_FAN2_TGT_HI, tach_hi) < 0) return -1;
    if (ismt_smbus_write_byte(MAX6620_ADDR, MAX6620_FAN2_TGT_LO, tach_lo) < 0) return -1;

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

    printf("Initializing iSMT controller at 0x%llx...\n",
           (unsigned long long)ISMT_MMIO_BASE);

    if (ismt_init() < 0)
        return 1;

    printf("\n--- Current status ---\n");
    print_status();

    if (rpm > 0) {
        printf("\n--- Setting speed ---\n");
        set_fan_speed(rpm);
        sleep(2);
        printf("\n--- Updated status ---\n");
        print_status();
    }

    ismt_cleanup();
    return 0;
}