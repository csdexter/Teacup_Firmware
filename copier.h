#ifndef  _COPIER_H
#define  _COPIER_H

// operation instructions
#define    CMD_PROGRAMMING_ENABLE        0xAC530000
#define    CMD_CHIP_ERASE                0xAC800000
#define    CMD_POLL                      0xF0000000

// load instructions
#define    CMD_LOAD_EXTENDED_ADDRESS      0x4D000000
#define    CMD_LOAD_PROGMEM_HIGH_BYTE    0x48000000
#define    CMD_LOAD_PROGMEM_LOW_BYTE      0x40000000
#define    CMD_LOAD_EEPROM_PAGE          0xC1000000

// read instructions
#define    CMD_READ_PROGMEM_HIGH_BYTE    0x28000000
#define    CMD_READ_PROGMEM_LOW_BYTE      0x20000000
#define    CMD_READ_EEPROM                0xA0000000
#define    CMD_READ_LOCK_BITS            0x58000000
#define    CMD_READ_SIGNATURE            0x30000000
#define    CMD_READ_FUSE_BITS            0x50000000
#define    CMD_READ_FUSE_HIGH_BITS        0x58080000
#define    CMD_READ_FUSE_EXTENDED_BITS    0x50080000
#define    CMD_READ_CALIBRATION_BYTE      0x38000000

// write instructions
#define    CMD_WRITE_PROGMEM_PAGE        0x4C000000
#define    CMD_WRITE_EEPROM              0xC0000000
#define    CMD_WRITE_EEPROM_PAGE          0xC2000000
#define    CMD_WRITE_LOCK_BITS            0xACE00000
#define    CMD_WRITE_FUSE_BITS            0xACA00000
#define    CMD_WRITE_FUSE_HIGH_BITS      0xACA80000
#define    CMD_WRITE_FUSE_EXTENDED_BITS  0xACA40000

//pinout
#define    COPIER_RESET      AIO1
#define    COPIER_SCK        AIO2
#define    COPIER_MOSI        AIO3
#define    COPIER_MISO        AIO4

//functions

void copy(void);

#endif  /* _COPIER_H */
