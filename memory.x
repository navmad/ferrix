/* Linker script for Elecrow ThinkNode M1 (Nordic nRF52840) */
/* Standard layout matching the factory Adafruit UF2 Bootloader (flash offset 0x26000) */

MEMORY
{
    /* 
     * FLASH layout:
     * 0x00000000 - 0x00001000 : MBR
     * 0x00001000 - 0x00026000 : SoftDevice / Bootloader reserved area
     * 0x00026000 - 0x000A0000 : Application Flash (~500 KB)
     * 0x000A0000 - 0x000B0000 : Persistent advert blobs (append log)
     * 0x000B0000 - 0x000CC000 : Persistent contacts (two 56 KB banks)
     * 0x000CC000 - 0x000EC000 : Persistent messages (two 64 KB banks)
     * 0x000EC000 - 0x000ED000 : Persistent configuration page (4 KB)
     * 0x000ED000 - 0x00100000 : Bootloader code & settings (~76 KB)
     */
    FLASH (rx)  : ORIGIN = 0x00026000, LENGTH = 0x000A0000 - 0x00026000

    /* 
     * RAM layout:
     * 0x20000000 - 0x20006000 : SoftDevice / Bootloader reserved RAM (24 KB)
     * 0x20006000 - 0x20040000 : Application RAM (232 KB)
     */
    RAM   (rwx) : ORIGIN = 0x20006000, LENGTH = 0x20040000 - 0x20006000
}

/* 
 * NOTE: If flashing baremetal via SWD (e.g. J-Link / CMSIS-DAP) with full chip erase:
 * FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 1024K
 * RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
 */

