/* Linker script for Seeed Wio Tracker L1 (Nordic nRF52840) */
/* Matches the factory Adafruit-derived UF2 bootloader with S140 v7.3.0 (app at 0x27000). */

MEMORY
{
    /*
     * FLASH layout:
     * 0x00000000 - 0x00001000 : MBR
     * 0x00001000 - 0x00027000 : SoftDevice S140 v7.3.0 (156 KB)
     * 0x00027000 - 0x000A0000 : Application Flash (~484 KB)
     * 0x000A0000 - 0x000B0000 : Config journal (sequential-storage map)
     * 0x000B0000 - 0x000B4000 : BLE bond map (sequential-storage map)
     * 0x000B4000 - 0x000ED000 : Unused / reserved (internal-storage profile
     *                           puts contacts and messages here)
     * 0x000ED000 - 0x00100000 : Bootloader code, settings and app data
     *
     * Contacts, advert blobs and messages live on the external QSPI flash.
     */
    FLASH (rx)  : ORIGIN = 0x00027000, LENGTH = 0x000A0000 - 0x00027000

    /*
     * RAM layout:
     * 0x20000000 - 0x20006000 : SoftDevice / Bootloader reserved RAM (24 KB)
     * 0x20006000 - 0x20040000 : Application RAM (232 KB)
     *
     * If the SoftDevice reports an insufficient-RAM error at enable time
     * (10 fast blinks on boot), raise the origin to 0x20008000.
     */
    RAM   (rwx) : ORIGIN = 0x20006000, LENGTH = 0x20040000 - 0x20006000
}

/*
 * External QSPI flash layout (2 MB P25Q16H, offsets within the chip).
 * These stores are not part of the MCU flash map above.
 *
 * 0x000000 - 0x040000 : Contacts + advert blobs (256 KB, keyed map)
 * 0x040000 - 0x140000 : Messages (1 MB, sequential-storage FIFO)
 * 0x140000 - 0x200000 : Unused
 */
