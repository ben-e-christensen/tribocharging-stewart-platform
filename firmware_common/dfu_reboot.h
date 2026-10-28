// dfu_reboot.h - software entry into the STM32F446's ROM DFU bootloader,
// so the Octopus can be reflashed over USB WITHOUT touching the BOOT0
// jumper or the reset button. Shared by ../octo and ../stewart_accel_cal
// (both get flashed onto the same physical board) via -I../firmware_common
// in each platformio.ini. Include from exactly ONE .cpp per project.
//
// HOW IT WORKS: the 'B' serial command calls request_dfu_reboot(), which
// stores a magic word in an RTC backup register (survives a soft reset,
// unlike normal RAM which the startup code zeroes) and resets the chip.
// On the way back up, maybe_jump_to_dfu() runs as a priority-100 static
// constructor - i.e. AFTER SystemInit() (which only sets up the FPU/vector
// table, clocks are still on the reset-default HSI) but BEFORE the
// STM32duino core's own premain() at priority 101 configures clocks, USB,
// SysTick etc. So the jump into ROM happens with the hardware still in
// essentially its just-reset state, which is what the ROM bootloader
// expects. Same approach Klipper uses for its F4 "reboot to DFU".
//
// Jumping straight to ROM from the running app (without the reset) is
// the commonly-posted alternative, but it means hand-unwinding every
// peripheral/IRQ the core turned on (USB especially) - much easier to get
// subtly wrong.
//
// RECOVERY: if this ever misbehaves, the BOOT0 jumper (J75) + reset
// method still works exactly as before - it doesn't depend on any of this.

#pragma once
#include <Arduino.h>

namespace dfu_reboot {

constexpr uint32_t MAGIC = 0xB007DF00;
constexpr uint32_t SYSMEM_BASE = 0x1FFF0000; // F446 system memory (ROM bootloader)

inline void enable_backup_access() {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR; // read-back so the clock enable lands before PWR access
    PWR->CR |= PWR_CR_DBP;
}

// Never returns.
inline void request_dfu_reboot() {
    enable_backup_access();
    // BKP19R (the last one) rather than BKP0R - the STM32RTC library uses
    // the low indices for its own bookkeeping if it ever gets pulled in.
    RTC->BKP19R = MAGIC;
    delay(50); // let the USB-CDC reply to 'B' actually leave the chip
    NVIC_SystemReset();
}

} // namespace dfu_reboot

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wprio-ctor-dtor" // 0-100 "reserved" warning; 100 is deliberate (must beat premain's 101)
__attribute__((constructor(100))) static void maybe_jump_to_dfu() {
    dfu_reboot::enable_backup_access();
    if (RTC->BKP19R != dfu_reboot::MAGIC) return;
    RTC->BKP19R = 0; // one-shot: next reset boots the app normally

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0; // alias system memory at 0x0
    SCB->VTOR = 0;

    const volatile uint32_t* sysmem = reinterpret_cast<const volatile uint32_t*>(dfu_reboot::SYSMEM_BASE);
    __set_MSP(sysmem[0]);
    reinterpret_cast<void (*)()>(sysmem[1])();
    while (true) {}
}
#pragma GCC diagnostic pop
