/* Linker script for the nrf52840_dk board (battery-free OpenWSN fork).
 *
 * GCC/ld adaptation of the SEGGER Embedded Studio-generated placement used by
 * this fork (see projects/nrf52840_dk/Output/Debug/Obj/03oos_openwsn/03oos_openwsn.ld
 * and flash_placement.xml). It keeps the SES section/symbol conventions expected
 * by the SDK startup files shipped in sdk/:
 *   - sdk/ses_startup_nrf52840.s places the vector table in section .vectors and
 *     default handlers + nRFInitialize in section .init
 *   - sdk/ses_startup_nrf_common.s implements Reset_Handler (section .init) and
 *     branches to _start
 *   - sdk/thumb_crt0.s implements _start (section .init) and consumes the
 *     __stack_*__, __data_load_start__/__data_start__, __bss_*__, __heap_*__,
 *     __ctors_*__, __dtors_*__, __fast_*__ and __tdata/tbss_*__ symbols below
 *
 * Memory layout (nRF52840 QIAAA, 1MB flash, 256KB RAM), matches
 * linker_section_placement_macros in nrf52840_dk.emProject:
 *   FLASH 0x0-0x100000, RAM 0x20000000-0x20040000, stack 8192, heap 8192
 */

MEMORY
{
  FLASH1 (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00100000
  RAM1   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00040000
}

__FLASH1_segment_start__ = 0x00000000;
__FLASH1_segment_end__   = 0x00100000;
__FLASH1_segment_size__  = 0x00100000;
__RAM1_segment_start__   = 0x20000000;
__RAM1_segment_end__     = 0x20040000;
__RAM1_segment_size__    = 0x00040000;

__HEAPSIZE__          = 8192;   /* matches arm_linker_heap_size in .emProject */
__STACKSIZE__         = 8192;   /* matches arm_linker_stack_size in .emProject */
__STACKSIZE_PROCESS__ = 0;

SECTIONS
{
  /* ---- vector table at flash start, entry point is Reset_Handler (in .init) */
  __vectors_load_start__ = 0x0;
  .vectors 0x0 : AT(0x0)
  {
    __vectors_start__ = .;
    KEEP(*(.vectors .vectors.*))
    __vectors_end__ = __vectors_start__ + SIZEOF(.vectors);
  }
  __vectors_size__ = SIZEOF(.vectors);

  /* ---- reset/startup code (.init holds Reset_Handler, nRFInitialize, _start) */
  __init_load_start__ = ALIGN(__vectors_end__, 4);
  .init ALIGN(__vectors_end__, 4) : AT(ALIGN(__vectors_end__, 4))
  {
    __init_start__ = .;
    KEEP(*(.init .init.*))
    __init_end__ = __init_start__ + SIZEOF(.init);
  }
  __init_size__ = SIZEOF(.init);

  /* ---- code */
  __text_load_start__ = ALIGN(__init_end__, 4);
  .text ALIGN(__init_end__, 4) : AT(ALIGN(__init_end__, 4))
  {
    __text_start__ = .;
    *(.text .text.*)
    __text_end__ = __text_start__ + SIZEOF(.text);
  }
  __text_size__ = SIZEOF(.text);

  /* ---- C++ (unused in C build, symbols kept for thumb_crt0.s) */
  __dtors_load_start__ = ALIGN(__text_end__, 4);
  .dtors ALIGN(__text_end__, 4) : AT(ALIGN(__text_end__, 4))
  {
    __dtors_start__ = .;
    KEEP(*(.dtors .dtors.*))
    __dtors_end__ = __dtors_start__ + SIZEOF(.dtors);
  }

  __ctors_load_start__ = ALIGN(__dtors_end__, 4);
  .ctors ALIGN(__dtors_end__, 4) : AT(ALIGN(__dtors_end__, 4))
  {
    __ctors_start__ = .;
    KEEP(*(.ctors .ctors.*))
    __ctors_end__ = __ctors_start__ + SIZEOF(.ctors);
  }

  /* ---- read-only data */
  __rodata_load_start__ = ALIGN(__ctors_end__, 4);
  .rodata ALIGN(__ctors_end__, 4) : AT(ALIGN(__ctors_end__, 4))
  {
    __rodata_start__ = .;
    *(.rodata .rodata.*)
    __rodata_end__ = __rodata_start__ + SIZEOF(.rodata);
  }

  /* ---- ARM exception unwind tables */
  __ARM_exidx_load_start__ = ALIGN(__rodata_end__, 4);
  .ARM.exidx ALIGN(__rodata_end__, 4) : AT(ALIGN(__rodata_end__, 4))
  {
    __ARM_exidx_start__ = .;
    *(.ARM.exidx .ARM.exidx.*)
    __ARM_exidx_end__ = .;
  }
  __exidx_start__ = __ARM_exidx_start__;
  __exidx_end__   = __ARM_exidx_end__;

  /* ---- initialized data: runs from RAM, load image in FLASH */
  __data_load_start__ = ALIGN(__ARM_exidx_end__, 4);
  .data 0x20000000 : AT(ALIGN(__ARM_exidx_end__, 4))
  {
    __data_start__ = .;
    *(.data .data.*)
    __data_end__ = .;
  }
  __data_load_end__ = __data_load_start__ + SIZEOF(.data);

  /* ---- fast code/data copied to RAM at boot (section exists empty in C builds) */
  __fast_load_start__ = ALIGN(__data_load_end__, 4);
  .fast ALIGN(__data_end__, 4) : AT(ALIGN(__data_load_end__, 4))
  {
    __fast_start__ = .;
    *(.fast .fast.*)
    __fast_end__ = .;
  }
  __fast_load_end__ = __fast_load_start__ + SIZEOF(.fast);

  /* ---- TLS (empty in C builds, symbols consumed by thumb_crt0.s) */
  __tdata_load_start__ = ALIGN(__fast_load_end__, 4);
  .tdata ALIGN(__fast_end__, 4) : AT(ALIGN(__fast_load_end__, 4))
  {
    __tdata_start__ = .;
    *(.tdata .tdata.*)
    __tdata_end__ = .;
  }

  __FLASH1_segment_used_end__ = ALIGN(__tdata_load_start__, 4) + SIZEOF(.tdata);

  /* ---- zero-initialized data */
  .tbss ALIGN(__tdata_end__, 4) (NOLOAD) :
  {
    __tbss_start__ = .;
    *(.tbss .tbss.*)
    __tbss_end__ = .;
  }

  .bss ALIGN(__tbss_end__, 4) (NOLOAD) :
  {
    __bss_start__ = .;
    *(.bss .bss.* COMMON)
    __bss_end__ = .;
  }

  .non_init ALIGN(__bss_end__, 4) (NOLOAD) :
  {
    __non_init_start__ = .;
    *(.non_init .non_init.*)
    __non_init_end__ = .;
  }

  /* ---- heap */
  .heap ALIGN(__non_init_end__, 4) (NOLOAD) :
  {
    __heap_start__ = .;
    *(.heap .heap.*)
    . = ALIGN(MAX(__heap_start__ + __HEAPSIZE__, .), 4);
    __heap_end__ = __heap_start__ + SIZEOF(.heap);
  }

  /* newlib (libnosys _sbrk) expects the classic `end`/`_end` symbol to mark the
     start of the free heap area; alias it to the end of statically placed data
     (start of the .heap reservation). */
  end = __heap_end__;
  _end = __heap_end__;

  /* ---- main stack at top of RAM, growing down */
  __stack_load_start__ = __RAM1_segment_start__ + (__RAM1_segment_size__ - __STACKSIZE__);
  .stack __RAM1_segment_start__ + (__RAM1_segment_size__ - __STACKSIZE__) (NOLOAD) :
    AT(__RAM1_segment_start__ + (__RAM1_segment_size__ - __STACKSIZE__))
  {
    __stack_start__ = .;
    __StackLimit = __stack_start__;
    *(.stack .stack.*)
    . = ALIGN(MAX(__stack_start__ + __STACKSIZE__, .), 8);
    __stack_end__ = __stack_start__ + SIZEOF(.stack);
    __StackTop = __stack_end__;
  }
  __stack_size__ = SIZEOF(.stack);

  /* ---- process stack (size 0 in this build; symbols consumed by thumb_crt0.s) */
  __stack_process_load_start__ = ALIGN(__stack_end__, 8);
  .stack_process ALIGN(__stack_end__, 8) (NOLOAD) : AT(ALIGN(__stack_end__, 8))
  {
    __stack_process_start__ = .;
    *(.stack_process .stack_process.*)
    . = ALIGN(MAX(__stack_process_start__ + __STACKSIZE_PROCESS__, .), 8);
    __stack_process_end__ = __stack_process_start__ + SIZEOF(.stack_process);
  }

  __RAM1_segment_used_end__ = ALIGN(__stack_end__, 8) + SIZEOF(.stack_process);

  . = ASSERT(__heap_end__ <= __stack_start__, "error: section .heap overlaps absolute placed section .stack");
  . = ASSERT(__stack_end__ <= __RAM1_segment_end__, "error: .stack exceeds RAM1 memory segment");
  . = ASSERT((__FLASH1_segment_used_end__ - __FLASH1_segment_start__) <= __FLASH1_segment_size__, "error: FLASH overflow");
}
