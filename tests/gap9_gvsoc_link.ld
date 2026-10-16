OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
    L2 : ORIGIN = 0x1c010000, LENGTH = 0x00180000
}

SECTIONS
{
    .init : { KEEP(*(.init)) } > L2
    .preinit_array : { KEEP(*(.preinit_array)) } > L2
    .init_array :
    {
        __preinit_array_start = .;
        KEEP(*(.init_array*));
        __init_array_start = .;
        __init_array_end = .;
    } > L2
    .fini_array : { KEEP(*(.fini_array*)) } > L2
    .rodata : { *(.rodata*) *(.srodata*) } > L2
    .data :
    {
        . = ALIGN(8);
        sdata = .;
        _sdata = .;
        *(.data*) *(.sdata*)
        . = ALIGN(8);
        edata = .;
        _edata = .;
    } > L2
    .bss (NOLOAD) :
    {
        . = ALIGN(8);
        _bss_start = .;
        *(.bss*) *(.sbss*) *(COMMON)
        . = ALIGN(8);
        _bss_end = .;
    } > L2
    .stack (NOLOAD) :
    {
        . = ALIGN(16);
        stack_start = .;
        . += 0x800;
        stack = .;
    } > L2
    .text :
    {
        . = ALIGN(16);
        *(.text*) *(.gnu.linkonce.t.*)
        . = ALIGN(16);
    } > L2
    __mem_end = ALIGN(8);
}
