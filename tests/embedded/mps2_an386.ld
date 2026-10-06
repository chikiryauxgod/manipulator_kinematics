ENTRY(Reset_Handler)
MEMORY {
    ROM (rx) : ORIGIN = 0x00000000, LENGTH = 1M
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 1M
}
_stack_top = ORIGIN(RAM) + LENGTH(RAM);
_stack_bottom = _stack_top - 16K;
SECTIONS {
    .vectors : { KEEP(*(.vectors)) } > ROM
    .text : { *(.text*) *(.rodata*) } > ROM
    .ARM.exidx : { *(.ARM.exidx*) } > ROM
    .data : {
        _data_start = .; *(.data*) _data_end = .;
    } > RAM AT> ROM
    _data_load = LOADADDR(.data);
    .bss (NOLOAD) : {
        _bss_start = .; *(.bss*) *(COMMON) _bss_end = .;
    } > RAM
    ASSERT(_bss_end < _stack_bottom, "Application buffers overlap reserved stack")
}
