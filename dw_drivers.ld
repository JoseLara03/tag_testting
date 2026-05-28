    .dw_drivers : SUBALIGN(4) {
        __dw_drivers_start = .;
        KEEP(*(.dw_drivers*))
        KEEP(*(._dw_drivers*))
        __dw_drivers_end = .;
    } GROUP_LINK_IN(ROMABLE_REGION)
