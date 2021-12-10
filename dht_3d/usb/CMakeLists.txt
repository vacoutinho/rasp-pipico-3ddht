if (TARGET tinyusb_device)
    add_executable(3d_dht
            3d_dht.c
            )

    # Pull in our pico_stdlib which aggregates commonly used features
    target_link_libraries(3d_dht pico_stdlib)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(3d_dht 1)
    pico_enable_stdio_uart(3d_dht 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(3d_dht)

    # add url via pico_set_program_url
    example_auto_set_url(3d_dht)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building 3d_dht because TinyUSB submodule is not initialized in the SDK")
endif()
