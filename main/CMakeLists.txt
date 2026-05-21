#set(COMPONENT_SRCS "main.c")
#set(COMPONENT_ADD_INCLUDEDIRS "")

#register_component()

idf_component_register(SRCS "main.c"
                            "wiimote_bt.c"
                            "wiimote_memory.c"
                            "wiimote_state.c"
                            "wiimote_reports.c"
                            "wiimote_protocol.c"
                            "wiimote_uart_input.c"
                            "wiimote_runtime.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_uart esp_timer
                    INCLUDE_DIRS ".")
