
set(SCREEN_DIR "controller_driver/ili9341"
            "controller_driver/st7789"
            "controller_driver/st7796"
            "controller_driver/nt35510"
            "controller_driver/ili9806"
            "controller_driver/ili9486"
            "controller_driver/ssd1351"
            "controller_driver/rm68120"
            "controller_driver/ssd1306"
            "controller_driver/ssd1307"
            "controller_driver/ssd1322"
            )

idf_component_register(SRC_DIRS "${SCREEN_DIR}" "screen_utility" "interface_driver" "."
                        INCLUDE_DIRS "${SCREEN_DIR}" "interface_driver" "."
                        PRIV_INCLUDE_DIRS "screen_utility"
                        REQUIRES bus
                        )

target_compile_options(${COMPONENT_LIB} PRIVATE 
                                        -Wno-unused-function)
