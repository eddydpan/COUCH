set(srcs "app_main.c" "my_platform.c")

set(requires "bluepad32" "btstack" "driver")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
