idf_component_register(
    SRCS
        "src/DW1000Ng.cpp"
        "src/DW1000NgRanging.cpp"
        "src/DW1000NgRTLS.cpp"
        "src/DW1000NgTime.cpp"
        "src/DW1000NgUtils.cpp"
        "src/DW1000NgPort.cpp"
        "src/SPIporting.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio esp_timer
)

# Upstream code, kept as close to the original as possible: relax the warnings
# ESP-IDF promotes to errors rather than editing the register logic.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-unused-but-set-variable
)
