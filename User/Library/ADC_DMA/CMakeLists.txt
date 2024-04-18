add_library(adc_dma STATIC)

target_sources(adc_dma
        PRIVATE
        Src/ADC_DMA.c
        )

target_include_directories(adc_dma
        INTERFACE
        ./Inc
        )

target_link_libraries(adc_dma
        PRIVATE
            hal
        )