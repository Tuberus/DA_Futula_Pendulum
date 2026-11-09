
idf_component_register( SRCS "./core/TimerHL.c"
			     "./commu/softap_hl.c"
                             "./commu/station_hl2.c"
                             
     INCLUDE_DIRS "../dsp" "./core" "./commu"

     # add_compile_definitions(USE_UDP=1)
     # target_compile_definitions(${COMPONENT_LIB} PRIVATE USE_UDP=1)

     REQUIRES driver esp_timer esp_adc nvs_flash esp_wifi esp_driver_gpio )
