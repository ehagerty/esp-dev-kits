# Board information output
message(STATUS "Selected Board: esp_SensairShuttle")
message(STATUS "Board Path: ./boards/esp_SensairShuttle")

idf_component_register(
    SRC_DIRS "." "../../boards/esp_SensairShuttle"
    INCLUDE_DIRS "." "../../boards/esp_SensairShuttle"
    REQUIRES esp_board_manager
)

# This is equivalent to adding WHOLE_ARCHIVE option to the idf_component_register call above:
idf_component_set_property(${COMPONENT_NAME} WHOLE_ARCHIVE TRUE)
