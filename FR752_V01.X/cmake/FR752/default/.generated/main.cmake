# cmake files support debug production
include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(FR752_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(FR752_default_default_XC8_FILE_TYPE_assemble)
add_library(FR752_default_default_XC8_assemble OBJECT ${FR752_default_default_XC8_FILE_TYPE_assemble})
    FR752_default_default_XC8_assemble_rule(FR752_default_default_XC8_assemble)
    list(APPEND FR752_default_library_list "$<TARGET_OBJECTS:FR752_default_default_XC8_assemble>")

endif()

# Handle files with suffix S, for group default-XC8
if(FR752_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(FR752_default_default_XC8_assemblePreprocess OBJECT ${FR752_default_default_XC8_FILE_TYPE_assemblePreprocess})
    FR752_default_default_XC8_assemblePreprocess_rule(FR752_default_default_XC8_assemblePreprocess)
    list(APPEND FR752_default_library_list "$<TARGET_OBJECTS:FR752_default_default_XC8_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group default-XC8
if(FR752_default_default_XC8_FILE_TYPE_compile)
add_library(FR752_default_default_XC8_compile OBJECT ${FR752_default_default_XC8_FILE_TYPE_compile})
    FR752_default_default_XC8_compile_rule(FR752_default_default_XC8_compile)
    list(APPEND FR752_default_library_list "$<TARGET_OBJECTS:FR752_default_default_XC8_compile>")

endif()


# Main target for this project
add_executable(FR752_default_image__xkAuNIl ${FR752_default_library_list})

set_target_properties(FR752_default_image__xkAuNIl PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${FR752_default_output_dir})
set_target_properties(FR752_default_image__xkAuNIl PROPERTIES OUTPUT_NAME "default")
set_target_properties(FR752_default_image__xkAuNIl PROPERTIES SUFFIX ".elf")

target_link_libraries(FR752_default_image__xkAuNIl PRIVATE ${FR752_default_default_XC8_FILE_TYPE_link})


# Add the link options from the rule file.
FR752_default_link_rule(FR752_default_image__xkAuNIl)



