## Define temporary variable
set(TEMP_GENERATE_MESSAGES "")

file(GLOB message_paths ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(message_path ${message_paths})
    if(IS_DIRECTORY ${message_path})
        get_filename_component(message_name ${message_path} NAME)
        # Call project api to set ros message namespace
        project(${message_name})
        set(cmake_message_namespace ${message_name})
        # Generate necessary package.xml file needed by catkin_tools
        configure_file(package.xml.conf ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
        # Generate messages
        file(GLOB msgs
            RELATIVE ${message_path}
            ${message_path}/*.msg
        )
        add_message_files(DIRECTORY ${message_name} FILES ${msgs})
        generate_messages(
            DEPENDENCIES
                std_msgs sensor_msgs
            LANGS
                gencpp genpy
        )
        list(APPEND TEMP_GENERATE_MESSAGES ${message_name}_generate_messages_cpp)
        # Geneate messages link into common include
        add_custom_command(
            TARGET ${message_name}_generate_messages_cpp
            PRE_BUILD
            # remove first to make sure corrent ln -s
            COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/../include/common/msgs/${message_name}
            COMMAND ln -s ${CATKIN_DEVEL_PREFIX}/include/${message_name}
            ${CMAKE_CURRENT_SOURCE_DIR}/../include/common/msgs/${message_name}
            COMMENT "Geneate messages link into common include"
        )
    endif()
endforeach()

## Export genearte messages
set(GENERATE_MESSAGES ${TEMP_GENERATE_MESSAGES} PARENT_SCOPE)