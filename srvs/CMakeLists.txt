## Define temporary variable
set(TEMP_GENERATE_SERVICE_MESSAGES "")

file(GLOB service_paths ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(service_path ${service_paths})
    if(IS_DIRECTORY ${service_path})
        get_filename_component(service_name ${service_path} NAME)
        # Call project api to set ros service [message] namespace
        project(${service_name})
        set(cmake_message_namespace ${service_name})
        # Generate necessary package.xml file needed by catkin_tools
        configure_file(package.xml.conf ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)
        # Generate services
        file(GLOB srvs
            RELATIVE ${service_path}
            ${service_path}/*.srv
        )
        add_service_files(DIRECTORY ${service_path} FILES ${srvs})
        generate_messages(
            DEPENDENCIES
                std_msgs sensor_msgs
            LANGS
                gencpp genpy
        )
        list(APPEND TEMP_GENERATE_SERVICE_MESSAGES ${service_name}_generate_messages_cpp)
        # Geneate services link into common include
        add_custom_command(
            TARGET ${service_name}_generate_messages_cpp
            PRE_BUILD
            # remove first to make sure corrent ln -s
            COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/../include/common/srvs/${service_name}
            COMMAND ln -s ${CATKIN_DEVEL_PREFIX}/include/${service_name}
            ${CMAKE_CURRENT_SOURCE_DIR}/../include/common/srvs/${service_name}
            COMMENT "Geneate services link into common include"
        )
    endif()
endforeach()

## Export genearte service messages
set(GENERATE_SERVICE_MESSAGES ${TEMP_GENERATE_SERVICE_MESSAGES} PARENT_SCOPE)