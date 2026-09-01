# This CMakeLists.txt file is responsible for including all subdirectories within the current source directory.
# It uses the file() command to retrieve all subdirectories relative to the current source directory.
# Then, it iterates over each subdirectory and checks if it contains a CMakeLists.txt file.
# If a subdirectory contains a CMakeLists.txt file, it adds the subdirectory using the add_subdirectory() command.
# If a subdirectory does not contain a CMakeLists.txt file, it displays a warning message and ignores the subdirectory.

file(GLOB all_subdirectories RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
set(added_runner_subdirs "")

# 检查是否设置了环境变量进行选择性编译
set(used_runners_env "$ENV{ENGINEAI_ROBOTICS_USED_RUNNERS}")

if(used_runners_env)
    message(STATUS "Selective runner compilation enabled: ${used_runners_env}")
    
    # 将环境变量中的runner列表转换为CMake列表
    string(REPLACE "," ";" used_runners_list "${used_runners_env}")
    list(TRANSFORM used_runners_list STRIP)

    # These three runners are unique markers for the merged T800 feature set.
    # If a selective build requests any marker, derive the complete T800 runner
    # set from the shipped motion and resident task YAML files. This keeps the
    # build guard aligned with configuration changes while leaving other robot
    # selective builds unchanged.
    set(t800_merged_feature_markers
        rl_walking_custom_example_runner
        rl_walking_leolab_example_runner
        rl_getup_example_runner)
    set(t800_merged_feature_selected FALSE)
    foreach(feature_marker ${t800_merged_feature_markers})
        list(FIND used_runners_list "${feature_marker}" feature_marker_index)
        if(NOT feature_marker_index EQUAL -1)
            set(t800_merged_feature_selected TRUE)
        endif()
    endforeach()
    if(t800_merged_feature_selected)
        set(t800_task_graph_files
            "${CMAKE_SOURCE_DIR}/assets/config/t800/task_motion/default.yaml"
            "${CMAKE_SOURCE_DIR}/assets/config/t800/task_resident/robot.yaml"
            "${CMAKE_SOURCE_DIR}/assets/config/t800/task_resident/sim.yaml")
        set(t800_required_runners "")
        foreach(task_graph_file ${t800_task_graph_files})
            if(NOT EXISTS "${task_graph_file}")
                message(FATAL_ERROR "T800 task graph file not found: ${task_graph_file}")
            endif()
            file(STRINGS "${task_graph_file}" task_graph_runner_lines
                 REGEX "^[ \t]*-[ \t]*name:[ \t]*[A-Za-z0-9_]+")
            foreach(runner_line ${task_graph_runner_lines})
                string(REGEX REPLACE
                       "^[ \t]*-[ \t]*name:[ \t]*([A-Za-z0-9_]+).*$" "\\1"
                       task_graph_runner "${runner_line}")
                list(APPEND t800_required_runners "${task_graph_runner}")
            endforeach()
        endforeach()
        list(REMOVE_DUPLICATES t800_required_runners)
        if(NOT t800_required_runners)
            message(FATAL_ERROR "No runner names were found in the T800 task graph YAML files")
        endif()
        foreach(required_runner ${t800_required_runners})
            list(FIND used_runners_list "${required_runner}" required_runner_index)
            if(required_runner_index EQUAL -1)
                message(FATAL_ERROR
                    "T800 merged runtime requires '${required_runner}', as referenced by its task YAML files. "
                    "Current ENGINEAI_ROBOTICS_USED_RUNNERS: ${used_runners_env}")
            endif()
        endforeach()
    endif()
    
    # 从runner名称推导子目录名称
    # 始终包含basic目录（基础依赖）
    set(allowed_subdirs "basic" "motion_transition")
    
    foreach(used_runner ${used_runners_list})
        # runner名称格式: xxx_runner -> 子目录: xxx
        string(REGEX REPLACE "_runner$" "" runner_base "${used_runner}")
        # The SDK's mimic runner keeps the historical registry name
        # "rl_mimic_trajectory_runner" while its source directory is named
        # "rl_mimic_trajectory_example".
        if(used_runner STREQUAL "rl_mimic_trajectory_runner")
            set(runner_base "rl_mimic_trajectory_example")
        elseif(used_runner STREQUAL "t800_motor_pre_driver_safety_runner")
            set(runner_base "t800_safety")
        endif()
        list(APPEND allowed_subdirs "${runner_base}")
    endforeach()
    
    list(REMOVE_DUPLICATES allowed_subdirs)
    
    message(STATUS "Allowed runner subdirectories: ${allowed_subdirs}")

    # motion_transition and the T800-aware shared runners link this SDK safety
    # target. Define it before every selective build. AutoRegisterRunners also
    # registers both resident safety factories unconditionally; they are inert
    # unless a T800 task graph schedules them.
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/t800_safety/CMakeLists.txt")
        add_subdirectory(t800_safety)
        list(APPEND added_runner_subdirs "t800_safety")
        list(REMOVE_ITEM all_subdirectories "t800_safety")
        message(STATUS "Including runner subdirectory first: t800_safety")
    endif()
    
    foreach(subdir ${all_subdirectories})
        set(subdir_cmake_file ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)

        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} AND EXISTS ${subdir_cmake_file})
            # 检查该子目录是否在允许列表中
            list(FIND allowed_subdirs "${subdir}" subdir_index)
            if(NOT subdir_index EQUAL -1)
                add_subdirectory(${subdir})
                list(APPEND added_runner_subdirs "${subdir}")
                message(STATUS "Including runner subdirectory: ${subdir}")
            else()
                message(STATUS "Skipping runner subdirectory (not in used list): ${subdir}")
            endif()
        elseif(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
            message(WARNING "[ ${subdir} ] is a directory but does not contain a CMakeLists.txt file and will be ignored.")
        endif()
    endforeach()
else()
    message(STATUS "Building all runners (no environment filter)")

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/t800_safety/CMakeLists.txt")
        add_subdirectory(t800_safety)
        list(APPEND added_runner_subdirs "t800_safety")
        list(REMOVE_ITEM all_subdirectories "t800_safety")
        message(STATUS "Including runner subdirectory first: t800_safety")
    endif()
    
    foreach(subdir ${all_subdirectories})
        set(subdir_cmake_file ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)

        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} AND EXISTS ${subdir_cmake_file})
            add_subdirectory(${subdir})
            list(APPEND added_runner_subdirs "${subdir}")
        elseif(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
            message(WARNING "[ ${subdir} ] is a directory but does not contain a CMakeLists.txt file and will be ignored.")
        endif()
    endforeach()
endif()
