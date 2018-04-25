/*
 * Copyright 2018 <Lennart Nachtigall> <firesurfer127@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once
#include <string>
#include <limits.h>
#include <unistd.h>
#include <link.h>
#include <vector>
#include "Reflect.h"
namespace  ros2_components {

/**
 * @brief The BuildInfo class provides a string that contains the date and time of the build. (The time when this file was processed by the preprocessor)
 */
class BuildInfo
{
public:
    static std::string get_build_date()
    {
#ifndef BUILD_TIME
        return std::string(__DATE__) + "  " + std::string(__TIME__);
#else
        return std::string(BUILD_TIME);
#endif
    }

    static std::string get_build_version()
    {
#ifndef BUILD_VERSION
#warning "BUILD_VERSION variable not set"
#define BUILD_VERSION "not set"
#endif
        return std::string(BUILD_VERSION);
    }

    static std::string get_exe_path()
    {
        char result[ PATH_MAX ];
        ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
        return std::string( result, (count > 0) ? count : 0 );
    }
    static std::string get_current_directory()
    {
        char* cur_dir_ptr = get_current_dir_name();
        std::string current_dir_name = std::string(cur_dir_ptr);
        std::free(cur_dir_ptr);
        return current_dir_name;
    }
    static std::vector<std::string> get_loaded_libraries()
    {
        libraries_names.clear();
        dl_iterate_phdr(iterate_libraries,NULL);
        return BuildInfo::libraries_names;
    }
private:
    static std::vector<std::string> libraries_names;

    static int iterate_libraries(struct dl_phdr_info *info, size_t size, void *data)
    {
        UNUSED(size);
        UNUSED(data);
        std::string name = std::string(info->dlpi_name);

        BuildInfo::libraries_names.push_back(name);
        return 0;
    }
};
}

