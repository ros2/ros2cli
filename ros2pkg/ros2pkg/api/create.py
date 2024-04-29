# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from io import StringIO
import os
import sys

import em
try:
    import importlib.resources as importlib_resources
except ModuleNotFoundError:
    import importlib_resources


def _expand_template(template_file, data, output_file):
    output = StringIO()
    interpreter = em.Interpreter(
        output=output,
        options={
            em.BUFFERED_OPT: True,
            em.RAW_OPT: True,
        },
        globals=data,
    )
    with open(template_file, 'r') as h:
        try:
            interpreter.file(h)
            content = output.getvalue()
        except Exception as e:
            if os.path.exists(output_file):
                os.remove(output_file)
            print("Exception when expanding '%s' into '%s': %s" %
                  (template_file, output_file, e), file=sys.stderr)
            raise
        finally:
            interpreter.shutdown()

    if os.path.exists(output_file):
        with open(output_file, 'r') as h:
            if h.read() == content:
                return
    else:
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

    with open(output_file, 'w') as h:
        h.write(content)


def _create_folder(folder_name, base_directory, exist_ok=True):
    folder_path = os.path.join(base_directory, folder_name)

    print('creating folder', folder_path)
    os.makedirs(folder_path, exist_ok=exist_ok)

    return folder_path


def _create_template_file(
    template_subdir, template_file_name, output_directory, output_file_name, template_config
):
    full_package = 'ros2pkg.resource.' + template_subdir
    with importlib_resources.path(full_package, template_file_name) as path:
        template_path = str(path)
    if not os.path.exists(template_path):
        raise FileNotFoundError('template not found:', template_path)

    output_file_path = os.path.join(output_directory, output_file_name)

    print('creating', output_file_path)
    _expand_template(template_path, template_config, output_file_path)


def create_package_environment(package, destination_directory):
    package_directory = _create_folder(package.name, destination_directory)

    package_xml_config = {
        'package_format': package.package_format,
        'package_name': package.name,
        'package_description': package.description,
        'maintainer_email': package.maintainers[0].email,
        'maintainer_name': package.maintainers[0].name,
        'package_license': package.licenses[0],
        'buildtool_dependencies': package.buildtool_depends,
        'dependencies': package.build_depends,
        'test_dependencies': package.test_depends,
        'exports': package.exports,
    }
    _create_template_file(
        'package_environment',
        'package.xml.em',
        package_directory,
        'package.xml',
        package_xml_config)

    source_directory = None
    include_directory = None
    if package.get_build_type() == 'cmake' or package.get_build_type() == 'ament_cmake':
        print('creating source and include folder')
        source_directory = _create_folder('src', package_directory)
        include_directory = _create_folder(package.name, package_directory + os.sep + 'include')
    if package.get_build_type() == 'ament_python':
        print('creating source folder')
        source_directory = _create_folder(package.name, package_directory)
    if package.get_build_type() == 'ament_cmake_python':
        print('creating source folder')
        source_directory = _create_folder('src', package_directory)
        include_directory = _create_folder(package.name, package_directory + os.sep + 'inlclude')

    return package_directory, source_directory, include_directory


def populate_ament_python(package, package_directory, source_directory, python_node_name):
    setup_py_config = {
        'project_name': package.name,
        'maintainer_email': package.maintainers[0].email,
        'maintainer_name': package.maintainers[0].name,
        'package_license': package.licenses[0],
        'node_name': python_node_name,
        'test_dependencies': package.test_depends,
        'package_description': package.description
    }

    _create_template_file('ament_python',
                          'setup.py.em',
                          package_directory,
                          'setup.py',
                          setup_py_config)

    setup_cfg_config = {'project_name': package.name}
    _create_template_file('ament_python',
                          'setup.cfg.em',
                          package_directory,
                          'setup.cfg',
                          setup_cfg_config)

    resource_directory = _create_folder('resource', package_directory)
    _create_template_file('ament_python',
                          'resource_file.em',
                          resource_directory,
                          package.name,
                          {})

    _create_template_file('ament_python',
                          'init.py.em',
                          source_directory,
                          '__init__.py',
                          {})

    test_directory = _create_folder('test', package_directory)
    _create_template_file('ament_python',
                          'test_copyright.py.em',
                          test_directory,
                          'test_copyright.py',
                          {})
    _create_template_file('ament_python',
                          'test_flake8.py.em',
                          test_directory,
                          'test_flake8.py',
                          {})
    _create_template_file('ament_python',
                          'test_pep257.py.em',
                          test_directory,
                          'test_pep257.py',
                          {})


def populate_python_node(package, source_directory, python_node_name):
    main_py_config = {
        'project_name': package.name
    }
    _create_template_file('ament_python',
                          'main.py.em',
                          source_directory,
                          python_node_name + '.py',
                          main_py_config)


def populate_python_libary(package, source_directory, python_library_name):
    library_directory = _create_folder(python_library_name, source_directory)
    _create_template_file('ament_python',
                          'init.py.em',
                          library_directory,
                          '__init__.py',
                          {})


def populate_cmake(package, package_directory, cpp_node_name, cpp_library_name):
    cmakelists_config = {
        'project_name': package.name,
        'dependencies': [str(dep) for dep in package.build_depends],
        'cpp_node_name': cpp_node_name,
        'cpp_library_name': cpp_library_name,
    }
    _create_template_file(
        'cmake',
        'CMakeLists.txt.em',
        package_directory,
        'CMakeLists.txt',
        cmakelists_config)

    cmake_config = {
        'project_name': package.name,
        'cpp_library_name': cpp_library_name,
        'cpp_node_name': cpp_node_name,
    }
    _create_template_file(
        'cmake',
        'Config.cmake.in.em',
        package_directory,
        package.name + 'Config.cmake.in',
        cmake_config)

    version_config = {
        'project_name': package.name,
    }
    _create_template_file(
        'cmake',
        'ConfigVersion.cmake.in.em',
        package_directory,
        package.name + 'ConfigVersion.cmake.in',
        version_config)


def populate_ament_cmake(package, package_directory, cpp_node_name, cpp_library_name):
    cmakelists_config = {
        'project_name': package.name,
        'dependencies': [str(dep) for dep in package.build_depends],
        'cpp_node_name': cpp_node_name,
        'cpp_library_name': cpp_library_name,
    }
    _create_template_file(
        'ament_cmake',
        'CMakeLists.txt.em',
        package_directory,
        'CMakeLists.txt',
        cmakelists_config)


def populate_cpp_node(package, source_directory, cpp_node_name):
    cpp_node_config = {
        'package_name': package.name,
    }
    _create_template_file(
        'cpp',
        'main.cpp.em',
        source_directory,
        cpp_node_name + '.cpp',
        cpp_node_config)


def populate_cpp_library(package, source_directory, include_directory, cpp_library_name):
    class_name = cpp_library_name.replace('_', ' ').title()
    class_name = ''.join(x for x in class_name if not x.isspace())
    cpp_header_config = {
        'package_name': package.name,
        'library_name': cpp_library_name,
        'class_name': class_name,
    }
    _create_template_file(
        'cpp',
        'header.hpp.em',
        include_directory,
        cpp_library_name + '.hpp',
        cpp_header_config)

    cpp_library_config = {
        'package_name': package.name,
        'library_name': cpp_library_name,
        'class_name': class_name
    }
    _create_template_file(
        'cpp',
        'library.cpp.em',
        source_directory,
        cpp_library_name + '.cpp',
        cpp_library_config)

    visibility_config = {
        'package_name': package.name.upper(),
    }
    _create_template_file(
        'cpp',
        'visibility_control.h.em',
        include_directory,
        'visibility_control.h',
        visibility_config)


def populate_ament_cmake_python(package, package_directory, cpp_node_name, cpp_library_name):
    cmakelists_config = {
        'project_name': package.name,
        'dependencies': [str(dep) for dep in package.build_depends],
        'cpp_node_name': cpp_node_name,
        'cpp_library_name': cpp_library_name,
    }
    _create_template_file(
        'ament_cmake_python',
        'CMakeLists.txt.em',
        package_directory,
        'CMakeLists.txt',
        cmakelists_config)

    _create_folder(package.name, package_directory)
    _create_template_file('ament_cmake_python',
                          'init.py.em',
                          package_directory + os.sep + package.name,
                          '__init__.py',
                          {})

    _ = _create_folder('test', package_directory)
