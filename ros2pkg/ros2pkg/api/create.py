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

RESOURCE_PATH = os.path.join(os.path.dirname(__file__), '..', 'resource')


def expand_template(template_file, data, output_file):
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


def create_folder(folder_name, base_directory):
    folder_path = os.path.join(base_directory, folder_name)
    if os.path.exists(folder_path):
        print("cannot create '%s', folder exists" % folder_path, file=sys.stderr)
        return False

    print('creating folder', folder_path)
    os.makedirs(folder_path)

    return folder_path


def create_template_file(template_file_name, output_directory, output_file_name, template_config):
    template_path = os.path.abspath(
        os.path.join(__file__, RESOURCE_PATH, template_file_name))
    if not os.path.exists(template_path):
        raise FileNotFoundError('template not found:', template_path)

    output_file_path = os.path.join(output_directory, output_file_name)

    print('creating', output_file_path)
    expand_template(template_path, template_config, output_file_path)
