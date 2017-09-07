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


import em
from io import StringIO
import os
import sys

RELATIVE_RESOURCE_PATH = '../../resource/'


def expand_template(template_file, data, output_file, minimum_timestamp=None):
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
        except Exception:
            if os.path.exists(output_file):
                os.remove(output_file)
            print("Exception when expanding '%s' into '%s'" %
                  (template_file, output_file), file=sys.stderr)
            raise
        finally:
            interpreter.shutdown()

    # only overwrite file if necessary
    # which is either when the timestamp is too old or when the content is different
    if os.path.exists(output_file):
        timestamp = os.path.getmtime(output_file)
        if minimum_timestamp is None or timestamp > minimum_timestamp:
            with open(output_file, 'r') as h:
                if h.read() == content:
                    return
    else:
        # create folder if necessary
        try:
            os.makedirs(os.path.dirname(output_file))
        except FileExistsError:
            pass

    with open(output_file, 'w') as h:
        h.write(content)


def create_folder(folder_name, base_directory):
    # if not os.path.isabs(base_directory):
    #     print("cannot create '%s', folder path is not absolute" % base_directory)
    #     return False

    folder_path = os.path.join(base_directory, folder_name)
    if os.path.exists(folder_path):
        print("cannot create '%s', folder exists" % folder_path)
        return False

    print('creating folder', folder_path)
    os.makedirs(folder_path)

    return folder_path


def create_template_file(template_file_name, output_directory, output_file_name, template_config):
    template_path = os.path.abspath(
        os.path.join(os.path.realpath(__file__), RELATIVE_RESOURCE_PATH + template_file_name))
    if not os.path.exists(template_path):
        raise IOError('template not found:', template_path)

    print("joining '%s' and '%s'" % (output_directory, output_file_name))
    output_file_path = os.path.join(output_directory, output_file_name)

    print('creating', output_file_path)
    expand_template(template_path, template_config, output_file_path)
