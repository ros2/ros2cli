# Copyright 2020 Open Source Robotics Foundation, Inc.
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


def fullname(klass):
    return f'{klass.__module__}.{klass.__name__}'


def end_any_with_slots(unmarshaller, data, type_):
    unmarshaller._stack[-1] = type_(**unmarshaller._stack[-1])
    unmarshaller._value = 0


def dump_any_with_slots(marshaller, value, write, transform=None):
    write(f'<value><{fullname(type(value))}>')
    slots = value.__slots__
    if transform is not None:
        slots = map(transform, slots)
    marshaller.dump_struct({name: getattr(value, name) for name in slots}, write)
    write(f'</{fullname(type(value))}></value>')


def end_any_enum(unmarshaller, data, enum_):
    unmarshaller.append(enum_(int(data)))
    unmarshaller._value = 0


def dump_any_enum(marshaller, value, write):
    write(f'<value><{fullname(type(value))}>')
    write(str(value.value))
    write(f'</{fullname(type(value))}></value>')
