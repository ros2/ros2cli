// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_type_hash_t *
ros2cli_test_interfaces__msg__ShortVaried__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf1, 0x87, 0x20, 0x5b, 0x0b, 0x7e, 0xaf, 0x6f,
      0xc9, 0x7d, 0xfd, 0xac, 0xf8, 0xd6, 0xda, 0xd2,
      0xd4, 0x20, 0x84, 0x2c, 0x00, 0x2b, 0x6e, 0x92,
      0x69, 0xb9, 0xe3, 0xcd, 0x2d, 0x19, 0xc5, 0xc2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME[] = "ros2cli_test_interfaces/msg/ShortVaried";

// Define type names, field names, and default values
static char ros2cli_test_interfaces__msg__ShortVaried__FIELD_NAME__bool_value[] = "bool_value";
static char ros2cli_test_interfaces__msg__ShortVaried__FIELD_NAME__bool_values[] = "bool_values";

static rosidl_runtime_c__type_description__Field ros2cli_test_interfaces__msg__ShortVaried__FIELDS[] = {
  {
    {ros2cli_test_interfaces__msg__ShortVaried__FIELD_NAME__bool_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__msg__ShortVaried__FIELD_NAME__bool_values, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2cli_test_interfaces__msg__ShortVaried__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME, 39, 39},
      {ros2cli_test_interfaces__msg__ShortVaried__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A constant\n"
  "bool BOOL_CONST=true # Comment - Nesting Level 1: 1 of 2\n"
  "\n"
  "# Bool and array of bools\n"
  "bool bool_value\n"
  "bool[<=3] bool_values # Comment - Nesting Level 1: 2 of 2\n"
  "\n"
  "# Trailing comment";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros2cli_test_interfaces__msg__ShortVaried__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 191, 191},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2cli_test_interfaces__msg__ShortVaried__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2cli_test_interfaces__msg__ShortVaried__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
