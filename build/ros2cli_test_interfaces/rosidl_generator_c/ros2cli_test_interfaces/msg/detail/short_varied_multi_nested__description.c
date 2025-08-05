// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros2cli_test_interfaces:msg/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

#include "ros2cli_test_interfaces/msg/detail/short_varied_multi_nested__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_type_hash_t *
ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5c, 0x43, 0xd7, 0x0c, 0x2f, 0xb2, 0x63, 0xbe,
      0xd3, 0xef, 0x24, 0x72, 0x8c, 0x0a, 0xba, 0xac,
      0x37, 0xa4, 0xca, 0x0d, 0xc2, 0x9a, 0x97, 0x9f,
      0x8a, 0x57, 0xb8, 0xc0, 0x82, 0xc2, 0x21, 0xb2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t ros2cli_test_interfaces__msg__ShortVaried__EXPECTED_HASH = {1, {
    0xf1, 0x87, 0x20, 0x5b, 0x0b, 0x7e, 0xaf, 0x6f,
    0xc9, 0x7d, 0xfd, 0xac, 0xf8, 0xd6, 0xda, 0xd2,
    0xd4, 0x20, 0x84, 0x2c, 0x00, 0x2b, 0x6e, 0x92,
    0x69, 0xb9, 0xe3, 0xcd, 0x2d, 0x19, 0xc5, 0xc2,
  }};
static const rosidl_type_hash_t ros2cli_test_interfaces__msg__ShortVariedNested__EXPECTED_HASH = {1, {
    0xd2, 0x39, 0x3c, 0xfc, 0xf7, 0x84, 0x2e, 0x12,
    0xc3, 0x7e, 0x03, 0x3a, 0xf0, 0x72, 0xc9, 0x29,
    0x9a, 0x58, 0x85, 0xb9, 0x8e, 0xc7, 0x85, 0xb8,
    0x0d, 0x49, 0x2a, 0x80, 0x5e, 0xdd, 0x5b, 0x2f,
  }};
#endif

static char ros2cli_test_interfaces__msg__ShortVariedMultiNested__TYPE_NAME[] = "ros2cli_test_interfaces/msg/ShortVariedMultiNested";
static char ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME[] = "ros2cli_test_interfaces/msg/ShortVaried";
static char ros2cli_test_interfaces__msg__ShortVariedNested__TYPE_NAME[] = "ros2cli_test_interfaces/msg/ShortVariedNested";

// Define type names, field names, and default values
static char ros2cli_test_interfaces__msg__ShortVariedMultiNested__FIELD_NAME__short_varied_nested[] = "short_varied_nested";

static rosidl_runtime_c__type_description__Field ros2cli_test_interfaces__msg__ShortVariedMultiNested__FIELDS[] = {
  {
    {ros2cli_test_interfaces__msg__ShortVariedMultiNested__FIELD_NAME__short_varied_nested, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2cli_test_interfaces__msg__ShortVariedNested__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros2cli_test_interfaces__msg__ShortVariedMultiNested__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__msg__ShortVariedNested__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2cli_test_interfaces__msg__ShortVariedMultiNested__TYPE_NAME, 50, 50},
      {ros2cli_test_interfaces__msg__ShortVariedMultiNested__FIELDS, 1, 1},
    },
    {ros2cli_test_interfaces__msg__ShortVariedMultiNested__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&ros2cli_test_interfaces__msg__ShortVaried__EXPECTED_HASH, ros2cli_test_interfaces__msg__ShortVaried__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ros2cli_test_interfaces__msg__ShortVaried__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros2cli_test_interfaces__msg__ShortVariedNested__EXPECTED_HASH, ros2cli_test_interfaces__msg__ShortVariedNested__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = ros2cli_test_interfaces__msg__ShortVariedNested__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A short, varied, and nested type\n"
  "ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 1\n"
  "# Trailing comment";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2cli_test_interfaces__msg__ShortVariedMultiNested__TYPE_NAME, 50, 50},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 128, 128},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_individual_type_description_source(NULL),
    sources[1] = *ros2cli_test_interfaces__msg__ShortVaried__get_individual_type_description_source(NULL);
    sources[2] = *ros2cli_test_interfaces__msg__ShortVariedNested__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
