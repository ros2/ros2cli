// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ros2cli_test_interfaces:srv/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_type_hash_t *
ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x45, 0x0d, 0x49, 0x42, 0x24, 0xad, 0x6a, 0x6e,
      0xa3, 0x52, 0x6f, 0xb9, 0x9e, 0x9a, 0x56, 0xcf,
      0x0c, 0x4c, 0xf1, 0x2c, 0x27, 0x8e, 0x9c, 0x1f,
      0x76, 0x4a, 0x27, 0x06, 0x70, 0x6d, 0x38, 0x03,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_type_hash_t *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x05, 0x49, 0x38, 0x25, 0x3a, 0x68, 0x83, 0xb2,
      0x14, 0x06, 0x91, 0x79, 0x08, 0x36, 0x8a, 0x21,
      0x0a, 0xc4, 0x3e, 0x53, 0x84, 0xf9, 0x5f, 0xac,
      0x48, 0xd2, 0xc5, 0xb6, 0x73, 0x37, 0x92, 0x3c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_type_hash_t *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe2, 0xb3, 0x49, 0x1b, 0x14, 0xe0, 0xe2, 0xa8,
      0x8c, 0x4a, 0x47, 0x76, 0x8f, 0xf0, 0x34, 0xca,
      0xb1, 0xb1, 0xcb, 0x22, 0x7d, 0xab, 0x6e, 0x25,
      0x99, 0x58, 0x1d, 0x6f, 0x84, 0xc9, 0x59, 0xd2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_type_hash_t *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x91, 0x72, 0xc5, 0x41, 0xa8, 0xf7, 0xd0, 0x5d,
      0x72, 0xee, 0x25, 0x9b, 0x98, 0x89, 0xcc, 0x04,
      0x95, 0x05, 0xc5, 0x9a, 0xfc, 0xc6, 0x73, 0xc2,
      0x84, 0x11, 0x4a, 0x40, 0x13, 0x9c, 0x7f, 0xbc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
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
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char ros2cli_test_interfaces__srv__ShortVariedMultiNested__TYPE_NAME[] = "ros2cli_test_interfaces/srv/ShortVariedMultiNested";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME[] = "ros2cli_test_interfaces/msg/ShortVaried";
static char ros2cli_test_interfaces__msg__ShortVariedNested__TYPE_NAME[] = "ros2cli_test_interfaces/msg/ShortVariedNested";
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__TYPE_NAME[] = "ros2cli_test_interfaces/srv/ShortVariedMultiNested_Event";
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__TYPE_NAME[] = "ros2cli_test_interfaces/srv/ShortVariedMultiNested_Request";
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__TYPE_NAME[] = "ros2cli_test_interfaces/srv/ShortVariedMultiNested_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELD_NAME__request_message[] = "request_message";
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELD_NAME__response_message[] = "response_message";
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELDS[] = {
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__TYPE_NAME, 58, 58},
    },
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__TYPE_NAME, 59, 59},
    },
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__TYPE_NAME, 56, 56},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros2cli_test_interfaces__srv__ShortVariedMultiNested__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__msg__ShortVariedNested__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__TYPE_NAME, 56, 56},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__TYPE_NAME, 58, 58},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__TYPE_NAME, 59, 59},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested__TYPE_NAME, 50, 50},
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested__FIELDS, 3, 3},
    },
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros2cli_test_interfaces__msg__ShortVaried__EXPECTED_HASH, ros2cli_test_interfaces__msg__ShortVaried__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = ros2cli_test_interfaces__msg__ShortVaried__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros2cli_test_interfaces__msg__ShortVariedNested__EXPECTED_HASH, ros2cli_test_interfaces__msg__ShortVariedNested__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = ros2cli_test_interfaces__msg__ShortVariedNested__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__FIELD_NAME__short_varied_nested[] = "short_varied_nested";

static rosidl_runtime_c__type_description__Field ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__FIELDS[] = {
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__FIELD_NAME__short_varied_nested, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ros2cli_test_interfaces__msg__ShortVariedNested__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__TYPE_NAME, 58, 58},
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__FIELDS, 1, 1},
    },
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
// Define type names, field names, and default values
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__FIELD_NAME__bool_value[] = "bool_value";

static rosidl_runtime_c__type_description__Field ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__FIELDS[] = {
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__FIELD_NAME__bool_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__TYPE_NAME, 59, 59},
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELD_NAME__info[] = "info";
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELD_NAME__request[] = "request";
static char ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELDS[] = {
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__TYPE_NAME, 58, 58},
    },
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__TYPE_NAME, 59, 59},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__msg__ShortVaried__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__msg__ShortVariedNested__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__TYPE_NAME, 58, 58},
    {NULL, 0, 0},
  },
  {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__TYPE_NAME, 59, 59},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__TYPE_NAME, 56, 56},
      {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__FIELDS, 3, 3},
    },
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros2cli_test_interfaces__msg__ShortVaried__EXPECTED_HASH, ros2cli_test_interfaces__msg__ShortVaried__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = ros2cli_test_interfaces__msg__ShortVaried__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&ros2cli_test_interfaces__msg__ShortVariedNested__EXPECTED_HASH, ros2cli_test_interfaces__msg__ShortVariedNested__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = ros2cli_test_interfaces__msg__ShortVariedNested__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Request\n"
  "ShortVariedNested short_varied_nested # Comment - Nesting Level 3: 1 of 2\n"
  "---\n"
  "# Response\n"
  "bool bool_value # Comment - Nesting Level 3: 2 of 2";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested__TYPE_NAME, 50, 50},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 151, 151},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__TYPE_NAME, 58, 58},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__TYPE_NAME, 59, 59},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__TYPE_NAME, 56, 56},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *ros2cli_test_interfaces__msg__ShortVaried__get_individual_type_description_source(NULL);
    sources[3] = *ros2cli_test_interfaces__msg__ShortVariedNested__get_individual_type_description_source(NULL);
    sources[4] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_individual_type_description_source(NULL);
    sources[5] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_individual_type_description_source(NULL);
    sources[6] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_individual_type_description_source(NULL),
    sources[1] = *ros2cli_test_interfaces__msg__ShortVaried__get_individual_type_description_source(NULL);
    sources[2] = *ros2cli_test_interfaces__msg__ShortVariedNested__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *ros2cli_test_interfaces__msg__ShortVaried__get_individual_type_description_source(NULL);
    sources[3] = *ros2cli_test_interfaces__msg__ShortVariedNested__get_individual_type_description_source(NULL);
    sources[4] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_individual_type_description_source(NULL);
    sources[5] = *ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
