<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>@package_name</name>
  <version>0.0.0</version>
  <description>Description of @package_name</description>
  <maintainer email="@maintainer_email">@maintainer_name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

@[if dependencies]@
@[for deb in dependencies]@
  <build_depend>@deb</build_depend>
@[end for]@

@[for deb in dependencies]@
  <exec_depend>@deb</exec_depend>
@[end for]@

@[end if]@
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
