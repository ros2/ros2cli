<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>@package_name</name>
  <version>0.0.0</version>
  <description>@package_description</description>
  <maintainer email="@maintainer_email">@maintainer_name</maintainer>
  <license>@package_license</license>

  <buildtool_depend>@build_type</buildtool_depend>

@[if dependencies]@
@[  for dep in dependencies]@
  <depend>@dep</depend>
@[  end for]@

@[end if]@
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>@build_type</build_type>
  </export>
</package>
