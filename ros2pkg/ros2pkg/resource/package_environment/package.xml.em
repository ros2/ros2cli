<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="@package_format">
  <name>@package_name</name>
  <version>0.0.0</version>
  <description>@package_description</description>
  <maintainer email="@maintainer_email">@maintainer_name</maintainer>
  <license>@package_license</license>

@[if buildtool_dependencies]@
@[  for dep in buildtool_dependencies]@
  <buildtool_depend>@dep</buildtool_depend>
@[  end for]@

@[end if]@
@[if dependencies]@
@[  for dep in dependencies]@
  <depend>@dep</depend>
@[  end for]@

@[end if]@
@[if test_dependencies]@
@[  for dep in test_dependencies]@
  <test_depend>@dep</test_depend>
@[  end for]@

@[end if]@
  <export>
    <build_type>@build_type</build_type>
  </export>
</package>
