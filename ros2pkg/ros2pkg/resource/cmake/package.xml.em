<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="2">
  <name>@package_name</name>
  <version>0.0.0</version>
  <description>Description of @package_name</description>
  <maintainer email="@maintainer_email">@maintainer_name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>cmake</buildtool_depend>

@[if dependencies]@
@[  for deb in dependencies]@
  <depend>@deb</depend>
@[  end for]@

@[end if]@
  <export>
    <build_type>cmake</build_type>
  </export>
</package>
