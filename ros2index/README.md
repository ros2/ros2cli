# ros2index

A `ros2cli` command to list and inspect entries in the [ament resource index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md) — the same index queried by `ament_index_python` — exposed as `ros2 index` verbs instead of a standalone script.

## Usage

```sh
$ ros2 index --help
usage: ros2 index [-h] ...

Commands:
  get    Output the content of a specific resource in the ament resource index
  list   Output the resources of a given type in the ament resource index
  types  Output the resource types in the ament resource index
```

### `types` — list resource types

```sh
$ ros2 index types
packages
package_run_dependencies
...
```

### `list <type>` — list resources of a given type

Prints each resource name and the prefix path it was found under.

```sh
$ ros2 index list packages
ament_index_python	/opt/ros/rolling
ros2index		/opt/ros/rolling
...
```

### `get <type> <name>` — print a resource's content

Prints the prefix path, and the resource file's content if it's non-empty.

```sh
$ ros2 index get packages ros2index
/opt/ros/rolling
```

If the resource doesn't exist, the command prints an error and exits non-zero:

```sh
$ ros2 index get packages does_not_exist
Could not find the resource 'does_not_exist' of type 'packages'
```
