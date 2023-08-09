# ros2rosout

This is the `ros2 rosout print` utility command which displaies the content of `/rosout` topic in a nicely formatted and colorized output.

## Usage 

Run `ros2 rosout print` to get the live stream of the logs.

Run `ros2 rosout print -h/--help` to print all available command arguments.


## Output format

The command outputs the log line with the following format: 
`[` _datetime_ `] [` _level_ `] [` _nodename_ `]: ` _log_ `:[` _file_ `:` _line_ `(` _function_ `)]` 

File, line and function are all optional and are displays when the `-f/--function-detail` switch is provided. 


