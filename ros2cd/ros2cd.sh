# ros2 cd

function ros2cd {
    path=$(ros2 cd $1 2>&1 >/dev/null)
    cd $path
}