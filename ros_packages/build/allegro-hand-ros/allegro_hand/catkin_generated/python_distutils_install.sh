#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/work/code_repository/ros_packages/src/allegro-hand-ros/allegro_hand"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/work/code_repository/ros_packages/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/work/code_repository/ros_packages/install/lib/python2.7/dist-packages:/home/work/code_repository/ros_packages/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/work/code_repository/ros_packages/build" \
    "/usr/bin/python" \
    "/home/work/code_repository/ros_packages/src/allegro-hand-ros/allegro_hand/setup.py" \
    build --build-base "/home/work/code_repository/ros_packages/build/allegro-hand-ros/allegro_hand" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/work/code_repository/ros_packages/install" --install-scripts="/home/work/code_repository/ros_packages/install/bin"
