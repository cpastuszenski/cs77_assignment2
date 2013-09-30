#!/bin/bash

function check_error {
    if [ "$?" -ne "0" ]; then
        echo "An error occurred!"
        exit 1
    fi
}

echo "test01: Cylinder"
./view -i scenes/test01.json
check_error

echo "test02: Mesh with triangles"
./view -i scenes/test02.json
check_error

echo "test03: Mesh with quads and triangles"
./view -i scenes/test03.json
check_error

echo "test04: Catmull-Clark Subdiv"
./view -i scenes/test04.json
check_error

echo "test05: Transforms"
./view -i scenes/test05.json
check_error

echo "test06: Bezier Spline"
./view -i scenes/test06.json
check_error

echo "test07: Bezier Spline tessellated recursively"
./view -i scenes/test07.json
check_error

echo "test08: test07's Bezier Spline tessellated uniformly"
./view -i scenes/test08.json
check_error

echo "All completed successfully!"
