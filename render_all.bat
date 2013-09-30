@echo off

echo "test01: Cylinder"
view.exe -i scenes/test01.json || goto :error

echo "test02: Mesh with triangles"
view.exe -i scenes/test02.json || goto :error

echo "test03: Mesh with quads and triangles"
view.exe -i scenes/test03.json || goto :error

echo "test04: Catmull-Clark Subdiv"
view.exe -i scenes/test04.json || goto :error

echo "test05: Transforms"
view.exe -i scenes/test05.json || goto :error

echo "test06: Bezier Spline"
view.exe -i scenes/test06.json


echo "All completed successfully!"
goto :EOF

:error
echo "An error occurred!"
exit /b %errorlevel%
