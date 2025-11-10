## Building ROVTIO

### Installing dependencies:

From `/external/smores_drone_software`:

Build kindr: `cmake -S include/src/kindr/ -B include/build/kindr/ -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=include/install/kindr/`

Install kindr: `cmake --install include/build/kindr/ --config Release`

Build lightweight_filtering: `cmake -S include/src/lightweight_filtering/ -B include/build/lightweight_filtering/ -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=include/install/lightweight_filtering/`

Install lightweight_filtering: `cmake --install include/build/lightweight_filtering/ --config Release`

Run:

```bash
mkdir -p include/install/lightweight_filtering/share && cat > include/install/lightweight_filtering/share/Findlightweight_filtering.cmake << EOF
# This file was automatically generated during the installation of the lightweight_filtering library
# and can be used through cmake to find the corresponding header files. A copy of this
# file was created in /usr/share/cmake/Modules (depending on the CMAKE_ROOT variable).
set(lightweight_filtering_INCLUDE_DIRS $(pwd)/include/install/lightweight_filtering/include/LWF/include )
set(lightweight_filtering_FOUND TRUE)
message("-- LWF found (include: $(pwd)/include/install/lightweight_filtering/include/LWF/include)")
EOF
```

### Building ROVTIO

Run:

```bash
colcon build --symlink-install --packages-select rovtio --event-handlers console_direct+ --cmake-force-configure --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error -std=gnu++17" -DCMAKE_C_FLAGS="-Wno-error -std=gnu17" -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_PREFIX_PATH="$(pwd)/include/install/kindr/share/:$(pwd)/include/install/lightweight_filtering/share/" -DCMAKE_MODULE_PATH="$(pwd)/include/install/lightweight_filtering/share/" -Wno-dev -DROVIO_NCAM=2 -DROVIO_NMAXFEATURE=8
```

### Launching ROVTIO

Latest configs are under `src/rovtio/cfg/current/`

Launch using: `ros2 launch src/rovtio/cfg/current/rovtio.phxpro.launch.py`

Optionally append `> /dev/null` to stop logging.

### Important

Touch grass.

### Build on MacOS

```bash
colcon build --symlink-install --packages-select rovtio --event-handlers \
      console_direct+ --cmake-args -DCMAKE_C_COMPILER=/usr/bin/clang \
      -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DCMAKE_CXX_FLAGS="-std=c++17 \
      -Wno-error" -DCMAKE_C_FLAGS="-std=c17" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DCMAKE_PREFIX_PATH="/Users/aayush/smores/smores_drone_software/include/install/kindr/share/:/Users/aayush/smores/smores_drone_software/include/install/lightweight_filtering/share/opt/homebrew" \
      -DCMAKE_MODULE_PATH="/Users/aayush/smores/smores_drone_software/include/install/lightweight_filtering/share/" \
      -DCMAKE_SHARED_LINKER_FLAGS="-L/opt/homebrew/lib -lglog" \
      -DCMAKE_EXE_LINKER_FLAGS="-L/opt/homebrew/lib -lglog" -Wno-dev \
      -DROVIO_NCAM=2 -DROVIO_NMAXFEATURE=8 \
      -DPython3_EXECUTABLE=/Users/aayush/ros2/venv/bin/python \
      -DPYTHON_EXECUTABLE=/Users/aayush/ros2/venv/bin/python \
      -DPython3_FIND_STRATEGY=LOCATION -DPYTHON_CONFIG_EXECUTABLE=/opt/homebrew/opt/python@3.9/bin/python3.9-config
```
