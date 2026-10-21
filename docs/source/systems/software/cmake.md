# CMake Build Infrastructure

This project's C++ code is actually targeted at being built in two different ways - first via `colcon build` inside the Pixi-managed workspace, and second by using ordinary `cmake` and `make` commands directly, both of which have slightly different environments.

The first major difference between the Pixi-managed and standard builds is handling dependencies.
When the project is built via `colcon build` inside a Pixi environment, all of the ROS/RoboStack dependencies that a target specifies are made available to it like they've been installed (resolved from the `robostack-jazzy`/`conda-forge` channels Pixi manages) and the project can build with no particular extra steps.
However, when built through CMake normally (outside a Pixi environment, or for the bespoke libraries under `software/shared` that aren't packaged for RoboStack), the individual projects need to manually add the subdirectories of their dependencies.
They can detect whether the packages is present (installed) using the `find_package` command, and only attempt to manually include their dependencies if they aren't already available.

The other major change is related to the [build type](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html).
When you run CMake manually, there is by default no `CMAKE_BUILD_TYPE` specified.
In this instance, the projects have some additional config in their `CMakeLists.txt` files to default to the `Debug` release type.
Pixi/colcon builds, by contrast, _specify_ the `RelWithDebInfo` build type: `software/ros_ws/colcon_defaults.yaml` passes `-DCMAKE_BUILD_TYPE=RelWithDebInfo` to every package, and the Pixi `build-test` tasks (which pass their own `--cmake-args`, replacing the defaults-file list) repeat it explicitly.
`RelWithDebInfo` gives optimised binaries for the robot while keeping the debug info needed for useful `backward_ros` backtraces (the CMakeLists also convert a plain `Release` request into `RelWithDebInfo` for the same reason).
The reason the build type matters is that these projects add `-Werror` to the GCC build flags for any `Rel*` build, thus enforcing the [no warnings](project:/standards/software/general.md#warnings-are-not-acceptable) section of the software standards.

:::{warning}
If you pass your own `--cmake-args` to `colcon build`, they _replace_ the defaults file's list, so include `-DCMAKE_BUILD_TYPE=RelWithDebInfo` yourself or you silently get a `Debug` (`-O0`, no `-Werror`) build.
An existing `build/` directory that was configured as `Debug` is switched over on its next `pixi run -e <env> build`, because colcon re-runs CMake whenever the `--cmake-args` change.
:::

The out-of-workspace `software/arm-teleop-direct` project is built the same way by `pixi run -e default build-arm-teleop` (plain `cmake`, `RelWithDebInfo`), which CI also runs.

If you're curious as to what all this looks like in practice, check out the `software/templates/` directory.

:::{note}
Under the hood, `colcon build` invokes CMake (via `ament_cmake`) for each of these projects.
:::

## Testing

Individual projects are unit tested using [GoogleTest](https://google.github.io/googletest/) (AKA GTest).
This is the default testing solution supported for ROS2 projects, as documented [here](inv:ros#Tutorials/Intermediate/Testing/Cpp).
It works well enough that we're also using it for unit testing the shared and native software.

% TODO: Investigate using catch2 instead of GTest - maybe better?
