# Software

The software on the rover is split into the core software (mainly ROS2 code) and the web control UI.
Since the web UI runs on a completely different stack to the rest of the rover code, it just made more sense to separate it out and treat it separately.
:::{tip}
If you want to read through project usage, that'll be documented either in the source code (particularly for libraries) and as such be visible in the [generated documentation](project:/generated/exhale/index.rst), or in a README file in the project's directory.
:::

```{toctree}
:glob:
:hidden:
software/*
```
