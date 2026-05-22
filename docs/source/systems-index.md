# Systems

This section describes what systems are present on Perseus, what they are and what they do. For information how to develop each of these systems check their corresponding pages in this section.

Fundamentally, the rover is split into two main sub-systems: Hardware, and Software.
The [_software_ system](project:/systems/software-index.md) lays out how the _code_ interacts with itself and its environment, as well as which bits do what.
The [_hardware_ system](project:/systems/hardware-index.md) goes over how everything's _physically_ connected and laid out, as well as the electrical wiring.

Whilst those two documents contain the majority of the information you'll need day-to-day, there are also some other moving parts to consider as well:

- The [CI/CD](project:/systems/ci-cd.md) pipeline is what runs our automated testing and handles deploying this very website
- Speaking of this website, its build process can be a little bit convoluted, so it gets a [document](project:/systems/documentation.md) too

% A lite-specific system-architecture diagram is TBD; the upstream
% perseus-v2 diagram (CAN bus + VESC topology) does not represent the
% lite robot's hardware.

```{toctree}
:maxdepth: 1
:titlesonly:
:hidden:
:glob:
systems/hardware-index
systems/software-index
systems/*
```
