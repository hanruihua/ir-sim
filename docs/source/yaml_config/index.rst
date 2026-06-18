YAML Configuration Syntax
=============================

Every IR-SIM scenario is a single YAML file with up to four top-level blocks:

- ``world`` — the simulation space, clock, coordinate frame, and obstacle map.
- ``robot`` — one or more controllable robots (kinematics, shape, goal, behavior, sensors).
- ``obstacle`` — static or dynamic obstacles. Robots and obstacles share the **same object schema**, so any key valid for one is valid for the other.
- ``gui`` — optional rendering and interaction options.

The full reference below documents every key, with its default, and includes an interactive schema explorer.

.. toctree::
   :maxdepth: 2

   configuration
   




