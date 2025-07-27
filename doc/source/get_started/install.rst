.. _install:

Install
=======

IR-SIM supports Python 3.9+ on Linux, macOS, and Windows. You can use
pip or conda for installation.

.. card:: Instructions

    .. tab:: pip

        Install IR-SIM using `pip`_:

        ::

            pip install ir-sim

        This will install the package and core dependencies for the base environment.

    .. tab:: conda

        `conda`_ is a system for package and environment management.

        1. Install `conda`_.

        2. Create a new conda environment,
        ::

            conda create --name irsim_env
            conda activate irsim_env

        or activate an existing one

        3. Install ``ir-sim`` in the virtual environment
        ::

            pip install ir-sim

    .. tab:: uv

        `uv`_ is an extremely fast Python package and project manager.

        Install IR-SIM using `uv`_:
        
        ::

            uv pip install ir-sim
        
        or use uv's project management:

        ::

            uv add ir-sim

    .. tab:: Install from source

        We strongly recommend using a fresh virtual environment (uv or conda) when installing IR-SIM from source.

        Perform the following steps to install IR-SIM from source:

        1. Clone the official `IR-SIM git repository`_, or a newly minted fork of the IR-SIM repository.

        ::

            git clone https://github.com/hanruihua/ir-sim.git

        2. Navigate to the top-level of the cloned directory.

        ::

            cd ir-sim

        3. If you want to use IR-SIM with editable source code, run

        ::

            pip install -e .

        otherwise, run
        
        ::

            pip install .

        or for `uv`_ users, run

        ::

            uv sync
        
Install with Additional Features
---------------------------------

.. dropdown:: Keyboard Control
    :open:

    IR-SIM supports keyboard control for interactive robot simulation.
    Install the keyboard control dependencies:

    ::

        pip install ir-sim[keyboard]

    This installs:
    
    * `pynput`_ - For keyboard and mouse input handling
    * `tabulate`_ - For formatted table output

.. dropdown:: Testing

    IR-SIM comes with a comprehensive test suite.
    Install the testing dependencies:

    ::

        pip install ir-sim[test]

    This installs:
    
    * `pytest`_ - Testing framework
    * `pytest-cov`_ - Coverage reporting

.. dropdown:: Linting

    IR-SIM uses `black`_ for linting.

    ::

        pip install ir-sim[lint]

    This installs:

    * `black`_ - Linting tool

.. dropdown:: All Features

    To install all optional dependencies and features:

    ::

        pip install ir-sim[all]

    This includes:
    
    * Keyboard control features (`pynput`, `tabulate`)
    * Testing framework (`pytest`, `pytest-cov`)
    * Enhanced video support (`imageio[ffmpeg]`)
    * Linting (`black`)

Running the test suite
-----------------------
IR-SIM comes with a comprehensive test suite, which can be run after installing `pytest`_.
If installed from source, navigate to the root of the repository and run

::

    pytest

.. _uv: https://docs.astral.sh/uv/
.. _conda: https://docs.conda.io/en/latest/
.. _Matplotlib: https://matplotlib.org/
.. _Shapely: https://shapely.readthedocs.io/
.. _NumPy: https://www.numpy.org/
.. _PyYAML: https://pyyaml.org/
.. _ImageIO: https://imageio.readthedocs.io/
.. _Loguru: https://loguru.readthedocs.io/
.. _SciPy: https://www.scipy.org/
.. _pytest: https://docs.pytest.org/en/latest/
.. _pynput: https://pypi.org/project/pynput/
.. _tabulate: https://pypi.org/project/tabulate/
.. _IR-SIM git repository: https://github.com/hanruihua/ir-sim
.. _pip: https://pip.pypa.io/
.. _pyproject.toml: https://github.com/hanruihua/ir-sim/blob/main/pyproject.toml
.. _pytest-cov: https://pytest-cov.readthedocs.io/




