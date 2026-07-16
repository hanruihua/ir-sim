.. _install:

Install
=======

IR-SIM supports Python 3.10+ on Linux, macOS, and Windows. You can use
pip or conda for installation.

.. card:: Instructions

    .. tab:: pip

        Install IR-SIM using `pip`_:

        ::

            python -m pip install ir-sim

        This will install the package and core dependencies for the base environment.

    .. tab:: conda

        `conda`_ is a system for package and environment management.

        1. Install `conda`_.

        2. Create a new conda environment with Python and pip,
        ::

            conda create --name irsim_env python=3.12 pip
            conda activate irsim_env

        or activate an existing one

        3. Install ``ir-sim`` in the virtual environment
        ::

            python -m pip install ir-sim

    .. tab:: uv

        `uv`_ is an extremely fast Python package and project manager.

        Install IR-SIM using `uv`_:
        
        ::

            uv venv
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

            python -m pip install -e .

        otherwise, run
        
        ::

            python -m pip install .

        or for `uv`_ users, run

        ::

            uv sync

Verify the Installation
-----------------------

Using the Python interpreter from the environment where IR-SIM was installed,
print the installed version:

::

    python -c "import irsim; print(irsim.__version__)"

If this command prints a version without an import error, the core installation
is ready. Continue with the :doc:`Quick Start <quick_start>` to run a scene.

Install with Additional Features
---------------------------------

.. dropdown:: Keyboard Control
    :open:

    IR-SIM supports keyboard control for interactive robot simulation.
    Install the keyboard control dependencies:

    ::

        python -m pip install "ir-sim[keyboard]"

    This installs:
    
    * `pynput`_ - For keyboard and mouse input handling


.. dropdown:: All Features

    To install all optional runtime dependencies and features:

    ::

        python -m pip install "ir-sim[all]"

    This includes:

    * Keyboard control (`pynput`)
    * Enhanced video support (`imageio[ffmpeg]`)
    * ORCA group behavior (`pyrvo`)


Developer Setup
---------------

The following dependency groups are only needed when developing IR-SIM itself.
They are not required to run normal simulations.


.. dropdown:: Linting

    IR-SIM uses `Ruff`_ for linting. To install the dependency-groups for linting locally, run

    ::

        uv sync --group lint

    This includes:

    * `Ruff`_ - Linting tool
    * `ty`_ - Type hinting tool
    * `black`_ - Code formatter
    
.. dropdown:: Testing

    Install test dependencies using the test group (from ``pyproject.toml``):

    ::

        uv sync --group test

    Run the tests:

    ::

        uv run pytest

    Generate coverage:

    ::

        uv run pytest --cov . --cov-report=xml --cov-report=html

    Type-check the codebase:

    ::

        uvx ty check


.. dropdown:: Documentation

    Install documentation dependencies using the docs group (from ``pyproject.toml``):

    ::

        uv sync --group docs

    Build the docs locally (HTML):

    ::

        cd docs
        make html

    The output will be available under ``docs/build/html`` (or ``docs/_build/html`` depending on your environment setup).


.. dropdown:: All Development Groups

    To install all development dependency groups:

    ::

        uv sync --all-groups
    
    This includes:

    * lint group
    * test group
    * docs group


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
.. _IR-SIM git repository: https://github.com/hanruihua/ir-sim
.. _pip: https://pip.pypa.io/
.. _pyproject.toml: https://github.com/hanruihua/ir-sim/blob/main/pyproject.toml
.. _pytest-cov: https://pytest-cov.readthedocs.io/
.. _Ruff: https://docs.astral.sh/ruff/
.. _black: https://black.readthedocs.io/
.. _ty: https://docs.astral.sh/ty/


