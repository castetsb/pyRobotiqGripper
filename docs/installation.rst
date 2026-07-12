Installation
============

Install the pyRobotiqGripper python package using pip or uv.

.. youtube:: TuXBFMDqD1g

We recommend using `uv <https://docs.astral.sh/uv/>`_, a fast, modern Python
package and project manager, but a traditional pip installation works just
as well.

Using uv (recommended)
-----------------------

.. note::
    If not already installed, follow the instructions to install uv from the
    official documentation:
    https://docs.astral.sh/uv/getting-started/installation/

``uv add`` requires a uv-managed project, i.e. a directory with a
``pyproject.toml``. If your project doesn't have one yet, create it first:

.. code-block:: bash

    # in a new, empty folder
    uv init my-project
    cd my-project

    # or, in an existing folder that already has code
    cd my-existing-project
    uv init

Then add the package:

.. code-block:: bash

    uv add "pyRobotiqGripper[all]"

Otherwise, uv can also be used as a fast drop-in replacement for pip inside
a virtual environment:

.. code-block:: bash

    uv venv
    uv pip install "pyRobotiqGripper[all]"

Using pip
---------

.. code-block:: bash

    python -m venv .venv

    # Windows
    .venv\Scripts\activate
    # macOS/Linux
    source .venv/bin/activate

    pip install "pyRobotiqGripper[all]"

The `all` extra includes the optional dependencies used by the joystick CLI and \
history/data helper methods.
