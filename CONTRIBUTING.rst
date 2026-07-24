Contributing
============

Contributions to pyRobotiqGripper are welcome.

1-Branch and pull request
--------------------------

Contributions must go through a branch and a pull request: do not commit
directly to ``master``.

1. Create a branch for your change.
2. Make your change on that branch.
3. Open a pull request against ``master`` on
   `GitHub <https://github.com/castetsb/pyRobotiqGripper>`_ describing what
   the change does and why.

2-Version bump
--------------

Any change intended to be released must bump the package version in
``pyproject.toml``:

.. code-block:: toml

    [project]
    version = "3.3.3"

Follow `semantic versioning <https://semver.org/>`_: increment the patch
number for bug fixes, the minor number for backward-compatible features, and
the major number for breaking changes.

3-Update the lock file
-----------------------

After bumping the version (or changing any dependency), regenerate
``uv.lock`` so it stays in sync with ``pyproject.toml``:

.. code-block:: bash

    uv lock --upgrade

Commit the updated ``uv.lock`` together with your change.