# Contributing

Contributions to pyRobotiqGripper are welcome.

## 1. Branch and pull request

Contributions must go through a branch and a pull request: do not commit
directly to `master`.

1. Create a branch for your change.
2. Make your change on that branch.
3. Open a pull request against `master` on
   [GitHub](https://github.com/castetsb/pyRobotiqGripper) describing what
   the change does and why.

## 2. Version bump

Any change intended to be released must bump the package version in
`pyproject.toml`:

```toml
[project]
version = "3.3.3"
```

Follow [semantic versioning](https://semver.org/): increment the patch
number for bug fixes, the minor number for backward-compatible features, and
the major number for breaking changes.

## 3. Update the lock file

After bumping the version (or changing any dependency), regenerate
`uv.lock` so it stays in sync with `pyproject.toml`:

```bash
uv lock --upgrade
```

Commit the updated `uv.lock` together with your change.

## 4. Render the documentation locally

The documentation lives in `docs/` (Markdown rendered by Sphinx with
`myst-parser`) and is published on
[readthedocs](https://pyrobotiqgripper.readthedocs.io/en/latest/). To preview
your changes before opening a pull request:

1. Install the `docs` extra:

   ```bash
   uv sync --extra docs
   ```

2. Build the HTML docs:

   ```bash
   uv run --extra docs sphinx-build -b html docs docs/_build/html
   ```

3. Serve the built site and open it in your browser:

   ```bash
   python -m http.server 8000 --directory docs/_build/html
   ```

   Then browse to <http://localhost:8000/>.

After editing a file, rerun the `sphinx-build` command from step 2 and
refresh the page; there is no need to restart the HTTP server.
