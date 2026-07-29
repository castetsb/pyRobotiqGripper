# Contribute

Contributions to pyRobotiqGripper are welcome.

## Branch and pull request

Contributions must go through a branch and a pull request: do not commit
directly to `master`.

1. Create a branch for your change.
2. Make your change on that branch.
3. Open a pull request against `master` on
   [GitHub](https://github.com/castetsb/pyRobotiqGripper) describing what
   the change does and why.

## Version bump

Any change intended to be released must bump the package version in
`pyproject.toml`:

```toml
[project]
version = "3.3.3"
```

Follow [semantic versioning](https://semver.org/): increment the patch
number for bug fixes, the minor number for backward-compatible features, and
the major number for breaking changes.

## Update the lock file

After bumping the version (or changing any dependency), regenerate
`uv.lock` so it stays in sync with `pyproject.toml`:

```bash
uv lock --upgrade
```

Commit the updated `uv.lock` together with your change.

## Render the documentation locally

The documentation lives in `docs/` (Markdown rendered by
[MkDocs](https://www.mkdocs.org/) with the
[Material](https://squidfunk.github.io/mkdocs-material/) theme and
[mkdocstrings](https://mkdocstrings.github.io/) for the API reference) and is
published on
[readthedocs](https://pyrobotiqgripper.readthedocs.io/en/latest/). To preview
your changes before opening a pull request:

1. Install the `docs` extra:

   ```bash
   uv sync --extra docs
   ```

2. Serve the site locally with live-reload -- it watches `docs/`, `mkdocs.yml`
   and the package source, and rebuilds on every save:

   ```bash
   uv run --extra docs mkdocs serve
   ```

   Then browse to <http://localhost:8000/>. Stop it with Ctrl+C.

Alternatively, build a static copy of the site into `site/`:

```bash
uv run --extra docs mkdocs build
```
