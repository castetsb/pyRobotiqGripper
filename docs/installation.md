# Installation

Install the pyRobotiqGripper python package using pip or uv.

<div class="video-wrapper">
  <iframe src="https://www.youtube.com/embed/TuXBFMDqD1g" title="YouTube video player" allowfullscreen></iframe>
</div>

We recommend using [uv](https://docs.astral.sh/uv/), a fast, modern Python
package and project manager, but a traditional pip installation works just
as well.

## 1. Using uv (recommended)

If you don't already have a pyproject.toml in you project folder, use UV to
create it.

```bash
uv init
```

Install pyrobotiqgripper with minimal dependencies.

```bash
uv add pyrobotiqgripper
```

Install pyrobotiqgripper with all dependencies to have extra features like:

- Gripper command and status history accessible as a pandas table
- A CLI (Command Line Tool) to control the gripper with a mouse or a joystick.

```bash
uv add "pyrobotiqgripper[all]"
```

!!! note
    uv automatically create a virtual environment in the project folder ./.venv

    If not already installed, follow the instructions to install uv from the
    official documentation:
    https://docs.astral.sh/uv/getting-started/installation/

    `uv add` requires a uv-managed project, i.e. a directory with a
    `pyproject.toml`. If your project doesn't have one yet, create it first:

    ```bash
    uv init
    ```

## 2. Using pip

It is good practice to work in a virtual environment.
Here below are instructions to create and activate virtual environment to
install pyrobotiq gripper in it.

```bash
#Create a virtual environment ./.venv
python -m venv .venv
```

If you are on Windows use the following command to activate the virtual environment

```bash
.venv\Scripts\activate
```

If you are on macOS/Linux use the following command to activate the virtual environment

```bash
source .venv/bin/activate
```

Install pyrobotiqgripper with minimal dependencies.

```bash
# base install: gripper control only
pip install pyRobotiqGripper
```

Install pyrobotiqgripper with all dependencies to have extra features like:
- Gripper command and status history accessible as a pandas table
- A CLI (Command Line Tool) to control the gripper with a mouse or a joystick.

```bash
# or, with optional features (joystick CLI, history DataFrame helpers)
pip install "pyRobotiqGripper[all]"
```
