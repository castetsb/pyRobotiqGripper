# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys

# Get the project root dir, which is the parent dir of this
cwd = os.getcwd()
project_root = os.path.dirname(cwd)

# Insert the project root dir as the first element in the PYTHONPATH.
# This lets us ensure that the source package is imported, and that its
# version is used.
sys.path.insert(0, project_root)

# Import version from package
try:
    from pyrobotiqgripper import __version__, __author__, __email__, __license__, __url__, __project__
    release = __version__
    version=__version__
    project = __project__
    copyright = '2026, Benoit CASTETS'
    author = __author__

except ImportError:
    raise ImportError("Could not import pyrobotiqgripper package. Make sure it is installed and that the PYTHONPATH is set correctly.")

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.duration',
    'sphinx.ext.napoleon',  # Support Google/NumPy docstring styles
    "sphinxcontrib.youtube",
]

# Autodoc configuration
autodoc_default_options = {
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
}
autodoc_typehints = 'description'
autodoc_typehints_format = 'short'

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# build even without pandas or numpy
autodoc_mock_imports = ["pandas", "numpy"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

"""
Tells the project to use sphinx pygments for color coding code examples.
"""

pygments_style = 'sphinx'

# Do not prepend module name to functions/classes in the TOC/sidebar
add_module_names = False

#html_css_files = [
#    "custom.css",
#]