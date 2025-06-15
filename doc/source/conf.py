# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))
import os
import sys
from unittest.mock import MagicMock
import importlib.metadata

class Mock(MagicMock):
    @classmethod
    def __getattr__(cls, name):
        return MagicMock()

MOCK_MODULES = ['pynput', 'loguru']
sys.modules.update((mod_name, Mock()) for mod_name in MOCK_MODULES)

os.environ['PYNPUT_BACKEND'] = 'dummy'

# Rest of your conf.py content

# -- Project information -----------------------------------------------------

project = 'IR-SIM'
copyright = '2024, Ruihua Han'
author = 'Ruihua Han'

# The full version, including alpha/beta/rc tags
release = importlib.metadata.version("ir-sim")

# print(os.path.abspath('../../'))
# sys.path.insert(0, os.path.abspath('../../'))
# sys.path.insert(0, os.path.abspath("../.."))
# sys.path.insert(0, os.path.abspath("./"))
# sys.path.insert(0, os.path.join(os.path.dirname((os.path.abspath('.')), 'irsim')))

# root_path = os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
# sys.path.insert(0, os.path.dirname(__file__))

# sys.path.insert(0, os.path.abspath("../.."))
# print(os.path.abspath('./'))
# sys.path.insert(0, os.path.abspath('./'))
# sys.path.insert(0, os.path.abspath('../src'))
# sys.path.append(os.path.abspath('../../'))
# sys.path.insert(0, os.path.abspath(".."))

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.viewcode', 'sphinx.ext.napoleon', 'myst_parser', 'sphinx_multiversion', 'sphinx_copybutton'
]

myst_enable_extensions = [
    "amsmath",
    "deflist",
    "html_admonition",
    "html_image",
    "colon_fence",
    # Add other extensions as needed
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# root_doc = 'irsim'
# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# html_theme = "sphinx_rtd_theme"
html_theme = "pydata_sphinx_theme"

autodoc_member_order = 'bysource'

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# myst_enable_extensions = [
#     "amsmath",
#     "attrs_inline",
#     "colon_fence",
#     "deflist",
#     "dollarmath",
#     "fieldlist",
#     "html_admonition",
#     "html_image",
#     "replacements",
#     "smartquotes",
#     "strikethrough",
#     "substitution",
#     "tasklist",
# ]

# json_url = "https://ir-sim.readthedocs.io/en/dev/_static/switcher.json"

# html_theme_options = {
#     "switcher": {
#         "json_url": json_url,
#         "version_match": release,
#     },
# }


def setup(app):
    app.add_css_file('my_theme.css')