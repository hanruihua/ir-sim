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
import os
import sys
# Add the project root to Python path
sys.path.insert(0, os.path.abspath('../../'))
sys.path.insert(0, os.path.abspath('.'))
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

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc', 
    'autoapi.extension',  # Add AutoAPI for automatic API documentation
    'sphinx.ext.viewcode', 
    'sphinx.ext.napoleon', 
    'myst_parser', 
    'sphinx_multiversion', 
    'sphinx_copybutton', 
    'sphinx_design', 
    'sphinx_inline_tabs',
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

# Suppress specific warnings - using regex patterns to catch duplicate warnings
suppress_warnings = [
    'ref.python',  # Suppress python reference warnings including duplicates
    'autosummary',  # Suppress autosummary warnings  
    'autodoc.import_object',  # Suppress autodoc import warnings
    'autoapi.python_import_resolution',  # Suppress AutoAPI import resolution warnings
    'autodoc.duplicate_object',  # Suppress duplicate object description warnings
    'app.add_directive',  # Suppress app directive warnings
    # Add pattern to suppress all duplicate warnings
    'sphinx.domains.python',  # Suppress Python domain warnings including duplicates
    'toc.not_readable',  # Suppress toctree warnings
]

# Ignore nitpicky warnings for duplicates
nitpicky = False
nitpick_ignore = [
    ('py:obj', 'irsim.world.object_base.ObjectBase.state_shape'),
    ('py:obj', 'irsim.world.object_base.ObjectBase.vel_shape'),
    ('py:obj', 'irsim.world.object_base.ObjectBase.state'),
    ('py:obj', 'irsim.world.object_base.ObjectBase.wheelbase'),
    # Add GUI duplicates
    ('py:obj', 'irsim.gui.MouseControl.mouse_pos'),
    ('py:obj', 'irsim.gui.MouseControl.left_click_pos'),
    ('py:obj', 'irsim.gui.MouseControl.right_click_pos'),
]

# AutoAPI configuration - Automatic API documentation generation
autoapi_type = 'python'

# Try to find the irsim package - handle both local and ReadTheDocs environments
_current_dir = os.path.dirname(__file__) if '__file__' in globals() else os.getcwd()


autoapi_dirs = ['irsim']

# If source directory not found, try installed package
if autoapi_dirs is None:
    try:
        import irsim
        irsim_path = os.path.dirname(irsim.__file__)
        if os.path.exists(irsim_path) and os.listdir(irsim_path):
            autoapi_dirs = [irsim_path]
            print(f"Using installed package at: {irsim_path}")
    except ImportError as e:
        print(f"Could not import irsim package: {e}")

autoapi_root = 'api'  # Directory where API docs will be generated
autoapi_keep_files = True  # Keep generated files for inspection
autoapi_add_toctree_entry = False  # Don't automatically add to main toctree
autoapi_generate_api_docs = True  # Generate API documentation
autoapi_python_class_content = 'both'  # Include both class and __init__ docstrings
autoapi_ignore = [
    # Ignore problematic modules that cause import issues
    '**/test_*',
    '**/tests/*',
    '**/*test*',
    # Ignore specific usage scripts that create standalone modules
    '**/usage/01empty_world/**',
    '**/usage/02robot_world/**',
    '**/usage/03obstacle_world/**',
    '**/usage/04collision_world/**',
    '**/usage/05lidar_world/**',
    '**/usage/06multi_objects_world/**',
    '**/usage/07render_world/**',
    '**/usage/08random_obstacle/**',
    '**/usage/09keyboard_control/**',
    '**/usage/10grid_map/**',
    '**/usage/11collision_avoidance/**',
    '**/usage/12dynamic_obstacle/**',
    '**/usage/13custom_behavior/**',
    '**/usage/14world_3d_plot/**',
    '**/usage/15fov_world/**',
    '**/usage/16noise_world/**',
    '**/usage/17gui_world/**',
    # Ignore version module
    '**/version.py',
]
autoapi_options = [
    'members',
    'undoc-members', 
    'show-inheritance',
    'show-module-summary',
    'imported-members',
]

# Configure AutoAPI to be more tolerant of import errors
autoapi_python_use_implicit_namespaces = True

# Configure AutoAPI to handle import errors gracefully
autoapi_ignore_import_errors = True

# Add error handling for AutoAPI
def autoapi_skip_member(app, what, name, obj, skip, options):
    """Skip problematic members that cause import issues."""
    try:
        # Skip private members that start with underscore
        if name.startswith('_') and not name.startswith('__'):
            return True
        # Skip test functions and classes
        if 'test' in name.lower():
            return True
        return skip
    except Exception as e:
        print(f"Error in autoapi_skip_member for {name}: {e}")
        return True  # Skip if there's any error

def setup(app):
    """Setup function for Sphinx."""
    app.connect('autoapi-skip-member', autoapi_skip_member)

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

# html_sidebars = {
#     "index": ["sidebar-nav-bs"],  # Homepage without search button in sidebar
#     "**": ["search-button-field", "sidebar-nav-bs"]  # All other pages with search button in sidebar
# }

html_sidebars = {
    "index": ["sidebar-nav-bs", "search-button-field"],  # Homepage: no search in sidebar (will be in navbar)
    "**": ["sidebar-nav-bs", "search-button-field"],  # Other pages: search in sidebar
    "community/index": [
        "sidebar-nav-bs",
        "custom-template",
    ],  # This ensures we test for custom sidebars
}

html_sidebars = {
    "**": ["sidebar-nav-bs"]  # Use default navigation for all pages
}

html_js_files = [
    ("custom-icons.js", {"defer": "defer"}),
]

autodoc_member_order = 'bysource'

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

html_theme_options = {
    "icon_links": [
      {
        "name": "GitHub",
        "url": "https://github.com/hanruihua/ir-sim",
        "icon": "fa-brands fa-github",
      },
      {
        "name": "PyPI",
        "url": "https://pypi.org/project/ir-sim/",
        "icon": "fa-custom fa-pypi",
    }
    ],
    # "logo": {
    #     "text": "IR-SIM",
    # },
    "navbar_start": ["navbar-logo"],
    "navbar_end": ["version-switcher", "theme-switcher", "navbar-icon-links"],
    "switcher": {
        "json_url": 'doc/source/_static/switcher.json',
        "version_match": release,
    },
    # Primary sidebar navigation configuration
    "show_nav_level": 3,  # Show top-level pages and their immediate children by default
    "navigation_depth": 4,  # Maximum levels shown in sidebar (default: 4)
    "collapse_navigation": False,  # Keep expandable navigation enabled
    # Secondary sidebar configuration
    "secondary_sidebar_items": ["page-toc", "edit-this-page", "sourcelink"],
    # Show more levels in the table of contents by default
    "show_toc_level": 3,  # Shows headings up to level 2 by default
    # "navbar_persistent": [],  # Enable search in navbar
}


def setup(app):
    app.add_css_file('my_theme.css')
    
    # Filter out duplicate object warnings using logging
    import logging
    class DuplicateWarningFilter(logging.Filter):
        def filter(self, record):
            # Suppress duplicate object description warnings
            if hasattr(record, 'msg') and 'duplicate object description' in str(record.msg):
                return False
            if hasattr(record, 'message') and 'duplicate object description' in str(record.message):
                return False
            return True
    
    # Apply filter to sphinx logger
    logging.getLogger('sphinx').addFilter(DuplicateWarningFilter())
    
    # Add custom JavaScript to handle conditional search button display
    # app.add_js_file('conditional-search.js')