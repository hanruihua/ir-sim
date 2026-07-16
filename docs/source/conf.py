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
import warnings

# Add the project root to Python path
sys.path.insert(0, os.path.abspath("../../"))
sys.path.insert(0, os.path.abspath("."))
import importlib.metadata
from importlib.metadata import PackageNotFoundError
from unittest.mock import MagicMock


class Mock(MagicMock):
    @classmethod
    def __getattr__(cls, name):
        return MagicMock()


MOCK_MODULES = ["pynput", "loguru"]
sys.modules.update((mod_name, Mock()) for mod_name in MOCK_MODULES)

os.environ["PYNPUT_BACKEND"] = "dummy"

# Rest of your conf.py content

# -- Project information -----------------------------------------------------

project = "IR-SIM"
copyright = "2024, Ruihua Han"
author = "Ruihua Han"

language = 'en'

locale_dirs = ['../locale/']
gettext_compact = False

templates_path = ['_templates']
html_context = {
    'display_language_switch': True,
}

# The full version, including alpha/beta/rc tags
try:
    release = importlib.metadata.version("ir-sim")
except PackageNotFoundError:
    release = os.environ.get("IR_SIM_DOC_VERSION", "local")

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "autoapi.extension",
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx.ext.napoleon",
    "myst_parser",
    "sphinx_copybutton",
    "sphinx_design",
    "sphinx_inline_tabs",
    "sphinxext.opengraph",
    "sphinx_sitemap",
]

ENABLE_AUTOAPI = True

# Render Google-style "Attributes:" sections as :ivar: fields instead of
# standalone ".. attribute::" directives. The latter collide with the members
# AutoAPI documents for the same class, producing "duplicate object
# description" warnings; :ivar: keeps the descriptions without a second target.
napoleon_use_ivar = True

myst_enable_extensions = [
    "amsmath",
    "deflist",
    "html_admonition",
    "html_image",
    "colon_fence",
    # Add other extensions as needed
]

# Auto-generate slug anchors for h1–h4 so in-page/cross-doc
# `[text](page.md#heading-slug)` links resolve consistently.
myst_heading_anchors = 4

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# Suppress specific warnings - using regex patterns to catch duplicate warnings
suppress_warnings = [
    "ref.python",  # Suppress python reference warnings including duplicates
    "autosummary",  # Suppress autosummary warnings
    "autodoc.import_object",  # Suppress autodoc import warnings
    "autoapi.python_import_resolution",  # Suppress AutoAPI import resolution warnings
    "autodoc.duplicate_object",  # Suppress duplicate object description warnings
    "app.add_directive",  # Suppress app directive warnings
    # Add pattern to suppress all duplicate warnings
    "sphinx.domains.python",  # Suppress Python domain warnings including duplicates
    "toc.not_readable",  # Suppress toctree warnings
]

# Ignore nitpicky warnings for duplicates
nitpicky = False
nitpick_ignore = [
    ("py:obj", "irsim.world.object_base.ObjectBase.state_shape"),
    ("py:obj", "irsim.world.object_base.ObjectBase.vel_shape"),
    ("py:obj", "irsim.world.object_base.ObjectBase.state"),
    ("py:obj", "irsim.world.object_base.ObjectBase.wheelbase"),
    # Add GUI duplicates
    ("py:obj", "irsim.gui.MouseControl.mouse_pos"),
    ("py:obj", "irsim.gui.MouseControl.left_click_pos"),
    ("py:obj", "irsim.gui.MouseControl.right_click_pos"),
]

# AutoAPI configuration - Automatic API documentation generation
autoapi_type = "python"
autoapi_dirs = ["../../irsim"]  # Path to your source code
autoapi_root = "api"  # Directory where API docs will be generated
autoapi_keep_files = True  # Keep generated files for inspection
autoapi_add_toctree_entry = False  # Don't automatically add to main toctree
autoapi_generate_api_docs = True  # Generate API documentation
autoapi_python_class_content = "both"  # Include both class and __init__ docstrings
autoapi_ignore = [
    # Ignore problematic modules that cause import issues
    "**/tests/*",
    # Ignore specific usage scripts that create standalone modules
    "**/usage/**",
    # Ignore binary map generator (has external dependencies)
    "**/binary_map_generator_hm3d/**",
    # Ignore version module
    "**/version.py",
]
autoapi_options = [
    "members",
    "undoc-members",
    "show-inheritance",
    "show-module-summary",
    "imported-members",
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
        if name.startswith("_") and not name.startswith("__"):
            return True
        # Skip test functions and classes
        if "test" in name.lower():
            return True
        return skip
    except Exception as e:
        print(f"Error in autoapi_skip_member for {name}: {e}")
        return True  # Skip if there's any error


# root_doc = 'irsim'
# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = 'alabaster'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

# html_theme = "sphinx_rtd_theme"
html_theme = "pydata_sphinx_theme"

# html_sidebars = {
#     "index": ["sidebar-nav-bs"],  # Homepage without search button in sidebar
#     "**": ["search-button-field", "sidebar-nav-bs"]  # All other pages with search button in sidebar
# }

html_sidebars = {
    "index": [
        "sidebar-nav-bs",
        "search-button-field",
    ],  # Homepage: no search in sidebar (will be in navbar)
    "**": ["sidebar-nav-bs", "search-button-field"],  # Other pages: search in sidebar
    "community/index": [
        "sidebar-nav-bs",
        "custom-template",
    ],  # This ensures we test for custom sidebars
}

# Version switcher moved out of the crowded navbar into the sidebar top.
html_sidebars = {
    "**": [
        "version-switcher",
        "navbar-lang-switch",
        "sidebar-nav-bs",
    ]
}

html_js_files = [
    # Loaded early (no defer) so the saved text size applies
    # before first paint — avoids a font-size flash.
    "fontsize-init.js",
    ("custom-icons.js", {"defer": "defer"}),
]

autodoc_member_order = "bysource"

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
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
        },
        {
            "name": "Issues",
            "url": "https://github.com/hanruihua/ir-sim/issues",
            "icon": "fa-solid fa-bug",
        },
    ],
    "logo": {
        "text": "IR-SIM documentation",
    },
    "navbar_start": ["navbar-logo"],
    "navbar_center": ["navbar-nav"],
    "navbar_end": [
        "theme-switcher",
        "navbar-icon-links",
        "font-size-switch",
    ],
    # Keep every top-level project page visible in the desktop navbar.
    "header_links_before_dropdown": 8,
    "switcher": {
        "json_url": "https://raw.githubusercontent.com/hanruihua/ir-sim/main/docs/source/_static/switcher.json",
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
    # Add search button to navbar
    "navbar_persistent": ["search-button"],
    # Web analytics is handled by Read the Docs (enable it in the RTD dashboard:
    # Admin -> Analytics) — no config is needed here. To use Google Analytics or
    # Plausible instead, add e.g.:
    #   "analytics": {"google_analytics_id": "G-XXXXXXXXXX"},
}

# -- SEO and social sharing --------------------------------------------------
# Canonical base URL of the published docs, used for <link rel="canonical">,
# the sitemap, and Open Graph cards. On Read the Docs, READTHEDOCS_CANONICAL_URL
# is set per version *and* language (e.g. .../en/latest/, .../zh-cn/stable/), so
# those stay correct across every build; fall back to the stable English URL for
# local builds.
html_baseurl = os.environ.get(
    "READTHEDOCS_CANONICAL_URL", "https://ir-sim.readthedocs.io/en/stable/"
)

# sphinx-sitemap: emit sitemap.xml so search engines can index the docs.
# {link} appends the page path to html_baseurl (which already carries the
# version/language), so per-version sitemaps are correct.
sitemap_url_scheme = "{link}"

# sphinxext-opengraph: rich link-preview cards when the docs are shared on
# social/chat platforms (X, Slack, Reddit, LinkedIn, ...).
ogp_site_url = html_baseurl
ogp_enable_meta_description = True
ogp_description_length = 200
# Fallback preview image (the signature multi-robot RVO demo) used on pages
# whose first image is externally hosted — most importantly the landing page.
ogp_image = "https://github.com/user-attachments/assets/5930b088-d400-4943-8ded-853c22eae75b"
# Prefer each page's own first image as its preview card when one is available
# (guide pages with local gifs); otherwise fall back to `ogp_image` above.
ogp_use_first_image = True
# NOTE: auto-generated `ogp_social_cards` is intentionally left off — its card
# renderer ships only a Latin font, so Chinese (zh_CN) page titles render as
# tofu. To get branded image cards, bundle a CJK font for the card renderer or
# set a static `ogp_image` banner.


def setup(app):
    """Register custom Sphinx integrations."""
    app.add_css_file("my_theme.css")
    if ENABLE_AUTOAPI:
        app.connect("autoapi-skip-member", autoapi_skip_member)

    # Suppress AutoAPI "duplicate object description" warnings — benign artifacts
    # of documenting a module variable twice (optional-import idiom) or a module
    # and class sharing a name. The filter must live on the *handlers* of the
    # "sphinx" logger: a logger-level filter is NOT applied to records propagated
    # from child loggers (e.g. sphinx.domains.python), which is exactly where
    # these originate — so a logger-level filter leaks them under `-W` /
    # fail_on_warning. Handlers exist by "builder-inited", so install there.
    import logging

    class DuplicateWarningFilter(logging.Filter):
        def filter(self, record):
            try:
                msg = record.getMessage()
            except Exception:
                msg = str(record.msg)
            return "duplicate object description" not in msg

    def _install_duplicate_filter(app, *args):
        flt = DuplicateWarningFilter()
        logger = logging.getLogger("sphinx")
        logger.addFilter(flt)
        for handler in logger.handlers:
            handler.addFilter(flt)

    app.connect("builder-inited", _install_duplicate_filter)

    # Add custom JavaScript to handle conditional search button display
    # app.add_js_file('conditional-search.js')
