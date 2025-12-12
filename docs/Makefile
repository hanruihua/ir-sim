# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = build
LOCALEDIR     ?= locale
I18N_LANGUAGE ?= zh_CN

# Multi-language configuration (override via environment if needed)
LANGUAGES     ?= en zh_CN
DEFAULT_LANG  ?= en

.PHONY: help html html-all gettext translations translations-zh html-i18n $(addprefix html-lang-,$(LANGUAGES))

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

gettext:
	@echo "[sphinx] extracting source strings into gettext catalog"
	@$(SPHINXBUILD) -M gettext "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

update-po: gettext
	@echo "[sphinx-intl] updating translation catalogs for '$(I18N_LANGUAGE)'"
	@sphinx-intl update -p build/gettext -l $(I18N_LANGUAGE)

translations translations-zh:
	@echo "[sphinx-intl] compiling message catalogs for '$(I18N_LANGUAGE)'"
	@sphinx-intl build --locale-dir "$(LOCALEDIR)" --language "$(I18N_LANGUAGE)"

html-main:
	@echo "[sphinx] building default language '$(DEFAULT_LANG)'"
	@$(SPHINXBUILD) -M html "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# html-en:
# 	@echo "[sphinx] building language '$*' into $(BUILDDIR)/$*/html"
# 	@$(SPHINXBUILD) -b html "$(SOURCEDIR)" "$(BUILDDIR)/en"

html-zh: translations-zh
	@echo "[sphinx] building language '$*' into $(BUILDDIR)/$*/html"
	@$(SPHINXBUILD) -b html "$(SOURCEDIR)" "$(BUILDDIR)/$(I18N_LANGUAGE)" -D language=$(I18N_LANGUAGE)

html-i18n: translations-zh
	@$(MAKE) html-zh

html:
	@$(MAKE) html-main
# 	@$(MAKE) html-en
	@$(MAKE) html-zh
.PHONY: Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
