# Make file to generate documentation

SOURCE     = source
OUT        = build
LINKCHECKDIR  = $(OUT)/linkcheck
PYTHON := python3
ifeq ($(OS),Windows_NT)
    PYTHON := python
endif
BASH := bash
BUILD      = $(PYTHON) -m sphinx
OPTS       =-c . -W # Treat warnings as errors
LIVE_HOST  ?= 0.0.0.0
LIVE_PORT  ?= 2022

TOOLS_DIR     ?= tools
DIFF_BASE     ?=
STATUS_FILE   ?=
PR_NUMBER     ?=
REPOSITORY    ?=

DICTIONARIES := codespell_dictionary.txt codespell_whitelist.txt

help:
	@$(BUILD) -M help "$(SOURCE)" "$(OUT)" $(OPTS)
	@echo "  multiversion to build documentation for all branches"

multiversion: Makefile
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=lyrical/index.html\" /></head></html>" > build/html/index.html
	$(PYTHON) make_sitemapindex.py

%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)

lint:
	./sphinx-lint-with-ros source

test:
	doc8 --ignore D001  --ignore-path $(OUT) -- $(SOURCE)

test-tools:
	$(PYTHON) -m pytest test/

spellcheck:
	git ls-files '*.md' '*.rst' | xargs codespell --config codespell.cfg

ensure-meta-tags:
ifndef DIFF_BASE
	$(error DIFF_BASE is required)
endif
ifndef STATUS_FILE
	$(error STATUS_FILE is required)
endif
	$(PYTHON) $(TOOLS_DIR)/ensure_meta_tags.py \
	  --config $(TOOLS_DIR)/meta_tags.yaml \
	  --diff-base $(DIFF_BASE) \
	  --status-file $(STATUS_FILE)

supersede-meta-tag-reviews:
ifndef PR_NUMBER
	$(error PR_NUMBER is required)
endif
ifndef REPOSITORY
	$(error REPOSITORY is required)
endif
	$(BASH) $(TOOLS_DIR)/supersede_meta_tag_reviews.sh

check-dictionaries:
	@echo "Checking dictionaries..."
	@for dict in $(DICTIONARIES); do \
		echo "Checking $$dict..."; \
		if grep -E -n "^\s*$$|\s$$|^\s" $$dict; then \
			echo "Dictionary $$dict contains empty lines or leading/trailing spaces, triming..."; \
			sed -E -i.bak -e 's/^[[:space:]]+//; s/[[:space:]]+$$//; /^$$/d' $$dict && rm $$dict.bak; \
		fi; \
	done

sort-dictionaries:
	@echo "Sorting dictionaries..."
	@for dict in $(DICTIONARIES); do \
		echo "Sorting $$dict..."; \
		if ! LC_ALL=C sort -f -b -c $$dict; then \
			echo "Dictionary $$dict is not sorted, sorting..."; \
			LC_ALL=C sort -f -b -o $$dict $$dict; \
		fi; \
	done

linkcheck:
	$(BUILD) -b linkcheck $(OPTS) $(SOURCE) $(LINKCHECKDIR)
	@echo
	@echo "Check finished. Report is in $(LINKCHECKDIR)."

serve:
	sphinx-autobuild --host $(LIVE_HOST) --port $(LIVE_PORT) -c . $(SOURCE) $(OUT)/html

.PHONY: help Makefile multiversion test test-tools linkcheck serve lint spellcheck check-dictionaries sort-dictionaries ensure-meta-tags supersede-meta-tag-reviews $(MAKEFILE_LIST)
