# Make file to generate documentation

SOURCE     = source
OUT        = build
LINKCHECKDIR  = $(OUT)/linkcheck
PYTHON := python3
ifeq ($(OS),Windows_NT)
    PYTHON := python
endif
BUILD      = $(PYTHON) -m sphinx
OPTS       =-c . -W # Treat warnings as errors

DICTIONARIES := codespell_dictionary.txt codespell_whitelist.txt

help:
	@$(BUILD) -M help "$(SOURCE)" "$(OUT)" $(OPTS)
	@echo "  multiversion to build documentation for all branches"

multiversion: Makefile
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=jazzy/index.html\" /></head></html>" > build/html/index.html
	$(PYTHON) make_sitemapindex.py

%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)

lint:
	./sphinx-lint-with-ros source

test:
	doc8 --ignore D001 --ignore-path build

spellcheck:
	git ls-files '*.md' '*.rst' | xargs codespell --config codespell.cfg

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

.PHONY: help Makefile multiversion test linkcheck lint spellcheck check-dictionaries sort-dictionaries
