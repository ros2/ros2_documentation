# Make file to generate documentation

SOURCE     = source
OUT        = build
LINKCHECKDIR  = $(OUT)/linkcheck
BUILD      = python3 -m sphinx
LANG 	   = en
OPTS       = -c . -D language=$(LANG)

help:
	@$(BUILD) -M help "$(SOURCE)" "$(OUT)" $(OPTS)
	@echo "  multiversion to build documentation for all branches"

multiversion: Makefile
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html/$(LANG)
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=iron/index.html\" /></head></html>" > build/html/$(LANG)/index.html
	python3 make_sitemapindex.py

%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)

test:
	doc8 --ignore D001 --ignore-path build

linkcheck:
	$(BUILD) -b linkcheck $(OPTS) $(SOURCE) $(LINKCHECKDIR)
	@echo
	@echo "Check finished. Report is in $(LINKCHECKDIR)."

updatepo:
	sphinx-intl update -p "$(OUT)/gettext" -d "locale" -l $(LANG)

html:
	$(BUILD) -b html $(OPTS) $(SOURCE) $(OUT)/html/$(LANG);

.PHONY: help Makefile multiversion test linkcheck
