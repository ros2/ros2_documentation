# Make file to generate documentation

SOURCE     = source
OUT        = build
BUILD      = python3 -m sphinx
OPTS       =-c .

help:
	@$(BUILD) -M help "$(SOURCE)" "$(OUT)" $(OPTS)

<<<<<<< HEAD
.PHONY: help Makefile
=======
multiversion: Makefile
	sphinx-multiversion $(OPTS) "$(SOURCE)" build/html
	@echo "<html><head><meta http-equiv=\"refresh\" content=\"0; url=galactic/index.html\" /></head></html>" > build/html/index.html
	python3 make_sitemapindex.py

.PHONY: help Makefile multiversion
>>>>>>> fc8a4a5 (Add sitemap (#2261))
%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)

test:
	doc8 --ignore D001 --ignore-path build
