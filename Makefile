# Make file to generate documentation

SOURCE     = source
OUT        = build
BUILD      = python -m sphinx
OPTS       =-c .

help:
	@$(BUILD) -M help "$(SOURCE)" "$(OUT)" $(OPTS)

.PHONY: help Makefile
%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS)
