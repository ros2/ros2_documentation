# Make file to generate documentation

SOURCE     = source
BUILD      = build
BUILD      = sphinx-build
OPTS       =-c .

help:
	@$(BUILD) -M help "$(SOURCE)" "$(BUILD)" $(OPTS) $(O)

.PHONY: help Makefile
%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(BUILD)" $(OPTS) $(O)
