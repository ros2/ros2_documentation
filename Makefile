# Make file to generate documentation

SOURCE     = source
OUT        = build
BUILD      = sphinx-build
OPTS       =-c .

help:
	@$(BUILD) -M help "$(SOURCE)" "$(OUT)" $(OPTS) $(O)

.PHONY: help Makefile
%: Makefile
	@$(BUILD) -M $@ "$(SOURCE)" "$(OUT)" $(OPTS) $(O)
