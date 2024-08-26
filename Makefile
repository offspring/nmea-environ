CLANG_FORMAT:=clang-format

SRCFILES:=$(shell find $(CURDIR)/src/ -name '*.cpp' -o -name '*.h')

all:

.PHONY: dos2unix
dos2unix:
	dos2unix $(SRCFILES)

.PHONY: clang-format
clang-format:
	$(CLANG_FORMAT) -i --verbose -style=file $(SRCFILES)
