SHELL := bash
.ONESHELL:
.SHELLFLAGS := -eu -o pipefail -c
.DELETE_ON_ERROR:
MAKEFLAGS += --warn-undefined-variables
MAKEFLAGS += --no-builtin-rules
MAKEFLAGS += --no-silent


format: clang-format

clang-format:

lint: clang-tidy


clang-tidy:


cppcheck:

xacro:


.PHONY: format clang-format xacro clang-tidy cppcheck lint


# default: format shellharden add-commit-push
	
# add-commit-push:
# 	git add . && git commit -m "add" && git push

# format: format-sh format-fish

# format-sh:
# 	shfmt -sr -w -bn -ci -s ./scripts/

# format-fish:
# 	find ./scripts/ -name "*.fish" -print -exec fish_indent -w {} \+

# shellharden:
# 	find ./scripts/ -name "*.sh" -print -exec shellharden --replace {} \+
# #shellharden --replace ./scripts/**/*.sh

# .PHONY: default add-commit-push format format-fish format-sh shellharden