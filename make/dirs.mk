PWD=$(shell pwd)

BUILD_DIR?=$(PWD)/build
INCLUDE_DIR=$(PWD)/include
SRC_DIR=$(PWD)/src

BUILDIT_DIR?=$(PWD)/deps/buildit
BUILDIT_DEPS=$(wildcard $(BUILDIT_DIR)/src/*) $(wildcard $(BUILDIT_DIR)/include/*) 

SAMPLES_DIR=$(PWD)/samples
SAMPLES_SRCS=$(wildcard $(SAMPLES_DIR)/*.cpp)
SAMPLES=$(subst $(SAMPLES_DIR),$(BUILD_DIR),$(SAMPLES_SRCS:.cpp=))

LIBRARY_SRCS=$(wildcard $(SRC_DIR)/*.cpp)
LIBRARY_OBJS=$(subst $(SRC_DIR),$(BUILD_DIR),$(LIBRARY_SRCS:.cpp=.o))

EIGEN_DIR=$(PWD)/deps/eigen
BLAZE_DIR=$(PWD)/deps/blaze

# Try to find VAMP automatically (unless VAMP_DIR is already set)
ifndef VAMP_DIR
    # Check common locations relative to this directory
    VAMP_SEARCH_PATHS := \
        $(PWD)/../../../vamp \
        $(HOME)/vamp

    # Find the first path that exists
    VAMP_FOUND := $(firstword $(foreach path,$(VAMP_SEARCH_PATHS),$(wildcard $(path)/src/impl)))

    ifneq ($(VAMP_FOUND),)
        # Found VAMP - set VAMP_DIR to the parent directory
        VAMP_DIR := $(patsubst %/src/impl,%,$(VAMP_FOUND))
        $(info Found VAMP at: $(VAMP_DIR))
    else
        $(info VAMP not found. Checked: $(VAMP_SEARCH_PATHS))
        $(info Set VAMP_DIR to enable VAMP-dependent targets (e.g., homog_bench))
    endif
endif

INCLUDES=$(wildcard $(INCLUDE_DIR)/*.h)
