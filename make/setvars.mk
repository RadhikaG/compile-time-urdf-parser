#if debug is not set, default to false
DEBUG?=0

# Query CFLAGS and LD_FLAGS from BuildIt
# BuildIt doesn't have to be built for this

ifeq ($(DEBUG), 1)
CFLAGS=-O0 -g
else 
CFLAGS=-O3
endif
LDFLAGS=

CFLAGS_INTERNAL=-std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wmissing-declarations 
CFLAGS_INTERNAL+=-Woverloaded-virtual -Wno-deprecated -Wdelete-non-virtual-dtor -Werror -Wno-vla -pedantic-errors 
CFLAGS_INTERNAL+=-Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function -Wno-ignored-attributes -Wno-sign-compare

CFLAGS+=$(shell make --no-print-directory -C $(BUILDIT_DIR)/ DEBUG=$(DEBUG) compile-flags)
LDFLAGS+=$(shell make --no-print-directory -C $(BUILDIT_DIR)/ DEBUG=$(DEBUG) linker-flags)

DEPS=$(BUILD_DIR)/buildit.dep
INCLUDE_FLAGS=-I $(INCLUDE_DIR) -I $(EIGEN_DIR) -I $(BLAZE_DIR)

# Only add VAMP include path if VAMP is available
ifdef VAMP_DIR
    INCLUDE_FLAGS+= -I $(VAMP_DIR)/src/impl
endif

LIBRARY=$(BUILD_DIR)/lib$(LIBRARY_NAME).a

LDFLAGS+=-L$(BUILD_DIR) -l$(LIBRARY_NAME)
