#if debug is not set, default to false
DEBUG?=0

#if expr_return is not set, default to false
EXPR_RETURN?=0

# Query CFLAGS and LD_FLAGS from BuildIt
# BuildIt doesn't have to be built for this

ifeq ($(DEBUG), 1)
CFLAGS=-O0 -g
else 
CFLAGS=-O3
endif
LDFLAGS=

CFLAGS+=$(shell make --no-print-directory -C $(BUILDIT_DIR)/ DEBUG=$(DEBUG) compile-flags)
LDFLAGS+=$(shell make --no-print-directory -C $(BUILDIT_DIR)/ DEBUG=$(DEBUG) linker-flags)

CFLAGS+=$(shell export PKG_CONFIG_PATH=$(PINOCCHIO_DIR)/install/lib/pkgconfig:$PKG_CONFIG_PATH && pkg-config --cflags pinocchio)
LDFLAGS+=$(shell export PKG_CONFIG_PATH=$(PINOCCHIO_DIR)/install/lib/pkgconfig:$PKG_CONFIG_PATH && pkg-config --libs pinocchio)

DEPS+=$(BUILD_DIR)/buildit.dep
INCLUDE_FLAGS=-I $(INCLUDE_DIR) -I $(EIGEN_DIR)

DEPS+=$(BUILD_DIR)/pinocchio.dep

LIBRARY=$(BUILD_DIR)/lib$(LIBRARY_NAME).a

LDFLAGS+=-L$(BUILD_DIR) -l$(LIBRARY_NAME)

ifeq ($(EXPR_RETURN), 1)
CFLAGS+=-DEXPR_RETURN
endif
