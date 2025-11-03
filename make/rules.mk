# (1) Add paths to each app dir here
APP_DIRS=apps/sample10 apps/sample13 apps/sample15

# Only build homog_bench if VAMP is available
ifdef VAMP_DIR
    ifneq ($(wildcard $(VAMP_DIR)/src/impl),)
        APP_DIRS += apps/homog_bench
    endif
endif

# (2) This rule makes a $(BUILD_DIR)/apps/[dir name]/driver executable for each app
APPS=$(patsubst apps/%,$(BUILD_DIR)/apps/%/driver,$(APP_DIRS))

.PRECIOUS: $(BUILD_DIR)/%.o 
.PRECIOUS: $(BUILD_DIR)/samples/%.o 

$(LIBRARY_OBJS): $(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp $(INCLUDES) $(DEPS)
	@mkdir -p $(@D)
	$(CXXV) $(CFLAGS_INTERNAL) $(CFLAGS) $< -o $@ $(INCLUDE_FLAGS) -c

$(BUILD_DIR)/samples/%.o: $(SAMPLES_DIR)/%.cpp $(INCLUDES) $(DEPS)
	@mkdir -p $(@D)
	$(CXXV) $(CFLAGS_INTERNAL) $(CFLAGS) $< -o $@ $(INCLUDE_FLAGS) -c

$(LIBRARY): $(LIBRARY_OBJS) $(DEPS)
	@mkdir -p $(@D)
	$(ARV) cr $(LIBRARY) $(LIBRARY_OBJS)

$(BUILD_DIR)/sample%: $(BUILD_DIR)/samples/sample%.o $(LIBRARY) $(DEPS)
	@mkdir -p $(@D)
	$(CXXLDV) -o $@ $< $(LDFLAGS)

$(BUILD_DIR)/apps/%/driver.o: apps/%/driver.cpp $(INCLUDES) $(DEPS)
	@mkdir -p $(@D)
	$(CXXV) $(CFLAGS_INTERNAL) $(CFLAGS) -mavx2 $< -o $@ $(INCLUDE_FLAGS) -c

$(BUILD_DIR)/apps/%/driver: $(BUILD_DIR)/apps/%/driver.o $(LIBRARY) $(DEPS)
	@mkdir -p $(@D)
	$(CXXLDV) -o $@ $< $(LDFLAGS)

.PHONY: executables
executables: $(SAMPLES) $(APPS)
