
$(BUILD_DIR)/buildit.dep: $(BUILDIT_DEPS) 
	$(MAKE) -C $(BUILDIT_DIR) DEBUG=$(DEBUG)
	touch $(BUILD_DIR)/buildit.dep

$(BUILD_DIR)/pinocchio.dep: $(PINOCCHIO_DEPS) 
	cd $(PINOCCHIO_DIR) && mkdir -p build && mkdir -p install && cd build && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_PYTHON_INTERFACE=OFF -DCMAKE_INSTALL_PREFIX=$(PINOCCHIO_DIR)/install
	$(MAKE) -C $(PINOCCHIO_DIR)/build
	$(MAKE) -C $(PINOCCHIO_DIR)/build install
	touch $(BUILD_DIR)/pinocchio.dep
