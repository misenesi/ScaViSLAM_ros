all: sophus_built

SOURCE_DIR = sophus_src
TOP_DIR = $(PWD)

sophus_built:
	-rm -rf lib include bin
	-mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR)  && make -j8 install
	touch sophus_built

clean:
	rm -rf lib bin include $(SOURCE_DIR)/build $(SOURCE_DIR)/bin $(SOURCE_DIR)/lib
	rm -rf sophus_built
	rm -rf sophus # old target
