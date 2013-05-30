all: pangolin_built

SOURCE_DIR = pangolin_src
TOP_DIR = $(PWD)

pangolin_built:
	-rm -rf lib include bin
	-mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR) && make -j8 install
	touch pangolin_built

clean:
	rm -rf lib bin include $(SOURCE_DIR)/build $(SOURCE_DIR)/bin $(SOURCE_DIR)/lib
	rm -rf pangolin_built
	rm -rf pangolin # old target
