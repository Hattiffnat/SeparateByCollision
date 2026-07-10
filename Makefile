RUST_BACKTRACE=1

ADDON_ID=separate_by_collision
EXTENSIONS_PATH=~/.config/blender/5.1/extensions/user_default
ADDON_PATH=$(EXTENSIONS_PATH)/$(ADDON_ID)

RELEASE_FOLDER=extension_release
RELEASE_PATH=$(RELEASE_FOLDER)/$(ADDON_ID)

# BLENDER = /usr/bin/blender
BLENDER = ~/BlenderVersions/blender-5.1.2-linux-x64/blender

TEST_FILE=blend/bench.blend
# TEST_FILE=blend/ultratest_1.blend
# TEST_FILE=blend/suzanne_cloud.blend
# TEST_FILE=blend/test_42.blend
# TEST_FILE=blend/concept_demo.blend
# TEST_FILE=errors/file20260611.blend

build-bin-release:
	cd collisions && \
	cargo build --release --target x86_64-pc-windows-gnu && \
	cargo build --release --target x86_64-unknown-linux-gnu && \
	cargo zigbuild --release --target universal2-apple-darwin


build-extension-archive-release:
	make build-bin-release
	rm -rf $(RELEASE_PATH)
	mkdir -p $(RELEASE_PATH)/bin
	cp ./collisions/target/x86_64-pc-windows-gnu/release/collisions.dll $(RELEASE_PATH)/bin
	cp ./collisions/target/x86_64-unknown-linux-gnu/release/libcollisions.so $(RELEASE_PATH)/bin
	cp ./collisions/target/universal2-apple-darwin/release/libcollisions.dylib $(RELEASE_PATH)/bin
	cp $(ADDON_ID)/blender_manifest.toml $(ADDON_ID)/*.py $(RELEASE_PATH)/
	cd ./$(RELEASE_FOLDER) && 7z a -tzip ./$(ADDON_ID)_extension.zip $(ADDON_ID)/ && cd ..

build-extension-archive:
	cd collisions && cargo build --target x86_64-unknown-linux-gnu
	rm -rf $(RELEASE_PATH)
	mkdir -p $(RELEASE_PATH)/bin
	cp ./collisions/target/x86_64-unknown-linux-gnu/debug/libcollisions.so $(RELEASE_PATH)/bin
	cp $(ADDON_ID)/blender_manifest.toml $(ADDON_ID)/*.py $(RELEASE_PATH)/
	cd ./$(RELEASE_FOLDER) && 7z a -tzip ./$(ADDON_ID)_extension.zip $(ADDON_ID)/ && cd ..

run-in-blender:
	make build-extension-archive
	rm -rf $(ADDON_PATH)
	mkdir -p $(EXTENSIONS_PATH)
	cp -r $(RELEASE_PATH) $(ADDON_PATH)
	$(BLENDER) $(TEST_FILE)

run-in-blender-release:
	make build-extension-archive-release
	rm -rf $(ADDON_PATH)
	mkdir -p $(EXTENSIONS_PATH)
	cp -r $(RELEASE_PATH) $(ADDON_PATH)
	$(BLENDER) $(TEST_FILE)

fmt:
	isort .
	ruff format .
