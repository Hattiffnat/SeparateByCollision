ADDON_ID=separate_by_collision
EXTENSIONS_PATH=~/.config/blender/4.2/extensions/user_default
ADDON_PATH=$(EXTENSIONS_PATH)/$(ADDON_ID)

RELEASE_FOLDER=extension_release
RELEASE_PATH=$(RELEASE_FOLDER)/$(ADDON_ID)

# BLENDER = /usr/bin/blender
BLENDER = ~/BlenderVersions/blender-4.2.0-linux-x64/blender

# TEST_FILE=blend/bench.blend
# TEST_FILE=blend/ultratest_1.blend
# TEST_FILE=blend/suzanne_cloud.blend
TEST_FILE=blend/test_42.blend

export CC=aarch64-apple-darwin20.4-clang
export CXX=aarch64-apple-darwin20.4-clang++
export AR=aarch64-apple-darwin20.4-ar

build-bin:
	cd collisions && \
	cargo build --release --target x86_64-pc-windows-gnu && \
	cargo build --release --target x86_64-unknown-linux-gnu && \
	# cargo build --release --target x86_64-apple-darwin
	# cargo build --release --target aarch64-apple-darwin


build-extension-archive:
	make build-bin
	rm -rf $(RELEASE_PATH)
	mkdir -p $(RELEASE_PATH)/bin
	cp ./collisions/target/release/libcollisions.so $(RELEASE_PATH)/bin
	cp $(ADDON_ID)/blender_manifest.toml $(ADDON_ID)/*.py $(RELEASE_PATH)/
	cd ./$(RELEASE_FOLDER) && 7z a -tzip ./$(ADDON_ID)_extension.zip $(ADDON_ID)/ && cd ..

run-in-blender:
	make build-extension-archive
	rm -rf $(ADDON_PATH)
	mkdir -p $(EXTENSIONS_PATH)
	cp -r $(RELEASE_PATH) $(ADDON_PATH)

	$(BLENDER) $(TEST_FILE)

fmt:
	isort .
	ruff format .
