ADDON_ID=separate_by_collision
EXTENSIONS_PATH=~/.config/blender/4.2/extensions/user_default
ADDON_PATH=$(EXTENSIONS_PATH)/$(ADDON_ID)

RELEASE_FOLDER=extension_release
RELEASE_PATH=$(RELEASE_FOLDER)/$(ADDON_ID)

# BLENDER = /usr/bin/blender
BLENDER = ~/BlenderVersions/blender-4.2.0-linux-x64/blender

# TEST_FILE=blend/bench.blend
TEST_FILE=blend/plane_and_vert.blend

build-extension-archive:
	rm -rf $(RELEASE_PATH)
	mkdir -p $(RELEASE_PATH)
	cp $(ADDON_ID)/blender_manifest.toml $(ADDON_ID)/*.py $(RELEASE_PATH)/
	cd ./$(RELEASE_FOLDER) && zip -ru ./$(ADDON_ID)_extension.zip .

run-in-blender:
	make build-extension-archive
	rm -rf $(ADDON_PATH)
	mkdir -p $(EXTENSIONS_PATH)
	cp -r $(RELEASE_PATH) $(ADDON_PATH)

	$(BLENDER) $(TEST_FILE)

fmt:
	isort .
	ruff format .
