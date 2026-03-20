ADDON_ID=separate_by_collision
EXTENSIONS_PATH=~/.config/blender/5.0/extensions/user_default
ADDON_PATH=$(EXTENSIONS_PATH)/$(ADDON_ID)

RELEASE_FOLDER=extension_release
RELEASE_PATH=$(RELEASE_FOLDER)/$(ADDON_ID)

TEST_FILE=blend/bench.blend

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

	/usr/bin/blender $(TEST_FILE)

fmt:
	isort .
	ruff format .
