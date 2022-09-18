TARGET_URL ?= hopper.local
TARGET_HOST ?= dweis@$(TARGET_URL)

HUB_URL ?= homepi.local
HUB_HOST ?= pi@$(HUB_URL)

REMOTE_DIRECTORY ?= ~
DEB_BUILD_PATH ?= target/debian/hopper_*.deb

TARGET_ARCH := armv7-unknown-linux-musleabihf
RELEASE_BINARY_PATH := target/release/hopper
TARGET_PATH := ~/hopper_rust

VERSION_TAG = $(shell cargo get version)

MENDER_ARTIFACT_NAME ?= hopper-$(VERSION_TAG)
MENDER_ARTIFACT_FILE ?= $(MENDER_ARTIFACT_NAME).mender
MENDER_ARTIFACT_OUTPUT_PATH := target/mender

.PHONY: build
build:
	cargo build --release --no-default-features --bin hopper

.PHONY: deploy-build
deploy-binary: build
	rsync -c ${RELEASE_BINARY_PATH} ${TARGET_HOST}:${TARGET_PATH}

.PHONY: build-deb
build-deb:
	cargo deb --no-build

.PHONE: install
install: build-deb
	sudo dpkg -i $(DEB_BUILD_PATH)

.PHONY: deploy
deploy: build-deb
	@echo "Sending $(DEB_BUILD_PATH) to $(TARGET_HOST):$(REMOTE_DIRECTORY)"
	rsync -avz --delete $(DEB_BUILD_PATH) $(TARGET_HOST):$(REMOTE_DIRECTORY)

.PHONY: build-artifact
build-artifact: build-deb
	mkdir -p $(MENDER_ARTIFACT_OUTPUT_PATH)
	rm -f $(MENDER_ARTIFACT_OUTPUT_PATH)/*
	mender-artifact write module-image --type deb \
		--artifact-name $(MENDER_ARTIFACT_NAME) \
		--device-type raspberrypi4 \
		--device-type raspberrypi3 \
		--output-path $(MENDER_ARTIFACT_OUTPUT_PATH)/$(MENDER_ARTIFACT_FILE) \
		--file $(DEB_BUILD_PATH)

.PHONY: publish-mender-artifact
publish-mender-artifact: build-artifact
	mender-cli artifacts --server https://hosted.mender.io upload $(MENDER_ARTIFACT_OUTPUT_PATH)/$(MENDER_ARTIFACT_FILE)

.PHONY: serve-artifact
serve-artifact: build-artifact
	@echo http://$(HOSTNAME):8000
	python3 -m http.server 8000 --directory $(MENDER_ARTIFACT_OUTPUT_PATH)

.PHONY: push-to-hopper
push-to-hopper:
	rsync -avzhP --stats --exclude 'target/' . \
		$(TARGET_HOST):$(REMOTE_DIRECTORY)/src/hopper_rust

.PHONY: push-to-hub
push-to-hub:
	rsync -avzhP --stats --exclude 'target/' . \
		$(HUB_HOST):$(REMOTE_DIRECTORY)/src/hopper_rust

.PHONY: install-dependencies
install-dependencies:
	sudo apt update && sudo apt install libasound2-dev libudev-dev liblzma-dev -y
	cargo install cargo-deb cargo-get
