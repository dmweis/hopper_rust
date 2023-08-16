TARGET_HOST ?= hopper
TARGET_USERNAME ?= $$USER
TARGET_HOST_USER ?= $(TARGET_USERNAME)@$(TARGET_HOST)

REMOTE_DIRECTORY ?= ~
DEB_BUILD_PATH ?= target/debian/hopper_*.deb

TARGET_ARCH := aarch64-unknown-linux-musl
RELEASE_BINARY_PATH := target/release/hopper
RELEASE_CROSS_BINARY_PATH := ./target/${TARGET_ARCH}/release/hopper

TARGET_PATH := ~/src/hopper_rust/

VERSION_TAG = $(shell cargo get version)

MENDER_ARTIFACT_NAME ?= hopper-$(VERSION_TAG)
MENDER_ARTIFACT_FILE ?= $(MENDER_ARTIFACT_NAME).mender
MENDER_ARTIFACT_OUTPUT_PATH := target/mender

.PHONY: build
build:
	cargo build \
		--release \
		--no-default-features \
		--bin hopper \
		--features audio

.PHONY: deploy-binary
deploy-binary: build
	rsync -c ${RELEASE_BINARY_PATH} ${TARGET_HOST}:${TARGET_PATH}

.PHONY: build-deb
build-deb: build
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
		--software-name hopper_rust \
		--software-version $(VERSION_TAG) \
		--artifact-name $(MENDER_ARTIFACT_NAME) \
		--device-type raspberrypi4 \
		--device-type raspberrypi3 \
		--output-path $(MENDER_ARTIFACT_OUTPUT_PATH)/$(MENDER_ARTIFACT_FILE) \
		--file $(DEB_BUILD_PATH) \
		--script mender/ArtifactInstall_Enter_00

.PHONY: publish-mender-artifact
publish-mender-artifact: build-artifact
	mender-cli artifacts --server https://hosted.mender.io upload $(MENDER_ARTIFACT_OUTPUT_PATH)/$(MENDER_ARTIFACT_FILE)

.PHONY: install-dependencies
install-dependencies:
	sudo apt update && sudo apt install libasound2-dev libudev-dev liblzma-dev libclang-dev protobuf-compiler -y
	cargo install cargo-deb cargo-get

.PHONY: build-docker
build-docker:
	rm -rf docker_out
	mkdir docker_out
	DOCKER_BUILDKIT=1 docker build --tag hopper-builder --file Dockerfile --output type=local,dest=docker_out .

.PHONY: push-docker-built
push-docker-built: build-docker
	rsync -avz --delete docker_out/* $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/hopper
	rsync -avz --delete scripts/add_udev_rules $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/hopper/add_udev_rules
