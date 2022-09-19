TARGET_URL ?= hopper.local
TARGET_HOST ?= dweis@$(TARGET_URL)

HUB_URL ?= homepi.local
HUB_HOST ?= pi@$(HUB_URL)

REMOTE_DIRECTORY ?= ~
DEB_BUILD_PATH ?= target/debian/hopper_*.deb

TARGET_ARCH := aarch64-unknown-linux-musl
RELEASE_BINARY_PATH := target/release/hopper
TARGET_PATH := ~/src/hopper_rust/

VERSION_TAG = $(shell cargo get version)

MENDER_ARTIFACT_NAME ?= hopper-$(VERSION_TAG)
MENDER_ARTIFACT_FILE ?= $(MENDER_ARTIFACT_NAME).mender
MENDER_ARTIFACT_OUTPUT_PATH := target/mender

.PHONY: build
build:
	cargo build --release --no-default-features --bin hopper

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
		--artifact-name $(MENDER_ARTIFACT_NAME) \
		--device-type raspberrypi4 \
		--device-type raspberrypi3 \
		--output-path $(MENDER_ARTIFACT_OUTPUT_PATH)/$(MENDER_ARTIFACT_FILE) \
		--file $(DEB_BUILD_PATH) \
		--script mender/ArtifactInstall_Enter_00

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

.PHONY: cursed
cursed: push-to-hub
	# this is nasty
	ssh -t $(HUB_HOST) "cd src/hopper_rust && \
		/home/pi/.cargo/bin/cargo build --release --no-default-features --bin hopper && \
		rsync -c ${RELEASE_BINARY_PATH} ${TARGET_HOST}:${TARGET_PATH} && \
		ssh -t $(TARGET_HOST) 'cd src/hopper_rust && ./hopper'"

.PHONY: build-cross
build-cross:
	cargo build --release --target=${TARGET_ARCH} --no-default-features

.PHONY: deploy-cross
deploy-cross: build-cross
	rsync -c ${SOURCE_PATH} ${TARGET_HOST}:${TARGET_PATH}

.PHONY: remote-run
remote-run: deploy-cross
	ssh -t $(TARGET_HOST) ./face_demo
