TARGET_HOST ?= hopper
TARGET_USERNAME ?= $$USER
TARGET_HOST_USER ?= $(TARGET_USERNAME)@$(TARGET_HOST)

REMOTE_DIRECTORY ?= ~
DEB_BUILD_PATH ?= target/debian/hopper_*.deb

TARGET_ARCH := aarch64-unknown-linux-musl
RELEASE_BINARY_PATH := target/release/hopper
RELEASE_CROSS_BINARY_PATH := ./target/${TARGET_ARCH}/release/hopper

TARGET_PATH := ~/src/hopper_rust/

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

.PHONY: install-dependencies
install-dependencies:
	sudo apt update && sudo apt install libasound2-dev libudev-dev liblzma-dev libclang-dev protobuf-compiler -y
	cargo install cargo-deb

.PHONY: build-docker
build-docker:
	rm -rf docker_out
	mkdir docker_out
	DOCKER_BUILDKIT=1 docker build --tag hopper-builder --file Dockerfile --output type=local,dest=docker_out .

.PHONY: push-docker
push-docker: build-docker
	rsync -avz --delete docker_out/* $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/hopper
	rsync -avz --delete scripts/add_udev_rules $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/hopper/add_udev_rules
	rsync -avz --delete scripts/install_wait_loop $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/hopper/install_wait_loop

.PHONY: deploy-docker
deploy-docker: push-docker
	@echo "Installing hopper on $(TARGET_HOST)"
	mosquitto_pub -h homepi -t "hopper/build" -n

.PHONY: deploy-with-ez-cd
deploy-with-ez-cd: build-docker
	ez-cd-cli -f docker_out/hopper-rust.deb -d hopper
