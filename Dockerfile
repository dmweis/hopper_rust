FROM balenalib/raspberrypi3-64-debian as chef

WORKDIR /app

# Install dependancies
RUN apt-get update && apt-get install -y lld \
    clang \
    autoconf \
    libtool \
    pkg-config \
    build-essential \
    unzip \
    wget \
    librust-libudev-sys-dev \
    libasound2-dev \
    libssl-dev \
    libv4l-dev \
    libclang-dev

# install protoc
RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v23.4/protoc-23.4-linux-aarch_64.zip
RUN unzip protoc-23.4-linux-aarch_64.zip
RUN cp -r /app/include/* /usr/local/include/.
RUN cp /app/bin/protoc /usr/local/bin/.

# Install rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

RUN cargo install cargo-chef

# rust layer caching
FROM chef as planner
COPY . .
RUN cargo chef prepare --recipe-path recipe.json

# rebuild dependencies if changed
FROM chef as builder
# install deb because it doesn't chage often
RUN cargo install cargo-deb

# dependancies rebuild
COPY --from=planner /app/recipe.json recipe.json
RUN cargo chef cook --release --recipe-path recipe.json

# Now copy code
COPY . .

# Build
RUN cargo build \
    --release \
    --bin hopper \
    --features audio

RUN cargo deb --no-build --fast

# Copy to exporter
FROM scratch AS export
COPY --from=builder /app/target/debian/hopper-rust*.deb /
COPY --from=builder /app/target/debian/hopper-rust*.deb /hopper-rust.deb
COPY --from=builder /app/target/release/hopper /
