FROM debian:trixie-slim

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        binutils-arm-none-eabi \
        gcc-arm-none-eabi \
        make \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/reconstruction

CMD ["make"]
