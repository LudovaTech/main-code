#!/usr/bin/env bash
# run rust-analyzer inside cross for the target

exec cross-util run -i --target=aarch64-unknown-linux-gnu -- 'rust-analyzer $@'
