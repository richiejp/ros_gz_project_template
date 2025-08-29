#!/bin/sh -eu

GO="$1"
OUT="$2"

. ./cgo-flags.env
CGO_ENABLED=1 $GO build -o "$OUT"
