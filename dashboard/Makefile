.PHONY: build run live test clean

# Override when something else already owns the default port, e.g.
# make live MAVLINK_ADDR=127.0.0.1:14550
MAVLINK_ADDR ?= 127.0.0.1:14552

build:
	GOOS=js GOARCH=wasm go build -ldflags="-s -w" -o web/app.wasm ./cmd/app
	cp "$$(go env GOROOT)/lib/wasm/wasm_exec.js" web/wasm_exec.js

run: build
	go run ./cmd/serve

live: build
	go run ./cmd/serve -mavlink $(MAVLINK_ADDR)

test:
	go test ./...
	GOOS=js GOARCH=wasm go vet ./cmd/app

clean:
	rm -f web/app.wasm web/wasm_exec.js
