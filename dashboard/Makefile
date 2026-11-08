.PHONY: build run live test clean

build:
	GOOS=js GOARCH=wasm go build -ldflags="-s -w" -o web/app.wasm ./cmd/app
	cp "$$(go env GOROOT)/lib/wasm/wasm_exec.js" web/wasm_exec.js

run: build
	go run ./cmd/serve

live: build
	go run ./cmd/serve -mavlink 127.0.0.1:14552

test:
	go test ./...
	GOOS=js GOARCH=wasm go vet ./cmd/app

clean:
	rm -f web/app.wasm web/wasm_exec.js
