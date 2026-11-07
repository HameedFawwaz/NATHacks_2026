.PHONY: build run test clean

build:
	GOOS=js GOARCH=wasm go build -ldflags="-s -w" -o web/app.wasm ./cmd/app
	cp "$$(go env GOROOT)/lib/wasm/wasm_exec.js" web/wasm_exec.js

run: build
	go run ./cmd/serve

test:
	go test ./...
	GOOS=js GOARCH=wasm go vet ./cmd/app

clean:
	rm -f web/app.wasm web/wasm_exec.js
