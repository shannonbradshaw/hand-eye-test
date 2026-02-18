GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/hand-eye-test

all: $(MODULE_BINARY)

$(MODULE_BINARY): Makefile go.mod *.go cmd/main.go
	mkdir -p bin
	GOOS=$(VIAM_BUILD_OS) GOARCH=$(VIAM_BUILD_ARCH) $(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) ./cmd/

lint:
	gofmt -s -w .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY)
	tar czf $@ meta.json $(MODULE_BINARY)

module: module.tar.gz

setup:
	go mod tidy

clean:
	rm -rf bin module.tar.gz
