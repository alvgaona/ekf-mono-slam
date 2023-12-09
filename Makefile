.PHONY: all
all: format test build

.PHONY: format
format:
	clang-format src/**/* include/**/* -i

.PHONY: build
build:
	mkdir -p build
	cd build && \
	cmake .. && \
	make

.PHONY: setup
setup:
	curl https://pub-db0cd070a4f94dabb9b58161850d4868.r2.dev/AGZ_subset.zip -o ./test/resources/AGZ_subset.zip
	unzip ./test/resources/AGZ_subset -d ./test/resources/

.PHONY: test
test:
	mkdir -p build
	cd build && \
	cmake .. && \
	make && \
	./test

.PHONY: debug
debug:
	mkdir -p build
	cd build && \
	cmake -DCMAKE_BUILD_TYPE=debug .. && \
	make

.PHONY: clean
clean:
	rm -rf build

