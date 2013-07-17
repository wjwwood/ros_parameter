all:
	-mkdir build
	cd build && cmake .. -DCMAKE_INSTALL_PREFIX=./install
	cd build && make

tests: all
	cd build && make tests

run_tests: tests
	cd build && make run_tests

clean:
	-rm -rf build
