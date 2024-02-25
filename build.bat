rmdir /s build
mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DFILTERS_BUILD_DYNAMIC=1 -DFILTERS_BUILD_STATIC=1 -DFILTERS_BUILD_EXE=1 ..
make
cd ..