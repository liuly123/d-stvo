echo "Building 3rdparty/line_descriptor ... "
cd 3rdparty/line_descriptor
mkdir build
cd build
cmake ..
make -j4
cd ../../../

echo "Building StVO-PL ... "
mkdir build
cd build
cmake ..
make -j4
