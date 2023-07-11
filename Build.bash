# sudo rm -rf build
# mkdir build
cd build
cmake -G "Unix Makefiles" ..
make
sudo make install
# ls ./bin
