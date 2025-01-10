sudo rm -rf build
mkdir build
# sudo source /opt/ros/noetic/setup.bashCreating build group

# dependencies
sudo apt-get update
sudo apt-get install cmake
sudo apt-get install build-essential
sudo apt-get install libusb-1.0
sudo apt-get install liblog4cxx-dev

cd build
cmake -G "Unix Makefiles" ..
make
sudo make install
# ls ./bin