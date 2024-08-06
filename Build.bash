# sudo rm -rf build
mkdir build
# sudo source /opt/ros/noetic/setup.bashCreating build group
cd build
cmake -G "Unix Makefiles" ..
make
sudo make install
# ls ./bin
