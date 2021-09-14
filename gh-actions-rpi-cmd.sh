# note that this is running on a virtual rpi
export VIRTUAL_PI=1

apt-get update -y && apt-get full-upgrade -y
./install-deps.sh

g++ -Wall -o prog src/test.cpp src/SI7021.cpp -I include -o test -llgpio
