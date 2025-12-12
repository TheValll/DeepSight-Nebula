sudo apt-get update
sudo apt-get install libboost-all-dev
g++ test_hiwonder_xarm_esp32.cpp hiwonder_xarm_esp32.cpp -o test_hiwonder -lboost_system -lpthread
