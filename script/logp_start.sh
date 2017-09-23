g++-5 -g -std=c++11 -Wall logp.cpp -o logp
./logp >> lopp.log &
ps -ef|grep logp|grep -v grep

