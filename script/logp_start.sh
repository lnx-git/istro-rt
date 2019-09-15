g++ -g -std=c++11 -Wall logp.cpp -o logp
./logp >> logp.log &
ps -ef|grep logp|grep -v grep

