CXX      = c++
CXXFLAGS = --std=c++11 -O2 -Iinclude

bence: test_send.out test_receive.out

test_send.out: Networking.o TestSend.o
	$(CXX) Networking.o TestSend.o -o test_send.out

test_receive.out: Networking.o TestReceive.o
	$(CXX) Networking.o TestReceive.o -o test_receive.out

Networking.o: include/SPLStandardMessage.h include/Networking.h src/Networking.cpp
	$(CXX) $(CXXFLAGS) src/Networking.cpp -c -o Networking.o

TestSend.o: include/SPLStandardMessage.h include/Networking.h test/TestSend.cpp
	$(CXX) $(CXXFLAGS) test/TestSend.cpp -c -o TestSend.o

TestReceive.o: include/SPLStandardMessage.h include/Networking.h test/TestReceive.cpp
	$(CXX) $(CXXFLAGS) test/TestReceive.cpp -c -o TestReceive.o

clean:
	rm *.o *.out
