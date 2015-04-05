CXX      = c++
CXXFLAGS = --std=c++11 -O2 -Iinclude

test.out: Networking.o NetworkingTest.o
	$(CXX) Networking.o NetworkingTest.o -o test.out

Networking.o: include/SPLStandardMessage.h include/Networking.h src/Networking.cpp
	$(CXX) $(CXXFLAGS) src/Networking.cpp -c -o Networking.o

NetworkingTest.o: include/SPLStandardMessage.h include/Networking.h test/NetworkingTest.cpp
	$(CXX) $(CXXFLAGS) test/NetworkingTest.cpp -c -o NetworkingTest.o

clean:
	rm *.o test.out
