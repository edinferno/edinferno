CXX      = c++
CXXFLAGS = --std=c++11 -O2

test.out: Networking.o NetworkingTest.o
	$(CXX) Networking.o NetworkingTest.o -o test.out

Networking.o: Networking.cpp
	$(CXX) $(CXXFLAGS) Networking.cpp -c -o Networking.o

NetworkingTest.o: NetworkingTest.cpp
	$(CXX) $(CXXFLAGS) NetworkingTest.cpp -c -o NetworkingTest.o

clean:
	rm *.o test.out
