CXX      = c++
CXXFLAGS = --std=c++11 -O2

test.out: Client.o NetworkingTest.o
	$(CXX) Client.o NetworkingTest.o -o test.out

Client.o: Client.cpp
	$(CXX) $(CXXFLAGS) Client.cpp -c -o Client.o

NetworkingTest.o: NetworkingTest.cpp
	$(CXX) $(CXXFLAGS) NetworkingTest.cpp -c -o NetworkingTest.o

clean:
	rm *.o test.out
