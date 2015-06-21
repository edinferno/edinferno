CXX      = c++
CXXFLAGS = --std=c++11 -O2 -Iinclude

bence: test_send.out test_receive.out

test_send.out: NetSender.o TestSend.o
	$(CXX) NetSender.o TestSend.o -o test_send.out

test_receive.out: NetReceiver.o TestReceive.o
	$(CXX) NetReceiver.o TestReceive.o -o test_receive.out

NetSender.o: include/SPLStandardMessage.h include/NetSender.h src/NetSender.cpp
	$(CXX) $(CXXFLAGS) src/NetSender.cpp -c -o NetSender.o

NetReceiver.o: include/SPLStandardMessage.h include/NetReceiver.h src/NetReceiver.cpp
	$(CXX) $(CXXFLAGS) src/NetReceiver.cpp -c -o NetReceiver.o

TestSend.o: include/SPLStandardMessage.h include/NetSender.h test/TestSend.cpp
	$(CXX) $(CXXFLAGS) test/TestSend.cpp -c -o TestSend.o

TestReceive.o: include/SPLStandardMessage.h include/NetReceiver.h test/TestReceive.cpp
	$(CXX) $(CXXFLAGS) test/TestReceive.cpp -c -o TestReceive.o

clean:
	rm *.o *.out
