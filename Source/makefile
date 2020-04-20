output: main.o
	g++ main.o -o output

main.o: main.cpp onboard.h stateDerivative.h macro.h
	g++ -c main.cpp

clean: 
	rm *.o output