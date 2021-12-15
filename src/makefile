all: Complex.o Quaternion.o Octonion.o Sedenion.o Trigintaduonion.o Bezier.o Slerp.o Spline.o

int: iComplex.o

real: Complex.o Quaternion.o Octonion.o Sedenion.o Trigintaduonion.o

Support.o: Support.cpp Support.h
	g++ -O2 -c Support.cpp

Complex.o: Complex.cpp Complex.h Support.o Support.h
	g++ -O2 -c Complex.cpp

Quaternion.o: Quaternion.cpp Quaternion.h Support.o Support.h
	g++ -O2 -c Quaternion.cpp

Octonion.o: Octonion.cpp Octonion.h Support.o Support.h
	g++ -O2 -c Octonion.cpp

Sedenion.o: Sedenion.cpp Sedenion.h Support.o Support.h
	g++ -O2 -c Sedenion.cpp

Trigintaduonion.o: Trigintaduonion.cpp Trigintaduonion.h Support.o Support.h
	g++ -O2 -c Trigintaduonion.cpp

Slerp.o: Slerp.cpp Slerp.h Support.o Support.h Complex.o Quaternion.o Octonion.o Sedenion.o Trigintaduonion.o
	g++ -O2 -c Slerp.cpp

Bezier.o: Bezier.cpp Bezier.h Slerp.o Support.o Support.h Complex.o Quaternion.o Octonion.o Sedenion.o Trigintaduonion.o
	g++ -O2 -c Bezier.cpp

Spline.o: Spline.cpp Spline.h Slerp.o Support.o Support.h Complex.o Quaternion.o Octonion.o Sedenion.o Trigintaduonion.o
	g++ -O2 -c Spline.cpp

iSupport.o: iSupport.cpp iSupport.h
	g++ -O2 -c iSupport.cpp

iComplex.o: iComplex.cpp iComplex.h iSupport.o iSupport.h
	g++ -O2 -c iComplex.cpp

clean:
	rm -f *.o core
