CFLAGS += -DDEBUG -Wall

all: libdvisca.so testvisca

libdvisca.so: visca.c visca.h
	cc $(CFLAGS) -o libdvisca.so visca.c -shared -fPIC

testvisca: testvisca.c visca.h
	cc $(CFLAGS) -o testvisca -L. -ldvisca testvisca.c

clean: 
	rm -rf *.o *.so testvisca
