CFLAGS += -DDEBUG -Wall -I../include `pkg-config --libs opencv`
fbtrack_cli = fbtrack_cli

all: libdtld.so fbtrack_cli

libdtld.so: fbtrack.c tld.h
	cc $(CFLAGS) -o $@ -shared -fPIC $<

fbtrack_cli: fbtrack_cli.c
	cc $(CFLAGS) -o $@ -L. -ldtld $<
clean:
	rm -rf *.o *.so fbtrack_cli
