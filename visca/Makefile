CFLAGS += -DDEBUG -Wall
CHECK_FILE = check_visca

all: libdvisca.so check_visca

libdvisca.so: visca.c visca.h
	cc $(CFLAGS) -o libdvisca.so -lpthread visca.c -shared -fPIC

$(CHECK_FILE): $(CHECK_FILE).c visca.h
	cc $(CFLAGS) -o $@ -L. -ldvisca -lcheck $<

clean: 
	rm -rf *.o *.so $(CHECK_FILE)
