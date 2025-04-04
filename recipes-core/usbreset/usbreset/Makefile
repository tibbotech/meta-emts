
all:
	$(CC) $(CFLAGS) $(LDFLAGS) usbreset.c -o usbreset

install:
	install -d $(DESTDIR)/home/root
	install usbreset $(DESTDIR)/home/root

clean:
	$(RM) -rf *.o