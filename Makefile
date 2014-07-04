NAME = channel-delay
BUNDLE = $(NAME).lv2
LIBS = `pkg-config --cflags --libs lv2-plugin`
DEBUG = #-DDEBUG

NAME_STEREO = channel-delay-stereo

ALL_TTL = manifest.ttl $(NAME_STEREO).ttl

$(BUNDLE): clean $(ALL_TTL) $(NAME).so
	mkdir $(BUNDLE)
	cp $(ALL_TTL) $(NAME).so $(BUNDLE)

$(NAME).so:
	gcc -shared -fPIC -DPIC src/$(NAME).c $(LIBS) -o $(NAME).so $(DEBUG)

clean:
	rm -rf $(BUNDLE) $(NAME).so
