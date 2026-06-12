all:
	$(MAKE) -C drumrobot

clean:
	$(MAKE) -C drumrobot clean

run:
	$(MAKE) -C drumrobot
	sudo ./drumrobot/bin/main.out