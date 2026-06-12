all:
	$(MAKE) -C drumrobot_server

clean:
	$(MAKE) -C drumrobot_server clean

run:
	$(MAKE) -C drumrobot_server
	sudo ./drumrobot_server/bin/main.out