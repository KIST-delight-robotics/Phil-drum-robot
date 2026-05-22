all:
	$(MAKE) -C drumrobot

clean:
	$(MAKE) -C drumrobot clean

run:
	$(MAKE) -C drumrobot
	sudo ./drumrobot/bin/main.out

run-llm:
	$(MAKE) -C drumrobot
	sudo ./drumrobot/bin/main.out --mode llm

run-keyboard:
	$(MAKE) -C drumrobot
	sudo ./drumrobot/bin/main.out --mode keyboard