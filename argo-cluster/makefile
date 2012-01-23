all:
	@rm -f tasks.zip
	@#./generate-rconf-task1
	@#./generate-rconf-task2
	./generate-wumpus-task3
	@#./generate-race-task10
	@#./generate-race-task10
	@#./generate-ctp3-task14
	@#./generate-ctp3-task15
	@#./generate-ctp3-task16
	@#./generate-ctp3-task17
	@#./generate-ctp3-task18
	@#./generate-ctp3-task19
	@#./generate-sailing-task20
	@#./generate-sailing-task21
	@#./generate-sailing-task22
	@echo "#tasks=" `ls *_mutask*_* | wc -l`
	@zip -qr tasks.zip *_mutask*_*
	@rm -f *_mutask*_*
