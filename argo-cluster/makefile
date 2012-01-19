all:
	@rm -f tasks.zip
	@#./generate-race-task10
	@#./generate-ctp3-task17
	@#./generate-ctp3-task18
	@#./generate-ctp3-task19
	@#./generate-sailing-task20
	@echo "#tasks=" `ls *_mutask*_* | wc -l`
	@zip -qr tasks.zip *_mutask*_*
	@rm -f *_mutask*_*
