all:
	@rm -f tasks.zip
	@#./generate-ctp2
	@#./generate-ctp3-paper
	@#./generate-ctp3-paper-2
	@#./generate-ctp3-paper-3
	@#./generate-ctp3-paper-4
	@#./generate-ctp3-task17
	@#./generate-ctp3-task18
	@./generate-ctp3-task19
	@#./generate-race
	@#./generate-race-paper
	@#./generate-rect
	@#./generate-sailing
	@#./generate-sailing-2
	@#./generate-tree
	@#./generate-wet
	@echo "#tasks=" `ls *_mutask*_* | wc -l`
	@zip -qr tasks.zip *_mutask*_*
	@rm -f *_mutask*_*
