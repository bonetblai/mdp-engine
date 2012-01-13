all:
	@rm -f tasks.zip
	@#./generate-ctp2
	@#./generate-ctp3-paper
	@./generate-ctp3-paper-2
	@#./generate-race
	@#./generate-race-paper
	@#./generate-rect
	@#./generate-sailing
	@#./generate-tree
	@#./generate-wet
	@echo "#tasks=" `ls *_mutask*_* | wc -l`
	@zip -qr tasks.zip *_mutask*_*
	@rm -f *_mutask*_*
