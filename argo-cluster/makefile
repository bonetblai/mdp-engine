all:
	@rm -f tasks.zip
	@#./generate-ctp2
	@./generate-ctp3-paper
	@#./generate-race
	@#./generate-rect
	@#./generate-sailing
	@#./generate-tree
	@#./generate-wet
	@echo "#tasks=" `ls *_mutask?_* | wc -l`
	@zip -qr tasks.zip *_mutask?_*
	@rm -f *_mutask?_*
