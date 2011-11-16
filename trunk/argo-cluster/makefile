all:
	@rm -f tasks.zip
	@./generate-ctp
	@./generate-race
	@#./generate-rect
	@#./generate-sailing
	@#./generate-tree
	@#./generate-wet
	@echo "#tasks=" `ls *_mutask_* | wc -l`
	@zip -qr tasks.zip *_mutask_*
	@rm -f *_mutask_*
