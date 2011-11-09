all:
	@rm -f tasks.zip
	@#./generate-sailing
	@#./generate-rect
	@#./generate-wet
	@#./generate-tree
	@./generate-race
	@echo "#tasks=" `ls *_mutask_* | wc -l`
	@zip -qr tasks.zip *_mutask_*
	@rm -f *_mutask_*
