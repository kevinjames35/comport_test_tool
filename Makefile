#simple make file
all:
	gcc comport_tool.c -o com_util -lpthread
clean:
	rm -f com_util
