
SDKINCPATH =../include/
DRVINCPATH =../../iodrv/include/


PSP_PRODUCT= $(shell grep "efine PSP_" $(DRVINCPATH)config.h | awk -F "_" '{print tolower($$2)}')
export PSP_MAJOR= $(shell grep "PKG_MAJOR" $(DRVINCPATH)config.h | awk -F " " '{print tolower($$3)}')
export PSP_MINOR= $(shell grep "PKG_MINOR" $(DRVINCPATH)config.h | awk -F " " '{print tolower($$3)}')
export PSP_BUILD= $(shell grep "PKG_BUILD" $(DRVINCPATH)config.h | awk -F " " '{print tolower($$3)}')


LIBMOD=lmbapi
CPPFILE=$(LIBMOD)lib
SDKLIBVER=$(PSP_MAJOR).$(PSP_MINOR).$(PSP_BUILD)

RED=\033[1;31m
GREEN=\033[1;32m
NC=\033[0m



ifeq ("$(BIN64_PATH)", "")
	LIB64_PATH=../../bin/amd64/lib/
	BIN64_PATH=../../bin/amd64/utils/
	LIB32_PATH=../../bin/i386/lib/
	BIN32_PATH=../../bin/i386/utils/
	LMBAPI_NAME=lmbapi
endif


RM=rm -rf
CP=cp -f
CC=gcc

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
   BINOUT_PATH=$(BIN64_PATH)
   LIBOUT_PATH=$(LIB64_PATH)
   CBITS=-m64
else
   BINOUT_PATH=$(BIN32_PATH)
   LIBOUT_PATH=$(LIB32_PATH)
   CBITS=-m32
endif


.PHONY : default all amd64 i386 clean

default: dependos

dependos:
	@echo -e "$(GREEN)build lib$(LIBMOD) $(SDKLIBVER) $(LBITS)-bit .......$(NC)"
	@gcc $(CBITS) -fPIC -Wall -g -c $(LIBMOD).c -o lib$(LIBMOD).o -I$(SDKINCPATH) -I$(DRVINCPATH)
	@gcc $(CBITS) -g -shared -Wl,-soname,lib$(LIBMOD).so -o sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so lib$(LIBMOD).o -lc -lm
	@ar -rcs lib$(LIBMOD).a lib$(LIBMOD).o
	@ln -sf sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so lib$(LIBMOD).so
	@mv -f sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so* $(LIBOUT_PATH) 
	@mv -f lib$(LIBMOD).so $(LIBOUT_PATH) 
	@mv -f lib$(LIBMOD).a $(LIBOUT_PATH) 
	@rm *.o 
	@echo -e "$(GREEN)........................ Complied OK$(NC)"

all:  amd64 i386

amd64: 
	@echo -e "$(GREEN)build lib$(LIBMOD) $(SDKLIBVER) 64-bit .......$(NC)"
	@gcc -m64 -fPIC -Wall -g -c $(LIBMOD).c -o lib$(LIBMOD).o -I$(SDKINCPATH) -I$(DRVINCPATH)
	@gcc -m64 -g -shared -Wl,-soname,lib$(LIBMOD).so -o sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so lib$(LIBMOD).o -lc -lm
	@ar -rcs lib$(LIBMOD).a lib$(LIBMOD).o
	@ln -sf sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so lib$(LIBMOD).so
	@mv -f sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so* $(LIB64_PATH) 
	@mv -f lib$(LIBMOD).so $(LIB64_PATH) 
	@mv -f lib$(LIBMOD).a $(LIB64_PATH) 
	@rm *.o 
	@echo -e "$(GREEN)........................ Complied OK$(NC)"


i386: 
	@echo -e "$(GREEN)build lib$(LIBMOD) $(SDKLIBVER) 32-bit .......$(NC)"
	@gcc -m32 -fPIC -Wall -g -c $(LIBMOD).c -o lib$(LIBMOD).o -I$(SDKINCPATH) -I$(DRVINCPATH)
	@gcc -m32 -g -shared -Wl,-soname,lib$(LIBMOD).so -o sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so lib$(LIBMOD).o -lc -lm
	@ar -rcs lib$(LIBMOD).a lib$(LIBMOD).o
	@ln -sf sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so lib$(LIBMOD).so
	@mv -f sdk-$(PSP_PRODUCT)-$(SDKLIBVER).so* $(LIB32_PATH) 
	@mv -f lib$(LIBMOD).so $(LIB32_PATH) 
	@mv -f lib$(LIBMOD).a $(LIB32_PATH) 
	@rm *.o 
	@echo -e "$(GREEN)........................ Complied OK$(NC)"


clean:
	@-$(RM) $(LIB64_PATH)lib$(LIBMOD).so
	@-$(RM) $(LIB64_PATH)lib$(LIBMOD).a
	@-$(RM) $(LIB64_PATH)sdk-*-$(SDKLIBVER).so
	@-$(RM) $(LIB32_PATH)lib$(LIBMOD).so
	@-$(RM) $(LIB32_PATH)lib$(LIBMOD).a
	@-$(RM) $(LIB32_PATH)sdk-*-$(SDKLIBVER).so
	@echo -e "\e[1;32m<< lib$(LIBMOD) $(SDKLIBVER) clear done >>\e[m"

