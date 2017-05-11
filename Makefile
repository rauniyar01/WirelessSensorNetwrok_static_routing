CONTIKI_PROJECT = Customer_routing
all: $(CONTIKI_PROJECT)
	
#UIP_CONF_IPV6=1

CONTIKI = $(HOME)/contiki
include $(CONTIKI)/Makefile.include
