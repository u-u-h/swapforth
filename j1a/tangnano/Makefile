VERILOGS = j1a.v uart.v ../verilog/j1.v ../verilog/stack2.v

SUBDIRS = ..

all: $(SUBDIRS) j1a.bin

# we are using 2 big rams instead of 16 small ones on the GWN1N, so we need a modified ram generator
ram.v: ../build/nuc.hex mkrom.py
	python mkrom.py

j1a.bin: $(VERILOGS) j1a.pcf Makefile ram.v
	yosys  -q -p "synth_gowin -top top -vout j1a.synth.v" $(VERILOGS)
	@echo "Currently you'll have to use gowin tools for PNR"

j1a: j1a.bin
	@echo "Program j1a.bin using vendor tool or openfpga uploader"


$(SUBDIRS):
	$(MAKE) -C $@

clean:
	rm -f j1a.blif j1a.txt j1a.bin

.PHONY: subdirs
.PHONY: subdirs $(SUBDIRS)
.PHONY: clean
.PHONY: j1a

