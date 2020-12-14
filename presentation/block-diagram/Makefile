TARGET := block-diagram.pdf

BIB := $(wildcard *.bib)
SVG := $(wildcard *.svg)
PDF_TEX := $(SVG:.svg=.pdf_tex)

.PHONY: all
all: $(TARGET)

$(TARGET): $(basename $(TARGET)).tex
	latexmk -interaction=nonstopmode -xelatex $(basename $@)

%.pdf_tex: %.svg
	inkscape -D -z --file=$< --export-pdf=$(basename $<).pdf --export-latex

.PHONY: clean
clean:
	latexmk -xelatex -C
	rm -f $(TARGET)
