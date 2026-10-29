CC ?= gcc
CFLAGS ?= -O3 -Wall -Wextra -std=c99 -Iinclude
LDFLAGS ?= -lm

SRCDIR = src
INCDIR = include
BUILDDIR = build

SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(patsubst $(SRCDIR)/%.c, $(BUILDDIR)/%.o, $(SOURCES))
TARGET = $(BUILDDIR)/harmonic_suppressor.exe

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJECTS)
	@mkdir -p $(BUILDDIR)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)
	@echo "Build complete: $@"

$(BUILDDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf $(BUILDDIR)/*.o $(TARGET)
