ifeq ($(origin CC),default)
CC := clang
endif
CROSS_COMPILE ?= arm-none-eabi-
CFLAGS       ?= -std=c11 -Wall -Wextra -Werror -O2 -Iinclude
SAN_FLAGS    := -fsanitize=address,undefined -fno-omit-frame-pointer -g
LDFLAGS      := -lm

BUILD_DIR    := build

CORE_SRCS    := src/hks_crc32.c \
                src/hks_sha256.c \
                src/hks_contract.c \
                src/hks_flash_log.c \
                src/hks_protocol.c \
                src/hks_attitude.c

MOCK_SRCS    := hal/hal_mock.c
STUB_SRCS    := hal/hal_nrf5340_stub.c

CORE_OBJS    := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(CORE_SRCS))
MOCK_OBJ     := $(BUILD_DIR)/hal_mock.o
LIB_TARGET   := $(BUILD_DIR)/libhks_fw.a
CLI_TARGET   := $(BUILD_DIR)/hks_fw_sim_cli
TEST_TARGET  := $(BUILD_DIR)/test_firmware
STUB_OBJ     := $(BUILD_DIR)/hal_nrf5340_stub.o

.PHONY: all cli test nrf5340-stub clean

all: $(LIB_TARGET) $(CLI_TARGET) $(TEST_TARGET) nrf5340-stub

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(MOCK_OBJ): $(MOCK_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB_TARGET): $(CORE_OBJS) $(MOCK_OBJ)
	ar rcs $@ $^

$(CLI_TARGET): tools/hks_fw_sim_cli.c $(LIB_TARGET) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< $(LIB_TARGET) $(LDFLAGS) -o $@

cli: $(CLI_TARGET)

$(TEST_TARGET): tests/test_firmware.c $(CORE_SRCS) $(MOCK_SRCS) | $(BUILD_DIR)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 $(SAN_FLAGS) -Iinclude $^ $(LDFLAGS) -o $@

test: $(TEST_TARGET)
	./$(TEST_TARGET)

nrf5340-stub: | $(BUILD_DIR)
	@if command -v $(CROSS_COMPILE)gcc >/dev/null 2>&1; then \
		echo "Compiling nRF5340 stub with $(CROSS_COMPILE)gcc"; \
		$(CROSS_COMPILE)gcc -std=c11 -Wall -Wextra -Werror -O2 -mcpu=cortex-m33 -mthumb -ffreestanding -Iinclude -c $(STUB_SRCS) -o $(STUB_OBJ); \
		for f in $(CORE_SRCS); do \
			$(CROSS_COMPILE)gcc -std=c11 -Wall -Wextra -Werror -O2 -mcpu=cortex-m33 -mthumb -ffreestanding -Iinclude -c $$f -o $(BUILD_DIR)/$$(basename $$f .c).arm.o; \
		done; \
	else \
		echo "Compiling nRF5340 stub with clang --target=arm-none-eabi -mcpu=cortex-m33"; \
		$(CC) --target=arm-none-eabi -mcpu=cortex-m33 -ffreestanding -nostdlib -std=c11 -Wall -Wextra -Werror -O2 -Iinclude -Ihal/freestanding_inc -c $(STUB_SRCS) -o $(STUB_OBJ); \
		for f in $(CORE_SRCS); do \
			$(CC) --target=arm-none-eabi -mcpu=cortex-m33 -ffreestanding -nostdlib -std=c11 -Wall -Wextra -Werror -O2 -Iinclude -Ihal/freestanding_inc -c $$f -o $(BUILD_DIR)/$$(basename $$f .c).arm.o; \
		done; \
	fi

clean:
	rm -rf $(BUILD_DIR)
