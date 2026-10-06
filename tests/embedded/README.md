# Cortex-M4 smoke

Нужны `arm-none-eabi-gcc`, binutils, newlib и `qemu-system-arm`. Тестовая машина — MPS2 AN386, Cortex-M4; используется software floating point. GNU-специфичные startup, semihosting и linker script относятся к тестовому адаптеру; ядро остаётся C11.

Полный pytest-прогон собирает и выполняет оба профиля, сохраняет ELF/map и проверяет память:

```sh
python3 -m pytest tests/integration/test_embedded.py --embedded -q
```

Инструменты ищутся в PATH. При установке в отдельный каталог задайте `MK_ARM_TOOLCHAIN_DIR` (каталог bin) и `MK_QEMU_PATH` (путь к бинарнику QEMU). Динамические зависимости QEMU должны разрешаться средой. Отсутствие инструментов с флагом `--embedded` считается ошибкой.

Для ручного запуска одного профиля:

```sh
cmake -S . -B build-arm -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm_none_eabi.cmake \
  -DMK_REAL_TYPE=double -DMK_BUILD_EMBEDDED_SMOKE=ON
cmake --build build-arm
qemu-system-arm -M mps2-an386 -cpu cortex-m4 -nographic \
  -semihosting-config enable=on,target=native \
  -kernel build-arm/manipulator_smoke.elf
```

Успех: три строки `STACK n bytes`, `SMOKE_OK`, код завершения 0. Fault или отказ завершаются ненулевым кодом. Smoke проверяет пространственную 3R-модель, смешанную R/P-цепь, базу/инструмент, нечисловые входы, точную последнюю строку, субнормальные значения и переполнение.

Измерение стека выполняется при n=3/6/64 через окрашивание свободной области. Оно включает тестовый вызывающий код и runtime, а также запас 256 байт при окрашивании; это не точный размер одного вызова ядра. pytest дополнительно проверяет статические stack-usage отчёты и отсутствие heap/I/O-символов в ELF. Результаты измерений и ограничения изложены в [валидации](../../docs/validation.md).

Для собственной платы приложение предоставляет startup, linker script и выбранный runtime. QEMU не подтверждает время расчёта или работу периферии реального микроконтроллера.
