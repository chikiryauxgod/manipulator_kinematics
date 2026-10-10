# Сборка и тестирование

## Сборка и пример

Нужны CMake 3.20+, C11-компилятор и математический runtime. По умолчанию используется double; для float задайте `-DMK_REAL_TYPE=float` в отдельном каталоге сборки.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMK_BUILD_EXAMPLES=ON
cmake --build build
./build/manipulator_example
```

Пример 3R печатает положение инструмента примерно `(0.3, 0, 0.3)` м. [Исходный пример](../examples/forward_example.c) показывает и позу, и получение кадров с заранее выделенными буферами.

## Интеграция

В CMake-проекте:

```cmake
add_subdirectory(path/to/manipulator_kinematics)
target_link_libraries(your_application PRIVATE manipulator::kinematics)
```

Либо установите пакет и подключите его через `find_package`:

```sh
cmake --install build --prefix "$PWD/install"
```

```cmake
find_package(manipulator 0.1 CONFIG REQUIRED)
target_link_libraries(your_application PRIVATE manipulator::kinematics)
```

Передайте расположение пакета через `CMAKE_PREFIX_PATH`. При нестандартном каталоге установки библиотеки укажите `manipulator_DIR` непосредственно на каталог с manipulatorConfig.cmake. Численный профиль публичных заголовков должен совпадать с библиотекой; несовпадение проверяется при линковке. Подробности функций, единиц, владения памятью и ограничений — в [руководстве C API](c-api.md).

## Тестирование

GoogleTest 1.14+ нужен только для unit-тестов; Python, pytest и Ninja — только для интеграционных проверок.

```sh
cmake -S . -B build-tests -DCMAKE_BUILD_TYPE=Debug \
  -DMK_BUILD_UNIT_TESTS=ON -DMK_BUILD_TEST_DRIVER=ON
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
python3 -m venv .venv
.venv/bin/python -m pip install -r tests/requirements.txt
.venv/bin/python -m pytest -q
```

pytest сам собирает потоковый C11-бинарник для float/double и отправляет ему запросы в stdin. [Протокол и ручной запуск драйвера](../tests/driver/README.md) описаны отдельно. Для обязательной embedded-проверки установите Arm GNU toolchain/newlib, QEMU и Ninja, затем запустите `.venv/bin/python -m pytest --embedded -q`. [Инструкция Cortex-M4](../tests/embedded/README.md) и [подготовка CI](../ci/README.md) описывают окружение.

