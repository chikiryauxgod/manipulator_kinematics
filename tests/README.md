# Тесты

В `unit/` находятся Google Test-проверки модели, преобразований, FK и IK. В `integration/` — pytest-тесты потокового драйвера для float/double, включая ошибки входных данных и проверку результата IK через FK.

Из корня проекта; для unit-тестов нужен Google Test 1.14+:

```sh
cmake -S . -B build-tests -DK_BUILD_UNIT_TESTS=ON -DK_BUILD_TEST_DRIVER=ON
cmake --build build-tests --parallel
ctest --test-dir build-tests --output-on-failure
python3 -m venv .venv
.venv/bin/python -m pip install -r tests/requirements.txt
.venv/bin/python -m pytest -q
```

- [Потоковый драйвер](driver/README.md)
- [Проверка на Cortex-M4](embedded/README.md)
