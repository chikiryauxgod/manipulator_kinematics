# Примеры

`forward_example.c` показывает настройку 3R-модели и расчёт положения инструмента. `inverse_example.c` задаёт ограничения суставов и находит углы для целевой точки TCP.

Из корня проекта:

```sh
cmake -S . -B build -DK_BUILD_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/manipulator_example
./build/manipulator_inverse_example
```

[Пример обмена через stdin/stdout](../tests/driver/README.md)
