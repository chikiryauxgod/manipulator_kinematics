# Проверки CI

Workflow выполняет восемь host-комбинаций GCC/Clang × Debug/Release × float/double. Clang Debug включает ASan/UBSan; один из этих jobs дополнительно проверяет потоковый драйвер обоих профилей с санитайзерами. Отдельный обязательный job запускает pytest, включая пакетные проверки и Cortex-M4/QEMU для обоих профилей.

Образ Ubuntu и actions закреплены digest/commit, критические пакеты — точными версиями в test-toolchains.lock, Python-зависимости — в tests/requirements.txt. Host-окружение явно устанавливает libclang-rt-18-dev: без него --no-install-recommends оставляет Clang без runtime ASan/UBSan. install_dependencies.py используется только при подготовке отдельного CI-контейнера и требует root внутри него; CMake не запускает этот скрипт и ничего не скачивает.

При удалении закреплённого выпуска из apt-репозитория подготовка окружения должна завершиться ошибкой. Версии обновляются явным изменением lock и повторным прогоном всех проверок. ОС runtime-зависимости базового контейнера и непрямые apt-зависимости не являются полностью герметичным архивом пакетов; для долгосрочного воспроизведения следует сохранить успешный подготовленный образ в выбранном registry.

Локально достаточно установленных CMake, GCC/Clang, GoogleTest 1.14+, Python и pytest из requirements. Полный интеграционный набор требует Ninja для проверки multi-config генератора. Embedded-job дополнительно требует arm-none-eabi-gcc с newlib и qemu-system-arm. Если они распакованы без системной установки, укажите K_ARM_TOOLCHAIN_DIR и K_QEMU_PATH; runtime-зависимости QEMU должны разрешаться средой.

## Удалённый запуск

Remote настраивает владелец проекта. После настройки origin отправьте ветку:

```sh
git push -u origin feature/project_init
```

Это автоматически запускает workflow C11 kinematics. В GitHub откройте Actions и проверьте все восемь host jobs и integration-and-embedded; результаты должны относиться к отправленному commit. Ручной запуск через Run workflow доступен, когда файл workflow присутствует в default branch репозитория, согласно [инструкции GitHub](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/manually-run-a-workflow). Если push уже запустил проверки, повторный ручной запуск не нужен.

Успешный локальный Docker-прогон не считается удалённым запуском GitHub Actions. [Удалённый запуск](https://github.com/chikiryauxgod/manipulator_kinematics/actions/runs/38043984848) для commit 15c4d8f завершился успешно: все девять jobs прошли, артефакты сохранены.
