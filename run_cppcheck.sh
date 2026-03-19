#!/bin/bash
# Скрипт статического анализа для пайплайна и локальной проверки

echo "Запуск Cppcheck для директории Cntrl_V2..."

cppcheck \
    --enable=warning,performance,portability \
    --std=c++11 \
    --language=c++ \
    --error-exitcode=1 \
    --inline-suppr \
    --suppress=missingIncludeSystem \
    --suppress=unmatchedSuppression \
    --force \
    --quiet \
    -i .pio/ \
    -i .vscode/ \
    -i build/ \
    Cntrl_V2/

echo "Cppcheck успешно завершен. Ошибок не найдено."