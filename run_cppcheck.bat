@echo off
chcp 65001 > nul
echo Запуск Cppcheck для директории Cntrl_V2...

cppcheck ^
    --enable=warning,performance,portability ^
    --std=c++11 ^
    --language=c++ ^
    --error-exitcode=1 ^
    --inline-suppr ^
    --suppress=missingIncludeSystem ^
    --suppress=unmatchedSuppression ^
    --force ^
    --quiet ^
    -i .pio\ ^
    -i .vscode\ ^
    -i build\ ^
    Cntrl_V2\

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ОШИБКА] Cppcheck нашел проблемы в коде!
    exit /b %ERRORLEVEL%
)

echo.
echo [УСПЕХ] Cppcheck завершен. Ошибок не найдено.