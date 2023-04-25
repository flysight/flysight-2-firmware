@echo off

for /f "tokens=* delims=" %%a in ('git describe --tags --always') do set GIT_TAG=%%a

(
echo #ifndef VERSION_H
echo #define VERSION_H
echo.
echo #define GIT_TAG "%GIT_TAG%"
echo.
echo #endif // VERSION_H
) > ..\FlySight\version.h
