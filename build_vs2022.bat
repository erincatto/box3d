@echo off
rem Use this batch file to generate Visual Studio 2022 project files for box3d.
rem
rem Usage:
rem   build_vs2022.bat          Configure or update project files in build\
rem   build_vs2022.bat fresh    Delete build\ first, then configure from scratch
rem
rem Re-running without "fresh" updates the existing project files in place.
rem CMake rewrites .vcxproj/.sln files atomically, so this works even while
rem Visual Studio has the solution open -- VS will detect the change and
rem prompt to reload.

if /i "%~1"=="fresh" (
    if exist build (
        echo Fresh build requested, removing build\ ...
        rmdir /s /q build
        if exist build (
            echo ERROR: failed to remove build\ -- is something holding files open?
            exit /b 1
        )
    )
)

cmake -S . -B build -G "Visual Studio 17 2022"
