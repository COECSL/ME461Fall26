@echo off
setlocal

rem Change to the directory where this batch file is located
cd /d "%~dp0"

echo Base directory:
echo %CD%
echo.

for /d %%D in (*) do (

    echo ============================================================
    echo Checking folder: %%D
    echo ============================================================

    if exist "%%D\cpu01\ccs\" (

        pushd "%%D\cpu01\ccs"

        for %%F in (*.projectspec) do (
            if exist "%%F" (
                echo.
                echo Editing:
                echo %CD%\%%F
                echo.

                edit "%%F"

                echo.
                echo Finished editing %%F
            )
        )

        popd

    ) else (
        echo cpu01\ccs folder does not exist.
    )

    echo.
    echo Press any key to continue to the next folder...
    pause >nul
    echo.
)

echo ============================================================
echo Finished checking all folders.
echo ============================================================

pause